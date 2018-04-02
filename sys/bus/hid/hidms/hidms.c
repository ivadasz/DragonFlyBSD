/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * HID spec: http://www.usb.org/developers/devclass_docs/HID1_11.pdf
 */

#include "opt_hid.h"
#include "opt_evdev.h"

#include <sys/stdint.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/priv.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/sbuf.h>
#include <sys/flexfifo.h>
#include <sys/udev.h>

#include <bus/hid/hid_common.h>
#include <bus/hid/hidvar.h>
#include "hid_if.h"

#ifdef EVDEV_SUPPORT
#include <dev/misc/evdev/input.h>
#include <dev/misc/evdev/evdev.h>
#include <bus/hid/hid_evdev_compat.h>
#endif

#include <sys/mouse.h>

#define	MOUSE_FLAGS_MASK (HIO_CONST|HIO_RELATIVE)
#define	MOUSE_FLAGS (HIO_RELATIVE)

#define	HIDMS_BUF_SIZE      8		/* bytes */
#define	HIDMS_IFQ_MAXLEN   50		/* units */
#define	HIDMS_BUTTON_MAX   31		/* exclusive, must be less than 32 */
#define	HIDMS_BUT(i) ((i) < 3 ? (((i) + 2) % 3) : (i))
#define	HIDMS_INFO_MAX	  2		/* maximum number of HID sets */

#define DPRINTFN(l, f, ...)	\
    do { if (hidms_debug >= (l)) { kprintf(f,## __VA_ARGS__); } } while(0)

#define DPRINTF(f, ...) DPRINTFN(1, f,## __VA_ARGS__)

struct hidms_status {
	uint32_t buttons;
	int32_t dx;
	int32_t dy;
	int32_t dz;
	int32_t dt;
};

struct hidms_info {
	struct hid_location sc_loc_w;
	struct hid_location sc_loc_x;
	struct hid_location sc_loc_y;
	struct hid_location sc_loc_z;
	struct hid_location sc_loc_t;
	struct hid_location sc_loc_btn[HIDMS_BUTTON_MAX];

	uint32_t sc_flags;
#define	HIDMS_FLAG_X_AXIS     0x0001
#define	HIDMS_FLAG_Y_AXIS     0x0002
#define	HIDMS_FLAG_Z_AXIS     0x0004
#define	HIDMS_FLAG_T_AXIS     0x0008
#define	HIDMS_FLAG_SBU        0x0010	/* spurious button up events */
#define	HIDMS_FLAG_REVZ	    0x0020	/* Z-axis is reversed */
#define	HIDMS_FLAG_W_AXIS     0x0040

	uint8_t	sc_iid_w;
	uint8_t	sc_iid_x;
	uint8_t	sc_iid_y;
	uint8_t	sc_iid_z;
	uint8_t	sc_iid_t;
	uint8_t	sc_iid_btn[HIDMS_BUTTON_MAX];
	uint8_t	sc_buttons;
};

struct hidms_softc {
	device_t dev;
	struct flexfifo *sc_fifo;
	struct lock sc_lock;
	struct callout sc_callout;
	struct hidms_info sc_info[HIDMS_INFO_MAX];

	mousehw_t sc_hw;
	mousemode_t sc_mode;
	mousestatus_t sc_status;

	int sc_opened;
#ifdef EVDEV_SUPPORT
	int sc_evflags;
#define	HIDMS_EVDEV_OPENED	1
#endif

	uint8_t	sc_buttons;
	uint8_t	sc_iid;
	uint8_t	sc_temp[64];

#ifdef EVDEV_SUPPORT
	struct evdev_dev *sc_evdev;
#endif
};

static int hidms_debug = 0;
TUNABLE_INT("hw.hid.hidms_debug", &hidms_debug);

static void hidms_put_queue_timeout(void *__sc);

static device_probe_t hidms_probe;
static device_attach_t hidms_attach;
static device_detach_t hidms_detach;

#ifdef EVDEV_SUPPORT
static evdev_open_t hidms_ev_open;
static evdev_close_t hidms_ev_close;
static void hidms_evdev_push(struct hidms_softc *, int32_t, int32_t,
    int32_t, int32_t, int32_t);
#endif

static void	hidms_put_queue(struct hidms_softc *, int32_t, int32_t,
		    int32_t, int32_t, int32_t);
#if 0 /* XXX */
static int	hidms_sysctl_handler_parseinfo(SYSCTL_HANDLER_ARGS);
#endif

static u_int hidms_pktlen(void *arg);
static u_int hidms_evtopkt(void *arg, uint8_t *ev, uint8_t *pkt);
static int hidms_ioctl(void *arg, caddr_t data, u_long cmd);
static void hidms_open(void *arg);
static void hidms_close(void *arg);

static struct flexfifo_ops hidms_fifo_ops = {
	.pktlen = hidms_pktlen,
	.evtopkt = hidms_evtopkt,
	.ioctl = hidms_ioctl,
	.open = hidms_open,
	.close = hidms_close,
};

#ifdef EVDEV_SUPPORT
static const struct evdev_methods hidms_evdev_methods = {
	.ev_open = &hidms_ev_open,
	.ev_close = &hidms_ev_close,
};
#endif

static void
hidms_put_queue_timeout(void *__sc)
{
	struct hidms_softc *sc = __sc;

	KKASSERT(lockowned(&sc->sc_lock));

	hidms_put_queue(sc, 0, 0, 0, 0, 0);
#ifdef EVDEV_SUPPORT
	hidms_evdev_push(sc, 0, 0, 0, 0, 0);
#endif
}

static void
hidms_input_handler(uint8_t id, uint8_t *buf, int len, void *arg)
{
	struct hidms_softc *sc = arg;
	struct hidms_info *info = &sc->sc_info[0];
	int32_t buttons = 0;
	int32_t buttons_found = 0;
#ifdef EVDEV_SUPPORT
	int32_t buttons_reported = 0;
#endif
	int32_t dw = 0;
	int32_t dx = 0;
	int32_t dy = 0;
	int32_t dz = 0;
	int32_t dt = 0;
	uint8_t i;

	DPRINTFN(6, "sc=%p actlen=%d\n", sc, len);

	if (len == 0)
		return;

	DPRINTFN(6, "data = %02x %02x %02x %02x "
	    "%02x %02x %02x %02x\n",
	    (len > 0) ? buf[0] : 0, (len > 1) ? buf[1] : 0,
	    (len > 2) ? buf[2] : 0, (len > 3) ? buf[3] : 0,
	    (len > 4) ? buf[4] : 0, (len > 5) ? buf[5] : 0,
	    (len > 6) ? buf[6] : 0, (len > 7) ? buf[7] : 0);

	if (sc->sc_info[0].sc_flags & HIDMS_FLAG_SBU) {
		if ((*buf == 0x14) || (*buf == 0x15))
			return;
	}

repeat:
	if ((info->sc_flags & HIDMS_FLAG_W_AXIS) &&
	    (id == info->sc_iid_w))
		dw += hid_get_data(buf, len, &info->sc_loc_w);

	if ((info->sc_flags & HIDMS_FLAG_X_AXIS) &&
	    (id == info->sc_iid_x))
		dx += hid_get_data(buf, len, &info->sc_loc_x);

	if ((info->sc_flags & HIDMS_FLAG_Y_AXIS) &&
	    (id == info->sc_iid_y))
		dy = -hid_get_data(buf, len, &info->sc_loc_y);

	if ((info->sc_flags & HIDMS_FLAG_Z_AXIS) &&
	    (id == info->sc_iid_z)) {
		int32_t temp;
		temp = hid_get_data(buf, len, &info->sc_loc_z);
		if (info->sc_flags & HIDMS_FLAG_REVZ)
			temp = -temp;
		dz -= temp;
	}

	if ((info->sc_flags & HIDMS_FLAG_T_AXIS) &&
	    (id == info->sc_iid_t))
		dt -= hid_get_data(buf, len, &info->sc_loc_t);

	for (i = 0; i < info->sc_buttons; i++) {
		uint32_t mask;
		mask = 1UL << HIDMS_BUT(i);
		/* check for correct button ID */
		if (id != info->sc_iid_btn[i])
			continue;
		/* check for button pressed */
		if (hid_get_data(buf, len, &info->sc_loc_btn[i]))
			buttons |= mask;
		/* register button mask */
		buttons_found |= mask;
	}

	if (++info != &sc->sc_info[HIDMS_INFO_MAX])
		goto repeat;

#ifdef EVDEV_SUPPORT
	buttons_reported = buttons;
#endif
	/* keep old button value(s) for non-detected buttons */
	buttons |= sc->sc_status.button & ~buttons_found;

	if (dx || dy || dz || dt || dw ||
	    (buttons != sc->sc_status.button)) {

		DPRINTFN(6, "x:%d y:%d z:%d t:%d w:%d buttons:0x%08x\n",
		    dx, dy, dz, dt, dw, buttons);

		/* translate T-axis into button presses until further */
		if (dt > 0)
			buttons |= 1UL << 3;
		else if (dt < 0)
			buttons |= 1UL << 4;

		sc->sc_status.button = buttons;
		sc->sc_status.dx += dx;
		sc->sc_status.dy += dy;
		sc->sc_status.dz += dz;
		/*
		 * sc->sc_status.dt += dt;
		 * no way to export this yet
		 */

		/*
		 * The Qtronix keyboard has a built in PS/2
		 * port for a mouse.  The firmware once in a
		 * while posts a spurious button up
		 * event. This event we ignore by doing a
		 * timeout for 50 msecs.  If we receive
		 * dx=dy=dz=buttons=0 before we add the event
		 * to the queue.  In any other case we delete
		 * the timeout event.
		 */
		if ((sc->sc_info[0].sc_flags & HIDMS_FLAG_SBU) &&
		    (dx == 0) && (dy == 0) && (dz == 0) && (dt == 0) &&
		    (dw == 0) && (buttons == 0)) {

			callout_reset(&sc->sc_callout, hz / 20,
			    &hidms_put_queue_timeout, sc);
		} else {

			callout_stop(&sc->sc_callout);

			hidms_put_queue(sc, dx, dy, dz, dt, buttons);
#ifdef EVDEV_SUPPORT
			hidms_evdev_push(sc, dx, dy, dz, dt,
			    buttons_reported);
#endif
		}
	}
}

static int
hidms_probe(device_t dev)
{
	const char *buf;
	uint16_t len;

	DPRINTFN(11, "\n");

	if (hid_get_bootproto(dev) == HID_BOOTPROTO_MOUSE) {
		device_set_desc(dev, "HID Mouse device");
		return (BUS_PROBE_DEFAULT);
	}

	HID_GET_DESCRIPTOR(device_get_parent(dev), &buf, &len);

	if (hid_is_mouse(buf, len))
		device_set_desc(dev, "HID Mouse device");
	else
		return (ENXIO);

#if 0
	/* XXX Do equivalent check in parent uhid driver. */
	if (usb_test_quirk(uaa, UQ_HIDMS_IGNORE))
		return (ENXIO);
#endif

	return (BUS_PROBE_DEFAULT);
}

static void
hidms_hid_parse(struct hidms_softc *sc, device_t dev, const uint8_t *buf,
    uint16_t len, uint8_t index)
{
	struct hidms_info *info = &sc->sc_info[index];
	uint32_t flags;
	uint8_t i;
	uint8_t j;

	if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_X),
	    hid_input, index, &info->sc_loc_x, &flags, &info->sc_iid_x)) {

		if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
			info->sc_flags |= HIDMS_FLAG_X_AXIS;
		}
	}
	if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_Y),
	    hid_input, index, &info->sc_loc_y, &flags, &info->sc_iid_y)) {

		if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
			info->sc_flags |= HIDMS_FLAG_Y_AXIS;
		}
	}
	/* Try the wheel first as the Z activator since it's tradition. */
	if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP,
	    HUG_WHEEL), hid_input, index, &info->sc_loc_z, &flags,
	    &info->sc_iid_z) ||
	    hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP,
	    HUG_TWHEEL), hid_input, index, &info->sc_loc_z, &flags,
	    &info->sc_iid_z)) {
		if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
			info->sc_flags |= HIDMS_FLAG_Z_AXIS;
		}
		/*
		 * We might have both a wheel and Z direction, if so put
		 * put the Z on the W coordinate.
		 */
		if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP,
		    HUG_Z), hid_input, index, &info->sc_loc_w, &flags,
		    &info->sc_iid_w)) {

			if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
				info->sc_flags |= HIDMS_FLAG_W_AXIS;
			}
		}
	} else if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP,
	    HUG_Z), hid_input, index, &info->sc_loc_z, &flags,
	    &info->sc_iid_z)) {

		if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
			info->sc_flags |= HIDMS_FLAG_Z_AXIS;
		}
	}
	/*
	 * The Microsoft Wireless Intellimouse 2.0 reports it's wheel
	 * using 0x0048, which is HUG_TWHEEL, and seems to expect you
	 * to know that the byte after the wheel is the tilt axis.
	 * There are no other HID axis descriptors other than X,Y and
	 * TWHEEL
	 */
	if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP,
	    HUG_TWHEEL), hid_input, index, &info->sc_loc_t,
	    &flags, &info->sc_iid_t)) {

		info->sc_loc_t.pos += 8;

		if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
			info->sc_flags |= HIDMS_FLAG_T_AXIS;
		}
	} else if (hid_locate(buf, len, HID_USAGE2(HUP_CONSUMER,
		HUC_AC_PAN), hid_input, index, &info->sc_loc_t,
		&flags, &info->sc_iid_t)) {

		if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS)
			info->sc_flags |= HIDMS_FLAG_T_AXIS;
	}
	/* figure out the number of buttons */

	for (i = 0; i < HIDMS_BUTTON_MAX; i++) {
		if (!hid_locate(buf, len, HID_USAGE2(HUP_BUTTON, (i + 1)),
		    hid_input, index, &info->sc_loc_btn[i], NULL,
		    &info->sc_iid_btn[i])) {
			break;
		}
	}

	/* detect other buttons */

	for (j = 0; (i < HIDMS_BUTTON_MAX) && (j < 2); i++, j++) {
		if (!hid_locate(buf, len, HID_USAGE2(HUP_MICROSOFT, (j + 1)),
		    hid_input, index, &info->sc_loc_btn[i], NULL,
		    &info->sc_iid_btn[i])) {
			break;
		}
	}

	info->sc_buttons = i;

	if (i > sc->sc_buttons)
		sc->sc_buttons = i;

	if (info->sc_flags == 0)
		return;

	/* announce information about the mouse */
	device_printf(dev, "%d buttons and [%s%s%s%s%s] coordinates ID=%u\n",
	    (info->sc_buttons),
	    (info->sc_flags & HIDMS_FLAG_X_AXIS) ? "X" : "",
	    (info->sc_flags & HIDMS_FLAG_Y_AXIS) ? "Y" : "",
	    (info->sc_flags & HIDMS_FLAG_Z_AXIS) ? "Z" : "",
	    (info->sc_flags & HIDMS_FLAG_T_AXIS) ? "T" : "",
	    (info->sc_flags & HIDMS_FLAG_W_AXIS) ? "W" : "",
	    info->sc_iid_x);
}

static int
hidms_attach(device_t dev)
{
	struct hidms_softc *sc = device_get_softc(dev);
	const char *d_ptr = NULL;
#ifdef EVDEV_SUPPORT
	struct hidms_info *info;
	int err;
#endif
	uint16_t d_len;
	uint8_t i;
#ifdef HID_DEBUG
	uint8_t j;
#endif

	DPRINTFN(11, "sc=%p\n", sc);

	sc->dev = dev;
	lockinit(&sc->sc_lock, "hidms lock", 0, LK_CANRECURSE);
	callout_init_lk(&sc->sc_callout, &sc->sc_lock);

	HID_GET_DESCRIPTOR(device_get_parent(dev), &d_ptr, &d_len);

	/*
	 * The Microsoft Wireless Notebook Optical Mouse seems to be in worse
	 * shape than the Wireless Intellimouse 2.0, as its X, Y, wheel, and
	 * all of its other button positions are all off. It also reports that
	 * it has two addional buttons and a tilt wheel.
	 */
#if 0
	if (usb_test_quirk(uaa, UQ_MS_BAD_CLASS)) {

		sc->sc_iid = 0;

		info = &sc->sc_info[0];
		info->sc_flags = (HIDMS_FLAG_X_AXIS |
		    HIDMS_FLAG_Y_AXIS |
		    HIDMS_FLAG_Z_AXIS |
		    HIDMS_FLAG_SBU);
		info->sc_buttons = 3;
		isize = 5;
		/* 1st byte of descriptor report contains garbage */
		info->sc_loc_x.pos = 16;
		info->sc_loc_x.size = 8;
		info->sc_loc_y.pos = 24;
		info->sc_loc_y.size = 8;
		info->sc_loc_z.pos = 32;
		info->sc_loc_z.size = 8;
		info->sc_loc_btn[0].pos = 8;
		info->sc_loc_btn[0].size = 1;
		info->sc_loc_btn[1].pos = 9;
		info->sc_loc_btn[1].size = 1;
		info->sc_loc_btn[2].pos = 10;
		info->sc_loc_btn[2].size = 1;

		/* Announce device */
		device_printf(dev, "3 buttons and [XYZ] "
		    "coordinates ID=0\n");

	} else
#endif
	{
		/* Search the HID descriptor and announce device */
		for (i = 0; i < HIDMS_INFO_MAX; i++) {
			hidms_hid_parse(sc, dev, d_ptr, d_len, i);
		}
	}

#if 0
	if (usb_test_quirk(uaa, UQ_MS_REVZ)) {
		info = &sc->sc_info[0];
		/* Some wheels need the Z axis reversed. */
		info->sc_flags |= HIDMS_FLAG_REVZ;
	}
#endif
	d_ptr = NULL;

#ifdef HID_DEBUG
	for (j = 0; j < HIDMS_INFO_MAX; j++) {
		info = &sc->sc_info[j];

		DPRINTF("sc=%p, index=%d\n", sc, j);
		DPRINTF("X\t%d/%d id=%d\n", info->sc_loc_x.pos,
		    info->sc_loc_x.size, info->sc_iid_x);
		DPRINTF("Y\t%d/%d id=%d\n", info->sc_loc_y.pos,
		    info->sc_loc_y.size, info->sc_iid_y);
		DPRINTF("Z\t%d/%d id=%d\n", info->sc_loc_z.pos,
		    info->sc_loc_z.size, info->sc_iid_z);
		DPRINTF("T\t%d/%d id=%d\n", info->sc_loc_t.pos,
		    info->sc_loc_t.size, info->sc_iid_t);
		DPRINTF("W\t%d/%d id=%d\n", info->sc_loc_w.pos,
		    info->sc_loc_w.size, info->sc_iid_w);

		for (i = 0; i < info->sc_buttons; i++) {
			DPRINTF("B%d\t%d/%d id=%d\n",
			    i + 1, info->sc_loc_btn[i].pos,
			    info->sc_loc_btn[i].size, info->sc_iid_btn[i]);
		}
	}
#if 0	/* This information is already printed by the HID bus parent. */
	DPRINTF("size=%d, id=%d\n", isize, sc->sc_iid);
#endif
#endif

	char buf[16];
	ksnprintf(buf, sizeof(buf), "hidms%d", device_get_unit(dev));
	sc->sc_fifo = flexfifo_create(sizeof(struct hidms_status), 256,
	    &hidms_fifo_ops, device_get_unit(dev), buf, sc, 8);
	udev_dict_set_cstr(flexfifo_get_cdev(sc->sc_fifo),
	    "subsystem", "mouse");

#ifdef EVDEV_SUPPORT
	sc->sc_evdev = evdev_alloc();
	evdev_set_name(sc->sc_evdev, device_get_desc(dev));
	evdev_set_phys(sc->sc_evdev, device_get_nameunit(dev));
	evdev_set_id(sc->sc_evdev, hid_bustype_to_evdev(hid_get_bustype(dev)),
	    hid_get_vendor(dev), hid_get_product(dev), 0);
	evdev_set_serial(sc->sc_evdev, hid_get_serial(dev));
	evdev_set_methods(sc->sc_evdev, sc, &hidms_evdev_methods);
	evdev_support_prop(sc->sc_evdev, INPUT_PROP_POINTER);
	evdev_support_event(sc->sc_evdev, EV_SYN);
	evdev_support_event(sc->sc_evdev, EV_REL);
	evdev_support_event(sc->sc_evdev, EV_KEY);

	info = &sc->sc_info[0];

	if (info->sc_flags & HIDMS_FLAG_X_AXIS)
		evdev_support_rel(sc->sc_evdev, REL_X);

	if (info->sc_flags & HIDMS_FLAG_Y_AXIS)
		evdev_support_rel(sc->sc_evdev, REL_Y);

	if (info->sc_flags & HIDMS_FLAG_Z_AXIS)
		evdev_support_rel(sc->sc_evdev, REL_WHEEL);

	if (info->sc_flags & HIDMS_FLAG_T_AXIS)
		evdev_support_rel(sc->sc_evdev, REL_HWHEEL);

	for (i = 0; i < info->sc_buttons; i++)
		evdev_support_key(sc->sc_evdev, BTN_MOUSE + i);

	err = evdev_register_mtx(sc->sc_evdev, &sc->sc_lock);
	if (err)
		goto detach;
#endif

#if 0
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "parseinfo", CTLTYPE_STRING|CTLFLAG_RD,
	    sc, 0, hidms_sysctl_handler_parseinfo,
	    "", "Dump of parsed HID report descriptor");
#endif

	HID_SET_HANDLER(device_get_parent(dev), hidms_input_handler, NULL, sc);

	return (0);

#ifdef EVDEV_SUPPORT
detach:
	hidms_detach(dev);
	return (ENOMEM);
#endif
}

static int
hidms_detach(device_t self)
{
	struct hidms_softc *sc = device_get_softc(self);

	DPRINTF("sc=%p\n", sc);

	HID_STOP_READ(device_get_parent(self));
	HID_SET_HANDLER(device_get_parent(self), NULL, NULL, NULL);
	callout_drain(&sc->sc_callout);

	if (sc->sc_fifo != NULL)
		flexfifo_destroy(sc->sc_fifo);

#ifdef EVDEV_SUPPORT
	evdev_free(sc->sc_evdev);
#endif

	lockuninit(&sc->sc_lock);

	return (0);
}

static void
hidms_reset(struct hidms_softc *sc)
{

	/* reset all USB mouse parameters */

	if (sc->sc_buttons > MOUSE_MSC_MAXBUTTON)
		sc->sc_hw.buttons = MOUSE_MSC_MAXBUTTON;
	else
		sc->sc_hw.buttons = sc->sc_buttons;

	sc->sc_hw.iftype = MOUSE_IF_USB;
	sc->sc_hw.type = MOUSE_MOUSE;
	sc->sc_hw.model = MOUSE_MODEL_GENERIC;
	sc->sc_hw.hwid = 0;

	sc->sc_mode.protocol = MOUSE_PROTO_MSC;
	sc->sc_mode.rate = -1;
	sc->sc_mode.resolution = MOUSE_RES_UNKNOWN;
	sc->sc_mode.accelfactor = 0;
	sc->sc_mode.level = 0;
	sc->sc_mode.packetsize = MOUSE_MSC_PACKETSIZE;
	sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
	sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;

	/* reset status */

	sc->sc_status.flags = 0;
	sc->sc_status.button = 0;
	sc->sc_status.obutton = 0;
	sc->sc_status.dx = 0;
	sc->sc_status.dy = 0;
	sc->sc_status.dz = 0;
	/* sc->sc_status.dt = 0; */
}

#if ((MOUSE_SYS_PACKETSIZE != 8) || \
     (MOUSE_MSC_PACKETSIZE != 5))
#error "Software assumptions are not met. Please update code."
#endif

static void
hidms_put_queue(struct hidms_softc *sc, int32_t dx, int32_t dy,
    int32_t dz, int32_t dt, int32_t buttons)
{
	struct hidms_status status;

	status = (struct hidms_status){ buttons, dx, dy, dz, dt };
	if (sc->sc_fifo != NULL)
		flexfifo_enqueue_ring(sc->sc_fifo, (void *)&status);
}

static u_int
hidms_evtopkt(void *arg, uint8_t *ev, uint8_t *pkt)
{
	struct hidms_softc *sc = arg;
	struct hidms_status *status = (struct hidms_status *)ev;
	uint32_t buttons = status->buttons;
	int32_t dx = status->dx;
	int32_t dy = status->dy;
	int32_t dz = status->dz;
	int32_t dt = status->dt;

	if (dx > 254)
		dx = 254;
	if (dx < -256)
		dx = -256;
	if (dy > 254)
		dy = 254;
	if (dy < -256)
		dy = -256;
	if (dz > 126)
		dz = 126;
	if (dz < -128)
		dz = -128;
	if (dt > 126)
		dt = 126;
	if (dt < -128)
		dt = -128;

	pkt[0] = sc->sc_mode.syncmask[1];
	pkt[0] |= (~buttons) & MOUSE_MSC_BUTTONS;
	pkt[1] = dx >> 1;
	pkt[2] = dy >> 1;
	pkt[3] = dx - (dx >> 1);
	pkt[4] = dy - (dy >> 1);

	if (sc->sc_mode.level == 1) {
		pkt[5] = dz >> 1;
		pkt[6] = dz - (dz >> 1);
		pkt[7] = (((~buttons) >> 3) & MOUSE_SYS_EXTBUTTONS);
		return 8;
	}
	return 5;
}

static u_int
hidms_pktlen(void *arg)
{
	struct hidms_softc *sc = arg;

	if (sc->sc_mode.level == 1)
		return 8;
	else
		return 5;
}

#ifdef EVDEV_SUPPORT
static void
hidms_evdev_push(struct hidms_softc *sc, int32_t dx, int32_t dy,
    int32_t dz, int32_t dt, int32_t buttons)
{
	if (evdev_rcpt_mask & EVDEV_RCPT_HW_MOUSE) {
		/* Push evdev event */
		evdev_push_rel(sc->sc_evdev, REL_X, dx);
		evdev_push_rel(sc->sc_evdev, REL_Y, -dy);
		evdev_push_rel(sc->sc_evdev, REL_WHEEL, -dz);
		evdev_push_rel(sc->sc_evdev, REL_HWHEEL, dt);
		evdev_push_mouse_btn(sc->sc_evdev,
		    (buttons & ~MOUSE_STDBUTTONS) |
		    (buttons & (1 << 2) ? MOUSE_BUTTON1DOWN : 0) |
		    (buttons & (1 << 1) ? MOUSE_BUTTON2DOWN : 0) |
		    (buttons & (1 << 0) ? MOUSE_BUTTON3DOWN : 0));
		evdev_sync(sc->sc_evdev);
	}
}
#endif

#ifdef EVDEV_SUPPORT
static int
hidms_ev_open(struct evdev_dev *evdev, void *ev_softc)
{
	struct hidms_softc *sc = (struct hidms_softc *)ev_softc;

	KKASSERT(lockowned(&sc->sc_lock));

	sc->sc_evflags = HIDMS_EVDEV_OPENED;

	if (sc->sc_opened == 0)
		HID_START_READ(device_get_parent(sc->dev), 0);

	return (0);
}

static void
hidms_ev_close(struct evdev_dev *evdev, void *ev_softc)
{
	struct hidms_softc *sc = (struct hidms_softc *)ev_softc;

	KKASSERT(lockowned(&sc->sc_lock));

	sc->sc_evflags = 0;

	if (sc->sc_opened == 0) {
		HID_STOP_READ(device_get_parent(sc->dev));
		callout_stop(&sc->sc_callout);
	}
}
#endif

static void
hidms_open(void *arg)
{
	struct hidms_softc *sc = arg;

	hidms_reset(sc);
	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	sc->sc_opened = 1;
#ifdef EVDEV_SUPPORT
	if (sc->sc_evflags == 0)
#endif
		HID_START_READ(device_get_parent(sc->dev), 0);
	lockmgr(&sc->sc_lock, LK_RELEASE);
}

static void
hidms_close(void *arg)
{
	struct hidms_softc *sc = arg;

	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	sc->sc_opened = 0;
#ifdef EVDEV_SUPPORT
	if (sc->sc_evflags == 0)
#endif
	{
		HID_STOP_READ(device_get_parent(sc->dev));
		callout_stop(&sc->sc_callout);
	}
	lockmgr(&sc->sc_lock, LK_RELEASE);
}

static int
hidms_ioctl(void *arg, caddr_t data, u_long cmd)
{
	struct hidms_softc *sc = arg;
	mousemode_t mode;
	int error = 0;

	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);

	switch (cmd) {
	case MOUSE_GETHWINFO:
		*(mousehw_t *)data = sc->sc_hw;
		break;

	case MOUSE_GETMODE:
		*(mousemode_t *)data = sc->sc_mode;
		break;

	case MOUSE_SETMODE:
		mode = *(mousemode_t *)data;

		if (mode.level == -1) {
			/* don't change the current setting */
		} else if ((mode.level < 0) || (mode.level > 1)) {
			error = EINVAL;
			break;
		} else {
			sc->sc_mode.level = mode.level;
		}

		if (sc->sc_mode.level == 0) {
			if (sc->sc_buttons > MOUSE_MSC_MAXBUTTON)
				sc->sc_hw.buttons = MOUSE_MSC_MAXBUTTON;
			else
				sc->sc_hw.buttons = sc->sc_buttons;
			sc->sc_mode.protocol = MOUSE_PROTO_MSC;
			sc->sc_mode.packetsize = MOUSE_MSC_PACKETSIZE;
			sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
			sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->sc_mode.level == 1) {
			if (sc->sc_buttons > MOUSE_SYS_MAXBUTTON)
				sc->sc_hw.buttons = MOUSE_SYS_MAXBUTTON;
			else
				sc->sc_hw.buttons = sc->sc_buttons;
			sc->sc_mode.protocol = MOUSE_PROTO_SYSMOUSE;
			sc->sc_mode.packetsize = MOUSE_SYS_PACKETSIZE;
			sc->sc_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
			sc->sc_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}

		/* Check if we should override the default polling interval */
		if (mode.rate > 0) {
			if (mode.rate > 1000)
				mode.rate = 1000;
			hid_set_interval_ms(sc->dev, 1000 / mode.rate);
			HID_START_READ(device_get_parent(sc->dev), 0);
		}
		break;

	case MOUSE_GETLEVEL:
		*(int *)data = sc->sc_mode.level;
		break;

	case MOUSE_SETLEVEL:
		if (*(int *)data < 0 || *(int *)data > 1) {
			error = EINVAL;
			break;
		}
		sc->sc_mode.level = *(int *)data;

		if (sc->sc_mode.level == 0) {
			if (sc->sc_buttons > MOUSE_MSC_MAXBUTTON)
				sc->sc_hw.buttons = MOUSE_MSC_MAXBUTTON;
			else
				sc->sc_hw.buttons = sc->sc_buttons;
			sc->sc_mode.protocol = MOUSE_PROTO_MSC;
			sc->sc_mode.packetsize = MOUSE_MSC_PACKETSIZE;
			sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
			sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->sc_mode.level == 1) {
			if (sc->sc_buttons > MOUSE_SYS_MAXBUTTON)
				sc->sc_hw.buttons = MOUSE_SYS_MAXBUTTON;
			else
				sc->sc_hw.buttons = sc->sc_buttons;
			sc->sc_mode.protocol = MOUSE_PROTO_SYSMOUSE;
			sc->sc_mode.packetsize = MOUSE_SYS_PACKETSIZE;
			sc->sc_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
			sc->sc_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		break;

	case MOUSE_GETSTATUS:{
			mousestatus_t *status = (mousestatus_t *)data;

			*status = sc->sc_status;
			sc->sc_status.obutton = sc->sc_status.button;
			sc->sc_status.button = 0;
			sc->sc_status.dx = 0;
			sc->sc_status.dy = 0;
			sc->sc_status.dz = 0;
			/* sc->sc_status.dt = 0; */

			if (status->dx || status->dy || status->dz /* || status->dt */ ) {
				status->flags |= MOUSE_POSCHANGED;
			}
			if (status->button != status->obutton) {
				status->flags |= MOUSE_BUTTONSCHANGED;
			}
			break;
		}
	default:
		error = ENOTTY;
		break;
	}

	lockmgr(&sc->sc_lock, LK_RELEASE);
	return (error);
}

#if 0 /* XXX */
static int
hidms_sysctl_handler_parseinfo(SYSCTL_HANDLER_ARGS)
{
	struct hidms_softc *sc = arg1;
	struct hidms_info *info;
	struct sbuf *sb;
	int i, j, err, had_output;

	sb = sbuf_new_auto();
	for (i = 0, had_output = 0; i < HIDMS_INFO_MAX; i++) {
		info = &sc->sc_info[i];

		/* Don't emit empty info */
		if ((info->sc_flags &
		    (HIDMS_FLAG_X_AXIS | HIDMS_FLAG_Y_AXIS | HIDMS_FLAG_Z_AXIS |
		     HIDMS_FLAG_T_AXIS | HIDMS_FLAG_W_AXIS)) == 0 &&
		    info->sc_buttons == 0)
			continue;

		if (had_output)
			sbuf_printf(sb, "\n");
		had_output = 1;
		sbuf_printf(sb, "i%d:", i + 1);
		if (info->sc_flags & HIDMS_FLAG_X_AXIS)
			sbuf_printf(sb, " X:r%d, p%d, s%d;",
			    (int)info->sc_iid_x,
			    (int)info->sc_loc_x.pos,
			    (int)info->sc_loc_x.size);
		if (info->sc_flags & HIDMS_FLAG_Y_AXIS)
			sbuf_printf(sb, " Y:r%d, p%d, s%d;",
			    (int)info->sc_iid_y,
			    (int)info->sc_loc_y.pos,
			    (int)info->sc_loc_y.size);
		if (info->sc_flags & HIDMS_FLAG_Z_AXIS)
			sbuf_printf(sb, " Z:r%d, p%d, s%d;",
			    (int)info->sc_iid_z,
			    (int)info->sc_loc_z.pos,
			    (int)info->sc_loc_z.size);
		if (info->sc_flags & HIDMS_FLAG_T_AXIS)
			sbuf_printf(sb, " T:r%d, p%d, s%d;",
			    (int)info->sc_iid_t,
			    (int)info->sc_loc_t.pos,
			    (int)info->sc_loc_t.size);
		if (info->sc_flags & HIDMS_FLAG_W_AXIS)
			sbuf_printf(sb, " W:r%d, p%d, s%d;",
			    (int)info->sc_iid_w,
			    (int)info->sc_loc_w.pos,
			    (int)info->sc_loc_w.size);

		for (j = 0; j < info->sc_buttons; j++) {
			sbuf_printf(sb, " B%d:r%d, p%d, s%d;", j + 1,
			    (int)info->sc_iid_btn[j],
			    (int)info->sc_loc_btn[j].pos,
			    (int)info->sc_loc_btn[j].size);
		}
	}
	sbuf_finish(sb);
	err = SYSCTL_OUT(req, sbuf_data(sb), sbuf_len(sb) + 1);
	sbuf_delete(sb);

	return (err);
}
#endif

static devclass_t hidms_devclass;

static device_method_t hidms_methods[] = {
	DEVMETHOD(device_probe, hidms_probe),
	DEVMETHOD(device_attach, hidms_attach),
	DEVMETHOD(device_detach, hidms_detach),
	DEVMETHOD_END
};

static driver_t hidms_driver = {
	.name = "hidms",
	.methods = hidms_methods,
	.size = sizeof(struct hidms_softc),
};

DRIVER_MODULE(hidms, usbhid, hidms_driver, hidms_devclass, NULL, NULL);
#ifdef EVDEV_SUPPORT
MODULE_DEPEND(hidms, evdev, 1, 1, 1);
#endif
MODULE_VERSION(hidms, 1);

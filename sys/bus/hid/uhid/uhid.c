/*	$NetBSD: uhid.c,v 1.46 2001/11/13 06:24:55 lukem Exp $	*/

/* Also already merged from NetBSD:
 *	$NetBSD: uhid.c,v 1.54 2002/09/23 05:51:21 simonb Exp $
 */

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

#include <sys/stdint.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>
#include <sys/fcntl.h>
#include <sys/queue.h>
#include <sys/flexfifo.h>

#include <bus/u4b/input/usb_rdesc.h>

#define UHID_INPUT_REPORT 0x01
#define UHID_OUTPUT_REPORT 0x02
#define UHID_FEATURE_REPORT 0x03

#include <bus/hid/hid_common.h>
#include <bus/hid/hid_ioctl.h>
#include <bus/hid/hidvar.h>
#include "hid_if.h"

#include "usbdevs.h"

static int uhid_debug = 0;

static SYSCTL_NODE(_hw_hid, OID_AUTO, uhid, CTLFLAG_RW, 0, "USB uhid");
SYSCTL_INT(_hw_hid_uhid, OID_AUTO, debug, CTLFLAG_RW,
    &uhid_debug, 0, "Debug level");

#define DPRINTFN(l, f, ...)     \
    do { if (uhid_debug >= (l)) { kprintf(f,## __VA_ARGS__); } } while(0)

#define DPRINTF(f, ...) DPRINTFN(1, f,## __VA_ARGS__)

static int uhid_probe_kbd_ms = 0;
TUNABLE_INT("hw.hid.uhid.probe_kbdms", &uhid_probe_kbd_ms);

struct hid_stream_report {
	uint16_t len;
	uint8_t id;
	unsigned char report[];
};

struct hid_report_list {
	uint16_t len;
	uint8_t id;
	unsigned char *report;
	int done;
	void *arg;
	void(*cb)(void *arg, struct hid_report_list *e);
	STAILQ_ENTRY(hid_report_list) entries;
};

struct uhid_softc {
	device_t sc_dev;
	struct lock sc_lock;
	struct flexfifo *sc_fifo;
	int sc_detaching;
	int sc_writing;
	STAILQ_HEAD(, hid_report_list) sc_pending_evs;
	struct hid_report_list sc_rep;

	uint32_t sc_isize;
	uint32_t sc_osize;
	uint32_t sc_fsize;

	const char *sc_repdesc_ptr;
	uint16_t sc_repdesc_size;

	struct hid_stream_report *tmpbuf;

	uint8_t	sc_iid;
	uint8_t	sc_oid;
	uint8_t	sc_fid;
	uint8_t	sc_flags;
#define	UHID_FLAG_STATIC_DESC  0x04	/* set if report descriptors are
					 * static */
};

static const uint8_t uhid_graphire_report_descr[] = {USBHID_GRAPHIRE_REPORT_DESCR()};
static const uint8_t uhid_graphire3_4x5_report_descr[] = {USBHID_GRAPHIRE3_4X5_REPORT_DESCR()};

/* prototypes */

static device_probe_t uhid_probe;
static device_attach_t uhid_attach;
static device_detach_t uhid_detach;

static u_int uhid_pktlen(void *arg);
static u_int uhid_evtopkt(void *arg, uint8_t *ev, uint8_t *pkt);
static void uhid_sendev(void *arg, u_int8_t *ev);
static int uhid_pkttoev(void *arg, u_int8_t *ev, u_int8_t *pkt, u_int len);
static int uhid_ioctl(void *arg, caddr_t data, u_long cmd, int fflags);
static void uhid_open(void *arg);
static void uhid_close(void *arg);

static struct flexfifo_ops uhid_fifo_ops = {
	.pktlen = uhid_pktlen,
	.evtopkt = uhid_evtopkt,
	.sendev = uhid_sendev,
	.pkttoev = uhid_pkttoev,
	.ioctl = uhid_ioctl,
	.open = uhid_open,
	.close = uhid_close,
};

static void
uhid_input_handler(uint8_t id, uint8_t *buf, int len, void *arg)
{
	struct uhid_softc *sc = arg;

	sc->tmpbuf->len = len;
	sc->tmpbuf->id = id;
	memcpy(sc->tmpbuf->report, buf, len);
	flexfifo_enqueue_ring(sc->sc_fifo, (void *)sc->tmpbuf);
}

static void
uhid_output_handler(uint8_t *buf, int len, void *arg)
{
	struct uhid_softc *sc = arg;

	if (buf != NULL) {
		struct hid_report_list *rep;

		lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
		rep = STAILQ_FIRST(&sc->sc_pending_evs);
		if (rep->report == buf) {
			STAILQ_REMOVE_HEAD(&sc->sc_pending_evs, entries);
			rep->done = 1;
			rep->cb(rep->arg, rep);
		}
		if (sc->sc_detaching) {
			lockmgr(&sc->sc_lock, LK_RELEASE);
			return;
		}
		rep = STAILQ_FIRST(&sc->sc_pending_evs);
		if (rep == NULL) {
			sc->sc_writing = 0;
		} else {
			HID_SET_REPORT(device_get_parent(sc->sc_dev),
			    rep->id, rep->report, rep->len);
		}
		lockmgr(&sc->sc_lock, LK_RELEASE);
	}
}

static u_int
uhid_pktlen(void *arg)
{
	struct uhid_softc *sc = arg;
	struct hid_stream_report *buf;
	u_int len;

	buf = (void *)flexfifo_peek_ring(sc->sc_fifo);
	if (buf == NULL)
		return 0;
	len = buf->len;
	if (sc->sc_iid != 0)
		len++;
	return len;
}

static u_int
uhid_evtopkt(void *arg, uint8_t *ev, uint8_t *pkt)
{
	struct uhid_softc *sc = arg;
	struct hid_stream_report *buf = (void *)ev;
	u_int len = buf->len;

	if (sc->sc_iid != 0) {
		pkt[0] = buf->id;
		len++;
		memcpy(&pkt[1], buf->report, buf->len);
	} else {
		memcpy(pkt, buf->report, buf->len);
	}
	return len;
}

static void
uhid_output_write_done(void *arg, struct hid_report_list *e)
{
	struct uhid_softc *sc = arg;

	flexfifo_write_done(sc->sc_fifo);
}

static void
uhid_start_writing(struct uhid_softc *sc)
{
	struct hid_report_list *rep;

	if (sc->sc_writing == 0) {
		sc->sc_writing = 1;
		rep = STAILQ_FIRST(&sc->sc_pending_evs);
		if (rep != NULL) {
			HID_SET_REPORT(device_get_parent(sc->sc_dev),
			    rep->id, rep->report, rep->len);
		}
	}
}

static void
uhid_sendev(void *arg, u_int8_t *ev)
{
	struct uhid_softc *sc = arg;
	struct hid_report_list *rep = &sc->sc_rep;
	struct hid_stream_report *buf = (void *)ev;

	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	if (sc->sc_detaching) {
		lockmgr(&sc->sc_lock, LK_RELEASE);
		return;
	}

	rep->len = buf->len;
	rep->id = buf->id;
	rep->report = buf->report;
	rep->done = 0;
	rep->arg = sc;
	rep->cb = uhid_output_write_done;
	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	STAILQ_INSERT_TAIL(&sc->sc_pending_evs, rep, entries);
	uhid_start_writing(sc);
	lockmgr(&sc->sc_lock, LK_RELEASE);
}

static int
uhid_pkttoev(void *arg, u_int8_t *ev, u_int8_t *pkt, u_int len)
{
	struct uhid_softc *sc = arg;
	struct hid_stream_report *buf = (void *)ev;

	/* XXX Check that len matches the length for the Report ID */
	if (sc->sc_iid != 0) {
		KKASSERT(len > 0);
		buf->len = len - 1;
		buf->id = pkt[0];
		memcpy(buf->report, &pkt[1], len - 1);
	} else {
		buf->len = len;
		buf->id = 0;
		memcpy(buf->report, pkt, len);
	}
	return 0;
}

static void
uhid_output_ioctl_done(void *arg, struct hid_report_list *e)
{
	kfree(e->report, M_DEVBUF);
	wakeup(e);
}

static int
uhid_ioctl(void *arg, caddr_t data, u_long cmd, int fflags)
{
	struct uhid_softc *sc = arg;
	struct hid_report_desc *repdesc;
	struct hid_report_request *request;
	char *repbuf;
	uint16_t bufsize;
	int error = 0;

	switch (cmd) {
	case UHID_GET_REPORT_DESC:
		repdesc = (void *)data;

		bufsize = repdesc->len;

		if (repdesc->report_desc == NULL) {
			repdesc->len = sc->sc_repdesc_size;
			break;		/* descriptor length only */
		}
		repdesc->len = imin(bufsize, sc->sc_repdesc_size);
		error = copyout(sc->sc_repdesc_ptr, repdesc->report_desc,
		    repdesc->len);
		break;

	case UHID_GET_REPORT:
		if (!(fflags & FREAD)) {
			error = EPERM;
			break;
		}
		request = (void *)data;
		if (request->len == 0)
			return (EINVAL);
		switch (request->kind) {
		case UHID_INPUT_REPORT:
			/*
			 * XXX Instead check whether the length matches our
			 *     expectation for this Report ID.
			 */
			if (request->len > sc->sc_isize)
				return (EINVAL);
			repbuf = kmalloc(request->len, M_DEVBUF,
			    M_WAITOK | M_ZERO);
			error = HID_GET_REPORT(device_get_parent(sc->sc_dev),
			    request->id, repbuf, request->len, hid_input);
			break;
		case UHID_FEATURE_REPORT:
			if (request->len > sc->sc_fsize)
				return (EINVAL);
			repbuf = kmalloc(request->len, M_DEVBUF,
			    M_WAITOK | M_ZERO);
			error = HID_GET_REPORT(device_get_parent(sc->sc_dev),
			    request->id, repbuf, request->len, hid_feature);
			break;
		case UHID_OUTPUT_REPORT:
		default:
			return (EINVAL);
		}
		if (repbuf != NULL) {
			error = copyout(repbuf, request->report, request->len);
			kfree(repbuf, M_DEVBUF);
		}
		if (error != 0)
			return (EIO);
		break;

	case UHID_SET_REPORT:
		if (!(fflags & FWRITE)) {
			error = EPERM;
			break;
		}
		request = (void *)data;
		if (request->len == 0)
			return (EINVAL);
		switch (request->kind) {
		case UHID_OUTPUT_REPORT:
			if (request->len > sc->sc_osize)
				return (EINVAL);
			break;
		case UHID_FEATURE_REPORT:
			if (request->len > sc->sc_fsize)
				return (EINVAL);
			break;
		case UHID_INPUT_REPORT:
		default:
			return (EINVAL);
		}
		repbuf = kmalloc(request->len, M_DEVBUF, M_WAITOK | M_ZERO);
		error = copyin(request->report, repbuf, request->len);
		if (error == 0 && request->kind == UHID_FEATURE_REPORT) {
			error = HID_SET_FEATURE(device_get_parent(sc->sc_dev),
			    request->id, repbuf, request->len);
		} else if (error == 0 && request->kind == UHID_OUTPUT_REPORT) {
			struct hid_report_list rep;

			rep.len = request->len;
			rep.id = request->id;
			rep.report = repbuf;
			rep.done = 0;
			rep.arg = NULL;
			rep.cb = uhid_output_ioctl_done;
			lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
			STAILQ_INSERT_TAIL(&sc->sc_pending_evs, &rep, entries);
			uhid_start_writing(sc);
			while (rep.done == 0 && sc->sc_detaching == 0) {
				lksleep(&sc->sc_pending_evs, &sc->sc_lock,
				    PCATCH, "uhid-wait", 0);
			}
			lockmgr(&sc->sc_lock, LK_RELEASE);
			break;
		}
		if (repbuf != NULL)
			kfree(repbuf, M_DEVBUF);
		break;

	/* XXX Add an ioctl for changing the in/out FIFO length. */

	default:
		error = EINVAL;
		break;
	}
	return (error);
}


static void
uhid_open(void *arg)
{
	struct uhid_softc *sc = arg;

	HID_SET_HANDLER(device_get_parent(sc->sc_dev), uhid_input_handler,
	    uhid_output_handler, sc);
	/* XXX Actually might have to subtract 1 from sc->sc_isize. */
	HID_START_READ(device_get_parent(sc->sc_dev), sc->sc_isize);
}

static void
uhid_close(void *arg)
{
	struct uhid_softc *sc = arg;

	HID_STOP_READ(device_get_parent(sc->sc_dev));
	HID_SET_HANDLER(device_get_parent(sc->sc_dev), NULL, NULL, NULL);
	STAILQ_INIT(&sc->sc_pending_evs);
	sc->sc_writing = 0;
}

static int
uhid_probe(device_t dev)
{
//	struct uhid_softc *sc = device_get_softc(dev);

	DPRINTFN(11, "\n");

	/*
	 * Don't attach to mouse and keyboard devices, hence then no
	 * "nomatch" event is generated and then hidms and hidkbd won't
	 * attach properly when loaded.
	 */
	if (uhid_probe_kbd_ms == 0 &&
	    (hid_get_protocol(dev) == HID_PROTOCOL_BOOT_KEYBOARD ||
	     hid_get_protocol(dev) == HID_PROTOCOL_MOUSE))
		return (ENXIO);

	device_set_desc(dev, "Generic HID device");

	return (BUS_PROBE_GENERIC);
}

static int
uhid_attach(device_t dev)
{
	struct uhid_softc *sc = device_get_softc(dev);
	char buf[16];
	int error = 0;

	DPRINTFN(10, "sc=%p\n", sc);

	lockinit(&sc->sc_lock, "uhid lock", 0, LK_CANRECURSE);

	sc->sc_dev = dev;
	STAILQ_INIT(&sc->sc_pending_evs);

	if (hid_get_bustype(dev) == HID_BUS_USB &&
	    hid_get_vendor(dev) == USB_VENDOR_WACOM) {

		/* the report descriptor for the Wacom Graphire is broken */

		if (hid_get_product(dev) == USB_PRODUCT_WACOM_GRAPHIRE) {

			sc->sc_repdesc_size = sizeof(uhid_graphire_report_descr);
			sc->sc_repdesc_ptr = (const char *)&uhid_graphire_report_descr;
			sc->sc_flags |= UHID_FLAG_STATIC_DESC;

		} else if (hid_get_product(dev) == USB_PRODUCT_WACOM_GRAPHIRE3_4X5) {

			static uint8_t reportbuf[] = {2, 2};

			/*
			 * The Graphire3 needs 0x0202 to be written to
			 * feature report ID 2 before it'll start
			 * returning digitizer data.
			 */
			error = HID_SET_FEATURE(device_get_parent(dev), 2,
			    reportbuf, sizeof(reportbuf));

			if (error) {
#if 0
				DPRINTF("set report failed, error=%s (ignored)\n",
				    usbd_errstr(error));
#else
				DPRINTF(
				    "set report failed, error=%d (ignored)\n",
				    error);
#endif
			}
			sc->sc_repdesc_size = sizeof(uhid_graphire3_4x5_report_descr);
			sc->sc_repdesc_ptr = (const char *)&uhid_graphire3_4x5_report_descr;
			sc->sc_flags |= UHID_FLAG_STATIC_DESC;
		}
	}
	if (sc->sc_repdesc_ptr == NULL) {
		HID_GET_DESCRIPTOR(device_get_parent(dev), &sc->sc_repdesc_ptr,
		    &sc->sc_repdesc_size);
	}
	HID_SETIDLE(device_get_parent(dev), 0, 0);

	sc->sc_isize = hid_report_size_a
	    (sc->sc_repdesc_ptr, sc->sc_repdesc_size, hid_input, &sc->sc_iid);

	sc->sc_osize = hid_report_size_a
	    (sc->sc_repdesc_ptr, sc->sc_repdesc_size, hid_output, &sc->sc_oid);

	sc->sc_fsize = hid_report_size_a
	    (sc->sc_repdesc_ptr, sc->sc_repdesc_size, hid_feature, &sc->sc_fid);

	sc->tmpbuf = kmalloc(sizeof(struct hid_stream_report) + sc->sc_isize,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	ksnprintf(buf, sizeof(buf), "uhid%d", device_get_unit(dev));
	/* XXX Extend flexfifo, to include an optional write buffer. */
	/* XXX Compute fifo buffer size, depending on the input report size. */
	sc->sc_fifo = flexfifo_create(
	    sizeof(struct hid_stream_report) + sc->sc_isize,
	    64, &uhid_fifo_ops, device_get_unit(dev), buf, sc, sc->sc_isize);
	flexfifo_setflags(sc->sc_fifo, FLEXFIFO_FLAG_SINGLEPKT);

	return (0);			/* success */
}

static int
uhid_detach(device_t dev)
{
	struct uhid_softc *sc = device_get_softc(dev);

	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	sc->sc_detaching = 1;
	lockmgr(&sc->sc_lock, LK_RELEASE);
	wakeup(&sc->sc_pending_evs);

	HID_STOP_READ(device_get_parent(dev));
	HID_SET_HANDLER(device_get_parent(dev), NULL, NULL, NULL);

	if (sc->sc_fifo != NULL)
		flexfifo_destroy(sc->sc_fifo);

	if (sc->tmpbuf != NULL)
		kfree(sc->tmpbuf, M_DEVBUF);

	lockuninit(&sc->sc_lock);

	return (0);
}

static devclass_t uhid_devclass;

static device_method_t uhid_methods[] = {
	DEVMETHOD(device_probe, uhid_probe),
	DEVMETHOD(device_attach, uhid_attach),
	DEVMETHOD(device_detach, uhid_detach),

	DEVMETHOD_END
};

static driver_t uhid_driver = {
	.name = "uhid",
	.methods = uhid_methods,
	.size = sizeof(struct uhid_softc),
};

DRIVER_MODULE(uhid, usbhid, uhid_driver, uhid_devclass, NULL, NULL);
MODULE_DEPEND(uhid, hidbus, 1, 1, 1);
MODULE_VERSION(uhid, 1);

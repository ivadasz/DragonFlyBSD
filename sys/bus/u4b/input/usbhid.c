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

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>

#include "usbdevs.h"
#include <bus/u4b/usb.h>
#include <bus/u4b/usbdi.h>
#include <bus/u4b/usbdi_util.h>
#include <bus/u4b/usbhid.h>
#include <bus/u4b/usb_core.h>

#define	USB_DEBUG_VAR usbhid_debug
#include <bus/u4b/usb_debug.h>

#include <bus/u4b/input/usb_rdesc.h>
#include <bus/u4b/quirk/usb_quirk.h>

#include <bus/hid/hidvar.h>
#include <bus/hid/hid_common.h>
#include "hid_if.h"

#ifdef USB_DEBUG
static int usbhid_debug = 0;

static SYSCTL_NODE(_hw_usb, OID_AUTO, usbhid, CTLFLAG_RW, 0, "USB usbhid");
SYSCTL_INT(_hw_usb_usbhid, OID_AUTO, debug, CTLFLAG_RW,
    &usbhid_debug, 0, "Debug level");
TUNABLE_INT("hw.usb.usbhid.debug", &usbhid_debug);
#endif

#define	USBHID_BSIZE	1024		/* bytes, buffer size */
#define	USBHID_FRAME_NUM 	  50		/* bytes, frame number */

enum {
	USBHID_INTR_DT_WR,
	USBHID_INTR_DT_RD,
	USBHID_CTRL_DT_WR,
#if 0
	USBHID_CTRL_DT_RD,
#endif
	USBHID_N_TRANSFER,
};

struct usbhid_ivars {
	uint8_t bootproto;
	uint16_t product_id;
	uint16_t vendor_id;
	const char *serial_str;
	int interval_ms;
	int protocol;
};

struct usbhid_softc {
	device_t sc_dev;
	struct lock sc_lock;

	struct usbhid_ivars ivar;

	struct usb_xfer *sc_xfer[USBHID_N_TRANSFER];
	struct usb_device *sc_udev;
	void   *sc_repdesc_ptr;

	usb_timeout_t sc_orig_interval;

	uint32_t sc_isize;
	uint32_t sc_osize;
	uint32_t sc_fsize;

	uint32_t sc_max_isize;

	uint16_t sc_repdesc_size;

	uint8_t sc_buffer[1024];

	uint8_t	sc_iface_no;
	uint8_t	sc_iface_index;
	uint8_t	sc_iid;
	uint8_t	sc_oid;
	uint8_t	sc_fid;
	uint8_t	sc_flags;
#define	USBHID_FLAG_IMMED        0x01	/* set if read should be immediate */
#define	USBHID_FLAG_STATIC_DESC  0x04	/* set if report descriptors are
					 * static */

	int sc_have_multi_id;
	device_t child;
	hid_input_handler_t input_handler;
	hid_output_handler_t output_handler;
	void *handler_arg;

	char *output;
	uint16_t output_len;
	uint8_t output_id;
};

static const uint8_t usbhid_xb360gp_report_descr[] = {USBHID_XB360GP_REPORT_DESCR()};

/* prototypes */

static device_probe_t usbhid_probe;
static device_attach_t usbhid_attach;
static device_detach_t usbhid_detach;

static usb_callback_t usbhid_intr_write_callback;
static usb_callback_t usbhid_intr_read_callback;
static usb_callback_t usbhid_write_callback;
#if 0
static usb_callback_t usbhid_read_callback;
#endif

static void usbhid_dump(u_char *ptr, uint16_t len);

static void
usbhid_intr_write_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct usbhid_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
	case USB_ST_SETUP:
tr_setup:
		if (sc->output == NULL || sc->output_len == 0)
			return;
		pc = usbd_xfer_get_frame(xfer, 0);
		if (sc->sc_have_multi_id) {
			usbd_copy_in(pc, 0, &sc->output_id, 1);
			usbd_copy_in(pc, 1, sc->output, sc->output_len);
			usbd_xfer_set_frame_len(xfer, 0, sc->output_len + 1);
		} else {
			usbd_copy_in(pc, 0, sc->output, sc->output_len);
			usbd_xfer_set_frame_len(xfer, 0, sc->output_len);
		}
		usbd_transfer_submit(xfer);
		if (sc->output_handler != NULL) {
			hid_output_handler_t fn = sc->output_handler;
			void *arg = sc->handler_arg;
			char *buffer = sc->output;
			uint16_t len = sc->output_len;

			sc->output = NULL;
			sc->output_len = 0;
			sc->output_id = 0;
			lockmgr(&sc->sc_lock, LK_RELEASE);
			fn(buffer, len, arg);
			lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
		} else {
			sc->output = NULL;
			sc->output_len = 0;
			sc->output_id = 0;
		}
		return;

	default:			/* Error */
		if (error != USB_ERR_CANCELLED) {
			/* try to clear stall first */
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		return;
	}
}

static void
usbhid_intr_read_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct usbhid_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;
	int actlen;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		DPRINTF("transferred!\n");

		pc = usbd_xfer_get_frame(xfer, 0);

		/* 
		 * If the ID byte is non zero we allow descriptors
		 * having multiple sizes:
		 */
		if ((actlen >= (int)sc->sc_max_isize) ||
		    ((actlen > 0) && (sc->sc_iid != 0))) {
			/* limit report length to the maximum */
			if (actlen > (int)sc->sc_max_isize)
				actlen = sc->sc_max_isize;
			usbd_copy_out(pc, 0, sc->sc_buffer, actlen);
#ifdef USB_DEBUG
			if (usbhid_debug >= 6) {
				device_printf(sc->sc_dev,
				    "report (length: %u):\n",
				    sc->sc_max_isize);
				usbhid_dump(sc->sc_buffer, actlen);
			}
#endif
			if (sc->input_handler != NULL) {
				uint8_t id = 0;
				uint8_t *buffer = sc->sc_buffer;
				if (sc->sc_have_multi_id) {
					id = buffer[0];
					buffer++;
					actlen--;
				}
				usbd_xfer_set_frame_len(xfer, 0,
				    sc->sc_max_isize);
				usbd_transfer_submit(xfer);
				hid_input_handler_t fn = sc->input_handler;
				void *arg = sc->handler_arg;
				lockmgr(&sc->sc_lock, LK_RELEASE);
				fn(id, buffer, actlen, arg);
				lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
				return;
			}
		} else {
			/* ignore it */
			DPRINTF("ignored transfer, %d bytes\n", actlen);
		}

	case USB_ST_SETUP:
re_submit:
		usbd_xfer_set_frame_len(xfer, 0, sc->sc_max_isize);
		usbd_transfer_submit(xfer);
		return;

	default:			/* Error */
		if (error != USB_ERR_CANCELLED) {
			/* try to clear stall first */
			usbd_xfer_set_stall(xfer);
			goto re_submit;
		}
		return;
	}
}

static void
usbhid_fill_set_report(struct usb_device_request *req, uint8_t iface_no,
    uint8_t type, uint8_t id, uint16_t size)
{
	req->bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req->bRequest = UR_SET_REPORT;
	USETW2(req->wValue, type, id);
	req->wIndex[0] = iface_no;
	req->wIndex[1] = 0;
	USETW(req->wLength, size);
}

#if 0
static void
usbhid_fill_get_report(struct usb_device_request *req, uint8_t iface_no,
    uint8_t type, uint8_t id, uint16_t size)
{
	req->bmRequestType = UT_READ_CLASS_INTERFACE;
	req->bRequest = UR_GET_REPORT;
	USETW2(req->wValue, type, id);
	req->wIndex[0] = iface_no;
	req->wIndex[1] = 0;
	USETW(req->wLength, size);
}
#endif

static void
usbhid_write_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct usbhid_softc *sc = usbd_xfer_softc(xfer);
	struct usb_device_request req;
	struct usb_page_cache *pc;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
	case USB_ST_SETUP:
		if (sc->output == NULL || sc->output_len == 0)
			return;
		usbhid_fill_set_report(&req, sc->sc_iface_no,
		    UHID_OUTPUT_REPORT, sc->output_id, sc->output_len);
		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_in(pc, 0, &req, sizeof(req));
		pc = usbd_xfer_get_frame(xfer, 1);
		usbd_copy_in(pc, 0, sc->output, sc->output_len);
		usbd_xfer_set_frame_len(xfer, 0, sizeof(req));
		usbd_xfer_set_frame_len(xfer, 1, sc->output_len);
		usbd_xfer_set_frames(xfer, 2);
		usbd_transfer_submit(xfer);
		if (sc->output_handler != NULL) {
			hid_output_handler_t fn = sc->output_handler;
			void *arg = sc->handler_arg;
			char *buffer = sc->output;
			uint16_t len = sc->output_len;

			sc->output = NULL;
			sc->output_len = 0;
			sc->output_id = 0;
			lockmgr(&sc->sc_lock, LK_RELEASE);
			fn(buffer, len, arg);
			lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
		} else {
			sc->output = NULL;
			sc->output_len = 0;
			sc->output_id = 0;
		}
		return;

	default:
		/* bomb out */
		return;
	}
}

#if 0
static void
usbhid_read_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct usbhid_softc *sc = usbd_xfer_softc(xfer);
	struct usb_device_request req;
	struct usb_page_cache *pc;

	pc = usbd_xfer_get_frame(xfer, 0);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		/* XXX handle data */
		usbd_copy_out(pc, sizeof(req), sc->sc_buffer, sc->sc_max_isize);
		kprintf("usbhid: %s: report (length: %u):\n",
		    __func__, sc->sc_max_isize);
//		usbhid_dump(sc->sc_buffer, sc->sc_max_isize);
		return;

	case USB_ST_SETUP:
		usbhid_fill_get_report
		    (&req, sc->sc_iface_no, UHID_INPUT_REPORT,
		    sc->sc_iid, sc->sc_max_isize);

		usbd_copy_in(pc, 0, &req, sizeof(req));

		usbd_xfer_set_frame_len(xfer, 0, sizeof(req));
		usbd_xfer_set_frame_len(xfer, 1, sc->sc_max_isize);
		usbd_xfer_set_frames(xfer, sc->sc_max_isize ? 2 : 1);
		usbd_transfer_submit(xfer);
		return;

	default:			/* Error */
		/* bomb out */
		/* XXX signal error somehow */
		return;
	}
}
#endif

static const struct usb_config usbhid_config[USBHID_N_TRANSFER] = {

	[USBHID_INTR_DT_WR] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.flags = {.pipe_bof = 1,.no_pipe_ok = 1, },
		.bufsize = USBHID_BSIZE,
		.callback = &usbhid_intr_write_callback,
	},

	[USBHID_INTR_DT_RD] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.bufsize = USBHID_BSIZE,
		.callback = &usbhid_intr_read_callback,
	},

	[USBHID_CTRL_DT_WR] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(struct usb_device_request) + USBHID_BSIZE,
		.callback = &usbhid_write_callback,
		.timeout = 1000,	/* 1 second */
	},

#if 0
	[USBHID_CTRL_DT_RD] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(struct usb_device_request) + USBHID_BSIZE,
		.callback = &usbhid_read_callback,
		.timeout = 1000,	/* 1 second */
	},
#endif
};

static void
usbhid_dump(u_char *ptr, uint16_t len)
{
	unsigned i;

#define GET(n) ((n) < len ? ptr[(n)] : 0)

	for (i = 0; i < len; i += 8)
		kprintf("%02x %02x %02x %02x %02x %02x %02x %02x\n",
		    GET(i), GET(i+1), GET(i+2), GET(i+3),
		    GET(i+4), GET(i+5), GET(i+6), GET(i+7));

#undef GET
}

static const STRUCT_USB_HOST_ID usbhid_devs[] = {
	/* generic HID class */
	{USB_IFACE_CLASS(UICLASS_HID),},
	/* the Xbox 360 gamepad doesn't use the HID class */
	{USB_IFACE_CLASS(UICLASS_VENDOR),
	 USB_IFACE_SUBCLASS(UISUBCLASS_XBOX360_CONTROLLER),
	 USB_IFACE_PROTOCOL(UIPROTO_XBOX360_GAMEPAD),},
};

static int
usbhid_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	int error;

	DPRINTFN(11, "\n");

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);

	error = usbd_lookup_id_by_uaa(usbhid_devs, sizeof(usbhid_devs), uaa);
	if (error)
		return (error);

	if (usb_test_quirk(uaa, UQ_HID_IGNORE))
		return (ENXIO);

	return (BUS_PROBE_GENERIC);
}

static int
usbhid_have_multi_id(void *buf, int len, int kind)
{
	struct hid_data *d;
	struct hid_item h;

	h.report_ID = 0;
	for (d = hid_start_parse(buf, len, kind); hid_get_item(d, &h); ) {
		if (h.report_ID != 0)
			return (1);
	}
	hid_end_parse(d);
	return (0);
}

static int
usbhid_maxrepid(void *buf, int len)
{
	struct hid_data *d;
	struct hid_item h;
	int maxid;

	maxid = -1;
	h.report_ID = 0;
	for (d = hid_start_parse(buf, len, hid_input); hid_get_item(d, &h); ) {
		if (h.report_ID > maxid)
			maxid = h.report_ID;
	}
	hid_end_parse(d);
	return (maxid);
}

static void
usbhid_parse_descriptor(device_t dev)
{
	struct usbhid_softc *sc = device_get_softc(dev);
	void *buf = sc->sc_repdesc_ptr;
	int len = sc->sc_repdesc_size;
	int i, maxid, sz;

	maxid = usbhid_maxrepid(buf, len);
	device_printf(dev, "max report id: %d\n", maxid);

	for (i = 0; i <= maxid; i++) {
		sz = hid_report_size(buf, len, hid_input, i);
		device_printf(dev, "usbhid: report id=%d size=%d\n", i, sz);
	}
	if (sc->child == NULL) {
		sc->child = device_add_child(dev, NULL, -1);
		device_set_ivars(sc->child, &sc->ivar);
	}
}

static void
usbhid_get_descriptor(device_t dev, char **descp, uint16_t *sizep)
{
	struct usbhid_softc *sc = device_get_softc(dev);

	*descp = sc->sc_repdesc_ptr;
	*sizep = sc->sc_repdesc_size;
}

static void
usbhid_set_handler(device_t dev, hid_input_handler_t input,
    hid_output_handler_t output, void *arg)
{
	struct usbhid_softc *sc = device_get_softc(dev);

	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	sc->input_handler = input;
	sc->output_handler = output;
	sc->handler_arg = arg;
	lockmgr(&sc->sc_lock, LK_RELEASE);
}

static void
usbhid_start_read(device_t dev, uint16_t max_len)
{
	struct usbhid_softc *sc = device_get_softc(dev);

	if (max_len > 0 && max_len + (sc->sc_have_multi_id ? 1 : 0) >
	    (int)usbd_xfer_max_framelen(sc->sc_xfer[USBHID_INTR_DT_RD])) {
		DPRINTF("WARNING: input report size, %d bytes, is larger "
		    "than interrupt size, %d bytes!\n", max_len,
		    usbd_xfer_max_framelen(sc->sc_xfer[USBHID_INTR_DT_RD]));
	}

	if (max_len > USBHID_BSIZE) {
		DPRINTF("input size is too large, %d bytes (truncating)\n",
		    max_len);
		max_len = USBHID_BSIZE;
	}

	if (max_len == 0)
		max_len = sc->sc_isize;

	device_printf(dev, "starting interrupt input transfer\n");
	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	sc->sc_max_isize = max_len;
	usbd_transfer_stop(sc->sc_xfer[USBHID_INTR_DT_RD]);
	if (sc->ivar.interval_ms >= 0 && sc->ivar.interval_ms <= 1000) {
		usbd_xfer_set_interval(sc->sc_xfer[USBHID_INTR_DT_RD],
		    sc->ivar.interval_ms);
	}
	usbd_transfer_start(sc->sc_xfer[USBHID_INTR_DT_RD]);
	lockmgr(&sc->sc_lock, LK_RELEASE);
}

static void
usbhid_stop_read(device_t dev)
{
	struct usbhid_softc *sc = device_get_softc(dev);

	device_printf(dev, "stopping interrupt input transfer\n");
	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	usbd_transfer_stop(sc->sc_xfer[USBHID_INTR_DT_RD]);
	lockmgr(&sc->sc_lock, LK_RELEASE);
}

static void
usbhid_input_poll(device_t dev)
{
	struct usbhid_softc *sc = device_get_softc(dev);

	usbd_transfer_poll(&sc->sc_xfer[USBHID_INTR_DT_RD], 1);
}

static void
usbhid_setidle(device_t dev, uint8_t duration, uint8_t id)
{
	struct usbhid_softc *sc = device_get_softc(dev);
	int error;

	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	error = usbd_req_set_idle(sc->sc_udev, &sc->sc_lock,
	    sc->sc_iface_index, duration, id);
	lockmgr(&sc->sc_lock, LK_RELEASE);

	if (error) {
		DPRINTF("set idle failed, error=%s (ignored)\n",
		    usbd_errstr(error));
	}
}

static void
usbhid_set_report(device_t dev, uint8_t id, uint8_t *buf, uint16_t len)
{
	struct usbhid_softc *sc = device_get_softc(dev);
	int needstart;

	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	needstart = sc->output == NULL;
	if (sc->output != NULL && sc->output_handler != NULL)
		sc->output_handler(sc->output, 0, sc->handler_arg);

	sc->output = buf;
	sc->output_len = len;
	sc->output_id = id;

	if (sc->output == NULL || sc->output_len == 0) {
		usbd_transfer_stop(sc->sc_xfer[USBHID_INTR_DT_WR]);
		usbd_transfer_stop(sc->sc_xfer[USBHID_CTRL_DT_WR]);
		sc->output = NULL;
		sc->output_len = 0;
		sc->output_id = 0;
	} else if (needstart) {
		if (sc->sc_xfer[USBHID_INTR_DT_WR] == NULL) {
			usbd_transfer_start(sc->sc_xfer[USBHID_CTRL_DT_WR]);
		} else {
			usbd_transfer_start(sc->sc_xfer[USBHID_INTR_DT_WR]);
		}
	}
	lockmgr(&sc->sc_lock, LK_RELEASE);
}

static int
usbhid_set_feature(device_t dev, uint8_t id, uint8_t *buf, uint16_t len)
{
	struct usbhid_softc *sc = device_get_softc(dev);
	int err;

	err = usbd_req_set_report(sc->sc_udev, NULL, buf, len,
	    sc->sc_iface_index, hid_feature, id);
	if (err)
		return (ENXIO);

	return 0;
}

static int
usbhid_get_report(device_t dev, uint8_t id, uint8_t *buf, uint16_t len, int type)
{
	struct usbhid_softc *sc = device_get_softc(dev);
	int err;

	KKASSERT(type == hid_input || type == hid_feature);

	err = usbd_req_get_report(sc->sc_udev, NULL, buf, len,
	    sc->sc_iface_index, type, id);
	if (err)
		return (ENXIO);

	return 0;
}

static int
usbhid_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct usbhid_softc *sc = device_get_softc(dev);
	int error = 0;

	DPRINTFN(10, "sc=%p\n", sc);

	sc->sc_dev = dev;
	device_set_usb_desc(dev);

	lockinit(&sc->sc_lock, "usbhid lock", 0, LK_CANRECURSE);

	sc->sc_udev = uaa->device;

	sc->sc_iface_no = uaa->info.bIfaceNum;
	sc->sc_iface_index = uaa->info.bIfaceIndex;

	error = usbd_transfer_setup(uaa->device,
	    &uaa->info.bIfaceIndex, sc->sc_xfer, usbhid_config,
	    USBHID_N_TRANSFER, sc, &sc->sc_lock);

	if (error) {
		DPRINTF("error=%s\n", usbd_errstr(error));
		goto detach;
	}
	if ((uaa->info.bInterfaceClass == UICLASS_VENDOR) &&
		    (uaa->info.bInterfaceSubClass == UISUBCLASS_XBOX360_CONTROLLER) &&
	    (uaa->info.bInterfaceProtocol == UIPROTO_XBOX360_GAMEPAD)) {

		/* the Xbox 360 gamepad has no report descriptor */
		sc->sc_repdesc_size = sizeof(usbhid_xb360gp_report_descr);
		sc->sc_repdesc_ptr = __DECONST(void *, &usbhid_xb360gp_report_descr);
		sc->sc_flags |= USBHID_FLAG_STATIC_DESC;
	}
	if (sc->sc_repdesc_ptr == NULL) {
		error = usbd_req_get_hid_desc(uaa->device, NULL,
		    &sc->sc_repdesc_ptr, &sc->sc_repdesc_size,
		    M_USBDEV, uaa->info.bIfaceIndex);

		if (error) {
			device_printf(dev, "no report descriptor\n");
			goto detach;
		}
	}

	sc->sc_isize = hid_report_size_a
	    (sc->sc_repdesc_ptr, sc->sc_repdesc_size, hid_input, &sc->sc_iid);

	sc->sc_osize = hid_report_size_a
	    (sc->sc_repdesc_ptr, sc->sc_repdesc_size, hid_output, &sc->sc_oid);

	sc->sc_fsize = hid_report_size_a
	    (sc->sc_repdesc_ptr, sc->sc_repdesc_size, hid_feature, &sc->sc_fid);

	sc->sc_have_multi_id = usbhid_have_multi_id(sc->sc_repdesc_ptr,
	    sc->sc_repdesc_size, hid_input);

	if (sc->sc_isize >
	    (int)usbd_xfer_max_framelen(sc->sc_xfer[USBHID_INTR_DT_RD])) {
		DPRINTF("WARNING: input report size, %d bytes, is larger "
		    "than interrupt size, %d bytes!\n", sc->sc_isize,
		    usbd_xfer_max_framelen(sc->sc_xfer[USBHID_INTR_DT_RD]));
	}

	if (sc->sc_isize > USBHID_BSIZE) {
		DPRINTF("input size is too large, "
		    "%d bytes (truncating)\n",
		    sc->sc_isize);
		sc->sc_isize = USBHID_BSIZE;
	}
	if (sc->sc_osize > USBHID_BSIZE) {
		DPRINTF("output size is too large, "
		    "%d bytes (truncating)\n",
		    sc->sc_osize);
		sc->sc_osize = USBHID_BSIZE;
	}
	if (sc->sc_fsize > USBHID_BSIZE) {
		DPRINTF("feature size is too large, "
		    "%d bytes (truncating)\n",
		    sc->sc_fsize);
		sc->sc_fsize = USBHID_BSIZE;
	}

	device_printf(dev,
	    "iid=%u isize=%u, oid=%u osize=%u, fid=%u fsize=%u\n",
	    sc->sc_iid, sc->sc_isize,
	    sc->sc_oid, sc->sc_osize,
	    sc->sc_fid, sc->sc_fsize);
	device_printf(dev, "report descriptor (length: %u):\n",
	    sc->sc_repdesc_size);
	usbhid_dump(sc->sc_repdesc_ptr, sc->sc_repdesc_size);

	usbhid_parse_descriptor(dev);
	sc->ivar.bootproto = HID_BOOTPROTO_OTHER;
	if (uaa->info.bInterfaceSubClass == UISUBCLASS_BOOT) {
		if (uaa->info.bInterfaceProtocol == UIPROTO_BOOT_KEYBOARD &&
		    usb_test_quirk(uaa, UQ_KBD_BOOTPROTO)) {
			/* Prefer boot protocol for quirked devices. */
			usbd_req_set_protocol(uaa->device, NULL,
			    uaa->info.bIfaceIndex, 0);
			sc->ivar.bootproto = HID_BOOTPROTO_KEYBOARD;
		} else {
			usbd_req_set_protocol(uaa->device, NULL,
			    uaa->info.bIfaceIndex, 1);
		}
	}
	sc->sc_orig_interval = sc->sc_xfer[USBHID_INTR_DT_RD]->interval;
	sc->ivar.product_id = uaa->info.idProduct;
	sc->ivar.vendor_id = uaa->info.idVendor;
	sc->ivar.serial_str = usb_get_serial(uaa->device);
	sc->ivar.interval_ms = -1;
	if (uaa->info.bInterfaceProtocol == UIPROTO_BOOT_KEYBOARD)
		sc->ivar.protocol = HID_PROTOCOL_BOOT_KEYBOARD;
	else if (uaa->info.bInterfaceProtocol == UIPROTO_MOUSE)
		sc->ivar.protocol = HID_PROTOCOL_MOUSE;
	else
		sc->ivar.protocol = HID_PROTOCOL_OTHER;
	bus_generic_attach(dev);

	return (0);			/* success */

detach:
	usbhid_detach(dev);
	return (ENOMEM);
}

static int
usbhid_detach(device_t dev)
{
	struct usbhid_softc *sc = device_get_softc(dev);

	if (bus_generic_detach(dev) != 0)
		return (EBUSY);

	usbd_transfer_unsetup(sc->sc_xfer, USBHID_N_TRANSFER);

	if (sc->sc_repdesc_ptr) {
		if (!(sc->sc_flags & USBHID_FLAG_STATIC_DESC)) {
			kfree(sc->sc_repdesc_ptr, M_USBDEV);
		}
	}
	lockuninit(&sc->sc_lock);

	return (0);
}

static void
usbhid_child_detached(device_t dev, device_t child __unused)
{
	struct usbhid_softc *sc = device_get_softc(dev);

	lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
	sc->ivar.interval_ms = -1;
	usbd_transfer_stop(sc->sc_xfer[USBHID_INTR_DT_RD]);
	/* XXX Maybe also restore interval, when cdev is closed. */
	usbd_xfer_set_interval(sc->sc_xfer[USBHID_INTR_DT_RD],
	    sc->sc_orig_interval);
	lockmgr(&sc->sc_lock, LK_RELEASE);
}

static int
usbhid_read_ivar(device_t bus, device_t child, int which, uintptr_t *result)
{
	struct usbhid_ivars *ivar = device_get_ivars(child);

	switch (which) {
	case HID_IVAR_BOOTPROTO:
		*(int *)result = ivar->bootproto;
		break;
	case HID_IVAR_BUSTYPE:
		*(int *)result = HID_BUS_USB;
		break;
	case HID_IVAR_PRODUCT:
		*(uint16_t *)result = ivar->product_id;
		break;
	case HID_IVAR_VENDOR:
		*(uint16_t *)result = ivar->vendor_id;
		break;
	case HID_IVAR_SERIAL:
		*(const char **)result = ivar->serial_str;
		break;
	case HID_IVAR_PROTOCOL:
		*(int *)result = ivar->protocol;
		break;
	default:
		return (EINVAL);
	}
	return (0);
}

static int
usbhid_write_ivar(device_t bus, device_t child, int which, uintptr_t value)
{
	struct usbhid_ivars *ivar = device_get_ivars(child);

	switch (which) {
	case HID_IVAR_INTERVAL_MS:
		if (value <= 1000)
			ivar->interval_ms = value;
		else
			return (EINVAL);
		break;
	default:
		return (EINVAL);
	}
	return (0);
}

static devclass_t usbhid_devclass;

static device_method_t usbhid_methods[] = {
	DEVMETHOD(device_probe, usbhid_probe),
	DEVMETHOD(device_attach, usbhid_attach),
	DEVMETHOD(device_detach, usbhid_detach),

	DEVMETHOD(bus_child_detached, usbhid_child_detached),
	DEVMETHOD(bus_read_ivar, usbhid_read_ivar),
	DEVMETHOD(bus_write_ivar, usbhid_write_ivar),

	DEVMETHOD(hid_get_descriptor, usbhid_get_descriptor),
	DEVMETHOD(hid_set_handler, usbhid_set_handler),
	DEVMETHOD(hid_start_read, usbhid_start_read),
	DEVMETHOD(hid_stop_read, usbhid_stop_read),
	DEVMETHOD(hid_input_poll, usbhid_input_poll),
	DEVMETHOD(hid_setidle, usbhid_setidle),
	DEVMETHOD(hid_set_report, usbhid_set_report),
	DEVMETHOD(hid_set_feature, usbhid_set_feature),
	DEVMETHOD(hid_get_report, usbhid_get_report),

	DEVMETHOD_END
};

static driver_t usbhid_driver = {
	.name = "usbhid",
	.methods = usbhid_methods,
	.size = sizeof(struct usbhid_softc),
};

DRIVER_MODULE(usbhid, uhub, usbhid_driver, usbhid_devclass, NULL, NULL);
MODULE_DEPEND(usbhid, usb, 1, 1, 1);
MODULE_DEPEND(usbhid, hidbus, 1, 1, 1);
MODULE_VERSION(usbhid, 1);

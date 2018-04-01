#include <sys/cdefs.h>
__FBSDID("$FreeBSD: head/sys/dev/usb/input/ukbd.c 262972 2014-03-10 08:52:30Z hselasky $");


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
 *
 */

/*
 * HID spec: http://www.usb.org/developers/devclass_docs/HID1_11.pdf
 */

#include "opt_kbd.h"
#include "opt_hidkbd.h"
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
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/thread2.h>

#include <bus/hid/hid_common.h>
#include <bus/hid/hidvar.h>
#include "hid_if.h"

#ifdef EVDEV_SUPPORT
#include <dev/misc/evdev/input.h>
#include <dev/misc/evdev/evdev.h>
#include <bus/hid/hid_evdev_compat.h>
#endif

#include <sys/filio.h>
#include <sys/tty.h>
#include <sys/kbio.h>

#include <dev/misc/kbd/kbdreg.h>

/* the initial key map, accent map and fkey strings */
#if defined(HIDKBD_DFLT_KEYMAP) && !defined(KLD_MODULE)
#define	KBD_DFLT_KEYMAP
#include "hidkbdmap.h"
#endif

/* the following file must be included after "hidkbdmap.h" */
#include <dev/misc/kbd/kbdtables.h>

static int hidkbd_debug = 0;
static int hidkbd_no_leds = 0;
#ifdef USB_DEBUG
static int hidkbd_pollrate = 0;

static SYSCTL_NODE(_hw_hid, OID_AUTO, hidkbd, CTLFLAG_RW, 0, "USB keyboard");
SYSCTL_INT(_hw_hid_hidkbd, OID_AUTO, debug, CTLFLAG_RW,
    &hidkbd_debug, 0, "Debug level");
SYSCTL_INT(_hw_hid_hidkbd, OID_AUTO, no_leds, CTLFLAG_RW,
    &hidkbd_no_leds, 0, "Disables setting of keyboard leds");
SYSCTL_INT(_hw_hid_hidkbd, OID_AUTO, pollrate, CTLFLAG_RW,
    &hidkbd_pollrate, 0, "Force this polling rate, 1-1000Hz");

TUNABLE_INT("hw.hid.hidkbd.pollrate", &hidkbd_pollrate);
#endif
TUNABLE_INT("hw.hid.hidkbd_no_leds", &hidkbd_no_leds);
TUNABLE_INT("hw.hid.hidkbd_debug", &hidkbd_debug);

#define DPRINTFN(l, f, ...)	\
    do { if (hidkbd_debug >= (l)) { kprintf(f,## __VA_ARGS__); } } while(0)

#define DPRINTF(f, ...) DPRINTFN(1, f,## __VA_ARGS__)

#define	HIDKBD_EMULATE_ATSCANCODE       1
#define	HIDKBD_DRIVER_NAME		"hidkbd"
#define	HIDKBD_NMOD                     8	/* units */
#define	HIDKBD_NKEYCODE                 6	/* units */
#define	HIDKBD_IN_BUF_SIZE  (2*(HIDKBD_NMOD + (2*HIDKBD_NKEYCODE))) /* bytes */
#define	HIDKBD_IN_BUF_FULL  (HIDKBD_IN_BUF_SIZE / 2)	/* bytes */
#define	HIDKBD_NFKEY        (sizeof(fkey_tab)/sizeof(fkey_tab[0]))	/* units */
#define	HIDKBD_BUFFER_SIZE	      64	/* bytes */

struct hidkbd_data {
	uint16_t	modifiers;
#define	MOD_CONTROL_L	0x01
#define	MOD_CONTROL_R	0x10
#define	MOD_SHIFT_L	0x02
#define	MOD_SHIFT_R	0x20
#define	MOD_ALT_L	0x04
#define	MOD_ALT_R	0x40
#define	MOD_WIN_L	0x08
#define	MOD_WIN_R	0x80
/* internal */
#define	MOD_EJECT	0x0100
#define	MOD_FN		0x0200
	uint8_t	keycode[HIDKBD_NKEYCODE];
};

struct hidkbd_softc {
	device_t sc_dev;
	keyboard_t sc_kbd;
	keymap_t sc_keymap;
	accentmap_t sc_accmap;
	fkeytab_t sc_fkeymap[HIDKBD_NFKEY];
	struct hid_location sc_loc_apple_eject;
	struct hid_location sc_loc_apple_fn;
	struct hid_location sc_loc_ctrl_l;
	struct hid_location sc_loc_ctrl_r;
	struct hid_location sc_loc_shift_l;
	struct hid_location sc_loc_shift_r;
	struct hid_location sc_loc_alt_l;
	struct hid_location sc_loc_alt_r;
	struct hid_location sc_loc_win_l;
	struct hid_location sc_loc_win_r;
	struct hid_location sc_loc_events;
	struct hid_location sc_loc_numlock;
	struct hid_location sc_loc_capslock;
	struct hid_location sc_loc_scrolllock;
	struct callout sc_callout;
	struct hidkbd_data sc_ndata;
	struct hidkbd_data sc_odata;

	struct thread *sc_poll_thread;
#ifdef EVDEV_SUPPORT
	struct evdev_dev *sc_evdev;
#endif

	uint32_t sc_ntime[HIDKBD_NKEYCODE];
	uint32_t sc_otime[HIDKBD_NKEYCODE];
	uint32_t sc_input[HIDKBD_IN_BUF_SIZE];	/* input buffer */
	uint32_t sc_time_ms;
	uint32_t sc_composed_char;	/* composed char code, if non-zero */
#ifdef HIDKBD_EMULATE_ATSCANCODE
	uint32_t sc_buffered_char[2];
#endif
	uint32_t sc_flags;		/* flags */
#define	HIDKBD_FLAG_COMPOSE	0x00000001
#define	HIDKBD_FLAG_POLLING	0x00000002
#if 0
#define	HIDKBD_FLAG_SET_LEDS	0x00000004
#endif
#define	HIDKBD_FLAG_ATTACHED	0x00000010
#define	HIDKBD_FLAG_GONE		0x00000020

#define	HIDKBD_FLAG_HID_MASK	0x003fffc0
#define	HIDKBD_FLAG_APPLE_EJECT	0x00000040
#define	HIDKBD_FLAG_APPLE_FN	0x00000080
#define	HIDKBD_FLAG_APPLE_SWAP	0x00000100
#define	HIDKBD_FLAG_TIMER_RUNNING	0x00000200
#define	HIDKBD_FLAG_CTRL_L	0x00000400
#define	HIDKBD_FLAG_CTRL_R	0x00000800
#define	HIDKBD_FLAG_SHIFT_L	0x00001000
#define	HIDKBD_FLAG_SHIFT_R	0x00002000
#define	HIDKBD_FLAG_ALT_L		0x00004000
#define	HIDKBD_FLAG_ALT_R		0x00008000
#define	HIDKBD_FLAG_WIN_L		0x00010000
#define	HIDKBD_FLAG_WIN_R		0x00020000
#define	HIDKBD_FLAG_EVENTS	0x00040000
#define	HIDKBD_FLAG_NUMLOCK	0x00080000
#define	HIDKBD_FLAG_CAPSLOCK	0x00100000
#define	HIDKBD_FLAG_SCROLLLOCK 	0x00200000

	int	sc_mode;		/* input mode (K_XLATE,K_RAW,K_CODE) */
	int	sc_state;		/* shift/lock key state */
	int	sc_accents;		/* accent key index (> 0) */
	uint16_t sc_led_size;
	uint16_t sc_kbd_size;
	uint8_t sc_led_id;

	uint8_t *sc_outbuf[2];

	uint16_t sc_inputs;
	uint16_t sc_inputhead;
	uint16_t sc_inputtail;
	uint16_t sc_modifiers;

	uint8_t	sc_leds;		/* store for async led requests */
	uint8_t	sc_iface_index;
	uint8_t	sc_iface_no;
	uint8_t sc_id_apple_eject;
	uint8_t sc_id_apple_fn;
	uint8_t sc_id_ctrl_l;
	uint8_t sc_id_ctrl_r;
	uint8_t sc_id_shift_l;
	uint8_t sc_id_shift_r;
	uint8_t sc_id_alt_l;
	uint8_t sc_id_alt_r;
	uint8_t sc_id_win_l;
	uint8_t sc_id_win_r;
	uint8_t sc_id_event;
	uint8_t sc_id_numlock;
	uint8_t sc_id_capslock;
	uint8_t sc_id_scrolllock;
	uint8_t sc_id_events;
	uint8_t sc_kbd_id;
};

#define	KEY_ERROR	  0x01

#define	KEY_PRESS	  0
#define	KEY_RELEASE	  0x400
#define	KEY_INDEX(c)	  ((c) & 0xFF)

#define	SCAN_PRESS	  0
#define	SCAN_RELEASE	  0x80
#define	SCAN_PREFIX_E0	  0x100
#define	SCAN_PREFIX_E1	  0x200
#define	SCAN_PREFIX_CTL	  0x400
#define	SCAN_PREFIX_SHIFT 0x800
#define	SCAN_PREFIX	(SCAN_PREFIX_E0  | SCAN_PREFIX_E1 | \
			 SCAN_PREFIX_CTL | SCAN_PREFIX_SHIFT)
#define	SCAN_CHAR(c)	((c) & 0x7f)

#define	HIDKBD_LOCK(sc)	lockmgr(&(sc)->sc_kbd.kb_lock, LK_EXCLUSIVE)
#define	HIDKBD_UNLOCK(sc)	lockmgr(&(sc)->sc_kbd.kb_lock, LK_RELEASE)

#ifdef	INVARIANTS

/*
 * Assert that the lock is held in all contexts
 * where the code can be executed.
 */
#define	HIDKBD_LOCK_ASSERT()

/*
 * Assert that the lock is held in the contexts
 * where it really has to be so.
 */
#define	HIDKBD_CTX_LOCK_ASSERT()
#else

#define HIDKBD_LOCK_ASSERT()	(void)0
#define HIDKBD_CTX_LOCK_ASSERT()	(void)0

#endif

struct hidkbd_mods {
	uint32_t mask, key;
};

static const struct hidkbd_mods hidkbd_mods[HIDKBD_NMOD] = {
	{MOD_CONTROL_L, 0xe0},
	{MOD_CONTROL_R, 0xe4},
	{MOD_SHIFT_L, 0xe1},
	{MOD_SHIFT_R, 0xe5},
	{MOD_ALT_L, 0xe2},
	{MOD_ALT_R, 0xe6},
	{MOD_WIN_L, 0xe3},
	{MOD_WIN_R, 0xe7},
};

#define	NN 0				/* no translation */
/*
 * Translate USB keycodes to AT keyboard scancodes.
 */
/*
 * FIXME: Mac USB keyboard generates:
 * 0x53: keypad NumLock/Clear
 * 0x66: Power
 * 0x67: keypad =
 * 0x68: F13
 * 0x69: F14
 * 0x6a: F15
 */
static const uint8_t hidkbd_trtab[256] = {
	0, 0, 0, 0, 30, 48, 46, 32,	/* 00 - 07 */
	18, 33, 34, 35, 23, 36, 37, 38,	/* 08 - 0F */
	50, 49, 24, 25, 16, 19, 31, 20,	/* 10 - 17 */
	22, 47, 17, 45, 21, 44, 2, 3,	/* 18 - 1F */
	4, 5, 6, 7, 8, 9, 10, 11,	/* 20 - 27 */
	28, 1, 14, 15, 57, 12, 13, 26,	/* 28 - 2F */
	27, 43, 43, 39, 40, 41, 51, 52,	/* 30 - 37 */
	53, 58, 59, 60, 61, 62, 63, 64,	/* 38 - 3F */
	65, 66, 67, 68, 87, 88, 92, 70,	/* 40 - 47 */
	104, 102, 94, 96, 103, 99, 101, 98,	/* 48 - 4F */
	97, 100, 95, 69, 91, 55, 74, 78,/* 50 - 57 */
	89, 79, 80, 81, 75, 76, 77, 71,	/* 58 - 5F */
	72, 73, 82, 83, 86, 107, 122, NN,	/* 60 - 67 */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* 68 - 6F */
	NN, NN, NN, NN, 115, 108, 111, 113,	/* 70 - 77 */
	109, 110, 112, 118, 114, 116, 117, 119,	/* 78 - 7F */
	121, 120, NN, NN, NN, NN, NN, 123,	/* 80 - 87 */
	124, 125, 126, 127, 128, NN, NN, NN,	/* 88 - 8F */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* 90 - 97 */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* 98 - 9F */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* A0 - A7 */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* A8 - AF */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* B0 - B7 */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* B8 - BF */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* C0 - C7 */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* C8 - CF */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* D0 - D7 */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* D8 - DF */
	29, 42, 56, 105, 90, 54, 93, 106,	/* E0 - E7 */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* E8 - EF */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* F0 - F7 */
	NN, NN, NN, NN, NN, NN, NN, NN,	/* F8 - FF */
};

static const uint8_t hidkbd_boot_desc[] = {
	0x05, 0x01, 0x09, 0x06, 0xa1,
	0x01, 0x05, 0x07, 0x19, 0xe0,
	0x29, 0xe7, 0x15, 0x00, 0x25,
	0x01, 0x75, 0x01, 0x95, 0x08,
	0x81, 0x02, 0x95, 0x01, 0x75,
	0x08, 0x81, 0x01, 0x95, 0x03,
	0x75, 0x01, 0x05, 0x08, 0x19,
	0x01, 0x29, 0x03, 0x91, 0x02,
	0x95, 0x05, 0x75, 0x01, 0x91,
	0x01, 0x95, 0x06, 0x75, 0x08,
	0x15, 0x00, 0x26, 0xff, 0x00,
	0x05, 0x07, 0x19, 0x00, 0x2a,
	0xff, 0x00, 0x81, 0x00, 0xc0
};

/* prototypes */
static void	hidkbd_timeout(void *);
static void	hidkbd_set_leds(struct hidkbd_softc *, uint8_t);
#ifdef HIDKBD_EMULATE_ATSCANCODE
static int	hidkbd_key2scan(struct hidkbd_softc *, int, int, int);
#endif
static uint32_t	hidkbd_read_char(keyboard_t *, int);
static void	hidkbd_clear_state(keyboard_t *);
static int	hidkbd_ioctl(keyboard_t *, u_long, caddr_t);
static int	hidkbd_enable(keyboard_t *);
static int	hidkbd_disable(keyboard_t *);
static void	hidkbd_interrupt(struct hidkbd_softc *);
static void	hidkbd_event_keyinput(struct hidkbd_softc *);

static device_probe_t hidkbd_probe;
static device_attach_t hidkbd_attach;
static device_detach_t hidkbd_detach;
static device_resume_t hidkbd_resume;

#ifdef EVDEV_SUPPORT
static const struct evdev_methods hidkbd_evdev_methods = {
	.ev_event = evdev_ev_kbd_event,
};
#endif

static uint8_t
hidkbd_any_key_pressed(struct hidkbd_softc *sc)
{
	uint8_t i;
	uint8_t j;

	for (j = i = 0; i < HIDKBD_NKEYCODE; i++)
		j |= sc->sc_odata.keycode[i];

	return (j ? 1 : 0);
}

static void
hidkbd_start_timer(struct hidkbd_softc *sc)
{
	sc->sc_flags |= HIDKBD_FLAG_TIMER_RUNNING;
	callout_reset(&sc->sc_callout, hz / 40, &hidkbd_timeout, sc);
}

static void
hidkbd_put_key(struct hidkbd_softc *sc, uint32_t key)
{

	HIDKBD_CTX_LOCK_ASSERT();

	DPRINTF("0x%02x (%d) %s\n", key, key,
	    (key & KEY_RELEASE) ? "released" : "pressed");

#ifdef EVDEV_SUPPORT
	if (evdev_rcpt_mask & EVDEV_RCPT_HW_KBD && sc->sc_evdev != NULL) {
		evdev_push_event(sc->sc_evdev, EV_KEY,
		    evdev_hid2key(KEY_INDEX(key)), !(key & KEY_RELEASE));
		evdev_sync(sc->sc_evdev);
	}
#endif

	if (sc->sc_inputs < HIDKBD_IN_BUF_SIZE) {
		sc->sc_input[sc->sc_inputtail] = key;
		++(sc->sc_inputs);
		++(sc->sc_inputtail);
		if (sc->sc_inputtail >= HIDKBD_IN_BUF_SIZE) {
			sc->sc_inputtail = 0;
		}
	} else {
		DPRINTF("input buffer is full\n");
	}
}

static void
hidkbd_do_poll(struct hidkbd_softc *sc, uint8_t wait)
{

	HIDKBD_CTX_LOCK_ASSERT();
	KASSERT((sc->sc_flags & HIDKBD_FLAG_POLLING) != 0,
	    ("hidkbd_do_poll called when not polling\n"));
	DPRINTFN(2, "polling\n");
#if 0 /* XXX */
	if (!kdb_active && !SCHEDULER_STOPPED()) {
		/*
		 * In this context the kernel is polling for input,
		 * but the USB subsystem works in normal interrupt-driven
		 * mode, so we just wait on the USB threads to do the job.
		 * Note that we currently hold the Giant, but it's also used
		 * as the transfer mtx, so we must release it while waiting.
		 */
		while (sc->sc_inputs == 0) {
			/*
			 * Give USB threads a chance to run.  Note that
			 * kern_yield performs DROP_GIANT + PICKUP_GIANT.
			 */
			lwkt_yield();
			if (!wait)
				break;
		}
		return;
	}
#endif

	while (sc->sc_inputs == 0) {

		HID_INPUT_POLL(device_get_parent(sc->sc_dev));

		/* Delay-optimised support for repetition of keys */
		if (hidkbd_any_key_pressed(sc)) {
			/* a key is pressed - need timekeeping */
			DELAY(1000);

			/* 1 millisecond has passed */
			sc->sc_time_ms += 1;
		}

		hidkbd_interrupt(sc);

		if (!wait)
			break;
	}
}

static int32_t
hidkbd_get_key(struct hidkbd_softc *sc, uint8_t wait)
{
	int32_t c;

	HIDKBD_CTX_LOCK_ASSERT();
#if 0
	KASSERT((!kdb_active && !SCHEDULER_STOPPED())
	    || (sc->sc_flags & HIDKBD_FLAG_POLLING) != 0,
	    ("not polling in kdb or panic\n"));
#endif

	if (sc->sc_inputs == 0 &&
	    (sc->sc_flags & HIDKBD_FLAG_GONE) == 0) {
		/* start transfer, if not already started */
		HID_START_READ(device_get_parent(sc->sc_dev));
	}

	if (sc->sc_flags & HIDKBD_FLAG_POLLING)
		hidkbd_do_poll(sc, wait);

	if (sc->sc_inputs == 0) {
		c = -1;
	} else {
		c = sc->sc_input[sc->sc_inputhead];
		--(sc->sc_inputs);
		++(sc->sc_inputhead);
		if (sc->sc_inputhead >= HIDKBD_IN_BUF_SIZE) {
			sc->sc_inputhead = 0;
		}
	}
	return (c);
}

static void
hidkbd_interrupt(struct hidkbd_softc *sc)
{
	uint32_t n_mod;
	uint32_t o_mod;
	uint32_t now = sc->sc_time_ms;
	uint32_t dtime;
	uint8_t key;
	uint8_t i;
	uint8_t j;

	HIDKBD_CTX_LOCK_ASSERT();

	if (sc->sc_ndata.keycode[0] == KEY_ERROR)
		return;

	n_mod = sc->sc_ndata.modifiers;
	o_mod = sc->sc_odata.modifiers;
	if (n_mod != o_mod) {
		for (i = 0; i < HIDKBD_NMOD; i++) {
			if ((n_mod & hidkbd_mods[i].mask) !=
			    (o_mod & hidkbd_mods[i].mask)) {
				hidkbd_put_key(sc, hidkbd_mods[i].key |
				    ((n_mod & hidkbd_mods[i].mask) ?
				    KEY_PRESS : KEY_RELEASE));
			}
		}
	}
	/* Check for released keys. */
	for (i = 0; i < HIDKBD_NKEYCODE; i++) {
		key = sc->sc_odata.keycode[i];
		if (key == 0) {
			continue;
		}
		for (j = 0; j < HIDKBD_NKEYCODE; j++) {
			if (sc->sc_ndata.keycode[j] == 0) {
				continue;
			}
			if (key == sc->sc_ndata.keycode[j]) {
				goto rfound;
			}
		}
		hidkbd_put_key(sc, key | KEY_RELEASE);
rfound:	;
	}

	/* Check for pressed keys. */
	for (i = 0; i < HIDKBD_NKEYCODE; i++) {
		key = sc->sc_ndata.keycode[i];
		if (key == 0) {
			continue;
		}
		sc->sc_ntime[i] = now + sc->sc_kbd.kb_delay1;
		for (j = 0; j < HIDKBD_NKEYCODE; j++) {
			if (sc->sc_odata.keycode[j] == 0) {
				continue;
			}
			if (key == sc->sc_odata.keycode[j]) {

				/* key is still pressed */

				sc->sc_ntime[i] = sc->sc_otime[j];
				dtime = (sc->sc_otime[j] - now);

				if (!(dtime & 0x80000000)) {
					/* time has not elapsed */
					goto pfound;
				}
				sc->sc_ntime[i] = now + sc->sc_kbd.kb_delay2;
				break;
			}
		}
		hidkbd_put_key(sc, key | KEY_PRESS);

		/*
                 * If any other key is presently down, force its repeat to be
                 * well in the future (100s).  This makes the last key to be
                 * pressed do the autorepeat.
                 */
		for (j = 0; j != HIDKBD_NKEYCODE; j++) {
			if (j != i)
				sc->sc_ntime[j] = now + (100 * 1000);
		}
pfound:	;
	}

	sc->sc_odata = sc->sc_ndata;

	memcpy(sc->sc_otime, sc->sc_ntime, sizeof(sc->sc_otime));

	hidkbd_event_keyinput(sc);
}

static void
hidkbd_event_keyinput(struct hidkbd_softc *sc)
{
	int c;

	HIDKBD_CTX_LOCK_ASSERT();

	if ((sc->sc_flags & HIDKBD_FLAG_POLLING) != 0)
		return;

	if (sc->sc_inputs == 0)
		return;

	if (KBD_IS_ACTIVE(&sc->sc_kbd) &&
	    KBD_IS_BUSY(&sc->sc_kbd)) {
		/* let the callback function process the input */
		(sc->sc_kbd.kb_callback.kc_func) (&sc->sc_kbd, KBDIO_KEYINPUT,
		    sc->sc_kbd.kb_callback.kc_arg);
	} else {
		/* read and discard the input, no one is waiting for it */
		do {
			c = hidkbd_read_char(&sc->sc_kbd, 0);
		} while (c != NOKEY);
	}
}

static void
hidkbd_timeout(void *arg)
{
	struct hidkbd_softc *sc = arg;

	HIDKBD_LOCK_ASSERT();

	sc->sc_time_ms += 25;	/* milliseconds */

	hidkbd_interrupt(sc);

	/* Make sure any leftover key events gets read out */
	hidkbd_event_keyinput(sc);

	if (hidkbd_any_key_pressed(sc) || (sc->sc_inputs != 0)) {
		hidkbd_start_timer(sc);
	} else {
		sc->sc_flags &= ~HIDKBD_FLAG_TIMER_RUNNING;
	}
}

static uint8_t
hidkbd_apple_fn(uint8_t keycode) {
	switch (keycode) {
	case 0x28: return 0x49; /* RETURN -> INSERT */
	case 0x2a: return 0x4c; /* BACKSPACE -> DEL */
	case 0x50: return 0x4a; /* LEFT ARROW -> HOME */
	case 0x4f: return 0x4d; /* RIGHT ARROW -> END */
	case 0x52: return 0x4b; /* UP ARROW -> PGUP */
	case 0x51: return 0x4e; /* DOWN ARROW -> PGDN */
	default: return keycode;
	}
}

static uint8_t
hidkbd_apple_swap(uint8_t keycode) {
	switch (keycode) {
	case 0x35: return 0x64;
	case 0x64: return 0x35;
	default: return keycode;
	}
}

static void
hidkbd_input_handler(uint8_t id, uint8_t *buf, int len, void *arg)
{
	struct hidkbd_softc *sc = arg;
	uint8_t i;

	DPRINTF("actlen=%d bytes\n", len);

	if (len == 0) {
		DPRINTF("zero length data\n");
		return;
	}

	HIDKBD_LOCK(sc);
	/* scan through HID data */
	if ((sc->sc_flags & HIDKBD_FLAG_APPLE_EJECT) &&
	    (id == sc->sc_id_apple_eject)) {
		if (hid_get_data(buf, len, &sc->sc_loc_apple_eject))
			sc->sc_modifiers |= MOD_EJECT;
		else
			sc->sc_modifiers &= ~MOD_EJECT;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_APPLE_FN) &&
	    (id == sc->sc_id_apple_fn)) {
		if (hid_get_data(buf, len, &sc->sc_loc_apple_fn))
			sc->sc_modifiers |= MOD_FN;
		else
			sc->sc_modifiers &= ~MOD_FN;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_CTRL_L) &&
	    (id == sc->sc_id_ctrl_l)) {
		if (hid_get_data(buf, len, &sc->sc_loc_ctrl_l))
		  sc->	sc_modifiers |= MOD_CONTROL_L;
		else
		  sc->	sc_modifiers &= ~MOD_CONTROL_L;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_CTRL_R) &&
	    (id == sc->sc_id_ctrl_r)) {
		if (hid_get_data(buf, len, &sc->sc_loc_ctrl_r))
			sc->sc_modifiers |= MOD_CONTROL_R;
		else
			sc->sc_modifiers &= ~MOD_CONTROL_R;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_SHIFT_L) &&
	    (id == sc->sc_id_shift_l)) {
		if (hid_get_data(buf, len, &sc->sc_loc_shift_l))
			sc->sc_modifiers |= MOD_SHIFT_L;
		else
			sc->sc_modifiers &= ~MOD_SHIFT_L;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_SHIFT_R) &&
	    (id == sc->sc_id_shift_r)) {
		if (hid_get_data(buf, len, &sc->sc_loc_shift_r))
			sc->sc_modifiers |= MOD_SHIFT_R;
		else
			sc->sc_modifiers &= ~MOD_SHIFT_R;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_ALT_L) &&
	    (id == sc->sc_id_alt_l)) {
		if (hid_get_data(buf, len, &sc->sc_loc_alt_l))
			sc->sc_modifiers |= MOD_ALT_L;
		else
			sc->sc_modifiers &= ~MOD_ALT_L;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_ALT_R) &&
	    (id == sc->sc_id_alt_r)) {
		if (hid_get_data(buf, len, &sc->sc_loc_alt_r))
			sc->sc_modifiers |= MOD_ALT_R;
		else
			sc->sc_modifiers &= ~MOD_ALT_R;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_WIN_L) &&
	    (id == sc->sc_id_win_l)) {
		if (hid_get_data(buf, len, &sc->sc_loc_win_l))
			sc->sc_modifiers |= MOD_WIN_L;
		else
			sc->sc_modifiers &= ~MOD_WIN_L;
	}
	if ((sc->sc_flags & HIDKBD_FLAG_WIN_R) &&
	    (id == sc->sc_id_win_r)) {
		if (hid_get_data(buf, len, &sc->sc_loc_win_r))
			sc->sc_modifiers |= MOD_WIN_R;
		else
			sc->sc_modifiers &= ~MOD_WIN_R;
	}

	sc->sc_ndata.modifiers = sc->sc_modifiers;

	if ((sc->sc_flags & HIDKBD_FLAG_EVENTS) &&
	    (id == sc->sc_id_events)) {
		for (i = 0; i < sc->sc_loc_events.count; i++) {
			if (i < HIDKBD_NKEYCODE) {
				sc->sc_ndata.keycode[i] = hid_get_arraydata(
				    buf, len, &sc->sc_loc_events, i);
			}
		}
	}

#ifdef USB_DEBUG
	DPRINTF("modifiers = 0x%04x\n", (int)sc->sc_modifiers);
	for (i = 0; i < HIDKBD_NKEYCODE; i++) {
		if (sc->sc_ndata.keycode[i]) {
			DPRINTF("[%d] = 0x%02x\n",
			    (int)i, (int)sc->sc_ndata.keycode[i]);
		}
	}
#endif
	if (sc->sc_modifiers & MOD_FN) {
		for (i = 0; i < HIDKBD_NKEYCODE; i++) {
			sc->sc_ndata.keycode[i] =
			    hidkbd_apple_fn(sc->sc_ndata.keycode[i]);
		}
	}

	if (sc->sc_flags & HIDKBD_FLAG_APPLE_SWAP) {
		for (i = 0; i < HIDKBD_NKEYCODE; i++) {
			sc->sc_ndata.keycode[i] =
			    hidkbd_apple_swap(sc->sc_ndata.keycode[i]);
		}
	}

	hidkbd_interrupt(sc);

	if (!(sc->sc_flags & HIDKBD_FLAG_TIMER_RUNNING)) {
		if (hidkbd_any_key_pressed(sc)) {
			hidkbd_start_timer(sc);
		}
	}
	HIDKBD_UNLOCK(sc);
}

static void
hidkbd_output_handler(uint8_t *buf, int len, void *arg)
{
	struct hidkbd_softc *sc = arg;

	HIDKBD_LOCK(sc);
	if (sc->sc_outbuf[0] == NULL)
		sc->sc_outbuf[0] = buf;
	else if (sc->sc_outbuf[1] == NULL)
		sc->sc_outbuf[1] = buf;
	HIDKBD_UNLOCK(sc);
}

static int
hidkbd_probe(device_t dev)
{
	keyboard_switch_t *sw = kbd_get_switch(HIDKBD_DRIVER_NAME);
	char *d_ptr;
	int error;
	uint16_t d_len;

	HIDKBD_LOCK_ASSERT();
	DPRINTFN(11, "\n");

	if (sw == NULL) {
		return (ENXIO);
	}

#if 0
	if (usb_test_quirk(uaa, UQ_KBD_IGNORE))
		return (ENXIO);
#endif

	if (hid_get_bootproto(dev) == HID_BOOTPROTO_KEYBOARD) {
		device_set_desc(dev, "HID Keyboard device");
		return (BUS_PROBE_DEFAULT);
	}

	HID_GET_DESCRIPTOR(device_get_parent(dev), &d_ptr, &d_len);

	if (hid_is_keyboard(d_ptr, d_len)) {
		if (hid_is_mouse(d_ptr, d_len)) {
			/*
			 * NOTE: We currently don't support USB mouse
			 * and USB keyboard on the same USB endpoint.
			 * Let "ums" driver win.
			 */
			error = ENXIO;
		} else {
			device_set_desc(dev, "HID Keyboard device");
			error = BUS_PROBE_DEFAULT;
		}
	} else {
		error = ENXIO;
	}
	return (error);
}

static void
hidkbd_parse_hid(struct hidkbd_softc *sc, const uint8_t *ptr, uint32_t len)
{
	uint32_t flags;
	uint8_t outid;
	int any = 0;

	/* reset detected bits */
	sc->sc_flags &= ~HIDKBD_FLAG_HID_MASK;

	/* check if there is an ID byte */
	sc->sc_kbd_size = hid_report_size_a(ptr, len,
	    hid_input, &sc->sc_kbd_id);

	/* investigate if this is an Apple Keyboard */
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_CONSUMER, HUG_APPLE_EJECT),
	    hid_input, 0, &sc->sc_loc_apple_eject, &flags,
	    &sc->sc_id_apple_eject)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_APPLE_EJECT |
			    HIDKBD_FLAG_APPLE_SWAP;
		DPRINTFN(1, "Found Apple eject-key\n");
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(0xFFFF, 0x0003),
	    hid_input, 0, &sc->sc_loc_apple_fn, &flags,
	    &sc->sc_id_apple_fn)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_APPLE_FN;
		DPRINTFN(1, "Found Apple FN-key\n");
	}
	/* figure out some keys */
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0xE0),
	    hid_input, 0, &sc->sc_loc_ctrl_l, &flags,
	    &sc->sc_id_ctrl_l)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_CTRL_L;
		DPRINTFN(1, "Found left control\n");
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0xE4),
	    hid_input, 0, &sc->sc_loc_ctrl_r, &flags,
	    &sc->sc_id_ctrl_r)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_CTRL_R;
		DPRINTFN(1, "Found right control\n");
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0xE1),
	    hid_input, 0, &sc->sc_loc_shift_l, &flags,
	    &sc->sc_id_shift_l)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_SHIFT_L;
		DPRINTFN(1, "Found left shift\n");
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0xE5),
	    hid_input, 0, &sc->sc_loc_shift_r, &flags,
	    &sc->sc_id_shift_r)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_SHIFT_R;
		DPRINTFN(1, "Found right shift\n");
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0xE2),
	    hid_input, 0, &sc->sc_loc_alt_l, &flags,
	    &sc->sc_id_alt_l)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_ALT_L;
		DPRINTFN(1, "Found left alt\n");
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0xE6),
	    hid_input, 0, &sc->sc_loc_alt_r, &flags,
	    &sc->sc_id_alt_r)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_ALT_R;
		DPRINTFN(1, "Found right alt\n");
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0xE3),
	    hid_input, 0, &sc->sc_loc_win_l, &flags,
	    &sc->sc_id_win_l)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_WIN_L;
		DPRINTFN(1, "Found left GUI\n");
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0xE7),
	    hid_input, 0, &sc->sc_loc_win_r, &flags,
	    &sc->sc_id_win_r)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_WIN_R;
		DPRINTFN(1, "Found right GUI\n");
	}
	/* figure out event buffer */
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_KEYBOARD, 0x00),
	    hid_input, 0, &sc->sc_loc_events, &flags,
	    &sc->sc_id_events)) {
		if (flags & HIO_VARIABLE) {
			DPRINTFN(1, "Ignoring keyboard event control\n");
		} else {
			sc->sc_flags |= HIDKBD_FLAG_EVENTS;
			DPRINTFN(1, "Found keyboard event array\n");
		}
	}

	/* figure out leds on keyboard */
	/* Assumption: All led bits must be in the same ID. */
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_LEDS, 0x01),
	    hid_output, 0, &sc->sc_loc_numlock, &flags,
	    &sc->sc_id_numlock)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_NUMLOCK;
		DPRINTFN(1, "Found keyboard numlock\n");
		if (any == 0) {
			outid = sc->sc_id_numlock;
			any = 1;
		}
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_LEDS, 0x02),
	    hid_output, 0, &sc->sc_loc_capslock, &flags,
	    &sc->sc_id_capslock)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_CAPSLOCK;
		DPRINTFN(1, "Found keyboard capslock\n");
		if (any == 0) {
			outid = sc->sc_id_capslock;
			any = 1;
		} else if (outid != sc->sc_id_capslock) {
			DPRINTFN(1, "Capslock on different id %d\n",
			    sc->sc_id_capslock);
			sc->sc_flags &= ~HIDKBD_FLAG_CAPSLOCK;
		}
	}
	if (hid_locate(ptr, len,
	    HID_USAGE2(HUP_LEDS, 0x03),
	    hid_output, 0, &sc->sc_loc_scrolllock, &flags,
	    &sc->sc_id_scrolllock)) {
		if (flags & HIO_VARIABLE)
			sc->sc_flags |= HIDKBD_FLAG_SCROLLLOCK;
		DPRINTFN(1, "Found keyboard scrolllock\n");
		if (any == 0) {
			outid = sc->sc_id_scrolllock;
			any = 1;
		} else if (outid != sc->sc_id_capslock) {
			DPRINTFN(1, "Scrolllock on different id %d\n",
			    sc->sc_id_scrolllock);
			sc->sc_flags &= ~HIDKBD_FLAG_SCROLLLOCK;
		}
	}

	if (any != 0) {
		sc->sc_led_size = hid_report_size(ptr, len,
		    hid_output, outid);
		sc->sc_led_id = outid;
	} else {
		sc->sc_led_size = 0;
		sc->sc_led_id = 0;
	}
}

static int
hidkbd_attach(device_t dev)
{
	struct hidkbd_softc *sc = device_get_softc(dev);
	int32_t unit = device_get_unit(dev);
	keyboard_t *kbd = &sc->sc_kbd;
	char *hid_ptr = NULL;
	uint16_t n;
	uint16_t hid_len;
#ifdef EVDEV_SUPPORT
	struct evdev_dev *evdev;
	int i;
#endif
#if 0
#ifdef USB_DEBUG
	int rate;
#endif
#endif

	HIDKBD_LOCK_ASSERT();

	kbd_init_struct(kbd, HIDKBD_DRIVER_NAME, KB_OTHER,
            unit, 0, KB_PRI_USB, 0, 0);

	kbd->kb_data = (void *)sc;

	sc->sc_dev = dev;
	sc->sc_mode = K_XLATE;

	callout_init_lk(&sc->sc_callout, &kbd->kb_lock);

	/* setup default keyboard maps */

	sc->sc_keymap = key_map;
	sc->sc_accmap = accent_map;
	for (n = 0; n < HIDKBD_NFKEY; n++) {
		sc->sc_fkeymap[n] = fkey_tab[n];
	}

	kbd_set_maps(kbd, &sc->sc_keymap, &sc->sc_accmap,
	    sc->sc_fkeymap, HIDKBD_NFKEY);

	KBD_FOUND_DEVICE(kbd);

	hidkbd_clear_state(kbd);

	/*
	 * FIXME: set the initial value for lock keys in "sc_state"
	 * according to the BIOS data?
	 */
	KBD_PROBE_DONE(kbd);

	/* get HID descriptor */
	HID_GET_DESCRIPTOR(device_get_parent(dev), &hid_ptr, &hid_len);

	if (hid_get_bootproto(dev) == HID_BOOTPROTO_KEYBOARD) {
		DPRINTF("Using boot protocol\n");

		hidkbd_parse_hid(sc, hidkbd_boot_desc, sizeof(hidkbd_boot_desc));
	} else {
		DPRINTF("Parsing HID descriptor of %d bytes\n",
		    (int)hid_len);

		hidkbd_parse_hid(sc, hid_ptr, hid_len);
	}

#if 0
	/* check if we should use the boot protocol */
	if (usb_test_quirk(uaa, UQ_KBD_BOOTPROTO) ||
	    (err != 0) || (!(sc->sc_flags & HIDKBD_FLAG_EVENTS))) {

		DPRINTF("Forcing boot protocol\n");

		err = usbd_req_set_protocol(sc->sc_udev, NULL,
			sc->sc_iface_index, 0);

		if (err != 0) {
			DPRINTF("Set protocol error=%s (ignored)\n",
			    usbd_errstr(err));
		}

		hidkbd_parse_hid(sc, hidkbd_boot_desc, sizeof(hidkbd_boot_desc));
	}
#endif

	if (sc->sc_led_size > 0) {
		/*
		 * The current buffering in the HID bus, requires us to have
		 * one buffer in reserve.
		 */
		sc->sc_outbuf[0] = kmalloc(sc->sc_led_size, M_DEVBUF,
		    M_WAITOK | M_ZERO);
		sc->sc_outbuf[1] = kmalloc(sc->sc_led_size, M_DEVBUF,
		    M_WAITOK | M_ZERO);
	} else {
		sc->sc_outbuf[0] = NULL;
		sc->sc_outbuf[1] = NULL;
	}

	/* hidkbd_input_handler will only be called after HID_START_READ. */
	HID_SET_HANDLER(device_get_parent(dev), hidkbd_input_handler,
	    hidkbd_output_handler, sc);

	/* ignore if SETIDLE fails, hence it is not crucial */
	HID_SETIDLE(device_get_parent(dev), 0, 0);

	hidkbd_ioctl(kbd, KDSETLED, (caddr_t)&sc->sc_state);

	KBD_INIT_DONE(kbd);

	if (kbd_register(kbd) < 0) {
		goto detach;
	}
	KBD_CONFIG_DONE(kbd);

	hidkbd_enable(kbd);

#ifdef KBD_INSTALL_CDEV
	if (kbd_attach(kbd)) {
		goto detach;
	}
#endif

#ifdef EVDEV_SUPPORT
	evdev = evdev_alloc();
	evdev_set_name(evdev, device_get_desc(dev));
	evdev_set_phys(evdev, device_get_nameunit(dev));
	evdev_set_id(evdev, hid_bustype_to_evdev(hid_get_bustype(dev)),
	    hid_get_vendor(dev), hid_get_product(dev), 0);
	evdev_set_serial(evdev, hid_get_serial(dev));
	evdev_set_methods(evdev, kbd, &hidkbd_evdev_methods);
	evdev_support_event(evdev, EV_SYN);
	evdev_support_event(evdev, EV_KEY);
	if (sc->sc_flags & (HIDKBD_FLAG_NUMLOCK | HIDKBD_FLAG_CAPSLOCK |
			    HIDKBD_FLAG_SCROLLLOCK))
		evdev_support_event(evdev, EV_LED);
	evdev_support_event(evdev, EV_REP);

	for (i = 0x00; i <= 0xFF; i++)
		evdev_support_key(evdev, evdev_hid2key(i));
	if (sc->sc_flags & HIDKBD_FLAG_NUMLOCK)
		evdev_support_led(evdev, LED_NUML);
	if (sc->sc_flags & HIDKBD_FLAG_CAPSLOCK)
		evdev_support_led(evdev, LED_CAPSL);
	if (sc->sc_flags & HIDKBD_FLAG_SCROLLLOCK)
		evdev_support_led(evdev, LED_SCROLLL);

	if (evdev_register(evdev))
		evdev_free(evdev);
	else
		sc->sc_evdev = evdev;
#endif

	sc->sc_flags |= HIDKBD_FLAG_ATTACHED;

	if (bootverbose) {
		genkbd_diag(kbd, bootverbose);
	}

#if 0
#ifdef USB_DEBUG
	/* check for polling rate override */
	rate = ukbd_pollrate;
	if (rate > 0) {
		if (rate > 1000)
			rate = 1;
		else
			rate = 1000 / rate;

		/* set new polling interval in ms */
		usbd_xfer_set_interval(sc->sc_xfer[UKBD_INTR_DT], rate);
	}
#endif
#endif
	/* start the keyboard */
	HID_START_READ(device_get_parent(dev));

	return (0);			/* success */
detach:
	hidkbd_detach(dev);
	return (ENXIO);			/* error */
}

static int
hidkbd_detach(device_t dev)
{
	struct hidkbd_softc *sc = device_get_softc(dev);
	int error;

	HIDKBD_LOCK_ASSERT();

	DPRINTF("\n");

	HID_STOP_READ(device_get_parent(dev));
	HID_SET_HANDLER(device_get_parent(dev), NULL, NULL, NULL);

	crit_enter();
	sc->sc_flags |= HIDKBD_FLAG_GONE;

	callout_stop(&sc->sc_callout);

	/* kill any stuck keys */
	if (sc->sc_flags & HIDKBD_FLAG_ATTACHED) {
		/* release all leftover keys, if any */
		memset(&sc->sc_ndata, 0, sizeof(sc->sc_ndata));

		/* process releasing of all keys */
		hidkbd_interrupt(sc);
	}

	hidkbd_disable(&sc->sc_kbd);

	callout_drain(&sc->sc_callout);

#ifdef KBD_INSTALL_CDEV
	if (sc->sc_flags & HIDKBD_FLAG_ATTACHED) {
		error = kbd_detach(&sc->sc_kbd);
		if (error) {
			/* usb attach cannot return an error */
			device_printf(dev, "WARNING: kbd_detach() "
			    "returned non-zero! (ignored)\n");
		}
	}
#endif

#ifdef EVDEV_SUPPORT
	evdev_free(sc->sc_evdev);
#endif

	if (KBD_IS_CONFIGURED(&sc->sc_kbd)) {
		/*
		 * kbd_unregister requires kb_lock to be held
		 * but lockuninits it then
		 */
		HIDKBD_LOCK(sc);
		error = kbd_unregister(&sc->sc_kbd);
		if (error) {
			/* usb attach cannot return an error */
			device_printf(dev, "WARNING: kbd_unregister() "
			    "returned non-zero! (ignored)\n");
		}
	}
	sc->sc_kbd.kb_flags = 0;

	crit_exit();

	DPRINTF("%s: disconnected\n",
	    device_get_nameunit(dev));

	if (sc->sc_outbuf[0] != NULL)
		kfree(sc->sc_outbuf[0], M_DEVBUF);
	if (sc->sc_outbuf[1] != NULL)
		kfree(sc->sc_outbuf[1], M_DEVBUF);

	return (0);
}

static int
hidkbd_resume(device_t dev)
{
	struct hidkbd_softc *sc = device_get_softc(dev);

	HIDKBD_LOCK_ASSERT();

	hidkbd_clear_state(&sc->sc_kbd);

	return (0);
}

/* early keyboard probe, not supported */
static int
hidkbd_configure(int flags)
{
	return (0);
}

/* detect a keyboard, not used */
static int
hidkbd__probe(int unit, void *arg, int flags)
{
	return (ENXIO);
}

/* reset and initialize the device, not used */
static int
hidkbd_init(int unit, keyboard_t **kbdp, void *arg, int flags)
{
	return (ENXIO);
}

/* test the interface to the device, not used */
static int
hidkbd_test_if(keyboard_t *kbd)
{
	return (0);
}

/* finish using this keyboard, not used */
static int
hidkbd_term(keyboard_t *kbd)
{
	return (ENXIO);
}

/* keyboard interrupt routine, not used */
static int
hidkbd_intr(keyboard_t *kbd, void *arg)
{
	return (0);
}

/* lock the access to the keyboard, not used */
static int
hidkbd_lock(keyboard_t *kbd, int lock)
{
	return (1);
}

/*
 * Enable the access to the device; until this function is called,
 * the client cannot read from the keyboard.
 */
static int
hidkbd_enable(keyboard_t *kbd)
{
	crit_enter();
	KBD_ACTIVATE(kbd);
	crit_exit();

	return (0);
}

/* disallow the access to the device */
static int
hidkbd_disable(keyboard_t *kbd)
{
	crit_enter();
	KBD_DEACTIVATE(kbd);
	crit_exit();

	return (0);
}

/* check if data is waiting */
/* Currently unused. */
static int
hidkbd_check(keyboard_t *kbd)
{
	struct hidkbd_softc *sc = kbd->kb_data;

	HIDKBD_CTX_LOCK_ASSERT();

	if (!KBD_IS_ACTIVE(kbd))
		return (0);

	if (sc->sc_flags & HIDKBD_FLAG_POLLING)
		hidkbd_do_poll(sc, 0);

#ifdef HIDKBD_EMULATE_ATSCANCODE
	if (sc->sc_buffered_char[0]) {
		return (1);
	}
#endif
	if (sc->sc_inputs > 0) {
		return (1);
	}
	return (0);
}

/* check if char is waiting */
static int
hidkbd_check_char_locked(keyboard_t *kbd)
{
	struct hidkbd_softc *sc = kbd->kb_data;

	HIDKBD_CTX_LOCK_ASSERT();

	if (!KBD_IS_ACTIVE(kbd))
		return (0);

	if ((sc->sc_composed_char > 0) &&
	    (!(sc->sc_flags & HIDKBD_FLAG_COMPOSE))) {
		return (1);
	}
	return (hidkbd_check(kbd));
}

static int
hidkbd_check_char(keyboard_t *kbd)
{
	int result;
#if 0
	struct hidkbd_softc *sc = kbd->kb_data;

	HIDKBD_LOCK(sc);
#endif
	result = hidkbd_check_char_locked(kbd);
#if 0
	HIDKBD_UNLOCK(sc);
#endif

	return (result);
}

/* read one byte from the keyboard if it's allowed */
/* Currently unused. */
static int
hidkbd_read(keyboard_t *kbd, int wait)
{
	struct hidkbd_softc *sc = kbd->kb_data;
	int32_t hidcode;
#ifdef HIDKBD_EMULATE_ATSCANCODE
	uint32_t keycode;
	uint32_t scancode;

#endif

	HIDKBD_CTX_LOCK_ASSERT();

	if (!KBD_IS_ACTIVE(kbd))
		return (-1);

#ifdef HIDKBD_EMULATE_ATSCANCODE
	if (sc->sc_buffered_char[0]) {
		scancode = sc->sc_buffered_char[0];
		if (scancode & SCAN_PREFIX) {
			sc->sc_buffered_char[0] &= ~SCAN_PREFIX;
			return ((scancode & SCAN_PREFIX_E0) ? 0xe0 : 0xe1);
		}
		sc->sc_buffered_char[0] = sc->sc_buffered_char[1];
		sc->sc_buffered_char[1] = 0;
		return (scancode);
	}
#endif					/* HIDKBD_EMULATE_ATSCANCODE */

	/* XXX */
	hidcode = hidkbd_get_key(sc, (wait == FALSE) ? 0 : 1);
	if (!KBD_IS_ACTIVE(kbd) || (hidcode == -1))
		return (-1);

	++(kbd->kb_count);

#ifdef HIDKBD_EMULATE_ATSCANCODE
	keycode = hidkbd_trtab[KEY_INDEX(hidcode)];
	if (keycode == NN) {
		return -1;
	}
	return (hidkbd_key2scan(sc, keycode, sc->sc_ndata.modifiers,
	    (hidcode & KEY_RELEASE)));
#else					/* !HIDKBD_EMULATE_ATSCANCODE */
	return (hidcode);
#endif					/* HIDKBD_EMULATE_ATSCANCODE */
}

/* read char from the keyboard */
static uint32_t
hidkbd_read_char_locked(keyboard_t *kbd, int wait)
{
	struct hidkbd_softc *sc = kbd->kb_data;
	uint32_t action;
	uint32_t keycode;
	int32_t hidcode;
#ifdef HIDKBD_EMULATE_ATSCANCODE
	uint32_t scancode;
#endif

	HIDKBD_CTX_LOCK_ASSERT();

	if (!KBD_IS_ACTIVE(kbd))
		return (NOKEY);

next_code:

	/* do we have a composed char to return ? */

	if ((sc->sc_composed_char > 0) &&
	    (!(sc->sc_flags & HIDKBD_FLAG_COMPOSE))) {

		action = sc->sc_composed_char;
		sc->sc_composed_char = 0;

		if (action > 0xFF) {
			goto errkey;
		}
		goto done;
	}
#ifdef HIDKBD_EMULATE_ATSCANCODE

	/* do we have a pending raw scan code? */

	if (sc->sc_mode == K_RAW) {
		scancode = sc->sc_buffered_char[0];
		if (scancode) {
			if (scancode & SCAN_PREFIX) {
				sc->sc_buffered_char[0] = (scancode & ~SCAN_PREFIX);
				return ((scancode & SCAN_PREFIX_E0) ? 0xe0 : 0xe1);
			}
			sc->sc_buffered_char[0] = sc->sc_buffered_char[1];
			sc->sc_buffered_char[1] = 0;
			return (scancode);
		}
	}
#endif					/* HIDKBD_EMULATE_ATSCANCODE */

	/* see if there is something in the keyboard port */
	/* XXX */
	hidcode = hidkbd_get_key(sc, (wait == FALSE) ? 0 : 1);
	if (hidcode == -1) {
		return (NOKEY);
	}
	++kbd->kb_count;

#ifdef HIDKBD_EMULATE_ATSCANCODE
	/* USB key index -> key code -> AT scan code */
	keycode = hidkbd_trtab[KEY_INDEX(hidcode)];
	if (keycode == NN) {
		return (NOKEY);
	}
	/* return an AT scan code for the K_RAW mode */
	if (sc->sc_mode == K_RAW) {
		return (hidkbd_key2scan(sc, keycode, sc->sc_ndata.modifiers,
		    (hidcode & KEY_RELEASE)));
	}
#else					/* !HIDKBD_EMULATE_ATSCANCODE */

	/* return the byte as is for the K_RAW mode */
	if (sc->sc_mode == K_RAW) {
		return (hidcode);
	}
	/* USB key index -> key code */
	keycode = hidkbd_trtab[KEY_INDEX(hidcode)];
	if (keycode == NN) {
		return (NOKEY);
	}
#endif					/* HIDKBD_EMULATE_ATSCANCODE */

	switch (keycode) {
	case 0x38:			/* left alt (compose key) */
		if (hidcode & KEY_RELEASE) {
			if (sc->sc_flags & HIDKBD_FLAG_COMPOSE) {
				sc->sc_flags &= ~HIDKBD_FLAG_COMPOSE;

				if (sc->sc_composed_char > 0xFF) {
					sc->sc_composed_char = 0;
				}
			}
		} else {
			if (!(sc->sc_flags & HIDKBD_FLAG_COMPOSE)) {
				sc->sc_flags |= HIDKBD_FLAG_COMPOSE;
				sc->sc_composed_char = 0;
			}
		}
		break;
		/* XXX: I don't like these... */
	case 0x5c:			/* print screen */
		if (sc->sc_flags & ALTS) {
			keycode = 0x54;	/* sysrq */
		}
		break;
	case 0x68:			/* pause/break */
		if (sc->sc_flags & CTLS) {
			keycode = 0x6c;	/* break */
		}
		break;
	}

	/* return the key code in the K_CODE mode */
	if (hidcode & KEY_RELEASE) {
		keycode |= SCAN_RELEASE;
	}
	if (sc->sc_mode == K_CODE) {
		return (keycode);
	}
	/* compose a character code */
	if (sc->sc_flags & HIDKBD_FLAG_COMPOSE) {
		switch (keycode) {
			/* key pressed, process it */
		case 0x47:
		case 0x48:
		case 0x49:		/* keypad 7,8,9 */
			sc->sc_composed_char *= 10;
			sc->sc_composed_char += keycode - 0x40;
			goto check_composed;

		case 0x4B:
		case 0x4C:
		case 0x4D:		/* keypad 4,5,6 */
			sc->sc_composed_char *= 10;
			sc->sc_composed_char += keycode - 0x47;
			goto check_composed;

		case 0x4F:
		case 0x50:
		case 0x51:		/* keypad 1,2,3 */
			sc->sc_composed_char *= 10;
			sc->sc_composed_char += keycode - 0x4E;
			goto check_composed;

		case 0x52:		/* keypad 0 */
			sc->sc_composed_char *= 10;
			goto check_composed;

			/* key released, no interest here */
		case SCAN_RELEASE | 0x47:
		case SCAN_RELEASE | 0x48:
		case SCAN_RELEASE | 0x49:	/* keypad 7,8,9 */
		case SCAN_RELEASE | 0x4B:
		case SCAN_RELEASE | 0x4C:
		case SCAN_RELEASE | 0x4D:	/* keypad 4,5,6 */
		case SCAN_RELEASE | 0x4F:
		case SCAN_RELEASE | 0x50:
		case SCAN_RELEASE | 0x51:	/* keypad 1,2,3 */
		case SCAN_RELEASE | 0x52:	/* keypad 0 */
			goto next_code;

		case 0x38:		/* left alt key */
			break;

		default:
			if (sc->sc_composed_char > 0) {
				sc->sc_flags &= ~HIDKBD_FLAG_COMPOSE;
				sc->sc_composed_char = 0;
				goto errkey;
			}
			break;
		}
	}
	/* keycode to key action */
	action = genkbd_keyaction(kbd, SCAN_CHAR(keycode),
	    (keycode & SCAN_RELEASE),
	    &sc->sc_state, &sc->sc_accents);
	if (action == NOKEY) {
		goto next_code;
	}
done:
	return (action);

check_composed:
	if (sc->sc_composed_char <= 0xFF) {
		goto next_code;
	}
errkey:
	return (ERRKEY);
}

/* Currently wait is always false. */
static uint32_t
hidkbd_read_char(keyboard_t *kbd, int wait)
{
	uint32_t keycode;
#if 0
	struct hidkbd_softc *sc = kbd->kb_data;

	HIDKBD_LOCK(sc);
#endif
	keycode = hidkbd_read_char_locked(kbd, wait);
#if 0
	HIDKBD_UNLOCK(sc);
#endif

	return (keycode);
}

/* some useful control functions */
static int
hidkbd_ioctl_locked(keyboard_t *kbd, u_long cmd, caddr_t arg)
{
	struct hidkbd_softc *sc = kbd->kb_data;
	int i;

	switch (cmd) {
	case KDGKBMODE:		/* get keyboard mode */
		*(int *)arg = sc->sc_mode;
		break;
	case KDSKBMODE:		/* set keyboard mode */
		switch (*(int *)arg) {
		case K_XLATE:
			if (sc->sc_mode != K_XLATE) {
				/* make lock key state and LED state match */
				sc->sc_state &= ~LOCK_MASK;
				sc->sc_state |= KBD_LED_VAL(kbd);
			}
			/* FALLTHROUGH */
		case K_RAW:
		case K_CODE:
			if (sc->sc_mode != *(int *)arg) {
				if ((sc->sc_flags & HIDKBD_FLAG_POLLING) == 0)
					hidkbd_clear_state(kbd);
				sc->sc_mode = *(int *)arg;
			}
			break;
		default:
			return (EINVAL);
		}
		break;

	case KDGETLED:			/* get keyboard LED */
		*(int *)arg = KBD_LED_VAL(kbd);
		break;
	case KDSETLED:			/* set keyboard LED */
		/* NOTE: lock key state in "sc_state" won't be changed */
		if (*(int *)arg & ~LOCK_MASK)
			return (EINVAL);

		i = *(int *)arg;

		/* replace CAPS LED with ALTGR LED for ALTGR keyboards */
		if (sc->sc_mode == K_XLATE &&
		    kbd->kb_keymap->n_keys > ALTGR_OFFSET) {
			if (i & ALKED)
				i |= CLKED;
			else
				i &= ~CLKED;
		}
		if (KBD_HAS_DEVICE(kbd))
			hidkbd_set_leds(sc, i);

		KBD_LED_VAL(kbd) = *(int *)arg;
		break;
	case KDGKBSTATE:		/* get lock key state */
		*(int *)arg = sc->sc_state & LOCK_MASK;
		break;
	case KDSKBSTATE:		/* set lock key state */
		if (*(int *)arg & ~LOCK_MASK) {
			return (EINVAL);
		}
		sc->sc_state &= ~LOCK_MASK;
		sc->sc_state |= *(int *)arg;

		/* set LEDs and quit */
		return (hidkbd_ioctl(kbd, KDSETLED, arg));

	case KDSETREPEAT:		/* set keyboard repeat rate (new
					 * interface) */
		if (!KBD_HAS_DEVICE(kbd)) {
			return (0);
		}
		if (((int *)arg)[1] < 0) {
			return (EINVAL);
		}
		if (((int *)arg)[0] < 0) {
			return (EINVAL);
		}
		if (((int *)arg)[0] < 200)	/* fastest possible value */
			kbd->kb_delay1 = 200;
		else
			kbd->kb_delay1 = ((int *)arg)[0];
		kbd->kb_delay2 = ((int *)arg)[1];
#ifdef EVDEV_SUPPORT
		if (sc->sc_evdev != NULL)
			evdev_push_repeats(sc->sc_evdev, kbd);
#endif
		return (0);

	case PIO_KEYMAP:		/* set keyboard translation table */
	case PIO_KEYMAPENT:		/* set keyboard translation table
					 * entry */
	case PIO_DEADKEYMAP:		/* set accent key translation table */
		sc->sc_accents = 0;
		/* FALLTHROUGH */
	default:
		return (genkbd_commonioctl(kbd, cmd, arg));
	}

	return (0);
}

static int
hidkbd_ioctl(keyboard_t *kbd, u_long cmd, caddr_t arg)
{
	int result;
	struct hidkbd_softc *sc = kbd->kb_data;

	/*
	 * XXX KDGKBSTATE, KDSKBSTATE and KDSETLED can be called from any
	 * context where printf(9) can be called, which among other things
	 * includes interrupt filters and threads with any kinds of locks
	 * already held.  For this reason it would be dangerous to acquire
	 * the Giant here unconditionally.  On the other hand we have to
	 * have it to handle the ioctl.
	 * So we make our best effort to auto-detect whether we can grab
	 * the Giant or not.  Blame syscons(4) for this.
	 */
	switch (cmd) {
	case KDGKBSTATE:
	case KDSKBSTATE:
	case KDSETLED:
		if(!lockowned(&kbd->kb_lock)) {
			return (EDEADLK);	/* best I could come up with */
		}
		/* FALLTHROUGH */
	default:
		HIDKBD_LOCK(sc);
		result = hidkbd_ioctl_locked(kbd, cmd, arg);
		HIDKBD_UNLOCK(sc);
		return (result);
	}
}


/* clear the internal state of the keyboard */
static void
hidkbd_clear_state(keyboard_t *kbd)
{
	struct hidkbd_softc *sc = kbd->kb_data;

	HIDKBD_CTX_LOCK_ASSERT();

	sc->sc_flags &= ~(HIDKBD_FLAG_COMPOSE | HIDKBD_FLAG_POLLING);
	sc->sc_state &= LOCK_MASK;	/* preserve locking key state */
	sc->sc_accents = 0;
	sc->sc_composed_char = 0;
#ifdef HIDKBD_EMULATE_ATSCANCODE
	sc->sc_buffered_char[0] = 0;
	sc->sc_buffered_char[1] = 0;
#endif
	memset(&sc->sc_ndata, 0, sizeof(sc->sc_ndata));
	memset(&sc->sc_odata, 0, sizeof(sc->sc_odata));
	memset(&sc->sc_ntime, 0, sizeof(sc->sc_ntime));
	memset(&sc->sc_otime, 0, sizeof(sc->sc_otime));
}

/* save the internal state, not used */
static int
hidkbd_get_state(keyboard_t *kbd, void *buf, size_t len)
{
	return (len == 0) ? 1 : -1;
}

/* set the internal state, not used */
static int
hidkbd_set_state(keyboard_t *kbd, void *buf, size_t len)
{
	return (EINVAL);
}

static int
hidkbd_poll(keyboard_t *kbd, int on)
{
	struct hidkbd_softc *sc = kbd->kb_data;

	HIDKBD_LOCK(sc);
	if (on) {
		sc->sc_flags |= HIDKBD_FLAG_POLLING;
		sc->sc_poll_thread = curthread;
	} else {
		sc->sc_flags &= ~HIDKBD_FLAG_POLLING;
		hidkbd_start_timer(sc);	/* start timer */
	}
	HIDKBD_UNLOCK(sc);

	return (0);
}

/* local functions */

static void
hidkbd_set_leds(struct hidkbd_softc *sc, uint8_t leds)
{
	HIDKBD_LOCK_ASSERT();
	DPRINTF("leds=0x%02x\n", leds);
	uint8_t *buf = NULL;

	sc->sc_leds = leds;

	if (hidkbd_no_leds || sc->sc_led_size == 0)
		return;

	if (sc->sc_outbuf[0] != NULL) {
		buf = sc->sc_outbuf[0];
		sc->sc_outbuf[0] = NULL;
	} else if (sc->sc_outbuf[1] != NULL) {
		buf = sc->sc_outbuf[1];
		sc->sc_outbuf[1] = NULL;
	}

	/* This should not be possible. */
	if (buf == NULL)
		return;

	memset(buf, 0, sc->sc_led_size);

	if (sc->sc_flags & HIDKBD_FLAG_NUMLOCK) {
		if (sc->sc_leds & NLKED) {
			hid_put_data_unsigned(buf, sc->sc_led_size,
			    &sc->sc_loc_numlock, 1);
		}
	}

	if (sc->sc_flags & HIDKBD_FLAG_SCROLLLOCK) {
		if (sc->sc_leds & SLKED) {
			hid_put_data_unsigned(buf, sc->sc_led_size,
			    &sc->sc_loc_scrolllock, 1);
		}
	}

	if (sc->sc_flags & HIDKBD_FLAG_CAPSLOCK) {
		if (sc->sc_leds & CLKED) {
			hid_put_data_unsigned(buf, sc->sc_led_size,
			    &sc->sc_loc_capslock, 1);
		}
	}

#ifdef EVDEV_SUPPORT
	if (sc->sc_evdev != NULL)
		evdev_push_leds(sc->sc_evdev, sc->sc_leds);
#endif

	HID_SET_REPORT(device_get_parent(sc->sc_dev), sc->sc_led_id,
	    buf, sc->sc_led_size);
}

#ifdef HIDKBD_EMULATE_ATSCANCODE
static int
hidkbd_key2scan(struct hidkbd_softc *sc, int code, int shift, int up)
{
	static const int scan[] = {
		/* 89 */
		0x11c,	/* Enter */
		/* 90-99 */
		0x11d,	/* Ctrl-R */
		0x135,	/* Divide */
		0x137 | SCAN_PREFIX_SHIFT,	/* PrintScreen */
		0x138,	/* Alt-R */
		0x147,	/* Home */
		0x148,	/* Up */
		0x149,	/* PageUp */
		0x14b,	/* Left */
		0x14d,	/* Right */
		0x14f,	/* End */
		/* 100-109 */
		0x150,	/* Down */
		0x151,	/* PageDown */
		0x152,	/* Insert */
		0x153,	/* Delete */
		0x146,	/* XXX Pause/Break */
		0x15b,	/* Win_L(Super_L) */
		0x15c,	/* Win_R(Super_R) */
		0x15d,	/* Application(Menu) */

		/* SUN TYPE 6 USB KEYBOARD */
		0x168,	/* Sun Type 6 Help */
		0x15e,	/* Sun Type 6 Stop */
		/* 110 - 119 */
		0x15f,	/* Sun Type 6 Again */
		0x160,	/* Sun Type 6 Props */
		0x161,	/* Sun Type 6 Undo */
		0x162,	/* Sun Type 6 Front */
		0x163,	/* Sun Type 6 Copy */
		0x164,	/* Sun Type 6 Open */
		0x165,	/* Sun Type 6 Paste */
		0x166,	/* Sun Type 6 Find */
		0x167,	/* Sun Type 6 Cut */
		0x125,	/* Sun Type 6 Mute */
		/* 120 - 128 */
		0x11f,	/* Sun Type 6 VolumeDown */
		0x11e,	/* Sun Type 6 VolumeUp */
		0x120,	/* Sun Type 6 PowerDown */

		/* Japanese 106/109 keyboard */
		0x73,	/* Keyboard Intl' 1 (backslash / underscore) */
		0x70,	/* Keyboard Intl' 2 (Katakana / Hiragana) */
		0x7d,	/* Keyboard Intl' 3 (Yen sign) (Not using in jp106/109) */
		0x79,	/* Keyboard Intl' 4 (Henkan) */
		0x7b,	/* Keyboard Intl' 5 (Muhenkan) */
		0x5c,	/* Keyboard Intl' 6 (Keypad ,) (For PC-9821 layout) */
	};

	if ((code >= 89) && (code < (int)(89 + NELEM(scan)))) {
		code = scan[code - 89];
	}
	/* Pause/Break */
	if ((code == 104) && (!(shift & (MOD_CONTROL_L | MOD_CONTROL_R)))) {
		code = (0x45 | SCAN_PREFIX_E1 | SCAN_PREFIX_CTL);
	}
	if (shift & (MOD_SHIFT_L | MOD_SHIFT_R)) {
		code &= ~SCAN_PREFIX_SHIFT;
	}
	code |= (up ? SCAN_RELEASE : SCAN_PRESS);

	if (code & SCAN_PREFIX) {
		if (code & SCAN_PREFIX_CTL) {
			/* Ctrl */
			sc->sc_buffered_char[0] = (0x1d | (code & SCAN_RELEASE));
			sc->sc_buffered_char[1] = (code & ~SCAN_PREFIX);
		} else if (code & SCAN_PREFIX_SHIFT) {
			/* Shift */
			sc->sc_buffered_char[0] = (0x2a | (code & SCAN_RELEASE));
			sc->sc_buffered_char[1] = (code & ~SCAN_PREFIX_SHIFT);
		} else {
			sc->sc_buffered_char[0] = (code & ~SCAN_PREFIX);
			sc->sc_buffered_char[1] = 0;
		}
		return ((code & SCAN_PREFIX_E0) ? 0xe0 : 0xe1);
	}
	return (code);

}

#endif					/* HIDKBD_EMULATE_ATSCANCODE */

static keyboard_switch_t hidkbdsw = {
	.probe = &hidkbd__probe,
	.init = &hidkbd_init,
	.term = &hidkbd_term,
	.intr = &hidkbd_intr,
	.test_if = &hidkbd_test_if,
	.enable = &hidkbd_enable,
	.disable = &hidkbd_disable,
	.read = &hidkbd_read,
	.check = &hidkbd_check,
	.read_char = &hidkbd_read_char,
	.check_char = &hidkbd_check_char,
	.ioctl = &hidkbd_ioctl,
	.lock = &hidkbd_lock,
	.clear_state = &hidkbd_clear_state,
	.get_state = &hidkbd_get_state,
	.set_state = &hidkbd_set_state,
	.get_fkeystr = &genkbd_get_fkeystr,
	.poll = &hidkbd_poll,
	.diag = &genkbd_diag,
};

KEYBOARD_DRIVER(hidkbd, hidkbdsw, hidkbd_configure);

static int
hidkbd_driver_load(module_t mod, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		kbd_add_driver(&hidkbd_kbd_driver);
		break;
	case MOD_UNLOAD:
		kbd_delete_driver(&hidkbd_kbd_driver);
		break;
	}
	return (0);
}

static devclass_t hidkbd_devclass;

static device_method_t hidkbd_methods[] = {
	DEVMETHOD(device_probe, hidkbd_probe),
	DEVMETHOD(device_attach, hidkbd_attach),
	DEVMETHOD(device_detach, hidkbd_detach),
	DEVMETHOD(device_resume, hidkbd_resume),
	DEVMETHOD_END
};

static driver_t hidkbd_driver = {
	.name = "hidkbd",
	.methods = hidkbd_methods,
	.size = sizeof(struct hidkbd_softc),
};

DRIVER_MODULE(hidkbd, usbhid, hidkbd_driver, hidkbd_devclass,
    hidkbd_driver_load, NULL);
#ifdef EVDEV_SUPPORT
MODULE_DEPEND(hidkbd, evdev, 1, 1, 1);
#endif
MODULE_VERSION(ukbd, 1);

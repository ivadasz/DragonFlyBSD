#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/lock.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/uio.h>
#include <sys/fcntl.h>
#include <sys/uuid.h>
#include <sys/event.h>
#include <sys/devfs.h>
#include <sys/vnode.h>

#include <bus/u4b/hid_common.h>

#include "hid_if.h"

struct elopacket {
	uint8_t sync;
	uint8_t cmd;
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;
	uint8_t byte5;
	uint8_t byte6;
	uint8_t byte7;
	uint8_t byte8;
	uint8_t csum;
} __packed;

typedef struct elopacket elopacket_t;

typedef struct hidts_track {
	uint16_t x;
	uint16_t y;
	uint16_t pressure;
	int	status;
	int	report;		/* what we have to report */
} hidts_track_t;

#define IICHID_TRACK_RELEASED	0
#define IICHID_TRACK_PRESSED	1

#define IICHID_REPORT_PRESS	0x0001
#define IICHID_REPORT_MOVE	0x0002
#define IICHID_REPORT_RELEASE	0x0004

#define IICHID_MAXTRACK		100

#define IICHID_FLAG_X	0x1
#define IICHID_FLAG_Y	0x2
#define IICHID_FLAG_TIP	0x4

struct hidts_softc {
	device_t	dev;

	int		isize;
	char		*report_desc;
	uint16_t	desc_len;
	int		report_id;
	struct lock	lk;

	struct kqinfo	kqinfo;
	int		count;
	int		unit;
	cdev_t		devnode;
	hidts_track_t	track[IICHID_MAXTRACK];
	int		tracking;
	elopacket_t	pend_rep;	/* pending reply to command */
	int		pend_ack;	/* pending reply mode */
	int		data_signal;	/* something ready to read */
	int		blocked;	/* someone is blocking */
	int		fill;		/* fill level of track */
	int		state;

	struct hid_location sc_loc_x;
	struct hid_location sc_loc_y;
	struct hid_location sc_loc_tip;

	uint8_t sc_iid_x;
	uint8_t sc_iid_y;
	uint8_t sc_iid_tip;

	int flags;
	uint8_t sc_iid;

	/* range (inclusive) of reported x/y values */
	int sc_minx;
	int sc_maxx;
	int sc_miny;
	int sc_maxy;

	int have_minmax;

	/* XXX */
};

#define PEND_ACK_NONE		0	/* no reply to command pending */
#define PEND_ACK_RESPOND	1	/* reply w/response and ack */
#define PEND_ACK_ACK		2	/* reply w/ack only */

static int	hidts_probe(device_t dev);
static int	hidts_attach(device_t dev);
static int	hidts_detach(device_t dev);

static void	hidts_handle_input(struct hidts_softc *sc, char *buf, int len);
//static void	hidts_intr(void *arg);
static void	hidts_init_digitizer(struct hidts_softc *sc);

static void	hidts_find_active_state(struct hidts_softc *sc);
static void	hidts_notify(struct hidts_softc *sc);
static void	hidts_input(struct hidts_softc *sc, uint16_t x, uint16_t y,
		    int pressed);

SYSCTL_NODE(_hw, OID_AUTO, hidts, CTLFLAG_RD, 0, "hidts driver");

int hidts_debug = 0;
TUNABLE_INT("hw.hidts.debug", &hidts_debug);
SYSCTL_INT(_hw_hidts, OID_AUTO, debug, CTLFLAG_RW, &hidts_debug, 0, "Debug level");

static	d_open_t	hidts_open;
static	d_close_t	hidts_close;
static	d_ioctl_t	hidts_ioctl;
static	d_read_t	hidts_read;
static	d_write_t	hidts_write;
static	d_kqfilter_t	hidts_kqfilter;

static struct dev_ops hidts_ops = {
	{ "hidts", 0, 0 },
	.d_open =	hidts_open,
	.d_close =	hidts_close,
	.d_ioctl =	hidts_ioctl,
	.d_read =	hidts_read,
	.d_write =	hidts_write,
	.d_kqfilter =	hidts_kqfilter,
};

static void
hidts_handle_input(struct hidts_softc *sc, char *buf, int len)
{
#if 0
	u_char *buf = (u_char *)sc->input_report + 2;
	uint32_t len = count - 2;
	int32_t x = 0, y = 0, tip = 0;
	uint8_t id;

	if (sc->sc_iid) {
		id = *(uint8_t *)buf;

		len--;
		buf++;
	} else {
		id = 0;
	}
	if (count <= 2 || (sc->sc_iid && count <= 3)) {
		device_printf(sc->dev,
		    "input report too short, count=%d\n", count);
		return;
	}

	if (id == sc->sc_iid_x && (sc->flags & IICHID_FLAG_X)) {
		x = hid_get_data(buf, len, &sc->sc_loc_x);
	}
	if (id == sc->sc_iid_y && (sc->flags & IICHID_FLAG_Y)) {
		y = hid_get_data(buf, len, &sc->sc_loc_y);
	}
	if (id == sc->sc_iid_tip && (sc->flags & IICHID_FLAG_TIP)) {
		tip = hid_get_data_unsigned(buf, len, &sc->sc_loc_tip);
	}

	if (hidts_debug > 0) {
		device_printf(sc->dev, "tip: %d raw: x=%d, y=%d\n",
		    tip, x, y);
	}

	hidts_input(sc, x, y, tip);
#endif
}

/* XXX for now use a handle_report method */
#if 0
static void
iichid_intr(void *arg)
{
	struct iichid_softc *sc = (struct iichid_softc *)arg;
	int count, val;

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	val = iichid_fetch_report(sc, &count);
	if (val == -1)
		wakeup(sc);
	if (val == 0) {
		if (iichid_debug > 1) {
			device_printf(sc->dev, "input report:\n");
			iichid_dump_bytes(sc->input_report, count);
		}
		iichid_handle_input(sc, count);
	}
	lockmgr(&sc->lk, LK_RELEASE);
}
#endif

static void
hidts_init_digitizer(struct hidts_softc *sc)
{
	char *buf = sc->report_desc;
	uint32_t len = sc->desc_len;
	int flags, id;

	id = sc->report_id;
	sc->isize = hid_report_size(buf, len, hid_input, id);
	device_printf(sc->dev, "%s: isize=%d report-id=%d\n",
	    __func__, sc->isize, id);

	if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_X),
	    hid_input, 0, &sc->sc_loc_x, &flags, &sc->sc_iid_x)) {
		device_printf(sc->dev, "%s: X: sc_iid_x=%d, flags=0x%x\n",
		    __func__, sc->sc_iid_x, flags);
		sc->flags |= IICHID_FLAG_X;
	} else {
		device_printf(sc->dev, "%s: failed for X\n", __func__);
	}

	if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_Y),
	    hid_input, 0, &sc->sc_loc_y, &flags, &sc->sc_iid_y)) {
		device_printf(sc->dev, "%s: Y: sc_iid_y=%d, flags=0x%x\n",
		    __func__, sc->sc_iid_y, flags);
		sc->flags |= IICHID_FLAG_Y;
	} else {
		device_printf(sc->dev, "%s: failed for Y\n", __func__);
	}

	if (hid_locate(buf, len, HID_USAGE2(HUP_DIGITIZERS, HUD_TIP_SWITCH),
	    hid_input, 1, &sc->sc_loc_tip, &flags, &sc->sc_iid_tip)) {
		device_printf(sc->dev, "%s: TIP: sc_iid_tip=%d, flags=0x%x\n",
		    __func__, sc->sc_iid_tip, flags);
		sc->flags |= IICHID_FLAG_TIP;
	} else {
		device_printf(sc->dev, "%s: failed for TIP\n", __func__);
	}

	/* XXX */

	struct hid_data * d;
	struct hid_item item;
	int havex = 0, havey = 0;

	d = hid_start_parse(buf, len, 1 << hid_input);
	while (hid_get_item(d, &item)) {
		if (item.kind != hid_input
		    || HID_GET_USAGE_PAGE(item.usage) != HUP_GENERIC_DESKTOP
		    || item.report_ID != sc->sc_iid_x)
			continue;
		if (HID_GET_USAGE(item.usage) == HUG_X && !havex) {
			sc->sc_minx = item.logical_minimum;
			sc->sc_maxx = item.logical_maximum;
			havex = 1;
		}
		if (HID_GET_USAGE(item.usage) == HUG_Y && !havey) {
			sc->sc_miny = item.logical_minimum;
			sc->sc_maxy = item.logical_maximum;
			havey = 1;
		}
	}
        hid_end_parse(d);

	if (havex && havey) {
		/* Always report detected minimum/maximum values */
		device_printf(sc->dev, "minx=%d maxx=%d miny=%d maxy=%d\n",
		    sc->sc_minx, sc->sc_maxx, sc->sc_miny, sc->sc_maxy);
	}

	/*
	 * Only flag min/max values valid if it's trivial to utilize them
	 * with the elographics xorg input protocol (i.e. all values would
	 * fit into a 16 bit value).
	 */
	if (havex && havey &&
	    sc->sc_maxx > sc->sc_minx &&
	    sc->sc_maxy > sc->sc_miny &&
	    sc->sc_maxx < (1 << 15) && sc->sc_maxy < (1 << 15) &&
	    sc->sc_minx >= 0 && sc->sc_miny >= 0) {
		sc->have_minmax = 1;
	} else {
		device_printf(sc->dev,
		    "No usable minimum/maximum coordinate values\n");
		sc->have_minmax = 0;
	}
}

static void
hidts_input_handler(uint8_t id, uint8_t *buf, int count, void *arg)
{
	struct hidts_softc *sc = arg;
	uint32_t x = 0, y = 0;

	if (id == sc->sc_iid_x && (sc->flags & IICHID_FLAG_X)) {
		x = hid_get_data(buf, count, &sc->sc_loc_x);
	}
	if (id == sc->sc_iid_y && (sc->flags & IICHID_FLAG_Y)) {
		y = hid_get_data(buf, count, &sc->sc_loc_y);
	}

	device_printf(sc->dev, "Got %d bytes input on report id %u x=%u y=%u\n",
	    count, id, x, y);

	/* XXX */
}

/* user device I/O functions */
static int
hidts_open(struct dev_open_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct hidts_softc *sc = dev->si_drv1;

	if (sc == NULL)
		return (ENXIO);

	if (sc->count != 0)
		return (EBUSY);

	sc->count++;

	return (0);
}

static int
hidts_close(struct dev_close_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct hidts_softc *sc = dev->si_drv1;

	if (sc == NULL)
		return (ENXIO);

	if (sc->count == 0) {
		/* This is not supposed to happen. */
		return (0);
	}

	sc->count--;

	return (0);
}

static int
hidts_read(struct dev_read_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct hidts_softc *sc = dev->si_drv1;
	int error;
	struct uio *uio = ap->a_uio;
	int ioflag = ap->a_ioflag;
	size_t n;
	elopacket_t pkt;
	hidts_track_t *track;

	/*
	 * Load next ready event, block if necessary.
	 */
	lockmgr(&sc->lk, LK_EXCLUSIVE);
	for (;;) {
		error = 0;

		switch(sc->pend_ack) {
		case PEND_ACK_NONE:
			if (sc->fill > 0) {
				/*
				 * Report ready
				 */
				track = &sc->track[0];
				bzero(&pkt, sizeof(pkt));
				pkt.cmd = 'T';
				if (track->report & IICHID_REPORT_PRESS) {
					pkt.byte2 |= 0x01;
					track->report &= ~IICHID_REPORT_PRESS;
				} else if (track->report & IICHID_REPORT_MOVE) {
					pkt.byte2 |= 0x02;
					track->report &= ~IICHID_REPORT_MOVE;
				} else if (track->report &
					   IICHID_REPORT_RELEASE) {
					pkt.byte2 |= 0x04;
					track->report &= ~IICHID_REPORT_RELEASE;
				}
				pkt.byte3 = track->x & 0xFF;
				pkt.byte4 = track->x >> 8;
				pkt.byte5 = track->y & 0xFF;
				pkt.byte6 = track->y >> 8;
				pkt.byte7 = track->pressure & 0xFF;
				pkt.byte8 = track->pressure >> 8;
				sc->fill--;
				memmove(&sc->track[0], &sc->track[1],
				    sc->fill * sizeof(hidts_track_t));
			} else if (ioflag & IO_NDELAY) {
				/*
				 * Non-blocking, nothing ready
				 */
				error = EWOULDBLOCK;
			} else {
				/*
				 * Blocking, nothing ready
				 */
				sc->data_signal = 0;
				sc->blocked = 1;
				error = lksleep(&sc->blocked, &sc->lk,
				    PCATCH, "hidtsw", 0);
				if (error == 0)
					continue;
			}
			break;
		case PEND_ACK_RESPOND:
			pkt = sc->pend_rep;
			sc->pend_ack = PEND_ACK_ACK;
			break;
		case PEND_ACK_ACK:
			bzero(&pkt, sizeof(pkt));
			pkt.cmd = 'A';
			sc->pend_ack = PEND_ACK_NONE;
			break;
		}
		hidts_find_active_state(sc);
		break;
	}
	lockmgr(&sc->lk, LK_RELEASE);

	/*
	 * If no error we can return the event loaded into pkt.
	 */
	if (error == 0) {
		uint8_t csum = 0xAA;
		int i;

		pkt.sync = 'U';
		for (i = 0; i < sizeof(pkt) - 1; ++i)
			csum += ((uint8_t *)&pkt)[i];
		pkt.csum = csum;
		n = uio->uio_resid;
		if (n > sizeof(pkt))
			n = sizeof(pkt);
		error = uiomove((void *)&pkt, n, uio);
	}

	return (error);
}

static int
hidts_write(struct dev_write_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct hidts_softc *sc = dev->si_drv1;
	struct uio *uio = ap->a_uio;
	elopacket_t pkt;
	int error;
	size_t n;

	error = 0;

	while (uio->uio_resid) {
		bzero(&pkt, sizeof(pkt));
		n = uio->uio_resid;
		if (n > sizeof(pkt))
			n = sizeof(pkt);
		error = uiomove((void *)&pkt, n, uio);
		if (error)
			break;
		lockmgr(&sc->lk, LK_EXCLUSIVE);
		switch(pkt.cmd) {
		case 'i':
			/*
			 * ELO_ID request id
			 */
			bzero(&sc->pend_rep, sizeof(sc->pend_rep));
			sc->pend_rep.cmd = 'I';
			sc->pend_ack = PEND_ACK_RESPOND;
			break;
		case 'o':
			/*
			 * ELO_OWNER request owner
			 */
			bzero(&sc->pend_rep, sizeof(sc->pend_rep));
			sc->pend_rep.cmd = 'O';
			sc->pend_ack = PEND_ACK_RESPOND;
			break;
		case 'm':
			/*
			 * ELO_MODE control packet
			 */
			sc->pend_ack = PEND_ACK_ACK;
			break;
		case 'r':
			/*
			 * ELO_REPORT control packet
			 */
			sc->pend_ack = PEND_ACK_ACK;
			break;
		}
		lockmgr(&sc->lk, LK_RELEASE);
	}
	return (error);
}

static void hidts_filt_detach(struct knote *);
static int hidts_filt(struct knote *, long);

static struct filterops hidts_filtops =
	{ FILTEROP_ISFD, NULL, hidts_filt_detach, hidts_filt };

static int
hidts_kqfilter(struct dev_kqfilter_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct hidts_softc *sc = dev->si_drv1;
	struct knote *kn = ap->a_kn;
	struct klist *klist;

	switch(kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &hidts_filtops;
		kn->kn_hook = (void *)sc;
		ap->a_result = 0;
		break;
	default:
		ap->a_result = EOPNOTSUPP;
		return (0);
	}
	klist = &sc->kqinfo.ki_note;
	knote_insert(klist, kn);

	return (0);
}

static void
hidts_filt_detach(struct knote *kn)
{
	struct hidts_softc *sc = (struct hidts_softc *)kn->kn_hook;
	struct klist *klist;

	klist = &sc->kqinfo.ki_note;
	knote_remove(klist, kn);
}

static int
hidts_filt(struct knote *kn, long hint)
{
	struct hidts_softc *sc = (struct hidts_softc *)kn->kn_hook;
	int ready;

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	if (sc->data_signal)
		ready = 1;
	else
		ready = 0;
	lockmgr(&sc->lk, LK_RELEASE);

	return (ready);
}

static int
hidts_ioctl(struct dev_ioctl_args *ap)
{
	return (ENOTTY);
}

/*
 * Notify if possible receive data ready.  Must be called
 * without the lock held to avoid deadlocking in kqueue.
 */
static void
hidts_notify(struct hidts_softc *sc)
{
	if (sc->data_signal) {
		KNOTE(&sc->kqinfo.ki_note, 0);
		lockmgr(&sc->lk, LK_EXCLUSIVE);
		if (sc->blocked) {
			sc->blocked = 0;
			wakeup(&sc->blocked);
		}
		lockmgr(&sc->lk, LK_RELEASE);
	}
}

static void
hidts_input(struct hidts_softc *sc, uint16_t x, uint16_t y, int pressed)
{
	hidts_track_t *track;
	int donotify = 0;

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	if (sc->count <= 0)
		goto done;
	/* Drop packets if buffer full */
	if (sc->fill >= IICHID_MAXTRACK) {
		donotify = 1;
		goto done;
	}
	track = &sc->track[sc->fill];
	bzero(track, sizeof(*track));
	sc->fill++;
	track->x = x;
	track->y = y;
	track->pressure = pressed ? 0xffff : 0;
	if (pressed) {
		if (sc->state) {
			track->report = IICHID_REPORT_MOVE;
		} else {
			sc->state = 1;
			track->report = IICHID_REPORT_PRESS;
		}
	} else {
		sc->state = 0;
		track->report = IICHID_REPORT_RELEASE;
	}
	hidts_find_active_state(sc);
	donotify = 1;
done:
	lockmgr(&sc->lk, LK_RELEASE);
	if (donotify)
		hidts_notify(sc);
}

/*
 * Calculate currently active state, if any
 */
static void
hidts_find_active_state(struct hidts_softc *sc)
{
	if (sc->fill <= 0 && sc->pend_ack == PEND_ACK_NONE) {
		sc->data_signal = 0;
	} else {
		sc->data_signal = 1;
	}
}

/* device_if functions */
static int
hidts_probe(device_t dev)
{
	char *buf;
	uint16_t len;

	HID_GET_DESCRIPTOR(device_get_parent(dev), &buf, &len);

	if (hid_is_digitizer(buf, len))
		device_set_desc(dev, "HID Touchscreen device");
	else if (hid_is_absmouse(buf, len))
		device_set_desc(dev, "HID Tablet device");
	else
		return (ENXIO);

	return (0);
}

static int
hidts_attach(device_t dev)
{
	struct hidts_softc *sc = device_get_softc(dev);

	sc->dev = dev;

	lockinit(&sc->lk, "hidlk", 0, LK_CANRECURSE);

	HID_GET_DESCRIPTOR(device_get_parent(dev), &sc->report_desc,
	    &sc->desc_len);

	hidts_init_digitizer(sc);

	/* Enable input report handler */
	HID_SET_HANDLER(device_get_parent(dev), hidts_input_handler, sc);
	(void)hidts_handle_input;
	(void)hidts_input;

	sc->unit = device_get_unit(dev);
	if (sc->unit & 0x0400) {
		sc->devnode = make_dev(&hidts_ops, sc->unit,
		    UID_ROOT, GID_WHEEL, 0600, "hidts%d-%02x",
		    sc->unit >> 11, sc->unit % 1023);
	} else {
		sc->devnode = make_dev(&hidts_ops, sc->unit,
		    UID_ROOT, GID_WHEEL, 0600, "hidts%d", sc->unit);
	}
	sc->devnode->si_drv1 = sc;

	/*
	 * XXX We need a way to report the minimum/maximum x/y values of
	 *     the touchscreen to userspace
	 */

	HID_START(device_get_parent(dev));

	return (0);
}

static int
hidts_detach(device_t dev)
{
	struct hidts_softc *sc = device_get_softc(dev);

	HID_STOP(device_get_parent(dev));

	/* Disable input report handler */
	HID_SET_HANDLER(device_get_parent(dev), NULL, NULL);

	if (sc->devnode) {
		dev_ops_remove_minor(&hidts_ops, sc->unit);
		devfs_assume_knotes(sc->devnode, &sc->kqinfo);
	}

	return (0);
}

static device_method_t hidts_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, hidts_probe),
	DEVMETHOD(device_attach, hidts_attach),
	DEVMETHOD(device_detach, hidts_detach),

	DEVMETHOD_END
};

static driver_t hidts_driver = {
	"hidts",
	hidts_methods,
	sizeof(struct hidts_softc),
};
static devclass_t hidts_devclass;

DRIVER_MODULE(hidts, uhid, hidts_driver, hidts_devclass, NULL, NULL);
MODULE_VERSION(hidts, 1);

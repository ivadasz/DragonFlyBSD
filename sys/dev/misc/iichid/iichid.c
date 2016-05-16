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

#include "opt_acpi.h"
#include "acpi.h"
#include <dev/acpica/acpivar.h>

#include <bus/pci/pcivar.h>

#include <bus/gpio/gpio_acpi/gpio_acpivar.h>
#include <bus/smbus/smbacpi/smbus_acpivar.h>

#include <bus/u4b/hid.h>

#include "iichid.h"

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

typedef struct iichid_track {
	uint16_t x;
	uint16_t y;
	uint16_t pressure;
	int	status;
	int	report;		/* what we have to report */
} iichid_track_t;

#define IICHID_TRACK_RELEASED	0
#define IICHID_TRACK_PRESSED	1

#define IICHID_REPORT_PRESS	0x0001
#define IICHID_REPORT_MOVE	0x0002
#define IICHID_REPORT_RELEASE	0x0004

#define IICHID_MAXTRACK		100

#define IICHID_FLAG_X	0x1
#define IICHID_FLAG_Y	0x2
#define IICHID_FLAG_TIP	0x4

struct iichid_softc {
	device_t	dev;
	ACPI_HANDLE	handle;

	struct iicserial_resource *iicres;
	struct gpioint_resource *intres;

	struct iic_hid_descriptor hid_descriptor;
	u_char		*report_descriptor;
	u_char		*input_report;
	int		poweron;
	struct lock	lk;

	struct kqinfo	kqinfo;
	int		count;
	int		unit;
	cdev_t		devnode;
	iichid_track_t	track[IICHID_MAXTRACK];
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

static int	iichid_probe(device_t dev);
static int	iichid_attach(device_t dev);
static int	iichid_detach(device_t dev);

static int	iichid_get_descriptor_address(struct iichid_softc *sc,
		    uint16_t *addr);
static void	iichid_dump_bytes(uint8_t *buf, u_int count);
static int	iichid_readreg(struct iichid_softc *sc, uint16_t reg,
		    u_char *buf, int count, int *actualp);
static int	iichid_readreg_check(struct iichid_softc *sc, uint16_t reg,
		    u_char *buf, int count);
static int	iichid_writereg16(struct iichid_softc *sc, uint16_t reg,
		    uint16_t value);
static int	iichid_reset(struct iichid_softc *sc);
static int	iichid_setpower(struct iichid_softc *sc, int on);
static int	iichid_fetch_report(struct iichid_softc *sc, int *actualp);
static void	iichid_handle_input(struct iichid_softc *sc, int count);
static void	iichid_intr(void *arg, enum gpio_event event);
static void	iichid_init_digitizer(struct iichid_softc *sc);

static void	iichid_find_active_state(struct iichid_softc *sc);
static void	iichid_notify(struct iichid_softc *sc);
static void	iichid_input(struct iichid_softc *sc, uint16_t x, uint16_t y,
		    int pressed);

static int	uhidev_maxrepid(void *buf, int len);

SYSCTL_NODE(_hw, OID_AUTO, iichid, CTLFLAG_RD, 0, "iichid driver");

int iichid_debug = 0;
TUNABLE_INT("hw.iichid.debug", &iichid_debug);
SYSCTL_INT(_hw_iichid, OID_AUTO, debug, CTLFLAG_RW, &iichid_debug, 0, "Debug level");

static	d_open_t	iichid_open;
static	d_close_t	iichid_close;
static	d_ioctl_t	iichid_ioctl;
static	d_read_t	iichid_read;
static	d_write_t	iichid_write;
static	d_kqfilter_t	iichid_kqfilter;

static struct dev_ops iichid_ops = {
	{ "iichid", 0, 0 },
	.d_open =	iichid_open,
	.d_close =	iichid_close,
	.d_ioctl =	iichid_ioctl,
	.d_read =	iichid_read,
	.d_write =	iichid_write,
	.d_kqfilter =	iichid_kqfilter,
};

static int
iichid_get_descriptor_address(struct iichid_softc *sc, uint16_t *addr)
{
	ACPI_BUFFER retbuf = { ACPI_ALLOCATE_BUFFER, NULL };
	ACPI_OBJECT_LIST arglist;
	ACPI_OBJECT arg[4];
	ACPI_STATUS status;
	ACPI_OBJECT *retobj;
	const char *uuidstr = "3cdff6f7-4267-4555-ad05-b30a3d8938de";
	struct uuid uuid;
	uint8_t dsmuuid[16];

	if (parse_uuid(uuidstr, &uuid) != 0)
		return (1);
	le_uuid_enc(dsmuuid, &uuid);

	arglist.Pointer = arg;
	arglist.Count = 4;
	arg[0].Type = ACPI_TYPE_BUFFER;
	arg[0].Buffer.Length = sizeof(dsmuuid);
	arg[0].Buffer.Pointer = dsmuuid;
	arg[1].Type = ACPI_TYPE_INTEGER;
	arg[1].Integer.Value = 0;
	arg[2].Type = ACPI_TYPE_INTEGER;
	arg[2].Integer.Value = 1;
	arg[3].Type = ACPI_TYPE_PACKAGE;
	arg[3].Package.Count = 0;
	arg[3].Package.Elements = NULL;

	status = AcpiEvaluateObject(sc->handle, "_DSM", &arglist, &retbuf);
	if (ACPI_FAILURE(status))
		return (1);
	retobj = retbuf.Pointer;
	if (retobj->Type != ACPI_TYPE_INTEGER) {
		AcpiOsFree(retbuf.Pointer);
		return (1);
	}
	*addr = retobj->Integer.Value;

	return (0);
}

static void
iichid_dump_bytes(uint8_t *buf, u_int count)
{
	u_int i, j;

	for (i = 0; i < count; i += 8) {
		kprintf("0x%x:\t", i);
		for (j = i; j < i + 8 && j < count; j++)
			kprintf("0x%02x ", buf[j]);
		kprintf("\n");
	}
}

static int
iichid_readreg(struct iichid_softc *sc, uint16_t reg, u_char *buf, int count,
    int *actualp)
{
	if (iicserial_rawtrans(sc->iicres, (char *)&reg, 2, buf, count,
	    actualp) != 0 ) {
		device_printf(sc->dev,
		    "failed to read %d bytes from reg 0x%04x\n", count, reg);
		return (1);
	}
	return (0);
}

static int
iichid_readreg_check(struct iichid_softc *sc, uint16_t reg, u_char *buf,
    int count)
{
	int actualp, val;

	if ((val = iichid_readreg(sc, reg, buf, count, &actualp)) != 0)
		return (val);

	if (actualp != count) {
		device_printf(sc->dev,
		    "read wrong number of bytes from reg 0x%04x, wanted "
		    "%d bytes, got %d\n", reg, count, actualp);
		return (1);
	}
	return (0);
}

static int
iichid_writereg16(struct iichid_softc *sc, uint16_t reg, uint16_t value)
{
	uint16_t data[2] = { reg, value };

	if (iicserial_rawtrans(sc->iicres, (char *)data, 4, NULL, 0, NULL)
	    != 0) {
		device_printf(sc->dev, "failed to set reg 0x%04x to 0x%04x\n",
		    reg, value);
		return (1);
	}

	return (0);
}

static int
iichid_reset(struct iichid_softc *sc)
{
	uint16_t req = 0x0100;

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	if (iichid_writereg16(sc, sc->hid_descriptor.wCommandRegister, req)
	    != 0) {
		lockmgr(&sc->lk, LK_RELEASE);
		return (1);
	}

	/* Wait for GPIO interrupt and 2 byte input report with length 0 */
	if (lksleep(sc, &sc->lk, 0, "iichid", 5*hz) == EWOULDBLOCK) {
		device_printf(sc->dev, "reset timed out\n");
		lockmgr(&sc->lk, LK_RELEASE);
		return 1;
	}

	lockmgr(&sc->lk, LK_RELEASE);
	return (0);
}

static int
iichid_setpower(struct iichid_softc *sc, int on)
{
	uint16_t req = (0x08 << 8) | (on ? 0x00 : 0x01);
	int error = 0;

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	if (iichid_writereg16(sc, sc->hid_descriptor.wCommandRegister, req)
	    != 0) {
		device_printf(sc->dev, "%s(sc, %d) failed\n", __func__, on);
		error = 1;
	}
	lockmgr(&sc->lk, LK_RELEASE);

	return (error);
}

static int
iichid_fetch_report(struct iichid_softc *sc, int *actualp)
{
	uint16_t length;
	uint16_t reg = sc->hid_descriptor.wInputRegister;
	uint16_t maxcount = sc->hid_descriptor.wMaxInputLength;
	int count;

	if (iichid_readreg(sc, reg, sc->input_report, maxcount, &count) != 0) {
		device_printf(sc->dev, "fetching input report failed\n");
		return (1);
	}
	if (count < 2) {
		device_printf(sc->dev, "input report too short, %d bytes\n",
		    count);
		return (1);
	}
	length = *(uint16_t *)&sc->input_report[0];
	if (iichid_debug > 1) {
		device_printf(sc->dev, "length: 0x%04x actualp: 0x%04x\n",
		    length, count);
	}
	if (length == 0) {
		device_printf(sc->dev, "reset found\n");
		return (-1);
	}
	if (count != length) {
		device_printf(sc->dev, "length != count\n");
		return (1);
	}
	*actualp = length;

	return (0);
}

static void
iichid_handle_input(struct iichid_softc *sc, int count)
{
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

	if (iichid_debug > 0) {
		device_printf(sc->dev, "tip: %d raw: x=%d, y=%d\n",
		    tip, x, y);
	}

	iichid_input(sc, x, y, tip);
}

static void
iichid_intr(void *arg, enum gpio_event event)
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

static void
iichid_init_digitizer(struct iichid_softc *sc)
{
	u_char *buf = (u_char *)sc->report_descriptor;
	uint32_t len = sc->hid_descriptor.wReportDescLength;
	int flags, isize;

	int nrepid = uhidev_maxrepid(buf, len);
	device_printf(sc->dev, "nrepid=%d\n", nrepid);

	isize = hid_report_size(buf, len, hid_input, &sc->sc_iid);
	device_printf(sc->dev, "%s: isize=%d sc->sc_iid=0x%02x\n",
	    __func__, isize, sc->sc_iid);

	if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_X),
	    hid_input, 1, &sc->sc_loc_x, &flags, &sc->sc_iid_x)) {
		device_printf(sc->dev, "%s: X: sc_iid_x=%d, flags=0x%x\n",
		    __func__, sc->sc_iid_x, flags);
		sc->flags |= IICHID_FLAG_X;
	} else {
		device_printf(sc->dev, "%s: failed for X\n", __func__);
	}

	if (hid_locate(buf, len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_Y),
	    hid_input, 1, &sc->sc_loc_y, &flags, &sc->sc_iid_y)) {
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

/* user device I/O functions */
static int
iichid_open(struct dev_open_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct iichid_softc *sc = dev->si_drv1;

	if (sc == NULL)
		return (ENXIO);

	if (sc->count != 0)
		return (EBUSY);

	if (sc->poweron) {
		device_printf(sc->dev, "Already powered on\n");
	} else {
		iichid_setpower(sc, 1);
		sc->poweron = 1;
	}

	sc->count++;

	return (0);
}

static int
iichid_close(struct dev_close_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct iichid_softc *sc = dev->si_drv1;

	if (sc == NULL)
		return (ENXIO);

	if (sc->count == 0) {
		/* This is not supposed to happen. */
		return (0);
	}

	if (sc->poweron) {
		iichid_setpower(sc, 0);
		sc->poweron = 0;
	} else {
		device_printf(sc->dev, "Already powered off\n");
	}

	sc->count--;

	return (0);
}

static int
iichid_read(struct dev_read_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct iichid_softc *sc = dev->si_drv1;
	int error;
	struct uio *uio = ap->a_uio;
	int ioflag = ap->a_ioflag;
	size_t n;
	elopacket_t pkt;
	iichid_track_t *track;

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
				    sc->fill * sizeof(iichid_track_t));
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
				    PCATCH, "iichidw", 0);
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
		iichid_find_active_state(sc);
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
iichid_write(struct dev_write_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct iichid_softc *sc = dev->si_drv1;
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

static void iichid_filt_detach(struct knote *);
static int iichid_filt(struct knote *, long);

static struct filterops iichid_filtops =
	{ FILTEROP_ISFD, NULL, iichid_filt_detach, iichid_filt };

static int
iichid_kqfilter(struct dev_kqfilter_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct iichid_softc *sc = dev->si_drv1;
	struct knote *kn = ap->a_kn;
	struct klist *klist;

	switch(kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &iichid_filtops;
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
iichid_filt_detach(struct knote *kn)
{
	struct iichid_softc *sc = (struct iichid_softc *)kn->kn_hook;
	struct klist *klist;

	klist = &sc->kqinfo.ki_note;
	knote_remove(klist, kn);
}

static int
iichid_filt(struct knote *kn, long hint)
{
	struct iichid_softc *sc = (struct iichid_softc *)kn->kn_hook;
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
iichid_ioctl(struct dev_ioctl_args *ap)
{
	return (ENOTTY);
}

/*
 * Notify if possible receive data ready.  Must be called
 * without the lock held to avoid deadlocking in kqueue.
 */
static void
iichid_notify(struct iichid_softc *sc)
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
iichid_input(struct iichid_softc *sc, uint16_t x, uint16_t y, int pressed)
{
	iichid_track_t *track;
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
	track->x = x;
	track->y = y;
	track->pressure = pressed ? 0xffff : 0;
	if (sc->fill > 0) {
		if (track->x == sc->track[sc->fill - 1].x &&
		    track->y == sc->track[sc->fill - 1].y &&
		    track->pressure == sc->track[sc->fill - 1].pressure)
			goto done;
	}
	sc->fill++;
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
	iichid_find_active_state(sc);
	donotify = 1;
done:
	lockmgr(&sc->lk, LK_RELEASE);
	if (donotify)
		iichid_notify(sc);
}

/*
 * Calculate currently active state, if any
 */
static void
iichid_find_active_state(struct iichid_softc *sc)
{
	if (sc->fill <= 0 && sc->pend_ack == PEND_ACK_NONE) {
		sc->data_signal = 0;
	} else {
		sc->data_signal = 1;
	}
}

/* device_if functions */
static int
uhidev_maxrepid(void *buf, int len)
{
	struct hid_data *d;
	struct hid_item h;
	int maxid;

	maxid = -1;
	h.report_ID = 0;
	for (d = hid_start_parse(buf, len, hid_input); hid_get_item(d, &h); )
		if (h.report_ID > maxid)
			maxid = h.report_ID;
	hid_end_parse(d);
	return (maxid);
}

static int
iichid_probe(device_t dev)
{
	static char *iichid_ids[] = { "PNP0C50", NULL };

	if (acpi_disabled("iichid") ||
	    ACPI_ID_PROBE(device_get_parent(dev), dev, iichid_ids) == NULL)
		return (ENXIO);

	device_set_desc(dev, "I2C HID device");
	return (0);
}

static int
iichid_attach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);
	uint16_t desc_addr;

	sc->dev = dev;
	sc->handle = acpi_get_handle(dev);

	lockinit(&sc->lk, "iiclk", 0, LK_CANRECURSE);

	sc->iicres = iic_alloc_resource(dev, 0);
	if (sc->iicres == NULL) {
		device_printf(dev, "Failed to alloc I2cSerialBus resource\n");
		return (ENXIO);
	}
	sc->intres = gpioint_alloc_resource(dev, 0);
	if (sc->intres == NULL) {
		device_printf(dev, "Failed to alloc GpioInt resource\n");
		iic_free_resource(dev, sc->iicres);
		return (ENXIO);
	}

	pci_set_powerstate(dev, PCI_POWERSTATE_D0);

	if (iichid_get_descriptor_address(sc, &desc_addr) != 0) {
		device_printf(dev,
		    "Fetching descriptor address via _DSM failed\n");
		pci_set_powerstate(dev, PCI_POWERSTATE_D3);
		iic_free_resource(dev, sc->iicres);
		gpioint_free_resource(dev, sc->intres);
		return (ENXIO);
	}
	device_printf(dev, "Fetching I2C-HID descriptor from addr 0x%04x\n",
	    desc_addr);
	KKASSERT(sizeof(sc->hid_descriptor) == 30);
	if (iichid_readreg_check(sc, desc_addr, (u_char *)&sc->hid_descriptor,
	    30) != 0) {
		pci_set_powerstate(dev, PCI_POWERSTATE_D3);
		iic_free_resource(dev, sc->iicres);
		gpioint_free_resource(dev, sc->intres);
		return (ENXIO);
	}
	device_printf(dev, "HID descriptor:\n");
	iichid_dump_bytes((u_char *)&sc->hid_descriptor, 30);

	sc->report_descriptor = kmalloc(sc->hid_descriptor.wReportDescLength,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	if (iichid_readreg_check(sc, sc->hid_descriptor.wReportDescRegister,
	    sc->report_descriptor, sc->hid_descriptor.wReportDescLength) != 0) {
		pci_set_powerstate(dev, PCI_POWERSTATE_D3);
		iic_free_resource(dev, sc->iicres);
		gpioint_free_resource(dev, sc->intres);
		kfree(sc->report_descriptor, M_DEVBUF);
		return (ENXIO);
	}
	device_printf(dev, "Report descriptor (%d bytes):\n",
	    sc->hid_descriptor.wReportDescLength);
	iichid_dump_bytes(sc->report_descriptor,
	    sc->hid_descriptor.wReportDescLength);

	sc->input_report = kmalloc(sc->hid_descriptor.wMaxInputLength,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	gpioint_establish_interrupt(sc->intres, iichid_intr, sc);

	iichid_setpower(sc, 1);

	if (iichid_reset(sc) != 0) {
		iichid_detach(dev);
		return (ENXIO);
	}

	iichid_setpower(sc, 0);
	sc->poweron = 0;

	if (hid_is_digitizer(sc->report_descriptor,
	    sc->hid_descriptor.wReportDescLength)) {
		device_printf(sc->dev, "I2C HID digitizer found\n");
		iichid_init_digitizer(sc);
	}

	sc->unit = device_get_unit(dev);
	if (sc->unit & 0x0400) {
		sc->devnode = make_dev(&iichid_ops, sc->unit,
		    UID_ROOT, GID_WHEEL, 0600, "iichid%d-%02x",
		    sc->unit >> 11, sc->unit % 1023);
	} else {
		sc->devnode = make_dev(&iichid_ops, sc->unit,
		    UID_ROOT, GID_WHEEL, 0600, "iichid%d", sc->unit);
	}
	sc->devnode->si_drv1 = sc;

	/*
	 * XXX We need a way to report the minimum/maximum x/y values of
	 *     the touchscreen to userspace
	 */

	return (0);
}

static int
iichid_detach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);

	if (sc->devnode) {
		dev_ops_remove_minor(&iichid_ops, sc->unit);
		devfs_assume_knotes(sc->devnode, &sc->kqinfo);
	}

	if (sc->intres != NULL)
		gpioint_release_interrupt(sc->intres);

	if (sc->poweron) {
		iichid_setpower(sc, 0);
		sc->poweron = 0;
	}

	pci_set_powerstate(dev, PCI_POWERSTATE_D3);

	if (sc->intres != NULL)
		gpioint_free_resource(dev, sc->intres);
	if (sc->iicres != NULL)
		iic_free_resource(dev, sc->iicres);

	if (sc->report_descriptor != NULL)
		kfree(sc->report_descriptor, M_DEVBUF);
	if (sc->input_report != NULL)
		kfree(sc->input_report, M_DEVBUF);

	return (0);
}

static device_method_t iichid_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, iichid_probe),
	DEVMETHOD(device_attach, iichid_attach),
	DEVMETHOD(device_detach, iichid_detach),

	DEVMETHOD_END
};

static driver_t iichid_driver = {
	"iichid",
	iichid_methods,
	sizeof(struct iichid_softc),
};
static devclass_t iichid_devclass;

DRIVER_MODULE(iichid, acpi, iichid_driver, iichid_devclass, NULL, NULL);
MODULE_DEPEND(iichid, acpi, 1, 1, 1);
MODULE_DEPEND(iichid, gpio_acpi, 1, 1, 1);
MODULE_DEPEND(iichid, smbacpi, 1, 1, 1);
MODULE_VERSION(iichid, 1);

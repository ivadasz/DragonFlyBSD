#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/event.h>
#include <sys/device.h>
#include <sys/uio.h>
#include <sys/vnode.h>
#include <sys/lock.h>
#include <sys/filio.h>
#include <sys/filedesc.h>
#include <sys/devfs.h>
#include <sys/signalvar.h>
#include <sys/flexfifo.h>

MALLOC_DEFINE(M_FLEXFIFO, "flexfifo", "Flexfifo FIFO device allocations");

struct flexfifo {
	cdev_t dev;
	int flags;
	u_int chunksz;
	u_int length;
	u_int start;
	u_int fill;
	u_int dropped;
	u_int pktlen;
	struct lock lk;
	struct kqinfo rkq;
	struct sigio *sigio;
	int async;
	int opened;
	void *arg;
	struct flexfifo_ops *ops;
	u_int8_t *mem;
	u_int8_t *temp;
};

static int
flexfifo_open(struct dev_open_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct flexfifo *fifo;
	int ret;

	if (dev->si_drv1 == NULL)
		return ENXIO;

	fifo = dev->si_drv1;
	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	if (dev->si_drv1 == NULL) {
		ret = ENXIO;
		goto done;
	}

	if (fifo->opened) {
		ret = EBUSY;
		goto done;
	}

	if (fifo->ops->evtopkt != NULL) {
		fifo->temp = kmalloc(fifo->pktlen, M_FLEXFIFO,
		    M_WAITOK | M_ZERO);
	} else {
		fifo->temp = NULL;
	}
	fifo->opened = 1;
	fifo->sigio = NULL;
	KKASSERT(fifo->mem == NULL);
	fifo->mem = kmalloc(fifo->chunksz * fifo->length, M_FLEXFIFO,
	    M_WAITOK | M_ZERO);

	if (fifo->ops->open != NULL)
		fifo->ops->open(fifo->arg);

	ret = 0;

done:
	lockmgr(&fifo->lk, LK_RELEASE);
	return ret;
}

static int
flexfifo_close(struct dev_close_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct flexfifo *fifo;
	int ret;

	if (dev->si_drv1 == NULL)
		return 0;

	fifo = dev->si_drv1;
	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	if (dev->si_drv1 == NULL) {
		ret = 0;
		goto done;
	}

	if (fifo->ops->close != NULL)
		fifo->ops->close(fifo->arg);

	if (fifo->mem != NULL) {
		kfree(fifo->mem, M_FLEXFIFO);
		fifo->mem = NULL;
	}
	if (fifo->temp != NULL) {
		kfree(fifo->temp, M_FLEXFIFO);
		fifo->temp = NULL;
	}

	funsetown(&fifo->sigio);
	fifo->opened = 0;
	fifo->async = 0;
	ret = 0;

done:
	lockmgr(&fifo->lk, LK_RELEASE);
	return ret;
}

static int
flexfifo_read(struct dev_read_args *ap)
{
	struct uio *uio = ap->a_uio;
	cdev_t dev = ap->a_head.a_dev;
	struct flexfifo *fifo;
	uint8_t *buf;
	u_int len;
	int error = 0, val, cnt = 0;

	if (dev->si_drv1 == NULL)
		return EIO;

	fifo = dev->si_drv1;
	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	if (dev->si_drv1 == NULL) {
		lockmgr(&fifo->lk, LK_RELEASE);
		return EIO;
	}
	buf = fifo->temp;
	if (fifo->ops->pktlen == NULL)
		len = fifo->pktlen;
	else
		len = fifo->ops->pktlen(fifo->arg);
	/*
	 * XXX If the read buffer is too small for even one packet, return a
	 *     partial packet, and buffer the rest of the packet for later.
	 */
	while (fifo->fill <= 0) {
		/* Buffer too small to fit a complete mouse packet */
		if (uio->uio_resid < len) {
			error = EIO;
			goto done;
		}
		if (ap->a_ioflag & IO_NDELAY) {
			error = EAGAIN;
			goto done;
		}
		error = lksleep(fifo, &fifo->lk, PCATCH, "fiforead", 0);
		if (error == EINTR || error == ERESTART) {
			goto done;
		}
		/* Output package size may have changed. */
		if (fifo->ops->pktlen != NULL)
			len = fifo->ops->pktlen(fifo->arg);
	}

	do {
		uint8_t *place;
		char *src;

		/* Buffer too small to fit a complete mouse packet */
		if (uio->uio_resid < len) {
			error = EIO;
			goto done;
		}
		place = fifo->mem + fifo->start * fifo->chunksz;
		KKASSERT(len <= fifo->pktlen);
		if (fifo->ops->evtopkt == NULL) {
			val = len;
			src = place;
		} else {
			val = fifo->ops->evtopkt(fifo->arg, place, buf);
			src = buf;
		}
		if (val > 0) {
			error = uiomove(src, val, uio);
			if (error != 0)
				goto done;
			cnt++;
		}
		fifo->fill--;
		fifo->start = (fifo->start + 1) % fifo->length;

		if (fifo->flags & FLEXFIFO_FLAG_SINGLEPKT)
			goto done;
	} while (fifo->fill > 0);

done:
	lockmgr(&fifo->lk, LK_RELEASE);
	if (cnt > 0 && error != EFAULT)
		return 0;
	return error;
}

static void
flexfifo_filter_detach(struct knote *kn)
{
	struct flexfifo *fifo = (void *)kn->kn_hook;

	knote_remove(&fifo->rkq.ki_note, kn);
}

static int
flexfifo_filter(struct knote *kn, long hint)
{
	struct flexfifo *fifo = (void *)kn->kn_hook;
	int ready = 0;

	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	if (fifo->fill > 0) {
		ready = 1;
		kn->kn_data = 0;
	}
	lockmgr(&fifo->lk, LK_RELEASE);

	return ready;
}

static struct filterops flexfifo_filtops =
    { FILTEROP_MPSAFE | FILTEROP_ISFD, NULL, flexfifo_filter_detach,
      flexfifo_filter };

static int
flexfifo_kqfilter(struct dev_kqfilter_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct flexfifo *fifo;
	struct knote *kn = ap->a_kn;

	if (dev->si_drv1 == NULL)
		return ENXIO;

	fifo = dev->si_drv1;
	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	if (dev->si_drv1 == NULL) {
		lockmgr(&fifo->lk, LK_RELEASE);
		return ENXIO;
	}

	ap->a_result = 0;

	switch (kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &flexfifo_filtops;
		kn->kn_hook = (caddr_t)fifo;
		knote_insert(&fifo->rkq.ki_note, kn);
		break;
	default:
		ap->a_result = EOPNOTSUPP;
		break;
	}
	lockmgr(&fifo->lk, LK_RELEASE);

	return (0);
}

static int
flexfifo_ioctl(struct dev_ioctl_args *ap)
{
	cdev_t dev = ap->a_head.a_dev;
	struct flexfifo *fifo;
	int ret = 0;

	if (dev->si_drv1 == NULL)
		return ENXIO;

	fifo = dev->si_drv1;
	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	if (dev->si_drv1 == NULL) {
		lockmgr(&fifo->lk, LK_RELEASE);
		return ENXIO;
	}

	switch (ap->a_cmd) {
	case FIOSETOWN:
		fsetown(*(int *)ap->a_data, &fifo->sigio);
		break;
	case FIOGETOWN:
		*(int *)ap->a_data = fgetown(&fifo->sigio);
		break;
	case FIOASYNC:
		if (*(int *)ap->a_data)
			fifo->async = 1;
		else
			fifo->async = 0;
		break;
	case FIONREAD:
		if (fifo->flags & FLEXFIFO_FLAG_SINGLEPKT) {
			ret = ENXIO;
			break;
		}
		if (fifo->fill == 0) {
			*(int *)ap->a_data = 0;
		} else if (fifo->ops->pktlen == NULL) {
			*(int *)ap->a_data = fifo->fill * fifo->pktlen;
		} else {
			*(int *)ap->a_data =
			    fifo->fill * fifo->ops->pktlen(fifo->arg);
		}
		break;
	default:
		if (fifo->ops->ioctl == NULL) {
			ret = ENXIO;
			break;
		}
		ret = fifo->ops->ioctl(fifo->arg, ap->a_data, ap->a_cmd,
		    ap->a_fflag);
		break;
	}

	lockmgr(&fifo->lk, LK_RELEASE);
	return ret;
}

static struct dev_ops flexfifo_ops = {
	{ "flexfifo", 0, D_MPSAFE },
	.d_open =	flexfifo_open,
	.d_close =	flexfifo_close,
	.d_read =	flexfifo_read,
	.d_ioctl =	flexfifo_ioctl,
	.d_kqfilter =	flexfifo_kqfilter,
};

struct flexfifo *
flexfifo_create(u_int chunk, u_int count, struct flexfifo_ops *ops, int minor,
    const char *name, void *arg, u_int max_pktlen)
{
	struct flexfifo *f;

	f = kmalloc(sizeof(struct flexfifo), M_FLEXFIFO, M_WAITOK | M_ZERO);
	f->flags = 0;
	f->chunksz = chunk;
	f->length = count;
	f->start = 0;
	f->fill = 0;
	f->dropped = 0;
	f->pktlen = max_pktlen;
	f->sigio = NULL;
	f->async = 0;
	f->ops = ops;
	f->arg = arg;
	f->mem = NULL;
	f->temp = NULL;
	lockinit(&f->lk, "flexfifo", 0, LK_CANRECURSE);
	f->dev = make_dev(&flexfifo_ops, minor, UID_ROOT, GID_WHEEL, 0600,
	    name);
	f->dev->si_drv1 = f;
	return f;
}

void
flexfifo_setflags(struct flexfifo *fifo, int flags)
{
	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	fifo->flags = flags;
	lockmgr(&fifo->lk, LK_RELEASE);
}

cdev_t
flexfifo_get_cdev(struct flexfifo *fifo)
{
	return fifo->dev;
}

void
flexfifo_destroy(struct flexfifo *fifo)
{
	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	fifo->dev->si_drv1 = NULL;
	lockmgr(&fifo->lk, LK_RELEASE);
	destroy_dev(fifo->dev);
	devfs_config();
	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	funsetown(&fifo->sigio);
	lockmgr(&fifo->lk, LK_RELEASE);
	wakeup(fifo);
	devfs_assume_knotes(fifo->dev, &fifo->rkq);
	if (fifo->mem != NULL)
		kfree(fifo->mem, M_FLEXFIFO);
	if (fifo->temp != NULL)
		kfree(fifo->temp, M_FLEXFIFO);
	/*
	 * XXX This still feels slightly racy. Some thread might be just
	 *     before the lockmgr call in a flexfifo_ops method.
	 */
	kfree(fifo, M_FLEXFIFO);
}

static void
flexfifo_wakeup(struct flexfifo *fifo)
{
	wakeup(fifo);
	KNOTE(&fifo->rkq.ki_note, 0);
}

/* This overwrites the oldest entry, when the fifo is full. */
void
flexfifo_enqueue_ring(struct flexfifo *fifo, uint8_t *chunk)
{
	uint8_t *place;
	u_int fill;

	lockmgr(&fifo->lk, LK_EXCLUSIVE);
	if (fifo->opened == 0) {
		lockmgr(&fifo->lk, LK_RELEASE);
		return;
	}
	if (fifo->fill >= fifo->length) {
		fifo->fill = fifo->length;
		place = fifo->mem + fifo->start * fifo->chunksz;
		/* XXX Enforcing powerof2 lengths could improve performance. */
		fifo->start = (fifo->start + 1) % fifo->length;
		fifo->dropped++;
	} else {
		u_int pos;

		pos = (fifo->start + fifo->fill) % fifo->length;
		place = fifo->mem + pos * fifo->chunksz;
		fifo->fill++;
	}
	memcpy(place, chunk, fifo->chunksz);
	fill = fifo->fill;
	if (fill == 1 && fifo->async)
		pgsigio(fifo->sigio, SIGIO, 0);
	lockmgr(&fifo->lk, LK_RELEASE);
	if (fill == 1)
		flexfifo_wakeup(fifo);
}

uint8_t *
flexfifo_peek_ring(struct flexfifo *fifo)
{
	if (fifo->fill == 0)
		return NULL;

	return fifo->mem + fifo->start * fifo->chunksz;
}

/*-
 * Copyright (c) 2014 Imre Vadász <imre@vdsz.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer as
 *    the first lines of this file unmodified.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "opt_txtdev.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/globaldata.h>
#include <sys/mutex2.h>
#include <sys/proc.h>
#include <sys/priv.h>
#include <sys/kernel.h>
#include <sys/devfs.h>
#include <sys/stat.h>
//#include <sys/txtdevio.h>

#include "txtdev.h"

/*
 * txtdev output
 */
struct txtdev_output {
	int				flags;
	void				*cookie;
	struct txtdev_sw		*sw;
	txtdev_release_cb		*releasecb;
	void				*conscookie;
	int				unit;
	cdev_t				devnode;
	SLIST_ENTRY(txtdev_output)	next;
};

static SLIST_HEAD(, txtdev_output) txtdev_lst = SLIST_HEAD_INITIALIZER(txtdev_lst);

static struct txtdev_output boot_output;
static struct mtx txt_mtx = MTX_INITIALIZER;

static d_open_t		txtdev_open;
static d_close_t	txtdev_close;
static d_read_t		txtdev_read;
static d_write_t	txtdev_write;
static d_ioctl_t	txtdev_ioctl;
static d_mmap_t		txtdev_mmap;
static d_mmap_single_t		txtdev_mmap_single;

static struct dev_ops txtdev_cdevsw = {
	{ "txtdev", 0, D_MPSAFE },
	.d_open =	txtdev_open,
	.d_close =	txtdev_close,
	.d_read =	txtdev_read,
	.d_write =	txtdev_write,
	.d_ioctl =	txtdev_ioctl,
	.d_mmap =	txtdev_mmap,
	.d_mmap_single = txtdev_mmap_single,
};

/* XXX needs a corresponding unregister_txtdev method */
int
register_txtdev(void *cookie, struct txtdev_sw *sw, int how)
{
	struct txtdev_output *out, *np;
	int i = 0;

	if (sw == NULL)
		return 1;

	mtx_spinlock(&txt_mtx);

	SLIST_FOREACH(np, &txtdev_lst, next) {
		if (np->unit != i)
			break;
		i++;
	}

	/* allocate new output */
	if (boot_output.sw == NULL) {
		out = &boot_output;
	} else {
		out = kmalloc(sizeof(struct txtdev_output), M_DEVBUF,
		    M_ZERO | M_WAITOK);
	}

	/* initialize output properties */
	out->flags = how;
	out->cookie = cookie;
	out->sw = sw;
	out->releasecb = NULL;
	out->conscookie = NULL;
	out->unit = i;
	out->devnode = NULL;

	/* insert new output into the list */
	if (SLIST_EMPTY(&txtdev_lst) || SLIST_FIRST(&txtdev_lst)->unit > i) {
		SLIST_INSERT_HEAD(&txtdev_lst, out, next);
	} else {
		SLIST_FOREACH(np, &txtdev_lst, next) {
			if (SLIST_NEXT(np, next) == NULL ||
			    SLIST_NEXT(np, next)->unit > i)
				break;
		}
		SLIST_INSERT_AFTER(np, out, next);
	}

	/* mark VGA output as dead */
	if (how & TXTDEV_REPLACE_VGA) {
		SLIST_FOREACH(np, &txtdev_lst, next) {
			if (np == out)
				continue;
			if (np->flags & TXTDEV_IS_DEAD)
				continue;
			if (np->flags & TXTDEV_IS_VGA) {
				np->flags |= TXTDEV_IS_DEAD;
				destroy_dev(np->devnode);
				np->devnode = NULL;
				if (np->releasecb != NULL) {
					np->releasecb(np->conscookie,
					    np->cookie);
				}
				break;
			}
		}
	}
	out->devnode = make_dev(&txtdev_cdevsw, out->unit, 0, 0,
	    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, "txtdev/output%d", out->unit);

	mtx_spinunlock(&txt_mtx);
	return 0;
}

/* Check whether a txtdev is available */
int
available_txtdev(void)
{
	struct txtdev_output *np;
	int val = 0;

	mtx_spinlock(&txt_mtx);

	SLIST_FOREACH(np, &txtdev_lst, next) {
		if (!(np->flags & TXTDEV_IS_DEAD)) {
			val = 1;
			break;
		}
	}

	mtx_spinunlock(&txt_mtx);
	return val;
}

/* try to acquire a txtdev, returns 0 on success */
int
acquire_txtdev(void **cookie, struct txtdev_sw **sw, txtdev_release_cb *cb,
    void *cc)
{
	struct txtdev_output *np, *chosen;

	if (cb == NULL)
		return 1;

	mtx_spinlock(&txt_mtx);

	if (SLIST_EMPTY(&txtdev_lst))
		return 1;

	chosen = NULL;
	SLIST_FOREACH(np, &txtdev_lst, next) {
		if (np->flags & TXTDEV_IS_DEAD)
			continue;
		if (np->releasecb == NULL) {
			chosen = np;
			break;
		}
	}

	if (chosen == NULL) {
		mtx_spinunlock(&txt_mtx);
		return 1;
	}

	np->releasecb = cb;
	np->conscookie = cc;
	*sw = np->sw;
	*cookie = np->cookie;

	mtx_spinunlock(&txt_mtx);
	return 0;
}

int
release_txtdev(void *cookie, struct txtdev_sw *sw)
{
	struct txtdev_output *np;
	int val = 1;

	mtx_spinlock(&txt_mtx);
	SLIST_FOREACH(np, &txtdev_lst, next) {
		if (np->cookie == cookie && np->sw == sw) {
			np->releasecb = NULL;
			np->conscookie = NULL;
			val = 0;
			break;
		}
	}

	mtx_spinunlock(&txt_mtx);
	return val;
}

int
txtdev_open(struct dev_open_args *ap)
{
	return (ENODEV);
}

int
txtdev_close(struct dev_close_args *ap)
{
	return (ENODEV);
}

int
txtdev_read(struct dev_read_args *ap)
{
	return (ENODEV);
}

int
txtdev_write(struct dev_write_args *ap)
{
	return (ENODEV);
}

int
txtdev_ioctl(struct dev_ioctl_args *ap)
{
	return (ENODEV);
}

int
txtdev_mmap(struct dev_mmap_args *ap)
{
	return (ENODEV);
}
int
txtdev_mmap_single(struct dev_mmap_single_args *ap)
{
	return (ENODEV);
}

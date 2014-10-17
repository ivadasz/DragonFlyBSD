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

//#include "opt_txtdev.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/globaldata.h>
#include <sys/mutex2.h>
#include <sys/proc.h>
#include <sys/priv.h>
#include <sys/kernel.h>

#include "txtdev.h"

static int owned = 0;
static int myflags = 0;
static void *mycookie = NULL;
static struct txtdev_sw *mysw = NULL;
static txtdev_newdev_cb *newdevcb = NULL;
static void *conscookie = NULL;
static struct mtx txt_mtx = MTX_INITIALIZER;

/* XXX needs a corresponding unregister_txtdev method */
int
register_txtdev(void *cookie, struct txtdev_sw *sw, int how)
{
	void *oldcookie = mycookie;
	int replacing = 0;

	if (sw == NULL)
		return 1;

	mtx_spinlock(&txt_mtx);

//	/* Only allow replacing early attachements */
//	if ((myflags & TXTDEV_IS_EARLY) != 0) {
		if ((myflags & TXTDEV_IS_VGA) != 0 &&
		    (how & TXTDEV_REPLACE_VGA) != 0) {
			mycookie = NULL;
			mysw = NULL;
			myflags = 0;
			replacing = 1;
		}
//	}

	if (mycookie != NULL || mysw != NULL) {
		mtx_spinunlock(&txt_mtx);
		return 1;
	}

	mycookie = cookie;
	mysw = sw;
	myflags = how;

	/* Initialize */
//	mysw->setcursor(mycookie, -1);
//	mysw->setcurmode(mycookie, TXTDEV_CURSOR_BLINK);

	/* Register with virtual terminal */
	if (replacing && owned) {
		KASSERT(newdevcb != NULL, ("no way to replace txtdev"));

		/*
		 * We expect the console to call release_txtdev on the old
		 * txtdev and then retrieve the new txtdev with acquire_txtdev.
		 */
		newdevcb(conscookie, oldcookie);
	} else if (!owned && newdevcb != NULL) {
		/*
		 * We expect the console to call acquire_txtdev to acquire this
		 * txtdev.
		 */
		newdevcb(conscookie, NULL);
	}
	mtx_spinunlock(&txt_mtx);

	return 0;
}

/* use acquire_txtdev(NULL,NULL,NULL) to test whether a txtdev is available */
int
acquire_txtdev(void **cookie, struct txtdev_sw **sw, txtdev_newdev_cb *cb,
    void *cc)
{
	if (sw != NULL && cookie != NULL && cb == NULL)
		return 1;

	mtx_spinlock(&txt_mtx);

	if (!owned && mysw != NULL) {
		if (sw != NULL && cookie != NULL) {
			*sw = mysw;
			*cookie = mycookie;
			newdevcb = cb;
			conscookie = cc;
			owned = 1;
		}
		mtx_spinunlock(&txt_mtx);
		return 0;
	}

	*sw = NULL;
	*cookie = NULL;
	if (cb != NULL && newdevcb == NULL) {
		newdevcb = cb;
		conscookie = cc;
		mtx_spinunlock(&txt_mtx);
		return 2;
	}

	mtx_spinunlock(&txt_mtx);
	return 1;
}

int
release_txtdev(void *cookie, struct txtdev_sw *sw)
{
	mtx_spinlock(&txt_mtx);
	if (owned) {
		owned = 0;
		newdevcb = NULL;
		conscookie = NULL;
		mtx_spinunlock(&txt_mtx);
		return 0;
	}

	mtx_spinunlock(&txt_mtx);
	return 1;
}

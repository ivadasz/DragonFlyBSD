/*-
 * (MPSAFE)
 *
 * Copyright (c) 1999 Kazutaka YOKOTA <yokota@zodiac.mech.utsunomiya-u.ac.jp>
 * All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Sascha Wildner <saw@online.de>
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
 *
 * $FreeBSD: src/sys/dev/syscons/scvgarndr.c,v 1.5.2.3 2001/07/28 12:51:47 yokota Exp $
 */

#include "opt_syscons.h"
#include "opt_vga.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/thread.h>
#include <sys/thread2.h>

#include <machine/console.h>

#include <dev/video/fb/fbreg.h>
#include <dev/video/fb/vgareg.h>
#include "syscons.h"
#include "../txt/txtdev.h"

static vr_draw_t		vga_txtdraw;
static vr_draw_cursor_t		vga_txtcursor;

#if 0
static void			vga_nop(scr_stat *scp, ...);
#endif

static sc_rndr_sw_t txtrndrsw = {
	vga_txtdraw,	
	vga_txtcursor,
};
RENDERER(vga, V_INFO_MM_TEXT, txtrndrsw, vga_set);

RENDERER_MODULE(vga, vga_set);

#if 0
static void
vga_nop(scr_stat *scp, ...)
{
}
#endif

/* text mode renderer */

static void
vga_txtdraw(scr_stat *scp, int from, int count, int flip)
{
	sc_softc_t *sc = (sc_softc_t *)scp->sc;
	uint16_t val;
	int a, c;

	if (from + count > scp->xsize*scp->ysize)
		count = scp->xsize*scp->ysize - from;

	if (flip) {
		for (; count-- > 0; ++from) {
			c = sc_vtb_getc(&scp->vtb, from);
			a = sc_vtb_geta(&scp->vtb, from);
			a = (a & 0x8800) | ((a & 0x7000) >> 4) 
				| ((a & 0x0700) << 4);
			if (sc->txtdevsw != NULL) {
				val = c | a;
				sc->txtdevsw->putchars(sc->txtdev_cookie,
				    from % scp->xsize, from / scp->xsize,
				    &val, 1);
			}
		}
	} else if (sc->txtdevsw != NULL) {
		sc->txtdevsw->putchars(sc->txtdev_cookie,
		    from % scp->xsize, from / scp->xsize,
		    &scp->vtb.vtb_buffer[from], count);
	}
}

static void
vga_txtcursor(scr_stat *scp, int at, int blink, int on)
{
	sc_softc_t *sc = (sc_softc_t *)scp->sc;
	int cursormode, col, row;

	if (blink) {
		scp->status |= VR_CURSOR_BLINK;
		cursormode = TXTDEV_CURSOR_HW;
	} else {
		scp->status &= ~VR_CURSOR_BLINK;
		cursormode = TXTDEV_CURSOR_CHAR;
	}

	if (on) {
		scp->status |= VR_CURSOR_ON;
		col = at % scp->xsize;
		row = at / scp->xsize;
	} else {
		col = -1;
		row = -1;
	}
	if (sc->txtdevsw != NULL) {
		sc->txtdevsw->setcursor(sc->txtdev_cookie,
		    col, row, cursormode);
	}
	if (!on)
		scp->status &= ~VR_CURSOR_ON;
}

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

#include <bus/isa/isareg.h>

static vr_draw_border_t		vga_txtborder;
static vr_draw_t		vga_txtdraw;
static vr_set_cursor_t		vga_txtcursor_shape;
static vr_draw_cursor_t		vga_txtcursor;
static vr_blink_cursor_t	vga_txtblink;

#if 0
static void			vga_nop(scr_stat *scp, ...);
#endif

static sc_rndr_sw_t txtrndrsw = {
	vga_txtborder,
	vga_txtdraw,	
	vga_txtcursor_shape,
	vga_txtcursor,
	vga_txtblink,
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
vga_txtborder(scr_stat *scp, int color)
{
#if 0
	(*vidsw[scp->sc->adapter]->set_border)(scp->sc->adp, color);
#endif
}

static void
vga_txtdraw(scr_stat *scp, int from, int count, int flip)
{
	sc_softc_t *sc = (sc_softc_t *)scp->sc;
//	uint16_t *p;
	uint16_t val;
	int c;
	int a;

	if (from + count > scp->xsize*scp->ysize)
		count = scp->xsize*scp->ysize - from;

	if (flip) {
		for (; count-- > 0; ++from) {
			c = sc_vtb_getc(&scp->vtb, from);
			a = sc_vtb_geta(&scp->vtb, from);
			a = (a & 0x8800) | ((a & 0x7000) >> 4) 
				| ((a & 0x0700) << 4);
//			p = sc_vtb_putchar(&scp->scr, p, c, a);
			if (sc->txtdevsw != NULL) {
				val = c | a;
				sc->txtdevsw->putchars(sc->txtdev_cookie,
				    from % 80, from / 80, &val, 1);
			}
		}
	} else {
		c = sc_vtb_getc(&scp->vtb, from);
		a = sc_vtb_geta(&scp->vtb, from);
//		sc_vtb_copy(&scp->vtb, from, &scp->scr, from, count);
		for (; count-- > 0; ++from) {
			val = c | a;
			sc->txtdevsw->putchars(sc->txtdev_cookie,
			    from % 80, from / 80, &val, 1);
		}
	}
}

static void 
vga_txtcursor_shape(scr_stat *scp, int blink)
{
#if 0
	(*vidsw[scp->sc->adapter]->set_hw_cursor_shape)(scp->sc->adp,
							0, 16, 16, blink);
#endif
}

#if 0
static void
draw_txtcharcursor(scr_stat *scp, int at, u_short c, u_short a, int flip)
{
	sc_softc_t *sc;

	sc = scp->sc;
	scp->cursor_saveunder_char = c;
	scp->cursor_saveunder_attr = a;

	{
		if ((a & 0x7000) == 0x7000) {
			a &= 0x8f00;
			if ((a & 0x0700) == 0)
				a |= 0x0700;
		} else {
			a |= 0x7000;
			if ((a & 0x0700) == 0x0700)
				a &= 0xf000;
		}
		if (flip)
			a = (a & 0x8800)
				| ((a & 0x7000) >> 4) | ((a & 0x0700) << 4);
		sc_vtb_putc(&scp->scr, at, c, a);
	}
}
#endif

static void
vga_txtcursor(scr_stat *scp, int at, int blink, int on, int flip)
{
#if 0
	video_adapter_t *adp;
	int cursor_attr;

	adp = scp->sc->adp;
	if (blink) {
		scp->status |= VR_CURSOR_BLINK;
		if (on) {
			scp->status |= VR_CURSOR_ON;
			(*vidsw[adp->va_index]->set_hw_cursor)(adp,
							       at%scp->xsize,
							       at/scp->xsize); 
		} else {
			if (scp->status & VR_CURSOR_ON)
				(*vidsw[adp->va_index]->set_hw_cursor)(adp,
								       -1, -1);
			scp->status &= ~VR_CURSOR_ON;
		}
	} else {
		scp->status &= ~VR_CURSOR_BLINK;
		if (on) {
			scp->status |= VR_CURSOR_ON;
			draw_txtcharcursor(scp, at,
					   sc_vtb_getc(&scp->scr, at),
					   sc_vtb_geta(&scp->scr, at),
					   flip);
		} else {
			cursor_attr = scp->cursor_saveunder_attr;
			if (flip)
				cursor_attr = (cursor_attr & 0x8800)
					| ((cursor_attr & 0x7000) >> 4)
					| ((cursor_attr & 0x0700) << 4);
			if (scp->status & VR_CURSOR_ON)
				sc_vtb_putc(&scp->scr, at,
					    scp->cursor_saveunder_char,
					    cursor_attr);
			scp->status &= ~VR_CURSOR_ON;
		}
	}
#endif
}

static void
vga_txtblink(scr_stat *scp, int at, int flip)
{
}

int sc_txtmouse_no_retrace_wait;

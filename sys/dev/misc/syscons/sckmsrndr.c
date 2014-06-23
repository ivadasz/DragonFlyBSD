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

#include "opt_syscons.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/thread.h>
#include <sys/thread2.h>

#include <machine/console.h>

#include "syscons.h"
#include "fbinfo.h"

#include <bus/isa/isareg.h>

static vr_draw_border_t		kms_txtborder;
static vr_draw_t		kms_txtdraw;
static vr_set_cursor_t		kms_txtcursor_shape;
static vr_draw_cursor_t		kms_txtcursor;
static vr_blink_cursor_t	kms_txtblink;
#ifndef SC_NO_CUTPASTE
static vr_draw_mouse_t		kms_txtmouse;
#else
#define	kms_txtmouse		(vr_draw_mouse_t *)kms_nop
#endif

#ifndef SC_NO_MODE_CHANGE
static vr_draw_border_t		kms_grborder;
#endif

#ifdef SC_NO_CUTPASTE
static void			kms_nop(scr_stat *scp, ...);
#endif

static sc_rndr_sw_t txtrndrsw = {
	kms_txtborder,
	kms_txtdraw,
	kms_txtcursor_shape,
	kms_txtcursor,
	kms_txtblink,
	kms_txtmouse,
};
RENDERER(kms, V_INFO_MM_TEXT, txtrndrsw, kms_set);

#ifndef SC_NO_MODE_CHANGE
static sc_rndr_sw_t grrndrsw = {
	kms_grborder,
	(vr_draw_t *)kms_nop,
	(vr_set_cursor_t *)kms_nop,
	(vr_draw_cursor_t *)kms_nop,
	(vr_blink_cursor_t *)kms_nop,
	(vr_draw_mouse_t *)kms_nop,
};
RENDERER(kms, V_INFO_MM_OTHER, grrndrsw, kms_set);
#endif /* SC_NO_MODE_CHANGE */

RENDERER_MODULE(kms, kms_set);

uint32_t colormap[16] = {
	0x00000000,	/* BLACK */
	0x00000080,	/* BLUE */
	0x00008000,	/* GREEN */
	0x00008080,	/* CYAN */
	0x00800000,	/* RED */
	0x00800080,	/* MAGENTA */
	0x00808000,	/* BROWN */
	0x00c0c0c0,	/* WHITE */
	0x00808080,	/* HIGHLIGHT BLACK */
	0x000000ff,	/* HIGHLIGHT BLUE */
	0x0000ff00,	/* HIGHLIGHT GREEN */
	0x0000ffff,	/* HIGHLIGHT CYAN */
	0x00ff0000,	/* HIGHLIGHT RED */
	0x00ff00ff,	/* HIGHLIGHT MAGENTA */
	0x00ffff00,	/* HIGHLIGHT BROWN */
	0x00ffffff,	/* HIGHLIGHT WHITE */
};

#ifdef SC_NO_CUTPASTE
static void
kms_nop(scr_stat *scp, ...)
{
}
#endif

/* text mode renderer */

static void
kms_txtborder(scr_stat *scp, int color)
{
}

static void
draw_glyphs32(scr_stat *scp, int from, int count, int flip, int reverse)
{
	sc_softc_t *sc = scp->sc;
	uint32_t *fb = (uint32_t *)sc->fbi->vaddr;
	uint16_t width = sc->fbi->width;
//	uint16_t stride = sc->fbi->stride;
	uint16_t height = sc->fbi->height;
	u_char *font = sc->fbfont;
	u_char *glyph;
	short fnwidth = sc->fbfontwidth;
	short fnstride = sc->fbfontstride;
	short fnheight = sc->fbfontheight;
	int col, row;
	int i, j, k;
	uint16_t idx;
	uint8_t glyphidx;
	uint8_t attr;
	uint32_t bg;
	uint32_t fg;

	if (from + count > scp->xsize * scp->ysize)
		count = scp->xsize * scp->ysize - from;

	for (i = from; i < from + count; i++) {
		idx = scp->vtb.vtb_buffer[i];
		glyphidx = idx & 0xff;
		attr = (idx >> 8) & 0xff;
		if (flip) {
			attr = (attr & 0x88) | ((attr & 0x70) >> 4)
			        | ((attr & 0x07) << 4);
		}
		fg = colormap[attr & 0xf];
		bg = colormap[(attr >> 4) & 0xf];
		if (reverse) {
			uint32_t tmp = bg;
			bg = fg;
			fg = tmp;
		}

		if (glyphidx < sc->fbfontstart)
			continue;
		glyph = &font[(glyphidx - sc->fbfontstart) * fnstride * fnheight];
		col = i % scp->xsize;
		row = i / scp->xsize;
		if (col * fnwidth > width || row * fnheight > height)
			continue;
		for (j = 0; j < fnheight; j++) {
			for (k = 0; k < fnwidth; k++) {
				fb[(row*fnheight+j)*width + col*fnwidth + k] =
				    ((glyph[j * fnstride + k/8] >>
				    (7 - (k%8))) & 1) ? fg : bg;
			}
		}
	}
}

static void
kms_txtdraw(scr_stat *scp, int from, int count, int flip)
{
	draw_glyphs32(scp, from, count, flip, 0);
}

static void
kms_txtcursor_shape(scr_stat *scp, int base, int height, int blink)
{
	if (base < 0 || base >= scp->sc->fbfontheight)
		return;

#if notyet
	/* the caller may set height <= 0 in order to disable the cursor */
#if 0
	scp->cursor_base = base;
	scp->cursor_height = height;
#endif
	(*vidsw[scp->sc->adapter]->set_hw_cursor_shape)(scp->sc->adp,
							base, height,
							scp->font_size, blink);
#endif
}

static void
kms_txtcursor(scr_stat *scp, int at, int blink, int on, int flip)
{
	if (on)
		draw_glyphs32(scp, at, 1, flip, 1);
	else
		draw_glyphs32(scp, at, 1, flip, 0);
}

static void
kms_txtblink(scr_stat *scp, int at, int flip)
{
}

#ifndef SC_NO_CUTPASTE

static void
draw_txtmouse(scr_stat *scp, int x, int y)
{
}

static void
remove_txtmouse(scr_stat *scp, int x, int y)
{
}

static void
kms_txtmouse(scr_stat *scp, int x, int y, int on)
{
	if (on)
		draw_txtmouse(scp, x, y);
	else
		remove_txtmouse(scp, x, y);
}

#endif /* SC_NO_CUTPASTE */

#ifndef SC_NO_MODE_CHANGE

/* graphics mode renderer */

static void
kms_grborder(scr_stat *scp, int color)
{
}

#endif

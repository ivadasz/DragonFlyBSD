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

static inline void
idxtofgbg(uint16_t idx, int flip, uint32_t *bg, uint32_t *fg, int reverse)
{
	uint32_t mybg, myfg;
	uint8_t attr;

	attr = (idx >> 8) & 0xff;
	if (flip) {
		attr = (attr & 0x88) | ((attr & 0x70) >> 4)
		        | ((attr & 0x07) << 4);
	}
	myfg = colormap[attr & 0xf];
	mybg = colormap[(attr >> 4) & 0xf];
	if (reverse) {
		*bg = myfg;
		*fg = mybg;
	} else {
		*bg = mybg;
		*fg = myfg;
	}
}

/* startcol and count can be arbitrary */
static void
put_glyphs32_single(scr_stat *scp, uint32_t *fb, int row, int startcol, int count, int flip,
    int reverse)
{
	sc_softc_t *sc = scp->sc;
	uint32_t *pos, *line;
	uint16_t width = sc->fbi->width;
//	uint16_t stride = sc->fbi->stride;
	uint16_t height = sc->fbi->height;
	u_char *font = sc->fbfont;
	u_char *glyph;
	short fnwidth = sc->fbfontwidth;
	short fnstride = sc->fbfontstride;
	short fnheight = sc->fbfontheight;
	int endcol = startcol + count - 1;
	int i, j, k, at;
	uint16_t idx;
	uint8_t glyphidx;
	uint32_t bg, fg;

	if ((row+1) * fnheight > height)
		return;
	if ((startcol + count) * fnwidth > width)
		endcol = width / fnwidth - 1;

	line = &fb[row * fnheight * width];
	at = scp->xsize * row + startcol;
	for (i = startcol; i <= endcol; i++, at++) {
		if (sc_shadow_changed(scp, scp->vtb.vtb_buffer, at, 1) == 0)
			continue;
		pos = &line[i * fnwidth];
		idx = scp->vtb.vtb_buffer[at];
		glyphidx = idx & 0xff;
		idxtofgbg(idx, flip, &bg, &fg, reverse);

		if (glyphidx < sc->fbfontstart) {
			if (glyphidx == 0 && sc->fbfontstart <= ' ')
				glyphidx = ' ';
			else
				glyphidx = sc->fbfontstart;
		}
		glyph = &font[(glyphidx - sc->fbfontstart) * fnstride * fnheight];
		for (j = 0; j < fnheight; j++) {
			for (k = 0; k < fnwidth; k++) {
				pos[j * width + k] =
				    ((glyph[j * fnstride + k/8] >>
				    (7 - (k%8))) & 1) ? fg : bg;
			}
		}
	}
}

/* startcol and count must be multiples of 2 */
/* For now assume that each line starts on a cacheline boundary */
static void
put_glyphs32_dual(scr_stat *scp, uint32_t *fb, int row, int startcol, int count, int flip,
    int reverse)
{
	sc_softc_t *sc = scp->sc;
	uint32_t *pos, *line;
	uint16_t width = sc->fbi->width;
//	uint16_t stride = sc->fbi->stride;
	uint16_t height = sc->fbi->height;
	u_char *font = sc->fbfont;
	u_char *glyph[2];
	short fnwidth = sc->fbfontwidth;
	short fnstride = sc->fbfontstride;
	short fnheight = sc->fbfontheight;
	int endcol = startcol + count - 1;
	int i, j, k, at;
	uint16_t idx[2];
	uint8_t glyphidx[2];
	uint32_t bg[2], fg[2];

	if ((row + 1) * fnheight > height)
		return;
	if ((startcol + count) * fnwidth > width)
		endcol = width / fnwidth - 1;

	line = &fb[row * fnheight * width];
	at = scp->xsize * row + startcol;
	for (i = startcol; i <= endcol; i+=2, at+=2) {
		if (sc_shadow_changed(scp, scp->vtb.vtb_buffer, at, 2) == 0)
			continue;
		pos = &line[i * fnwidth];
		idx[0] = scp->vtb.vtb_buffer[at];
		idx[1] = scp->vtb.vtb_buffer[at+1];
		glyphidx[0] = idx[0] & 0xff;
		glyphidx[1] = idx[1] & 0xff;
		idxtofgbg(idx[0], flip, &bg[0], &fg[0], reverse);
		idxtofgbg(idx[1], flip, &bg[1], &fg[1], reverse);

		if (glyphidx[0] < sc->fbfontstart) {
			if (glyphidx[0] == 0 && sc->fbfontstart <= ' ')
				glyphidx[0] = ' ';
			else
				glyphidx[0] = sc->fbfontstart;
		}
		if (glyphidx[1] < sc->fbfontstart) {
			if (glyphidx[1] == 0 && sc->fbfontstart <= ' ')
				glyphidx[1] = ' ';
			else
				glyphidx[1] = sc->fbfontstart;
		}
		glyph[0] = &font[(glyphidx[0] - sc->fbfontstart) * fnstride * fnheight];
		glyph[1] = &font[(glyphidx[1] - sc->fbfontstart) * fnstride * fnheight];
		for (j = 0; j < fnheight; j++) {
			for (k = 0; k < fnwidth; k++) {
				pos[j * width + k] =
				    ((glyph[0][j * fnstride + k/8] >>
				    (7 - (k%8))) & 1) ? fg[0] : bg[0];
			}
			for (k = 0; k < fnwidth; k++) {
				pos[j * width + k + fnwidth] =
				    ((glyph[1][j * fnstride + k/8] >>
				    (7 - (k%8))) & 1) ? fg[1] : bg[1];
			}
		}
	}
}

static uint32_t monomap[256*8];
static int monomap_initted = 0;

static void
initmonomap(void)
{
	uint32_t val;
	int i;

	for (i = 0; i < 256; i++) {
		val = i;
		monomap[i*8+0] = ((val >> 7) & 1) ? 0xffffffff : 0x00000000;
		monomap[i*8+1] = ((val >> 6) & 1) ? 0xffffffff : 0x00000000;
		monomap[i*8+2] = ((val >> 5) & 1) ? 0xffffffff : 0x00000000;
		monomap[i*8+3] = ((val >> 4) & 1) ? 0xffffffff : 0x00000000;
		monomap[i*8+4] = ((val >> 3) & 1) ? 0xffffffff : 0x00000000;
		monomap[i*8+5] = ((val >> 2) & 1) ? 0xffffffff : 0x00000000;
		monomap[i*8+6] = ((val >> 1) & 1) ? 0xffffffff : 0x00000000;
		monomap[i*8+7] = ((val >> 0) & 1) ? 0xffffffff : 0x00000000;
	}
	monomap_initted = 1;
}

/* startcol and count must be multiples of 4 */
/* For now assume that each line starts on a cacheline boundary */
/* Assumes that sc->fbfontwidth is a multiple of 2 */
static void
put_glyphs32_quad(scr_stat *scp, uint32_t *fb, int row, int startcol, int count, int flip,
    int reverse)
{
	sc_softc_t *sc = scp->sc;
	uint32_t *pos, *line;
	uint16_t width = sc->fbi->width;
//	uint16_t stride = sc->fbi->stride;
	uint16_t height = sc->fbi->height;
	u_char *font = sc->fbfont;
	u_char *glyph[4];
	short fnwidth = sc->fbfontwidth;
	short fnstride = sc->fbfontstride;
	short fnheight = sc->fbfontheight;
	int endcol = startcol + count - 1;
	int i, j, k, l, m, at;
	uint16_t idx[4];
	uint8_t glyphidx[4];
	uint32_t bg[4], fg[4];
	uint64_t mybg[4], myfg[4];
	uint32_t *map;
	uint64_t val;

	if ((row+1) * fnheight > height)
		return;
	if ((startcol + count) * fnwidth > width)
		endcol = width / fnwidth - 1;

	line = &fb[row * fnheight * width];
	at = scp->xsize * row + startcol;
	for (i = startcol; i <= endcol; i+=4, at+=4) {
		if (sc_shadow_changed(scp, scp->vtb.vtb_buffer, at, 4) == 0)
			continue;
		pos = &line[i * fnwidth];
		idx[0] = scp->vtb.vtb_buffer[at];
		idx[1] = scp->vtb.vtb_buffer[at+1];
		idx[2] = scp->vtb.vtb_buffer[at+2];
		idx[3] = scp->vtb.vtb_buffer[at+3];
		glyphidx[0] = idx[0] & 0xff;
		glyphidx[1] = idx[1] & 0xff;
		glyphidx[2] = idx[2] & 0xff;
		glyphidx[3] = idx[3] & 0xff;
		if (glyphidx[0] < sc->fbfontstart) {
			if (glyphidx[0] == 0 && sc->fbfontstart <= ' ')
				glyphidx[0] = ' ';
			else
				glyphidx[0] = sc->fbfontstart;
		}
		if (glyphidx[1] < sc->fbfontstart) {
			if (glyphidx[1] == 0 && sc->fbfontstart <= ' ')
				glyphidx[1] = ' ';
			else
				glyphidx[1] = sc->fbfontstart;
		}
		if (glyphidx[2] < sc->fbfontstart) {
			if (glyphidx[2] == 0 && sc->fbfontstart <= ' ')
				glyphidx[2] = ' ';
			else
				glyphidx[2] = sc->fbfontstart;
		}
		if (glyphidx[3] < sc->fbfontstart) {
			if (glyphidx[3] == 0 && sc->fbfontstart <= ' ')
				glyphidx[3] = ' ';
			else
				glyphidx[3] = sc->fbfontstart;
		}

		idxtofgbg(idx[0], flip, &bg[0], &fg[0], reverse);
		idxtofgbg(idx[1], flip, &bg[1], &fg[1], reverse);
		idxtofgbg(idx[2], flip, &bg[2], &fg[2], reverse);
		idxtofgbg(idx[3], flip, &bg[3], &fg[3], reverse);
		mybg[0] = bg[0] | (((uint64_t)bg[0]) << 32);
		mybg[1] = bg[1] | (((uint64_t)bg[1]) << 32);
		mybg[2] = bg[2] | (((uint64_t)bg[2]) << 32);
		mybg[3] = bg[3] | (((uint64_t)bg[3]) << 32);
		myfg[0] = fg[0] | (((uint64_t)fg[0]) << 32);
		myfg[1] = fg[1] | (((uint64_t)fg[1]) << 32);
		myfg[2] = fg[2] | (((uint64_t)fg[2]) << 32);
		myfg[3] = fg[3] | (((uint64_t)fg[3]) << 32);

		glyph[0] = &font[(glyphidx[0] - sc->fbfontstart) * fnstride * fnheight];
		glyph[1] = &font[(glyphidx[1] - sc->fbfontstart) * fnstride * fnheight];
		glyph[2] = &font[(glyphidx[2] - sc->fbfontstart) * fnstride * fnheight];
		glyph[3] = &font[(glyphidx[3] - sc->fbfontstart) * fnstride * fnheight];
		for (j = 0; j < fnheight; j++) {
			m = 0;
			for (k = 0; k < fnstride; k++) {
				map = &monomap[glyph[0][j * fnstride + k]*8];
				int q = MIN(8, fnwidth - m);
				for (l = 0; l < q; l+=2) {
					val = *(uint64_t *)&map[l];
					val = (val & myfg[0]) | ((~val) & mybg[0]);
					*(uint64_t *)&pos[j*width+m] = val;
					m+=2;
				}
			}
			m = 0;
			for (k = 0; k < fnstride; k++) {
				map = &monomap[glyph[1][j * fnstride + k]*8];
				int q = MIN(8, fnwidth - m);
				for (l = 0; l < q; l+=2) {
					val = *(uint64_t *)&map[l];
					val = (val & myfg[1]) | ((~val) & mybg[1]);
					*(uint64_t *)&pos[j*width+m+fnwidth] = val;
					m+=2;
				}
			}
			m = 0;
			for (k = 0; k < fnstride; k++) {
				map = &monomap[glyph[2][j * fnstride + k]*8];
				int q = MIN(8, fnwidth - m);
				for (l = 0; l < q; l+=2) {
					val = *(uint64_t *)&map[l];
					val = (val & myfg[2]) | ((~val) & mybg[2]);
					*(uint64_t *)&pos[j*width+m+2*fnwidth] = val;
					m+=2;
				}
			}
			m = 0;
			for (k = 0; k < fnstride; k++) {
				map = &monomap[glyph[3][j * fnstride + k]*8];
				int q = MIN(8, fnwidth - m);
				for (l = 0; l < q; l+=2) {
					val = *(uint64_t *)&map[l];
					val = (val & myfg[3]) | ((~val) & mybg[3]);
					*(uint64_t *)&pos[j*width+m+3*fnwidth] = val;
					m+=2;
				}
			}
#if 0
			for (k = 0; k < fnwidth; k++) {
				pos[j * width + k] =
				    ((glyph[0][j * fnstride + k/8] >>
				    (7 - (k%8))) & 1) ? fg[0] : bg[0];
			}
			for (k = 0; k < fnwidth; k++) {
				pos[j * width + k + fnwidth] =
				    ((glyph[1][j * fnstride + k/8] >>
				    (7 - (k%8))) & 1) ? fg[1] : bg[1];
			}
			for (k = 0; k < fnwidth; k++) {
				pos[j * width + k + 2*fnwidth] =
				    ((glyph[2][j * fnstride + k/8] >>
				    (7 - (k%8))) & 1) ? fg[2] : bg[2];
			}
			for (k = 0; k < fnwidth; k++) {
				pos[j * width + k + 3*fnwidth] =
				    ((glyph[3][j * fnstride + k/8] >>
				    (7 - (k%8))) & 1) ? fg[3] : bg[3];
			}
#endif
		}
	}
}

static void
draw_glyphline32(scr_stat *scp, int row, int startcol, int count, int flip,
    int reverse)
{
	sc_softc_t *sc = scp->sc;
	uint32_t *fb = (uint32_t *)sc->fbi->vaddr;
	short fnwidth = sc->fbfontwidth;
	int i;

//#if 0
	/* safety checks */
	if (startcol + count > scp->xsize)
		count = scp->xsize - startcol;

	if (count < 1)
		return;
//#endif

	if (fnwidth % 16 == 0) {
		put_glyphs32_single(scp, fb, row, startcol, count, flip, reverse);
	} else if ((fnwidth * 2) % 16 == 0) {
		i = startcol;
		while (i < startcol + count &&
		    ((i & 1) || (startcol + count - i < 4))) {
			put_glyphs32_single(scp, fb, row, i, 1, flip, reverse);
			i++;
		}
		while (i + 2 <= startcol + count) {
			int step = (startcol + count - i) & ~1;
			if (step < 1)
				break;
			put_glyphs32_dual(scp, fb, row, i, step, flip, reverse);
			i += step;
		}
		if (i < startcol + count)
			put_glyphs32_single(scp, fb, row, i, startcol + count - i, flip, reverse);
	} else {
		i = startcol;
		while (i < startcol + count &&
		    ((i & 3) || (startcol + count - i < 4))) {
			put_glyphs32_single(scp, fb, row, i, 1, flip, reverse);
			i++;
		}
		while (i + 4 <= startcol + count) {
			int step = (startcol + count - i) & (~3);
			if (step < 1)
				break;
			put_glyphs32_quad(scp, fb, row, i, step, flip, reverse);
			i += step;
		}
		if (i < startcol + count)
			put_glyphs32_single(scp, fb, row, i, startcol + count - i, flip, reverse);
	}
}

static void
draw_glyphs32(scr_stat *scp, int from, int count, int flip, int reverse)
{
	int startcol, startrow, endcol, endrow;
	int i;

	if (from + count > scp->xsize * scp->ysize)
		count = scp->xsize * scp->ysize - from;

	if (count < 1)
		return;

	if (!monomap_initted)
		initmonomap();

	startrow = from / scp->xsize;
	startcol = from % scp->xsize;
	endrow = (from + count - 1) / scp->xsize;
	endcol = (from + count - 1) % scp->xsize;

	if (startrow == endrow) {
		draw_glyphline32(scp, startrow, startcol, count, flip, reverse);
	} else {
		draw_glyphline32(scp, startrow, startcol, scp->xsize - startcol, flip, reverse);
		for (i = startrow + 1; i < endrow; i++) {
			draw_glyphline32(scp, i, 0, scp->xsize, flip, reverse);
		}
		draw_glyphline32(scp, endrow, 0, endcol + 1, flip, reverse);
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
	sc_softc_t *sc = scp->sc;

	if (sc->shadow != NULL)
		sc->shadow[at] = 0xff00;
	if (on)
		draw_glyphs32(scp, at, 1, flip, 1);
	else
		draw_glyphs32(scp, at, 1, flip, 0);
	if (sc->shadow != NULL)
		sc->shadow[at] = 0xff00;
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

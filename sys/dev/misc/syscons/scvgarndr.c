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

#include <bus/isa/isareg.h>

static vr_draw_border_t		vga_txtborder;
static vr_draw_t		vga_txtdraw;
static vr_set_cursor_t		vga_txtcursor_shape;
static vr_draw_cursor_t		vga_txtcursor;
static vr_blink_cursor_t	vga_txtblink;
#ifndef SC_NO_CUTPASTE
static vr_draw_mouse_t		vga_txtmouse;
#else
#define	vga_txtmouse		(vr_draw_mouse_t *)vga_nop
#endif

#ifndef SC_NO_MODE_CHANGE
static vr_draw_border_t		vga_grborder;
#endif

static void			vga_nop(scr_stat *scp, ...);

static sc_rndr_sw_t txtrndrsw = {
	vga_txtborder,
	vga_txtdraw,	
	vga_txtcursor_shape,
	vga_txtcursor,
	vga_txtblink,
	vga_txtmouse,
};
RENDERER(vga, V_INFO_MM_TEXT, txtrndrsw, vga_set);

#ifndef SC_NO_MODE_CHANGE
static sc_rndr_sw_t grrndrsw = {
	vga_grborder,
	(vr_draw_t *)vga_nop,
	(vr_set_cursor_t *)vga_nop,
	(vr_draw_cursor_t *)vga_nop,
	(vr_blink_cursor_t *)vga_nop,
	(vr_draw_mouse_t *)vga_nop,
};
RENDERER(vga, V_INFO_MM_OTHER, grrndrsw, vga_set);
#endif /* SC_NO_MODE_CHANGE */

RENDERER_MODULE(vga, vga_set);

#ifndef SC_NO_CUTPASTE
static u_short mouse_and_mask[16] = {
	0xc000, 0xe000, 0xf000, 0xf800, 0xfc00, 0xfe00, 0xff00, 0xff80,
	0xfe00, 0x1e00, 0x1f00, 0x0f00, 0x0f00, 0x0000, 0x0000, 0x0000
};
static u_short mouse_or_mask[16] = {
	0x0000, 0x4000, 0x6000, 0x7000, 0x7800, 0x7c00, 0x7e00, 0x6800,
	0x0c00, 0x0c00, 0x0600, 0x0600, 0x0000, 0x0000, 0x0000, 0x0000
};
#endif

static void
vga_nop(scr_stat *scp, ...)
{
}

/* text mode renderer */

static void
vga_txtborder(scr_stat *scp, int color)
{
	(*vidsw[scp->sc->adapter]->set_border)(scp->sc->adp, color);
}

static void
vga_txtdraw(scr_stat *scp, int from, int count, int flip)
{
	uint16_t *p;
	int c;
	int a;

	if (from + count > scp->xsize*scp->ysize)
		count = scp->xsize*scp->ysize - from;

	if (flip) {
		for (p = scp->scr.vtb_buffer + from; count-- > 0; ++from) {
			c = sc_vtb_getc(&scp->vtb, from);
			a = sc_vtb_geta(&scp->vtb, from);
			a = (a & 0x8800) | ((a & 0x7000) >> 4) 
				| ((a & 0x0700) << 4);
			p = sc_vtb_putchar(&scp->scr, p, c, a);
		}
	} else {
		sc_vtb_copy(&scp->vtb, from, &scp->scr, from, count);
	}
}

static void 
vga_txtcursor_shape(scr_stat *scp, int base, int height, int blink)
{
	if (base < 0 || base >= scp->font_size)
		return;

	/* the caller may set height <= 0 in order to disable the cursor */
#if 0
	scp->cursor_base = base;
	scp->cursor_height = height;
#endif
	(*vidsw[scp->sc->adapter]->set_hw_cursor_shape)(scp->sc->adp,
							base, height,
							scp->font_size, blink);

}

static void
draw_txtcharcursor(scr_stat *scp, int at, u_short c, u_short a, int flip)
{
	sc_softc_t *sc;

	sc = scp->sc;
	scp->cursor_saveunder_char = c;
	scp->cursor_saveunder_attr = a;

#ifndef SC_NO_FONT_LOADING
	if (sc->flags & SC_CHAR_CURSOR) {
		unsigned char *font;
		int h;
		int i;

		if (scp->font_size < 14) {
			font = sc->font_8;
			h = 8;
		} else if (scp->font_size >= 16) {
			font = sc->font_16;
			h = 16;
		} else {
			font = sc->font_14;
			h = 14;
		}
		if (scp->cursor_base >= h)
			return;
		if (flip)
			a = (a & 0x8800)
				| ((a & 0x7000) >> 4) | ((a & 0x0700) << 4);
		bcopy(font + c*h, font + sc->cursor_char*h, h);
		font = font + sc->cursor_char*h;
		for (i = imax(h - scp->cursor_base - scp->cursor_height, 0);
			i < h - scp->cursor_base; ++i) {
			font[i] ^= 0xff;
		}
		sc->font_loading_in_progress = TRUE;
		/* XXX */
		(*vidsw[sc->adapter]->load_font)(sc->adp, 0, h, font,
						 sc->cursor_char, 1);
		sc->font_loading_in_progress = FALSE;
		sc_vtb_putc(&scp->scr, at, sc->cursor_char, a);
	} else
#endif /* SC_NO_FONT_LOADING */
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

static void
vga_txtcursor(scr_stat *scp, int at, int blink, int on, int flip)
{
	video_adapter_t *adp;
	int cursor_attr;

	if (scp->cursor_height <= 0)	/* the text cursor is disabled */
		return;

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
}

static void
vga_txtblink(scr_stat *scp, int at, int flip)
{
}

int sc_txtmouse_no_retrace_wait;

#ifndef SC_NO_CUTPASTE

static void
draw_txtmouse(scr_stat *scp, int x, int y)
{
#ifndef SC_ALT_MOUSE_IMAGE
    if (ISMOUSEAVAIL(scp->sc->adp->va_flags)) {
	u_char font_buf[128];
	u_short cursor[32];
	u_char c;
	int pos;
	int xoffset, yoffset;
	int i;

	/* prepare mousepointer char's bitmaps */
	pos = (y/scp->font_size - scp->yoff)*scp->xsize + x/8 - scp->xoff;
	bcopy(scp->font + sc_vtb_getc(&scp->scr, pos)*scp->font_size,
	      &font_buf[0], scp->font_size);
	bcopy(scp->font + sc_vtb_getc(&scp->scr, pos + 1)*scp->font_size,
	      &font_buf[32], scp->font_size);
	bcopy(scp->font 
		 + sc_vtb_getc(&scp->scr, pos + scp->xsize)*scp->font_size,
	      &font_buf[64], scp->font_size);
	bcopy(scp->font
		 + sc_vtb_getc(&scp->scr, pos + scp->xsize + 1)*scp->font_size,
	      &font_buf[96], scp->font_size);
	for (i = 0; i < scp->font_size; ++i) {
		cursor[i] = font_buf[i]<<8 | font_buf[i+32];
		cursor[i + scp->font_size] = font_buf[i+64]<<8 | font_buf[i+96];
	}

	/* now and-or in the mousepointer image */
	xoffset = x%8;
	yoffset = y%scp->font_size;
	for (i = 0; i < 16; ++i) {
		cursor[i + yoffset] =
	    		(cursor[i + yoffset] & ~(mouse_and_mask[i] >> xoffset))
	    		| (mouse_or_mask[i] >> xoffset);
	}
	for (i = 0; i < scp->font_size; ++i) {
		font_buf[i] = (cursor[i] & 0xff00) >> 8;
		font_buf[i + 32] = cursor[i] & 0xff;
		font_buf[i + 64] = (cursor[i + scp->font_size] & 0xff00) >> 8;
		font_buf[i + 96] = cursor[i + scp->font_size] & 0xff;
	}

#if 1
	/* wait for vertical retrace to avoid jitter on some videocards */
	while (!sc_txtmouse_no_retrace_wait &&
	    !(inb(CRTC + 6) & 0x08))
		/* idle */ ;
#endif
	c = scp->sc->mouse_char;
	(*vidsw[scp->sc->adapter]->load_font)(scp->sc->adp, 0, 32, font_buf,
					      c, 4); 

	sc_vtb_putc(&scp->scr, pos, c, sc_vtb_geta(&scp->scr, pos));
	/* FIXME: may be out of range! */
	sc_vtb_putc(&scp->scr, pos + scp->xsize, c + 2,
		    sc_vtb_geta(&scp->scr, pos + scp->xsize));
	if (x < (scp->xsize - 1)*8) {
		sc_vtb_putc(&scp->scr, pos + 1, c + 1,
			    sc_vtb_geta(&scp->scr, pos + 1));
		sc_vtb_putc(&scp->scr, pos + scp->xsize + 1, c + 3,
			    sc_vtb_geta(&scp->scr, pos + scp->xsize + 1));
	}
    } else
#endif /* SC_ALT_MOUSE_IMAGE */
    {
	/* Red, magenta and brown are mapped to green to to keep it readable */
	static const int col_conv[16] = {
		6, 6, 6, 6, 2, 2, 2, 6, 14, 14, 14, 14, 10, 10, 10, 14
	};
	int pos;
	int color;
	int a;

	pos = (y/scp->font_size - scp->yoff)*scp->xsize + x/8 - scp->xoff;
	a = sc_vtb_geta(&scp->scr, pos);
	if (scp->sc->adp->va_flags & V_ADP_COLOR)
		color = (col_conv[(a & 0xf000) >> 12] << 12)
			| ((a & 0x0f00) | 0x0800);
	else
		color = ((a & 0xf000) >> 4) | ((a & 0x0f00) << 4);
	sc_vtb_putc(&scp->scr, pos, sc_vtb_getc(&scp->scr, pos), color);
    }

}

static void
remove_txtmouse(scr_stat *scp, int x, int y)
{
}

static void 
vga_txtmouse(scr_stat *scp, int x, int y, int on)
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
vga_grborder(scr_stat *scp, int color)
{
	lwkt_gettoken(&tty_token);
	(*vidsw[scp->sc->adapter]->set_border)(scp->sc->adp, color);
	lwkt_reltoken(&tty_token);
}

#endif

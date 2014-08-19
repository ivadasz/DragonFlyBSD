/*-
 * (MPSAFE)
 *
 * Copyright (c) 1998 Kazutaka YOKOTA <yokota@zodiac.mech.utsunomiya-u.ac.jp>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/dev/syscons/scvidctl.c,v 1.19.2.2 2000/05/05 09:16:08 nyan Exp $
 * $DragonFly: src/sys/dev/misc/syscons/scvidctl.c,v 1.16 2007/08/19 11:39:11 swildner Exp $
 */

#include "opt_syscons.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/signalvar.h>
#include <sys/tty.h>
#include <sys/kernel.h>
#include <sys/thread2.h>

#include <machine/console.h>

#include <dev/video/fb/fbreg.h>
#include "syscons.h"
#include "../txt/txtdev.h"

int
sc_set_text_mode(scr_stat *scp, struct tty *tp, int mode, int xsize, int ysize,
		 int fontsize)
{
    video_info_t info;
    int prev_ysize;
    int new_ysize;
    int error;

    lwkt_gettoken(&tty_token);
    if ((*vidsw[scp->sc->adapter]->get_info)(scp->sc->adp, mode, &info)) {
	lwkt_reltoken(&tty_token);
	return ENODEV;
    }
    lwkt_reltoken(&tty_token);

    if ((xsize <= 0) || (xsize > info.vi_width))
	xsize = info.vi_width;
    if ((ysize <= 0) || (ysize > info.vi_height))
	ysize = info.vi_height;

    /* stop screen saver, etc */
    crit_enter();
    if ((error = sc_clean_up(scp))) {
	crit_exit();
	return error;
    }

    /* set up scp */
    new_ysize = 0;
#ifndef SC_NO_HISTORY
    if (scp->history != NULL) {
	sc_hist_save(scp);
	new_ysize = sc_vtb_rows(scp->history); 
    }
#endif
    prev_ysize = scp->ysize;
    /*
     * This is a kludge to fend off scrn_update() while we
     * muck around with scp. XXX
     */
    scp->status |= UNKNOWN_MODE;
    scp->status &= ~GRAPHICS_MODE;
    scp->mode = mode;
    scp->model = V_INFO_MM_TEXT;
    scp->xsize = xsize;
    scp->ysize = ysize;

    /* allocate buffers */
    sc_alloc_scr_buffer(scp, TRUE, TRUE);
    sc_init_emulator(scp, NULL);
#ifndef SC_NO_HISTORY
    sc_alloc_history_buffer(scp, new_ysize, prev_ysize, FALSE);
#endif
    crit_exit();

    if (scp == scp->sc->cur_scp)
	set_mode(scp);
    scp->status &= ~UNKNOWN_MODE;

    if (tp == NULL)
	return 0;
    DPRINTF(5, ("ws_*size (%d,%d), size (%d,%d)\n",
	tp->t_winsize.ws_col, tp->t_winsize.ws_row, scp->xsize, scp->ysize));
    if (tp->t_winsize.ws_col != scp->xsize
	|| tp->t_winsize.ws_row != scp->ysize) {
	tp->t_winsize.ws_col = scp->xsize;
	tp->t_winsize.ws_row = scp->ysize;
	pgsignal(tp->t_pgrp, SIGWINCH, 1);
    }

    return 0;
}

#define fb_ioctl(a, c, d)		\
	(((a) == NULL) ? ENODEV : 	\
			 (*vidsw[(a)->va_index]->ioctl)((a), (c), (caddr_t)(d)))

int
sc_vid_ioctl(struct tty *tp, u_long cmd, caddr_t data, int flag)
{
    sc_softc_t *sc;
    scr_stat *scp;
    video_adapter_t *adp;
    int error, ret;

	KKASSERT(tp->t_dev);

    scp = SC_STAT(tp->t_dev);
    if (scp == NULL)		/* tp == SC_MOUSE */
		return ENOIOCTL;
    sc = (sc_softc_t *)scp->sc;
    adp = sc->adp;
    if (adp == NULL)		/* shouldn't happen??? */
		return ENODEV;

    lwkt_gettoken(&tty_token);
    switch (cmd) {

    case CONS_CURRENTADP:	/* get current adapter index */
    case FBIO_ADAPTER:
	ret = fb_ioctl(adp, FBIO_ADAPTER, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case CONS_CURRENT:  	/* get current adapter type */
    case FBIO_ADPTYPE:
	ret = fb_ioctl(adp, FBIO_ADPTYPE, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case CONS_ADPINFO:		/* adapter information */
    case FBIO_ADPINFO:
	if (((video_adapter_info_t *)data)->va_index >= 0) {
	    adp = vid_get_adapter(((video_adapter_info_t *)data)->va_index);
	    if (adp == NULL) {
		lwkt_reltoken(&tty_token);
		return ENODEV;
	    }
	}
	ret = fb_ioctl(adp, FBIO_ADPINFO, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case CONS_GET:      	/* get current video mode */
    case FBIO_GETMODE:
	*(int *)data = scp->mode;
	lwkt_reltoken(&tty_token);
	return 0;

    case CONS_MODEINFO:		/* get mode information */
    case FBIO_MODEINFO:
	ret = fb_ioctl(adp, FBIO_MODEINFO, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case CONS_FINDMODE:		/* find a matching video mode */
    case FBIO_FINDMODE:
	ret = fb_ioctl(adp, FBIO_FINDMODE, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case CONS_SETWINORG:	/* set frame buffer window origin */
    case FBIO_SETWINORG:
	if (scp != scp->sc->cur_scp) {
	    lwkt_reltoken(&tty_token);
	    return ENODEV;	/* XXX */
	}
	ret = fb_ioctl(adp, FBIO_SETWINORG, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case FBIO_GETWINORG:	/* get frame buffer window origin */
	if (scp != scp->sc->cur_scp) {
	    lwkt_reltoken(&tty_token);
	    return ENODEV;	/* XXX */
	}
	ret = fb_ioctl(adp, FBIO_GETWINORG, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case FBIO_GETDISPSTART:
    case FBIO_SETDISPSTART:
    case FBIO_GETLINEWIDTH:
    case FBIO_SETLINEWIDTH:
	if (scp != scp->sc->cur_scp) {
	    lwkt_reltoken(&tty_token);
	    return ENODEV;	/* XXX */
	}
	ret = fb_ioctl(adp, cmd, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case FBIO_GETPALETTE:
    case FBIO_SETPALETTE:
    case FBIOPUTCMAP:
    case FBIOGETCMAP:
    case FBIOGTYPE:
    case FBIOGATTR:
    case FBIOSVIDEO:
    case FBIOGVIDEO:
    case FBIOSCURSOR:
    case FBIOGCURSOR:
    case FBIOSCURPOS:
    case FBIOGCURPOS:
    case FBIOGCURMAX:
	if (scp != scp->sc->cur_scp) {
	    lwkt_reltoken(&tty_token);
	    return ENODEV;	/* XXX */
	}
	ret = fb_ioctl(adp, cmd, data);
	lwkt_reltoken(&tty_token);
	return ret;

    case KDSETMODE:     	/* set current mode of this (virtual) console */
	switch (*(int *)data) {
	case KD_TEXT:   	/* switch to TEXT (known) mode */
	    /*
	     * If scp->mode is of graphics modes, we don't know which
	     * text mode to switch back to...
	     */
	    if (scp->status & GRAPHICS_MODE) {
	        lwkt_reltoken(&tty_token);
		return EINVAL;
	    }

	    /* move hardware cursor out of the way */
	    if (sc->txtdevsw != NULL) {
		sc->txtdevsw->setcurmode(sc->txtdev_cookie,
		    TXTDEV_CURSOR_BLOCK);
		sc->txtdevsw->setcursor(sc->txtdev_cookie, -1, -1);
	    }
	    /* FALL THROUGH */

	case KD_TEXT1:  	/* switch to TEXT (known) mode */
	    /*
	     * If scp->mode is of graphics modes, we don't know which
	     * text/pixel mode to switch back to...
	     */
	    if (scp->status & GRAPHICS_MODE) {
	        lwkt_reltoken(&tty_token);
		return EINVAL;
	    }
	    crit_enter();
	    if ((error = sc_clean_up(scp))) {
		crit_exit();
		lwkt_reltoken(&tty_token);
		return error;
	    }
	    scp->status |= UNKNOWN_MODE;
	    crit_exit();
	    if (scp == scp->sc->cur_scp)
		set_mode(scp);
	    sc_clear_screen(scp);
	    scp->status &= ~UNKNOWN_MODE;
	    lwkt_reltoken(&tty_token);
	    return 0;

	case KD_GRAPHICS:	/* switch to GRAPHICS (unknown) mode */
	    crit_enter();
	    if ((error = sc_clean_up(scp))) {
		crit_exit();
		lwkt_reltoken(&tty_token);
		return error;
	    }
	    scp->status |= UNKNOWN_MODE;
	    crit_exit();
	    lwkt_reltoken(&tty_token);
	    return 0;

	default:
	    lwkt_reltoken(&tty_token);
	    return EINVAL;
	}
	/* NOT REACHED */

    case KDGETMODE:     	/* get current mode of this (virtual) console */
	/* 
	 * From the user program's point of view, KD_PIXEL is the same 
	 * as KD_TEXT... 
	 */
	*data = ISGRAPHSC(scp) ? KD_GRAPHICS : KD_TEXT;
	lwkt_reltoken(&tty_token);
	return 0;

    case KDSBORDER:     	/* set border color of this (virtual) console */
	scp->border = *data;
#if 0
	if (scp == scp->sc->cur_scp)
	    sc_set_border(scp, scp->border);
#endif
	lwkt_reltoken(&tty_token);
	return 0;
    }

    lwkt_reltoken(&tty_token);
    return ENOIOCTL;
}

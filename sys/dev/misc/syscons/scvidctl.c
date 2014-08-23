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

//#include <dev/video/fb/fbreg.h>
#include "syscons.h"
#include "../txt/txtdev.h"

int
sc_vid_ioctl(struct tty *tp, u_long cmd, caddr_t data, int flag)
{
    sc_softc_t *sc;
    scr_stat *scp;
    int error;

    KKASSERT(tp->t_dev);

    scp = SC_STAT(tp->t_dev);
    if (scp == NULL)		/* tp == SC_MOUSE */
	return ENOIOCTL;
    sc = (sc_softc_t *)scp->sc;

    lwkt_gettoken(&tty_token);
    switch (cmd) {

    case KDSETMODE:     	/* set current mode of this (virtual) console */
	switch (*(int *)data) {
	case KD_TEXT:   	/* switch to TEXT (known) mode */
	    /* move hardware cursor out of the way */
	    if (sc->txtdevsw != NULL) {
		sc->txtdevsw->setcursor(sc->txtdev_cookie, -1);
	    }
	    /* FALL THROUGH */

	case KD_TEXT1:  	/* switch to TEXT (known) mode */
	    crit_enter();
	    if ((error = sc_clean_up(scp))) {
		crit_exit();
		lwkt_reltoken(&tty_token);
		return error;
	    }
	    scp->status |= UNKNOWN_MODE;
	    crit_exit();
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
#if 0
	scp->border = *data;
	if (scp == scp->sc->cur_scp)
	    sc_set_border(scp, scp->border);
#endif
	lwkt_reltoken(&tty_token);
	return 0;
    }

    lwkt_reltoken(&tty_token);
    return ENOIOCTL;
}

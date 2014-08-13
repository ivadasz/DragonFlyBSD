/*-
 * Copyright (c) 1999 Kazutaka YOKOTA <yokota@zodiac.mech.utsunomiya-u.ac.jp>
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
 *
 * $FreeBSD: src/sys/dev/syscons/scmouse.c,v 1.12.2.3 2001/07/28 12:51:47 yokota Exp $
 */

#include "opt_syscons.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/signalvar.h>
#include <sys/proc.h>
#include <sys/tty.h>
#include <sys/thread2.h>
#include <sys/mplock2.h>

#include <machine/console.h>
#include <sys/mouse.h>

#include "syscons.h"

#ifdef SC_TWOBUTTON_MOUSE
#define SC_MOUSE_PASTEBUTTON	MOUSE_BUTTON3DOWN	/* right button */
#define SC_MOUSE_EXTENDBUTTON	MOUSE_BUTTON2DOWN	/* not really used */
#else
#define SC_MOUSE_PASTEBUTTON	MOUSE_BUTTON2DOWN	/* middle button */
#define SC_MOUSE_EXTENDBUTTON	MOUSE_BUTTON3DOWN	/* right button */
#endif /* SC_TWOBUTTON_MOUSE */

#define SC_WAKEUP_DELTA		20

#ifndef SC_NO_SYSMOUSE

/* local functions */
static void sc_mouse_init(void *);
static void sc_mouse_uninit(void *);

static void set_mouse_pos(scr_stat *scp);

/* move mouse */
void
sc_mouse_move(scr_stat *scp, int x, int y)
{
    crit_enter();
    scp->mouse_xpos = scp->mouse_oldxpos = x;
    scp->mouse_ypos = scp->mouse_oldypos = y;
    if (scp->font_size <= 0)
	scp->mouse_pos = scp->mouse_oldpos = 0;
    else
	scp->mouse_pos = scp->mouse_oldpos = 
	    (y/scp->font_size - scp->yoff)*scp->xsize + x/8 - scp->xoff;
    scp->status |= MOUSE_MOVED;
    crit_exit();
}

/* adjust mouse position */
static void
set_mouse_pos(scr_stat *scp)
{
    if (scp->mouse_xpos < scp->xoff*8)
	scp->mouse_xpos = scp->xoff*8;
    if (scp->mouse_ypos < scp->yoff*scp->font_size)
	scp->mouse_ypos = scp->yoff*scp->font_size;
    if (ISGRAPHSC(scp)) {
        if (scp->mouse_xpos > scp->xpixel-1)
	    scp->mouse_xpos = scp->xpixel-1;
        if (scp->mouse_ypos > scp->ypixel-1)
	    scp->mouse_ypos = scp->ypixel-1;
	return;
    } else {
	if (scp->mouse_xpos > (scp->xsize + scp->xoff)*8 - 1)
	    scp->mouse_xpos = (scp->xsize + scp->xoff)*8 - 1;
	if (scp->mouse_ypos > (scp->ysize + scp->yoff)*scp->font_size - 1)
	    scp->mouse_ypos = (scp->ysize + scp->yoff)*scp->font_size - 1;
    }

    if (scp->mouse_xpos != scp->mouse_oldxpos || scp->mouse_ypos != scp->mouse_oldypos) {
	scp->status |= MOUSE_MOVED;
    	scp->mouse_pos =
	    (scp->mouse_ypos/scp->font_size - scp->yoff)*scp->xsize 
		+ scp->mouse_xpos/8 - scp->xoff;
    }
}

static void
sc_mouse_exit1_proc(struct proc *p)
{
    scr_stat *scp;

    scp = p->p_drv_priv;
    KKASSERT(scp != NULL);

    get_mplock();
    KKASSERT(scp->mouse_proc == p);
    KKASSERT(scp->mouse_pid == p->p_pid);

    scp->mouse_signal = 0;
    scp->mouse_proc = NULL;
    scp->mouse_pid = 0;
    rel_mplock();

    PRELE(p);
    p->p_flags &= ~P_SCMOUSE;
    p->p_drv_priv = NULL;
}

/*
 * sc_mouse_exit1:
 *
 *	Handle exit1 for processes registered as MOUSE_MODE handlers.
 *	We must remove a process hold, established when MOUSE_MODE
 *	was enabled.
 */
static void
sc_mouse_exit1(struct thread *td)
{
    struct proc *p;

    p = td->td_proc;
    KKASSERT(p != NULL);

    if ((p->p_flags & P_SCMOUSE) == 0)
	return;


    sc_mouse_exit1_proc(p);
}

int
sc_mouse_ioctl(struct tty *tp, u_long cmd, caddr_t data, int flag)
{
    mouse_info_t *mouse;
    scr_stat *cur_scp;
    scr_stat *scp;
    int f;

    scp = SC_STAT(tp->t_dev);

    switch (cmd) {

    case CONS_MOUSECTL:		/* control mouse arrow */
	mouse = (mouse_info_t*) data;
	cur_scp = scp->sc->cur_scp;

	switch (mouse->operation) {
	/*
	 * Setup a process to receive signals on mouse events.
	 */
	case MOUSE_MODE:
	    get_mplock();

	    if (!ISSIGVALID(mouse->u.mode.signal)) {
		/* Setting MOUSE_MODE w/ an invalid signal is used to disarm */
		if (scp->mouse_proc == curproc) {
		    sc_mouse_exit1_proc(curproc);
		    rel_mplock();
		    return 0;
		} else {
		    rel_mplock();
		    return EINVAL;
		}
	    } else {
		/* Only one mouse process per syscons */
		if (scp->mouse_proc) {
		    rel_mplock();
		    return EINVAL;
		}

		/* Only one syscons signal source per process */
		if (curproc->p_flags & P_SCMOUSE) {
		    rel_mplock();
		    return EINVAL;
		}

	        /*
	         * Process is stabilized by a hold, which is removed from
	         * sc_mouse_exit1. scp's mouse_{signal,proc,pid} fields
	         * are synchronized by the MP Lock.
	         */
	        scp->mouse_signal = mouse->u.mode.signal;
	        scp->mouse_proc = curproc;
	        scp->mouse_pid = curproc->p_pid;
	        curproc->p_flags |= P_SCMOUSE;
		KKASSERT(curproc->p_drv_priv == NULL);
	        curproc->p_drv_priv = scp;
	        PHOLD(curproc);

	        rel_mplock();
	        return 0;
            }
	    /*NOTREACHED*/
	    break;

	case MOUSE_MOVEABS:
	    crit_enter();
	    scp->mouse_xpos = mouse->u.data.x;
	    scp->mouse_ypos = mouse->u.data.y;
	    set_mouse_pos(scp);
	    crit_exit();
	    break;

	case MOUSE_MOVEREL:
	    crit_enter();
	    scp->mouse_xpos += mouse->u.data.x;
	    scp->mouse_ypos += mouse->u.data.y;
	    set_mouse_pos(scp);
	    crit_exit();
	    break;

	case MOUSE_GETINFO:
	    mouse->u.data.x = scp->mouse_xpos;
	    mouse->u.data.y = scp->mouse_ypos;
	    mouse->u.data.z = 0;
	    mouse->u.data.buttons = scp->mouse_buttons;
	    return 0;

	case MOUSE_ACTION:
	case MOUSE_MOTION_EVENT:
	    /* send out mouse event on /dev/sysmouse */
#if 0
	    /* this should maybe only be settable from /dev/consolectl SOS */
	    if (SC_VTY(tp->t_dev) != SC_CONSOLECTL)
		return ENOTTY;
#endif
	    crit_enter();
	    if (mouse->u.data.x != 0 || mouse->u.data.y != 0) {
		cur_scp->mouse_xpos += mouse->u.data.x;
		cur_scp->mouse_ypos += mouse->u.data.y;
		set_mouse_pos(cur_scp);
	    }
	    f = 0;
	    if (mouse->operation == MOUSE_ACTION) {
		f = cur_scp->mouse_buttons ^ mouse->u.data.buttons;
		cur_scp->mouse_buttons = mouse->u.data.buttons;
	    }
	    crit_exit();

	    if (sysmouse_event(mouse) == 0)
		return 0;

	    get_mplock();
	    if (cur_scp->mouse_signal) {
		KKASSERT(cur_scp->mouse_proc != NULL);
		ksignal(cur_scp->mouse_proc, cur_scp->mouse_signal);
		rel_mplock();
	        break;
	    }
	    rel_mplock();
	    break;

	case MOUSE_BUTTON_EVENT:
	    if ((mouse->u.event.id & MOUSE_BUTTONS) == 0)
		return EINVAL;
	    if (mouse->u.event.value < 0)
		return EINVAL;
#if 0
	    /* this should maybe only be settable from /dev/consolectl SOS */
	    if (SC_VTY(tp->t_dev) != SC_CONSOLECTL)
		return ENOTTY;
#endif
	    if (mouse->u.event.value > 0)
		cur_scp->mouse_buttons |= mouse->u.event.id;
	    else
		cur_scp->mouse_buttons &= ~mouse->u.event.id;

	    if (sysmouse_event(mouse) == 0)
		return 0;

	    get_mplock();
	    if (cur_scp->mouse_signal) {
		KKASSERT(cur_scp->mouse_proc != NULL);
		ksignal(cur_scp->mouse_proc, cur_scp->mouse_signal);
		rel_mplock();
	        break;
	    }
	    rel_mplock();
	    break;

	case MOUSE_MOUSECHAR:
	    if (mouse->u.mouse_char < 0) {
		mouse->u.mouse_char = scp->sc->mouse_char;
	    } else {
		if (mouse->u.mouse_char >= (unsigned char)-1 - 4)
		    return EINVAL;
		crit_enter();
		scp->sc->mouse_char = mouse->u.mouse_char;
		crit_exit();
	    }
	    break;

	default:
	    return EINVAL;
	}

	return 0;
    }

    return ENOIOCTL;
}

void
sc_mouse_init(void *unused)
{
    at_exit(sc_mouse_exit1);
}

void
sc_mouse_uninit(void *unused)
{
}

SYSINIT(sc_mouse_init, SI_SUB_DRIVERS, SI_ORDER_ANY, sc_mouse_init, NULL);
SYSUNINIT(sc_mouse_uninit, SI_SUB_DRIVERS, SI_ORDER_ANY, sc_mouse_uninit, NULL);

#endif /* SC_NO_SYSMOUSE */

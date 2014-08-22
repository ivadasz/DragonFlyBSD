/*-
 * Copyright (c) 1995-1998 Søren Schmidt
 * All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Sascha Wildner <saw@online.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
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
 * $FreeBSD: src/sys/dev/syscons/syscons.h,v 1.60.2.6 2002/09/15 22:30:45 dd Exp $
 * $DragonFly: src/sys/dev/misc/syscons/syscons.h,v 1.21 2008/08/03 03:00:21 dillon Exp $
 */

#ifndef _DEV_SYSCONS_SYSCONS_H_
#define	_DEV_SYSCONS_SYSCONS_H_

#include <sys/malloc.h>

MALLOC_DECLARE(M_SYSCONS);

/* default values for configuration options */

#ifndef MAXCONS
#define MAXCONS		16	/* power of 2 */
#endif

#ifndef SC_CURSOR_CHAR
#define SC_CURSOR_CHAR	(0x07)
#endif

#ifndef SC_DEBUG_LEVEL
#define SC_DEBUG_LEVEL	0
#endif

#define DPRINTF(l, p)	if (SC_DEBUG_LEVEL >= (l)) kprintf p

#define SC_DRIVER_NAME	"sc"
#define SC_VTY(dev)	minor(dev)
#define SC_DEV(sc, vty)	((sc)->dev ? (sc)->dev[(vty) - (sc)->first_vty] : NULL)
#define SC_STAT(dev)	((scr_stat *)(dev)->si_drv1)

/* printable chars */
#ifndef PRINTABLE
#define PRINTABLE(ch)	((ch) > 0x1b || ((ch) > 0x0d && (ch) < 0x1b) \
			 || (ch) < 0x07)
#endif

/* macros for "intelligent" screen update */
#define mark_for_update(scp, x)	{\
			  	    if ((x) < scp->start) scp->start = (x);\
				    else if ((x) > scp->end) scp->end = (x);\
				}
#define mark_all(scp)		{\
				    scp->start = 0;\
				    scp->end = scp->xsize * scp->ysize - 1;\
				}

/* vty status flags (scp->status) */
#define UNKNOWN_MODE	0x00010		/* unknown video mode */
#define SWITCH_WAIT_REL	0x00080		/* waiting for vty release */
#define SWITCH_WAIT_ACQ	0x00100		/* waiting for vty ack */
#define BUFFER_SAVED	0x00200		/* vty buffer is saved */
#define CURSOR_ENABLED 	0x00400		/* text cursor is enabled */
//#define MOUSE_MOVED	0x01000		/* mouse cursor has moved */
//#define MOUSE_CUTTING	0x02000		/* mouse cursor is cutting text */
//#define MOUSE_VISIBLE	0x04000		/* mouse cursor is showing */
//#define GRAPHICS_MODE	0x08000		/* vty is in a graphics mode */
//#define PIXEL_MODE	0x10000		/* vty is in a raster text mode */
//#define SAVER_RUNNING	0x20000		/* screen saver is running */
//#define VR_CURSOR_BLINK	0x40000		/* blinking text cursor */
//#define VR_CURSOR_ON	0x80000		/* text cursor is on */
//#define MOUSE_HIDDEN	0x100000	/* mouse cursor is temporarily hidden */

/* misc defines */
#define FALSE		0
#define TRUE		1
#define	COL		80
#define	ROW		25
#define PCBURST		128

#ifndef BELL_DURATION
#define BELL_DURATION	((5 * hz + 99) / 100)
#define BELL_PITCH	800
#endif

/* virtual terminal buffer */
typedef struct sc_vtb {
	int		vtb_flags;
#define VTB_VALID	(1 << 0)
#define VTB_ALLOCED	(1 << 1)
	int		vtb_type;
#define VTB_INVALID	0
#define VTB_MEMORY	1
//#define VTB_FRAMEBUFFER	2
#define VTB_RINGBUFFER	3
	int		vtb_cols;
	int		vtb_rows;
	int		vtb_size;
	uint16_t	*vtb_buffer;
	int		vtb_tail;	/* valid for VTB_RINGBUFFER only */
} sc_vtb_t;

/* softc */

struct keyboard;
struct video_adapter;
struct scr_stat;
struct tty;
struct dev_ioctl_args;

typedef struct sc_softc {
	int		unit;			/* unit # */
	int		config;			/* configuration flags */
//#define SC_VESA800X600	(1 << 7)
#define SC_AUTODETECT_KBD (1 << 8)
#define SC_KERNEL_CONSOLE (1 << 9)

	int		flags;			/* status flags */
#define SC_VISUAL_BELL	(1 << 0)
#define SC_QUIET_BELL	(1 << 1)
#define SC_BLINK_CURSOR	(1 << 2)
#define SC_CHAR_CURSOR	(1 << 3)
//#define SC_MOUSE_ENABLED (1 << 4)
//#define	SC_SCRN_IDLE	(1 << 5)
//#define	SC_SCRN_BLANKED	(1 << 6)
//#define	SC_SAVER_FAILED	(1 << 7)
#define	SC_SCRN_VTYLOCK	(1 << 8)

#define	SC_INIT_DONE	(1 << 16)
//#define	SC_SPLASH_SCRN	(1 << 17)

	int		keyboard;		/* -1 if unavailable */
	struct keyboard	*kbd;

	void		*txtdev_cookie;
	struct txtdev_sw *txtdevsw;

	int		adapter;
	struct video_adapter *adp;

	int		first_vty;
	int		vtys;
	cdev_t		*dev;
	struct scr_stat	*console_scp;
	struct scr_stat	*cur_scp;
	struct scr_stat	*new_scp;
	struct scr_stat	*old_scp;
	int     	delayed_next_scr;

	char        	switch_in_progress;
	char        	videoio_in_progress;
	char        	write_in_progress;
	char        	blink_in_progress;

	struct callout	scrn_timer_ch;

	u_char      	scr_map[256];
	u_char      	scr_rmap[256];

#ifdef _SC_MD_SOFTC_DECLARED_
	sc_md_softc_t	md;			/* machine dependent vars */
#endif

	u_char		cursor_char;

} sc_softc_t;

/* virtual screen */
typedef struct scr_stat {
	int		index;			/* index of this vty */
	struct sc_softc *sc;			/* pointer to softc */
	sc_vtb_t	vtb;

	int 		xpos;			/* current X position */
	int 		ypos;			/* current Y position */
	int 		xsize;			/* X text size */
	int 		ysize;			/* Y text size */

	int		start;			/* modified area start */
	int		end;			/* modified area end */

	struct sc_term_sw *tsw;
	void		*ts;

	int	 	status;			/* status (bitfield) */
	int		kbd_mode;		/* keyboard I/O mode */

	int		cursor_pos;		/* cursor buffer position */
	int		cursor_oldpos;		/* cursor old buffer position */

	u_short		bell_duration;
	u_short		bell_pitch;

	struct callout	blink_screen_ch;

	u_char		border;			/* border color */
	pid_t 		pid;			/* pid of controlling proc */
	struct proc 	*proc;			/* proc* of controlling proc */
	struct vt_mode 	smode;			/* switch mode */

	sc_vtb_t	*history;		/* circular history buffer */
	int		history_pos;		/* position shown on screen */
	int		history_size;		/* size of history buffer */

#ifdef _SCR_MD_STAT_DECLARED_
	scr_md_stat_t	md;			/* machine dependent vars */
#endif
} scr_stat;

#ifndef SC_NORM_ATTR
#define SC_NORM_ATTR		(FG_LIGHTGREY | BG_BLACK)
#endif
#ifndef SC_NORM_REV_ATTR
#define SC_NORM_REV_ATTR	(FG_BLACK | BG_LIGHTGREY)
#endif
#ifndef SC_KERNEL_CONS_ATTR
#define SC_KERNEL_CONS_ATTR	(FG_WHITE | BG_BLACK)
#endif
#ifndef SC_KERNEL_CONS_REV_ATTR
#define SC_KERNEL_CONS_REV_ATTR	(FG_BLACK | BG_LIGHTGREY)
#endif

/* terminal emulator */

#ifndef SC_DFLT_TERM
#define SC_DFLT_TERM	"*"			/* any */
#endif

typedef int	sc_term_init_t(scr_stat *scp, void **tcp, int code);
#define SC_TE_COLD_INIT	0
#define SC_TE_WARM_INIT	1
typedef int	sc_term_term_t(scr_stat *scp, void **tcp);
typedef void	sc_term_puts_t(scr_stat *scp, u_char *buf, int len);
typedef int	sc_term_ioctl_t(scr_stat *scp, struct tty *tp, u_long cmd,
				caddr_t data, int flag);
typedef int	sc_term_reset_t(scr_stat *scp, int code);
#define SC_TE_HARD_RESET 0
#define SC_TE_SOFT_RESET 1
typedef void	sc_term_default_attr_t(scr_stat *scp, int norm, int rev);
typedef void	sc_term_clear_t(scr_stat *scp);
typedef void	sc_term_notify_t(scr_stat *scp, int event);
#define SC_TE_NOTIFY_VTSWITCH_IN	0
#define SC_TE_NOTIFY_VTSWITCH_OUT	1
typedef int	sc_term_input_t(scr_stat *scp, int c, struct tty *tp);

typedef struct sc_term_sw {
	LIST_ENTRY(sc_term_sw)	link;
	char 			*te_name;	/* name of the emulator */
	char 			*te_desc;	/* description */
	size_t			te_size;	/* size of internal buffer */
	int			te_refcount;	/* reference counter */
	sc_term_init_t		*te_init;
	sc_term_term_t		*te_term;
	sc_term_puts_t		*te_puts;
	sc_term_ioctl_t		*te_ioctl;
	sc_term_reset_t		*te_reset;
	sc_term_default_attr_t	*te_default_attr;
	sc_term_clear_t		*te_clear;
	sc_term_notify_t	*te_notify;
	sc_term_input_t		*te_input;
} sc_term_sw_t;

extern struct linker_set scterm_set;

#define SCTERM_MODULE(name, sw)					\
	DATA_SET(scterm_set, sw);				\
	static int						\
	scterm_##name##_event(module_t mod, int type, void *data) \
	{							\
		switch (type) {					\
		case MOD_LOAD:					\
			return sc_term_add(&sw);		\
		case MOD_UNLOAD:				\
			if (sw.te_refcount > 0)			\
				return EBUSY;			\
			return sc_term_remove(&sw);		\
		default:					\
			break;					\
		}						\
		return 0;					\
	}							\
	static moduledata_t scterm_##name##_mod = {		\
		"scterm-" #name,				\
		scterm_##name##_event,				\
		NULL,						\
	};							\
	DECLARE_MODULE(scterm_##name, scterm_##name##_mod,	\
		       SI_SUB_DRIVERS, SI_ORDER_MIDDLE)

typedef struct {
	int		cursor_start;
	int		cursor_end;
	int		shift_state;
	int		bell_pitch;
} bios_values_t;

/* other macros */
#define ISTEXTSC(scp)	(!((scp)->status & UNKNOWN_MODE))
#define ISGRAPHSC(scp)	((scp)->status & UNKNOWN_MODE)
#define ISUNKNOWNSC(scp) ((scp)->status & UNKNOWN_MODE)

#define ISMOUSEAVAIL(af) ((af) & V_ADP_FONT)
#define ISFONTAVAIL(af)	((af) & V_ADP_FONT)
#define ISPALAVAIL(af)	((af) & V_ADP_PALETTE)

#define ISSIGVALID(sig)	((sig) > 0 && (sig) < NSIG)

/* syscons.c */
int		sc_probe_unit(int unit, int flags);
int		sc_set_txtdev(void *cookie, struct txtdev_sw *sw);
int		sc_replace_txtdev(void *cookie, struct txtdev_sw *sw,
				  void *oldcookie);

void		sc_update_cursor_image(scr_stat *scp, int on);
int		sc_clean_up(scr_stat *scp);
int		sc_switch_scr(sc_softc_t *sc, u_int next_scr);
void		sc_alloc_scr_buffer(scr_stat *scp, int wait, int discard);
int		sc_init_emulator(scr_stat *scp, char *name);
void		sc_bell(scr_stat *scp, int pitch, int duration);

/* schistory.c */
#ifndef SC_NO_HISTORY
int		sc_alloc_history_buffer(scr_stat *scp, int lines,
					int prev_ysize, int wait);
void		sc_free_history_buffer(scr_stat *scp, int prev_ysize);
void		sc_hist_save(scr_stat *scp);
#define		sc_hist_save_one_line(scp, from)	\
		sc_vtb_append(&(scp)->vtb, (from), (scp)->history, (scp)->xsize)
int		sc_hist_restore(scr_stat *scp);
void		sc_hist_home(scr_stat *scp);
void		sc_hist_end(scr_stat *scp);
int		sc_hist_up_line(scr_stat *scp);
int		sc_hist_down_line(scr_stat *scp);
int		sc_hist_ioctl(struct tty *tp, u_long cmd, caddr_t data,
			      int flag);
#endif /* SC_NO_HISTORY */

/* scmouse.c */
#ifndef SC_NO_SYSMOUSE
int		sc_mouse_ioctl(struct tty *tp, u_long cmd, caddr_t data,
			       int flag);
#endif /* SC_NO_SYSMOUSE */

/* scvidctl.c */
int		sc_vid_ioctl(struct tty *tp, u_long cmd, caddr_t data,
				  int flag);

/* scvtb.c */
void		sc_vtb_init(sc_vtb_t *vtb, int type, int cols, int rows, 
			    void *buffer, int wait);
void		sc_vtb_destroy(sc_vtb_t *vtb);
size_t		sc_vtb_size(int cols, int rows);
void		sc_vtb_clear(sc_vtb_t *vtb, int c, int attr);

int		sc_vtb_getc(sc_vtb_t *vtb, int at);
int		sc_vtb_geta(sc_vtb_t *vtb, int at);
void		sc_vtb_putc(sc_vtb_t *vtb, int at, int c, int a);
uint16_t	*sc_vtb_putchar(sc_vtb_t *vtb, uint16_t *p, int c, int a);
int		sc_vtb_pos(sc_vtb_t *vtb, int pos, int offset);

#define		sc_vtb_tail(vtb)	((vtb)->vtb_tail)
#define		sc_vtb_rows(vtb)	((vtb)->vtb_rows)
#define		sc_vtb_cols(vtb)	((vtb)->vtb_cols)

void		sc_vtb_copy(sc_vtb_t *vtb1, int from, sc_vtb_t *vtb2, int to,
			    int count);
void		sc_vtb_append(sc_vtb_t *vtb1, int from, sc_vtb_t *vtb2,
			      int count);
void		sc_vtb_seek(sc_vtb_t *vtb, int pos);
void		sc_vtb_erase(sc_vtb_t *vtb, int at, int count, int c, int attr);
void		sc_vtb_move(sc_vtb_t *vtb, int from, int to, int count);
void		sc_vtb_delete(sc_vtb_t *vtb, int at, int count, int c, int attr);
void		sc_vtb_ins(sc_vtb_t *vtb, int at, int count, int c, int attr);

/* sysmouse.c */
int		sysmouse_event(mouse_info_t *info);

/* scterm.c */
void		sc_move_cursor(scr_stat *scp, int x, int y);
void		sc_clear_screen(scr_stat *scp);
int		sc_term_add(sc_term_sw_t *sw);
int		sc_term_remove(sc_term_sw_t *sw);
sc_term_sw_t	*sc_term_match(char *name);
sc_term_sw_t	*sc_term_match_by_number(int index);

/* machine dependent functions */
#if 0
sc_softc_t	*sc_find_softc(struct video_adapter *adp, struct keyboard *kbd);
#endif

#endif /* !_DEV_SYSCONS_SYSCONS_H_ */

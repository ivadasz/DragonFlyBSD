/* Copyright (c) Stichting Mathematisch Centrum, Amsterdam, 1985. */
/* hack.pager.c - version 1.0.3 */
/* $FreeBSD: src/games/hack/hack.pager.c,v 1.7 1999/11/16 02:57:09 billf Exp $ */
/* $DragonFly: src/games/hack/hack.pager.c,v 1.4 2006/08/21 19:45:32 pavalos Exp $ */

/* This file contains the command routine dowhatis() and a pager. */
/* Also readmail() and doshell(), and generally the things that
   contact the outside world. */

#include	<sys/types.h>
#include	<sys/signal.h>
#include "hack.h"
extern int CO, LI;	/* usually COLNO and ROWNO+2 */
extern char *CD;
extern char quitchars[];

static void	intruph(int);
static void	page_more(FILE *, int);

int
dowhatis(void)
{
	FILE *fp;
	char bufr[BUFSZ+6];
	char *buf = &bufr[6], *ep, q;

	if(!(fp = fopen(DATAFILE, "r")))
		pline("Cannot open data file!");
	else {
		pline("Specify what? ");
		q = readchar();
		if(q != '\t')
		while(fgets(buf,BUFSZ,fp))
		    if(*buf == q) {
			ep = index(buf, '\n');
			if(ep) *ep = 0;
			/* else: bad data file */
			/* Expand tab 'by hand' */
			if(buf[1] == '\t'){
				buf = bufr;
				buf[0] = q;
				strncpy(buf+1, "       ", 7);
			}
			pline(buf);
			if(ep[-1] == ';') {
				pline("More info? ");
				if(readchar() == 'y') {
					page_more(fp,1); /* does fclose() */
					return(0);
				}
			}
			fclose(fp); 	/* kopper@psuvax1 */
			return(0);
		    }
		pline("I've never heard of such things.");
		fclose(fp);
	}
	return(0);
}

/* make the paging of a file interruptible */
static int got_intrup;

static void
intruph(__unused int unused)
{
	got_intrup++;
}

/* simple pager, also used from dohelp() */
/* strip = nr of chars to be stripped from each line (0 or 1) */
static void
page_more(FILE *fp, int strip)
{
	char *bufr, *ep;
	sig_t prevsig = signal(SIGINT, intruph);

	set_pager(0);
	bufr = (char *) alloc((unsigned) CO);
	bufr[CO-1] = 0;
	while(fgets(bufr,CO-1,fp) && (!strip || *bufr == '\t') && !got_intrup){
		ep = index(bufr, '\n');
		if(ep)
			*ep = 0;
		if(page_line(bufr+strip)) {
			set_pager(2);
			goto ret;
		}
	}
	set_pager(1);
ret:
	free(bufr);
	fclose(fp);
	signal(SIGINT, prevsig);
	got_intrup = 0;
}

static boolean whole_screen = TRUE;
#define	PAGMIN	12	/* minimum # of lines for page below level map */

void
set_whole_screen(void)	/* called in termcap as soon as LI is known */
{
	whole_screen = (LI-ROWNO-2 <= PAGMIN || !CD);
}

#ifdef NEWS
bool
readnews(void)
{
	int ret;

	whole_screen = TRUE;	/* force a docrt(), our first */
	ret = page_file(NEWS, TRUE);
	set_whole_screen();
	return(ret);		/* report whether we did docrt() */
}
#endif /* NEWS */

void
set_pager(int mode)	/* 0: open  1: wait+close  2: close */
{
	static boolean so;
	if(mode == 0) {
		if(!whole_screen) {
			/* clear topline */
			clrlin();
			/* use part of screen below level map */
			curs(1, ROWNO+4);
		} else {
			cls();
		}
		so = flags.standout;
		flags.standout = 1;
	} else {
		if(mode == 1) {
			curs(1, LI);
			more();
		}
		flags.standout = so;
		if(whole_screen)
			docrt();
		else {
			curs(1, ROWNO+4);
			cl_eos();
		}
	}
}

bool
page_line(const char *s)	/* returns 1 if we should quit */
{
	if(cury == LI-1) {
		if(!*s)
			return(0);	/* suppress blank lines at top */
		putchar('\n');
		cury++;
		cmore("q\033");
		if(morc) {
			morc = 0;
			return(1);
		}
		if(whole_screen)
			cls();
		else {
			curs(1, ROWNO+4);
			cl_eos();
		}
	}
	puts(s);
	cury++;
	return(0);
}

/*
 * Flexible pager: feed it with a number of lines and it will decide
 * whether these should be fed to the pager above, or displayed in a
 * corner.
 * Call:
 *	cornline(0, title or 0)	: initialize
 *	cornline(1, text)	: add text to the chain of texts
 *	cornline(2, morcs)	: output everything and cleanup
 *	cornline(3, 0)		: cleanup
 */

void
cornline(int mode, const char *text)
{
	static struct line {
		struct line *next_line;
		char *line_text;
	} *texthead, *texttail;
	static int maxlen;
	static int linect;
	struct line *tl;

	if(mode == 0) {
		texthead = 0;
		maxlen = 0;
		linect = 0;
		if(text) {
			cornline(1, text);	/* title */
			cornline(1, "");	/* blank line */
		}
		return;
	}

	if(mode == 1) {
	    int len;

	    if(!text) return;	/* superfluous, just to be sure */
	    linect++;
	    len = strlen(text);
	    if(len > maxlen)
		maxlen = len;
	    tl = (struct line *)
		alloc((unsigned)(len + sizeof(struct line) + 1));
	    tl->next_line = 0;
	    tl->line_text = (char *)(tl + 1);
	    strcpy(tl->line_text, text);
	    if(!texthead)
		texthead = tl;
	    else
		texttail->next_line = tl;
	    texttail = tl;
	    return;
	}

	/* --- now we really do it --- */
	if(mode == 2 && linect == 1)			    /* topline only */
		pline(texthead->line_text);
	else
	if(mode == 2) {
	    int curline, lth;

	    if(flags.toplin == 1) more();	/* ab@unido */
	    remember_topl();

	    lth = CO - maxlen - 2;		   /* Use full screen width */
	    if (linect < LI && lth >= 10) {		     /* in a corner */
		home ();
		cl_end ();
		flags.toplin = 0;
		curline = 1;
		for (tl = texthead; tl; tl = tl->next_line) {
		    curs (lth, curline);
		    if(curline > 1)
			cl_end ();
		    putsym(' ');
		    putstr (tl->line_text);
		    curline++;
		}
		curs (lth, curline);
		cl_end ();
		cmore (text);
		home ();
		cl_end ();
		docorner (lth, curline-1);
	    } else {					/* feed to pager */
		set_pager(0);
		for (tl = texthead; tl; tl = tl->next_line) {
		    if (page_line (tl->line_text)) {
			set_pager(2);
			goto cleanup;
		    }
		}
		if(text) {
			cgetret(text);
			set_pager(2);
		} else
			set_pager(1);
	    }
	}

cleanup:
	while((tl = texthead)) {
		texthead = tl->next_line;
		free((char *) tl);
	}
}

int
dohelp(void)
{
	char c;

	pline ("Long or short help? ");
	while (((c = readchar ()) != 'l') && (c != 's') && !index(quitchars,c))
		bell ();
	if (!index(quitchars, c))
		page_file((c == 'l') ? HELP : SHELP, FALSE);
	return(0);
}

/* return: 0 - cannot open fnam; 1 - otherwise */
bool
page_file(const char *fnam, bool silent)
{
#ifdef DEF_PAGER			/* this implies that UNIX is defined */
      {
	/* use external pager; this may give security problems */

	int fd = open(fnam, 0);

	if(fd < 0) {
		if(!silent) pline("Cannot open %s.", fnam);
		return(0);
	}
	if(child(1)){
		extern char *catmore;

		/* Now that child() does a setuid(getuid()) and a chdir(),
		   we may not be able to open file fnam anymore, so make
		   it stdin. */
		close(0);
		if(dup(fd)) {
			if(!silent) printf("Cannot open %s as stdin.\n", fnam);
		} else {
			execl(catmore, "page", NULL);
			if(!silent) printf("Cannot exec %s.\n", catmore);
		}
		exit(1);
	}
	close(fd);
      }
#else /* DEF_PAGER */
      {
	FILE *f;			/* free after Robert Viduya */

	if ((f = fopen (fnam, "r")) == NULL) {
		if(!silent) {
			home(); perror (fnam); flags.toplin = 1;
			pline ("Cannot open %s.", fnam);
		}
		return(0);
	}
	page_more(f, 0);
      }
#endif /* DEF_PAGER */

	return(1);
}

#ifdef UNIX
#ifdef SHELL
int
dosh(void)
{
char *str;
	if(child(0)) {
		if((str = getenv("SHELL")))
			execl(str, str, NULL);
		else
			execl("/bin/sh", "sh", NULL);
		pline("sh: cannot execute.");
		exit(1);
	}
	return(0);
}
#endif /* SHELL */

#ifdef NOWAITINCLUDE
union wait {		/* used only for the cast  (union wait *) 0  */
	int w_status;
	struct {
		unsigned short w_Termsig:7;
		unsigned short w_Coredump:1;
		unsigned short w_Retcode:8;
	} w_T;
};

#else
#include	<sys/wait.h>
#endif /* NOWAITINCLUDE */

bool
child(bool wt)
{
	int status;
	int f;

	f = fork();
	if(f == 0){		/* child */
		settty(NULL);		/* also calls end_screen() */
		/* revoke */
		setgid(getgid());
#ifdef CHDIR
		chdir(getenv("HOME"));
#endif /* CHDIR */
		return(1);
	}
	if(f == -1) {	/* cannot fork */
		pline("Fork failed. Try again.");
		return(0);
	}
	/* fork succeeded; wait for child to exit */
	signal(SIGINT,SIG_IGN);
	signal(SIGQUIT,SIG_IGN);
	wait(&status);
	gettty();
	setftty();
	signal(SIGINT,done1);
#ifdef WIZARD
	if(wizard) signal(SIGQUIT,SIG_DFL);
#endif /* WIZARD */
	if(wt) getret();
	docrt();
	return(0);
}
#endif /* UNIX */

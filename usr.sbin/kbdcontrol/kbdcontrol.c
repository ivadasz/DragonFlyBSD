/*-
 * Copyright (c) 1994-1995 Søren Schmidt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    in this position and unchanged.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
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
 * $FreeBSD: src/usr.sbin/kbdcontrol/kbdcontrol.c,v 1.30.2.2 2001/06/08 18:27:32 sobomax Exp $
 */
#include <sys/kbio.h>

#include <ctype.h>
#include <err.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <machine/console.h>

#include <kbdmap.h>

char fkey_table[96][MAXFK] = {
/* 01-04 */	"\033[M", "\033[N", "\033[O", "\033[P",
/* 05-08 */	"\033[Q", "\033[R", "\033[S", "\033[T",
/* 09-12 */	"\033[U", "\033[V", "\033[W", "\033[X",
/* 13-16 */	"\033[Y", "\033[Z", "\033[a", "\033[b",
/* 17-20 */	"\033[c", "\033[d", "\033[e", "\033[f",
/* 21-24 */	"\033[g", "\033[h", "\033[i", "\033[j",
/* 25-28 */	"\033[k", "\033[l", "\033[m", "\033[n",
/* 29-32 */	"\033[o", "\033[p", "\033[q", "\033[r",
/* 33-36 */	"\033[s", "\033[t", "\033[u", "\033[v",
/* 37-40 */	"\033[w", "\033[x", "\033[y", "\033[z",
/* 41-44 */	"\033[@", "\033[[", "\033[\\","\033[]",
/* 45-48 */     "\033[^", "\033[_", "\033[`", "\033[{",
/* 49-52 */	"\033[H", "\033[A", "\033[I", "-"     ,
/* 53-56 */	"\033[D", "\033[E", "\033[C", "+"     ,
/* 57-60 */	"\033[F", "\033[B", "\033[G", "\033[L",
/* 61-64 */     "\177",   "\033[J", "\033[~", "\033[}",
/* 65-68 */	""      , ""      , ""      , ""      ,
/* 69-72 */	""      , ""      , ""      , ""      ,
/* 73-76 */	""      , ""      , ""      , ""      ,
/* 77-80 */	""      , ""      , ""      , ""      ,
/* 81-84 */	""      , ""      , ""      , ""      ,
/* 85-88 */	""      , ""      , ""      , ""      ,
/* 89-92 */	""      , ""      , ""      , ""      ,
/* 93-96 */	""      , ""      , ""      , ""      ,
	};

const int	delays[]  = {250, 500, 750, 1000};
const int	repeats[] = { 34,  38,  42,  46,  50,  55,  59,  63,
			      68,  76,  84,  92, 100, 110, 118, 126,
			     136, 152, 168, 184, 200, 220, 236, 252,
			     272, 304, 336, 368, 400, 440, 472, 504};
const int	ndelays = (sizeof(delays) / sizeof(int));
const int	nrepeats = (sizeof(repeats) / sizeof(int));

static void	mux_keyboard(u_int op, char *kbd);
static void	usage(void);

static const char *
nextarg(int ac, char **av, int *indp, int oc)
{
	if (*indp < ac)
		return(av[(*indp)++]);
	warnx("option requires two arguments -- %c", oc);
	usage();
	return("");
}

static void
load_keymap(char *opt, int dumponly)
{
	keymap_t keymap;
	accentmap_t accentmap;
	char *path, *cp;

	path = kbdmap_parse(opt, &keymap, &accentmap);
	if (path == NULL) {
		warn("failed to parse keymap %s", opt);
		return;
	}
	if (dumponly) {
		/* fix up the filename to make it a valid C identifier */
		for (cp = opt; *cp; cp++)
			if (!isalpha(*cp) && !isdigit(*cp)) *cp = '_';
		printf("/*\n"
		       " * Automatically generated from %s.\n"
	               " * DO NOT EDIT!\n"
		       " */\n", path);
		free(path);
		kbdmap_dump_key_definition(opt, &keymap);
		kbdmap_dump_accent_definition(opt, &accentmap);
		return;
	}
	free(path);
	if ((keymap.n_keys > 0) && (ioctl(0, PIO_KEYMAP, &keymap) < 0)) {
		warn("setting keymap");
		return;
	}
	if ((accentmap.n_accs > 0) 
		&& (ioctl(0, PIO_DEADKEYMAP, &accentmap) < 0)) {
		warn("setting accentmap");
		return;
	}
}

static void
print_keymap(int hex)
{
	keymap_t keymap;
	accentmap_t accentmap;

	if (ioctl(0, GIO_KEYMAP, &keymap) < 0)
		err(1, "getting keymap");
	if (ioctl(0, GIO_DEADKEYMAP, &accentmap) < 0)
		memset(&accentmap, 0, sizeof(accentmap));
	kbdmap_dump(stdout, &keymap, &accentmap, hex);
}


static void
load_default_functionkeys(void)
{
	fkeyarg_t fkey;
	int i;

	for (i=0; i<NUM_FKEYS; i++) {
		fkey.keynum = i;
		strcpy(fkey.keydef, fkey_table[i]);
		fkey.flen = strlen(fkey_table[i]);
		if (ioctl(0, SETFKEY, &fkey) < 0)
			warn("setting function key");
	}
}

static void
set_functionkey(char *keynumstr, const char *string)
{
	fkeyarg_t fkey;

	if (!strcmp(keynumstr, "load") && !strcmp(string, "default")) {
		load_default_functionkeys();
		return;
	}
	fkey.keynum = atoi(keynumstr);
	if (fkey.keynum < 1 || fkey.keynum > NUM_FKEYS) {
		warnx("function key number must be between 1 and %d",
			NUM_FKEYS);
		return;
	}
	if ((fkey.flen = strlen(string)) > MAXFK) {
		warnx("function key string too long (%d > %d)",
			fkey.flen, MAXFK);
		return;
	}
	strcpy(fkey.keydef, string);
	fkey.keynum -= 1;
	if (ioctl(0, SETFKEY, &fkey) < 0)
		warn("setting function key");
}


static void
set_bell_values(char *opt)
{
	int bell, duration = 0, pitch = 0;

	bell = 0;
	if (!strncmp(opt, "quiet.", 6)) {
		bell = 2;
		opt += 6;
	}
	if (!strcmp(opt, "visual"))
		bell |= 1;
	else if (!strcmp(opt, "normal"))
		duration = 5, pitch = 800;
	else if (!strcmp(opt, "off"))
		duration = 0, pitch = 0;
	else {
		char		*v1;

		bell = 0;
		duration = strtol(opt, &v1, 0);
		if ((duration < 0) || (*v1 != '.'))
			goto badopt;
		opt = ++v1;
		pitch = strtol(opt, &v1, 0);
		if ((pitch < 0) || (*opt == '\0') || (*v1 != '\0')) {
badopt:
			warnx("argument to -b must be duration.pitch or [quiet.]visual|normal|off");
			return;
		}
		if (pitch != 0)
			pitch = 1193182 / pitch;	/* in Hz */
		duration /= 10;	/* in 10 m sec */
	}

	ioctl(0, CONS_BELLTYPE, &bell);
	if ((bell & ~2) == 0)
		fprintf(stderr, "[=%d;%dB", pitch, duration);
}


static void
set_keyrates(char *opt)
{
	int arg[2];
	int repeat;
	int delay;
	int r, d;

	if (!strcmp(opt, "slow")) {
		delay = 1000, repeat = 500;
		d = 3, r = 31;
	} else if (!strcmp(opt, "normal")) {
		delay = 500, repeat = 125;
		d = 1, r = 15;
	} else if (!strcmp(opt, "fast")) {
		delay = repeat = 0;
		d = r = 0;
	} else {
		int		n;
		char		*v1;

		delay = strtol(opt, &v1, 0);
		if ((delay < 0) || (*v1 != '.'))
			goto badopt;
		opt = ++v1;
		repeat = strtol(opt, &v1, 0);
		if ((repeat < 0) || (*opt == '\0') || (*v1 != '\0')) {
badopt:
			warnx("argument to -r must be delay.repeat or slow|normal|fast");
			return;
		}
		for (n = 0; n < ndelays - 1; n++)
			if (delay <= delays[n])
				break;
		d = n;
		for (n = 0; n < nrepeats - 1; n++)
			if (repeat <= repeats[n])
				break;
		r = n;
	}

	arg[0] = delay;
	arg[1] = repeat;
	if (ioctl(0, KDSETREPEAT, arg)) {
		if (ioctl(0, KDSETRAD, (d << 5) | r))
			warn("setting keyboard rate");
	}
}


static void
set_history(char *opt)
{
	int size;

	size = atoi(opt);
	if ((*opt == '\0') || size < 0) {
		warnx("argument must be a positive number");
		return;
	}
	if (ioctl(0, CONS_HISTORY, &size) == -1)
		warn("setting history buffer size");
}

static const char *
get_kbd_type_name(int type)
{
	static struct {
		int type;
		const char *name;
	} name_table[] = {
		{ KB_84,	"AT 84" },
		{ KB_101,	"AT 101/102" },
		{ KB_OTHER,	"generic" },
	};
	size_t i;

	for (i = 0; i < sizeof(name_table)/sizeof(name_table[0]); ++i) {
		if (type == name_table[i].type)
			return name_table[i].name;
	}
	return "unknown";
}

static void
show_kbd_info(void)
{
	keyboard_info_t info;

	if (ioctl(0, KDGKBINFO, &info) == -1) {
		warn("unable to obtain keyboard information");
		return;
	}
	printf("kbd%d:\n", info.kb_index);
	printf("    %.*s%d, type:%s (%d)\n",
		(int)sizeof(info.kb_name), info.kb_name, info.kb_unit,
		get_kbd_type_name(info.kb_type), info.kb_type);
}


static void
set_keyboard(char *device)
{
	keyboard_info_t info;
	int fd;

	fd = open(device, O_RDONLY);
	if (fd < 0) {
		warn("cannot open %s", device);
		return;
	}
	if (ioctl(fd, KDGKBINFO, &info) == -1) {
		warn("unable to obtain keyboard information");
		close(fd);
		return;
	}
	/*
	 * The keyboard device driver won't release the keyboard by
	 * the following ioctl, but it automatically will, when the device 
	 * is closed.  So, we don't check error here.
	 */
	ioctl(fd, CONS_RELKBD, 0);
	close(fd);
#if 1
	printf("kbd%d\n", info.kb_index);
	printf("    %.*s%d, type:%s (%d)\n",
		(int)sizeof(info.kb_name), info.kb_name, info.kb_unit,
		get_kbd_type_name(info.kb_type), info.kb_type);
#endif

	if (ioctl(0, CONS_SETKBD, info.kb_index) == -1)
		warn("unable to set keyboard");
}


static void
release_keyboard(void)
{
	keyboard_info_t info;

	/*
	 * If stdin is not associated with a keyboard, the following ioctl
	 * will fail.
	 */
	if (ioctl(0, KDGKBINFO, &info) == -1) {
		warn("unable to obtain keyboard information");
		return;
	}
#if 1
	printf("kbd%d\n", info.kb_index);
	printf("    %.*s%d, type:%s (%d)\n",
		(int)sizeof(info.kb_name), info.kb_name, info.kb_unit,
		get_kbd_type_name(info.kb_type), info.kb_type);
#endif
	if (ioctl(0, CONS_RELKBD, 0) == -1)
		warn("unable to release the keyboard");
}

static void
mux_keyboard(u_int op, char *kbd)
{
	keyboard_info_t	info;
	char		*unit, *ep;

	/*
	 * If stdin is not associated with a keyboard, the following ioctl
	 * will fail.
	 */
	if (ioctl(0, KDGKBINFO, &info) == -1) {
		warn("unable to obtain keyboard information");
		return;
	}
#if 1
	printf("kbd%d\n", info.kb_index);
	printf("    %.*s%d, type:%s (%d)\n",
		(int)sizeof(info.kb_name), info.kb_name, info.kb_unit,
		get_kbd_type_name(info.kb_type), info.kb_type);
#endif
	/*
	 * split kbd into name and unit. find the right most part of the
	 * kbd string that consist of only digits.
	 */

	memset(&info, 0, sizeof(info));

	info.kb_unit = -1;
	ep = kbd - 1;

	do {
		unit = strpbrk(ep + 1, "0123456789");
		if (unit != NULL) {
			info.kb_unit = strtol(unit, &ep, 10);
			if (*ep != '\0')
				info.kb_unit = -1;
		}
	} while (unit != NULL && info.kb_unit == -1);

	if (info.kb_unit == -1) {
		warnx("unable to find keyboard driver unit in '%s'", kbd);
		return;
	}

	if (unit == kbd) {
		warnx("unable to find keyboard driver name in '%s'", kbd);
		return;
	}
	if (unit - kbd >= (int) sizeof(info.kb_name)) {
		warnx("keyboard name '%s' is too long", kbd);
		return;
	}

	strncpy(info.kb_name, kbd, unit - kbd);

	/*
	 * If stdin is not associated with a kbdmux(4) keyboard, the following
	 * ioctl will fail.
	 */

	if (ioctl(0, op, &info) == -1)
		warn("unable to (un)mux the keyboard");
}

static void
usage(void)
{
	fprintf(stderr, "%s\n%s\n%s\n",
"usage: kbdcontrol [-dFKix] [-A name] [-a name] [-b duration.pitch | [quiet.]belltype]",
"                  [-r delay.repeat | speed] [-l mapfile] [-f # string]",
"                  [-h size] [-k device] [-L mapfile]");
	exit(1);
}


int
main(int argc, char **argv)
{
	int		opt, hex = 0;

	while((opt = getopt(argc, argv, "A:a:b:df:h:iKk:Fl:L:r:x")) != -1)
		switch(opt) {
			case 'A':
			case 'a':
				mux_keyboard((opt == 'A')? KBRELKBD : KBADDKBD, optarg);
				break;
			case 'b':
				set_bell_values(optarg);
				break;
			case 'd':
				print_keymap(hex);
				break;
			case 'l':
				load_keymap(optarg, 0);
				break;
			case 'L':
				load_keymap(optarg, 1);
				break;
			case 'f':
				set_functionkey(optarg,
					nextarg(argc, argv, &optind, 'f'));
				break;
			case 'F':
				load_default_functionkeys();
				break;
			case 'h':
				set_history(optarg);
				break;
			case 'i':
				show_kbd_info();
				break;
			case 'K':
				release_keyboard();
				break;
			case 'k':
				set_keyboard(optarg);
				break;
			case 'r':
				set_keyrates(optarg);
				break;
			case 'x':
				hex = 1;
				break;
			default:
				usage();
		}
	if ((optind != argc) || (argc == 1))
		usage();
	exit(0);
}

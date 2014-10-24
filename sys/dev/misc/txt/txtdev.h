/*-
 * Copyright (c) 2014 Imre Vadász <imre@vdsz.com>
 * All rights reserved.
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
 */

#ifndef _DEV_TXT_H_
#define	_DEV_TXT_H_

#include <sys/malloc.h>

enum {
	TXTDEV_IS_EARLY = 1,
	TXTDEV_IS_VGA = 2,
	TXTDEV_REPLACE_VGA = 4,
	TXTDEV_IS_DEAD = 8,
};

enum {
	TXTDEV_CURSOR_BLINK,
	TXTDEV_CURSOR_BLOCK,
};

struct txtmode {
	int txt_columns;
	int txt_rows;
};

typedef int txtdev_getmode(void *cookie, struct txtmode *mode);
typedef int txtdev_setmode(void *cookie, struct txtmode *mode);
typedef int txtdev_putchars(void *cookie, int col, int row, uint16_t *buf,
			    int len);
typedef int txtdev_getchars(void *cookie, int col, int row, uint16_t *buf,
			    int len);
typedef int txtdev_setcursor(void *cookie, int pos);
typedef int txtdev_getcursor(void *cookie, int *pos);
typedef int txtdev_setcurmode(void *cookie, int mode);
typedef char *txtdev_getname(void *cookie);
typedef void txtdev_restore(void *cookie, int emergency);
typedef void txtdev_powersave(void *cookie, int on);

struct txtdev_sw {
	txtdev_getmode *getmode;
	txtdev_setmode *setmode;
	txtdev_putchars *putchars;
	txtdev_getchars *getchars;
	txtdev_setcursor *setcursor;
	txtdev_getcursor *getcursor;
	txtdev_setcurmode *setcurmode;
	txtdev_getname *getname;
	txtdev_restore *restore;
	txtdev_powersave *powersave;
};

typedef void txtdev_release_cb(void *conscookie, void *cookie);

int register_txtdev(void *cookie, struct txtdev_sw *sw, int how);
int available_txtdev(void);
int acquire_txtdev(void **cookie, struct txtdev_sw **sw, txtdev_release_cb *cb,
		   void *cc);
int release_txtdev(void *cookie, struct txtdev_sw *sw);

/* XXX */

#endif /* !_DEV_TXT_H_ */

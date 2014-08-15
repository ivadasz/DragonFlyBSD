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

//#include "opt_txtdev.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/priv.h>
#include <sys/kernel.h>

#include "txtdev.h"

int	sc_set_txtdev(void *cookie, struct txtdev_sw *sw);
int	sc_replace_txtdev(void *cookie, struct txtdev_sw *sw, void *oldcookie);

static int myflags = 0;
static void *mycookie = NULL;
static struct txtdev_sw *mysw = NULL;

/* XXX needs a corresponding unregister_txtdev method */
int
register_txtdev(void *cookie, struct txtdev_sw *sw, int how)
{
	void *oldcookie = mycookie;
	int replacing = 0;

	if (sw == NULL)
		return 1;

	/* Only allow replacing early attachements */
	if ((myflags & TXTDEV_IS_EARLY) != 0) {
		if ((myflags & TXTDEV_IS_VGA) != 0 &&
		    (how & TXTDEV_REPLACE_VGA) != 0) {
			mycookie = NULL;
			mysw = NULL;
			myflags = 0;
			replacing = 1;
		}
	}

	if (mycookie != NULL || mysw != NULL)
		return 1;

	mycookie = cookie;
	mysw = sw;
	myflags = how;

	char *dummy = "Hallo, World";
	uint16_t buf[128];
	int i;
	for (i = 0; i < strlen(dummy); i++)
		buf[i] = 0x2600 | (dummy[i] & 0x00ff);
	mysw->putchars(mycookie, 5, 30, buf, strlen(dummy));

	if (replacing)
		sc_replace_txtdev(mycookie, mysw, oldcookie);
	else
		sc_set_txtdev(mycookie, mysw);

	return 0;
}

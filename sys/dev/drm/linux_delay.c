/*
 * Copyright (c) 2026 Imre Vadász
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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

#include <linux/delay.h>

#include <sys/systm.h>
#include <sys/systimer.h>

static void
msleep_systimer(systimer_t info, int in_ipi __unused,
    struct intrframe *frame __unused)
{
	lwkt_schedule(info->data);
}

void
msleep(unsigned int msecs)
{
	if (msecs >= 10) {
		static int dummy;
		int delay = MAX(msecs*hz/1000, 1);

		tsleep(&dummy, 0, "linux_msleep", delay);
	} else {
		struct systimer info;

		crit_enter();
		systimer_init_oneshot(&info, msleep_systimer, curthread,
		    msecs*1000);
		lwkt_deschedule_self(curthread);
		crit_exit();
		lwkt_switch();
		systimer_del(&info); /* make sure it's gone */
	}
}

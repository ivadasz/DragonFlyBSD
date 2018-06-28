/*
 * Copyright (c) 2014 François Tigeot
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

#ifndef _LINUX_DELAY_H_
#define _LINUX_DELAY_H_

#include <linux/jiffies.h>
#include <sys/systm.h>
#include <sys/systimer.h>
#include <sys/thread2.h>

static void
msleep_wakeup(systimer_t info, int in_ipi, struct intrframe *frame)
{
	lwkt_schedule(info->data);
}

static inline void msleep(unsigned int msecs)
{
	static int dummy;

	if (msecs < (1000+hz-1)/hz) {
		thread_t td = curthread;
		struct systimer info;

		crit_enter_quick(td);
		systimer_init_oneshot(&info, msleep_wakeup, td, msecs*1000);
		lwkt_deschedule_self(td);
		crit_exit_quick(td);
		lwkt_switch();
		systimer_del(&info);
	} else {
		int delay = MAX(msecs*hz/1000, 1);

		tsleep(&dummy, 0, "linux_msleep", delay);
	}
}

static __inline void
mdelay(unsigned long msecs)
{
	int loops = msecs;
	while (loops--)
		DELAY(1000);
}

static inline void ndelay(unsigned long x)
{
	DELAY(howmany(x, 1000));
}

static __inline void
usleep_range(unsigned long min, unsigned long max)
{
	if (min < 20) {
		if (max > min)
			DELAY((max + min) / 2);
		else
			DELAY(min);
	} else {
		thread_t td = curthread;
		struct systimer info;
		unsigned long t = max > min ? ((max + min)/2) : min;

		if (t <= 100)
			cpu_mwait_cx_io_wait(mycpuid);
		crit_enter_quick(td);
		systimer_init_oneshot(&info, msleep_wakeup, td, t);
		lwkt_deschedule_self(td);
		crit_exit_quick(td);
		lwkt_switch();
		systimer_del(&info);
		if (t <= 100)
			cpu_mwait_cx_io_done(mycpuid);
	}
}

#endif	/* _LINUX_DELAY_H_ */

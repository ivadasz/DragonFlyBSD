/*
 * Copyright (c) 2006 The DragonFly Project.  All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Matthew Dillon <dillon@backplane.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/systimer.h>
#include <sys/sysctl.h>
#include <sys/signal.h>
#include <sys/interrupt.h>
#include <sys/time.h>
#include <machine/cpu.h>
#include <machine/clock.h>
#include <machine/globaldata.h>
#include <machine/md_var.h>

#include <sys/thread2.h>

#include <unistd.h>
#include <signal.h>

#if 0
int adjkerntz;
int wall_cmos_clock = 0;

static cothread_t vktimer_cotd;
static int vktimer_running;
static sysclock_t vktimer_target;
static struct timespec vktimer_ts;
static sysclock_t vktimer_reload[MAXCPU];

extern int use_precise_timer;

/*
 * SYSTIMER IMPLEMENTATION
 */
static sysclock_t vkernel_timer_get_timecount(void);
static void vkernel_timer_construct(struct cputimer *timer, sysclock_t oclock);
static void vktimer_thread(cothread_t cotd);

static struct cputimer vkernel_cputimer = {
	.next		= SLIST_ENTRY_INITIALIZER,
	.name		= "VKERNEL",
	.pri		= CPUTIMER_PRI_VKERNEL,
	.type		= CPUTIMER_VKERNEL,
	.count		= vkernel_timer_get_timecount,
	.fromhz		= cputimer_default_fromhz,
	.fromus		= cputimer_default_fromus,
	.construct	= vkernel_timer_construct,
	.destruct	= cputimer_default_destruct,
	.freq		= 1000000
};

static void	vktimer_intr_reload(struct cputimer_intr *, sysclock_t);
static void	vktimer_intr_initclock(struct cputimer_intr *, boolean_t);

static struct cputimer_intr vkernel_cputimer_intr = {
	.freq = 1000000,
	.reload = vktimer_intr_reload,
	.enable = cputimer_intr_default_enable,
	.config = cputimer_intr_default_config,
	.restart = cputimer_intr_default_restart,
	.pmfixup = cputimer_intr_default_pmfixup,
	.initclock = vktimer_intr_initclock,
	.pcpuhand = NULL,
	.next = SLIST_ENTRY_INITIALIZER,
	.name = "vkernel",
	.type = CPUTIMER_INTR_VKERNEL,
	.prio = CPUTIMER_INTR_PRIO_VKERNEL,
	.caps = CPUTIMER_INTR_CAP_NONE,
	.priv = NULL
};

/*
 * Initialize the systimer subsystem, called from MI code in early boot.
 */
static void
cpu_initclocks(void *arg __unused)
{
	kprintf("initclocks\n");
	cputimer_intr_register(&vkernel_cputimer_intr);
	cputimer_intr_select(&vkernel_cputimer_intr, 0);

	cputimer_register(&vkernel_cputimer);
	cputimer_select(&vkernel_cputimer, 0);
}
SYSINIT(clocksvk, SI_BOOT2_CLOCKREG, SI_ORDER_FIRST, cpu_initclocks, NULL);

/*
 * Constructor to initialize timer->base and get an initial count.
 */
static void
vkernel_timer_construct(struct cputimer *timer, sysclock_t oclock)
{
	timer->base = 0;
	timer->base = oclock - vkernel_timer_get_timecount();
}

/*
 * Get the current counter, with 2's complement rollover.
 *
 * NOTE! MPSAFE, possibly no critical section
 */
static sysclock_t
vkernel_timer_get_timecount(void)
{
	struct timespec ts;
	sysclock_t count;

	if (use_precise_timer)
		clock_gettime(CLOCK_MONOTONIC_PRECISE, &ts);
	else
		clock_gettime(CLOCK_MONOTONIC_FAST, &ts);
	count = ts.tv_nsec / 1000;
	count += ts.tv_sec * 1000000;

	return count;
}

/*
 * Initialize the interrupt for our core systimer.  Use the kqueue timer
 * support functions.
 */
static void
vktimer_intr_initclock(struct cputimer_intr *cti __unused,
		       boolean_t selected __unused)
{
	vktimer_target = sys_cputimer->count();

	vktimer_ts.tv_nsec = 1000000000 / 20;
	vktimer_cotd = cothread_create(vktimer_thread, NULL, NULL, "vktimer");
	while (vktimer_running == 0)
		usleep(1000000 / 10);
#if 0
	KKASSERT(kqueue_timer_info == NULL);
	kqueue_timer_info = kqueue_add_timer(vktimer_intr, NULL);
#endif
}

/*
 *
 */
static void
vktimer_sigint(int signo)
{
	/* do nothing, just interrupt */
}

static sysclock_t
vktimer_gettick_us(void)
{
	struct clockinfo info;
	int mib[] = { CTL_KERN, KERN_CLOCKRATE };
	size_t len = sizeof(info);

	if (sysctl(mib, NELEM(mib), &info, &len, NULL, 0) != 0 ||
	    len != sizeof(info)) {
		/* Assume 10 milliseconds (== 100hz) */
		return 1000000 / 100;
	} else if (info.tick < 999999) {
		return info.tick;
	} else {
		/* Assume 10 milliseconds (== 100hz) */
		return 1000000 / 100;
	}
}

static void
vktimer_thread(cothread_t cotd)
{
	struct sigaction sa;
	globaldata_t gscan;
	sysclock_t ticklength_us;

	bzero(&sa, sizeof(sa));
	sa.sa_handler = vktimer_sigint;
	sa.sa_flags |= SA_NODEFER;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);

	ticklength_us = vktimer_gettick_us();
	vktimer_running = 1;
	while (vktimer_cotd == NULL)
		usleep(1000000 / 10);

	for (;;) {
		sysclock_t curtime;
		sysclock_t reload;
		ssysclock_t delta;
		int n;
		uint32_t freq;

		/*
		 * Sleep
		 */
		cothread_sleep(cotd, &vktimer_ts);

rescan:
		freq = sys_cputimer->freq;
		curtime = sys_cputimer->count();
		reload = freq - 1;

		/*
		 * Reset the target
		 */
		for (n = 0; n < ncpus; ++n) {
			gscan = globaldata_find(n);
			delta = vktimer_reload[n] - curtime;
			if (delta <= 0 && TAILQ_FIRST(&gscan->gd_systimerq))
				pthread_kill(ap_tids[n], SIGURG);
			if (delta > 0 && reload > delta)
				reload = delta;
		}
		reload += curtime;
		vktimer_target = reload;

		/*
		 * Check for races
		 */
		reload -= curtime;
		for (n = 0; n < ncpus; ++n) {
			gscan = globaldata_find(n);
			delta = vktimer_reload[n] - curtime;
			if (delta > 0 && reload > delta)
				goto rescan;
		}
		if (sys_cputimer == &vkernel_cputimer &&
                    !use_precise_timer && reload < ticklength_us / 10) {
			/*
			 * Avoid pointless short sleeps, when we only measure
			 * the current time at tick precision.
			 */
			reload = ticklength_us / 10;
		}
		vktimer_ts.tv_nsec = ((uint64_t)reload * 1000000000) / freq;
	}
}

/*
 * Reload the interrupt for our core systimer.  Because the caller's
 * reload calculation can be negatively indexed, we need a minimal
 * check to ensure that a reasonable reload value is selected.
 */
static void
vktimer_intr_reload(struct cputimer_intr *cti __unused, sysclock_t reload)
{
	if (reload >= sys_cputimer->freq)
		reload = sys_cputimer->freq;
	reload += sys_cputimer->count();
	vktimer_reload[mycpu->gd_cpuid] = reload;
	if (vktimer_cotd && (ssysclock_t)(reload - vktimer_target) < 0) {
		while ((sysclock_t)(reload - vktimer_target) < 0)
			reload = atomic_swap_int(&vktimer_target, reload);
		cothread_wakeup(vktimer_cotd, &vktimer_ts);
	}
}

/*
 * pcpu clock interrupt (hard interrupt)
 */
void
vktimer_intr(struct intrframe *frame)
{
	struct globaldata *gd = mycpu;
	sysclock_t sysclock_count;

	sysclock_count = sys_cputimer->count();
	++gd->gd_cnt.v_timer;
	systimer_intr(&sysclock_count, 0, frame);
}
#endif

/*
 * Initialize the time of day register, based on the time base which is, e.g.
 * from a filesystem.
 */
void
inittodr(time_t base)
{
	struct timespec ts;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	ts.tv_sec = tv.tv_sec;
	ts.tv_nsec = tv.tv_usec * 1000;
	set_timeofday(&ts);
}

/*
 * Write system time back to the RTC
 */
void
resettodr(void)
{
}

/*
 * We need to enter a critical section to prevent signals from recursing
 * into pthreads.
 */
void
DELAY(int usec)
{
	crit_enter();
	usleep(usec);
	crit_exit();
}

void
DRIVERSLEEP(int usec)
{
        if (mycpu->gd_intr_nesting_level)
		DELAY(usec);
	else if (1000000 / usec >= hz)
		tsleep(DRIVERSLEEP, 0, "DELAY", 1000000 / usec / hz + 1);
	else
		DELAY(usec);
}

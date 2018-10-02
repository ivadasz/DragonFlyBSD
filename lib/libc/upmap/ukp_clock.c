/*
 * Copyright (c) 2014 The DragonFly Project.  All rights reserved.
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
 */

/*
 * UKP-Optimized clock_gettime().  Use the kpmap after the 10th call.
 */

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/upmap.h>
#include <sys/time.h>
#include <machine/cpufunc.h>
#include <errno.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
/*#include "un-namespace.h"*/
#include "libc_private.h"
#include "upmap.h"

extern int __sys_clock_gettime(clockid_t clock_id, struct timespec *ts);
int __clock_gettime(clockid_t clock_id, struct timespec *ts);
extern int __sys_gettimeofday(struct timeval *tp, struct timezone *tzp);
int __gettimeofday(struct timeval *tp, struct timezone *tzp);

static int fast_clock;
static int fast_count;
static int32_t *version;
static int *upticksp;
static struct timespec *ts_uptime;
static struct timespec *ts_realtime;
static int64_t *tsc_freq;
static uint32_t *tsc_shift;
static uint32_t *timer_base;
static int64_t *freq64_nsec;
static uint32_t *clock_base;
static uint32_t *clock_secs;
static struct timespec *ts_basetime;

static int64_t cputimer_freq;

int
__clock_gettime(clockid_t clock_id, struct timespec *ts)
{
	int res;
	int w;

	if (fast_clock == 0 && fast_count++ >= 10) {
		/* XXX Factor out into a function: */
		__kpmap_map(&version, &fast_clock, UKPTYPE_VERSION);
		__kpmap_map(&upticksp, &fast_clock, KPTYPE_UPTICKS);
		__kpmap_map(&ts_uptime, &fast_clock, KPTYPE_TS_UPTIME);
		__kpmap_map(&ts_realtime, &fast_clock, KPTYPE_TS_REALTIME);
		__kpmap_map(&tsc_freq, &fast_clock, KPTYPE_TSC_FREQ);
		if (*version >= 2) {
			__kpmap_map(&tsc_shift, &fast_clock, KPTYPE_TSC_SHIFT);
			__kpmap_map(&timer_base, &fast_clock, KPTYPE_TIMER_BASE);
			__kpmap_map(&freq64_nsec, &fast_clock, KPTYPE_FREQ_NSEC);
			__kpmap_map(&clock_base, &fast_clock, KPTYPE_CLOCK_BASE);
			__kpmap_map(&clock_secs, &fast_clock, KPTYPE_CLOCK_SECS);
			__kpmap_map(&ts_basetime, &fast_clock, KPTYPE_TS_BASETIME);
		}
		__kpmap_map(NULL, &fast_clock, 0);
		if (*version >= 2) {
			cputimer_freq = *tsc_freq >> *tsc_shift;
			/* XXX Check whether TSC is mpsafe and invariant. */
		}
	}
	if (fast_clock > 0) {
		switch(clock_id) {
		case CLOCK_UPTIME_FAST:
		case CLOCK_MONOTONIC_FAST:
			do {
				w = *upticksp;
				cpu_lfence();
				*ts = ts_uptime[w & 1];
				cpu_lfence();
				w = *upticksp - w;
			} while (w > 1);
			res = 0;
			break;
		case CLOCK_MONOTONIC: {
			uint32_t delta, now;
			uint64_t tsc;
			int64_t conv1, conv2;

			if (*version < 2) {
				res = __sys_clock_gettime(clock_id, ts);
				break;
			}

			conv1 = *freq64_nsec;
			/* This does what tsc_cputimer.count() does. */
			do {
				w = *upticksp;
				cpu_lfence();
				tsc = rdtsc();
				tsc >>= *tsc_shift;
				now = tsc + *timer_base;
				ts->tv_sec = clock_secs[w & 1];
				delta = now - clock_base[w & 1];
				cpu_lfence();
				w = *upticksp - w;
			} while (w > 1);
			conv2 = *freq64_nsec;
			if (conv1 == 0 || conv2 == 0 || cputimer_freq == 0) {
				res = __sys_clock_gettime(clock_id, ts);
				break;
			}
			if (delta >= cputimer_freq) {
				ts->tv_sec += delta / cputimer_freq;
				delta %= cputimer_freq;
			}
			ts->tv_nsec = (conv2 * delta) >> 32;
			res = 0;
			break; }

		case CLOCK_REALTIME_FAST:
		case CLOCK_SECOND:
			do {
				w = *upticksp;
				cpu_lfence();
				*ts = ts_realtime[w & 1];
				cpu_lfence();
				w = *upticksp - w;
			} while (w > 1);

			if (clock_id == CLOCK_SECOND)
				ts->tv_nsec = 0;
			res = 0;
			break;
		case CLOCK_REALTIME: {
			uint32_t delta, now;
			uint64_t tsc;
			int64_t conv1, conv2;
			struct timespec bt;

			if (*version < 2) {
				res = __sys_clock_gettime(clock_id, ts);
				break;
			}

			conv1 = *freq64_nsec;
			/* This does what tsc_cputimer.count() does. */
			do {
				w = *upticksp;
				cpu_lfence();
				tsc = rdtsc();
				tsc >>= *tsc_shift;
				now = tsc + *timer_base;
				ts->tv_sec = clock_secs[w & 1];
				delta = now - clock_base[w & 1];
				bt = ts_basetime[w & 1];
				cpu_lfence();
				w = *upticksp - w;
			} while (w > 1);
			conv2 = *freq64_nsec;
			if (conv1 == 0 || conv2 == 0 || cputimer_freq == 0) {
				res = __sys_clock_gettime(clock_id, ts);
				break;
			}
			if (delta >= cputimer_freq) {
				ts->tv_sec += delta / cputimer_freq;
				delta %= cputimer_freq;
			}
			ts->tv_nsec = (conv2 * delta) >> 32;
			ts->tv_sec += bt.tv_sec;
			ts->tv_nsec += bt.tv_nsec;
			if (ts->tv_nsec > 1000000000) {
				ts->tv_sec++;
				ts->tv_nsec -= 1000000000;
			}
			res = 0;
			break; }
		default:
			res = __sys_clock_gettime(clock_id, ts);
			break;
		}
	} else {
		res = __sys_clock_gettime(clock_id, ts);
	}
	return res;
}

int
__gettimeofday(struct timeval *tp, struct timezone *tzp)
{
	int res;
	int w;

	if (fast_clock == 0 && fast_count++ >= 10) {
		/* XXX Factor out into a function: */
		__kpmap_map(&version, &fast_clock, UKPTYPE_VERSION);
		__kpmap_map(&upticksp, &fast_clock, KPTYPE_UPTICKS);
		__kpmap_map(&ts_uptime, &fast_clock, KPTYPE_TS_UPTIME);
		__kpmap_map(&ts_realtime, &fast_clock, KPTYPE_TS_REALTIME);
		__kpmap_map(&tsc_freq, &fast_clock, KPTYPE_TSC_FREQ);
		if (*version >= 2) {
			__kpmap_map(&tsc_shift, &fast_clock, KPTYPE_TSC_SHIFT);
			__kpmap_map(&timer_base, &fast_clock, KPTYPE_TIMER_BASE);
			__kpmap_map(&freq64_nsec, &fast_clock, KPTYPE_FREQ_NSEC);
			__kpmap_map(&clock_base, &fast_clock, KPTYPE_CLOCK_BASE);
			__kpmap_map(&clock_secs, &fast_clock, KPTYPE_CLOCK_SECS);
			__kpmap_map(&ts_basetime, &fast_clock, KPTYPE_TS_BASETIME);
		}
		__kpmap_map(NULL, &fast_clock, 0);
		if (*version >= 2) {
			cputimer_freq = *tsc_freq >> *tsc_shift;
			/* XXX Check whether TSC is mpsafe and invariant. */
		}
	}
	if (fast_clock > 0 && *version >= 2) {
		if (tp != NULL && tzp == NULL) {
			uint32_t delta, now;
			uint64_t tsc;
			int64_t conv1, conv2;
			struct timespec bt;

			/* XXX Check whether tsc cputimer is used. */

			conv1 = *freq64_nsec;
			/* This does what tsc_cputimer.count() does. */
			do {
				w = *upticksp;
				cpu_lfence();
				tsc = rdtsc();
				tsc >>= *tsc_shift;
				now = tsc + *timer_base;
				tp->tv_sec = clock_secs[w & 1];
				delta = now - clock_base[w & 1];
				bt = ts_basetime[w & 1];
				cpu_lfence();
				w = *upticksp - w;
			} while (w > 1);
			conv2 = *freq64_nsec;
			if (conv1 == 0 || conv2 == 0 || cputimer_freq == 0) {
				return __sys_gettimeofday(tp, tzp);
			}
			if (delta >= cputimer_freq) {
				tp->tv_sec += delta / cputimer_freq;
				delta %= cputimer_freq;
			}
			tp->tv_usec = ((conv2 * delta) >> 32) / 1000;
			tp->tv_sec += bt.tv_sec;
			tp->tv_usec += bt.tv_nsec / 1000;
			if (tp->tv_usec > 1000000) {
				tp->tv_sec++;
				tp->tv_usec -= 1000000;
			}
			res = 0;
		} else {
			res = __sys_gettimeofday(tp, tzp);
		}
	} else {
		res = __sys_gettimeofday(tp, tzp);
	}
	return res;
}

__weak_reference(__clock_gettime, clock_gettime);
__weak_reference(__gettimeofday, gettimeofday);

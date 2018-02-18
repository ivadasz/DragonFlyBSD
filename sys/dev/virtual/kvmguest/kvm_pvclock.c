/*
 * Copyright (c) 2017 The DragonFly Project.  All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Imre Vadász <imre@vdsz.com>
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
 * Routines to handle paravirtualized timers in a KVM virtual machine.
 */

#if 0
#include "opt_kvmguest.h"
#endif

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/kernel.h>

#include <sys/thread2.h>

#include <machine/clock.h>
#include <machine/specialreg.h>

#include <cpu/kvmvirt.h>
#include <vm/pmap.h>

extern int i8254_cputimer_disable;

static sysclock_t kvmclock_cputimer_count(void);
static void kvmclock_cputimer_construct(struct cputimer *, sysclock_t);
static void kvmclock_cputimer_destruct(struct cputimer *cputimer);

static struct cputimer	kvmclock_cputimer = {
    .next	= SLIST_ENTRY_INITIALIZER,
    .name	= "KVMCLOCK",
    .pri	= CPUTIMER_PRI_VMM,
    .type	= CPUTIMER_VMM,
    .count	= kvmclock_cputimer_count,
    .fromhz	= cputimer_default_fromhz,
    .fromus	= cputimer_default_fromus,
    .construct	= kvmclock_cputimer_construct,
    .destruct	= kvmclock_cputimer_destruct,
    .freq	= 0,
};

/* KVMCLOCK stuff */
static struct pvclock_vcpu_time_info bspinfo __aligned(64);
static volatile struct pvclock_vcpu_time_info *infos = &bspinfo;
static int expected, done;

/* XXX Properly deal with initializing after SMP start. */
static void
kvmclock_cputimer_construct(struct cputimer *timer, sysclock_t oldclock)
{
	volatile struct pvclock_vcpu_time_info *info = &infos[0];

	bzero(info, sizeof(*info));
//	kprintf("enabling MSR_KVM_SYSTEM_TIME_NEW on BSP\n");
	wrmsr(MSR_KVM_SYSTEM_TIME_NEW, vtophys(info) | 1ULL);

	/* XXX Check whether we already have SMP running. */

	timer->base = oldclock - timer->count();
}

/* XXX Properly destruct on each cpu. */
static void
kvmclock_cputimer_destruct(struct cputimer *timer)
{
	int i, origcpu;

	/* XXX Technically, this could get called before SMP is up. */
        origcpu = mycpuid;
	for (i = 0; i < ncpus; i++) {
		lwkt_migratecpu(i);
		volatile struct pvclock_vcpu_time_info *info = &infos[mycpuid];
		kprintf("disabling MSR_KVM_SYSTEM_TIME_NEW on cpu %d\n",
		    mycpuid);
		wrmsr(MSR_KVM_SYSTEM_TIME_NEW, 0ULL);
		bzero(info, sizeof(*info));
	}
	lwkt_migratecpu(origcpu);
}

/* Deferred initialization of kvmclock on each cpu. */
static sysclock_t
kvmclock_cputimer_count_wrapper(void)
{
	volatile struct pvclock_vcpu_time_info *info = &infos[mycpuid];
	crit_enter();
	/* Checks whether this cpu was initialized. */
	if (info->tsc_to_system_mul == 0) {
//		kprintf("enabling MSR_KVM_SYSTEM_TIME_NEW on cpu%d\n", mycpuid);
		wrmsr(MSR_KVM_SYSTEM_TIME_NEW, vtophys(info) | 1ULL);
		KKASSERT(info->tsc_to_system_mul != 0);
		int old = atomic_fetchadd_int(&done, 1);
		if (old == expected - 1) {
			atomic_store_rel_ptr(&kvmclock_cputimer.count,
			    (uintptr_t)kvmclock_cputimer_count);
		}
	}
	crit_exit();

	return kvmclock_cputimer_count();
}

extern int naps;
static void
kvmclock_doalloc(void)
{
	int assumed_ncpus = naps + 1;
	struct pvclock_vcpu_time_info *n;

	if (kvmclock_cputimer.freq == 0)
		return;

	kprintf("%s: Allocating for %d cpus\n", __func__, assumed_ncpus);
	n = kmalloc_cachealign(
	    assumed_ncpus * sizeof(struct pvclock_vcpu_time_info),
	    M_DEVBUF, M_WAITOK | M_ZERO);
	expected = assumed_ncpus;
	done = 0;
	crit_enter();
	infos = n;
	kvmclock_cputimer.count = kvmclock_cputimer_count_wrapper;
	crit_exit();
}
SYSINIT(kvmclock_alloc, SI_BOOT2_CPU_TOPOLOGY, SI_ORDER_ANY, kvmclock_doalloc,
	0);

static __inline sysclock_t
kvmclock_cputimer_count(void)
{
	volatile struct pvclock_vcpu_time_info *info = &infos[mycpuid];
	uint64_t stamp, system_time, tsc, time;
	uint32_t v1, v2;
	uint32_t mul;
	int8_t shift;
	uint8_t flags;

retry:
	do {
		v1 = info->version;
		cpu_ccfence();
	} while (v1 & 1);
	cpu_lfence(); /* To prevent rdtsc() from happening before we read info->version. */

	stamp = info->tsc_timestamp;
	system_time = info->system_time;
	mul = info->tsc_to_system_mul;
	shift = info->tsc_shift;
	flags = info->flags;
	tsc = rdtsc();
	cpu_ccfence();
	v2 = info->version;

	if (v1 != v2)
		goto retry;

	if (flags & (1 << 1)) {
		info->flags &= ~(1 << 1);
		kprintf("%s: vcpu was paused (cpuid %d)\n", __func__, mycpuid);
	}

	if (tsc <= stamp) {
		kprintf("%s: tsc (0x%016jx) <= stamp (0x%016jx), weird\n",
		    __func__, tsc, stamp);
	}
	time = (tsc - stamp) >> 3;
	if (shift >= 0) {
		time <<= shift;
	} else {
		time >>= -shift;
	}

	/* Do the multiplication+shift in two halves, to avoid overflowing. */
	time = (((time & 0xffffffffUL) * mul) >> 32) + (time >> 32) * mul;
	time += system_time >> 3;

	if ((info->flags & (1 << 0)) || ncpus == 1)
		return (time + kvmclock_cputimer.base);

static uint64_t monotonic_time = 0;
	for (;;) {
		uint64_t val =  atomic_load_acq_long(&monotonic_time);
		if (time <= val) {
			return (val + kvmclock_cputimer.base);
		} else if (atomic_cmpset_long(&monotonic_time, val, time)) {
			return (time + kvmclock_cputimer.base);
		}
	}
}

/* XXX Properly deal with initializing during normal runtime. */
static void
kvmclock_cputimer_register(void)
{
	struct pvclock_vcpu_time_info info __aligned(4);
	uint64_t freq;
	int enable = -1;

	TUNABLE_INT_FETCH("hw.kvmclock_cputimer_enable", &enable);
	if (enable <= 0)
		return;

	if (!(kvm_feature & KVM_FEATURE_CLOCKSOURCE2))
		return;

	if (((uintptr_t)(&info) & 0x3) != 0) {
		kprintf("misaligned pvclock_vcpu_time_info struct\n");
		return;
	}

	wrmsr(MSR_KVM_SYSTEM_TIME_NEW, vtophys(&info) | 1ULL);

	kprintf("%s: version=0x%08x\n", __func__, info.version);
	kprintf("%s: tsc_timestamp=0x%016lx\n", __func__, info.tsc_timestamp);
	kprintf("%s: system_time=0x%016lx\n", __func__, info.system_time);
	kprintf("%s: tsc_to_system_mul=0x%08x\n", __func__, info.tsc_to_system_mul);
	kprintf("%s: tsc_shift=%d\n", __func__, info.tsc_shift);
	kprintf("%s: flags=0x%02x\n", __func__, info.flags);

	freq = (uint64_t)(1000*1000*1000) << 32;
	freq /= info.tsc_to_system_mul;
	if (info.tsc_shift >= 0)
		freq >>= info.tsc_shift;
	else
		freq <<= -info.tsc_shift;
	kprintf("TSC frequency determined from kvmclock: %lu Hz\n", freq);
	tsc_frequency = freq;

	wrmsr(MSR_KVM_SYSTEM_TIME_NEW, 0);

	if (!(info.flags & (1 << 0))) {
		kprintf("kvmclock not monotonic across multiple cpus\n");
		/*
		 * XXX A possibility would be, to later check whether we detect
		 *     the TSC as mpsync-ed, and treat KVMCLOCK as monotonic in
		 *     that case anyway.
		 */
	}

	if (!enable)
		return;

	kvmclock_cputimer.freq = 125 * 1000 * 1000;

	kprintf("KVMCLOCK: cputimer freq %ju\n",
	    (uintmax_t)kvmclock_cputimer.freq);

	cputimer_register(&kvmclock_cputimer);
	cputimer_select(&kvmclock_cputimer, 0);
	i8254_cputimer_disable = 1;
}
/* XXX Maybe rename to EARLY_TIMECOUNTER_INIT, to explain the purpose better. */
TIMECOUNTER_INIT(kvmclock, kvmclock_cputimer_register);

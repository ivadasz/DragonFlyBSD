/*
 * Copyright (c) 2015 Imre Vadász <imre@vdsz.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* sensor driver for effective CPU frequency, using APERF/MPERF MSRs */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/types.h>
#include <sys/sensors.h>
#include <sys/thread2.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>

#define MSR_MPERF	0xE7
#define MSR_APERF	0xE8

static struct ksensordev machdep_aperf_sensordev;
struct aperf_prev {
	uint64_t prevaperf;
	uint64_t prevmperf;
	uint64_t aperfval;
	uint64_t mperfval;
	struct ksensor sensor;
};
static struct aperf_prev *prevvals;
static int aperf_running;

extern int64_t tsc_frequency;

static void
aperf_update(struct aperf_prev *p)
{
	struct ksensor *s = &p->sensor;
	uint64_t ap, mp, val;

	crit_enter();
	ap = rdmsr(MSR_APERF);
	mp = rdmsr(MSR_MPERF);
	crit_exit();

	if (ap > p->prevaperf &&
	    mp > p->prevmperf) {
		p->aperfval = ap - p->prevaperf;
		p->mperfval = mp - p->prevmperf;
	}
	p->prevaperf = ap;
	p->prevmperf = mp;

	/* avoid division by zero */
	if (p->mperfval == 0)
		p->mperfval = 1;

	/* Using tsc_frequency/100 to avoid overflowing */
	val = ((p->aperfval * (tsc_frequency/100)) / p->mperfval) / 10000;
	s->value = val;
}

static void
aperf_refresh(void *arg)
{
	int i, origcpu;

	origcpu = mycpuid;

	for (i = 0; i < ncpus; i++) {
		lwkt_migratecpu(i);
		aperf_update(&prevvals[i]);
	}
	lwkt_migratecpu(origcpu);
}

static int
aperf_init(void)
{
	uint64_t ap, mp;
	u_int32_t regs  [4];
	int i, origcpu;
	char *drvname = "aperf";

	/* CPUID Fn0000_0006_ECX Effective Processor Frequency Interface */
	do_cpuid(0x00000006, regs);
	if ((regs[2] & 1) == 0)
		return 1;

	kprintf("%s: APERF/MPERF CPU frequency measuring enabled\n", drvname);

	strlcpy(machdep_aperf_sensordev.xname, drvname,
	    sizeof(machdep_aperf_sensordev.xname));

	origcpu = mycpuid;

	for (i = 0; i < ncpus; i++) {
		prevvals[i].sensor.type = SENSOR_INTEGER;
		ksnprintf(prevvals[i].sensor.desc,
		    sizeof(prevvals[i].sensor.desc), "Eff-Freq CPU%d (MHz)",
		    i);
		sensor_attach(&machdep_aperf_sensordev, &prevvals[i].sensor);
		lwkt_migratecpu(i);
		crit_enter();
		ap = rdmsr(MSR_APERF);
		mp = rdmsr(MSR_MPERF);
		crit_exit();

		prevvals[i].prevaperf = ap;
		prevvals[i].prevmperf = mp;
	}

	lwkt_migratecpu(origcpu);

	sensor_task_register(prevvals, aperf_refresh, 5);
	sensordev_install(&machdep_aperf_sensordev);
	aperf_running = 1;

	return (0);
}

static int
aperf_modevh(struct module *m, int what, void *arg __unused)
{
	int		error = 0;

	switch (what) {
	case MOD_LOAD:
		aperf_running = 0;
		prevvals = kmalloc(ncpus * sizeof(*prevvals), M_DEVBUF,
		    M_WAITOK | M_ZERO);
		error = aperf_init();
		break;
	case MOD_UNLOAD:
		if (aperf_running) {
			aperf_running = 0;
			sensordev_deinstall(&machdep_aperf_sensordev);
			sensor_task_unregister(prevvals);
		}
		if (prevvals != NULL)
			kfree(prevvals, M_DEVBUF);
		break;
	default:
		error = EINVAL;
		break;
	}
	return (error);
}
static moduledata_t aperf_mod = {
	"aperf",
	aperf_modevh,
	NULL,
};

DECLARE_MODULE(aperf, aperf_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);

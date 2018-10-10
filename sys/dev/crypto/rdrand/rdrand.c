/*
 * Copyright (c) 2012 Alex Hornung <alex@alexhornung.com>.
 * All rights reserved.
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
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kobj.h>
#include <sys/libkern.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/random.h>
#include <sys/sysctl.h>

#include <machine/specialreg.h>

#define	RDRAND_ALIGN(p)	(void *)(roundup2((uintptr_t)(p), 16))
#define RDRAND_SIZE	512

static int rdrand_debug;
SYSCTL_INT(_debug, OID_AUTO, rdrand, CTLFLAG_RW, &rdrand_debug, 0,
	   "Enable rdrand debugging");

struct rdrand_softc {
	struct callout	sc_rng_co;
	struct coarse_callout sc_rng_coarse;
	int32_t		sc_rng_ticks;
};

static int iter = 0;
extern int rand_bkgd_paused;
extern int random_read_requests;

static void rdrand_restart_callout(void *arg);


static void rdrand_rng_harvest(void *);
int rdrand_rng(uint8_t *out, long limit);

extern int in_powersave_mode;

static void
rdrand_identify(driver_t *drv, device_t parent)
{

	/* NB: order 10 is so we get attached after h/w devices */
	if (device_find_child(parent, "rdrand", -1) == NULL &&
	    BUS_ADD_CHILD(parent, parent, 10, "rdrand", -1) == 0)
		panic("rdrand: could not attach");
}


static int
rdrand_probe(device_t dev)
{

	if ((cpu_feature2 & CPUID2_RDRAND) == 0) {
		device_printf(dev, "No RdRand support.\n");
		return (EINVAL);
	}

	device_set_desc(dev, "RdRand RNG");
	return 0;
}


static int
rdrand_attach(device_t dev)
{
	struct rdrand_softc *sc;

	sc = device_get_softc(dev);

	if (hz > 10)
		sc->sc_rng_ticks = hz / 10;
	else
		sc->sc_rng_ticks = 1;

	register_random_background(rdrand_restart_callout, sc);

	callout_init_mp(&sc->sc_rng_co);
	callout_init_coarse(&sc->sc_rng_coarse);
	callout_reset(&sc->sc_rng_co, sc->sc_rng_ticks,
		      rdrand_rng_harvest, sc);

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "rng_ticks",
	    CTLFLAG_RW, &sc->sc_rng_ticks, 0, "control debugging");

	return 0;
}


static int
rdrand_detach(device_t dev)
{
	struct rdrand_softc *sc;

	sc = device_get_softc(dev);

	register_random_background(NULL, NULL);
	callout_stop_sync(&sc->sc_rng_co);
	callout_stop_coarse_sync(&sc->sc_rng_coarse);

	return (0);
}

static void
rdrand_restart_callout(void *arg)
{
	struct rdrand_softc *sc = arg;
	int timo;

	timo = sc->sc_rng_ticks <= 0 ?  1 :
	    (sc->sc_rng_ticks > hz ?  hz : sc->sc_rng_ticks);

	if (timo == hz && in_powersave_mode) {
		callout_start_coarse(&sc->sc_rng_coarse, 1, rdrand_rng_harvest,
		    sc);
	} else {
		callout_reset(&sc->sc_rng_co, timo, rdrand_rng_harvest, sc);
	}
}

static void
rdrand_rng_harvest(void *arg)
{
	struct rdrand_softc *sc = arg;
	uint8_t randomness[RDRAND_SIZE + 32];
	uint8_t *arandomness; /* randomness aligned */
	int cnt, timo;

	iter++;

	arandomness = RDRAND_ALIGN(randomness);

	cnt = rdrand_rng(arandomness, RDRAND_SIZE);
	if (cnt > 0 && cnt < sizeof(randomness)) {
		add_buffer_randomness_src(arandomness, cnt, RAND_SRC_RDRAND);

		if (rdrand_debug) {
			kprintf("rdrand(%d): %02x %02x %02x %02x...\n",
				cnt,
				arandomness[0],
				arandomness[1],
				arandomness[2],
				arandomness[3]);
		}
	}

	if (iter > 10 && random_read_requests == 0) {
		rand_bkgd_paused = 1;
		return;
	}
	atomic_store_rel_int(&random_read_requests, 0);

	timo = sc->sc_rng_ticks <= 0 ? 1 :
	    (sc->sc_rng_ticks > hz ?  hz : sc->sc_rng_ticks);

	if (timo == hz && in_powersave_mode) {
		callout_start_coarse(&sc->sc_rng_coarse, 1, rdrand_rng_harvest,
		    sc);
	} else  {
		callout_reset(&sc->sc_rng_co, timo, rdrand_rng_harvest, sc);
	}
}


static device_method_t rdrand_methods[] = {
	DEVMETHOD(device_identify, rdrand_identify),
	DEVMETHOD(device_probe, rdrand_probe),
	DEVMETHOD(device_attach, rdrand_attach),
	DEVMETHOD(device_detach, rdrand_detach),

	DEVMETHOD_END
};


static driver_t rdrand_driver = {
	"rdrand",
	rdrand_methods,
	sizeof(struct rdrand_softc),
};

static devclass_t rdrand_devclass;

DRIVER_MODULE(rdrand, nexus, rdrand_driver, rdrand_devclass, NULL, NULL);
MODULE_VERSION(rdrand, 1);
MODULE_DEPEND(rdrand, crypto, 1, 1, 1);

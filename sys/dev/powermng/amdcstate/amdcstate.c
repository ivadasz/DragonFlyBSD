/*
 * Copyright (c) 2014 Imre Vadász <imre@vdsz.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Device driver for AMD's Package C-state Residency feature via PCI.
 * Supports AMD's Family 14h Models 00h-0Fh.
 * This is also supposed to work on Family 12h, just need to add the
 * corresponding product id.
 */

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/sensors.h>

#include <bus/pci/pcivar.h>
#include "pcidevs.h"


/*
 * AMD Family 14h Processors, Function 6 -- Configuration Registers
 */

/* Function 6 Registers */
#define AMDCST_NONPC0_RESIDENCY	0xb0
#define AMDCST_PC1_RESIDENCY		0xb4
#define AMDCST_PC6_RESIDENCY		0xb8

#define AMDCST_COUNTER_ENABLE		0xe0
#define AMDCST_COUNTER_ENABLE_PC6	0x00000004
#define AMDCST_COUNTER_ENABLE_PC1	0x00000002
#define AMDCST_COUNTER_ENABLE_NONPC0	0x00000001

struct amdcstate_softc {
	device_t		sc_dev;
	struct ksensor		sc_sensors[3];
	struct ksensordev	sc_sensordev;
};

static void	amdcstate_identify(driver_t *, device_t);
static int	amdcstate_probe(device_t);
static int	amdcstate_attach(device_t);
static int	amdcstate_detach(device_t);
static void	amdcstate_refresh(void *);

static device_method_t amdcstate_methods[] = {
	DEVMETHOD(device_identify,	amdcstate_identify),
	DEVMETHOD(device_probe,		amdcstate_probe),
	DEVMETHOD(device_attach,	amdcstate_attach),
	DEVMETHOD(device_detach,	amdcstate_detach),
	{ NULL, NULL }
};

static driver_t amdcstate_driver = {
	"amdcstate",
	amdcstate_methods,
	sizeof(struct amdcstate_softc)
};

static devclass_t amdcstate_devclass;

DRIVER_MODULE(amdcstate, hostb, amdcstate_driver, amdcstate_devclass,
    NULL, NULL);

static void
amdcstate_identify(driver_t *driver, device_t parent)
{
	if (amdcstate_probe(parent) == ENXIO)
		return;
	if (device_find_child(parent, driver->name, -1) != NULL)
		return;
	device_add_child(parent, driver->name, -1);
}

static int
amdcstate_probe(device_t dev)
{
	char *desc;

	if (pci_get_vendor(dev) != PCI_VENDOR_AMD)
		return (ENXIO);

	switch (pci_get_device(dev)) {
	case PCI_PRODUCT_AMD_AMD64_F14_CONFIG6:
		desc = "AMD Family 14h Configuration Registers (function 6)";
		break;
	default:
		return (ENXIO);
	}

	if (device_get_desc(dev) == NULL)
		device_set_desc(dev, desc);
	return (0);
}

static int
amdcstate_attach(device_t dev)
{
	struct amdcstate_softc	*sc;
	uint32_t r;
	int i;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	/* Enable counters */
	r = pci_read_config(sc->sc_dev, AMDCST_COUNTER_ENABLE, 4);
	pci_write_config(sc->sc_dev, AMDCST_COUNTER_ENABLE,
	    r | AMDCST_COUNTER_ENABLE_PC6 | AMDCST_COUNTER_ENABLE_PC1 |
	    AMDCST_COUNTER_ENABLE_NONPC0, 4);

	/* Reset counters */
	pci_write_config(sc->sc_dev, AMDCST_NONPC0_RESIDENCY, 0, 4);
	pci_write_config(sc->sc_dev, AMDCST_PC1_RESIDENCY, 0, 4);
	pci_write_config(sc->sc_dev, AMDCST_PC6_RESIDENCY, 0, 4);

	/* Initialize sensors */
	strlcpy(sc->sc_sensordev.xname, device_get_nameunit(sc->sc_dev),
	    sizeof(sc->sc_sensordev.xname));

	strlcpy(sc->sc_sensors[0].desc, "Non-PC0 Residency",
	    sizeof(sc->sc_sensors[0].desc));
	strlcpy(sc->sc_sensors[1].desc, "PC1 Residency",
	    sizeof(sc->sc_sensors[0].desc));
	strlcpy(sc->sc_sensors[2].desc, "PC6 Residency",
	    sizeof(sc->sc_sensors[0].desc));
	for (i = 0; i < 3; i++) {
		sc->sc_sensors[i].type = SENSOR_PERCENT;
		sensor_attach(&sc->sc_sensordev, &sc->sc_sensors[i]);
	}

	sensor_task_register(sc, amdcstate_refresh, 5);

	sensordev_install(&sc->sc_sensordev);

	return (0);
}

static int
amdcstate_detach(device_t dev)
{
	struct amdcstate_softc	*sc = device_get_softc(dev);
	uint32_t r;

	/* Disable counters */
	r = pci_read_config(sc->sc_dev, AMDCST_COUNTER_ENABLE, 4);
	pci_write_config(sc->sc_dev, AMDCST_COUNTER_ENABLE,
	    r & ~(AMDCST_COUNTER_ENABLE_PC6 |
	          AMDCST_COUNTER_ENABLE_PC1 |
	          AMDCST_COUNTER_ENABLE_NONPC0), 4);

	sensordev_deinstall(&sc->sc_sensordev);
	sensor_task_unregister(sc);

	return 0;
}

static void
amdcstate_refresh(void *arg)
{
	struct amdcstate_softc	*sc = arg;
	struct ksensor	*s = sc->sc_sensors;
	uint32_t	r;

	/* Read counters */
	r = pci_read_config(sc->sc_dev, AMDCST_NONPC0_RESIDENCY, 4);
	s[0].value = (r * 8)/(1000 * 5);
	r = pci_read_config(sc->sc_dev, AMDCST_PC1_RESIDENCY, 4);
	s[1].value = (r * 8)/(1000 * 5);
	r = pci_read_config(sc->sc_dev, AMDCST_PC6_RESIDENCY, 4);
	s[2].value = (r * 8)/(1000 * 5);

	/* Reset counters */
	pci_write_config(sc->sc_dev, AMDCST_NONPC0_RESIDENCY, 0, 4);
	pci_write_config(sc->sc_dev, AMDCST_PC1_RESIDENCY, 0, 4);
	pci_write_config(sc->sc_dev, AMDCST_PC6_RESIDENCY, 0, 4);
}

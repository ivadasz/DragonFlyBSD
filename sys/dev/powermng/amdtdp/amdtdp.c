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
 * Device driver for AMD's TDP capping feature via PCI.
 * First introduced in AMD's Family 15h Models 30h-3fh (Kaveri) and Family
 * 16h Models 00h-0fh (Kabini) lines of processors.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#if 0
#include <sys/sensors.h>
#endif

#include <bus/pci/pcivar.h>
#include "pcidevs.h"


/*
 * AMD Family 10h/11h/14h/15h Processors, Function 4 -- XXX
 */

/* Function 4 Registers */
#define AMDTDP_TDP_LIMIT	0x250
#define AMDTDP_NODE_CAC		0x1c0

/* Operations on Reported Temperature Control Register */
#define AMDTDP_GET_TDPLIMIT(r)	(((r) >> 0) & 0xfff)

/* Operations on Thermtrip Status Register */
#define AMDTDP_GET_NODECAC(r)	(((r) >> 0) & 0xfff)


struct amdtdp_softc {
	struct device		*sc_dev;
	uint16_t		sc_maxtdp;
#if 0
	struct ksensor		sc_sensor;
	struct ksensordev	sc_sensordev;
#endif
};

static void	amdtdp_identify(driver_t *, struct device *);
static int	amdtdp_probe(struct device *);
static int	amdtdp_attach(struct device *);
static int	amdtdp_detach(struct device *);
#if 0
static void	km_refresh(void *);
#endif

static device_method_t km_methods[] = {
	DEVMETHOD(device_identify,	km_identify),
	DEVMETHOD(device_probe,		km_probe),
	DEVMETHOD(device_attach,	km_attach),
	DEVMETHOD(device_detach,	km_detach),
	{ NULL, NULL }
};

static driver_t km_driver = {
	"km",
	km_methods,
	sizeof(struct km_softc)
};

static devclass_t km_devclass;

DRIVER_MODULE(km, hostb, km_driver, km_devclass, NULL, NULL);


static void
km_identify(driver_t *driver, struct device *parent)
{
	if (km_probe(parent) == ENXIO)
		return;
	if (device_find_child(parent, driver->name, -1) != NULL)
		return;
	device_add_child(parent, driver->name, -1);
}

static int
km_probe(struct device *dev)
{
	char *desc;

	if (pci_get_vendor(dev) != PCI_VENDOR_AMD)
		return ENXIO;

	switch (pci_get_device(dev)) {
	case PCI_PRODUCT_AMD_AMD64_F10_MISC:
		desc = "AMD Family 10h temperature sensor";
		break;
	case PCI_PRODUCT_AMD_AMD64_F11_MISC:
		desc = "AMD Family 11h temperature sensor";
		break;
	case PCI_PRODUCT_AMD_AMD64_F14_MISC:
		desc = "AMD Family 14h temperature sensor";
		break;
	case PCI_PRODUCT_AMD_AMD64_F15_0x_MISC:
		desc = "AMD Family 15/0xh temperature sensor";
		break;
	case PCI_PRODUCT_AMD_AMD64_F15_1x_MISC:
		desc = "AMD Family 15/1xh temperature sensor";
		break;
	case PCI_PRODUCT_AMD_AMD64_F15_3x_MISC:
		desc = "AMD Family 15/3xh temperature sensor";
		break;
	case PCI_PRODUCT_AMD_AMD64_F16_MISC:
		desc = "AMD Family 16h temperature sensor";
		break;
	default:
		return ENXIO;
	}

	if (device_get_desc(dev) == NULL)
		device_set_desc(dev, desc);
	return 0;
}

static int
km_attach(struct device *dev)
{
	struct km_softc	*sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	strlcpy(sc->sc_sensordev.xname, device_get_nameunit(dev),
	    sizeof(sc->sc_sensordev.xname));

	sc->sc_sensor.type = SENSOR_TEMP;
	sensor_attach(&sc->sc_sensordev, &sc->sc_sensor);

	if (sensor_task_register(sc, km_refresh, 5)) {
		device_printf(dev, "unable to register update task\n");
		return ENXIO;
	}

	sensordev_install(&sc->sc_sensordev);
	return 0;
}

static int
km_detach(struct device *dev)
{
	struct km_softc	*sc = device_get_softc(dev);

	sensordev_deinstall(&sc->sc_sensordev);
	sensor_task_unregister(sc);
	return 0;
}

static void
km_refresh(void *arg)
{
	struct km_softc	*sc = arg;
	struct ksensor	*s = &sc->sc_sensor;
	uint32_t	r;
	int		c;

	r = pci_read_config(sc->sc_dev, KM_REP_TEMP_CONTR_R, 4);
	c = KM_GET_CURTMP(r);
	s->value = c * 125000 + 273150000;
}

/*
 * Copyright (c) 2018 The DragonFly Project.  All rights reserved.
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/bus.h>

#include "opt_acpi.h"
#include "acpi.h"
#include <dev/acpica/acpivar.h>

#include <bus/pci/pcivar.h>

#include <bus/hid/hid_common.h>
#include <bus/hid/hidvar.h>
#include "hid_if.h"

ACPI_MODULE_NAME("iichid");

static int iichid_probe(device_t dev);
static int iichid_attach(device_t dev);
static int iichid_detach(device_t dev);

struct iichid_softc {
	device_t dev;
	struct acpi_new_resource *iic_res;
};

static char *iichid_ids[] = {
	"PNP0C50",
	NULL
};

static int
iichid_probe(device_t dev)
{

        if (acpi_disabled("iichid") ||
            ACPI_ID_PROBE(device_get_parent(dev), dev, iichid_ids) == NULL)
                return (ENXIO);

	device_set_desc(dev, "I2C-HID device");

	return (BUS_PROBE_DEFAULT);
}

static int
iichid_attach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);
	struct acpi_new_resource *ires;

	sc->dev = dev;

	ires = ACPI_ALLOC_NEW_RESOURCE(device_get_parent(dev), dev,
	    NEW_RES_IIC, 0);
	if (ires == NULL) {
		device_printf(dev, "Failed to allocate I2C resource\n");
		return ENXIO;
	}
	sc->iic_res = ires;
	pci_set_powerstate(dev, PCI_POWERSTATE_D0);

	return 0;
}

static int
iichid_detach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);

	if (sc->iic_res != NULL) {
		ACPI_RELEASE_NEW_RESOURCE(device_get_parent(dev), sc->iic_res);
	}
	pci_set_powerstate(dev, PCI_POWERSTATE_D3);

	return 0;
}

static device_method_t iichid_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, iichid_probe),
	DEVMETHOD(device_attach, iichid_attach),
	DEVMETHOD(device_detach, iichid_detach),

	DEVMETHOD_END
};

static driver_t iichid_driver = {
        "iichid",
        iichid_methods,
        sizeof(struct iichid_softc),
	.gpri = KOBJ_GPRI_ACPI
};

static devclass_t iichid_devclass;

DRIVER_MODULE(iichid, acpi, iichid_driver, iichid_devclass, NULL, NULL);
MODULE_DEPEND(iichid, acpi, 1, 1, 1);

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
#include <sys/uuid.h>
#include <sys/bus.h>

#include "opt_acpi.h"
#include "acpi.h"
#include <dev/acpica/acpivar.h>

#include <bus/pci/pcivar.h>

#include <bus/smbus/smbacpi/smbacpi.h>

#include <bus/hid/hid_common.h>
#include <bus/hid/hidvar.h>
#include "hid_if.h"

ACPI_MODULE_NAME("iichid");

#ifndef _IICHID_H_
#define _IICHID_H_

struct iic_hid_descriptor {
	uint16_t wHIDDescLength;
	uint16_t bcdVersion;
	uint16_t wReportDescLength;
	uint16_t wReportDescRegister;
	uint16_t wInputRegister;
	uint16_t wMaxInputLength;
	uint16_t wOutputRegister;
	uint16_t wMaxOutputLength;
	uint16_t wCommandRegister;
	uint16_t wDataRegister;
	uint16_t wVendorID;
	uint16_t wProductID;
	uint16_t wVersionID;
	uint32_t reserved;
} __packed;

#endif	/* !_IICHID_H_ */

static int iichid_probe(device_t dev);
static int iichid_attach(device_t dev);
static int iichid_detach(device_t dev);

struct iichid_softc {
	device_t dev;
	uint16_t desc_reg;	/* I2C-HID descriptor register address. */
	struct acpi_new_resource *iic_res;
	struct iic_hid_descriptor hid_desc;

	device_t child;
	u_char *report_descriptor;
};

static char *iichid_ids[] = {
	"PNP0C50",
	NULL
};

static int
iichid_get_descriptor_address(struct iichid_softc *sc, uint16_t *addr)
{
	ACPI_BUFFER retbuf = { ACPI_ALLOCATE_BUFFER, NULL };
	ACPI_OBJECT_LIST arglist;
	ACPI_OBJECT arg[4];
	ACPI_STATUS status;
	ACPI_OBJECT *retobj;
	const char *uuidstr = "3cdff6f7-4267-4555-ad05-b30a3d8938de";
	struct uuid uuid;
	uint8_t dsmuuid[16];

	if (parse_uuid(uuidstr, &uuid) != 0)
		return (1);
	le_uuid_enc(dsmuuid, &uuid);

	arglist.Pointer = arg;
	arglist.Count = 4;
	arg[0].Type = ACPI_TYPE_BUFFER;
	arg[0].Buffer.Length = sizeof(dsmuuid);
	arg[0].Buffer.Pointer = dsmuuid;
	arg[1].Type = ACPI_TYPE_INTEGER;
	arg[1].Integer.Value = 0;
	arg[2].Type = ACPI_TYPE_INTEGER;
	arg[2].Integer.Value = 1;
	arg[3].Type = ACPI_TYPE_PACKAGE;
	arg[3].Package.Count = 0;
	arg[3].Package.Elements = NULL;

	status = AcpiEvaluateObject(acpi_get_handle(sc->dev), "_DSM", &arglist,
	    &retbuf);
	if (ACPI_FAILURE(status))
		return (1);
	retobj = retbuf.Pointer;
	if (retobj->Type != ACPI_TYPE_INTEGER) {
		AcpiOsFree(retbuf.Pointer);
		return (1);
	}
	*addr = retobj->Integer.Value;

	return (0);
}

static int
iichid_readreg(struct iichid_softc *sc, uint16_t reg, u_char *buf, int count,
    int *actualp)
{
	if (smbus_acpi_rawtrans(sc->iic_res, (char *)&reg, 2, buf, count,
	    actualp) != 0 ) {
		device_printf(sc->dev,
		    "failed to read %d bytes from reg 0x%04x\n", count, reg);
		return (1);
	}
	return (0);
}

static int
iichid_readreg_check(struct iichid_softc *sc, uint16_t reg, u_char *buf,
    int count)
{
	int actualp, val;

	if ((val = iichid_readreg(sc, reg, buf, count, &actualp)) != 0)
		return (val);

	if (actualp != count) {
		device_printf(sc->dev,
		    "read wrong number of bytes from reg 0x%04x, wanted "
		    "%d bytes, got %d\n", reg, count, actualp);
		return (1);
	}
	return (0);
}

static int
iichid_probe(device_t dev)
{

        if (acpi_disabled("iichid") ||
            ACPI_ID_PROBE(device_get_parent(dev), dev, iichid_ids) == NULL)
                return (ENXIO);

	device_set_desc(dev, "I2C-HID device");

	return (BUS_PROBE_DEFAULT);
}

static void
dump_bytes(device_t dev, uint8_t *buf, int length)
{
	int i;

	for (i = 0; i < length; i += 8) {
		int j;

		device_printf(dev, "0x%03x", i);
		for (j = i; j < i + 16 && j < length; j++) {
			kprintf(" %02x", buf[j]);
		}
		kprintf("\n");
	}
}

static int
iichid_attach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);
	struct acpi_new_resource *ires;

	sc->dev = dev;
	if (iichid_get_descriptor_address(sc, &sc->desc_reg) != 0) {
		device_printf(dev, "Failed to find descriptor register\n");
		return ENXIO;
	}

	ires = ACPI_ALLOC_NEW_RESOURCE(device_get_parent(dev), dev,
	    NEW_RES_IIC, 0);
	if (ires == NULL) {
		device_printf(dev, "Failed to allocate I2C resource\n");
		return ENXIO;
	}
	sc->iic_res = ires;
	pci_set_powerstate(dev, PCI_POWERSTATE_D0);

	device_printf(dev, "Fetching I2C-HID descriptor from reg 0x%04x\n",
	    sc->desc_reg);
	if (iichid_readreg_check(sc, sc->desc_reg,
	    (u_char *)&sc->hid_desc, sizeof(sc->hid_desc)) != 0) {
		device_printf(dev, "Failed to retrieve HID Descriptor\n");
		pci_set_powerstate(dev, PCI_POWERSTATE_D3);
		ACPI_RELEASE_NEW_RESOURCE(device_get_parent(dev), sc->iic_res);
		return (ENXIO);
	}
	sc->report_descriptor = kmalloc(sc->hid_desc.wReportDescLength,
	    M_DEVBUF, M_WAITOK | M_ZERO);
	device_printf(dev, "Getting %u byte report descriptor from register 0x%x\n",
	    sc->hid_desc.wReportDescLength, sc->hid_desc.wReportDescRegister);
	if (iichid_readreg_check(sc, sc->hid_desc.wReportDescRegister,
	    sc->report_descriptor, sc->hid_desc.wReportDescLength) != 0) {
		device_printf(dev, "Failed to retrieve Report Descriptor\n");
		pci_set_powerstate(dev, PCI_POWERSTATE_D3);
		ACPI_RELEASE_NEW_RESOURCE(device_get_parent(dev), sc->iic_res);
		kfree(sc->report_descriptor, M_DEVBUF);
		return (ENXIO);
	}
	if (bootverbose)
		dump_bytes(dev, sc->report_descriptor, sc->hid_desc.wReportDescLength);
	if (sc->child == NULL) {
		sc->child = device_add_child(dev, NULL, -1);
	}
	bus_generic_attach(dev);

	return 0;
}

static int
iichid_detach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);

	if (bus_generic_detach(dev) != 0)
		return EBUSY;

	device_delete_child(dev, sc->child);
	sc->child = NULL;
	if (sc->iic_res != NULL) {
		ACPI_RELEASE_NEW_RESOURCE(device_get_parent(dev), sc->iic_res);
	}
	pci_set_powerstate(dev, PCI_POWERSTATE_D3);
	kfree(sc->report_descriptor, M_DEVBUF);

	return 0;
}

static void
iichid_get_descriptor(device_t dev, char **descp, uint16_t *sizep)
{
	struct iichid_softc *sc = device_get_softc(dev);

	*descp = (void *)sc->report_descriptor;
	*sizep = sc->hid_desc.wReportDescLength;
}

static void
iichid_set_handler(device_t dev, hid_input_handler_t input,
    hid_output_handler_t output, void *arg)
{
	/* XXX */
}

static void
iichid_start_read(device_t dev, uint16_t max_len)
{
	/* XXX */
}

static void
iichid_stop_read(device_t dev)
{
	/* XXX */
}

static void
iichid_input_poll(device_t dev)
{
	/* XXX */
}

static void
iichid_setidle(device_t dev, uint8_t duration, uint8_t id)
{
	/* XXX */
}

static void
iichid_set_report(device_t dev, uint8_t id, uint8_t *buf, uint16_t len)
{
	/* XXX */
}

static void
iichid_set_feature(device_t dev, uint8_t id, uint8_t *buf, uint16_t len)
{
	/* XXX */
}

static int
iichid_get_report(device_t dev, uint8_t id, uint8_t *buf, uint16_t len,
    int type)
{
	/* XXX */
	return ENXIO;
}

static int
iichid_set_protocol(device_t dev, int protocol)
{
	/* XXX */
	return ENXIO;
}

static device_method_t iichid_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, iichid_probe),
	DEVMETHOD(device_attach, iichid_attach),
	DEVMETHOD(device_detach, iichid_detach),

	/* HID interface */
	DEVMETHOD(hid_get_descriptor, iichid_get_descriptor),
	DEVMETHOD(hid_set_handler, iichid_set_handler),
	DEVMETHOD(hid_start_read, iichid_start_read),
	DEVMETHOD(hid_stop_read, iichid_stop_read),
	DEVMETHOD(hid_input_poll, iichid_input_poll),
	DEVMETHOD(hid_setidle, iichid_setidle),
	DEVMETHOD(hid_set_report, iichid_set_report),
	DEVMETHOD(hid_set_feature, iichid_set_feature),
	DEVMETHOD(hid_get_report, iichid_get_report),
	DEVMETHOD(hid_set_protocol, iichid_set_protocol),

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
MODULE_DEPEND(iichid, smbacpi, 1, 1, 1);
MODULE_DEPEND(iichid, hidbus, 1, 1, 1);
MODULE_VERSION(iichid, 1);

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
#include <bus/gpio/gpio_acpi/gpio_acpivar.h>

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
	struct lock lk;
	uint16_t desc_reg;	/* I2C-HID descriptor register address. */
	struct acpi_new_resource *iic_res;
	struct acpi_new_resource *gpio_res;
	struct iic_hid_descriptor hid_desc;

	device_t child;
	u_char *report_descriptor;
	int have_multi_id;

	u_char *input_report;
	uint16_t max_len;
	hid_input_handler_t input_handler;
	void *handler_arg;
};

static char *iichid_ids[] = {
	"PNP0C50",
	NULL
};

static int
iichid_have_multi_id(void *buf, int len, int kind)
{
	struct hid_data *d;
	struct hid_item h;

	h.report_ID = 0;
	for (d = hid_start_parse(buf, len, kind); hid_get_item(d, &h); ) {
		if (h.report_ID != 0)
			return (1);
	}
	hid_end_parse(d);
	return (0);
}

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
iichid_writereg16(struct iichid_softc *sc, uint16_t reg, uint16_t value)
{
	uint16_t data[2] = { reg, value };

	if (smbus_acpi_rawtrans(sc->iic_res, (char *)data, 4, NULL, 0, NULL)
	    != 0) {
		device_printf(sc->dev, "failed to set reg 0x%04x to 0x%04x\n",
		    reg, value);
		return (1);
	}

	return (0);
}

static int
iichid_reset(struct iichid_softc *sc)
{
	uint16_t req = 0x0100;

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	if (iichid_writereg16(sc, sc->hid_desc.wCommandRegister, req) != 0) {
		lockmgr(&sc->lk, LK_RELEASE);
		return (1);
	}

	/* Wait for GPIO interrupt and 2 byte input report with length 0 */
	if (lksleep(sc, &sc->lk, 0, "iichid", 5*hz) == EWOULDBLOCK) {
		device_printf(sc->dev, "reset timed out\n");
		lockmgr(&sc->lk, LK_RELEASE);
		return 1;
	}

	lockmgr(&sc->lk, LK_RELEASE);
	return (0);
}


static int
iichid_setpower(struct iichid_softc *sc, int on)
{
	uint16_t req = (0x08 << 8) | (on ? 0x00 : 0x01);

	if (iichid_writereg16(sc, sc->hid_desc.wCommandRegister, req) != 0) {
		device_printf(sc->dev, "setpower to %s failed\n",
		    on ? "on" : "off");
		return (1);
	}

	return (0);
}

static int
iichid_fetch_report(struct iichid_softc *sc, int *actualp)
{
	uint16_t length;
	uint16_t reg = sc->hid_desc.wInputRegister;
	uint16_t maxcount = sc->hid_desc.wMaxInputLength;
	int count;

	if (iichid_readreg(sc, reg, sc->input_report, maxcount, &count) != 0) {
		device_printf(sc->dev, "fetching input report failed\n");
		return (1);
	}
	if (count < 2) {
		device_printf(sc->dev, "input report too short, %d bytes\n",
		    count);
		return (1);
	}
	length = *(uint16_t *)&sc->input_report[0];
	if (length == 0) {
		device_printf(sc->dev, "reset found\n");
		return (-1);
	}
	if (count != length) {
		device_printf(sc->dev, "length != count\n");
		return (1);
	}
	*actualp = length;

	return (0);
}

static void
iichid_intr(void *arg)
{
	struct iichid_softc *sc = (struct iichid_softc *)arg;
	int count, val;

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	val = iichid_fetch_report(sc, &count);
	if (val == -1)
		wakeup(sc);
	if (val == 0 && count > (sc->have_multi_id ? 1 : 0)) {
		if (sc->input_handler != NULL) {
			uint8_t id = 0;
			hid_input_handler_t fn;
			void *arg;
			uint8_t *buffer = sc->input_report;

			if (sc->have_multi_id) {
				id = buffer[0];
				buffer++;
				count--;
			}
			if (count > sc->max_len)
				count = sc->max_len;
			fn = sc->input_handler;
			arg = sc->handler_arg;
			lockmgr(&sc->lk, LK_RELEASE);
			fn(id, buffer, count, arg);
		}
	} else {
		lockmgr(&sc->lk, LK_RELEASE);
	}
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

/* XXX Currently requires interrupts to work already. */
static int
iichid_attach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);
	struct acpi_new_resource *ires, *gres;

	sc->dev = dev;
	lockinit(&sc->lk, "iiclk", 0, LK_CANRECURSE);
	if (iichid_get_descriptor_address(sc, &sc->desc_reg) != 0) {
		device_printf(dev, "Failed to find descriptor register\n");
		return ENXIO;
	}

	ires = acpi_alloc_new_resource(dev, NEW_RES_IIC, 0);
	if (ires == NULL) {
		device_printf(dev, "Failed to allocate I2C resource\n");
		return ENXIO;
	}
	sc->iic_res = ires;

	gres = acpi_alloc_new_resource(dev, NEW_RES_GPIOINT, 0);
	if (gres == NULL) {
		device_printf(dev, "Failed to allocate GPIO IRQ resource\n");
		acpi_release_new_resource(sc->iic_res);
		return ENXIO;
	}
	sc->gpio_res = gres;
	if (gpio_int_reserve_interrupt(sc->gpio_res) != 0) {
		device_printf(dev, "Failed to reserve GPIO IRQ resource\n");
		acpi_release_new_resource(sc->gpio_res);
		acpi_release_new_resource(sc->iic_res);
		return ENXIO;
	}
	pci_set_powerstate(dev, PCI_POWERSTATE_D0);

	device_printf(dev, "Fetching I2C-HID descriptor from reg 0x%04x\n",
	    sc->desc_reg);
	if (iichid_readreg_check(sc, sc->desc_reg,
	    (u_char *)&sc->hid_desc, sizeof(sc->hid_desc)) != 0) {
		device_printf(dev, "Failed to retrieve HID Descriptor\n");
		pci_set_powerstate(dev, PCI_POWERSTATE_D3);
		acpi_release_new_resource(sc->iic_res);
		gpio_int_unreserve_interrupt(sc->gpio_res);
		acpi_release_new_resource(sc->gpio_res);
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
		acpi_release_new_resource(sc->iic_res);
		gpio_int_unreserve_interrupt(sc->gpio_res);
		acpi_release_new_resource(sc->gpio_res);
		kfree(sc->report_descriptor, M_DEVBUF);
		return (ENXIO);
	}
	if (bootverbose)
		dump_bytes(dev, sc->report_descriptor, sc->hid_desc.wReportDescLength);
	sc->have_multi_id = iichid_have_multi_id(sc->report_descriptor,
	    sc->hid_desc.wReportDescLength, hid_input);
	sc->input_report = kmalloc(sc->hid_desc.wMaxInputLength, M_DEVBUF,
	    M_WAITOK | M_ZERO);
	if (sc->child == NULL) {
		sc->child = device_add_child(dev, NULL, -1);
	}
	bus_generic_attach(dev);
	gpio_int_establish_interrupt(sc->gpio_res, iichid_intr, sc);
	iichid_setpower(sc, 1);
	if (iichid_reset(sc) != 0) {
		iichid_detach(dev);
		return (ENXIO);
	}

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
	gpio_int_teardown_interrupt(sc->gpio_res);
	iichid_setpower(sc, 0);
	if (sc->iic_res != NULL) {
		acpi_release_new_resource(sc->iic_res);
		gpio_int_unreserve_interrupt(sc->gpio_res);
		acpi_release_new_resource(sc->gpio_res);
	}
	pci_set_powerstate(dev, PCI_POWERSTATE_D3);
	if (sc->report_descriptor != NULL)
		kfree(sc->report_descriptor, M_DEVBUF);
	if (sc->input_report != NULL)
		kfree(sc->input_report, M_DEVBUF);

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
	struct iichid_softc *sc = device_get_softc(dev);

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	sc->input_handler = input;
	sc->handler_arg = arg;
	lockmgr(&sc->lk, LK_RELEASE);
}

static void
iichid_start_read(device_t dev, uint16_t max_len)
{
	struct iichid_softc *sc = device_get_softc(dev);

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	sc->max_len = sc->hid_desc.wMaxInputLength;
	if (max_len > 0 && max_len < sc->max_len)
		sc->max_len = max_len;

	device_printf(dev, "starting input\n");
	lockmgr(&sc->lk, LK_RELEASE);
}

static void
iichid_stop_read(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);

	lockmgr(&sc->lk, LK_EXCLUSIVE);
	sc->max_len = 0;
	lockmgr(&sc->lk, LK_RELEASE);
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
MODULE_DEPEND(iichid, gpio_acpi, 1, 1, 1);
MODULE_DEPEND(iichid, hidbus, 1, 1, 1);
MODULE_VERSION(iichid, 1);

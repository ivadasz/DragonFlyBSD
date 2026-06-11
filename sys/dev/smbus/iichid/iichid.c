/*
 * IICHID - Driver for generic I2C-HID input devices
 */

#include <sys/kernel.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/serialize.h>
#include <sys/uuid.h>

#include "acpi.h"
#include "opt_acpi.h"
#include "acpi_if.h"
#include <dev/acpica/acpivar.h>

#include <bus/pci/pcivar.h>

struct iichid_softc {
	device_t 	dev;
	ACPI_HANDLE	handle;
	uint16_t	desc_reg;	/* I2C-HID descriptor address */
	struct resource	*i2c_res;
	struct resource	*irq_res;
	struct resource	*gpio_res;
	int		use_gpio_irq;
	void		*irq_cookie;
	struct lwkt_serialize slz;
	// xxx
};

static int iichid_probe(device_t);
static int iichid_attach(device_t);
static int iichid_detach(device_t);
static void iichid_intr(void *);
static int iichid_get_descriptor_address(struct iichid_softc *, uint16_t *);

static devclass_t iichid_devclass;

static device_method_t iichid_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		iichid_probe),
	DEVMETHOD(device_attach,	iichid_attach),
	DEVMETHOD(device_detach,	iichid_detach),

	DEVMETHOD_END
};

static driver_t iichid_driver = {
	"iichid",
	iichid_methods,
	sizeof(struct iichid_softc),
};

DRIVER_MODULE(iichid, acpi, iichid_driver, iichid_devclass, NULL, NULL);
MODULE_DEPEND(iichid, acpi, 1, 1, 1);
MODULE_VERSION(iichid, 1);

static char *iichid_ids[] = {
	"PNP0C50",
	NULL
};

static int
iichid_probe(device_t dev)
{
	if (acpi_disabled("iichid"))
		return (ENXIO);

	if (ACPI_ID_PROBE(device_get_parent(dev), dev, iichid_ids) == NULL)
		return (ENXIO);

	device_set_desc(dev, "I2C HID device");
	return (BUS_PROBE_DEFAULT);
}

static int
iichid_attach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);
	int i2c_rid = 0;
	int gpio_rid = 0;
	int irq_rid;
	int error;

	if (ACPI_ID_PROBE(device_get_parent(dev), dev, iichid_ids) == NULL)
		return (ENXIO);

	sc->dev = dev;
	sc->handle = acpi_get_handle(dev);

	if (iichid_get_descriptor_address(sc, &sc->desc_reg) != 0) {
		device_printf(dev, "Failed to find descriptor register\n");
		return (ENXIO);
	}

	pci_set_powerstate(dev, PCI_POWERSTATE_D0);

	sc->i2c_res = bus_alloc_resource_any(dev, SYS_RES_I2C, &i2c_rid, RF_ACTIVE);
	if (sc->i2c_res == NULL) {
		device_printf(dev, "Can't allocate I2C HID slave\n");
		iichid_detach(dev);
		return (ENOMEM);
	}
	device_printf(dev, "Got rid=%d for I2C HID\n", i2c_rid);

	sc->gpio_res = bus_alloc_resource_any(dev, SYS_RES_GPIO_IO, &gpio_rid,
	    RF_ACTIVE);
	if (sc->gpio_res == NULL) {
		device_printf(dev, "Can't allocate GPIO IO\n");
		iichid_detach(dev);
		return (ENOMEM);
	}
	device_printf(dev, "Got rid=%d for GPIO IO\n", gpio_rid);

	irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		irq_rid = 0;
		sc->use_gpio_irq = 1;
		sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_GPIO_IRQ,
		    &irq_rid, RF_ACTIVE);
	}
	if (sc->irq_res == NULL) {
		device_printf(dev, "Can't allocate GPIO IRQ\n");
		iichid_detach(dev);
		return (ENOMEM);
	}
	device_printf(dev, "Got rid=%d for GPIO IRQ\n", irq_rid);

	lwkt_serialize_init(&sc->slz);
	error = bus_setup_intr(dev, sc->irq_res, INTR_MPSAFE, iichid_intr, sc,
	    &sc->irq_cookie, &sc->slz);
	if (error) {
		iichid_detach(dev);
		return error;
	}

	return (0);
}

static int
iichid_detach(device_t dev)
{
	struct iichid_softc *sc = device_get_softc(dev);

	if (sc->irq_cookie != NULL) {
		lwkt_serialize_handler_disable(&sc->slz);
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
	}

	if (sc->i2c_res != NULL)
		bus_release_resource(dev, SYS_RES_I2C, 0, sc->i2c_res);
	if (sc->gpio_res != NULL)
		bus_release_resource(dev, SYS_RES_GPIO_IO, 0, sc->gpio_res);
	if (sc->irq_res != NULL) {
		bus_release_resource(dev,
		    sc->use_gpio_irq ? SYS_RES_GPIO_IRQ : SYS_RES_IRQ, 0,
		    sc->irq_res);
	}

	pci_set_powerstate(dev, PCI_POWERSTATE_D3);

	return (0);
}

static void
iichid_intr(void *arg)
{
	//struct iichid_softc *sc = arg;

	/* XXX */
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/lock.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/uio.h>
#include <sys/fcntl.h>
#include <sys/uuid.h>
#include <sys/event.h>
#include <sys/devfs.h>
#include <sys/vnode.h>

#include "opt_acpi.h"
#include "acpi.h"
#include <dev/acpica/acpivar.h>

#include <bus/pci/pcivar.h>

#include <bus/gpio/gpio_acpi/gpio_acpivar.h>
#include <bus/smbus/smbacpi/smbus_acpivar.h>

#include "axp20x.h"

struct intelpmic_softc {
	device_t	dev;
	ACPI_HANDLE	handle;

	struct lock	lk;

	struct iicserial_resource *iicres;
	struct gpioint_resource *intres;
};

static ACPI_STATUS	intel_xpower_pmic_gpio_handler(UINT32 Function,
			    ACPI_PHYSICAL_ADDRESS Address, UINT32 BitWidth,
			    UINT64 *Value, void *HandlerContext,
			    void *RegionContext);

static int	intelpmic_probe(device_t dev);
static int	intelpmic_attach(device_t dev);
static int	intelpmic_detach(device_t dev);

static ACPI_STATUS
intel_xpower_pmic_gpio_handler(UINT32 Function, ACPI_PHYSICAL_ADDRESS Address,
    UINT32 BitWidth, UINT64 *Value, void *HandlerContext, void *RegionContext)
{
	return (AE_OK);
}

static int
intelpmic_probe(device_t dev)
{
	/* X-Power PMIC */
	static char *intelpmic_ids[] = { "INT33F4", NULL };

	if (acpi_disabled("intelpmic") ||
	    ACPI_ID_PROBE(device_get_parent(dev), dev, intelpmic_ids) == NULL)
		return (ENXIO);

	device_set_desc(dev, "X-Power PMIC Controller");
	return (0);
}

static int
intelpmic_attach(device_t dev)
{
	struct intelpmic_softc *sc = device_get_softc(dev);

	sc->dev = dev;
	sc->handle = acpi_get_handle(dev);

	lockinit(&sc->lk, "iiclk", 0, LK_CANRECURSE);

	sc->iicres = iic_alloc_resource(dev, 0);
	if (sc->iicres == NULL) {
		device_printf(dev, "Failed to alloc I2cSerialBus resource\n");
		return (ENXIO);
	}
#if 0
	sc->intres = gpioint_alloc_resource(dev, 0);
	if (sc->intres == NULL) {
		device_printf(dev, "Failed to alloc GpioInt resource\n");
		iic_free_resource(dev, sc->iicres);
		return (ENXIO);
	}
#endif

	pci_set_powerstate(dev, PCI_POWERSTATE_D0);

	AcpiInstallAddressSpaceHandler(sc->handle, ACPI_ADR_SPACE_GPIO,
	    intel_xpower_pmic_gpio_handler, NULL, NULL);

	return (0);
}

static int
intelpmic_detach(device_t dev)
{
	struct intelpmic_softc *sc = device_get_softc(dev);

	AcpiRemoveAddressSpaceHandler(sc->handle, ACPI_ADR_SPACE_GPIO,
	    intel_xpower_pmic_gpio_handler);

	pci_set_powerstate(dev, PCI_POWERSTATE_D3);

#if 0
	if (sc->intres != NULL)
		gpioint_free_resource(dev, sc->intres);
#endif
	if (sc->iicres != NULL)
		iic_free_resource(dev, sc->iicres);

	return (0);
}

static device_method_t intelpmic_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, intelpmic_probe),
	DEVMETHOD(device_attach, intelpmic_attach),
	DEVMETHOD(device_detach, intelpmic_detach),

	DEVMETHOD_END
};

static driver_t intelpmic_driver = {
	"intelpmic",
	intelpmic_methods,
	sizeof(struct intelpmic_softc),
};
static devclass_t intelpmic_devclass;

DRIVER_MODULE(intelpmic, acpi, intelpmic_driver, intelpmic_devclass, NULL, NULL);
MODULE_DEPEND(intelpmic, acpi, 1, 1, 1);
MODULE_DEPEND(intelpmic, gpio_acpi, 1, 1, 1);
MODULE_DEPEND(intelpmic, smbacpi, 1, 1, 1);
MODULE_VERSION(intelpmic, 1);

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/resource.h>

#include "opt_acpi.h"
#include "acpi.h"
#include <dev/acpica/acpivar.h>

#include <bus/gpio/gpio_acpi/gpio_acpivar.h>

ACPI_MODULE_NAME("acpi_array");

struct button {
	struct acpi_array_softc *button_sc;
	struct gpioint_resource *res;
	int valid;
	uint32_t intidx;
	uint32_t page;
	uint32_t usage;
};

struct acpi_array_softc {
	device_t	dev;
	ACPI_HANDLE	handle;

	int nbuttons;
	struct button *buttons;
};

static void acpi_array_intr(void *arg);

static int
acpi_array_probe(device_t dev)
{
	static char *acpi_array_ids[] = { "ACPI0011", NULL };
//	static char *acpi_array_ids[] = { "PNP0C40", NULL };

	if (acpi_disabled("acpi_array") ||
	    ACPI_ID_PROBE(device_get_parent(dev), dev, acpi_array_ids) == NULL)
		return (ENXIO);

	device_set_desc(dev, "ACPI HID buttons");
	return (0);
}

static int
parse_dsd(struct acpi_array_softc *sc)
{
	ACPI_HANDLE handle = acpi_get_handle(sc->dev);
	ACPI_STATUS status;
	ACPI_BUFFER dsd_buf;
	ACPI_OBJECT *res, *buttons, *entry;
	uint32_t val;
	int i, cnt;

	dsd_buf.Length = ACPI_ALLOCATE_BUFFER;
	dsd_buf.Pointer = NULL;
	status = AcpiEvaluateObject(handle, "_DSD", NULL, &dsd_buf);
	if (ACPI_FAILURE(status)) {
		if (status != AE_NOT_FOUND)
			device_printf(sc->dev, "can't evaluate %s._DSD - %s\n",
			    acpi_name(handle), AcpiFormatException(status));
		return (ENXIO);
	}
	res = (ACPI_OBJECT *)dsd_buf.Pointer;
	if (!ACPI_PKG_VALID_EQ(res, 2)) {
		device_printf(sc->dev, "evaluation of %s._DSD makes no sense\n",
		    acpi_name(handle));
		AcpiOsFree(dsd_buf.Pointer);
		return (ENXIO);
	}

	/*
	 * XXX check if UUID (first entry of the package) is really
	 *     "fa6bd625-9ce8-470d-a2c7-b3ca36c4282e"
	 */

	buttons = &res->Package.Elements[1];
	if (!ACPI_PKG_VALID(buttons, 1)) {
		device_printf(sc->dev, "evaluation of %s._DSD makes no sense\n",
		    acpi_name(handle));
		AcpiOsFree(dsd_buf.Pointer);
		return (ENXIO);
	}

	cnt = 0;
	for (i = 0; i < buttons->Package.Count; i++) {
		entry = &buttons->Package.Elements[i];
		if (!ACPI_PKG_VALID(entry, 5)) {
			device_printf(sc->dev,
			    "evaluation of %s._DSD makes no sense\n",
			    acpi_name(handle));
			AcpiOsFree(dsd_buf.Pointer);
			return (ENXIO);
		}
		if (acpi_PkgInt32(entry, 0, &val) != 0) {
			device_printf(sc->dev,
			    "Can't extract _DSD %dth button Int32 value\n", i);
		}
		if (val == 1)
			cnt++;
	}
	device_printf(sc->dev, "%d buttons\n", cnt);
	sc->nbuttons = cnt;
	sc->buttons = kmalloc(cnt*sizeof(*sc->buttons), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	cnt = 0;
	for (i = 0; i < buttons->Package.Count; i++) {
		entry = &buttons->Package.Elements[i];
		if (acpi_PkgInt32(entry, 0, &val) != 0) {
			device_printf(sc->dev,
			    "Can't extract _DSD %dth button Int32 value\n", i);
			continue;
		}
		if (val == 0)
			continue;
		sc->buttons[cnt].button_sc = sc;
		if (acpi_PkgInt32(entry, 1, &val) != 0) {
			device_printf(sc->dev,
			    "Can't extract _DSD %dth button Int32 value\n", i);
			cnt++;
			continue;
		}
		sc->buttons[cnt].intidx = val;
		sc->buttons[cnt].valid = 1;
		if (acpi_PkgInt32(entry, 3, &val) != 0) {
			device_printf(sc->dev,
			    "Can't extract _DSD %dth button Int32 value\n", i);
			sc->buttons[cnt].page = 0;
		} else {
			sc->buttons[cnt].page = val;
		}
		if (acpi_PkgInt32(entry, 4, &val) != 0) {
			device_printf(sc->dev,
			    "Can't extract _DSD %dth button Int32 value\n", i);
			sc->buttons[cnt].usage = 0;
		} else {
			sc->buttons[cnt].usage = val;
		}
		cnt++;
	}

	AcpiOsFree(dsd_buf.Pointer);

	return (0);
}

static int
acpi_array_attach(device_t dev)
{
	struct acpi_array_softc *sc = device_get_softc(dev);
	int i, haveint = 0;

	sc->dev = dev;
	sc->handle = acpi_get_handle(dev);

	if (parse_dsd(sc) != 0)
		return (ENXIO);

	/* Allocate GPIO interrupt pins */
	for (i = 0; i < sc->nbuttons; i++) {
		if (!sc->buttons[i].valid)
			continue;
		sc->buttons[i].res = gpioint_alloc_resource(dev,
		    sc->buttons[i].intidx);
		if (sc->buttons[i].res == NULL) {
			device_printf(dev,
			    "Gpio Interrupt %d allocation failed\n", i);
		} else {
			haveint = 1;
		}
	}

	if (!haveint) {
		kfree(sc->buttons, M_DEVBUF);
		return (ENXIO);
	}

	for (i = 0; i < sc->nbuttons; i++) {
		if (sc->buttons[i].res != NULL) {
			gpioint_establish_interrupt(sc->buttons[i].res,
			    acpi_array_intr, &sc->buttons[i]);
		}
	}

	return (0);
}

static int
acpi_array_detach(device_t dev)
{
	struct acpi_array_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->nbuttons; i++) {
		if (sc->buttons[i].res != NULL) {
			gpioint_release_interrupt(sc->buttons[i].res);
			gpioint_free_resource(dev, sc->buttons[i].res);
		}
	}

	if (sc->buttons != NULL)
		kfree(sc->buttons, M_DEVBUF);

	return (0);
}

static void
acpi_array_powerbutton(struct acpi_array_softc *sc)
{
	struct acpi_softc *acpi_sc = acpi_device_get_parent_softc(sc->dev);

	ACPI_VPRINT(sc->dev, acpi_sc, "power button pressed\n");
        acpi_event_power_button_sleep(acpi_sc);
}


static void
acpi_array_intr(void *arg)
{
	struct button *button = (struct button *)arg;
	struct acpi_array_softc *sc = button->button_sc;

	device_printf(sc->dev, "gpio interrupt\n");

	if (button->page == 0x01 && button->usage == 0x81) {
		acpi_array_powerbutton(sc);
	}
}

static device_method_t acpi_array_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, acpi_array_probe),
	DEVMETHOD(device_attach, acpi_array_attach),
	DEVMETHOD(device_detach, acpi_array_detach),

	DEVMETHOD_END
};

static driver_t acpi_array_driver = {
	"acpi_array",
	acpi_array_methods,
	sizeof(struct acpi_array_softc),
};
static devclass_t acpi_array_devclass;

DRIVER_MODULE(acpi_array, acpi, acpi_array_driver, acpi_array_devclass, NULL,
    NULL);
MODULE_DEPEND(acpi_array, acpi, 1, 1, 1);
MODULE_DEPEND(acpi_array, gpio_acpi, 1, 1, 1);

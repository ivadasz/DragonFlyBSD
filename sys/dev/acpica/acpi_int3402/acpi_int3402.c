#include "opt_acpi.h"
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/sensors.h>

#include "acpi.h"

#include <dev/acpica/acpivar.h>

#define TZ_ZEROC	2732
#define TZ_KELVTOC(x)	(((x) - TZ_ZEROC) / 10), abs(((x) - TZ_ZEROC) % 10)

#define INT3402_PERF_CHANGED_EVENT	0x80
#define INT3402_THERMAL_EVENT		0x90

ACPI_MODULE_NAME("INT340X")

struct acpi_int3402_softc {
	device_t		dev;
	ACPI_HANDLE		handle;
	struct ksensor		sensor;
	struct ksensordev	sensordev;
	int			tmp_working;
};

static int	acpi_int3402_probe(device_t dev);
static int	acpi_int3402_attach(device_t dev);
static int	acpi_int3402_detach(device_t dev);
static void	acpi_int3402_notify_handler(ACPI_HANDLE h, UINT32 notify,
					    void *context);
static void	acpi_int3402_refresh(void *arg);
static void	acpi_int3402_infos(struct acpi_int3402_softc *sc);
static void	acpi_int3402_lookup(struct acpi_int3402_softc *sc, char *str);

static device_method_t acpi_int3402_methods[] = {
	DEVMETHOD(device_probe,		acpi_int3402_probe),
	DEVMETHOD(device_attach,	acpi_int3402_attach),
	DEVMETHOD(device_detach,	acpi_int3402_detach),

	DEVMETHOD_END
};

static driver_t acpi_int3402_driver = {
	"acpi_int3402tz",
	acpi_int3402_methods,
	sizeof(struct acpi_int3402_softc),
};

static devclass_t acpi_int3402_devclass;
DRIVER_MODULE(acpi_int3402, acpi, acpi_int3402_driver, acpi_int3402_devclass,
    NULL, NULL);
MODULE_DEPEND(acpi_int3402, acpi, 1, 1, 1);

static int
acpi_int3402_probe(device_t dev)
{
	static char *int3402_ids[] = {
		"INT3400", "INT3401", "INT3402", "INT3403", NULL
	};

	if (acpi_disabled("int3402") ||
	    ACPI_ID_PROBE(device_get_parent(dev), dev, int3402_ids) == NULL)
		return (ENXIO);

	device_set_desc(dev, "INT3402 thermal");
	return (0);
}

static int
acpi_int3402_attach(device_t dev)
{
	struct		acpi_int3402_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->handle = acpi_get_handle(dev);
	sc->tmp_working = 0;

	strlcpy(sc->sensordev.xname, device_get_nameunit(dev),
	    sizeof(sc->sensordev.xname));
	sc->sensor.type = SENSOR_TEMP;

	acpi_int3402_refresh(sc);

	acpi_int3402_infos(sc);

	AcpiInstallNotifyHandler(sc->handle, ACPI_DEVICE_NOTIFY,
				 acpi_int3402_notify_handler, sc);

	if (sc->tmp_working) {
		sensor_task_register(sc, acpi_int3402_refresh, 5);
		sensor_attach(&sc->sensordev, &sc->sensor);
		sensordev_install(&sc->sensordev);
	}

	return (0);
}

static int
acpi_int3402_detach(device_t dev)
{
	struct		acpi_int3402_softc *sc;

	sc = device_get_softc(dev);
	if (sc->tmp_working)
		sensor_task_unregister(sc);
	AcpiRemoveNotifyHandler(sc->handle, ACPI_DEVICE_NOTIFY,
	    acpi_int3402_notify_handler);

	if (sc->tmp_working)
		sensordev_deinstall(&sc->sensordev);

	return (0);
}

static void
acpi_int3402_notify_handler(ACPI_HANDLE h, UINT32 notify, void *context)
{
	struct acpi_int3402_softc *sc = (struct acpi_int3402_softc *)context;

	device_printf(sc->dev, "notify: 0x%02x\n", notify);

	switch (notify) {
	case INT3402_PERF_CHANGED_EVENT:
		break;
	case INT3402_THERMAL_EVENT:
		acpi_int3402_refresh(sc);
		break;
	}
}

static void
acpi_int3402_lookup(struct acpi_int3402_softc *sc, char *str)
{
	ACPI_STATUS	status;
	int		value;

	status = acpi_GetInteger(sc->handle, str, &value);
	if (ACPI_FAILURE(status)) {
		device_printf(sc->dev, "evaluating %s failed\n", str);
	} else if (value > TZ_ZEROC) {
		device_printf(sc->dev, "%s=%d.%dC\n", str, TZ_KELVTOC(value));
	}
}

static void
acpi_int3402_infos(struct acpi_int3402_softc *sc)
{
	acpi_int3402_lookup(sc, "_HOT");
	acpi_int3402_lookup(sc, "_CRT");
	acpi_int3402_lookup(sc, "_CR3");
	acpi_int3402_lookup(sc, "_PSV");
	acpi_int3402_lookup(sc, "_AC0");
	acpi_int3402_lookup(sc, "_AC1");
}

static void
acpi_int3402_refresh(void *arg)
{
	struct acpi_int3402_softc *sc = (struct acpi_int3402_softc *)arg;
	ACPI_STATUS status;
	int temp;

	status = acpi_GetInteger(sc->handle, "_TMP", &temp);
	if (ACPI_FAILURE(status)) {
		device_printf(sc->dev, "evaluating _TMP failed\n");
	} else {
		sc->tmp_working = 1;
	}
//	device_printf(sc->dev, "_TMP=%d\n", temp);

	sc->sensor.value = temp * 100000 - 50000;
}

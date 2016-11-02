#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/sensors.h>
#include <sys/eventhandler.h>
#include <sys/reboot.h>

#define BYT_PMC_FN_DISABLE		0x34
#define BYT_PMC_FN_DISABLE2		0x38
#define BYT_PMC_S0I_READY_RESIDENCY	0x80
#define BYT_PMC_S0I1_RESIDENCY		0x84
#define BYT_PMC_S0I2_RESIDENCY		0x88
#define BYT_PMC_S0I3_RESIDENCY		0x8c
#define BYT_PMC_POWER_STATUS		0x98

struct bytpmc_softc {
	device_t		sc_dev;
	struct resource		*sc_regs;
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;
	struct ksensor		sc_sensor[4];
	struct ksensordev	sc_sensordev;
	uint32_t		lastvals[4];
};

static int	bytpmc_probe(device_t);
static int	bytpmc_attach(device_t);
static int	bytpmc_detach(device_t);
static void	bytpmc_refresh(void *);
static void	bytpmc_shutdown_final(void *arg, int howto);

static device_method_t bytpmc_methods[] = {
	DEVMETHOD(device_probe,		bytpmc_probe),
	DEVMETHOD(device_attach,	bytpmc_attach),
	DEVMETHOD(device_detach,	bytpmc_detach),
	DEVMETHOD_END
};

static driver_t bytpmc_driver = {
	"bytpmc",
	bytpmc_methods,
	sizeof(struct bytpmc_softc)
};

static devclass_t bytpmc_devclass;

DRIVER_MODULE(bytpmc, nexus, bytpmc_driver, bytpmc_devclass, NULL, NULL);

static int
bytpmc_probe(device_t dev)
{
	if (device_get_unit(dev) != 0)
		return ENXIO;

	device_set_desc(dev, "Baytrail PCU PMC");

	return 0;
}

static int
bytpmc_attach(device_t dev)
{
	struct bytpmc_softc	*sc;
	int i, rid;

	if (device_get_unit(dev) != 0)
		return ENXIO;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	rid = 0;
	sc->sc_regs = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_regs == NULL) {
		device_printf(dev, "unable to map registers\n");
		return ENXIO;
	}
	sc->sc_iot = rman_get_bustag(sc->sc_regs);
	sc->sc_ioh = rman_get_bushandle(sc->sc_regs);

	device_printf(dev, "function disable: 0x%08x\n",
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, BYT_PMC_FN_DISABLE));
	device_printf(dev, "function disable 2: 0x%08x\n",
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, BYT_PMC_FN_DISABLE2));
	device_printf(dev, "power island power status: 0x%08x\n",
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, BYT_PMC_POWER_STATUS));

	strlcpy(sc->sc_sensordev.xname, device_get_nameunit(dev),
	    sizeof(sc->sc_sensordev.xname));

	for (i = 0; i < 4; i++) {
		sc->sc_sensor[i].type = SENSOR_PERCENT;
		sensor_attach(&sc->sc_sensordev, &sc->sc_sensor[i]);
	}
	sc->lastvals[0] = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    BYT_PMC_S0I_READY_RESIDENCY);
	sc->lastvals[1] = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    BYT_PMC_S0I1_RESIDENCY);
	sc->lastvals[2] = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    BYT_PMC_S0I2_RESIDENCY);
	sc->lastvals[3] = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    BYT_PMC_S0I3_RESIDENCY);

	sensor_task_register(sc, bytpmc_refresh, 1);

	sensordev_install(&sc->sc_sensordev);

	EVENTHANDLER_REGISTER(shutdown_final, bytpmc_shutdown_final, sc,
	    SHUTDOWN_PRI_LAST);

	return 0;
}

static int
bytpmc_detach(device_t dev)
{
	struct bytpmc_softc *sc = device_get_softc(dev);

	sensordev_deinstall(&sc->sc_sensordev);
	sensor_task_unregister(sc);

	bus_release_resource(sc->sc_dev, SYS_RES_MEMORY, 0, sc->sc_regs);

	return 0;
}

static void
bytpmc_refresh(void *arg)
{
	struct bytpmc_softc *sc = arg;
	struct ksensor *s;
	uint32_t vals[4];
	int i;

	vals[0] = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    BYT_PMC_S0I_READY_RESIDENCY);
	vals[1] = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    BYT_PMC_S0I1_RESIDENCY);
	vals[2] = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    BYT_PMC_S0I2_RESIDENCY);
	vals[3] = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    BYT_PMC_S0I3_RESIDENCY);

	for (i = 0; i < 4; i++) {
		s = &sc->sc_sensor[i];
		s->value = (uint32_t)((vals[i] - sc->lastvals[i]) * 32 / 1000);
		sc->lastvals[i] = vals[i];
	}
}

static void
bytpmc_shutdown_final(void *arg, int howto)
{
	if ((howto & RB_POWEROFF) == 0 && (howto & RB_HALT) == 0) {
		outb(0xCF9, 0x0E);
		DELAY(1000000);
		kprintf("bytpmc reset failed - timeout\n");
	}
}

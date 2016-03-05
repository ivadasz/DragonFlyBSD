#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/sensors.h>

#define BYT_PMC_FN_DISABLE	0x34
#define BYT_PMC_FN_DISABLE2	0x38

struct bytpmc_softc {
	struct device		*sc_dev;
	struct resource		*sc_regs;
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;
//	struct ksensor		sc_sensor;
//	struct ksensordev	sc_sensordev;
};

static int	bytpmc_probe(struct device *);
static int	bytpmc_attach(struct device *);
static int	bytpmc_detach(struct device *);
//static void	bytpmc_refresh(void *);

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
bytpmc_probe(struct device *dev)
{
	if (device_get_unit(dev) != 0)
		return ENXIO;

	device_set_desc(dev, "Baytrail PCU PMC");

	return 0;
}

static int
bytpmc_attach(struct device *dev)
{
	struct bytpmc_softc	*sc;
	int rid;

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

#if 0
	strlcpy(sc->sc_sensordev.xname, device_get_nameunit(dev),
	    sizeof(sc->sc_sensordev.xname));

	sc->sc_sensor.type = SENSOR_TEMP;
	sensor_attach(&sc->sc_sensordev, &sc->sc_sensor);

	sensor_task_register(sc, km_refresh, 5);

	sensordev_install(&sc->sc_sensordev);
#endif

	return 0;
}

static int
bytpmc_detach(struct device *dev)
{
	struct bytpmc_softc *sc = device_get_softc(dev);

	bus_release_resource(sc->sc_dev, SYS_RES_MEMORY, 0, sc->sc_regs);

#if 0

	sensordev_deinstall(&sc->sc_sensordev);
	sensor_task_unregister(sc);
#endif

	return 0;
}

#if 0
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
#endif

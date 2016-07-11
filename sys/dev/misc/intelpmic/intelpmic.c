#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/lock.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bitops.h>
#include <sys/sysctl.h>
#include <sys/uio.h>
#include <sys/fcntl.h>
#include <sys/uuid.h>
#include <sys/event.h>
#include <sys/devfs.h>
#include <sys/sensors.h>
#include <sys/vnode.h>

#include "opt_acpi.h"
#include "acpi.h"
#include <dev/acpica/acpivar.h>

#include <bus/pci/pcivar.h>

#include <bus/gpio/gpio_acpi/gpio_acpivar.h>
#include <bus/smbus/smbacpi/smbus_acpivar.h>

#include "axp20x.h"
#include "acpi_lpat.h"

#define PMIC_POWER_OPREGION_ID		0x8d
#define PMIC_THERMAL_OPREGION_ID	0x8c

struct intelpmic_softc {
	device_t	dev;
	ACPI_HANDLE	handle;

	struct lock	lk;
	struct lwkt_serialize power_slz;
	struct lwkt_serialize thermal_slz;

	struct iicserial_resource *iicres;
	struct gpioint_resource *intres;

	struct acpi_lpat_conversion_table *lpat_table;

	int checked_fuel_gauge_control;

	struct ksensor	watt_sensor;
	struct ksensordev sensordev;
};

static ACPI_STATUS	intel_xpower_pmic_gpio_handler(UINT32 Function,
			    ACPI_PHYSICAL_ADDRESS Address, UINT32 BitWidth,
			    UINT64 *Value, void *HandlerContext,
			    void *RegionContext);

static int	intelpmic_probe(device_t dev);
static int	intelpmic_attach(device_t dev);
static int	intelpmic_detach(device_t dev);

#define XPOWER_GPADC_LOW	0x5b

struct pmic_table {
	int address;	/* operation region address */
	int reg;	/* corresponding thermal register */
	int bit;	/* control bit for power */
};

static struct pmic_table power_table[] = {
	{
		.address = 0x00,
		.reg = 0x13,
		.bit = 0x05,
	},
	{
		.address = 0x04,
		.reg = 0x13,
		.bit = 0x06,
	},
	{
		.address = 0x08,
		.reg = 0x13,
		.bit = 0x07,
	},
	{
		.address = 0x0c,
		.reg = 0x12,
		.bit = 0x03,
	},
	{
		.address = 0x10,
		.reg = 0x12,
		.bit = 0x04,
	},
	{
		.address = 0x14,
		.reg = 0x12,
		.bit = 0x05,
	},
	{
		.address = 0x18,
		.reg = 0x12,
		.bit = 0x06,
	},
	{
		.address = 0x1c,
		.reg = 0x12,
		.bit = 0x00,
	},
	{
		.address = 0x20,
		.reg = 0x12,
		.bit = 0x01,
	},
	{
		.address = 0x24,
		.reg = 0x12,
		.bit = 0x02,
	},
	{
		.address = 0x28,
		.reg = 0x13,
		.bit = 0x02,
	},
	{
		.address = 0x2c,
		.reg = 0x13,
		.bit = 0x03,
	},
	{
		.address = 0x30,
		.reg = 0x13,
		.bit = 0x04,
	},
	{
		.address = 0x38,
		.reg = 0x10,
		.bit = 0x03,
	},
	{
		.address = 0x3c,
		.reg = 0x10,
		.bit = 0x06,
	},
	{
		.address = 0x40,
		.reg = 0x10,
		.bit = 0x05,
	},
	{
		.address = 0x44,
		.reg = 0x10,
		.bit = 0x04,
	},
	{
		.address = 0x48,
		.reg = 0x10,
		.bit = 0x01,
	},
	{
		.address = 0x4c,
		.reg = 0x10,
		.bit = 0x00
	},
};

/* TMP0 - TMP5 are the same, all from GPADC */
static struct pmic_table thermal_table[] = {
	{
		.address = 0x00,
		.reg = XPOWER_GPADC_LOW
	},
	{
		.address = 0x0c,
		.reg = XPOWER_GPADC_LOW
	},
	{
		.address = 0x18,
		.reg = XPOWER_GPADC_LOW
	},
	{
		.address = 0x24,
		.reg = XPOWER_GPADC_LOW
	},
	{
		.address = 0x30,
		.reg = XPOWER_GPADC_LOW
	},
	{
		.address = 0x3c,
		.reg = XPOWER_GPADC_LOW
	},
};

static int
intel_xpower_pmic_get_power(struct intelpmic_softc *sc, int reg, int bit,
    UINT64 *Value)
{
	uint8_t byte;

	if (iicserial_readb(sc->iicres, reg, &byte))
		return -EIO;

	*Value = (byte & __BIT(bit)) ? 1 : 0;
	return 0;
}

static int
intel_xpower_pmic_update_power(struct intelpmic_softc *sc, int reg, int bit,
    boolean_t on)
{
	uint8_t byte;

	if (iicserial_readb(sc->iicres, reg, &byte))
		return -EIO;

	if (on)
		byte |= __BIT(bit);
	else
		byte &= ~__BIT(bit);

	if (iicserial_writeb(sc->iicres, reg, byte))
		return -EIO;

        return 0;
}

static int
intel_xpower_pmic_get_raw_temp(struct intelpmic_softc *sc, int reg)
{
	uint16_t word;

	if (iicserial_readw(sc->iicres, 0x5a, &word))
		return -EIO;

	return (((uint8_t *)&word)[0] << 4) | (((uint8_t *)&word)[1] & 0x0f);
}

static ACPI_STATUS
intel_xpower_pmic_gpio_handler(UINT32 Function, ACPI_PHYSICAL_ADDRESS Address,
    UINT32 BitWidth, UINT64 *Value, void *HandlerContext, void *RegionContext)
{
	return (AE_OK);
}

static int
intelpmic_get_reg_bit(int address, struct pmic_table *table, int count,
    int *reg, int *bit)
{
	int i;

	for (i = 0; i < count; i++) {
		if (table[i].address == address) {
			*reg = table[i].reg;
			if (bit)
				*bit = table[i].bit;
			return (0);
		}
	}
	return (-ENOENT);
}

static ACPI_STATUS
intelpmic_power_handler(UINT32 Function, ACPI_PHYSICAL_ADDRESS Address,
    UINT32 BitWidth, UINT64 *Value, void *HandlerContext, void *RegionContext)
{
	struct intelpmic_softc *sc = RegionContext;
	int reg, bit, result;

	if (BitWidth != 32 || Value == NULL)
		return (AE_BAD_PARAMETER);

	if (Function == ACPI_WRITE && !(*Value == 0 || *Value == 1))
		return (AE_BAD_PARAMETER);

	result = intelpmic_get_reg_bit(Address, power_table,
	    NELEM(power_table), &reg, &bit);
	if (result == -ENOENT)
		return (AE_BAD_PARAMETER);

	lwkt_serialize_enter(&sc->power_slz);

	if (Function == ACPI_WRITE) {
		device_printf(sc->dev, "write: reg=0x%x bit=%u Value=%llu\n",
		    reg, bit, (unsigned long long)*Value);
	}

	result = Function == ACPI_READ ?
	    intel_xpower_pmic_get_power(sc, reg, bit, Value) :
	    intel_xpower_pmic_update_power(sc, reg, bit, *Value == 1);

	if (Function == ACPI_READ) {
		device_printf(sc->dev, "%s: read: reg=0x%x bit=%u Value=%llu\n",
		    __func__, reg, bit, (unsigned long long)*Value);
	}

	lwkt_serialize_exit(&sc->power_slz);

	return (result ? AE_ERROR : AE_OK);
}

static int
intelpmic_read_temp(struct intelpmic_softc *sc, int reg, UINT64 *Value)
{
	int raw_temp, temp;

	raw_temp = intel_xpower_pmic_get_raw_temp(sc, reg);
	if (raw_temp < 0)
		return (raw_temp);

	if (sc->lpat_table == NULL) {
		*Value = raw_temp;
		return (0);
	}

	temp = acpi_lpat_raw_to_temp(sc->lpat_table, raw_temp);
	if (temp < 0)
		return temp;

	*Value = temp;
	return (0);
}

static int
intelpmic_thermal_temp(struct intelpmic_softc *sc, int reg,
    UINT32 Function, UINT64 *Value)
{
        return Function == ACPI_READ ?
	    intelpmic_read_temp(sc, reg, Value) : -EINVAL;
}

static int
intelpmic_thermal_aux(struct intelpmic_softc *sc, int reg, UINT32 Function,
    UINT64 *Value)
{
	if (Function == ACPI_READ)
		return intelpmic_read_temp(sc, reg, Value);

	return -ENXIO;
}

static int
intelpmic_thermal_pen(struct intelpmic_softc *sc, int reg, UINT32 Function,
    UINT64 *Value)
{
	return -ENXIO;
}

static boolean_t
intelpmic_thermal_is_temp(int address)
{
	return (address <= 0x3c) && !(address % 12);
}

static boolean_t
intelpmic_thermal_is_aux(int address)
{
	return (address >= 4 && address <= 0x40 && !((address - 4) % 12)) ||
	       (address >= 8 && address <= 0x44 && !((address - 8) % 12));
}

static boolean_t
intelpmic_thermal_is_pen(int address)
{
	return address >= 0x48 && address <= 0x5c;
}

static ACPI_STATUS
intelpmic_thermal_handler(UINT32 Function,
    ACPI_PHYSICAL_ADDRESS Address, UINT32 BitWidth, UINT64 *Value,
    void *HandlerContext, void *RegionContext)
{
	struct intelpmic_softc *sc = RegionContext;
	int reg, result;

	if (BitWidth != 32 || Value == NULL)
		return (AE_BAD_PARAMETER);

	result = intelpmic_get_reg_bit(Address, thermal_table,
	    NELEM(thermal_table), &reg, NULL);
	if (result == -ENOENT)
		return (AE_BAD_PARAMETER);

        lwkt_serialize_enter(&sc->thermal_slz);

	if (intelpmic_thermal_is_temp(Address))
		result = intelpmic_thermal_temp(sc, reg, Function, Value);
	else if (intelpmic_thermal_is_aux(Address))
		result = intelpmic_thermal_aux(sc, reg, Function, Value);
	else if (intelpmic_thermal_is_pen(Address))
		result = intelpmic_thermal_pen(sc, reg, Function, Value);
	else
		result = -EINVAL;

        lwkt_serialize_exit(&sc->thermal_slz);

#if 0
	device_printf(sc->dev, "%s: read: reg=0x%x result=%d Value=0x%llx\n",
	    __func__, reg, result, (unsigned long long)*Value);
#endif

	if (result < 0) {
		if (result == -EINVAL)
			return (AE_BAD_PARAMETER);
		else
			return (AE_ERROR);
	}

	return (AE_OK);
}

static int
intelpmic_read_regpair(struct intelpmic_softc *sc, uint8_t reg, uint16_t *val)
{
	int ret;
	uint8_t a, b;

	ret = iicserial_readb(sc->iicres, reg, &a);
	if (ret != 0)
		return (ret);
	ret = iicserial_readb(sc->iicres, reg + 1, &b);
	if (ret != 0)
		return (ret);

	*val = (a << 4) | (b & 0x0f);
	return (0);
}

static void
intelpmic_refresh(void *arg)
{
	struct intelpmic_softc *sc = arg;
	int ret;
	uint16_t voltage = 0, current = 0;
	uint8_t a, b;

	ret = iicserial_readb(sc->iicres, 0x00, &a);
	if (ret == 0) {
		ret = iicserial_readb(sc->iicres, 0x01, &b);
		if (ret == 0) {
#if 0
			device_printf(sc->dev,
			    "power source: 0x%02x, power mode: 0x%02x\n",
			    a, b);
#endif
		} else {
			return;
		}
	} else {
		return;
	}

	if (!sc->checked_fuel_gauge_control) {
		uint8_t val;
		ret = iicserial_readb(sc->iicres, 0xb8, &val);
		if (ret == 0) {
			sc->checked_fuel_gauge_control = 1;
			device_printf(sc->dev, "fuel gauge control: 0x%02x\n",
			    val);
		}
	}

	ret = intelpmic_read_regpair(sc, 0x78, &voltage);
	if (ret != 0) {
		device_printf(sc->dev, "failed to read voltage\n");
	}

	if (a & 0x40) {
		ret = intelpmic_read_regpair(sc, 0x7a, &current);
		if (ret != 0) {
			device_printf(sc->dev, "failed to read current\n");
		}
	} else {
		ret = intelpmic_read_regpair(sc, 0x7c, &current);
		if (ret != 0) {
			device_printf(sc->dev, "failed to read current\n");
		}
	}
#if 0
	device_printf(sc->dev,
	    "voltage: %u mV, %s current: %u mA -> %u mW\n",
	    (voltage * 11) / 10,
	    (a & 0x40) ? "charging" : "discharging",
	    current, (voltage * 11 * current) / 10000);
#endif
	int64_t val = ((voltage * 11) * current) / 10;
	sc->watt_sensor.value = (a & 0x40) ? val : -val;
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
	lwkt_serialize_init(&sc->power_slz);
	lwkt_serialize_init(&sc->thermal_slz);

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

	sc->lpat_table = acpi_lpat_get_conversion_table(sc->handle);

	AcpiInstallAddressSpaceHandler(sc->handle, ACPI_ADR_SPACE_GPIO,
	    intel_xpower_pmic_gpio_handler, NULL, NULL);

	AcpiInstallAddressSpaceHandler(sc->handle, PMIC_POWER_OPREGION_ID,
	    intelpmic_power_handler, NULL, sc);

	AcpiInstallAddressSpaceHandler(sc->handle, PMIC_THERMAL_OPREGION_ID,
	    intelpmic_thermal_handler, NULL, sc);

	strlcpy(sc->sensordev.xname, device_get_nameunit(dev),
	    sizeof(sc->sensordev.xname));

	sc->watt_sensor.type = SENSOR_WATTS;
	sensor_attach(&sc->sensordev, &sc->watt_sensor);

	sensor_task_register(sc, intelpmic_refresh, 5);
	sensordev_install(&sc->sensordev);

	return (0);
}

static int
intelpmic_detach(device_t dev)
{
	struct intelpmic_softc *sc = device_get_softc(dev);

	sensordev_deinstall(&sc->sensordev);
	sensor_task_unregister(sc);

	AcpiRemoveAddressSpaceHandler(sc->handle, ACPI_ADR_SPACE_GPIO,
	    intel_xpower_pmic_gpio_handler);

	AcpiRemoveAddressSpaceHandler(sc->handle, PMIC_POWER_OPREGION_ID,
	    intelpmic_power_handler);

	AcpiRemoveAddressSpaceHandler(sc->handle, PMIC_THERMAL_OPREGION_ID,
	    intelpmic_thermal_handler);

	acpi_lpat_get_conversion_table(sc->lpat_table);

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

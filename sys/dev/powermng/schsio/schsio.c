/*
 * Copyright (c) 2014 Imre Vadasz <imre@vdsz.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <bus/isa/isavar.h>
#include <sys/systm.h>

#include <sys/sensors.h>

#if defined(SCHSIODEBUG)
#define DPRINTF(x)		do { kprintf x; } while (0)
#else
#define DPRINTF(x)
#endif

#define SCHSIO_NUM_SENSORS	17

/* chip ids */
#define SCHSIO_ID_SCH5627	0xC6

/* ctl registers */

#define SCHSIO_IOSIZE	0x09
#define SCHSIO_PORT_CONFIG	0x00

/* used in configuration mode */

#define SCHSIO_PORT_INDEX	0x00
#define SCHSIO_PORT_DATA	0x01

#define SCHSIO_CONFIG_ENTER	0x55
#define SCHSIO_CONFIG_LEAVE	0xaa

/* registers */

#define SCHSIO_IDX_LDEVSEL	0x07
#define SCHSIO_IDX_DEVICE	0x20
#define SCHSIO_IDX_ENABLE	0x30
#define SCHSIO_IDX_ADDR		0x66
#define SCHSIO_IDX_REV		0x21
#define SCHSIO_IDX_BASE_HI	0x60
#define SCHSIO_IDX_BASE_LO	0x61

/* Logical devices */
#define SCHSIO_LDEV_RUNTIME	0x0c
#define SCHSIO_LDEV_RUNTIME_SZ	10

/* Register access */

#define SCHSIO_HWM_INDEX	0x70
#define SCHSIO_HWM_DATA		0x71

/* Sensor registers */
/* Voltage */
#define SCHSIO_VOLT1_LSB	0xe4
#define SCHSIO_VOLT1_MSB	0x22
#define SCHSIO_VOLT2_LSB	0xe4
#define SCHSIO_VOLT2_MSB	0x23
#define SCHSIO_VOLT3_LSB	0xe3
#define SCHSIO_VOLT3_MSB	0x24
#define SCHSIO_VOLT4_LSB	0xe3
#define SCHSIO_VOLT4_MSB	0x25
#define SCHSIO_VOLT5_LSB	0x18a
#define SCHSIO_VOLT5_MSB	0x189

/* Temperature */
#define SCHSIO_TEMP1_LSB	0xe2
#define SCHSIO_TEMP1_MSB	0x2b
#define SCHSIO_TEMP2_LSB	0xe1
#define SCHSIO_TEMP2_MSB	0x26
#define SCHSIO_TEMP3_LSB	0xe1
#define SCHSIO_TEMP3_MSB	0x27
#define SCHSIO_TEMP4_LSB	0xe5
#define SCHSIO_TEMP4_MSB	0x28
#define SCHSIO_TEMP5_LSB	0xe5
#define SCHSIO_TEMP5_MSB	0x29
#define SCHSIO_TEMP6_LSB	0xe6
#define SCHSIO_TEMP6_MSB	0x2a
#define SCHSIO_TEMP7_LSB	0x182
#define SCHSIO_TEMP7_MSB	0x180
#define SCHSIO_TEMP8_LSB	0x182
#define SCHSIO_TEMP8_MSB	0x181

/* Fan speed */
#define SCHSIO_FAN1_LSB	0x2c
#define SCHSIO_FAN1_MSB	0x2d
#define SCHSIO_FAN2_LSB	0x2e
#define SCHSIO_FAN2_MSB	0x2f
#define SCHSIO_FAN3_LSB	0x30
#define SCHSIO_FAN3_MSB	0x31
#define SCHSIO_FAN4_LSB	0x32
#define SCHSIO_FAN4_MSB	0x33

/* Conversion parameters */
#define SCHSIO_CONV_VOLT1	10745
#define SCHSIO_CONV_VOLT2	3660
#define SCHSIO_CONV_VOLT3	9765
#define SCHSIO_CONV_VOLT4	10745
#define SCHSIO_CONV_VOLT5	3660

/* sch5627 specific */
#define SCHSIO_5627_CMD_READ	0x02
#define SCHSIO_5627_CMD_WRITE	0x03

#define SCHSIO_5627_HWMON_ID	0xa5
#define SCHSIO_5627_COMPANY_ID	0xa5
#define SCHSIO_5627_PRIMARY_ID	0xa5

#define SCHSIO_5627_REG_CTRL	0x40

struct schsio_softc;

struct schsio_sensor {
	char *desc;
	enum sensor_type type;
	u_int8_t bank;
	u_int8_t reg;
	void (*refresh)(struct schsio_softc *, int);
	int rfact;
};

struct schsio_softc {
	device_t sc_dev;

	struct resource *sc_iores;
	int sc_iorid;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;

	/* Used for triggering VBAT measurements */
	int sc_control;
	int sc_cnt;

	struct ksensor sensors[SCHSIO_NUM_SENSORS];
	struct ksensordev sensordev;
	u_int numsensors;
#if 0
	void (*refresh_sensor_data)(struct schsio_softc *);

	u_int8_t (*schsio_readreg)(struct schsio_softc *, int);
	void (*schsio_writereg)(struct schsio_softc *, int, int);
#endif
};

static int	schsio_probe(device_t);
static int	schsio_attach(device_t);
static int	schsio_detach(device_t);
static uint8_t	schsio_send_cmd(bus_space_tag_t iot, bus_space_handle_t ioh, uint8_t cmd, uint16_t reg, uint8_t val);
static uint8_t	schsio_readreg(struct schsio_softc *sc, uint16_t reg);
static void	schsio_writereg(struct schsio_softc *sc, uint16_t reg, uint32_t val);
static uint8_t	schsio_readconfig(bus_space_tag_t iot, bus_space_handle_t ioh, uint8_t reg);
static void	schsio_writeconfig(bus_space_tag_t iot, bus_space_handle_t ioh, uint8_t reg, uint8_t val);
static void	schsio_setup_volt(struct schsio_softc *sc, int start, int n);
static void	schsio_setup_temp(struct schsio_softc *sc, int start, int n);
static void	schsio_setup_fan(struct schsio_softc *sc, int start, int n);

static int	schsio_temp2muk(int temp);
static int	schsio_reg2rpm(unsigned long rpm);
static int	schsio_reg2muv(uint16_t v, uint16_t c);

static void	schsio_generic_stemp(struct schsio_softc *sc, struct ksensor *sensors);
static void	schsio_generic_svolt(struct schsio_softc *sc, struct ksensor *sensors);
static void	schsio_generic_fanrpm(struct schsio_softc *sc, struct ksensor *sensors);

static void	schsio_refresh_sensor_data(struct schsio_softc *sc);
static void	schsio_refresh(void *arg);

static device_method_t schsio_methods[] = {
	/* Methods from the device interface */
	DEVMETHOD(device_probe,         schsio_probe),
	DEVMETHOD(device_attach,        schsio_attach),
	DEVMETHOD(device_detach,        schsio_detach),

	/* Terminate method list */
	DEVMETHOD_END
};

static driver_t schsio_driver = {
	"schsio",
	schsio_methods,
	sizeof (struct schsio_softc)
};

static devclass_t schsio_devclass;

DRIVER_MODULE(schsio, isa, schsio_driver, schsio_devclass, NULL, NULL);

static int
schsio_probe(device_t dev)
{
	struct resource *iores;
	int iorid = 0;
	bus_space_tag_t iot;
	bus_space_handle_t ioh;
	uint8_t cr;

	iores = bus_alloc_resource(dev, SYS_RES_IOPORT, &iorid,
	    0ul, ~0ul, SCHSIO_IOSIZE, RF_ACTIVE);
	if (iores == NULL) {
		DPRINTF(("%s: can't map i/o space\n", __func__));
		return 1;
	}
	iot = rman_get_bustag(iores);
	ioh = rman_get_bushandle(iores);

	/* Check Vendor ID */
	bus_space_write_1(iot, ioh, SCHSIO_PORT_CONFIG, SCHSIO_CONFIG_ENTER);
	cr = schsio_readconfig(iot, ioh, SCHSIO_IDX_DEVICE);
	bus_space_write_1(iot, ioh, SCHSIO_PORT_CONFIG, SCHSIO_CONFIG_LEAVE);
	bus_release_resource(dev, SYS_RES_IOPORT, iorid, iores);
	DPRINTF(("schsio: vendor id 0x%x\n", cr));
	if (cr != SCHSIO_ID_SCH5627)
		return 1;

	return 0;
}

static int
schsio_attach(device_t dev)
{
	struct schsio_softc *sc = device_get_softc(dev);
	struct resource *conf_iores;
	bus_space_tag_t conf_iot;
	bus_space_handle_t conf_ioh;
	int conf_iorid = 0;
	int i;
	int val;
	uint16_t address;
//	uint8_t cr;
	uint8_t rev, chipid;
	uint8_t lsb, msb;

	sc->sc_dev = dev;
	conf_iores = bus_alloc_resource(dev, SYS_RES_IOPORT, &conf_iorid,
	    0ul, ~0ul, SCHSIO_IOSIZE, RF_ACTIVE);
	if (conf_iores == NULL) {
		device_printf(dev, "can't map i/o space\n");
		return 1;
	}
	conf_iot = rman_get_bustag(conf_iores);
	conf_ioh = rman_get_bushandle(conf_iores);

	/* Enter config mode */
	bus_space_write_1(conf_iot, conf_ioh, SCHSIO_PORT_CONFIG, SCHSIO_CONFIG_ENTER);

	rev = schsio_readconfig(conf_iot, conf_ioh, SCHSIO_IDX_REV);
	chipid = schsio_readconfig(conf_iot, conf_ioh, SCHSIO_IDX_DEVICE);

	schsio_writeconfig(conf_iot, conf_ioh, SCHSIO_IDX_LDEVSEL, SCHSIO_LDEV_RUNTIME);

	msb = schsio_readconfig(conf_iot, conf_ioh, SCHSIO_IDX_ADDR + 1);
	lsb = schsio_readconfig(conf_iot, conf_ioh, SCHSIO_IDX_ADDR);
	address = (msb << 8) | lsb;

	device_printf(dev, "ldev at address %04x\n", address);

	sc->sc_iorid = 1;
	sc->sc_iores = bus_alloc_resource(dev, SYS_RES_IOPORT, &sc->sc_iorid,
#if 0
	    address, ~0ul, SCHSIO_LDEV_RUNTIME_SZ, RF_ACTIVE | RF_SHAREABLE);
#endif
	    address, address + SCHSIO_LDEV_RUNTIME_SZ - 1, SCHSIO_LDEV_RUNTIME_SZ, RF_ACTIVE | RF_SHAREABLE);
	if (sc->sc_iores == NULL) {
		device_printf(dev, "can't map i/o space\n");
		bus_space_write_1(conf_iot, conf_ioh, SCHSIO_PORT_CONFIG, SCHSIO_CONFIG_LEAVE);
		bus_release_resource(dev, SYS_RES_IOPORT, conf_iorid, conf_iores);
		return 1;
	}
	sc->sc_iot = rman_get_bustag(sc->sc_iores);
	sc->sc_ioh = rman_get_bushandle(sc->sc_iores);

	sc->numsensors = SCHSIO_NUM_SENSORS;

	val = schsio_readreg(sc, SCHSIO_5627_REG_CTRL);

	if (!(val & 0x01))
		device_printf(dev, "hardware monitoring not enabled\n");

	sc->sc_control = val;
	sc->sc_cnt = 0;

	schsio_writereg(sc, SCHSIO_5627_REG_CTRL, val | 0x10);

	/* Exit config mode */
	bus_space_write_1(conf_iot, conf_ioh, SCHSIO_PORT_CONFIG, SCHSIO_CONFIG_LEAVE);
	bus_release_resource(dev, SYS_RES_IOPORT, conf_iorid, conf_iores);

	schsio_setup_fan(sc, 0, 4);
	schsio_setup_volt(sc, 4, 5);
	schsio_setup_temp(sc, 9, 8);

	sensor_task_register(sc, schsio_refresh, 5);

#if 0
	/* Activate monitoring */
	cr = it_readreg(sc, ITD_CONFIG);
	cr |= 0x01 | 0x08;
	it_writereg(sc, ITD_CONFIG, cr);
#endif

	/* Initialize sensors */
	strlcpy(sc->sensordev.xname, device_get_nameunit(sc->sc_dev),
	    sizeof(sc->sensordev.xname));
	for (i = 0; i < sc->numsensors; ++i)
		sensor_attach(&sc->sensordev, &sc->sensors[i]);
	sensordev_install(&sc->sensordev);

	return 0;
}

static int
schsio_detach(device_t dev)
{
	struct schsio_softc *sc = device_get_softc(dev);
	int error;

	sensordev_deinstall(&sc->sensordev);
	sensor_task_unregister(sc);

	error = bus_release_resource(dev, SYS_RES_IOPORT, sc->sc_iorid,
	    sc->sc_iores);
	if (error)
		return error;

	return 0;
}

static uint8_t
schsio_send_cmd(bus_space_tag_t iot, bus_space_handle_t ioh, uint8_t cmd,
    uint16_t reg, uint8_t val)
{
	uint8_t rv;
	int i;

	rv = bus_space_read_1(iot, ioh, 1);
	bus_space_write_1(iot, ioh, 1, val);

	bus_space_write_1(iot, ioh, 2, 0x00);
	bus_space_write_1(iot, ioh, 3, 0x80);

	bus_space_write_1(iot, ioh, 4, cmd);
	bus_space_write_1(iot, ioh, 5, 0x01);
	bus_space_write_1(iot, ioh, 2, 0x04);

	if (cmd == SCHSIO_5627_CMD_WRITE)
		bus_space_write_1(iot, ioh, 4, val);

	bus_space_write_1(iot, ioh, 6, reg & 0xff);
	bus_space_write_1(iot, ioh, 7, reg >> 8);

	bus_space_write_1(iot, ioh, 0, 0x01);

	/* wait for the interrupt-source register to become nonzero */
	/* not taking any chances for now */
	for (i = 0; i < 1024; i++) {
		rv = bus_space_read_1(iot, ioh, 8);

		/* clear interrupt-source register */
		if (rv)
			bus_space_write_1(iot, ioh, 8, rv);

		if (rv & 0x01)
			break;
	}

	if (i == 1024)
		kprintf("schsio: interrupt-source reg never became 0x01\n");

	/* read value from EC-to-Host Register */
	for (i = 0; i < 1024; i++) {
		rv = bus_space_read_1(iot, ioh, 1);

		if (rv == 0x01)
			break;
	}

	if (rv != 0x01)
		kprintf("schsio: command not completed\n");

	if (cmd == SCHSIO_5627_CMD_READ)
		return bus_space_read_1(iot, ioh, 4);

	return 0;
}

static uint8_t
schsio_readreg(struct schsio_softc *sc, uint16_t reg)
{
	return schsio_send_cmd(sc->sc_iot, sc->sc_ioh, SCHSIO_5627_CMD_READ,
	    reg, 0);
}

static void
schsio_writereg(struct schsio_softc *sc, uint16_t reg, uint32_t val)
{
	schsio_send_cmd(sc->sc_iot, sc->sc_ioh, SCHSIO_5627_CMD_WRITE,
	    reg, val);
}

static uint8_t
schsio_readconfig(bus_space_tag_t iot, bus_space_handle_t ioh, uint8_t reg)
{
	bus_space_write_1(iot, ioh, SCHSIO_PORT_INDEX, reg);
	return (bus_space_read_1(iot, ioh, SCHSIO_PORT_DATA));
}

static void
schsio_writeconfig(bus_space_tag_t iot, bus_space_handle_t ioh, uint8_t reg,
    uint8_t val)
{
	bus_space_write_1(iot, ioh, SCHSIO_PORT_INDEX, reg);
	bus_space_write_1(iot, ioh, SCHSIO_PORT_DATA, val);
}

void
schsio_setup_volt(struct schsio_softc *sc, int start, int n)
{
	int i;

	for (i = 0; i < n; ++i) {
		sc->sensors[start + i].type = SENSOR_VOLTS_DC;
	}

#if 0
	ksnprintf(sc->sensors[start + 0].desc, sizeof(sc->sensors[0].desc),
	    "VCORE_A");
	ksnprintf(sc->sensors[start + 1].desc, sizeof(sc->sensors[1].desc),
	    "VCORE_B");
	ksnprintf(sc->sensors[start + 2].desc, sizeof(sc->sensors[2].desc),
	    "+3.3V");
	ksnprintf(sc->sensors[start + 3].desc, sizeof(sc->sensors[3].desc),
	    "+5V");
	ksnprintf(sc->sensors[start + 4].desc, sizeof(sc->sensors[4].desc),
	    "+12V");
	ksnprintf(sc->sensors[start + 5].desc, sizeof(sc->sensors[5].desc),
	    "Unused");
	ksnprintf(sc->sensors[start + 6].desc, sizeof(sc->sensors[6].desc),
	    "-12V");
	ksnprintf(sc->sensors[start + 7].desc, sizeof(sc->sensors[7].desc),
	    "+5VSB");
	ksnprintf(sc->sensors[start + 8].desc, sizeof(sc->sensors[8].desc),
	    "VBAT");
#endif
}

void
schsio_setup_temp(struct schsio_softc *sc, int start, int n)
{
	int i;

	for (i = 0; i < n; ++i)
		sc->sensors[start + i].type = SENSOR_TEMP;
}

void
schsio_setup_fan(struct schsio_softc *sc, int start, int n)
{
	int i;

	for (i = 0; i < n; ++i)
		sc->sensors[start + i].type = SENSOR_FANRPM;
}

static inline int
schsio_temp2muk(int temp)
{
	return ((((temp * 625) / 10 - 64000) + 273150) * 1000);
}

static inline int
schsio_reg2rpm(unsigned long rpm)
{
	if (rpm == 0)
		return 0;

	return (int)((90000 * 60) / rpm);
}

static inline int
schsio_reg2muv(uint16_t v, uint16_t c)
{
	return (v * c + 5000) / 10000 * 1000;
}

static inline uint16_t
schsio_read_sensor(struct schsio_softc *sc, uint16_t lsbreg, uint16_t msbreg, int nibble) {
	uint8_t lsb, msb;

	if (nibble == -1) {
		lsb = schsio_readreg(sc, lsbreg);
		msb = schsio_readreg(sc, msbreg);
		return (msb << 8) | lsb;
	} else if (nibble == 0) {
		lsb = schsio_readreg(sc, lsbreg);
		msb = schsio_readreg(sc, msbreg);
		return (msb << 4) | (lsb & 0x0f);
	} else {
		lsb = schsio_readreg(sc, lsbreg);
		msb = schsio_readreg(sc, msbreg);
		return (msb << 4) | (lsb >> 4);
	}
}

static void
schsio_generic_stemp(struct schsio_softc *sc, struct ksensor *sensors)
{
	sensors[0].value = schsio_temp2muk(schsio_read_sensor(sc, SCHSIO_TEMP1_LSB, SCHSIO_TEMP1_MSB, 0));
	sensors[1].value = schsio_temp2muk(schsio_read_sensor(sc, SCHSIO_TEMP2_LSB, SCHSIO_TEMP2_MSB, 0));
	sensors[2].value = schsio_temp2muk(schsio_read_sensor(sc, SCHSIO_TEMP3_LSB, SCHSIO_TEMP3_MSB, 1));
	sensors[3].value = schsio_temp2muk(schsio_read_sensor(sc, SCHSIO_TEMP4_LSB, SCHSIO_TEMP4_MSB, 1));
	sensors[4].value = schsio_temp2muk(schsio_read_sensor(sc, SCHSIO_TEMP5_LSB, SCHSIO_TEMP5_MSB, 0));
	sensors[5].value = schsio_temp2muk(schsio_read_sensor(sc, SCHSIO_TEMP6_LSB, SCHSIO_TEMP6_MSB, 0));
	sensors[6].value = schsio_temp2muk(schsio_read_sensor(sc, SCHSIO_TEMP7_LSB, SCHSIO_TEMP7_MSB, 0));
	sensors[7].value = schsio_temp2muk(schsio_read_sensor(sc, SCHSIO_TEMP8_LSB, SCHSIO_TEMP8_MSB, 1));
#if 0
	int i, sdata;

	for (i = 0; i < 8; i++) {
		sdata = it_readreg(sc, ITD_SENSORTEMPBASE + i);
		/* Convert temperature to Fahrenheit degres */
		sensors[i].value = sdata * 1000000 + 273150000;
	}
#endif
}

static void
schsio_generic_svolt(struct schsio_softc *sc, struct ksensor *sensors)
{
	uint16_t reg;

	reg = schsio_read_sensor(sc, SCHSIO_VOLT1_LSB, SCHSIO_VOLT1_MSB, 1);
	sensors[0].value = schsio_reg2muv(reg, SCHSIO_CONV_VOLT1);
	reg = schsio_read_sensor(sc, SCHSIO_VOLT2_LSB, SCHSIO_VOLT2_MSB, 0);
	sensors[1].value = schsio_reg2muv(reg, SCHSIO_CONV_VOLT2);
	reg = schsio_read_sensor(sc, SCHSIO_VOLT3_LSB, SCHSIO_VOLT3_MSB, 1);
	sensors[2].value = schsio_reg2muv(reg, SCHSIO_CONV_VOLT3);
	reg = schsio_read_sensor(sc, SCHSIO_VOLT4_LSB, SCHSIO_VOLT4_MSB, 0);
	sensors[3].value = schsio_reg2muv(reg, SCHSIO_CONV_VOLT4);
	reg = schsio_read_sensor(sc, SCHSIO_VOLT5_LSB, SCHSIO_VOLT5_MSB, 1);
	sensors[4].value = schsio_reg2muv(reg, SCHSIO_CONV_VOLT5);

#if 0
	int i, sdata;

	for (i = 0; i < 5; i++) {
		sdata = it_readreg(sc, ITD_SENSORVOLTBASE + i);
		DPRINTF(("sdata[volt%d] 0x%x\n", i, sdata));
		/* voltage returned as (mV >> 4) */
		sensors[i].value = (sdata << 4);
		/* these two values are negative and formula is different */
		if (i == 5 || i == 6)
			sensors[i].value = ((sdata << 4) - IT_VREF);
		/* rfact is (factor * 10^4) */
		sensors[i].value *= it_vrfact[i];
		/* division by 10 gets us back to uVDC */
		sensors[i].value /= 10;
		if (i == 5 || i == 6)
			sensors[i].value += IT_VREF * 1000;
	}
#endif
}

static void
schsio_generic_fanrpm(struct schsio_softc *sc, struct ksensor *sensors)
{
	uint16_t reg;

	reg = schsio_read_sensor(sc, SCHSIO_FAN1_LSB, SCHSIO_FAN1_MSB, 0);
	sensors[0].flags &= ~SENSOR_FINVALID;
	if (reg == 0xffff)
		sensors[0].flags |= SENSOR_FINVALID;
	sensors[0].value = schsio_reg2rpm(reg);

	reg = schsio_read_sensor(sc, SCHSIO_FAN2_LSB, SCHSIO_FAN2_MSB, 0);
	sensors[1].flags &= ~SENSOR_FINVALID;
	if (reg == 0xffff)
		sensors[1].flags |= SENSOR_FINVALID;
	sensors[1].value = schsio_reg2rpm(reg);

	reg = schsio_read_sensor(sc, SCHSIO_FAN3_LSB, SCHSIO_FAN3_MSB, 0);
	sensors[2].flags &= ~SENSOR_FINVALID;
	if (reg == 0xffff)
		sensors[2].flags |= SENSOR_FINVALID;
	sensors[2].value = schsio_reg2rpm(reg);

	reg = schsio_read_sensor(sc, SCHSIO_FAN4_LSB, SCHSIO_FAN4_MSB, 0);
	sensors[3].flags &= ~SENSOR_FINVALID;
	if (reg == 0xffff)
		sensors[3].flags |= SENSOR_FINVALID;
	sensors[3].value = schsio_reg2rpm(reg);
#if 0
	int i, sdata, divisor, odivisor, ndivisor;

	odivisor = ndivisor = divisor = it_readreg(sc, ITD_FAN);
	for (i = 0; i < 4; i++) {
		sensors[i].flags &= ~SENSOR_FINVALID;
		if ((sdata = it_readreg(sc, ITD_SENSORFANBASE + i)) == 0xff) {
			sensors[i].flags |= SENSOR_FINVALID;
			if (i == 2)
				ndivisor ^= 0x40;
			else {
				ndivisor &= ~(7 << (i * 3));
				ndivisor |= ((divisor + 1) & 7) << (i * 3);
			}
		} else if (sdata == 0) {
			sensors[i].value = 0;
		} else {
			if (i == 2)
				divisor = divisor & 1 ? 3 : 1;
			sensors[i].value = 1350000 / (sdata << (divisor & 7));
		}
	}
	if (ndivisor != odivisor)
		it_writereg(sc, ITD_FAN, ndivisor);
#endif
}

/*
 * pre:  last read occurred >= 1.5 seconds ago
 * post: sensors[] current data are the latest from the chip
 */
static void
schsio_refresh_sensor_data(struct schsio_softc *sc)
{
	/* Trigger a VBAT measurement (Linux code suggests every 60 sec) */
	if (sc->sc_cnt % 1000 == 0)
		schsio_writereg(sc, SCHSIO_5627_REG_CTRL, sc->sc_control | 0x10);
	sc->sc_cnt++;

	/* Refresh our stored data for every sensor */
	schsio_generic_stemp(sc, &sc->sensors[9]);
	schsio_generic_svolt(sc, &sc->sensors[4]);
	schsio_generic_fanrpm(sc, &sc->sensors[0]);
}

static void
schsio_refresh(void *arg)
{
	struct schsio_softc *sc = (struct schsio_softc *)arg;

	schsio_refresh_sensor_data(sc);
}

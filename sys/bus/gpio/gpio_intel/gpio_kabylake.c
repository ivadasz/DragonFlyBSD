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
/*
 * Kabylake GPIO support.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/lock.h>
#include <sys/bus.h>

#include <sys/rman.h>

#include "opt_acpi.h"
#include "acpi.h"
#include <dev/acpica/acpivar.h>

#include "gpio_intel_var.h"

#include "gpio_if.h"

#define KBL_GPIO_REG_PAD_BASE	0x400

/* Community 0 Registers */
#define KBL_GPIO_OWNER_A0	0x20
#define KBL_GPIO_OWNER_A1	0x24
#define KBL_GPIO_OWNER_A2	0x28
#define KBL_GPIO_OWNER_B0	0x30
#define KBL_GPIO_OWNER_B1	0x34
#define KBL_GPIO_OWNER_B2	0x38
#define KBL_GPIO_CFGLOCK_A	0xa0
#define KBL_GPIO_CFGLOCK_B	0xa8
#define KBL_GPIO_SW_OWNER_A	0xd0
#define KBL_GPIO_SW_OWNER_B	0xd4
#define KBL_GPIO_REG_IS_A	0x100
#define KBL_GPIO_REG_IS_B	0x104
#define KBL_GPIO_REG_IE_A	0x120
#define KBL_GPIO_REG_IE_B	0x124

/* Community 1 Registers */
#define KBL_GPIO_OWNER_C0	0x20
#define KBL_GPIO_OWNER_C1	0x24
#define KBL_GPIO_OWNER_C2	0x28
#define KBL_GPIO_OWNER_D0	0x30
#define KBL_GPIO_OWNER_D1	0x34
#define KBL_GPIO_OWNER_D2	0x38
#define KBL_GPIO_OWNER_E0	0x40
#define KBL_GPIO_OWNER_E1	0x44
#define KBL_GPIO_OWNER_E2	0x48
#define KBL_GPIO_CFGLOCK_C	0xa0
#define KBL_GPIO_CFGLOCK_D	0xa8
#define KBL_GPIO_CFGLOCK_E	0xb0
#define KBL_GPIO_SW_OWNER_C	0xd0
#define KBL_GPIO_SW_OWNER_D	0xd4
#define KBL_GPIO_SW_OWNER_E	0xd8
#define KBL_GPIO_REG_IS_C	0x100
#define KBL_GPIO_REG_IS_D	0x104
#define KBL_GPIO_REG_IS_E	0x108
#define KBL_GPIO_REG_IE_C	0x120
#define KBL_GPIO_REG_IE_D	0x124
#define KBL_GPIO_REG_IE_E	0x128

/* Community 3 Registers */
#define KBL_GPIO_OWNER_F0	0x20
#define KBL_GPIO_OWNER_F1	0x24
#define KBL_GPIO_OWNER_F2	0x28
#define KBL_GPIO_OWNER_G0	0x30
#define KBL_GPIO_CFGLOCK_F	0xa0
#define KBL_GPIO_CFGLOCK_G	0xa8
#define KBL_GPIO_SW_OWNER_F	0xd0
#define KBL_GPIO_SW_OWNER_G	0xd4
#define KBL_GPIO_REG_IS_F	0x100
#define KBL_GPIO_REG_IS_G	0x104
#define KBL_GPIO_REG_IE_F	0x120
#define KBL_GPIO_REG_IE_G	0x124


#define CHV_GPIO_REG_PINS	0x4400	/* start of pin control registers */

#define CHV_GPIO_REGOFF_CTL0	0x0
#define CHV_GPIO_REGOFF_CTL1	0x4

#define CHV_GPIO_CTL0_RXSTATE	0x00000001u
#define CHV_GPIO_CTL0_TXSTATE	0x00000002u
#define CHV_GPIO_CTL0_GPIOCFG_MASK 0x00000700u
#define CHV_GPIO_CTL0_GPIOEN	0x00008000u
#define CHV_GPIO_CTL0_PULLUP	0x00800000u
#define CHV_GPIO_CTL1_INTCFG_MASK 0x00000007u
#define CHV_GPIO_CTL1_INVRXDATA	0x00000040u

#define CHV_GPIO_PINSIZE	0x8	/* 8 bytes for each pin */
#define CHV_GPIO_PINCHUNK	15	/* 15 pins at a time */
#define CHV_GPIO_PININC		0x400	/* every 0x400 bytes */

static void	gpio_kabylake_init(struct gpio_intel_softc *sc);
static void	gpio_kabylake_intr(void *arg);
static int	gpio_kabylake_map_intr(struct gpio_intel_softc *sc,
		    uint16_t pin, int trigger, int polarity, int termination);
static void	gpio_kabylake_unmap_intr(struct gpio_intel_softc *sc,
		    struct pin_intr_map *map);
static void	gpio_kabylake_enable_intr(struct gpio_intel_softc *sc,
		    struct pin_intr_map *map);
static void	gpio_kabylake_disable_intr(struct gpio_intel_softc *sc,
		    struct pin_intr_map *map);
static int	gpio_kabylake_check_io_pin(struct gpio_intel_softc *sc,
		    uint16_t pin, int flags);
static int	gpio_kabylake_read_pin(struct gpio_intel_softc *sc,
		    uint16_t pin);
static void	gpio_kabylake_write_pin(struct gpio_intel_softc *sc,
		    uint16_t pin, int value);

static struct gpio_intel_fns gpio_kabylake_fns = {
	.init = gpio_kabylake_init,
	.intr = gpio_kabylake_intr,
	.map_intr = gpio_kabylake_map_intr,
	.unmap_intr = gpio_kabylake_unmap_intr,
	.enable_intr = gpio_kabylake_enable_intr,
	.disable_intr = gpio_kabylake_disable_intr,
	.check_io_pin = gpio_kabylake_check_io_pin,
	.read_pin = gpio_kabylake_read_pin,
	.write_pin = gpio_kabylake_write_pin,
};

/* _UID=0 */
static struct pinrange kabylake_ranges[] = {
	{ 0, 151 },	/* Corresponds to Communities 0, 1, and 3 */
	{ -1, -1 }
};

static inline uint32_t
kblgpio_read(struct gpio_intel_softc *sc, bus_size_t offset, int comm)
{
	if (comm == 0)
		return bus_read_4(sc->mem_res, offset);
	else if (comm == 1)
		return bus_read_4(sc->mem_res1, offset);
	else if (comm == 3)
		return bus_read_4(sc->mem_res2, offset);
	else
		panic("Invalid PIN community, must be 0, 1 or 3");
}

static inline void
kblgpio_write(struct gpio_intel_softc *sc, bus_size_t offset, int comm,
    uint32_t val)
{
	if (comm == 0)
		bus_write_4(sc->mem_res, offset, val);
	else if (comm == 1)
		bus_write_4(sc->mem_res1, offset, val);
	else if (comm == 3)
		bus_write_4(sc->mem_res2, offset, val);
	else
		panic("Invalid PIN community, must be 0, 1 or 3");
}

int
gpio_kabylake_matchuid(struct gpio_intel_softc *sc)
{
	ACPI_HANDLE handle;

	handle = acpi_get_handle(sc->dev);
	if (acpi_MatchUid(handle, "0")) {
		sc->ranges = kabylake_ranges;
	} else {
		return (ENXIO);
	}

	sc->fns = &gpio_kabylake_fns;

	return (0);
}

static void
gpio_kabylake_init(struct gpio_intel_softc *sc)
{
	/* mask and clear all interrupt lines */
	kblgpio_write(sc, KBL_GPIO_REG_IE_A, 0, 0);
	kblgpio_write(sc, KBL_GPIO_REG_IE_B, 0, 0);
	kblgpio_write(sc, KBL_GPIO_REG_IS_A, 0, 0xffffff);
	kblgpio_write(sc, KBL_GPIO_REG_IS_B, 0, 0xffffff);
	kblgpio_write(sc, KBL_GPIO_REG_IE_C, 1, 0);
	kblgpio_write(sc, KBL_GPIO_REG_IE_D, 1, 0);
	kblgpio_write(sc, KBL_GPIO_REG_IE_E, 1, 0);
	kblgpio_write(sc, KBL_GPIO_REG_IS_C, 1, 0xffffff);
	kblgpio_write(sc, KBL_GPIO_REG_IS_D, 1, 0xffffff);
	kblgpio_write(sc, KBL_GPIO_REG_IS_E, 1, 0xffffff);
	kblgpio_write(sc, KBL_GPIO_REG_IE_F, 3, 0);
	kblgpio_write(sc, KBL_GPIO_REG_IE_G, 3, 0);
	kblgpio_write(sc, KBL_GPIO_REG_IS_F, 3, 0xffffff);
	kblgpio_write(sc, KBL_GPIO_REG_IS_G, 3, 0xff);
}

static void
gpio_kabylake_group_intr(struct gpio_intel_softc *sc, u_int startpin, int count,
    uint32_t reg, int comm)
{
	struct pin_intr_map *mapping;
	uint32_t status;
	int i;

	/*
	 * XXX Better strategy: Only read status register, when any of the pins
	 *     is actually allocated as a GPIO interrupt in the driver.
	 */
	KKASSERT(count == 8 || count == 24);
	status = kblgpio_read(sc, reg, comm);
	for (i = 0; i < count; i++) {
		if (status & (1U << i)) {
			mapping = &sc->intrmaps[i + startpin];
			if (!mapping->is_level) {
				kblgpio_write(sc, reg, comm, (1U << i));
			}
			if (mapping->pin != -1 && mapping->handler != NULL)
				mapping->handler(mapping->arg);
			if (mapping->is_level) {
				kblgpio_write(sc, reg, comm, (1U << i));
			}
		}
	}
}

static void
gpio_kabylake_intr(void *arg)
{
	struct gpio_intel_softc *sc = (struct gpio_intel_softc *)arg;

	gpio_kabylake_group_intr(sc, 0, 24, KBL_GPIO_REG_IS_A, 0);
	gpio_kabylake_group_intr(sc, 24, 24, KBL_GPIO_REG_IS_B, 0);
	gpio_kabylake_group_intr(sc, 48, 24, KBL_GPIO_REG_IS_C, 1);
	gpio_kabylake_group_intr(sc, 72, 24, KBL_GPIO_REG_IS_D, 1);
	gpio_kabylake_group_intr(sc, 96, 24, KBL_GPIO_REG_IS_E, 1);
	gpio_kabylake_group_intr(sc, 120, 24, KBL_GPIO_REG_IS_F, 3);
	gpio_kabylake_group_intr(sc, 144, 8, KBL_GPIO_REG_IS_G, 3);
}

static uint32_t
kblgpio_cfg_addr(uint16_t pin)
{
	if (pin < 48) {
		return KBL_GPIO_REG_PAD_BASE + pin * 8;
	} else if (pin < 120) {
		return KBL_GPIO_REG_PAD_BASE + (pin - 48) * 8;
	} else {
		return KBL_GPIO_REG_PAD_BASE + (pin - 120) * 8;
	}
}

/* Pin index to GPIO Community. */
static int
kblgpio_comm(uint16_t pin)
{
	if (pin < 48) {
		return 0;
	} else if (pin < 120) {
		return 1;
	} else {
		return 3;
	}
}

static uint8_t
kblgpio_pad_ownership(struct gpio_intel_softc *sc, uint16_t pin)
{
	uint32_t reg, val;

	if (pin < 24) {
		reg = KBL_GPIO_OWNER_A0 + 4 * (pin / 8);
	} else if (pin < 48) {
		pin -= 24;
		reg = KBL_GPIO_OWNER_B0 + 4 * (pin / 8);
	} else if (pin < 72) {
		pin -= 48;
		reg = KBL_GPIO_OWNER_C0 + 4 * (pin / 8);
	} else if (pin < 96) {
		pin -= 72;
		reg = KBL_GPIO_OWNER_D0 + 4 * (pin / 8);
	} else if (pin < 120) {
		pin -= 96;
		reg = KBL_GPIO_OWNER_E0 + 4 * (pin / 8);
	} else if (pin < 144) {
		pin -= 120;
		reg = KBL_GPIO_OWNER_F0 + 4 * (pin / 8);
	} else {
		reg = KBL_GPIO_OWNER_G0;
	}
	val = kblgpio_read(sc, reg, kblgpio_comm(pin));
	return (val >> ((pin % 8) * 4)) & 0x3;
}

static int
kblgpio_pad_config_lock(struct gpio_intel_softc *sc, uint16_t pin)
{
	uint32_t reg, val;

	if (pin < 24) {
		reg = KBL_GPIO_CFGLOCK_A;
	} else if (pin < 48) {
		reg = KBL_GPIO_CFGLOCK_B;
	} else if (pin < 72) {
		reg = KBL_GPIO_CFGLOCK_C;
	} else if (pin < 96) {
		reg = KBL_GPIO_CFGLOCK_D;
	} else if (pin < 120) {
		reg = KBL_GPIO_CFGLOCK_E;
	} else if (pin < 144) {
		reg = KBL_GPIO_CFGLOCK_F;
	} else {
		reg = KBL_GPIO_CFGLOCK_G;
	}
	val = kblgpio_read(sc, reg, kblgpio_comm(pin));
	return ((val >> (pin % 24)) & 1);
}

static void
kblgpio_pad_sw_own(struct gpio_intel_softc *sc, uint16_t pin, int own)
{
	uint32_t reg, val;

	if (pin < 24) {
		reg = KBL_GPIO_SW_OWNER_A;
	} else if (pin < 48) {
		reg = KBL_GPIO_SW_OWNER_B;
	} else if (pin < 72) {
		reg = KBL_GPIO_SW_OWNER_C;
	} else if (pin < 96) {
		reg = KBL_GPIO_SW_OWNER_D;
	} else if (pin < 120) {
		reg = KBL_GPIO_SW_OWNER_E;
	} else if (pin < 144) {
		reg = KBL_GPIO_SW_OWNER_F;
	} else {
		reg = KBL_GPIO_SW_OWNER_G;
	}
	val = kblgpio_read(sc, reg, kblgpio_comm(pin));
	if (own) {
		val |= (1 << (pin % 24));
		kblgpio_write(sc, reg, kblgpio_comm(pin), val);
	} else {
		val &= ~(1 << (pin % 24));
		kblgpio_write(sc, reg, kblgpio_comm(pin), val);
	}
}

static uint32_t
kblgpio_read_pinctl0(struct gpio_intel_softc *sc, uint16_t pin)
{
	return kblgpio_read(sc, kblgpio_cfg_addr(pin), kblgpio_comm(pin));
}

static uint32_t
kblgpio_read_pinctl1(struct gpio_intel_softc *sc, uint16_t pin)
{
	return kblgpio_read(sc, kblgpio_cfg_addr(pin) + 4, kblgpio_comm(pin));
}

static uint32_t
kblgpio_write_pinctl0(struct gpio_intel_softc *sc, uint16_t pin, uint32_t val)
{
	return kblgpio_write(sc, kblgpio_cfg_addr(pin), kblgpio_comm(pin), val);
}

/* XXX Add shared/exclusive argument. */
static int
gpio_kabylake_map_intr(struct gpio_intel_softc *sc, uint16_t pin, int trigger,
    int polarity, int termination)
{
	uint32_t reg, reg1, reg2;
	uint32_t new_gpiocfg;
	int i;

	reg1 = kblgpio_read_pinctl0(sc, pin);
	reg2 = kblgpio_read_pinctl1(sc, pin);
	device_printf(sc->dev,
	    "pin=%d trigger=%d polarity=%d ctrl0=0x%08x ctrl1=0x%08x\n",
	    pin, trigger, polarity, reg1, reg2);

	if (kblgpio_pad_ownership(sc, pin) != 0) {
		device_printf(sc->dev,
		    "Pin %u not owned by Host: Owner=0x%x\n",
		    pin, kblgpio_pad_ownership(sc, pin));
		return (ENXIO);
	}
	if (kblgpio_pad_config_lock(sc, pin) != 0) {
		device_printf(sc->dev, "Pin %u config locked\n", pin);
		return (ENXIO);
	}
	kblgpio_pad_sw_own(sc, pin, 1);

	if ((reg1 & 0xc00) != 0) {
		device_printf(sc->dev, "GPIO mode is disabled\n");
		return (ENXIO);
	}
	new_gpiocfg = reg1;
	if (reg1 & 0x200) {
		device_printf(sc->dev, "RX is disabled\n");
		new_gpiocfg &= ~0x200;
	}
	if (polarity == ACPI_ACTIVE_BOTH) {
		device_printf(sc->dev,
		    "ACTIVE_BOTH not supported\n");
		return (ENXIO);
	}
	if (trigger == ACPI_LEVEL_SENSITIVE) {
		if ((reg1 & 0x6000000) != 0) {
			device_printf(sc->dev,
			    "reg1 is 0x%x. Should be Level sensitive\n", reg1);
			return (ENXIO);
		}
		if (polarity == ACPI_ACTIVE_LOW) {
			if (!(reg1 & 0x800000)) {
				device_printf(sc->dev,
				    "Invert RX not enabled (needed for "
				    "level/low trigger/polarity)\n");
				return (ENXIO);
			}
		} else {
			if (reg2 & 0x800000) {
				device_printf(sc->dev,
				    "Invert RX should not be enabled for "
				    "level/high trigger/polarity\n");
				return (ENXIO);
			}
		}
	} else {
		/*
		 * For edge-triggered interrupts it's definitely harmless to
		 * change between rising-edge, falling-edge and both-edges
		 * triggering.
		 */
		if (polarity == ACPI_ACTIVE_HIGH && (reg1 & 0x6000000) != 0) {
			device_printf(sc->dev,
			    "Wrong interrupt configuration, is 0x%x should "
			    "be 0x%x\n", reg1, reg1 & ~0x6000000);
			if ((reg1 & 0x6000000) == 0x2000000 ||
			    (reg1 & 0x6000000) == 0x4000000) {
				new_gpiocfg &= ~0x6000000;
			} else {
				return (ENXIO);
			}
		} else if (polarity == ACPI_ACTIVE_LOW &&
		    (reg1 & 0x6000000) != 0x2000000) {
			device_printf(sc->dev,
			    "Wrong interrupt configuration, is 0x%x should "
			    "be 0x%x\n", reg1, (reg1 & ~0x6000000) | 0x2000000);
			if ((reg1 & 0x6000000) == 0 ||
			    (reg1 & 0x6000000) == 0x4000000) {
				new_gpiocfg &= ~0x6000000;
				new_gpiocfg |= 0x2000000;
			} else {
				return (ENXIO);
			}
		}
	}
	if (termination == ACPI_PIN_CONFIG_PULLUP &&
	    (reg2 & 0x3c00) != 0x2400 && (reg2 & 0x3c00) != 0x3000) {
		device_printf(sc->dev,
		    "Wrong termination, is 0x%x, should be pull-up\n",
		    (reg2 & 0x3c00) >> 10);
		return (ENXIO);
	} else if (termination == ACPI_PIN_CONFIG_PULLDOWN &&
	    (reg2 & 0x3c00) != 0x800 && (reg2 & 0x3c00) != 0x1000) {
		device_printf(sc->dev,
		    "Wrong termination, is 0x%x should be pull-down\n",
		    (reg2 & 0x3c00) >> 10);
		return (ENXIO);
	}
	if (new_gpiocfg != reg1) {
		device_printf(sc->dev,
		    "Switching reg1 gpio configuration from 0x%x to 0x%x\n",
		    reg1, new_gpiocfg);
		kblgpio_write_pinctl0(sc, pin, new_gpiocfg);
	}
	/* Interrupts are directly mapped here. */
	sc->intrmaps[pin].pin = pin;
	sc->intrmaps[pin].intidx = pin;
	sc->intrmaps[pin].orig_intcfg = reg2;
	sc->intrmaps[pin].orig_gpiocfg = reg1;

	if (trigger == ACPI_LEVEL_SENSITIVE)
		sc->intrmaps[pin].is_level = 1;
	else
		sc->intrmaps[pin].is_level = 0;

	return (0);
}

static void
gpio_kabylake_unmap_intr(struct gpio_intel_softc *sc,
    struct pin_intr_map *map)
{
	uint32_t reg, gpiocfg;
	uint16_t pin = map->pin;

	gpiocfg = map->orig_gpiocfg;

	map->pin = -1;
	map->intidx = -1;
	map->is_level = 0;
	map->orig_intcfg = 0;
	map->orig_gpiocfg = 0;

	/* Restore gpio configuration if needed */
	reg = kblgpio_read_pinctl0(sc, pin);
	if ((reg & ~0x3) != (gpiocfg & ~0x3)) {
		reg = (reg & 0x3) | (gpiocfg & ~0x3);
		kblgpio_write_pinctl0(sc, pin, reg);
	}

	kblgpio_pad_sw_own(sc, pin, 0);
}

static void
gpio_kabylake_enable_intr(struct gpio_intel_softc *sc,
    struct pin_intr_map *map)
{
	uint32_t reg;

	KKASSERT(map->intidx >= 0);

XXXXXXXXXXXX
	/* clear interrupt status flag */
	kblgpio_write(sc, CHV_GPIO_REG_IS, (1U << map->intidx));

	/* unmask interrupt */
	reg = kblgpio_read(sc, CHV_GPIO_REG_MASK);
	reg |= (1U << map->intidx);
	kblgpio_write(sc, CHV_GPIO_REG_MASK, reg);
}

static void
gpio_kabylake_disable_intr(struct gpio_intel_softc *sc,
    struct pin_intr_map *map)
{
	uint32_t reg;

	KKASSERT(map->intidx >= 0);

	/* mask interrupt line */
	reg = kblgpio_read(sc, CHV_GPIO_REG_MASK);
	reg &= ~(1U << map->intidx);
	kblgpio_write(sc, CHV_GPIO_REG_MASK, reg);
}

static int
gpio_kabylake_check_io_pin(struct gpio_intel_softc *sc, uint16_t pin,
    int flags)
{
	uint32_t reg1, reg2;

	reg1 = kblgpio_read(sc, PIN_CTL0(pin));
	if (flags & (1U << 0)) {
		/* Verify that RX is enabled */
		if ((reg1 & CHV_GPIO_CTL0_GPIOCFG_MASK) != 0 &&
		    (reg1 & CHV_GPIO_CTL0_GPIOCFG_MASK) != 0x200) {
			return (0);
		}
	}
	reg2 = kblgpio_read(sc, PIN_CTL1(pin));
	if (flags & (1U << 1)) {
		/* Verify that interrupt is disabled */
		if ((reg2 & CHV_GPIO_CTL1_INTCFG_MASK) != 0)
			return (0);
		/* Verify that TX is enabled */
		if ((reg1 & CHV_GPIO_CTL0_GPIOCFG_MASK) != 0 &&
		    (reg1 & CHV_GPIO_CTL0_GPIOCFG_MASK) != 0x100) {
			return (0);
		}
	}

	return (1);
}

static int
gpio_kabylake_read_pin(struct gpio_intel_softc *sc, uint16_t pin)
{
	uint32_t reg;
	int val;

	reg = kblgpio_read(sc, PIN_CTL0(pin));
	/* Verify that RX is enabled */
	KKASSERT((reg & CHV_GPIO_CTL0_GPIOCFG_MASK) == 0x0 ||
	    (reg & CHV_GPIO_CTL0_GPIOCFG_MASK) == 0x200);

	if (reg & CHV_GPIO_CTL0_RXSTATE)
		val = 1;
	else
		val = 0;

	return (val);
}

static void
gpio_kabylake_write_pin(struct gpio_intel_softc *sc, uint16_t pin, int value)
{
	uint32_t reg;

	reg = kblgpio_read(sc, PIN_CTL0(pin));
	/* Verify that TX is enabled */
	KKASSERT((reg & CHV_GPIO_CTL0_GPIOCFG_MASK) == 0 ||
	    (reg & CHV_GPIO_CTL0_GPIOCFG_MASK) == 0x100);

	if (value)
		reg |= CHV_GPIO_CTL0_TXSTATE;
	else
		reg &= ~CHV_GPIO_CTL0_TXSTATE;
	kblgpio_write(sc, PIN_CTL0(pin), reg);
}

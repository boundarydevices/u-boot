/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <i2c.h>
#include "bd_common.h"

#define MAX77823_CHG_CNFG_00	0xB7
#define MAX77823_CHG_CNFG_02	0xB9
#define MAX77823_CHG_CNFG_06	0xBD
#define MAX77823_CHG_CNFG_09	0xC0
#define MAX77823_CHG_CNFG_11	0xC2
#define MAX77823_CHG_CNFG_12	0xC3

#define CHG_CNFG_00_CHG_MASK	BIT(0)
#define CHG_CNFG_00_OTG_MASK	BIT(1)
#define CHG_CNFG_00_BUCK_MASK	BIT(2)
#define CHG_CNFG_00_BOOST_MASK	BIT(3)

void max77823_init(void)
{
	int ret;
	u8 orig_i2c_bus;
	u8 val8;
	u8 buf[2];

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_MAX77823);
#define I2C_ADDR_CHARGER	0x69
	val8 = 0x7f;	/* 4.0A source */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_09, 1, &val8, 1);
	val8 = 0x0c;	/* Protection allow 0xb9 write */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_06, 1, &val8, 1);
	val8 = 0x14;	/* 1A charge */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_02, 1, &val8, 1);
	val8 = 0x27;	/* enable charging from otg */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_12, 1, &val8, 1);
	val8 = 0x5;	/* enable charging mode */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, 1, &val8, 1);

#define I2C_ADDR_FUELGAUGE	0x36
#define MAX77823_REG_VCELL	0x09
	ret = i2c_read(I2C_ADDR_FUELGAUGE, MAX77823_REG_VCELL, 1, buf, 2);
	if (!ret) {
		u32 v = (buf[1] << 8) | buf[0];

		v = (v >> 3) * 625;
		printf("battery voltage = %d uV\n", v);
		if (v < 3000000) {
			printf("voltage = %d uV too low, powering off\n", v);
			board_poweroff();
		}
	} else {
		printf("error reading battery voltage\n");
	}
	i2c_set_bus_num(orig_i2c_bus);
}

static int max77823_update_reg(int i2c_addr, int reg, int val, int mask)
{
	unsigned char buf[4];
	int ret;

	ret = i2c_read(i2c_addr, reg, 1, buf, 1);
	if (!ret) {
		buf[0] &= ~mask;
		buf[0] |= val;
		i2c_write(i2c_addr, reg, 1, buf, 1);
		ret = buf[0];
	}
	return ret;
}

static int max77823_write_reg(int i2c_addr, int reg, int val)
{
	unsigned char buf[4];
	int ret;

	buf[0] = val;
	ret = i2c_write(i2c_addr, reg, 1, buf, 1);
	return ret;
}

static void max77823_otg_enable(void)
{
	/* Disable charging from CHRG_IN when we are supplying power */
	max77823_update_reg(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_12,
			0, 0x20);

	/* Update CHG_CNFG_11 to 0x54(5.1V) */
	max77823_write_reg(I2C_ADDR_CHARGER,
		MAX77823_CHG_CNFG_11, 0x54);

	/* OTG on, boost on */
	max77823_update_reg(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		CHG_CNFG_00_OTG_MASK | CHG_CNFG_00_BOOST_MASK,
		CHG_CNFG_00_OTG_MASK | CHG_CNFG_00_BOOST_MASK);
}

static void max77823_otg_disable(void)
{
	/* chrg on, OTG off, boost on/off, (buck on) */
	max77823_update_reg(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		CHG_CNFG_00_CHG_MASK | CHG_CNFG_00_BUCK_MASK,
		CHG_CNFG_00_CHG_MASK | CHG_CNFG_00_BUCK_MASK |
			CHG_CNFG_00_OTG_MASK | CHG_CNFG_00_BOOST_MASK);

	mdelay(50);

	/* Allow charging from CHRG_IN when we are not supplying power */
	max77823_update_reg(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_12,
			0x20, 0x20);
}

void max77823_otg_power(int enable)
{
	u8 orig_i2c_bus;

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_MAX77823);
	if (enable)
		max77823_otg_enable();
	else
		max77823_otg_disable();

	i2c_set_bus_num(orig_i2c_bus);
}

static void max77823_boost_enable(void)
{
	/* Update CHG_CNFG_11 to 0x54(5.1V) */
	max77823_write_reg(I2C_ADDR_CHARGER,
		MAX77823_CHG_CNFG_11, 0x54);

	/* charger on, otg off, buck on, boost on */
	max77823_update_reg(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		CHG_CNFG_00_BOOST_MASK, CHG_CNFG_00_BOOST_MASK);
}

static void max77823_boost_disable(void)
{
	/* chrg on, OTG off, buck on, boost off */
	max77823_update_reg(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		0, CHG_CNFG_00_BOOST_MASK);
}

void max77823_boost_power(int enable)
{
	u8 orig_i2c_bus;

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_MAX77823);
	if (enable)
		max77823_boost_enable();
	else
		max77823_boost_disable();

	i2c_set_bus_num(orig_i2c_bus);
}

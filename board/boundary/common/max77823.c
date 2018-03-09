/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/crm_regs.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <i2c.h>
#include "bd_common.h"

#define MAX77823_CHG_CNFG_00	0xB7
#define MAX77823_CHG_CNFG_02	0xB9
#define MAX77823_CHG_CNFG_06	0xBD
#define MAX77823_CHG_CNFG_09	0xC0
#define MAX77823_CHG_CNFG_10	0xC1
#define MAX77823_CHG_CNFG_11	0xC2
#define MAX77823_CHG_CNFG_12	0xC3

#define CHG_CNFG_00_CHG_MASK	BIT(0)
#define CHG_CNFG_00_OTG_MASK	BIT(1)
#define CHG_CNFG_00_BUCK_MASK	BIT(2)
#define CHG_CNFG_00_BOOST_MASK	BIT(3)

#ifdef CONFIG_OTG_CHARGER
void set_max_chrgin_current(int i2c_addr)
{
	int ret;
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 val;
	u8 chgin;
	u8 buf[2];

	/* turn on comparator, change threshold to 4.6V*/
	writel(BIT(20) | 6, &mxc_ccm->usb1_vbus_detect_set);
	/* reset default */
	writel(7 << 18, &mxc_ccm->usb1_chrg_detect_clr);
	ret = i2c_read(i2c_addr, MAX77823_CHG_CNFG_09, 1, buf, 1);
	if (ret)
		return;
	chgin = buf[0];
	/* Increase chgin until vbus is invalid */
	while (1) {
		val = readl(&mxc_ccm->usb1_vbus_det_stat);
		if (!(val & 0x0e)) {
			if (chgin == 3)
				return;
			chgin = 0x3;	/* 100 mA source */
			i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
			udelay(1000);
			continue;
		}
		if (!(val & 8) || (chgin > 0x78))
			break;
		chgin++;
		i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
	}
	/* Decrease chgin until vbus is valid */
	while (1) {
		val = readl(&mxc_ccm->usb1_vbus_det_stat);
		if (!(val & 0x0e)) {
			chgin = 0x3;	/* 100 mA source */
			i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
			return;
		}
		if ((val & 8) || (chgin <= 3))
			break;
		chgin--;
		i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
		udelay(100);
	}
	chgin--;
	if (chgin >= 3)
		i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
}
#endif

void max77823_init(void)
{
	int ret;
	u8 orig_i2c_bus;
	u8 val8;
	u8 buf[2];

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_MAX77823);
#define I2C_ADDR_CHARGER	0x69
#ifndef CONFIG_OTG_CHARGER
	val8 = 0x78;	/* 4.0A source */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_09, 1, &val8, 1);
#endif
	val8 = 0x26;	/* .76 A source */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_10, 1, &val8, 1);
	val8 = 0x0c;	/* Protection allow 0xb9 write */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_06, 1, &val8, 1);
	val8 = 0x2a;	/* 2.1A charge */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_02, 1, &val8, 1);
	/*
	 * enable charging from chgin(otg)/wcin,
	 * VCHGIN_REG - bits[4:3]
	 * 0 - Vchgin_reg = 4.5V, Vchgin_uvlo=4.3V
	 * 1 - Vchgin_reg = 4.9V, Vchgin_uvlo=4.7V
	 * 2 - Vchgin_reg = 5.0V, Vchgin_uvlo=4.8V
	 * 3 - Vchgin_reg = 5.1V, Vchgin_uvlo=4.9V
	 */
	val8 = 0x67 | (0 << 3);
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
#ifdef CONFIG_OTG_CHARGER
			val8 = 0x1e;	/* 1.0A source */
			i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_09, 1, &val8, 1);
#endif
			board_poweroff();
		}
	} else {
		printf("error reading battery voltage\n");
	}
#ifdef CONFIG_OTG_CHARGER
	set_max_chrgin_current(I2C_ADDR_CHARGER);
#endif
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

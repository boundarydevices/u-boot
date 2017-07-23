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
#include <init.h>
#include <linux/delay.h>
#include "bd_common.h"

#define I2C_ADDR_FUELGAUGE	0x36
#define MAX77823_REG_VCELL	0x09

#define I2C_ADDR_CHARGER	0x69

#define MAX77823_CHG_DETAILS_00	0xB3
#define MAX77823_CHG_DETAILS_01	0xB4
#define MAX77823_CHG_CNFG_00	0xB7
#define MAX77823_CHG_CNFG_01	0xB8
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

static int max77823_read_reg(int i2c_addr, int reg)
{
	unsigned char buf[4];
	int ret;

	ret = i2c_read(i2c_addr, reg, 1, buf, 1);
	if (!ret)
		ret = buf[0];
	return ret;
}

#ifdef CONFIG_OTG_CHARGER

#define ANADIG_USB1_CHRG_DETECT_CHK_CONTACT	BIT(18)
#define ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B	BIT(19)
#define ANADIG_USB1_CHRG_DETECT_EN_B		BIT(20)

#define ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT	BIT(0)
#define ANADIG_USB1_CHRG_DET_STAT_CHRG_DETECTED	BIT(1)

static int set_max_chrgin_current(int i2c_addr)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 val;
	u8 chgin = 0x3;
	int i = 0;
	int ilim_max = 3;

	/* turn on comparator, change threshold to 4.6V*/
	writel(BIT(20) | 6, &mxc_ccm->usb1_vbus_detect_set);
	writel(1, &mxc_ccm->usb1_vbus_detect_clr);

	/* Enable charger detect, contact detect */
	writel(ANADIG_USB1_CHRG_DETECT_EN_B, &mxc_ccm->usb1_chrg_detect_clr);
	writel(ANADIG_USB1_CHRG_DETECT_CHK_CONTACT |
		ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
		&mxc_ccm->usb1_chrg_detect_set);

	i2c_read(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
	chgin &= 0x7f;

	/* determine type of cable */
	/* Check if plug is connected */
	while (1) {
		val = readl(&mxc_ccm->usb1_chrg_det_stat);
		if (val & ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT) {
			break;
		} else {
			i++;
			if (i >= 10) {
				ilim_max = 0x78;
				break;
			}
			val = readl(&mxc_ccm->usb1_vbus_det_stat);
			if (!(val & 0x0e)) {
				if (chgin == 0xf)
					return chgin;
				chgin = 0x0f;
				i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
			}
			udelay(5000);
		}
	}
	writel(ANADIG_USB1_CHRG_DETECT_CHK_CONTACT |
		ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
		&mxc_ccm->usb1_chrg_detect_clr);

	if (val & ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT) {
		udelay(100000);
		val = readl(&mxc_ccm->usb1_chrg_det_stat);
		if (val & ANADIG_USB1_CHRG_DET_STAT_CHRG_DETECTED) {
			ilim_max = 0x78;	/* Charger */
		} else {
			ilim_max = 0xf;		/* Standard downstream port */
		}

	}
	/* Disable charger detect */
	writel(ANADIG_USB1_CHRG_DETECT_EN_B |
		ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
		&mxc_ccm->usb1_chrg_detect_set);

	if (chgin > ilim_max) {
		chgin = ilim_max;
		i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
		udelay(5000);
	}
	/* Increase chgin until vbus is invalid */
	while (chgin < ilim_max) {
		val = readl(&mxc_ccm->usb1_vbus_det_stat);
		if (!(val & 0x0e)) {
			if (chgin == 0xf)
				return chgin;
			chgin = 0xf;	/* 500 mA source */
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
			chgin = 0xf;	/* 500 mA source */
			i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
			return chgin;
		}
		if ((val & 8) || (chgin <= 3))
			break;
		chgin--;
		i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
		udelay(100);
	}
	if (chgin != ilim_max) {
		if (chgin > 3) {
			chgin--;
			i2c_write(i2c_addr, MAX77823_CHG_CNFG_09, 1, &chgin, 1);
		}
	}
	return chgin;
}
#endif

static void power_check(void)
{
	int ret;
	u8 val8;
	u8 buf[2];
	u8 boost = max77823_read_reg(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00) &
			CHG_CNFG_00_BOOST_MASK;


#ifdef CONFIG_OTG_CHARGER
	int chgin = set_max_chrgin_current(I2C_ADDR_CHARGER);
	if (chgin >= 0x5a) {	/* 3.0 amps */
		val8 = 0x5 | boost;	/* enable charging mode */
		i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, 1, &val8, 1);
		return;
	}
#else
	ret = i2c_read(I2C_ADDR_CHARGER, MAX77823_CHG_DETAILS_00, 1, &val8, 1);
	if (!ret) {
		/* check for VBUS is valid */
		if (((val8 >> 5) & 0x3) == 3) {
			val8 = 0x5 | boost;	/* enable charging mode */
			i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, 1, &val8, 1);
			return;
		}
	}
#endif
	/* chgin cannot supply enough, check battery */
	val8 = 0x4 | boost;	/* disable charging mode */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, 1, &val8, 1);
	udelay(5000);	/* 5 ms to let voltage stabilize */

	ret = i2c_read(I2C_ADDR_FUELGAUGE, MAX77823_REG_VCELL, 1, buf, 2);
	val8 = 0x5 | boost;	/* enable charging mode */
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, 1, &val8, 1);

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
}

/*
 * Output:
 *  0 - done charging,
 *  1 - charging,
 *  -1 : can't charge
 */
int max77823_is_charging(void)
{
	int ret;
	u8 buf[2];
	u8 orig_i2c_bus;
	u8 chg_dtls;

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_MAX77823);

	ret = i2c_read(I2C_ADDR_CHARGER, MAX77823_CHG_DETAILS_01, 1, buf, 2);
	if (!ret) {
		chg_dtls = buf[0] & 0xf;
		if ((chg_dtls >= 1) && (chg_dtls <= 3))
			ret = 1;
		else if (chg_dtls == 4)
			ret = 0;
		else
			ret = -1;
	} else {
		ret = -1;
	}
	i2c_set_bus_num(orig_i2c_bus);
	return ret;
}

void max77823_init(void)
{
	u8 orig_i2c_bus;
	u8 val8;

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_MAX77823);
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
	 * fast charge timer disable
	 * Switching frequency 2 MHz
	 * restart threshold 100mV below CHG_CV_PRM
	 */
	val8 = (0 << 0) | (0x01 << 3) | (0x0 << 4);
	i2c_write(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_01, 1, &val8, 1);
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
	power_check();
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
	/* chrg on, OTG off, boost unchanged, (buck on) */
	max77823_update_reg(I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		CHG_CNFG_00_CHG_MASK | CHG_CNFG_00_BUCK_MASK,
		CHG_CNFG_00_CHG_MASK | CHG_CNFG_00_BUCK_MASK |
			CHG_CNFG_00_OTG_MASK);

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

void max77834_power_check(void)
{
	u8 orig_i2c_bus;

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_MAX77823);
	power_check();
	i2c_set_bus_num(orig_i2c_bus);
}

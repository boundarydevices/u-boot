/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <config.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <i2c.h>
#include <init.h>
#include <log.h>
#include <linux/delay.h>
#include <linux/err.h>
#include "bd_common.h"

#define I2C_ADDR_FUELGAUGE	0x36
#define MAX17260_REG_VCELL	0x09

#define I2C_ADDR_CHARGER	0x6b
#define MAX77975_CHG_DETAILS_00	0x13
#define MAX77975_CHG_DETAILS_01	0x14
#define MAX77975_CHG_CNFG_00	0x16
#define MAX77975_CHG_CNFG_01	0x17
#define MAX77975_CHG_CNFG_02	0x18
#define MAX77975_CHG_CNFG_04	0x1a
#define MAX77975_CHG_CNFG_06	0x1c
#define MAX77975_CHG_CNFG_08	0x1e
#define MAX77975_CHG_CNFG_09	0x1f
#define MAX77975_CHG_CNFG_10	0x20
#define MAX77975_CHG_CNFG_11	0x21
#define MAX77975_CHG_CNFG_12	0x22

#define CHG_CNFG_00_CHG_MASK	BIT(0)
#define CHG_CNFG_00_OTG_MASK	BIT(1)
#define CHG_CNFG_00_BUCK_MASK	BIT(2)
#define CHG_CNFG_00_BOOST_MASK	BIT(3)

#define CNFG09_100ma	0x01
#define CNFG09_500ma	0x09
#define CNFG09_3000ma	0x3b
#define CNFG09_MASK	0x3f
#define CNFG09_MAX	0x3f

#if CONFIG_IS_ENABLED(DM_I2C)
#define BUS		bus
#define BUS_		bus,
#define BUS_PARM	struct udevice *bus
#define BUS_PARM_	struct udevice *bus,
#define BUS_T		struct udevice *
#define bus_error(a)	IS_ERR(a)

static int _i2c_write_mux(struct udevice *bus, u8 i2c_addr, u8 val)
{
	struct dm_i2c_ops *ops;
	unsigned char txbuf[4];
	struct i2c_msg msg;
	int ret;

	ops = i2c_get_ops(bus);
	msg.addr = i2c_addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = txbuf;
	txbuf[0] = val;
	ret = ops->xfer(bus, &msg, 1);
	if (ret < 0)
		printf("%s: 0x%x ret=%d\n", __func__, val, ret);
	else
		ret = 0;
	return ret;
}

static BUS_T get_bus(int bus_num)
{
	struct udevice *bus;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, bus_num & 0x0f, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return ERR_PTR(-ENODEV);
	}
	bus_num >>= 4;
	if (bus_num)
		_i2c_write_mux(bus, 0x70, bus_num);
	return bus;
}

static void restore_bus(BUS_T bus)
{
}

static int _i2c_read_reg(struct udevice *bus, u8 i2c_addr, u8 reg)
{
	struct dm_i2c_ops *ops;
	unsigned char txbuf[4];
	unsigned char rxbuf[4];
	struct i2c_msg msg[2];
	int ret;

	ops = i2c_get_ops(bus);
	msg[0].addr = i2c_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = txbuf;
	msg[1].addr = i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = rxbuf;
	txbuf[0] = reg;
	rxbuf[0] = 0;
	ret = ops->xfer(bus, msg, 2);
	if (ret < 0)
		printf("%s: 0x%x ? 0x%x ret=%d\n", __func__, reg,
			rxbuf[0], ret);
	else
		ret = rxbuf[0];
	return ret;
}

static int _i2c_read_u16(struct udevice *bus, u8 i2c_addr, u8 reg)
{
	struct dm_i2c_ops *ops;
	unsigned char txbuf[4];
	unsigned char rxbuf[4];
	struct i2c_msg msg[2];
	int ret;

	ops = i2c_get_ops(bus);
	msg[0].addr = i2c_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = txbuf;
	msg[1].addr = i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = rxbuf;
	txbuf[0] = reg;
	rxbuf[0] = 0;
	rxbuf[1] = 0;
	ret = ops->xfer(bus, msg, 2);
	if (ret < 0)
		printf("%s: 0x%x ? 0x%x%02x ret=%d\n", __func__, reg,
			rxbuf[1], rxbuf[0], ret);
	else
		ret = (rxbuf[1] << 8) | rxbuf[0];
	return ret;
}

static int _i2c_write_reg(struct udevice *bus, u8 i2c_addr, u8 reg, u8 val)
{
	struct dm_i2c_ops *ops;
	unsigned char txbuf[4];
	struct i2c_msg msg;
	int ret;

	ops = i2c_get_ops(bus);
	msg.addr = i2c_addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = txbuf;
	txbuf[0] = reg;
	txbuf[1] = val;
	ret = ops->xfer(bus, &msg, 1);
	if (ret < 0)
		printf("%s: 0x%x=0x%x ret=%d\n", __func__, reg,
			val, ret);
	else
		ret = 0;
	return ret;
}

#define i2c_read_reg(bus, i2c_addr, reg) _i2c_read_reg(bus, i2c_addr, reg)
#define i2c_read_u16(bus, i2c_addr, reg) _i2c_read_u16(bus, i2c_addr, reg)
#define i2c_write_reg(bus, i2c_addr, reg, val) _i2c_write_reg(bus, i2c_addr, reg, val)
#else
#define BUS
#define BUS_
#define BUS_PARM	void
#define BUS_PARM_
#define BUS_T		int
#define bus_error(a)	(a < 0)

static BUS_T get_bus(int bus_num)
{
	u8 orig_i2c_bus;

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(bus_num);
	return orig_i2c_bus;
}

static void restore_bus(BUS_T bus)
{
	i2c_set_bus_num(bus);
}

static int _i2c_read_reg(u8 i2c_addr, u8 reg)
{
	unsigned char buf[4];
	int ret;

	ret = i2c_read(i2c_addr, reg, 1, buf, 1);
	if (!ret)
		ret = buf[0];
	return ret;
}

static int _i2c_read_u16(u8 i2c_addr, u8 reg)
{
	unsigned char buf[4];
	int ret;

	ret = i2c_read(i2c_addr, reg, 1, buf, 2);
	if (!ret)
		ret = (buf[1] << 8) | buf[0];
	return ret;
}

static int _i2c_write_reg(u8 i2c_addr, u8 reg, u8 val)
{
	return i2c_write(i2c_addr, reg, 1, &val, 1);
}

#define i2c_read_reg(bus, i2c_addr, reg) _i2c_read_reg(i2c_addr, reg)
#define i2c_read_u16(bus, i2c_addr, reg) _i2c_read_u16(i2c_addr, reg)
#define i2c_write_reg(bus, i2c_addr, reg, val) _i2c_write_reg(i2c_addr, reg, val)
#endif

static int bus_i2c_read_u16(int bus_num, int i2c_addr, int reg)
{
	int ret = -ENODEV;
	BUS_T bus = get_bus(bus_num);

	if (!bus_error(bus)) {
		ret = i2c_read_u16(bus, i2c_addr, reg);
		restore_bus(bus);
	}
	return ret;
}

static void power_check(BUS_PARM)
{
	int ret;
	u8 val8;

	ret = i2c_read_reg(bus, I2C_ADDR_CHARGER, MAX77975_CHG_DETAILS_00);
	if (ret >= 0) {
		val8 = ret;
		/* check for VBUS is valid */
		if (((val8 >> 5) & 0x3) == 3) {
			val8 = 0x5;	/* enable charging mode */
			i2c_write_reg(bus, I2C_ADDR_CHARGER, MAX77975_CHG_CNFG_00, val8);
			return;
		}
	}

	/* chgin cannot supply enough, check battery */
	val8 = 0x4;	/* disable charging mode */
	i2c_write_reg(bus, I2C_ADDR_CHARGER, MAX77975_CHG_CNFG_00, val8);
	udelay(5000);	/* 5 ms to let voltage stabilize */

#ifdef CONFIG_MAX17260
	ret = bus_i2c_read_u16(CONFIG_I2C_BUS_MAX17260, I2C_ADDR_FUELGAUGE, MAX17260_REG_VCELL);
#endif
	val8 = 0x5;	/* enable charging mode */
	i2c_write_reg(bus, I2C_ADDR_CHARGER, MAX77975_CHG_CNFG_00, val8);

#ifdef CONFIG_MAX17260
	if (ret >= 0) {
		u32 v = ret;

		v = (v * 625) >> 3;
		printf("battery voltage = %d uV\n", v);
		if (v < 3000000) {
			printf("voltage = %d uV too low, powering off\n", v);
			board_poweroff();
		}
	} else {
		printf("error reading battery voltage\n");
	}
#endif
}

/*
 * Output:
 *  0 - done charging,
 *  1 - charging,
 *  -1 : can't charge
 */
int max77975_is_charging(void)
{
	int ret;
	BUS_T bus = get_bus(CONFIG_I2C_BUS_MAX77975);
	u8 chg_dtls;

	if (!bus_error(bus)) {
		ret = i2c_read_reg(bus, I2C_ADDR_CHARGER, MAX77975_CHG_DETAILS_01);
		if (ret >= 0) {
			chg_dtls = ret & 0xf;
			if ((chg_dtls >= 1) && (chg_dtls <= 3))
				ret = 1;
			else if (chg_dtls == 4)
				ret = 0;
			else
				ret = -1;
		} else {
			ret = -1;
		}
		restore_bus(bus);
	}
	return ret;
}

struct reg_val {
	unsigned char reg;
	unsigned char val;
};

static const struct reg_val charger_init_data[] = {
	{MAX77975_CHG_CNFG_00, 0x04,},	/* charger=off, otg=off, buck=on, boost=off */
	{MAX77975_CHG_CNFG_11, 0x00,},	/* Vbyp=5V */
	{MAX77975_CHG_CNFG_06, 0x0c,},	/* Write capability unlocked */
	{MAX77975_CHG_CNFG_02, 0x14,},	/* 1.0 amp fast-charge */
	{MAX77975_CHG_CNFG_04, 0x05,},	/* Charge termination 4.20V */
	{MAX77975_CHG_CNFG_08, 0x00,},	/* 2.6M FSW */
	{MAX77975_CHG_CNFG_10, 0x03,},  /* OTG output 800ma limit */
	{MAX77975_CHG_CNFG_01, 0x00,},  /* fast charge timer disable, restart threshold 100mV below CHG_CV_PRM */
	/*
	 * VCHGIN_REG - bits[5:4]
	 * 0 - Vchgin_reg = 4.5V, Vchgin_uvlo=4.7V
	 * 1 - Vchgin_reg = 4.6V, Vchgin_uvlo=4.8V
	 * 2 - Vchgin_reg = 4.7V, Vchgin_uvlo=4.9V
	 * 3 - Vchgin_reg = 4.85V, Vchgin_uvlo=5.05V
	 */
	{MAX77975_CHG_CNFG_12, 0 << 4,},
};

void charger_cmds(BUS_PARM_ const struct reg_val *p, int cnt)
{
	int i = 0;

	while (i < cnt) {
		i2c_write_reg(bus, I2C_ADDR_CHARGER, p->reg, p->val);
		p++;
		i++;
	}
}

void max77975_init(void)
{
	BUS_T bus = get_bus(CONFIG_I2C_BUS_MAX77975);

	if (!bus_error(bus)) {
		charger_cmds(BUS_ charger_init_data, ARRAY_SIZE(charger_init_data));
		power_check(BUS);
		restore_bus(bus);
	}
}

static void max77975_otg_enable(BUS_PARM)
{
	/* Update CHG_CNFG_11 to Vbyp = 5.1V */
	i2c_write_reg(bus, I2C_ADDR_CHARGER,
		MAX77975_CHG_CNFG_11, 0x1);

	/* OTG on, boost on */
	i2c_write_reg(bus, I2C_ADDR_CHARGER, MAX77975_CHG_CNFG_00,
		CHG_CNFG_00_OTG_MASK | CHG_CNFG_00_BOOST_MASK);
}

static void max77975_otg_disable(BUS_PARM)
{
	/* chrg on, OTG off, boost unchanged, (buck on) */
	i2c_write_reg(bus, I2C_ADDR_CHARGER, MAX77975_CHG_CNFG_00,
		CHG_CNFG_00_CHG_MASK | CHG_CNFG_00_BUCK_MASK);
}

void max77975_otg_power(int enable)
{
	BUS_T bus = get_bus(CONFIG_I2C_BUS_MAX77975);

	if (!bus_error(bus)) {
		if (enable)
			max77975_otg_enable(BUS);
		else
			max77975_otg_disable(BUS);
		restore_bus(bus);
	}
}

void max77975_power_check(void)
{
	BUS_T bus = get_bus(CONFIG_I2C_BUS_MAX77975);

	if (!bus_error(bus)) {
		power_check(BUS);
		restore_bus(bus);
	}
}

int max77975_set_chrgin_limit(int ma)
{
	u32 val;
	BUS_T bus;

	if ((ma > 3200) || (ma < 100))
		return -EINVAL;
	bus = get_bus(CONFIG_I2C_BUS_MAX77975);

	if (!bus_error(bus)) {
		val = (ma / 50) - 1;
		i2c_write_reg(bus, I2C_ADDR_CHARGER,
			MAX77975_CHG_CNFG_09, val);
		restore_bus(bus);
	}
	return 0;
}

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

#ifdef CONFIG_DM_I2C
#define BUS		bus
#define BUS_		bus,
#define BUS_PARM	struct udevice *bus
#define BUS_PARM_	struct udevice *bus,
#define BUS_T		struct udevice *

static BUS_T get_bus(void)
{
	struct udevice *bus;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, CONFIG_I2C_BUS_MAX77823, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return NULL;
	}
	return bus;
}

static void restore_bus(BUS_T bus)
{
}

static int _max77823_read_reg(struct udevice *bus, u8 i2c_addr, u8 reg)
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

static int _max77823_read_u16(struct udevice *bus, u8 i2c_addr, u8 reg)
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

static int _max77823_write_reg(struct udevice *bus, u8 i2c_addr, u8 reg, u8 val)
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

#define max77823_read_reg(bus, i2c_addr, reg) _max77823_read_reg(bus, i2c_addr, reg)
#define max77823_read_u16(bus, i2c_addr, reg) _max77823_read_u16(bus, i2c_addr, reg)
#define max77823_write_reg(bus, i2c_addr, reg, val) _max77823_write_reg(bus, i2c_addr, reg, val)
#else
#define BUS
#define BUS_
#define BUS_PARM	void
#define BUS_PARM_
#define BUS_T		int

static BUS_T get_bus(void)
{
	u8 orig_i2c_bus;

	orig_i2c_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_I2C_BUS_MAX77823);
	return orig_i2c_bus;
}

static void restore_bus(BUS_T bus)
{
	i2c_set_bus_num(bus);
}

static int _max77823_read_reg(u8 i2c_addr, u8 reg)
{
	unsigned char buf[4];
	int ret;

	ret = i2c_read(i2c_addr, reg, 1, buf, 1);
	if (!ret)
		ret = buf[0];
	return ret;
}

static int _max77823_read_u16(u8 i2c_addr, u8 reg)
{
	unsigned char buf[4];
	int ret;

	ret = i2c_read(i2c_addr, reg, 1, buf, 2);
	if (!ret)
		ret = (buf[1] << 8) | buf[0];
	return ret;
}

static int _max77823_write_reg(u8 i2c_addr, u8 reg, u8 val)
{
	return i2c_write(i2c_addr, reg, 1, &val, 1);
}

#define max77823_read_reg(bus, i2c_addr, reg) _max77823_read_reg(i2c_addr, reg)
#define max77823_read_u16(bus, i2c_addr, reg) _max77823_read_u16(i2c_addr, reg)
#define max77823_write_reg(bus, i2c_addr, reg, val) _max77823_write_reg(i2c_addr, reg, val)
#endif



static int max77823_update_reg(BUS_PARM_ u8 i2c_addr, u8 reg, u8 val, u8 mask)
{
	int ret;

	ret = max77823_read_reg(bus, i2c_addr, reg);
	if (ret >= 0) {
		ret = (ret & ~mask) | val;
		max77823_write_reg(bus, i2c_addr, reg, ret);
	}
	return ret;
}

#ifdef CONFIG_OTG_CHARGER

#if defined(CONFIG_IMX8M)

#define MX6_BM_NON_BURST_SETTING	BIT(1)
#define MX6_BM_OVER_CUR_DIS		BIT(7)

#define MX7D_USBNC_USB_CTRL2		0x4

#define MX7D_USBNC_AUTO_RESUME			BIT(2)
#define MX7D_USB_VBUS_WAKEUP_SOURCE_MASK	0x3
#define MX7D_USB_VBUS_WAKEUP_SOURCE(v)		(v << 0)
#define MX7D_USB_VBUS_WAKEUP_SOURCE_VBUS	MX7D_USB_VBUS_WAKEUP_SOURCE(0)
#define MX7D_USB_VBUS_WAKEUP_SOURCE_AVALID	MX7D_USB_VBUS_WAKEUP_SOURCE(1)
#define MX7D_USB_VBUS_WAKEUP_SOURCE_BVALID	MX7D_USB_VBUS_WAKEUP_SOURCE(2)
#define MX7D_USB_VBUS_WAKEUP_SOURCE_SESS_END	MX7D_USB_VBUS_WAKEUP_SOURCE(3)

#define MX7D_USB_OTG_PHY_CFG1		0x30
#define TXPREEMPAMPTUNE0_BIT		28
#define TXPREEMPAMPTUNE0_MASK		(3 << 28)
#define TXVREFTUNE0_BIT			20
#define TXVREFTUNE0_MASK		(0xf << 20)

#define MX7D_USB_OTG_PHY_CFG2_DRVVBUS0		BIT(16)
#define MX7D_USB_OTG_PHY_CFG2_OTGDISABLE0	BIT(10)
#define MX7D_USB_OTG_PHY_CFG2_CHRG_DCDENB	BIT(3)
#define MX7D_USB_OTG_PHY_CFG2_CHRG_VDATSRCENB0	BIT(2)
#define MX7D_USB_OTG_PHY_CFG2_CHRG_VDATDETENB0	BIT(1)
#define MX7D_USB_OTG_PHY_CFG2_CHRG_CHRGSEL	BIT(0)
#define MX7D_USB_OTG_PHY_CFG2		0x34

#define MX7D_USB_OTG_PHY_STATUS		0x3c
#define MX7D_USB_OTG_PHY_STATUS_CHRGDET		BIT(29)
#define MX7D_USB_OTG_PHY_STATUS_VBUS_VLD	BIT(3)
#define MX7D_USB_OTG_PHY_STATUS_LINE_STATE1	BIT(1)
#define MX7D_USB_OTG_PHY_STATUS_LINE_STATE0	BIT(0)

static int imx7d_charger_data_contact_detect(void)
{
	void *base = (void *)0x32e40200;
	u32 val;
	int i, data_pin_contact_count = 0;

	/* Enable Data Contact Detect (DCD) per the USB BC 1.2 */
	val = readl(base + MX7D_USB_OTG_PHY_CFG2);
	writel(val | MX7D_USB_OTG_PHY_CFG2_CHRG_DCDENB,
		base + MX7D_USB_OTG_PHY_CFG2);

	for (i = 0; i < 100; i = i + 1) {
		val = readl(base + MX7D_USB_OTG_PHY_STATUS);
		if (!(val & MX7D_USB_OTG_PHY_STATUS_LINE_STATE0)) {
			if (data_pin_contact_count++ > 5)
				/* Data pin makes contact */
				break;
			udelay(5000);
		} else {
			data_pin_contact_count = 0;
			udelay(5000);
		}
	}

	/* Disable DCD after finished data contact check */
	val = readl(base + MX7D_USB_OTG_PHY_CFG2);
	writel(val & ~MX7D_USB_OTG_PHY_CFG2_CHRG_DCDENB,
			base + MX7D_USB_OTG_PHY_CFG2);

	if (i == 100) {
		printf("VBUS is coming from a dedicated power supply.\n");
		return -ENXIO;
	}

	return 0;
}

static int usbmisc_imx7d_vbus_comparator_on(bool on)
{
	void *base = (void *)0x32e40200;
	u32 val;

	val = readl(base + MX7D_USB_OTG_PHY_CFG2);
	val &= ~(MX7D_USB_OTG_PHY_CFG2_DRVVBUS0 | MX7D_USB_OTG_PHY_CFG2_OTGDISABLE0);
	if (on)
		val |= MX7D_USB_OTG_PHY_CFG2_DRVVBUS0;

	writel(val, base + MX7D_USB_OTG_PHY_CFG2);
	return 0;
}

static int usbmisc_imx7d_init(void)
{
	void *base = (void *)0x32e40200;
	u32 reg;

	reg = readl(base);
	reg |= MX6_BM_OVER_CUR_DIS;
	writel(reg, base);

	/* SoC non-burst setting */
	reg = readl(base);
	writel(reg | MX6_BM_NON_BURST_SETTING, base);

	reg = readl(base + MX7D_USBNC_USB_CTRL2);
	reg &= ~MX7D_USB_VBUS_WAKEUP_SOURCE_MASK;
	writel(reg | MX7D_USB_VBUS_WAKEUP_SOURCE_BVALID
		| MX7D_USBNC_AUTO_RESUME,
		base + MX7D_USBNC_USB_CTRL2);
	/* PHY tuning for signal quality */
	reg = readl(base + MX7D_USB_OTG_PHY_CFG1);
	reg &= ~TXPREEMPAMPTUNE0_MASK;
	reg |= (3 << TXPREEMPAMPTUNE0_BIT);

	reg &= ~TXVREFTUNE0_MASK;
	reg |= (7 << TXVREFTUNE0_BIT);

	writel(reg, base + MX7D_USB_OTG_PHY_CFG1);
	return 0;
}

static int init_usbphy(BUS_PARM_ u8 i2c_addr, u8 *pchgin)
{
	int ret;
	u8 chgin = 0x3;
	u8 ilim_max = 3;

	enable_usboh3_clk(1);
	imx8m_usb_power(0, true);
	usbmisc_imx7d_init();
	usbmisc_imx7d_vbus_comparator_on(1);
	ret = imx7d_charger_data_contact_detect();

	if (ret < 0) {
		ilim_max = 0x78;	/* Charger */
	} else {
		ilim_max = 0xf;		/* Standard downstream port */
	}

	ret = max77823_read_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09);
	if (ret >= 0)
		chgin = ret;
	chgin &= 0x7f;
	*pchgin = chgin;
	return ilim_max;
}

static void finish_usbphy(void)
{
	usbmisc_imx7d_vbus_comparator_on(0);
}

static u32 get_vbus_stat(void)
{
	void *base = (void *)0x32e40200;
	u32 val;

	val = readl(base + MX7D_USB_OTG_PHY_STATUS);
	debug("%s: val=%x\n", __func__, val);
	return val & 0x0c;
}
#else
#define ANADIG_USB1_CHRG_DETECT_CHK_CONTACT	BIT(18)
#define ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B	BIT(19)
#define ANADIG_USB1_CHRG_DETECT_EN_B		BIT(20)

#define ANADIG_USB1_CHRG_DET_STAT_PLUG_CONTACT	BIT(0)
#define ANADIG_USB1_CHRG_DET_STAT_CHRG_DETECTED	BIT(1)

static int init_usbphy(BUS_PARM_ u8 i2c_addr, u8 *pchgin)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u8 chgin = 0x3;
	int ilim_max = 3;
	u32 val;
	int ret;
	int i;

	/* turn on comparator, change threshold to 4.6V*/
	writel(BIT(20) | 6, &mxc_ccm->usb1_vbus_detect_set);
	writel(1, &mxc_ccm->usb1_vbus_detect_clr);

	/* Enable charger detect, contact detect */
	writel(ANADIG_USB1_CHRG_DETECT_EN_B, &mxc_ccm->usb1_chrg_detect_clr);
	writel(ANADIG_USB1_CHRG_DETECT_CHK_CONTACT |
		ANADIG_USB1_CHRG_DETECT_CHK_CHRG_B,
		&mxc_ccm->usb1_chrg_detect_set);

	ret = max77823_read_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09);
	if (ret >= 0)
		chgin = ret;
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
				max77823_write_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09, chgin);
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
	*pchgin = chgin;
	return ilim_max;
}

static void finish_usbphy(void)
{
}

static u32 get_vbus_stat(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	return readl(&mxc_ccm->usb1_vbus_det_stat) & 0x0e;
}
#endif

static int set_max_chrgin_current(BUS_PARM_ u8 i2c_addr)
{
	u32 val;
	u8 chgin;
	int ilim_max;

	ilim_max = init_usbphy(BUS_ i2c_addr, &chgin);


	if (chgin > ilim_max) {
		chgin = ilim_max;
		max77823_write_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09, chgin);
		udelay(5000);
	}
	/* Increase chgin until vbus is invalid */
	while (chgin < ilim_max) {
		val = get_vbus_stat();
		if (!val) {
			if (chgin == 0xf)
				return chgin;
			chgin = 0xf;	/* 500 mA source */
			max77823_write_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09, chgin);
			udelay(1000);
			continue;
		}
		if (!(val & 8) || (chgin > 0x78))
			break;
		chgin++;
		max77823_write_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09, chgin);
	}
	/* Decrease chgin until vbus is valid */
	while (1) {
		val = get_vbus_stat();
		if (!val) {
			chgin = 0xf;	/* 500 mA source */
			max77823_write_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09, chgin);
			return chgin;
		}
		if ((val & 8) || (chgin <= 3))
			break;
		chgin--;
		max77823_write_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09, chgin);
		udelay(100);
	}
	if (chgin != ilim_max) {
		if (chgin > 3) {
			chgin--;
			max77823_write_reg(bus, i2c_addr, MAX77823_CHG_CNFG_09, chgin);
		}
	}
	finish_usbphy();
	return chgin;
}
#endif

static void power_check(BUS_PARM)
{
	int ret;
	u8 val8;
	u8 boost = max77823_read_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00) &
			CHG_CNFG_00_BOOST_MASK;


#ifdef CONFIG_OTG_CHARGER
	int chgin = set_max_chrgin_current(BUS_ I2C_ADDR_CHARGER);
	if (chgin >= 0x5a) {	/* 3.0 amps */
		val8 = 0x5 | boost;	/* enable charging mode */
		max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, val8);
		return;
	}
#else
	ret = max77823_read_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_DETAILS_00);
	if (ret >= 0) {
		val8 = ret;
		/* check for VBUS is valid */
		if (((val8 >> 5) & 0x3) == 3) {
			val8 = 0x5 | boost;	/* enable charging mode */
			max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, val8);
			return;
		}
	}
#endif
	/* chgin cannot supply enough, check battery */
	val8 = 0x4 | boost;	/* disable charging mode */
	max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, val8);
	udelay(5000);	/* 5 ms to let voltage stabilize */

	ret = max77823_read_u16(bus, I2C_ADDR_FUELGAUGE, MAX77823_REG_VCELL);
	val8 = 0x5 | boost;	/* enable charging mode */
	max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00, val8);

	if (ret >= 0) {
		u32 v = ret;

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
	BUS_T bus = get_bus();
	u8 chg_dtls;

	ret = max77823_read_u16(bus, I2C_ADDR_CHARGER, MAX77823_CHG_DETAILS_01);
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
	return ret;
}

void max77823_init(void)
{
	BUS_T bus = get_bus();
	u8 val8;

#ifndef CONFIG_OTG_CHARGER
	val8 = 0x78;	/* 4.0A source */
	max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_09, val8);
#endif
	val8 = 0x26;	/* .76 A source */
	max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_10, val8);
	val8 = 0x0c;	/* Protection allow 0xb9 write */
	max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_06, val8);
	val8 = 0x2a;	/* 2.1A charge */
	max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_02, val8);

	/*
	 * fast charge timer disable
	 * Switching frequency 2 MHz
	 * restart threshold 100mV below CHG_CV_PRM
	 */
	val8 = (0 << 0) | (0x01 << 3) | (0x0 << 4);
	max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_01, val8);
	/*
	 * enable charging from chgin(otg)/wcin,
	 * VCHGIN_REG - bits[4:3]
	 * 0 - Vchgin_reg = 4.5V, Vchgin_uvlo=4.3V
	 * 1 - Vchgin_reg = 4.9V, Vchgin_uvlo=4.7V
	 * 2 - Vchgin_reg = 5.0V, Vchgin_uvlo=4.8V
	 * 3 - Vchgin_reg = 5.1V, Vchgin_uvlo=4.9V
	 */
	val8 = 0x67 | (0 << 3);
	max77823_write_reg(bus, I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_12, val8);
	power_check(BUS);
	restore_bus(bus);
}

static void max77823_otg_enable(BUS_PARM)
{
	/* Disable charging from CHRG_IN when we are supplying power */
	max77823_update_reg(BUS_ I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_12,
			0, 0x20);

	/* Update CHG_CNFG_11 to 0x54(5.1V) */
	max77823_write_reg(bus, I2C_ADDR_CHARGER,
		MAX77823_CHG_CNFG_11, 0x54);

	/* OTG on, boost on */
	max77823_update_reg(BUS_ I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		CHG_CNFG_00_OTG_MASK | CHG_CNFG_00_BOOST_MASK,
		CHG_CNFG_00_OTG_MASK | CHG_CNFG_00_BOOST_MASK);
}

static void max77823_otg_disable(BUS_PARM)
{
	/* chrg on, OTG off, boost unchanged, (buck on) */
	max77823_update_reg(BUS_ I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		CHG_CNFG_00_CHG_MASK | CHG_CNFG_00_BUCK_MASK,
		CHG_CNFG_00_CHG_MASK | CHG_CNFG_00_BUCK_MASK |
			CHG_CNFG_00_OTG_MASK);

	mdelay(50);

	/* Allow charging from CHRG_IN when we are not supplying power */
	max77823_update_reg(BUS_ I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_12,
			0x20, 0x20);
}

void max77823_otg_power(int enable)
{
	BUS_T bus = get_bus();

	if (enable)
		max77823_otg_enable(BUS);
	else
		max77823_otg_disable(BUS);

	restore_bus(bus);
}

static void max77823_boost_enable(BUS_PARM)
{
	/* Update CHG_CNFG_11 to 0x54(5.1V) */
	max77823_write_reg(bus, I2C_ADDR_CHARGER,
		MAX77823_CHG_CNFG_11, 0x54);

	/* charger on, otg off, buck on, boost on */
	max77823_update_reg(BUS_ I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		CHG_CNFG_00_BOOST_MASK, CHG_CNFG_00_BOOST_MASK);
}

static void max77823_boost_disable(BUS_PARM)
{
	/* chrg on, OTG off, buck on, boost off */
	max77823_update_reg(BUS_ I2C_ADDR_CHARGER, MAX77823_CHG_CNFG_00,
		0, CHG_CNFG_00_BOOST_MASK);
}

void max77823_boost_power(int enable)
{
	BUS_T bus = get_bus();

	if (enable)
		max77823_boost_enable(BUS);
	else
		max77823_boost_disable(BUS);

	restore_bus(bus);
}

void max77834_power_check(void)
{
	BUS_T bus = get_bus();

	power_check(BUS);

	restore_bus(bus);
}

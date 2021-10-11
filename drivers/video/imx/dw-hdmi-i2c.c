// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DesignWare High-Definition Multimedia Interface (HDMI) driver
 *
 * Copyright (C) 2021 Boundary Devices Inc.
 * Copyright (C) 2013-2015 Mentor Graphics Inc.
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2010, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 */

#include <common.h>
#include <log.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/sys_proto.h>
#include <asm/io.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/pinctrl.h>
#include <fdtdec.h>
#include <i2c.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <regmap.h>
#include <watchdog.h>
#include "dw-hdmi.h"

#define DDC_CI_ADDR		0x37
#define DDC_SEGMENT_ADDR	0x30

struct dw_hdmi_i2c {
	struct udevice *bus;
	struct regmap *regm;
	/* Use gpio to force bus idle when bus state is abnormal */
	struct gpio_desc scl_gpio;
	struct gpio_desc sda_gpio;
	unsigned int reg_shift;
	u32 speed;
	u8 stat;
	struct clk *bulk_clks;

	u8 slave_reg;
	bool is_regaddr;
	bool is_segment;
};

static inline void hdmi_writeb(struct dw_hdmi_i2c *i2c, u8 val, int offset)
{
	regmap_raw_write(i2c->regm, offset << i2c->reg_shift, &val, REGMAP_SIZE_8);
}

static inline u8 hdmi_readb(struct dw_hdmi_i2c *i2c, int offset)
{
	unsigned int val = 0;

	regmap_raw_read(i2c->regm, offset << i2c->reg_shift, &val, REGMAP_SIZE_8);

	return val;
}

static void dw_hdmi_i2c_init(struct dw_hdmi_i2c *i2c)
{
	hdmi_writeb(i2c, HDMI_PHY_I2CM_INT_ADDR_DONE_POL,
		    HDMI_PHY_I2CM_INT_ADDR);

	hdmi_writeb(i2c, HDMI_PHY_I2CM_CTLINT_ADDR_NAC_POL |
		    HDMI_PHY_I2CM_CTLINT_ADDR_ARBITRATION_POL,
		    HDMI_PHY_I2CM_CTLINT_ADDR);

	/* Software reset */
	hdmi_writeb(i2c, 0x00, HDMI_I2CM_SOFTRSTZ);

	/* Set Standard Mode speed (determined to be 100KHz on iMX6) */
	hdmi_writeb(i2c, 0x00, HDMI_I2CM_DIV);

	/* Set done, not acknowledged and arbitration interrupt polarities */
	hdmi_writeb(i2c, HDMI_I2CM_INT_DONE_POL, HDMI_I2CM_INT);
	hdmi_writeb(i2c, HDMI_I2CM_CTLINT_NAC_POL | HDMI_I2CM_CTLINT_ARB_POL,
		    HDMI_I2CM_CTLINT);

	/* Clear DONE and ERROR interrupts */
	hdmi_writeb(i2c, HDMI_IH_I2CM_STAT0_ERROR | HDMI_IH_I2CM_STAT0_DONE,
		    HDMI_IH_I2CM_STAT0);

	/* Mute DONE and ERROR interrupts */
	hdmi_writeb(i2c, HDMI_IH_I2CM_STAT0_ERROR | HDMI_IH_I2CM_STAT0_DONE,
		    HDMI_IH_MUTE_I2CM_STAT0);
}

static bool dw_hdmi_i2c_unwedge(struct dw_hdmi_i2c *i2c)
{
	/* If no unwedge state then give up */
	debug("Attempting to unwedge stuck i2c bus\n");

	/*
	 * This is a huge hack to workaround a problem where the dw_hdmi i2c
	 * bus could sometimes get wedged.  Once wedged there doesn't appear
	 * to be any way to unwedge it (including the HDMI_I2CM_SOFTRSTZ)
	 * other than pulsing the SDA line.
	 *
	 * We appear to be able to pulse the SDA line (in the eyes of dw_hdmi)
	 * by:
	 * 1. Remux the pin as a GPIO output, driven low.
	 * 2. Wait a little while.  1 ms seems to work, but we'll do 10.
	 * 3. Immediately jump to remux the pin as dw_hdmi i2c again.
	 *
	 * At the moment of remuxing, the line will still be low due to its
	 * recent stint as an output, but then it will be pulled high by the
	 * (presumed) external pullup.  dw_hdmi seems to see this as a rising
	 * edge and that seems to get it out of its jam.
	 *
	 * This wedging was only ever seen on one TV, and only on one of
	 * its HDMI ports.  It happened when the TV was powered on while the
	 * device was plugged in.  A scope trace shows the TV bringing both SDA
	 * and SCL low, then bringing them both back up at roughly the same
	 * time.  Presumably this confuses dw_hdmi because it saw activity but
	 * no real STOP (maybe it thinks there's another master on the bus?).
	 * Giving it a clean rising edge of SDA while SCL is already high
	 * presumably makes dw_hdmi see a STOP which seems to bring dw_hdmi out
	 * of its stupor.
	 *
	 * Note that after coming back alive, transfers seem to immediately
	 * resume, so if we unwedge due to a timeout we should wait a little
	 * longer for our transfer to finish, since it might have just started
	 * now.
	 */
	pinctrl_select_state(i2c->bus, "unwedge");
	mdelay(10);
	pinctrl_select_state(i2c->bus, "default");

	return true;
}

static int dw_hdmi_i2c_poll(struct dw_hdmi_i2c *i2c)
{
	unsigned int stat;

	stat = hdmi_readb(i2c, HDMI_IH_I2CM_STAT0);
	if (!stat)
		return 0;

	hdmi_writeb(i2c, stat, HDMI_IH_I2CM_STAT0);
	i2c->stat = stat;
	return 1;
}

static int i2c_wait(struct dw_hdmi_i2c *i2c, int jiffies)
{
	ulong start_time = get_timer(0);
	ulong elapsed;

	do {
		elapsed = get_timer(start_time);
		if (dw_hdmi_i2c_poll(i2c))
			break;
		if (elapsed > jiffies)
			return -ETIMEDOUT;
		udelay(100);
	} while (1);
	return 0;
}

static int dw_hdmi_i2c_wait(struct dw_hdmi_i2c *i2c)
{
	int stat;

	stat = i2c_wait(i2c, CONFIG_SYS_HZ / 10);
	if (stat < 0) {
		/* If we can't unwedge, return timeout */
		if (!dw_hdmi_i2c_unwedge(i2c))
			return -EAGAIN;

		/* We tried to unwedge; give it another chance */
		stat = i2c_wait(i2c, CONFIG_SYS_HZ / 10);
		if (stat < 0)
			return -EAGAIN;
	}

	/* Check for error condition on the bus */
	if (i2c->stat & HDMI_IH_I2CM_STAT0_ERROR)
		return -EIO;

	return 0;
}

static int dw_hdmi_i2c_read(struct dw_hdmi_i2c *i2c,
			    unsigned char *buf, unsigned int length)
{
	int ret;

	if (!i2c->is_regaddr) {
		debug("set read register address to 0\n");
		i2c->slave_reg = 0x00;
		i2c->is_regaddr = true;
	}

	while (length--) {
		hdmi_writeb(i2c, i2c->slave_reg++, HDMI_I2CM_ADDRESS);
		if (i2c->is_segment)
			hdmi_writeb(i2c, HDMI_I2CM_OPERATION_READ_EXT,
				    HDMI_I2CM_OPERATION);
		else
			hdmi_writeb(i2c, HDMI_I2CM_OPERATION_READ,
				    HDMI_I2CM_OPERATION);

		ret = dw_hdmi_i2c_wait(i2c);
		if (ret)
			return ret;

		*buf++ = hdmi_readb(i2c, HDMI_I2CM_DATAI);
	}
	i2c->is_segment = false;

	return 0;
}

static int dw_hdmi_i2c_write(struct dw_hdmi_i2c *i2c,
			     unsigned char *buf, unsigned int length)
{
	int ret;

	if (!i2c->is_regaddr) {
		/* Use the first write byte as register address */
		i2c->slave_reg = buf[0];
		length--;
		buf++;
		i2c->is_regaddr = true;
	}

	while (length--) {
		hdmi_writeb(i2c, *buf++, HDMI_I2CM_DATAO);
		hdmi_writeb(i2c, i2c->slave_reg++, HDMI_I2CM_ADDRESS);
		hdmi_writeb(i2c, HDMI_I2CM_OPERATION_WRITE,
			    HDMI_I2CM_OPERATION);

		ret = dw_hdmi_i2c_wait(i2c);
		if (ret)
			return ret;
	}

	return 0;
}

static int dw_hdmi_i2c_xfer(struct udevice *bus, struct i2c_msg *msgs, int num)
{
	struct dw_hdmi_i2c *i2c = dev_get_priv(bus);
	u8 addr = msgs[0].addr;
	int i, ret = 0;

	if (addr == DDC_CI_ADDR)
		/*
		 * The internal I2C controller does not support the multi-byte
		 * read and write operations needed for DDC/CI.
		 * TOFIX: Blacklist the DDC/CI address until we filter out
		 * unsupported I2C operations.
		 */
		return -EOPNOTSUPP;

	debug("xfer: num: %d, addr: %#x\n", num, addr);

	for (i = 0; i < num; i++) {
		if (msgs[i].len == 0) {
			debug("unsupported transfer %d/%d, no data\n",
				i + 1, num);
			return -EOPNOTSUPP;
		}
	}

	mutex_lock(&i2c->lock);

	/* Unmute DONE and ERROR interrupts */
	hdmi_writeb(i2c, 0x00, HDMI_IH_MUTE_I2CM_STAT0);

	/* Set slave device address taken from the first I2C message */
	hdmi_writeb(i2c, addr, HDMI_I2CM_SLAVE);

	/* Set slave device register address on transfer */
	i2c->is_regaddr = false;

	/* Set segment pointer for I2C extended read mode operation */
	i2c->is_segment = false;

	for (i = 0; i < num; i++) {
		debug("xfer: num: %d/%d, len: %d, flags: %#x\n",
			i + 1, num, msgs[i].len, msgs[i].flags);
		if (msgs[i].addr == DDC_SEGMENT_ADDR && msgs[i].len == 1 &&
				!(msgs[i].flags & I2C_M_RD)) {
			i2c->is_segment = true;
			hdmi_writeb(i2c, DDC_SEGMENT_ADDR, HDMI_I2CM_SEGADDR);
			hdmi_writeb(i2c, *msgs[i].buf, HDMI_I2CM_SEGPTR);
		} if (msgs[i].addr == 0x59 && msgs[i].len == 1 &&
					(msgs[i].flags & I2C_M_RD)) {
			/* Sceptre monitor locks up edid if this is read */
			ret = -EIO;
		} else {
			if (msgs[i].flags & I2C_M_RD)
				ret = dw_hdmi_i2c_read(i2c, msgs[i].buf,
						       msgs[i].len);
			else
				ret = dw_hdmi_i2c_write(i2c, msgs[i].buf,
							msgs[i].len);
		}
		if (ret < 0)
			break;
	}

	if (!ret)
		ret = num;

	/* Mute DONE and ERROR interrupts */
	hdmi_writeb(i2c, HDMI_IH_I2CM_STAT0_ERROR | HDMI_IH_I2CM_STAT0_DONE,
		    HDMI_IH_MUTE_I2CM_STAT0);

	mutex_unlock(&i2c->lock);

	return ret;
}

static int dw_hdmi_i2c_set_bus_speed(struct udevice *bus, uint speed)
{
	struct dw_hdmi_i2c *i2c = dev_get_priv(bus);

	i2c->speed = speed;
	return 0;
}

static int dw_hdmi_i2c_probe_chip(struct udevice *bus, u32 chip_addr,
			      u32 chip_flags)
{
	struct i2c_msg msg;
	unsigned char buffer[4];
	int ret;

	msg.addr = chip_addr;
	msg.flags = I2C_M_RD;
	msg.len = 1;
	msg.buf = buffer;
	ret = dw_hdmi_i2c_xfer(bus, &msg, 1);
	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;
	return 0;
}

#if 0
static const char* const clock_names[] = {
	"iahb",    "isfr",    "phy_int", "prep_clk",
	"skp_clk", "sfr_clk", "pix_clk", "cec_clk",
	"apb_clk", "hpi_clk", "fdcc_ref", "pipe_clk"
};
#endif

static int dw_hdmi_i2c_probe(struct udevice *bus)
{
	struct dw_hdmi_i2c *i2c = dev_get_priv(bus);
	int ret;
#if 0
	int i;
#endif

	debug("%s\n", __func__);
	i2c->bus = bus;
	ret = regmap_init_mem(dev_ofnode(bus), &i2c->regm);
	if (ret) {
		debug("%s: Couldn't create regmap\n", __func__);
		return ret;
	}
	debug("%s:b\n", __func__);

#if 0
	ret = devm_clk_get_enable_bulk(bus, clock_names, ARRAY_SIZE(clock_names),
			4, &i2c->bulk_clks);
	if (ret)
		goto clks_disable;
#endif
	debug("%s:c\n", __func__);
	/*
	 * Reset HDMI DDC I2C master controller and mute I2CM interrupts.
	 * Even if we are using a separate i2c adapter doing this doesn't
	 * hurt.
	 */
	dw_hdmi_i2c_init(i2c);

	debug("%s: registered I2C bus driver\n", __func__);
	return 0;
#if 0
clks_disable:
	if (i2c->bulk_clks) {
		for (i = ARRAY_SIZE(clock_names) - 1; i >= 0; i--) {
			if (i2c->bulk_clks[i].dev)
				clk_disable_unprepare(&i2c->bulk_clks[i]);
		}
	}
	return ret;
#endif
}

static const struct dm_i2c_ops dw_hdmi_i2c_ops = {
	.xfer		= dw_hdmi_i2c_xfer,
	.probe_chip	= dw_hdmi_i2c_probe_chip,
	.set_bus_speed	= dw_hdmi_i2c_set_bus_speed,
};

static const struct udevice_id dw_hdmi_i2c_dt_ids[] = {
	{ .compatible = "fsl,imx6q-hdmi-i2c",},
	{ .compatible = "fsl,imx6dl-hdmi-i2c",},
	{ .compatible = "fsl,imx8mp-hdmi-i2c",},
	{},
};

U_BOOT_DRIVER(dw_hdmi_i2c) = {
	.name = "dw_hdmi_i2c",
	.id = UCLASS_I2C,
	.of_match = dw_hdmi_i2c_dt_ids,
	.probe = dw_hdmi_i2c_probe,
	.priv_auto_alloc_size = sizeof(struct dw_hdmi_i2c),
	.ops = &dw_hdmi_i2c_ops,
};

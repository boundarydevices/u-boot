// SPDX-License-Identifier: GPL-2.0-only
/*
 *  sn65dsi83.c - DVI output chip
 *
 *  Copyright (C) 2019 Boundary Devices. All Rights Reserved.
 */

#include <common.h>
#include <asm/gpio.h>
#include <clk.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <errno.h>
#include <i2c.h>
#include <linux/delay.h>
#include <log.h>
#include <panel.h>
#include <watchdog.h>

/* register definitions according to the sn65dsi83 data sheet */
#define SN_SOFT_RESET		0x09
#define SN_CLK_SRC		0x0a
#define SN_CLK_DIV		0x0b

#define SN_PLL_EN		0x0d
#define SN_DSI_LANES		0x10
#define SN_DSI_EQ		0x11
#define SN_DSI_CLK		0x12
#define SN_FORMAT		0x18

#define SN_LVDS_VOLTAGE		0x19
#define SN_LVDS_TERM		0x1a
#define SN_LVDS_CM_VOLTAGE	0x1b
#define SN_HACTIVE_LOW		0x20
#define SN_HACTIVE_HIGH		0x21
#define SN_VACTIVE_LOW		0x24
#define SN_VACTIVE_HIGH		0x25
#define SN_SYNC_DELAY_LOW	0x28
#define SN_SYNC_DELAY_HIGH	0x29
#define SN_HSYNC_LOW		0x2c
#define SN_HSYNC_HIGH		0x2d
#define SN_VSYNC_LOW		0x30
#define SN_VSYNC_HIGH		0x31
#define SN_HBP			0x34
#define SN_VBP			0x36
#define SN_HFP			0x38
#define SN_VFP			0x3a
#define SN_TEST_PATTERN		0x3c
#define SN_IRQ_EN		0xe0
#define SN_IRQ_MASK		0xe1
#define SN_IRQ_STAT		0xe5

struct sn65dsi83_priv
{
	struct udevice	*dev;
	ofnode	disp_node;
	ofnode	disp_dsi;

	struct gpio_desc	*gd_enable[2];
	struct gpio_desc	*gd_irq[2];
	struct clk		*mipi_clk;
	struct clk		*pixel_clk;
	struct display_timing	timings;
	u32			int_cnt;
	u8			chip_enabled;
	u8			irq_enabled;
	u8			show_reg;
	u8			dsi_lanes;
	u8			spwg;	/* lvds lane 3 has MSBs of color */
	u8			jeida;	/* lvds lane 3 has LSBs of color */
	u8			split_mode;
	u8			dsi_bpp;
	u16			sync_delay;
	u16			hbp;

	u8			dsi_clk_divider;
	u8			mipi_clk_index;
#define SN_STATE_OFF		0
#define SN_STATE_STANDBY	1
#define SN_STATE_ON		2
	u8			state;
	struct gpio_desc	gds_enable[2];
	struct gpio_desc	gds_irq[2];
};

/**
 * sn_i2c_read_reg - read data from a register of the i2c slave device.
 *
 */
static int sn_i2c_read_byte(struct sn65dsi83_priv *sn, u8 reg)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(sn->dev);
	uint8_t tx_buf[4];
	uint8_t rx_buf[4];
	struct i2c_msg msgs[2];
	int ret;

	tx_buf[0] = reg;
	msgs[0].addr = chip->chip_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = tx_buf;

	msgs[1].addr = chip->chip_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = rx_buf;
	ret = dm_i2c_xfer(sn->dev, msgs, 2);
	if (ret) {
		debug("%s reg(%x), failed(%d)\n", __func__, reg, ret);
		return ret;
	}
	return rx_buf[0];
}

/**
 * sn_i2c_write - write data to a register of the i2c slave device.
 *
 */
static int sn_i2c_write_byte(struct sn65dsi83_priv *sn, u8 reg, u8 val)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(sn->dev);
	uint8_t buf[4];
	struct i2c_msg msg;
	int ret;

	msg.addr = chip->chip_addr;
	msg.flags = 0;
	buf[0] = reg;
	buf[1] = val;
	msg.buf = buf;
	msg.len = 2;
	ret = dm_i2c_xfer(sn->dev, &msg, 1);
	if (ret) {
		debug("%s reg(%x)=%x, failed(%d)\n", __func__, reg, val, ret);
	}
	return ret;
}

static void sn_disable(struct sn65dsi83_priv *sn, int skip_irq)
{
	int i;

	if (sn->irq_enabled && !skip_irq) {
		sn->irq_enabled = 0;
		sn->int_cnt = 0;
	}
	sn->chip_enabled = 0;
	for (i = 0; i < ARRAY_SIZE(sn->gd_enable); i++) {
		if (!sn->gd_enable[i])
			break;
		dm_gpio_set_value(sn->gd_enable[i], 0);
	}
}

static void sn_enable_gp(struct sn65dsi83_priv *sn)
{
	int i;

	mdelay(15);	/* disabled for at least 10 ms */
	for (i = 0; i < ARRAY_SIZE(sn->gd_enable); i++) {
		if (!sn->gd_enable[i])
			break;
		dm_gpio_set_value(sn->gd_enable[i], 1);
	}
	mdelay(2);
}

static void sn_enable_irq(struct sn65dsi83_priv *sn)
{
	sn_i2c_write_byte(sn, SN_IRQ_STAT, 0xff);
	sn_i2c_write_byte(sn, SN_IRQ_MASK, 0x7f);
	sn_i2c_write_byte(sn, SN_IRQ_EN, 1);
}

static int sn_get_dsi_clk_divider(struct sn65dsi83_priv *sn)
{
	u32 dsi_clk_divider = 25;
	u32 mipi_clk_rate = 0;
	u8 mipi_clk_index;
	int ret;
	u32 pixelclock = sn->timings.pixelclock.typ;

#if CONFIG_IS_ENABLED(CLK)
	if (!sn->mipi_clk) {
		struct clk *mipi_clk;

		mipi_clk = devm_clk_get(sn->dev, "mipi_clk");
		if (IS_ERR(mipi_clk)) {
			debug("mipi_clk %ld\n", PTR_ERR(mipi_clk));
		} else {
			sn->mipi_clk = mipi_clk;
		}
	}
#endif
	if (sn->mipi_clk)
		mipi_clk_rate = clk_get_rate(sn->mipi_clk);
	if (!mipi_clk_rate) {
		pr_err("mipi clock is off\n");
		/* Divided by 2 because mipi output clock is DDR */
		mipi_clk_rate = pixelclock * sn->dsi_bpp / (sn->dsi_lanes * 2);
	}
	if (mipi_clk_rate > 500000000) {
		pr_err("mipi clock(%d) is too high\n", mipi_clk_rate);
		mipi_clk_rate = 500000000;
	}
	if (pixelclock)
		dsi_clk_divider = (mipi_clk_rate +  (pixelclock >> 1)) / pixelclock;
	if (sn->split_mode)
		dsi_clk_divider <<= 1;	/* double the divisor for split mode */

	if (dsi_clk_divider > 25)
		dsi_clk_divider = 25;
	else if (!dsi_clk_divider)
		dsi_clk_divider = 1;
	mipi_clk_index = mipi_clk_rate / 5000000;
	if (mipi_clk_index < 8)
		mipi_clk_index = 8;
	ret = (sn->dsi_clk_divider == dsi_clk_divider) &&
		(sn->mipi_clk_index == mipi_clk_index);
	if (!ret) {
		debug("dsi_clk_divider = %d, mipi_clk_index=%d, mipi_clk_rate=%d\n",
			dsi_clk_divider, mipi_clk_index, mipi_clk_rate);
		sn->dsi_clk_divider = dsi_clk_divider;
		sn->mipi_clk_index = mipi_clk_index;
	}
	return ret;
}

static int sn_check_videomode_change(struct sn65dsi83_priv *sn)
{
	struct display_timing timings;
	u32 pixelclock;
	int ret;

	ret = ofnode_decode_display_timing(sn->disp_dsi, 0, &timings);
	if (ret < 0)
		return ret;

	if (!IS_ERR(sn->pixel_clk)) {
		pixelclock = clk_get_rate(sn->pixel_clk);
		if (pixelclock)
			timings.pixelclock.typ = pixelclock;
	}

	if (sn->timings.pixelclock.typ != timings.pixelclock.typ ||
	    sn->timings.hactive.typ != timings.hactive.typ ||
	    sn->timings.hsync_len.typ != timings.hsync_len.typ ||
	    sn->timings.hback_porch.typ != timings.hback_porch.typ ||
	    sn->timings.hfront_porch.typ != timings.hfront_porch.typ ||
	    sn->timings.vactive.typ != timings.vactive.typ ||
	    sn->timings.vsync_len.typ != timings.vsync_len.typ ||
	    sn->timings.vback_porch.typ != timings.vback_porch.typ ||
	    sn->timings.vfront_porch.typ != timings.vfront_porch.typ ||
	    ((sn->timings.flags ^ timings.flags) & (DISPLAY_FLAGS_DE_LOW |
			    DISPLAY_FLAGS_HSYNC_HIGH |DISPLAY_FLAGS_VSYNC_HIGH))) {
		debug("%s: pixelclock=%d %dx%d, margins=%d,%d %d,%d  syncs=%d %d flags=%x\n",
			__func__, timings.pixelclock.typ, timings.hactive.typ, timings.vactive.typ,
			timings.hback_porch.typ, timings.hfront_porch.typ,
			timings.vback_porch.typ, timings.vfront_porch.typ,
			timings.hsync_len.typ, timings.vsync_len.typ, timings.flags);
		sn->timings = timings;
		ret = 1;
	}
	if (!sn_get_dsi_clk_divider(sn))
		ret |= 1;
	return ret;
}

static int sn_setup_regs(struct sn65dsi83_priv *sn)
{
	unsigned i = 5;
	int format = 0x10;
	u32 pixelclock;
	int hbp, hfp, hsa;

	pixelclock = sn->timings.pixelclock.typ;
	if (sn->split_mode)
		pixelclock >>= 1;
	if (pixelclock) {
		if (pixelclock > 37500000) {
			i = (pixelclock - 12500000) / 25000000;
			if (i > 5)
				i = 5;
		}
	}
	sn_i2c_write_byte(sn, SN_CLK_SRC, (i << 1) | 1);
	sn_i2c_write_byte(sn, SN_CLK_DIV, (sn->dsi_clk_divider - 1) << 3);

	sn_i2c_write_byte(sn, SN_DSI_LANES, ((4 - sn->dsi_lanes) << 3) | 0x20);
	sn_i2c_write_byte(sn, SN_DSI_EQ, 0);
	sn_i2c_write_byte(sn, SN_DSI_CLK, sn->mipi_clk_index);
	if (sn->timings.flags & DISPLAY_FLAGS_DE_LOW)
		format |= BIT(7);
	if (!(sn->timings.flags & DISPLAY_FLAGS_HSYNC_HIGH))
		format |= BIT(6);
	if (!(sn->timings.flags & DISPLAY_FLAGS_VSYNC_HIGH))
		format |= BIT(5);
	if (sn->dsi_bpp == 24) {
		if (sn->spwg) {
			/* lvds lane 3 has MSBs of color */
			format |= (sn->split_mode) ? BIT(3) | BIT(2) : BIT(3);
		} else if (sn->jeida) {
			/* lvds lane 3 has LSBs of color */
			format |= (sn->split_mode) ?
					BIT(3) | BIT(2) | BIT(1) | BIT(0) :
					BIT(3) | BIT(1);
		} else {
			/* unused lvds lane 3 has LSBs of color */
			format |= BIT(1);
		}
	}
	if (sn->split_mode)
		format &= ~BIT(4);
	sn_i2c_write_byte(sn, SN_FORMAT, format);

	sn_i2c_write_byte(sn, SN_LVDS_VOLTAGE, 5);
	sn_i2c_write_byte(sn, SN_LVDS_TERM, 3);
	sn_i2c_write_byte(sn, SN_LVDS_CM_VOLTAGE, 0);
	sn_i2c_write_byte(sn, SN_HACTIVE_LOW, (u8)sn->timings.hactive.typ);
	sn_i2c_write_byte(sn, SN_HACTIVE_HIGH, (u8)(sn->timings.hactive.typ >> 8));
	sn_i2c_write_byte(sn, SN_VACTIVE_LOW, (u8)sn->timings.vactive.typ);
	sn_i2c_write_byte(sn, SN_VACTIVE_HIGH, (u8)(sn->timings.vactive.typ >> 8));
	sn_i2c_write_byte(sn, SN_SYNC_DELAY_LOW, (u8)sn->sync_delay);
	sn_i2c_write_byte(sn, SN_SYNC_DELAY_HIGH, (u8)(sn->sync_delay >> 8));
	hsa = sn->timings.hsync_len.typ;
	hbp = sn->timings.hback_porch.typ;
	hfp = sn->timings.hfront_porch.typ;
	if (sn->hbp) {
		int diff = sn->hbp - hbp;

		hbp += diff;
		hfp -= diff;
		if (hfp < 1) {
			diff = 1 - hfp;
			hfp += diff;
			hsa -= diff;
			if (hsa < 1) {
				diff = 1 - hsa;
				hsa += diff;
				hbp -= diff;
			}
		}
	}
	sn_i2c_write_byte(sn, SN_HSYNC_LOW, (u8)hsa);
	sn_i2c_write_byte(sn, SN_HSYNC_HIGH, (u8)(hsa >> 8));
	sn_i2c_write_byte(sn, SN_VSYNC_LOW, (u8)sn->timings.vsync_len.typ);
	sn_i2c_write_byte(sn, SN_VSYNC_HIGH, (u8)(sn->timings.vsync_len.typ >> 8));
	sn_i2c_write_byte(sn, SN_HBP, (u8)hbp);
	sn_i2c_write_byte(sn, SN_VBP, (u8)sn->timings.vback_porch.typ);
	sn_i2c_write_byte(sn, SN_HFP, (u8)hfp);
	sn_i2c_write_byte(sn, SN_VFP, (u8)sn->timings.vfront_porch.typ);
	sn_i2c_write_byte(sn, SN_TEST_PATTERN, 0);
	return 0;
}

static void sn_enable_pll(struct sn65dsi83_priv *sn)
{
	if (!sn_get_dsi_clk_divider(sn)) {
		sn_i2c_write_byte(sn, SN_CLK_DIV, (sn->dsi_clk_divider - 1) << 3);
		sn_i2c_write_byte(sn, SN_DSI_CLK, sn->mipi_clk_index);
	}
	sn_i2c_write_byte(sn, SN_PLL_EN, 1);
	mdelay(5);
	sn_i2c_write_byte(sn, SN_SOFT_RESET, 1);
	sn_enable_irq(sn);
	debug("%s:reg0a=%02x\n", __func__, sn_i2c_read_byte(sn, SN_CLK_SRC));
}

static void sn_disable_pll(struct sn65dsi83_priv *sn)
{
	if (sn->chip_enabled)
		sn_i2c_write_byte(sn, SN_PLL_EN, 0);
}

static void sn_init(struct sn65dsi83_priv *sn)
{
	sn_i2c_write_byte(sn, SN_SOFT_RESET, 1);
	sn_i2c_write_byte(sn, SN_PLL_EN, 0);
	sn_i2c_write_byte(sn, SN_IRQ_MASK, 0x0);
	sn_i2c_write_byte(sn, SN_IRQ_EN, 1);
}

static void sn_standby(struct sn65dsi83_priv *sn)
{
	if (sn->state < SN_STATE_STANDBY) {
		sn_enable_gp(sn);
		sn_init(sn);
		if (!sn->irq_enabled) {
			sn->irq_enabled = 1;
		}
		sn->chip_enabled = 1;
		sn_setup_regs(sn);
		sn->state = SN_STATE_STANDBY;
		debug("%s:\n", __func__);
	}
}

static void sn_prepare(struct sn65dsi83_priv *sn)
{
	if (sn->state < SN_STATE_STANDBY)
		sn_standby(sn);
	if (sn->state < SN_STATE_ON) {
		mdelay(2);
		sn_enable_pll(sn);
		sn->state = SN_STATE_ON;
	}
}

static void sn_powerdown1(struct sn65dsi83_priv *sn, int skip_irq)
{
	debug("%s\n", __func__);
	if (sn->state) {
		sn_disable_pll(sn);
		sn_disable(sn, skip_irq);
		sn->state = SN_STATE_OFF;
	}
}

static void sn_powerdown(struct sn65dsi83_priv *sn)
{
	sn_powerdown1(sn, 0);
}

static void sn_powerup_lock(struct sn65dsi83_priv *sn)
{
	sn_prepare(sn);
}

static void sn_powerup(struct sn65dsi83_priv *sn)
{
	int ret = sn_check_videomode_change(sn);

	if (ret) {
		if (ret < 0) {
			debug("%s: videomode error\n", __func__);
			return;
		}
		sn_powerdown(sn);
	}
	debug("%s\n", __func__);
	sn_powerup_lock(sn);
}

#if 0
static void sn_powerup_begin(struct sn65dsi83_priv *sn)
{
	int ret = sn_check_videomode_change(sn);

	if (ret) {
		if (ret < 0) {
			debug("%s: videomode error\n", __func__);
			return;
		}
		sn_powerdown(sn);
	}
	debug("%s\n", __func__);
	sn_standby(sn);
	mdelay(1000);
	sn_powerup(sn);
}
#endif

/*
 * We only report errors in this handler
 */
int sn_irq_handler(struct sn65dsi83_priv *sn)
{
	int status = sn_i2c_read_byte(sn, SN_IRQ_STAT);

	if (status >= 0) {
		debug("%s: stat=%x\n", __func__, status);
		if (status)
			sn_i2c_write_byte(sn, SN_IRQ_STAT, status);
	} else {
		debug("%s: read error %d\n", __func__, status);
	}
	sn->int_cnt++;
	if (sn->int_cnt > 64) {
		debug("%s: status %x %x %x\n", __func__,
			status, sn_i2c_read_byte(sn, SN_CLK_SRC),
			sn_i2c_read_byte(sn, SN_IRQ_MASK));
		if (sn->irq_enabled) {
			sn->irq_enabled = 0;
			debug("%s: disabling irq, status=0x%x\n", __func__, status);
		}
		return 0;
	}

	if (status > 0) {
//		if (status & 1)
//			sn_i2c_write_byte(sn, SN_SOFT_RESET, 1);
		if (!(sn->int_cnt & 7) && sn->chip_enabled) {
			debug("%s: trying to reinit, status %x %x %x\n", __func__,
				status, sn_i2c_read_byte(sn, SN_CLK_SRC),
				sn_i2c_read_byte(sn, SN_IRQ_MASK));
			sn_powerdown1(sn, 1);
			sn_powerup_lock(sn);
		}
		return 1;
	}
	return 0;
}

struct sn65dsi83_priv *g_sn;

void sn65dsi83_poll(void)
{
	struct sn65dsi83_priv *sn = g_sn;
	int ret, i;

	if (!sn->irq_enabled)
		return;
	for (i = 0; i < ARRAY_SIZE(sn->gd_irq); i++) {
		struct gpio_desc *gd_irq = sn->gd_irq[i];

		if (!gd_irq)
			break;
		ret = dm_gpio_get_value(gd_irq);
		if (ret) {
			debug("%s: %d %s: %d\n", __func__, ret, gd_irq->dev->name, gd_irq->offset);
			sn_irq_handler(sn);
		}
	}
}

static int sn65dsi83_init(struct udevice *dev)
{
	debug("%s:\n", __func__);
	return 0;
}

static void sn65dsi83_uninit(struct udevice *dev)
{
	debug("%s:\n", __func__);
	set_poll_rtn(NULL);
}

static int sn65dsi83_enable_backlight(struct udevice *dev)
{
	struct sn65dsi83_priv *sn = dev_get_priv(dev);

	debug("%s:\n", __func__);
	set_poll_rtn(NULL);
	sn_powerup(sn);
	g_sn = sn;
	if (sn->gd_irq[0])
		set_poll_rtn(sn65dsi83_poll);
	return 0;
}

static int sn65dsi83_set_backlight(struct udevice *dev, int percent)
{
	struct sn65dsi83_priv *sn = dev_get_priv(dev);

	debug("%s:\n", __func__);
	if (percent) {
		sn65dsi83_enable_backlight(dev);
	} else {
		set_poll_rtn(NULL);
		sn_powerdown(sn);
	}
	return 0;
}

static int sn65dsi83_get_display_timing(struct udevice *dev,
					    struct display_timing *timings)
{
	struct sn65dsi83_priv *p = dev_get_priv(dev);

	memcpy(timings, &p->timings, sizeof(*timings));
	return 0;
}

static int of_parse_phandle(ofnode np, const char *propname, ofnode *ph)
{
	struct ofnode_phandle_args phandle;
	int ret;

	ret = ofnode_parse_phandle_with_args(np, propname, NULL, 0, 0, &phandle);
	if (ret) {
		printf("Can't find %s property (%d)\n", propname, ret);
		return ret;
	}
	*ph = phandle.node;
	return 0;
}

static int sn65dsi83_ofdata_to_platdata(struct udevice *dev)
{
	struct sn65dsi83_priv *sn = dev_get_priv(dev);
	ofnode np = dev_ofnode(dev);
	int ret;
	const char *df;
	u32 sync_delay, hbp;
	u32 dsi_lanes;
	struct clk *pixel_clk = NULL;
	int i, j;

#if CONFIG_IS_ENABLED(CLK)
	pixel_clk = devm_clk_get(dev, "pixel_clock");
	if (IS_ERR(pixel_clk)) {
		debug("pixel_clk %ld\n", PTR_ERR(pixel_clk));
		pixel_clk = NULL;
	}
#endif

	for (i = 0; i < ARRAY_SIZE(sn->gds_enable); i++) {
		ret = gpio_request_by_name(dev, "enable-gpios", i, &sn->gds_enable[i],
					   GPIOD_IS_OUT);
		if (ret) {
			debug("%s: Warning: cannot get enable-gpios: ret=%d\n",
			      __func__, ret);
			if (ret != -ENOENT) {
				debug("enable-gpios %d\n", ret);
				return log_ret(ret);
			}
		} else {
			debug("%s: Success, enable-gpios\n", __func__);
			sn->gd_enable[i] = &sn->gds_enable[i];
		}
	}
	sn_enable_gp(sn);

	for (i = 0, j = 0; i < ARRAY_SIZE(sn->gds_irq); i++) {
		ret = gpio_request_by_name(dev, "interrupts-gpios", i, &sn->gds_irq[i],
				   GPIOD_IS_IN);
		if (ret) {
			debug("%s: Warning: cannot get interrupts-gpios: ret=%d\n",
					__func__, ret);
			if (ret != -ENOENT) {
				debug("interrupts-gpios %d\n", ret);
				return log_ret(ret);
			}
		} else {
			sn->gd_irq[j] = &sn->gds_irq[i];
			ret = dm_gpio_get_value(sn->gd_irq[j]);
			if (!ret) {
				/* An interrupt is not pending, valid interrupt pin*/
				j++;
			}
		}
	}

	sn->dev = dev;
	sn->pixel_clk = pixel_clk;

	ret = of_parse_phandle(np, "display-dsi", &sn->disp_dsi);
	if (ret) {
		debug("display-dsi %d\n", ret);
		return ret;
	}
	ret = of_parse_phandle(np, "display", &sn->disp_node);
	if (ret) {
		debug("display %d\n", ret);
		return ret;
	}
	ret = ofnode_decode_display_timing(sn->disp_dsi, 0, &sn->timings);
	if (ret < 0) {
		debug("ofnode_decode_display_timing %d\n", ret);
		return ret;
	}

	sn->sync_delay = 0x120;
	if (!ofnode_read_u32(np, "sync-delay", &sync_delay)) {
		if (sync_delay > 0xfff) {
			debug("sync_delay == %d\n", sync_delay);
			return -EINVAL;
		}
		sn->sync_delay = sync_delay;
	}
	if (!ofnode_read_u32(np, "hbp", &hbp)) {
		if (hbp > 0xff) {
			debug("hbp == %d\n", hbp);
			return -EINVAL;
		}
		sn->hbp = hbp;
	}
	if (ofnode_read_u32(sn->disp_dsi, "dsi-lanes", &dsi_lanes) < 0)
		return -EINVAL;
	if (dsi_lanes < 1 || dsi_lanes > 4) {
		debug("dsi-lanes == %d\n", dsi_lanes);
		return -EINVAL;
	}
	sn->dsi_lanes = dsi_lanes;
	sn->spwg = ofnode_read_bool(np, "spwg");
	sn->jeida = ofnode_read_bool(np, "jeida");
	if (!sn->spwg && !sn->jeida) {
		sn->spwg = ofnode_read_bool(sn->disp_dsi, "spwg");
		sn->jeida = ofnode_read_bool(sn->disp_dsi, "jeida");
	}
	sn->split_mode = ofnode_read_bool(np, "split-mode");
	df = ofnode_read_string(sn->disp_dsi, "dsi-format");
	if (!df) {
		debug("dsi-format missing in display node\n");
		return ret;
	}
	sn->dsi_bpp = !strcmp(df, "rgb666") ? 18 : 24;
	return 0;
}

/*
 * I2C init/probing/exit functions
 */
static int sn65dsi83_probe(struct udevice *dev)
{
	struct sn65dsi83_priv *sn = dev_get_priv(dev);
	int ret, i;

	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;

	ret = sn_i2c_read_byte(sn, SN_CLK_SRC);
	if (ret < 0) {
		for (i = 0; i < ARRAY_SIZE(sn->gd_enable); i++) {
			if (!sn->gd_enable[i])
				break;
			/* enable might be used for something else, change to input */
			dm_gpio_set_dir_flags(sn->gd_enable[i], GPIOD_IS_IN);
		}
		debug("i2c read failed\n");
		return -ENODEV;
	}
	sn_init(sn);
	return 0;
}

static const struct panel_ops sn65dsi83_ops = {
	.init			= sn65dsi83_init,
	.uninit			= sn65dsi83_uninit,
	.enable_backlight	= sn65dsi83_enable_backlight,
	.get_display_timing	= sn65dsi83_get_display_timing,
	.set_backlight		= sn65dsi83_set_backlight,
};

static const struct udevice_id sn65dsi83_dt_match[] = {
	{.compatible = "ti,sn65dsi83"},
	{}
};

U_BOOT_DRIVER(sn65dsi83) = {
	.name	= "sn65dsi83",
	.id	= UCLASS_PANEL,
	.of_match = sn65dsi83_dt_match,
	.ofdata_to_platdata	= sn65dsi83_ofdata_to_platdata,
	.probe	= sn65dsi83_probe,
	.ops	= &sn65dsi83_ops,
	.priv_auto_alloc_size	= sizeof(struct sn65dsi83_priv),
};

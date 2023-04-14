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
#include <dm/device_compat.h>
#include <dm/ofnode.h>
#include <dm/pinctrl.h>
#include <errno.h>
#include <i2c.h>
#include <linux/delay.h>
#include <log.h>
#include <panel.h>
#include <watchdog.h>
#include "panel-sn65dsi83.h"

/**
 * sn_i2c_read_reg - read data from a register of the i2c slave device.
 *
 */
static int sn_i2c_read_byte(struct panel_sn65dsi83 *sn, u8 reg)
{
	struct dm_i2c_ops *ops = i2c_get_ops(sn->i2c);
	uint8_t tx_buf[4];
	uint8_t rx_buf[4];
	struct i2c_msg msgs[2];
	int ret;

	tx_buf[0] = reg;
	msgs[0].addr = sn->i2c_address;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = tx_buf;

	msgs[1].addr = sn->i2c_address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = rx_buf;
	ret = ops->xfer(sn->i2c, msgs, 2);
	if (ret < 0) {
		debug("%s reg(%x), failed(%d)\n", __func__, reg, ret);
		return ret;
	}
	return rx_buf[0];
}

/**
 * sn_i2c_write - write data to a register of the i2c slave device.
 *
 */
static int sn_i2c_write_byte(struct panel_sn65dsi83 *sn, u8 reg, u8 val)
{
	struct dm_i2c_ops *ops = i2c_get_ops(sn->i2c);
	uint8_t buf[4];
	struct i2c_msg msg;
	int ret;

	buf[0] = reg;
	buf[1] = val;

	msg.addr = sn->i2c_address;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;
	ret = ops->xfer(sn->i2c, &msg, 1);
	if (ret < 0) {
		debug("%s reg(%x)=%x, failed(%d)\n", __func__, reg, val, ret);
		mdelay(10);
		ret = ops->xfer(sn->i2c, &msg, 1);
	}
	return ret < 0 ? ret : 0;
}

static void sn_disable(struct panel_sn65dsi83 *sn, int skip_irq)
{
	int i;

	if (sn->irq_enabled && !skip_irq) {
		sn->irq_enabled = 0;
		sn->int_cnt = 0;
	}
	sn->chip_enabled = 0;
	for (i = 0; i < ARRAY_SIZE(sn->gp_en); i++) {
		if (!sn->gp_en[i])
			break;
		dm_gpio_set_value(sn->gp_en[i], 0);
	}
}

static void sn_enable_gp(struct panel_sn65dsi83 *sn)
{
	int i;

	mdelay(15);	/* disabled for at least 10 ms */

	for (i = 0; i < ARRAY_SIZE(sn->gp_en); i++) {
		if (!sn->gp_en[i])
			break;
		dm_gpio_set_value(sn->gp_en[i], 1);
	}
	mdelay(12);	/* enabled for at least 10 ms before i2c writes */
}

static void sn_enable_irq(struct panel_sn65dsi83 *sn)
{
	sn_i2c_write_byte(sn, SN_IRQ_STAT, 0xff);
	sn_i2c_write_byte(sn, SN_IRQ_MASK, 0x7f);
	sn_i2c_write_byte(sn, SN_IRQ_EN, 1);
}

static int sn_get_dsi_clk_divider(struct panel_sn65dsi83 *sn)
{
	u32 dsi_clk_divider = 25;
	u32 mipi_clk_rate = 0;
	u8 mipi_clk_index;
	int ret;
	u32 pixelclock = sn->timings.pixelclock.typ;

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

static int sn_check_videomode_change(struct panel_sn65dsi83 *sn)
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

static int sn_setup_regs(struct panel_sn65dsi83 *sn)
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
	/*
	 * changing from 62M to 63M (1->2) causes an unstable panel
	 * changing from 1->3 does not.
	 */
	if (i == 2)
		i = 3;
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

static void sn_enable_pll(struct panel_sn65dsi83 *sn)
{
	if (!sn_get_dsi_clk_divider(sn)) {
		sn_i2c_write_byte(sn, SN_CLK_DIV, (sn->dsi_clk_divider - 1) << 3);
		sn_i2c_write_byte(sn, SN_DSI_CLK, sn->mipi_clk_index);
	}
	sn_i2c_write_byte(sn, SN_PLL_EN, 1);
	mdelay(12);
	sn_i2c_write_byte(sn, SN_SOFT_RESET, 1);
	mdelay(12);
	debug("%s:reg0a=%02x\n", __func__, sn_i2c_read_byte(sn, SN_CLK_SRC));
}

static void sn_disable_pll(struct panel_sn65dsi83 *sn)
{
	if (sn->chip_enabled)
		sn_i2c_write_byte(sn, SN_PLL_EN, 0);
}

static void sn_init(struct panel_sn65dsi83 *sn)
{
	sn_i2c_write_byte(sn, SN_SOFT_RESET, 1);
	sn_i2c_write_byte(sn, SN_PLL_EN, 0);
	sn_i2c_write_byte(sn, SN_IRQ_MASK, 0x0);
	sn_i2c_write_byte(sn, SN_IRQ_EN, 1);
}

static void sn_standby(struct panel_sn65dsi83 *sn)
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

static void sn_prepare(struct panel_sn65dsi83 *sn)
{
	if (sn->state < SN_STATE_STANDBY)
		sn_standby(sn);
	if (sn->state < SN_STATE_ON) {
		mdelay(2);
		sn_enable_pll(sn);
		sn->state = SN_STATE_ON;
	}
}

static void sn_powerdown1(struct panel_sn65dsi83 *sn, int skip_irq)
{
	set_poll_rtn(NULL);
	debug("%s\n", __func__);
	if (sn->state) {
		sn_disable_pll(sn);
		sn_disable(sn, skip_irq);
		sn->state = SN_STATE_OFF;
	}
}

static void sn_powerdown(struct panel_sn65dsi83 *sn)
{
	sn_powerdown1(sn, 0);
}

void sn65_disable(struct panel_sn65dsi83 *sn)
{
	if (!sn->dev)
		return;
	sn_powerdown1(sn, 0);
}

static void sn_powerup_lock(struct panel_sn65dsi83 *sn)
{
	sn_prepare(sn);
}

static void sn_powerup(struct panel_sn65dsi83 *sn)
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
static void sn_powerup_begin(struct panel_sn65dsi83 *sn)
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
int sn_irq_handler(struct panel_sn65dsi83 *sn)
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

struct panel_sn65dsi83 *g_sn;

void sn65dsi83_poll(void)
{
	struct panel_sn65dsi83 *sn = g_sn;
	int ret, i;

	if (!sn->irq_enabled)
		return;
	for (i = 0; i < ARRAY_SIZE(sn->gp_irq); i++) {
		struct gpio_desc *gp_irq = sn->gp_irq[i];

		if (!gp_irq)
			break;
		ret = dm_gpio_get_value(gp_irq);
		if (ret) {
			debug("%s: %d %s: %d\n", __func__, ret, gp_irq->dev->name, gp_irq->offset);
			sn_irq_handler(sn);
		}
	}
}

void sn65_enable(struct panel_sn65dsi83 *sn)
{
	debug("%s:\n", __func__);
	set_poll_rtn(NULL);
	sn_powerup(sn);
	g_sn = sn;
	if (sn->gp_irq[0])
		set_poll_rtn(sn65dsi83_poll);
}

void sn65_enable2(struct panel_sn65dsi83 *sn)
{
	if (!sn->dev)
		return;
	mdelay(5);
	dev_dbg(sn->dev, "%s\n", __func__);
	sn_enable_irq(sn);
}

int sn65dsi83_ofdata_to_platdata(struct udevice *dev, struct panel_sn65dsi83 *sn,
		ofnode disp_dsi, ofnode np)
{
	int ret;
	const char *df;
	u32 sync_delay, hbp;
	u32 dsi_lanes;
	struct clk *pixel_clk = NULL;
	int i, j;

	ret = uclass_get_device_by_ofnode_prop(UCLASS_I2C, np,
			"i2c-bus", &sn->i2c);
	if (ret)
		printf("!!!i2c-bus not found %d\n", ret);
	ofnode_read_u32(np, "i2c-address", &sn->i2c_address);
	ofnode_read_u32(np, "i2c-max-frequency", &sn->i2c_max_frequency);

#if CONFIG_IS_ENABLED(CLK)
	pixel_clk = devm_clk_get(dev, "pixel_clock");
	if (IS_ERR(pixel_clk)) {
		debug("pixel_clk %ld\n", PTR_ERR(pixel_clk));
		pixel_clk = NULL;
	}
#endif

	for (i = 0; i < ARRAY_SIZE(sn->gds_en); i++) {
		ret = gpio_request_by_name_nodev(np, "enable-gpios", i, &sn->gds_en[i],
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
			sn->gp_en[i] = &sn->gds_en[i];
		}
	}

	for (i = 0, j = 0; i < ARRAY_SIZE(sn->gds_irq); i++) {
		ret = gpio_request_by_name_nodev(np, "interrupts-gpios", i, &sn->gds_irq[i],
				   GPIOD_IS_IN);
		if (ret) {
			debug("%s: Warning: cannot get interrupts-gpios: ret=%d\n",
					__func__, ret);
			if (ret != -ENOENT) {
				debug("interrupts-gpios %d\n", ret);
				return log_ret(ret);
			}
		} else {
			sn->gp_irq[j] = &sn->gds_irq[i];
		}
	}

	sn->dev = dev;
	sn->pixel_clk = pixel_clk;

	sn->disp_dsi = disp_dsi;
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
int sn65_init(struct panel_sn65dsi83 *sn)
{
	int ret, i, j;

	if (!sn->dev)
		return -ENODEV;
	ret = pinctrl_select_state(sn->dev, "sn65dsi83");
	sn_enable_gp(sn);

	for (i = 0, j = 0; i < ARRAY_SIZE(sn->gp_irq); i++) {
		if (sn->gp_irq[j]) {
			ret = dm_gpio_get_value(sn->gp_irq[j]);
			if (!ret) {
				/* An interrupt is not pending, valid interrupt pin */
				if (j) {
					sn->gp_irq[0] = sn->gp_irq[j];
				}
				sn->gp_irq[1] = NULL;
				break;
			}
		}
	}

	ret = sn_i2c_read_byte(sn, SN_CLK_SRC);
	if (ret < 0) {
		for (i = 0; i < ARRAY_SIZE(sn->gp_en); i++) {
			if (!sn->gp_en[i])
				break;
			/* enable might be used for something else, change to input */
			dm_gpio_set_dir_flags(sn->gp_en[i], GPIOD_IS_IN);
			sn->gp_en[i] = NULL;
		}
		debug("i2c read failed\n");
		gpio_free_list_nodev(sn->gds_en, ARRAY_SIZE(sn->gds_en));
		for (i = 0; i < ARRAY_SIZE(sn->gp_irq); i++) {
			sn->gp_irq[i] = NULL;
		}
		gpio_free_list_nodev(sn->gds_irq, ARRAY_SIZE(sn->gds_irq));
		return -ENODEV;
	}
	sn_init(sn);
	return 0;
}

int sn65_remove(struct panel_sn65dsi83 *sn)
{
	debug("%s:\n", __func__);
	sn_powerdown(sn);
	return 0;
}

// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek kd070fhfid015_dsi driver
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Tommy Chen <tommyyl.chen@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include "mtk_panel.h"

void panel_get_desc(struct panel_description **panel_desc);

struct dsi_info kd070fhfid015_dsi = {
	.lanes = 4,
	.flag = (MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM),
	.format = MIPI_DSI_FMT_RGB888,
};

struct panel_serializable_data STARTEK_KD070FHFID015 = {
	.vm = {
		.pixelclock = 163204,/* pixelclock in KHz */
		.pll_clk = 520,
		.hactive = 1200,
		.hfront_porch = 144,
		.hback_porch = 45,
		.hsync_len = 16,
		.vactive = 1920,
		.vfront_porch = 8,
		.vback_porch = 4,
		.vsync_len = 4,
		.vrefresh = 60,
	},
	.init_cmd = {
		INIT_DCS_CMD(0x01),
		INIT_DELAY_CMD(5),
		INIT_DCS_CMD(0x11),
		INIT_DELAY_CMD(120),
		INIT_GENERIC_CMD(0xB0, 0x04),
		INIT_DELAY_CMD(5),
		INIT_GENERIC_CMD(0xB3, 0x14, 0x08, 0x00, 0x22, 0x00),
		INIT_GENERIC_CMD(0xB4, 0x0C),
		INIT_GENERIC_CMD(0xB6, 0x3A, 0xD3),
		INIT_DCS_CMD(0x51, 0xE6),
		INIT_DCS_CMD(0x53, 0x2C),
		INIT_DCS_CMD(0x3A, 0x77),
		INIT_DCS_CMD(0x29),
		INIT_DELAY_CMD(20),
		INIT_END_CMD,
	},
	.dsi = &kd070fhfid015_dsi,
};

static void startek_kd070fhfid015_power_on(void)
{
	struct panel_description *panel_desc = NULL;

	panel_get_desc(&panel_desc);

	if (dm_gpio_is_valid(&panel_desc->reset_gpio) &&
	    dm_gpio_is_valid(&panel_desc->dcdc_en_gpio) &&
	    dm_gpio_is_valid(&panel_desc->enable_gpio)) {
		dm_gpio_set_value(&panel_desc->enable_gpio, 0);

		dm_gpio_set_value(&panel_desc->dcdc_en_gpio, 0);
		mdelay(1);

		dm_gpio_set_value(&panel_desc->reset_gpio, 0);
		mdelay(1);

		dm_gpio_set_value(&panel_desc->enable_gpio, 1);
		mdelay(10);

		dm_gpio_set_value(&panel_desc->dcdc_en_gpio, 1);
		mdelay(20);

		dm_gpio_set_value(&panel_desc->reset_gpio, 1);
		mdelay(5);

		dm_gpio_set_value(&panel_desc->reset_gpio, 0);
		mdelay(10);

		dm_gpio_set_value(&panel_desc->reset_gpio, 1);
		mdelay(5);
	} else {
		printf("Warning: %s some gpios invalid. could not power on panel\n", __func__);
	}
}

static void startek_kd070fhfid015_power_off(void)
{
	struct panel_description *panel_desc = NULL;

	panel_get_desc(&panel_desc);

	if (dm_gpio_is_valid(&panel_desc->enable_gpio))
		dm_gpio_set_value(&panel_desc->enable_gpio, 0);
	if (dm_gpio_is_valid(&panel_desc->reset_gpio))
		dm_gpio_set_value(&panel_desc->reset_gpio, 0);
	if (dm_gpio_is_valid(&panel_desc->dcdc_en_gpio))
		dm_gpio_set_value(&panel_desc->dcdc_en_gpio, 0);
}

static void enable_backlight(void)
{
	lcm_if_set_backlight(10, 16, false);
}

struct panel_description startek_kd070fhfid015_desc = {
	.name = "startek_kd070fhfid015",
	.compatible = "startek,kd070fhfid015",
	.s = &STARTEK_KD070FHFID015,
	.power_on = startek_kd070fhfid015_power_on,
	.power_off = startek_kd070fhfid015_power_off,
	.backlight_enable = NULL,
};

void panel_get_desc(struct panel_description **panel_desc)
{
	*panel_desc = &startek_kd070fhfid015_desc;
}

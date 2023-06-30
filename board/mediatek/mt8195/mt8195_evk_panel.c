// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek kd070fhfid078_dsi driver
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Tommy Chen <tommyyl.chen@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include "mtk_panel.h"

void panel_get_desc(struct panel_description **panel_desc);

struct dsi_info kd070fhfid078_dsi = {
	.lanes = 4,
	.flag = (MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM),
	.format = MIPI_DSI_FMT_RGB888,
};

struct panel_serializable_data STARTEK_KD070FHFID078 = {
	.vm = {
		.pixelclock = 157126,/* pixelclock in KHz */
		.pll_clk = 502,
		.hactive = 1200,
		.hfront_porch = 50,
		.hback_porch = 70,
		.hsync_len = 10,
		.vactive = 1920,
		.vfront_porch = 25,
		.vback_porch = 20,
		.vsync_len = 4,
		.vrefresh = 60,
	},
	.init_cmd = {
		INIT_GENERIC_CMD(0xB0, 0x05),
		INIT_GENERIC_CMD(0xB3, 0x52),
		INIT_GENERIC_CMD(0xB8, 0x7F),
		INIT_GENERIC_CMD(0xBC, 0x20),
		INIT_GENERIC_CMD(0xD6, 0x7F),
		INIT_GENERIC_CMD(0xB0, 0x01),
		INIT_GENERIC_CMD(0xC0, 0x0D),
		INIT_GENERIC_CMD(0xC1, 0x0D),
		INIT_GENERIC_CMD(0xC2, 0x06),
		INIT_GENERIC_CMD(0xC3, 0x06),
		INIT_GENERIC_CMD(0xC4, 0x08),
		INIT_GENERIC_CMD(0xC5, 0x08),
		INIT_GENERIC_CMD(0xC6, 0x0A),
		INIT_GENERIC_CMD(0xC7, 0x0A),
		INIT_GENERIC_CMD(0xC8, 0x0C),
		INIT_GENERIC_CMD(0xC9, 0x0C),
		INIT_GENERIC_CMD(0xCA, 0x00),
		INIT_GENERIC_CMD(0xCB, 0x00),
		INIT_GENERIC_CMD(0xCC, 0x0E),
		INIT_GENERIC_CMD(0xCD, 0x0E),
		INIT_GENERIC_CMD(0xCE, 0x01),
		INIT_GENERIC_CMD(0xCF, 0x01),
		INIT_GENERIC_CMD(0xD0, 0x04),
		INIT_GENERIC_CMD(0xD1, 0x04),
		INIT_GENERIC_CMD(0xD2, 0x00),
		INIT_GENERIC_CMD(0xD3, 0x00),
		INIT_GENERIC_CMD(0xD4, 0x0D),
		INIT_GENERIC_CMD(0xD5, 0x0D),
		INIT_GENERIC_CMD(0xD6, 0x05),
		INIT_GENERIC_CMD(0xD7, 0x05),
		INIT_GENERIC_CMD(0xD8, 0x07),
		INIT_GENERIC_CMD(0xD9, 0x07),
		INIT_GENERIC_CMD(0xDA, 0x09),
		INIT_GENERIC_CMD(0xDB, 0x09),
		INIT_GENERIC_CMD(0xDC, 0x0B),
		INIT_GENERIC_CMD(0xDD, 0x0B),
		INIT_GENERIC_CMD(0xDE, 0x00),
		INIT_GENERIC_CMD(0xDF, 0x00),
		INIT_GENERIC_CMD(0xE0, 0x0E),
		INIT_GENERIC_CMD(0xE1, 0x0E),
		INIT_GENERIC_CMD(0xE2, 0x01),
		INIT_GENERIC_CMD(0xE3, 0x01),
		INIT_GENERIC_CMD(0xE4, 0x03),
		INIT_GENERIC_CMD(0xE5, 0x03),
		INIT_GENERIC_CMD(0xE6, 0x00),
		INIT_GENERIC_CMD(0xE7, 0x00),
		INIT_GENERIC_CMD(0xB0, 0x03),
		INIT_GENERIC_CMD(0xBA, 0xF0),
		INIT_GENERIC_CMD(0xC8, 0x07),
		INIT_GENERIC_CMD(0xC9, 0x03),
		INIT_GENERIC_CMD(0xCA, 0x41),
		INIT_GENERIC_CMD(0xD2, 0x01),
		INIT_GENERIC_CMD(0xD3, 0x05),
		INIT_GENERIC_CMD(0xD4, 0x05),
		INIT_GENERIC_CMD(0xD5, 0x8A),
		INIT_GENERIC_CMD(0xE4, 0xC0),
		INIT_GENERIC_CMD(0xE5, 0x00),
		INIT_GENERIC_CMD(0xB0, 0x00),
		INIT_GENERIC_CMD(0xBF, 0x1F),
		INIT_GENERIC_CMD(0xC0, 0x12),
		INIT_GENERIC_CMD(0xC2, 0x1E),
		INIT_GENERIC_CMD(0xC4, 0x1E),
		INIT_GENERIC_CMD(0xB0, 0x06),
		INIT_GENERIC_CMD(0xB8, 0xA5),
		INIT_GENERIC_CMD(0xC0, 0xA5),
		INIT_GENERIC_CMD(0xBC, 0x11),
		INIT_GENERIC_CMD(0xD5, 0x48),
		INIT_GENERIC_CMD(0xB8, 0x00),
		INIT_GENERIC_CMD(0xC0, 0x00),
		INIT_DCS_CMD(0x11),
		INIT_DELAY_CMD(120),
		INIT_DCS_CMD(0x29),
		INIT_DELAY_CMD(20),
		INIT_END_CMD,
	},
	.dsi = &kd070fhfid078_dsi,
};

static void startek_kd070fhfid078_power_on(void)
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

static void startek_kd070fhfid078_power_off(void)
{
	struct panel_description *panel_desc = NULL;

	panel_get_desc(&panel_desc);

	if (dm_gpio_is_valid(&panel_desc->enable_gpio))
		dm_gpio_set_value(&panel_desc->enable_gpio, 0);
	if (dm_gpio_is_valid(&panel_desc->reset_gpio)) {
		dm_gpio_set_value(&panel_desc->reset_gpio, 0);
		mdelay(15);
	}
	if (dm_gpio_is_valid(&panel_desc->dcdc_en_gpio)) {
		dm_gpio_set_value(&panel_desc->dcdc_en_gpio, 0);
		mdelay(3);
	}
}

static void enable_backlight(void)
{
	lcm_if_set_backlight(10, 16, false);
}

struct panel_description startek_kd070fhfid078_desc = {
	.name = "startek_kd070fhfid078",
	.compatible = "startek,kd070fhfid078",
	.s = &STARTEK_KD070FHFID078,
	.power_on = startek_kd070fhfid078_power_on,
	.power_off = startek_kd070fhfid078_power_off,
	.backlight_enable = enable_backlight,
};

void panel_get_desc(struct panel_description **panel_desc)
{
	*panel_desc = &startek_kd070fhfid078_desc;
}

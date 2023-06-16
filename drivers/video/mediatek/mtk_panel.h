/* SPDX-License-Identifier: GPL-2.0
 *
 * MediaTek common dsi panel header
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Tommy Chen <tommyyl.chen@mediatek.com>
 */

#ifndef __MTK_PANEL_H__
#define __MTK_PANEL_H__

#include <linux/io.h>
#include <linux/bitops.h>
#include <asm/gpio.h>
#include <clk.h>
#include "mtk_dsi_common.h"

struct dsi_info {
	u8 lanes;    /* interace lanes numeber */
	u32 flag;
	u32 format;
};

/*
 * The data that to be serialized and put into CBFS.
 * Note some fields, for example edid.mode.name, were actually pointers and
 * cannot be really serialized.
 */
struct panel_serializable_data {
	struct videomode vm;  /* edid info of this panel */
	u8 init_cmd[2048]; /* A packed array of lcm_init_command with back scan disabled */
	u8 intf_mode;  /* interface mode,  dsi lvds or dpi */
	struct dsi_info *dsi;
};

struct panel_description {
	const char *name;  /* Panel name for constructing CBFS file name */
	const char *compatible;  /* Panel compatible string in kernel dts */
	struct panel_serializable_data *s;
	struct gpio_desc enable_gpio;
	struct gpio_desc reset_gpio;
	struct gpio_desc dcdc_en_gpio;
	struct clk pwm_main;
	struct clk pwm_mm;
	void (*power_on)(void);  /* Callback to turn on panel */
	void (*power_off)(void);  /* Callback to turn off panel */
	void (*backlight_enable)(void);
};

#define INIT_DCS_CMD(...) \
	LCM_DCS_CMD, \
	sizeof((u8[]){__VA_ARGS__}), \
	__VA_ARGS__

#define INIT_GENERIC_CMD(...) \
	LCM_GENERIC_CMD, \
	sizeof((u8[]){__VA_ARGS__}), \
	__VA_ARGS__

#define INIT_DELAY_CMD(delay) \
	LCM_DELAY_CMD, \
	delay

#define INIT_END_CMD \
	LCM_END_CMD

#endif /* __PANEL_H__ */

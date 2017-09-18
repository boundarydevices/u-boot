/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __BD_COMMON_H_
#define __BD_COMMON_H_     1
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>

struct button_key {
	char const	*name;
	unsigned short	gpnum;
	char		ident;
	char		active_low;
};

struct boot_mode;
extern const struct boot_mode board_boot_modes[];
extern const struct button_key board_buttons[];
extern struct fsl_esdhc_cfg board_usdhc_cfg[];
extern const char *board_type;
void board_preboot_keys(void);
void board_late_specific_init(void);
const char *board_get_board_type(void);
void set_gpios_in(const unsigned short *p, int cnt);
void set_gpios(const unsigned short *p, int cnt, int val);

struct display_info_t;
struct i2c_pads_info;

void common_board_init(const struct i2c_pads_info *p, int i2c_bus_cnt, int otg_id,
		const struct display_info_t *displays, int display_cnt,
		int gp_hd_detect);

#define MAX_BUTTONS	32

void max77823_init(void);
void max77823_otg_power(int enable);
void max77823_boost_power(int enable);

void fan53526_init(void);
#endif

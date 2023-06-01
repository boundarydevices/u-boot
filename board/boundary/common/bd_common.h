/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __BD_COMMON_H_
#define __BD_COMMON_H_     1
#include <asm/mach-imx/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <usb.h>

struct button_key {
	char const	*name;
	unsigned short	gpnum;
	char		ident;
	char		active_low;
	char		tamper;
};
#define TAMPER_CHECK	0xffff

struct boot_mode;
extern const struct boot_mode board_boot_modes[];
extern const struct button_key board_buttons[];
extern struct fsl_esdhc_cfg board_usdhc_cfg[];
extern const char *board_type;
void board_preboot_keys(void);
void board_late_specific_init(void);
void board_eth_addresses(void);
const char *board_get_board_type(void);
void board_usb_reset(int index, enum usb_init_type init);
void set_gpios_in(const unsigned short *p, int cnt);
void set_gpios(const unsigned short *p, int cnt, int val);

struct gpio_reserve {
	unsigned short gpio;
#define GPIOD_OUT_LOW	0
#define GPIOD_OUT_HIGH	1
#define GPIOD_IN	2
	unsigned char init_type;
#define GRF_FREE	BIT(0)
	unsigned char flags;
	const char* name;
};

void gpios_reserve(const struct gpio_reserve *p, int cnt);

struct display_info_t;
struct i2c_pads_info;

void common_board_init(const struct i2c_pads_info *p, int i2c_bus_cnt, int otg_id,
		const struct display_info_t *displays, int display_cnt,
		int gp_hd_detect);
struct snvs_regs;
void tamper_enable(struct snvs_regs *snvs);
void tamper_clear(struct snvs_regs *snvs);
void check_tamper(void);

#define MAX_BUTTONS	32

void max77823_init(void);
void max77823_otg_power(int enable);
void max77823_boost_power(int enable);
int max77823_is_charging(void);
void max77834_power_check(void);

void max77975_init(void);
void max77975_otg_power(int enable);
int max77975_set_chrgin_limit(int ma);

void bq25898_init(int i2c_bus, int i2c_address);

void fan53526_init(void);
int otg_power_detect(void);
int bdcommon_env_init(void);
void spl_dram_init(void);
int board_detect_lcd133(struct display_info_t const *di);
int board_detect_lcd133_x73(struct display_info_t const *di);
int board_detect_pca9540(struct display_info_t const *di);
int board_detect_pca9546(struct display_info_t const *di);
int board_detect_pca9546_x73(struct display_info_t const *di);
int board_detect_sn65_and_ts(struct display_info_t const *di);
int board_detect_pca9546_sn65(struct display_info_t const *di);
int board_detect_pca9546_sn65_x73(struct display_info_t const *di);
int board_detect_pca9546_2(struct display_info_t const *di);
int board_detect_pca9546_2_x73(struct display_info_t const *di);
int board_detect_gt911_common(struct display_info_t const *di,
	int sub_bus, int sub_bus2, int gp_reset, int gp_irq);
int board_detect_gt911_sn65_common(struct display_info_t const *di,
	int sub_bus, int sub_bus2, int gp_reset, int gp_irq);
int board_disable_i2c_mux(int bus_num);

#endif

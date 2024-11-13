/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __BD_COMMON_H_
#define __BD_COMMON_H_     1

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
int detect_common(struct display_info_t const *di, int sub_bus,
		int sub_bus2, int reg1, u8 val1, int reg2, u8 val2,
		int gp_reset, int gp_irq, int probe2);

#endif

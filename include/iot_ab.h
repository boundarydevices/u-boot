/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (C) 2023 MediaTek Inc.
 * Author: Howard Lin <howard-yh.lin@mediatek.com>
 */

#ifndef __MEDIATEK_IOT_AB_H
#define __MEDIATEK_IOT_AB_H

#include <memalign.h>
#include <linux/err.h>
#include <u-boot/crc.h>

#define BOOT_SLOT_NAME(name)	('a' + (name))

#define BOOTCTRL_PART		"0#misc"
#define BOOTCTRL_MAGIC		0x544F494D
#define BOOTCTRL_VERSION	1
#define BOOTCTRL_BOOT_SUCCESS	0
#define BOOTCTRL_SRC		1

#define BOOTCTRL_GET		0
#define BOOTCTRL_SET		1
#define BOOTCTRL_FINISH		2

#define PART_BOOT_A		0
#define PART_BOOT_B		1

#define BOOTCTRL_FIP_NUM	1
#define BOOTCTRL_FW_NUM		3

#define BOOTCTRL_DEV		"mmc"
#define BOOTCTRL_ENV		"boot_slot"
#define BOOTCTRL_ENV_DTS	"boot_dtb"

#define BOOTCTRL_DFU_ALT_LEN	SZ_1K

struct mtk_bl_ctrl {
	u32 slot_suffix;
	u32 magic;
	u8 version;
	u8 tries[2];
	u32 reserved[3];
	u32 crc32_le;
};

int iot_ab_boot_slot(int type);
void iot_ab_boot_select(void);
void iot_ab_boot_complete(void);

#endif /* __MEDIATEK_IOT_AB_H */

// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/*
 * Copyright (C) 2023 MediaTek Inc.
 * Author: Howard Lin <howard-yh.lin@mediatek.com>
 */

#include <blk.h>
#include <log.h>
#include <part.h>
#include <malloc.h>
#include <common.h>
#include <env.h>
#include <iot_ab.h>

static uint32_t iot_ab_handle_chk(struct mtk_bl_ctrl *bctrl)
{
	return crc32(0, (void *)bctrl, offsetof(typeof(*bctrl), crc32_le));
}

static void iot_ab_handle_init(struct mtk_bl_ctrl *bctrl)
{
	memset(bctrl, 0, sizeof(struct mtk_bl_ctrl));
	bctrl->magic = BOOTCTRL_MAGIC;
	bctrl->version = BOOTCTRL_VERSION;
	bctrl->slot_suffix = PART_BOOT_A;
	bctrl->tries[PART_BOOT_A] = BOOTCTRL_SRC;
	bctrl->tries[PART_BOOT_B] = BOOTCTRL_SRC;
}

static int iot_ab_handle_blctrl(struct blk_desc *dev_desc, struct disk_partition *part_info,
				struct mtk_bl_ctrl *bctrl)
{
	u32 blocks;
	int ret = -1;

	blocks = DIV_ROUND_UP(sizeof(struct mtk_bl_ctrl), part_info->blksz);
	bctrl->crc32_le = iot_ab_handle_chk(bctrl);
	ret = blk_dwrite(dev_desc, part_info->start, blocks, bctrl);
	if (ret < 0) {
		log_err("Could not handle boot control.\n");
		return ret;
	}
	return 0;
}

static int iot_ab_handle_slot(int type, struct blk_desc *dev_desc, struct disk_partition *part_info)
{
	struct mtk_bl_ctrl *bctrl = NULL;
	u32 crc32_le, blocks;
	int ret = -1;

	blocks = DIV_ROUND_UP(sizeof(struct mtk_bl_ctrl), part_info->blksz);
	bctrl = malloc_cache_aligned(blocks * part_info->blksz);
	if (!bctrl)
		return ret;

	ret = blk_dread(dev_desc, part_info->start, blocks, bctrl);
	if (ret < 0) {
		log_err("Could not handle boot control.\n");
		free(bctrl);
		return -1;
	}

	crc32_le = iot_ab_handle_chk(bctrl);
	if (bctrl->magic != BOOTCTRL_MAGIC || bctrl->crc32_le != crc32_le) {
		log_err("Invalid crc, expected %.8x .\n", crc32_le);
		iot_ab_handle_init(bctrl);
		iot_ab_handle_blctrl(dev_desc, part_info, bctrl);
		ret = -1;
		goto exit;
	}

	if (type == BOOTCTRL_SET) {
		bctrl->slot_suffix = (bctrl->slot_suffix == PART_BOOT_A)
							? PART_BOOT_B : PART_BOOT_A;
		bctrl->tries[bctrl->slot_suffix] = BOOTCTRL_SRC;
		iot_ab_handle_blctrl(dev_desc, part_info, bctrl);
	} else if (type == BOOTCTRL_FINISH) {
		bctrl->tries[bctrl->slot_suffix] = BOOTCTRL_BOOT_SUCCESS;
		iot_ab_handle_blctrl(dev_desc, part_info, bctrl);
	}
	ret = bctrl->slot_suffix;

exit:
	free(bctrl);
	return ret;
}

int iot_ab_boot_slot(int type)
{
	struct blk_desc *dev_desc;
	struct disk_partition part_info;
	char slot[2];
	int ret = -1;

	ret = part_get_info_by_dev_and_name_or_num(BOOTCTRL_DEV, BOOTCTRL_PART,
						   &dev_desc, &part_info, false);
	if (ret < 0) {
		log_err("Invalid boot control partition.\n");
		return ret;
	}

	ret = iot_ab_handle_slot(type, dev_desc, &part_info);
	if (ret < 0) {
		log_err("Invalid boot control.\n");
		return ret;
	}

	slot[0] = BOOT_SLOT_NAME(ret);
	slot[1] = '\0';
	env_set(BOOTCTRL_ENV, slot);
	sprintf(slot, "%d", BOOTCTRL_FW_NUM + ret);
	env_set(BOOTCTRL_ENV_DTS, slot);
	return ret;
}

void iot_ab_boot_select(void)
{
	char slot[2];
	int ret = iot_ab_boot_slot(BOOTCTRL_SET);

	if (ret < 0) {
		log_err("Invalid boot select, reset slot.\n");
		ret = 0;
	}

	slot[0] = BOOT_SLOT_NAME(ret);
	slot[1] = '\0';
	env_set(BOOTCTRL_ENV, slot);
	sprintf(slot, "%d", BOOTCTRL_FW_NUM + ret);
	env_set(BOOTCTRL_ENV_DTS, slot);
}

void iot_ab_boot_complete(void)
{
	if (iot_ab_boot_slot(BOOTCTRL_FINISH) < 0)
		log_err("Invalid boot complete.\n");
}


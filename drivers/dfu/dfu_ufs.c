// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 MediaTek Inc.
 * Author: Howard Lin <howard-yh.lin@mediatek.com>
 *
 * Reference: dfu_mmc.c
 * Copyright (C) 2012 Samsung Electronics
 * author: Lukasz Majewski <l.majewski@samsung.com>
 */

#include <common.h>
#include <malloc.h>
#include <blk.h>
#include <errno.h>
#include <dfu.h>

static int dfu_transfer_medium_ufs(enum dfu_op op, struct dfu_entity *dfu,
				   u64 offset, void *buf, long *len)
{
	ulong n, blk_start, blk_count;
	const char *if_name = blk_get_if_type_name(IF_TYPE_SCSI);

	if (dfu->layout != DFU_RAW_ADDR) {
		pr_err("Unsupported %s\n", dfu_get_layout(dfu->layout));
		return  -EINVAL;
	}

	*len = ALIGN(*len, dfu->data.ufs.lba_blk_size);
	blk_start = dfu->data.ufs.lba_start + (u32)lldiv(offset, dfu->data.ufs.lba_blk_size);
	blk_count = *len / dfu->data.ufs.lba_blk_size;

	if (blk_start + blk_count > dfu->data.ufs.lba_start + dfu->data.ufs.lba_size) {
		pr_err("Request exceeds allowed area.\n");
		return -EINVAL;
	}

	if (op == DFU_OP_WRITE) {
		if (blk_show_device(IF_TYPE_SCSI, dfu->data.ufs.device)) {
			pr_err("Request failed.\n");
			return -EINVAL;
		}

		n = blk_write_devnum(IF_TYPE_SCSI, dfu->data.ufs.device, blk_start, blk_count, buf);
		return n == blk_count ? 0 : 1;
	}

	pr_err("Request unsupported.\n");
	return -EINVAL;
}

static int dfu_write_medium_ufs(struct dfu_entity *dfu, u64 offset,
				void *buf, long *len)
{
	return dfu_transfer_medium_ufs(DFU_OP_WRITE, dfu, offset, buf, len);
}

int dfu_get_medium_size_ufs(struct dfu_entity *dfu, u64 *size)
{
	*size = dfu->data.ufs.lba_size;
	return 0;
}

static int dfu_read_medium_ufs(struct dfu_entity *dfu, u64 offset,
			       void *buf, long *len)
{
	return dfu_transfer_medium_ufs(DFU_OP_READ, dfu, offset, buf, len);
}

int dfu_fill_entity_ufs(struct dfu_entity *dfu, char *devstr, char **argv, int argc)
{
	char *s;
	ssize_t second_arg;
	size_t third_arg;

	if (argc < 3) {
		pr_err("Invalid number of arguments.\n");
		return -EINVAL;
	}

	second_arg = simple_strtol(argv[1], &s, 0);
	if (*s)
		return -EINVAL;
	third_arg = simple_strtoul(argv[2], &s, 0);
	if (*s)
		return -EINVAL;

	dfu->dev_type = DFU_DEV_UFS;

	if (!strcmp(argv[0], "raw")) {
		if (argc > 3) {
			if (strcmp(argv[3], "dev")) {
				pr_err("DFU ufs invalid.\n");
				return -EINVAL;
			}
			dfu->data.ufs.device = simple_strtoul(argv[4], NULL, 0);
		}

		const struct blk_desc *desc = blk_get_devnum_by_type(IF_TYPE_SCSI,
									dfu->data.ufs.device);

		dfu->layout = DFU_RAW_ADDR;
		dfu->data.ufs.lba_start		= second_arg;
		dfu->data.ufs.lba_size		= third_arg;
		dfu->data.ufs.lba_blk_size	= desc->blksz;
	} else if (!strcmp(argv[0], "part")) {
		struct disk_partition partinfo;
		const struct blk_desc *desc = blk_get_devnum_by_type(IF_TYPE_SCSI, second_arg);

		if (part_get_info(desc, third_arg, &partinfo)) {
			pr_err("Couldn't find the device.\n");
			return -ENODEV;
		}

		dfu->layout = DFU_RAW_ADDR;
		dfu->data.ufs.lba_start		= partinfo.start;
		dfu->data.ufs.lba_size		= partinfo.size;
		dfu->data.ufs.lba_blk_size	= partinfo.blksz;
		dfu->data.ufs.device = second_arg;
	} else {
		pr_err("Unsupported DFU ufs %s\n", argv[0]);
		return -ENODEV;
	}

	dfu->write_medium = dfu_write_medium_ufs;
	dfu->get_medium_size = dfu_get_medium_size_ufs;
	dfu->read_medium = dfu_read_medium_ufs;
	dfu->inited = 0;
	return 0;
}

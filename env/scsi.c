// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2008-2011 Freescale Semiconductor, Inc.
 */

/* #define DEBUG */

#include <common.h>
#include <asm/global_data.h>

#include <command.h>
#include <env.h>
#include <env_internal.h>
#include <fdtdec.h>
#include <linux/stddef.h>
#include <malloc.h>
#include <memalign.h>
#include <scsi.h>
#include <part.h>
#include <search.h>
#include <errno.h>

#define __STR(X) #X
#define STR(X) __STR(X)

DECLARE_GLOBAL_DATA_PTR;

static inline s64 scsi_offset(int copy)
{
	s64 offset = CONFIG_ENV_OFFSET;

#if defined(CONFIG_ENV_OFFSET_REDUND)
	if (copy)
		offset = CONFIG_ENV_OFFSET_REDUND;
#endif
	return offset;
}

__weak int scsi_get_env_addr(struct blk_desc *desc, int copy, u32 *env_addr)
{
	s64 offset = scsi_offset(copy);

	if (offset < 0)
		offset += desc->lba;

	*env_addr = offset;

	return 0;
}

#if defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_SPL_BUILD)
static inline int write_env(struct blk_desc *desc, unsigned long size,
			    unsigned long offset, const void *buffer)
{
	uint blk_start, blk_cnt, n;

	blk_start	= ALIGN(offset, desc->blksz) / desc->blksz;
	blk_cnt		= ALIGN(size, desc->blksz) / desc->blksz;

	n = blk_dwrite(desc, blk_start, blk_cnt, (u_char *)buffer);

	return (n == blk_cnt) ? 0 : -1;
}

static int env_scsi_save(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(env_t, env_new, 1);
	struct blk_desc *desc;
	u32 offset;
	int	ret, copy = 0;
	int dev = CONFIG_SYS_SCSI_ENV_DEV;

#ifndef CONFIG_DM_SCSI
	scsi_bus_reset(NULL);
#endif
	ret = scsi_scan(false);
	if (ret)
		return ret;

	desc = blk_get_devnum_by_type(IF_TYPE_SCSI, dev);
	if (!desc)
		return -ENODEV;

	ret = env_export(env_new);
	if (ret)
		return ret;

	if (scsi_get_env_addr(desc, 0, &offset))
		return -EIO;

	printf("Writing to %sSCSI device(%d)... ", copy ? "redundant " : "", dev);
	if (write_env(desc, CONFIG_ENV_SIZE, offset, (u_char *)env_new)) {
		puts("failed\n");
		return -EIO;
	}

	return ret;
}

static inline int erase_env(struct blk_desc *desc, unsigned long size,
			    unsigned long offset)
{
	uint blk_start, blk_cnt, n;

	blk_start	= ALIGN(offset, desc->blksz) / desc->blksz;
	blk_cnt		= ALIGN(size, desc->blksz) / desc->blksz;

	n = blk_derase(desc, blk_start, blk_cnt);
	printf("%d blocks erased: %s\n", n, (n == blk_cnt) ? "OK" : "ERROR");

	return (n == blk_cnt) ? 0 : 1;
}

static int env_scsi_erase(void)
{
	struct blk_desc *desc;
	u32 offset;
	int ret;

#ifndef CONFIG_DM_SCSI
	scsi_bus_reset(NULL);
#endif
	ret = scsi_scan(false);
	if (ret)
		return ret;

	desc = blk_get_devnum_by_type(IF_TYPE_SCSI, CONFIG_SYS_SCSI_ENV_DEV);
	if (!desc)
		return -ENODEV;

	if (scsi_get_env_addr(desc, 0, &offset))
		return -EIO;

	return erase_env(desc, CONFIG_ENV_SIZE, offset);
}
#endif /* CONFIG_CMD_SAVEENV && !CONFIG_SPL_BUILD */

static inline int read_env(struct blk_desc *desc, unsigned long size,
			   unsigned long offset, const void *buffer)
{
	uint blk_start, blk_cnt, n;

	blk_start	= ALIGN(offset, desc->blksz) / desc->blksz;
	blk_cnt		= ALIGN(size, desc->blksz) / desc->blksz;

	n = blk_dread(desc, blk_start, blk_cnt, (uchar *)buffer);

	return (n == blk_cnt) ? 0 : -1;
}

static int env_scsi_load(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(char, buf, CONFIG_ENV_SIZE);
	struct blk_desc *desc;
	env_t *ep = NULL;
	u32 offset;
	int ret;

#ifndef CONFIG_DM_SCSI
	scsi_bus_reset(NULL);
#endif
	ret = scsi_scan(false);
	if (ret)
		return ret;

	desc = blk_get_devnum_by_type(IF_TYPE_SCSI, CONFIG_SYS_SCSI_ENV_DEV);
	if (!desc)
		return -ENODEV;

	if (scsi_get_env_addr(desc, 0, &offset))
		return -EIO;

	if (read_env(desc, CONFIG_ENV_SIZE, offset, buf))
		return -EIO;

	ret = env_import(buf, 1, H_EXTERNAL);
	if (!ret) {
		ep = (env_t *)buf;
		gd->env_addr = (ulong)&ep->data;
	}

	return ret;
}

U_BOOT_ENV_LOCATION(scsi) = {
	.location	= ENVL_SCSI,
	ENV_NAME("scsi")
	.load		= env_scsi_load,
#ifndef CONFIG_SPL_BUILD
	.save		= env_save_ptr(env_scsi_save),
	.erase		= ENV_ERASE_PTR(env_scsi_erase)
#endif
};

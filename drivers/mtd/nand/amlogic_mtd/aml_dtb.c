/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
* *
This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* *
This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
* *
You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
* *
Description:
*/

#include <common.h>
#include <environment.h>
#include <nand.h>
#include <asm/io.h>
#include <malloc.h>
#include "aml_mtd.h"

int dtb_erase_blk = -1;

extern int get_partition_from_dts(unsigned char * buffer);

struct aml_nand_chip *aml_chip_dtb = NULL;

int amlnf_dtb_save(u8 *buf, unsigned int len)
{
	struct mtd_info *mtd = &aml_chip_dtb->mtd;
	u8 *dtb_buf = NULL;
	int ret = 0;

	printk("%s: ####\n", __func__);
	if (aml_chip_dtb == NULL) {
		printk("%s: amlnf not init yet!\n", __func__);
		return -EFAULT;
	}
	if (len > aml_chip_dtb->dtbsize) {
		printk("warnning!!! %s: length too much\n", __func__);
		len = aml_chip_dtb->dtbsize;
	}
	dtb_buf = kzalloc(aml_chip_dtb->dtbsize, GFP_KERNEL);
	if (dtb_buf == NULL) {
		printk("%s: malloc failed\n", __func__);
		ret = -1;
		goto exit_err;
	}
	memcpy(dtb_buf, buf, len);
	aml_nand_ext_save_rsv_info(mtd,
		aml_chip_dtb->aml_nanddtb_info, dtb_buf);
exit_err:
	kfree(dtb_buf);
	return ret;
}

int amlnf_dtb_erase(void)
{
	int ret = 0;
	struct mtd_info *mtd = &aml_chip_dtb->mtd;

	printk("%s: ####\n", __func__);
	if (aml_chip_dtb == NULL) {
		printk("%s amlnf not ready yet!\n", __func__);
		return -1;
	}
	ret = aml_nand_ext_erase_rsv_info(mtd, aml_chip_dtb->aml_nanddtb_info);
	if (ret) {
		printk("erase dtb error,%s\n", __func__);
		ret = -EFAULT;
	}

	return ret;
}

int amlnf_dtb_read(u8 *buf, int len)
{
	struct mtd_info *mtd = &aml_chip_dtb->mtd;
	u8 *dtb_buf = NULL;
	int ret = 0;

	printk("%s: ####\n", __func__);
	if (len > aml_chip_dtb->dtbsize) {
		printk("warnning!!! %s dtd length too much\n", __func__);
		len = aml_chip_dtb->dtbsize;
	}
	if (aml_chip_dtb == NULL) {
		memset(buf, 0x0, len);
		printk("%s amlnf not ready yet!\n", __func__);
		return 0;
	}
	dtb_buf = kzalloc(aml_chip_dtb->dtbsize, GFP_KERNEL);
	if (dtb_buf == NULL) {
		printk("%s: malloc failed\n", __func__);
		ret = -1;
		goto exit_err;
	}
	memset(dtb_buf, 0, aml_chip_dtb->dtbsize);
	ret = aml_nand_ext_read_rsv_info(mtd,
		aml_chip_dtb->aml_nanddtb_info, 0, dtb_buf);
	memcpy(buf, dtb_buf, len);
exit_err:
	kfree(dtb_buf);
	return ret;
}

int amlnf_dtb_init(struct aml_nand_chip *aml_chip)
{
	aml_chip_dtb = aml_chip;
	return 0;
}



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

#include "../include/phynand.h"


static struct amlnand_chip *aml_chip_key = NULL;

/*
 * This funcion reads the u-boot keys.
 */
int amlnf_key_read(u8 * buf, int len, uint32_t *actual_lenth)
{
	struct amlnand_chip * aml_chip = aml_chip_key;
	struct nand_menson_key *key_ptr = NULL;
	u32 keysize = aml_chip->keysize;
	int error = 0;

	*actual_lenth = keysize;

	if (aml_chip == NULL) {
		printk("%s(): amlnf key not ready yet!", __func__);
		return -EFAULT;
	}

	if (len > keysize) {
		/*
		No return here! keep consistent, should memset zero
		for the rest.
		*/
		printk("%s key data len too much\n",__func__);
		memset(buf + keysize, 0 , len - keysize);
		//return -EFAULT;
	}

	key_ptr = kzalloc(aml_chip->keysize + sizeof(u32), GFP_KERNEL);
	if (key_ptr == NULL)
		return -ENOMEM;

	error = amlnand_read_info_by_name(aml_chip,
		(u8 *)&(aml_chip->nand_key),
		(u8 *)key_ptr,
		(u8*)KEY_INFO_HEAD_MAGIC,
		aml_chip->keysize);
	if (error) {
		printk("%s: read key error\n",__func__);
		error = -EFAULT;
		goto exit;
	}

	memcpy(buf, key_ptr->data, keysize);

exit:
	kfree(key_ptr);
	return 0;
}

/*
 * This funcion write the keys.
 */
int amlnf_key_write(u8 *buf, int len, uint32_t *actual_lenth)
{
	struct amlnand_chip * aml_chip = aml_chip_key;
	struct nand_menson_key *key_ptr = NULL;
	u32 keysize = aml_chip->keysize;
	int error = 0;
	struct nand_flash *flash = &aml_chip_key->flash;

	if (aml_chip == NULL) {
		printk("%s(): amlnf key not ready yet!", __func__);
		return -EFAULT;
	}

	if (len > keysize) {
		/*
		No return here! keep consistent, should memset zero
		for the rest.
		*/
		printk("key data len too much,%s\n",__func__);
		memset(buf + keysize, 0 , len - keysize);
		//return -EFAULT;
	}

	key_ptr = kzalloc(aml_chip->keysize + flash->pagesize, GFP_KERNEL);
	if (key_ptr == NULL)
		return -ENOMEM;

	memcpy(key_ptr->data, buf, keysize);

	error = amlnand_save_info_by_name(aml_chip,
		(u8 *)&(aml_chip->nand_key),
		(u8 *)key_ptr,
		(u8 *)KEY_INFO_HEAD_MAGIC,
		aml_chip->keysize);
	if (error) {
		printk("save key error,%s\n",__func__);
		error = -EFAULT;
		goto exit;
	}
exit:
	kfree(key_ptr);
	return error;
}

int amlnf_key_erase(void)
{
	int ret = 0;
	if (aml_chip_key == NULL) {
		printk("%s amlnf not ready yet!\n", __func__);
		return -1;
	}
	ret = amlnand_erase_info_by_name(aml_chip_key,
		(u8 *)&(aml_chip_key->nand_key),
		(u8 *)KEY_INFO_HEAD_MAGIC);
	if (ret) {
		printk("%s erase key error\n", __func__);
		ret = -EFAULT;
	}
	return ret;
}


int aml_key_init(struct amlnand_chip *aml_chip)
{
	int ret = 0;
	struct nand_menson_key *key_ptr = NULL;

	/* avoid null */
	aml_chip_key = aml_chip;

	key_ptr = aml_nand_malloc(aml_chip->keysize +sizeof(u32));
	if (key_ptr == NULL) {
		printk("nand malloc for key_ptr failed");
		ret = -1;
		goto exit_error0;
	}
	memset(key_ptr, 0x0, aml_chip->keysize + sizeof(u32));
	printk("%s probe.\n", __func__);

	ret = amlnand_info_init(aml_chip,
		(u8 *)&(aml_chip->nand_key),
		(u8 *)key_ptr,(u8 *)KEY_INFO_HEAD_MAGIC,
		aml_chip->keysize);
	if (ret < 0) {
		printk("invalid nand key\n");
	}
exit_error0:
	if (key_ptr) {
		aml_nand_free(key_ptr);
		key_ptr =NULL;
	}
	return ret;
}


int aml_nand_update_key(struct amlnand_chip * aml_chip, char *key_ptr)
{
	int ret = 0;
	int malloc_flag = 0;
	char *key_buf = NULL;
	struct nand_flash *flash = &aml_chip->flash;

	if (key_buf == NULL) {
		key_buf = kzalloc(aml_chip->keysize + flash->pagesize, GFP_KERNEL);
		malloc_flag = 1;
		if (key_buf == NULL)
			return -ENOMEM;
		memset(key_buf, 0, aml_chip->keysize);
		ret = amlnand_read_info_by_name(aml_chip,
			(u8 *)&(aml_chip->nand_key),
			(u8 *)key_buf,
			(u8 *)KEY_INFO_HEAD_MAGIC,
			aml_chip->keysize);
		if (ret) {
			printk("%s: read key error\n", __func__);
			ret = -EFAULT;
			goto exit;
		}
	} else {
		key_buf = key_ptr;
	}

	printk("amlnf_key : type %d, valid %d, flag %d, blk %d, page %d\n",
		aml_chip->nand_key.arg_type,aml_chip->nand_key.arg_valid,
		aml_chip->nand_key.update_flag,aml_chip->nand_key.valid_blk_addr,
		aml_chip->nand_key.valid_page_addr);

	ret = amlnand_save_info_by_name( aml_chip,
		(u8 *)&(aml_chip->nand_key),
		(u8 *)key_buf,
		(u8 *)KEY_INFO_HEAD_MAGIC,
		aml_chip->keysize);
	if (ret < 0) {
		printk("%s : save key info failed\n", __func__);
	}

exit:
	if (malloc_flag &&(key_buf)) {
		kfree(key_buf);
		key_buf = NULL;
	}
	return 0;
}

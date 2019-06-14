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

#ifndef __STOARGE_IF_H__
#define __STOARGE_IF_H__


/***
upgrade_read_ops:

partition_name: env / logo / recovery /boot / system /cache /media

***/
int store_read_ops(unsigned char *partition_name,unsigned char * buf, uint64_t off, uint64_t size);


/***
upgrade_write_ops:

partition_name: env / logo / recovery /boot / system /cache /media

***/
int store_write_ops(unsigned char *partition_name,unsigned char * buf,uint64_t off, uint64_t size);


/***
upgrade_write_ops:

partition_name: env / logo / recovery /boot / system /cache /media

***/
int store_get_partititon_size(unsigned char *partition_name, uint64_t *size);


/***
upgrade_erase_ops:

partition_name: boot / data

flag = 0; indicate erase partition ;
flag = 1; indicate scurb whole nand;

***/
int store_erase_ops(unsigned char *par_name, uint64_t off, uint64_t size, unsigned char flag);

/***
bootloader:
***/
int store_boot_read(unsigned char * buf, uint64_t off, uint64_t size);

int store_boot_write(unsigned char * buf,uint64_t off, uint64_t size);

int store_init(unsigned  flag);

int store_exit(void);

//dtb read/write
//@rwFlag: 0---read, 1---write, 2---iread
int store_dtb_rw(void* buf, unsigned dtbSz, int rwFlag);

//key read/write
int store_key_read(uint8_t * buffer,
			uint32_t length, uint32_t *actual_lenth);
int store_key_write(uint8_t * buffer,
			uint32_t length, uint32_t *actual_lenth);

#endif//ifndef __STOARGE_IF_H__


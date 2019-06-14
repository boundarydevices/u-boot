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

#ifndef __UNIFYKEY_H__
#define __UNIFYKEY_H__

#include "ini_config.h"

#ifdef __cplusplus
extern "C" {
#endif

int readUKeyData(const char *key_name, unsigned char data_buf[], int rd_size);
int writeUKeyData(const char *key_name, unsigned char data_buf[], int wr_size);

#if (defined CC_UBOOT_RW_SIMULATE)

unsigned int crc32(unsigned int crc, const unsigned char *ptr, int buf_len);
int key_unify_write(const char* keyname, const void* keydata, const unsigned datalen);
int key_unify_read(const char* keyname, void* keydata, const unsigned bufLen);
int key_unify_query_size(const char* keyname, ssize_t* keysize);
int key_unify_query_exist(const char* keyname, int *exist);
int key_unify_query_secure(const char* keyname, int *isSecure);

#endif

#ifdef __cplusplus
}
#endif

#endif //__UNIFYKEY_H__

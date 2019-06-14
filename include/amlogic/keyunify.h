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

#ifndef __KEYUNIFY_H__
#define __KEYUNIFY_H__

//APIs of key_unify*: unify interfaces for nandkeys/emmckeys/efuse keys

int key_unify_init(const char* seednum, const char* dtbaddr);

int key_unify_uninit(void);

//keyType: user type to define how to parse/check the key value before burn to target
int key_unify_write(const char* keyname, const void* keydata, const unsigned datalen);

int key_unify_read(const char* keyname, void* keydata, const unsigned bufLen);

int key_unify_query_size(const char* keyname, ssize_t* keysize);

int key_unify_query_exist(const char* keyname, int* exist);

int key_unify_query_secure(const char* keyname, int* isSecure);

int key_unify_query_canOverWrite(const char* keyname, int* canOverWrite);

//Does the key configured in dts
int key_unify_query_key_has_configure(const char* keyname);

//Another APIs with APP concers, like special flower hdcp2
//These APIs are based on key_unify_*
//
int key_manage_init(const char* seednum, const char* dtbaddr);
int key_manage_exit(void);

int key_manage_write(const char* keyname, const void* keydata, const unsigned datalen);

int key_manage_read(const char* keyname, void* keydata, const unsigned bufLen);

int key_manage_query_size(const char* keyname, ssize_t* keysize);

int key_manage_query_exist(const char* keyname, int* exist);

int key_manage_query_secure(const char* keyname, int* isSecure);

int key_manage_query_canOverWrite(const char* keyname, int* canOverWrite);

#endif// #ifndef __KEYUNIFY_H__


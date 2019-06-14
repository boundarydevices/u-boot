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

#ifndef __STORAGEKEY_H__
#define __STORAGEKEY_H__


/* storage key interface */
/**
 *1.amlkey_init: init storage key
 * return ok 0, fail not 0
 */
extern int32_t amlkey_init(uint8_t *seed, uint32_t len);

/**
 *2. amlkey_isexsit: query key, already programmed ?
 * return: exsit 1, non 0
 */
extern int32_t amlkey_isexsit(const uint8_t * name);

/**
 * 3. query if the key is secure
 * return secure 1, non 0;
 */
extern int32_t amlkey_issecure(const uint8_t * name);

/**
 * 4. amlkey_size: actual bytes of key value
 *  return actual size.
 */
extern ssize_t amlkey_size(const uint8_t *name);

/**
 *5. amlkey_read: read non-secure key in bytes.
 * return actual size read back; 0 means read failed!
 */
extern ssize_t amlkey_read(const uint8_t *name,
				uint8_t *buffer, uint32_t len);

/**
 * 6. amlkey_write: write secure/non-secure key in bytes.
 * return actual size write down. 0 means write failed!
 */
extern ssize_t amlkey_write(const uint8_t *name,
				uint8_t *buffer, uint32_t len, uint32_t secure);

/**
 * 7. get the hash value of programmed secure key | 32bytes length, sha256
 * return success 0, fail -1
 */
extern int32_t amlkey_hash_4_secure(const uint8_t * name, uint8_t * hash);

/**
 * 8. amlkey_del: del key by name
 * return success 0, fail non-0.
 */
extern int32_t amlkey_del(const uint8_t * name);


#endif /* __STORAGEKEY_H__ */

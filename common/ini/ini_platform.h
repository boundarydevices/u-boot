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

#ifndef __INI_PLATFORM_H__
#define __INI_PLATFORM_H__

#if (defined CC_COMPILE_IN_UBOOT)
    #define strtoul simple_strtoul
    #define strtol simple_strtol
#endif

#ifdef __cplusplus
extern "C" {
#endif

//c basic lib
char* plat_strtok_r(char *str, const char *delim, char **saveptr);

//File functions
int iniIsFileExist(const char *file_path);
int iniGetFileSize(const char *file_path);
int iniReadFileToBuffer(const char *file_path, int offset, int rd_size, unsigned char data_buf[]);

#ifdef __cplusplus
}
#endif

#endif //__INI_PLATFORM_H__

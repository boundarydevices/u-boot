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

#ifndef __INI_CONFIG_H__
#define __INI_CONFIG_H__

#if (!defined(CC_COMPILE_IN_PC) && !defined(CC_COMPILE_IN_ANDROID))
    #define CC_COMPILE_IN_UBOOT
    #define CC_INI_IO_USE_UNIFY_KEY
#endif

#if (defined CC_COMPILE_IN_PC)
    #define CC_UBOOT_RW_SIMULATE
#endif

#if (defined CC_COMPILE_IN_PC || defined CC_COMPILE_IN_ANDROID)
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <ctype.h>
    #include <errno.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/stat.h>
#elif (defined CC_COMPILE_IN_UBOOT)
    #include <common.h>
    #include <command.h>
    #include <environment.h>
    #include <linux/ctype.h>
    #include <linux/string.h>
    #include <malloc.h>
    #include <amlogic/keyunify.h>
    #include <ext4fs.h>
    #include <linux/stat.h>
    #include <malloc.h>
    #include <fs.h>
    #include <emmc_partitions.h>
#endif

#endif //__INI_CONFIG_H__

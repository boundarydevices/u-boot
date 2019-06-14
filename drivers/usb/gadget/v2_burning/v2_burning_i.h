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


#ifndef __V2_BURNING_I_H__
#define __V2_BURNING_I_H__

#include <config.h>
#include <common.h>
#include <environment.h>
#include <asm/string.h>
#include <asm-generic/errno.h>
#include <asm/byteorder.h>
#include <malloc.h>
#include <u-boot/sha1.h>

#include <amlogic/aml_v2_burning.h>
//#include <asm/arch/reboot.h>
#include <asm/arch/romboot.h>
//#include <amlogic/aml_lcd.h>
#include <amlogic/storage_if.h>
#include "v2_common/sparse_format.h"
#include "v2_common/optimus_download.h"
#include "v2_common/amlImage_if.h"
#include "v2_common/optimus_progress_ui.h"

extern int cli_simple_parse_line(char *line, char *argv[]);

#endif//ifndef __V2_BURNING_I_H__


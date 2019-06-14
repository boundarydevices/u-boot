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


#include "ini_config.h"

#include "ini_log.h"

#if (defined CC_COMPILE_IN_PC || defined CC_COMPILE_IN_UBOOT)
static int gLogLevel = INI_LOG_DEFAULT;

int ini_set_log_level(int log_level) {
    int tmp_level = gLogLevel;
    gLogLevel = log_level;
    return tmp_level;
}

int ini_get_log_level(void) {
    return gLogLevel;
}
#endif

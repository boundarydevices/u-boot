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


#ifdef CONFIG_MDUMP_COMPRESS
#include "ramdump.h"

struct ram_compress_full __ramdump_data = {
	.store_phy_addr = (void *)CONFIG_COMPRESSED_DATA_ADDR,
	.full_memsize   = CONFIG_DDR_TOTAL_SIZE,
	.section_count  = CONFIG_COMPRESS_SECTION,
	.sections       = {
		{
			/* memory afer compressed data address */
			.phy_addr      = (void *)CONFIG_COMPRESSED_DATA_ADDR,
			.section_size  = CONFIG_DDR_TOTAL_SIZE -
					 CONFIG_COMPRESSED_DATA_ADDR,
			.section_index = 5,
			.compress_type = RAM_COMPRESS_NORMAL,
		},
		{
			/* memory in reserved bottom */
			.phy_addr      = (void *)CONFIG_COMPRESS_START_ADDR,
			.section_size  = CONFIG_1ST_RESERVED_SIZE,
			.section_index = 1,
			.compress_type = RAM_COMPRESS_SET,
			.set_value     = 0x0,
		},
		{
			/* memory before bl2 */
			.phy_addr      = (void *)CONFIG_1ST_RESERVED_END,
			.section_size  = CONFIG_BL2_IGNORE_ADDR -
					 CONFIG_1ST_RESERVED_END,
			.section_index = 2,
			.compress_type = RAM_COMPRESS_NORMAL,
		},
		{
			/* memory in reserved bl2 */
			.phy_addr      = (void *)CONFIG_BL2_IGNORE_ADDR,
			.section_size  = CONFIG_BL2_IGNORE_SIZE,
			.section_index = 3,
			.compress_type = RAM_COMPRESS_SET,
			.set_value     = 0x0,
		},
		{
			/* segment 4: normal compress */
			.phy_addr      = (void *)CONFIG_SEG4_ADDR,
			.section_size  = CONFIG_COMPRESSED_DATA_ADDR -
					 CONFIG_SEG4_ADDR,
			.section_index = 4,
			.compress_type = RAM_COMPRESS_NORMAL,
		}
	},
};
#endif /* CONFIG_MDUMP_COMPRESS */


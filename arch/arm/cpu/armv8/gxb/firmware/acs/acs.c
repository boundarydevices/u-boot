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

#include <asm/arch/acs.h>
#include <asm/arch/timing.h>
#include "timing.c"

//main acs struct
struct acs_setting __acs_set={
					.acs_magic		= "acs__",
					.chip_type		= 0x24,
					.version 		= 1,
					.acs_set_length	= sizeof(__acs_set),

					.ddr_magic		= "ddrs_",
					.ddr_set_version= 3, //2016.01.15 update to v2, add dqs gate tuning
										//2016.03.21 update to v3, add lpddr3 support
					.ddr_set_length	= sizeof(ddr_set_t),
					.ddr_set_addr	= (unsigned long)(&__ddr_setting),

					.ddrt_magic		= "ddrt_",
					.ddrt_set_version= 2, //2016.03.21 update to v2, add lpddr3 support
					.ddrt_set_length= sizeof(__ddr_timming),
					.ddrt_set_addr	= (unsigned long)(&__ddr_timming),

					.pll_magic		= "pll__",
					.pll_set_version= 1,
					.pll_set_length	= sizeof(__pll_setting),
					.pll_set_addr	= (unsigned long)(&__pll_setting),
};

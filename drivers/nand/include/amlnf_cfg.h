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

#ifndef __AML_NF_CFG_H__
#define __AML_NF_CFG_H__

/*************stage***************/
#define	AML_NAND_UBOOT	//For uboot
#define EXPORT_SYMBOL(...)
/**************PHY****************/
#define	AML_SLC_NAND_SUPPORT
#define	AML_MLC_NAND_SUPPORT

#define __DEBUG_L04__				(1)

#define	AML_NAND_DBG				(0)
#define	AML_CFG_INSIDE_PARTTBL		(0)
#define AML_CFG_2PLANE_READ_EN		(1)
/* support nand with readretry&e-slc */
#define	AML_CFG_NEW_NAND_SUPPORT	(1)
/* new oob mode */
#define AML_CFG_NEWOOB_EN			(1)
/* store dtd in rsv area! */
#define AML_CFG_DTB_RSV_EN			(1)
/* store key in rsv area */
#define AML_CFG_KEY_RSV_EN			(1)

#define NAND_ADJUST_PART_TABLE

#ifdef NAND_ADJUST_PART_TABLE
#define	ADJUST_BLOCK_NUM	4
#else
#define	ADJUST_BLOCK_NUM	0
#endif

/* ASYNC = 0x0   NVDDR_1= 0x10     NVDDR_2 = 0x20 */
#define NAND_OPMODE_ASYNC               0x0
#define NAND_OPMODE_NVDDR1              0x10
#define NAND_OPMODE_NVDDR2              0x20
#if 1
#define NAND_ONFI_MODE_OP               (NAND_OPMODE_ASYNC)
#else
#define NAND_ONFI_MODE_OP               (NAND_OPMODE_NVDDR1)
#endif
/*do not use rb irq under uboot*/
/* #define AML_NAND_RB_IRQ */

/*
#define AML_NAND_DMA_POLLING
*/

extern  int is_phydev_off_adjust(void);
extern  int get_adjust_block_num(void);

#endif //__AML_NF_CFG_H__

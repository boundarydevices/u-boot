/*
 * Copyright 2021 Boundary Devices
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
/* ------------------------------------------------------------------- */

struct phy_clk_priv {
	struct clk	clk;
	struct clk	parent_clk;
	unsigned long	frequency;
	void __iomem *regs;
};
#define to_clk_phy_clk(_clk) container_of(_clk, struct phy_clk_priv, clk)

#define PHY_PLL_REGS_NUM 48

struct phy_config {
	u32	clk_rate;
	u8 regs[PHY_PLL_REGS_NUM];
};

const struct phy_config samsung_phy_pll_cfg[] = {
	{	22250000, {
			0x00, 0xD1, 0x4B, 0xF1, 0x89, 0x88, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x15, 0x25, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		23750000, {
			0x00, 0xD1, 0x50, 0xF1, 0x86, 0x85, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x03, 0x25, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	},{
		24000000, {
			0x00, 0xD1, 0x50, 0xF0, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x01, 0x25, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	},{
		24024000, {
			0x00, 0xD1, 0x50, 0xF1, 0x99, 0x02, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x00, 0x25, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		25175000, {
			0x00, 0xD1, 0x54, 0xFC, 0xCC, 0x91, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xF5, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		25200000, {
			0x00, 0xD1, 0x54, 0xF0, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xF4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		26750000, {
			0x00, 0xD1, 0x5A, 0xF2, 0x89, 0x88, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xE6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		27000000, {
			0x00, 0xD1, 0x5A, 0xF0, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		27027000, {
			0x00, 0xD1, 0x5A, 0xF2, 0xFD, 0x0C, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		29500000, {
			0x00, 0xD1, 0x62, 0xF4, 0x95, 0x08, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xD1, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		30750000, {
			0x00, 0xD1, 0x66, 0xF4, 0x82, 0x01, 0x88, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xC8, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		30888000, {
			0x00, 0xD1, 0x66, 0xF4, 0x99, 0x18, 0x88, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xC7, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		33750000, {
			0x00, 0xD1, 0x70, 0xF4, 0x82, 0x01, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xB7, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		35000000, {
			0x00, 0xD1, 0x58, 0xB8, 0x8B, 0x88, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xB0, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8B, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		36000000, {
			0x00, 0xD1, 0x5A, 0xB0, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xAB, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8B, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		36036000, {
			0x00, 0xD1, 0x5A, 0xB2, 0xFD, 0x0C, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xAB, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8B, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		40000000, {
			0x00, 0xD1, 0x64, 0xB0, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x9A, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8B, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		43200000, {
			0x00, 0xD1, 0x5A, 0x90, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x8F, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x89, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		43243200, {
			0x00, 0xD1, 0x5A, 0x92, 0xFD, 0x0C, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x8F, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x89, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		44500000, {
			0x00, 0xD1, 0x5C, 0x92, 0x98, 0x11, 0x84, 0x41,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x8B, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x89, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		47000000, {
			0x00, 0xD1, 0x62, 0x94, 0x95, 0x82, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x83, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x89, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		47500000, {
			0x00, 0xD1, 0x63, 0x96, 0xA1, 0x82, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0x82, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x89, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		50349650, {
			0x00, 0xD1, 0x54, 0x7C, 0xC3, 0x8F, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xF5, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		50400000, {
			0x00, 0xD1, 0x54, 0x70, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xF4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		53250000, {
			0x00, 0xD1, 0x58, 0x72, 0x84, 0x03, 0x82, 0x41,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xE7, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		53500000, {
			0x00, 0xD1, 0x5A, 0x72, 0x89, 0x88, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xE6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		54000000, {
			0x00, 0xD1, 0x5A, 0x70, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		54054000, {
			0x00, 0xD1, 0x5A, 0x72, 0xFD, 0x0C, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		59000000, {
			0x00, 0xD1, 0x62, 0x74, 0x95, 0x08, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xD1, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		59340659, {
			0x00, 0xD1, 0x62, 0x74, 0xDB, 0x52, 0x88, 0x47,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xD0, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		59400000, {
			0x00, 0xD1, 0x63, 0x70, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xCF, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		61500000, {
			0x00, 0xD1, 0x66, 0x74, 0x82, 0x01, 0x88, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xC8, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		63500000, {
			0x00, 0xD1, 0x69, 0x74, 0x89, 0x08, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xC2, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x87, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		67500000, {
			0x00, 0xD1, 0x54, 0x52, 0x87, 0x03, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xB7, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		70000000, {
			0x00, 0xD1, 0x58, 0x58, 0x8B, 0x88, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xB0, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		72000000, {
			0x00, 0xD1, 0x5A, 0x50, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xAB, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		72072000, {
			0x00, 0xD1, 0x5A, 0x52, 0xFD, 0x0C, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xAB, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		74176000, {
			0x00, 0xD1, 0x5D, 0x58, 0xDB, 0xA2, 0x88, 0x41,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xA6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		74250000, {
			0x00, 0xD1, 0x5C, 0x52, 0x90, 0x0D, 0x84, 0x41,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xA6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		78500000, {
			0x00, 0xD1, 0x62, 0x54, 0x87, 0x01, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x9D, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		80000000, {
			0x00, 0xD1, 0x64, 0x50, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x9A, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		82000000, {
			0x00, 0xD1, 0x66, 0x54, 0x82, 0x01, 0x88, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x96, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		82500000, {
			0x00, 0xD1, 0x67, 0x54, 0x88, 0x01, 0x90, 0x49,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x95, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		89000000, {
			0x00, 0xD1, 0x70, 0x54, 0x84, 0x83, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x8B, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		90000000, {
			0x00, 0xD1, 0x70, 0x54, 0x82, 0x01, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x89, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		94000000, {
			0x00, 0xD1, 0x4E, 0x32, 0xA7, 0x10, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x83, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		95000000, {
			0x00, 0xD1, 0x50, 0x31, 0x86, 0x85, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x82, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		98901099, {
			0x00, 0xD1, 0x52, 0x3A, 0xDB, 0x4C, 0x88, 0x47,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x7D, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		99000000, {
			0x00, 0xD1, 0x52, 0x32, 0x82, 0x01, 0x88, 0x47,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0x7D, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		100699300, {
			0x00, 0xD1, 0x54, 0x3C, 0xC3, 0x8F, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xF5, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		100800000, {
			0x00, 0xD1, 0x54, 0x30, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xF4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		102500000, {
			0x00, 0xD1, 0x55, 0x32, 0x8C, 0x05, 0x90, 0x4B,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xF0, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		104750000, {
			0x00, 0xD1, 0x57, 0x32, 0x98, 0x07, 0x90, 0x49,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xEB, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		106500000, {
			0x00, 0xD1, 0x58, 0x32, 0x84, 0x03, 0x82, 0x41,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xE7, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		107000000, {
			0x00, 0xD1, 0x5A, 0x32, 0x89, 0x88, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xE6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		108000000, {
			0x00, 0xD1, 0x5A, 0x30, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		108108000, {
			0x00, 0xD1, 0x5A, 0x32, 0xFD, 0x0C, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		118000000, {
			0x00, 0xD1, 0x62, 0x34, 0x95, 0x08, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xD1, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		118800000, {
			0x00, 0xD1, 0x63, 0x30, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xCF, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		123000000, {
			0x00, 0xD1, 0x66, 0x34, 0x82, 0x01, 0x88, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xC8, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		127000000, {
			0x00, 0xD1, 0x69, 0x34, 0x89, 0x08, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xC2, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		135000000, {
			0x00, 0xD1, 0x70, 0x34, 0x82, 0x01, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xB7, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		135580000, {
			0x00, 0xD1, 0x71, 0x39, 0xE9, 0x82, 0x9C, 0x5B,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xB6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		137520000, {
			0x00, 0xD1, 0x72, 0x38, 0x99, 0x10, 0x85, 0x41,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xB3, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		138750000, {
			0x00, 0xD1, 0x73, 0x35, 0x88, 0x05, 0x90, 0x4D,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xB2, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		140000000, {
			0x00, 0xD1, 0x75, 0x36, 0xA7, 0x90, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xB0, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		144000000, {
			0x00, 0xD1, 0x78, 0x30, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xAB, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		148352000, {
			0x00, 0xD1, 0x7B, 0x35, 0xDB, 0x39, 0x90, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xA6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		148500000, {
			0x00, 0xD1, 0x7B, 0x35, 0x84, 0x03, 0x90, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xA6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x83, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		154000000, {
			0x00, 0xD1, 0x40, 0x18, 0x83, 0x01, 0x00, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0xA0, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		157000000, {
			0x00, 0xD1, 0x41, 0x11, 0xA7, 0x14, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x9D, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		160000000, {
			0x00, 0xD1, 0x42, 0x12, 0xA1, 0x20, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x9A, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		162000000, {
			0x00, 0xD1, 0x43, 0x18, 0x8B, 0x08, 0x96, 0x55,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x98, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		164000000, {
			0x00, 0xD1, 0x45, 0x11, 0x83, 0x82, 0x90, 0x4B,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x96, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		165000000, {
			0x00, 0xD1, 0x45, 0x11, 0x84, 0x81, 0x90, 0x4B,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x95, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		180000000, {
			0x00, 0xD1, 0x4B, 0x10, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x89, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		185625000, {
			0x00, 0xD1, 0x4E, 0x12, 0x9A, 0x95, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x85, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		188000000, {
			0x00, 0xD1, 0x4E, 0x12, 0xA7, 0x10, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x83, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		198000000, {
			0x00, 0xD1, 0x52, 0x12, 0x82, 0x01, 0x88, 0x47,
			0x4F, 0x30, 0x33, 0x65, 0x20, 0x7D, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		205000000, {
			0x00, 0xD1, 0x55, 0x12, 0x8C, 0x05, 0x90, 0x4B,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xF0, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		209500000, {
			0x00, 0xD1, 0x57, 0x12, 0x98, 0x07, 0x90, 0x49,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xEB, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		213000000, {
			0x00, 0xD1, 0x58, 0x12, 0x84, 0x03, 0x82, 0x41,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xE7, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		216000000, {
			0x00, 0xD1, 0x5A, 0x10, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		216216000, {
			0x00, 0xD1, 0x5A, 0x12, 0xFD, 0x0C, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		237600000, {
			0x00, 0xD1, 0x63, 0x10, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xCF, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		254000000, {
			0x00, 0xD1, 0x69, 0x14, 0x89, 0x08, 0x80, 0x40,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xC2, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		277500000, {
			0x00, 0xD1, 0x73, 0x15, 0x88, 0x05, 0x90, 0x4D,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xB2, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		288000000, {
			0x00, 0xD1, 0x78, 0x10, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xAB, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		297000000, {
			0x00, 0xD1, 0x7B, 0x15, 0x84, 0x03, 0x90, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xA6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, { /* sentinel */ },
};

static unsigned long phy_clk_recalc_rate(struct clk *clk)
{
	struct phy_clk_priv *clk_priv = dev_get_priv(clk->dev);

	debug("%s: %p(%s) %p: %lu\n", __func__, clk->dev, clk->dev->name, clk_priv, clk_priv->frequency);
	return clk_priv->frequency;
}

#define PHY_REGS_84 0x84

#define FIX_DA 0x2
#define MODE_SET_DONE 0x80

static unsigned long phy_clk_set_rate(struct clk *clk, unsigned long rate)
{
	struct phy_clk_priv *clk_priv = dev_get_priv(clk->dev);
	const struct phy_config *phy_cfg = samsung_phy_pll_cfg;
	int i;

	debug("%s: %p %lu\n", __func__, clk_priv, rate);

	for (; phy_cfg->clk_rate != 0; phy_cfg++)
		if (phy_cfg->clk_rate >= rate)
			break;

	if (phy_cfg->clk_rate == 0)
		return -EINVAL;
	clk_priv->frequency = rate;

	/* HDMI PHY init */
	writeb(FIX_DA, clk_priv->regs + PHY_REGS_84);

	for (i = 0; i < PHY_PLL_REGS_NUM; i++)
		writeb(phy_cfg->regs[i], clk_priv->regs + i * 4);

	writeb(FIX_DA | MODE_SET_DONE , clk_priv->regs + PHY_REGS_84);

	/* Wait for PHY PLL lock */
	mdelay(20);

	return 0;
}

static const char strclk[] = "hdmi_phy";
static const char strclk_parent[] = "hdmi_glb_24m";

static int phy_clk_probe(struct udevice *dev)
{
	struct phy_clk_priv *clk_priv = dev_get_priv(dev);
	const char* clk_parent_name = strclk_parent;
	const char* clk_name = strclk;
	void __iomem *regs;
	const char* name;
	struct clk *c;
	int ret;

	debug("%s\n", __func__);
	regs = (void *)dev_read_addr(dev);

	clk_priv->regs = regs;

	/* optional override of the clockname */
	name = dev_read_string(dev, "clock-output-names");
	if (name)
		clk_name = name;
	dev->name = name;

	ret = clk_get_by_name(dev, "parent", &clk_priv->parent_clk);
	if (!ret) {
		ret = clk_get_by_id(clk_priv->parent_clk.id, &c);
		debug("%s: b ret=%d id=%ld\n", __func__, ret, clk_priv->parent_clk.id);
		if (!ret) {
			clk_parent_name = c->dev->name;
		}
	}
	dev->uclass_priv = &clk_priv->clk;
	clk_priv->clk.dev = dev;
	if (!ret) {
		debug("%s: %s %s\n", __func__, clk_name, clk_parent_name);
		list_del(&dev->sibling_node);
		list_add_tail(&dev->sibling_node, &c->dev->child_head);
		dev->parent = c->dev;
	}
	return 0;
}

static struct clk_ops phy_clk_ops = {
	.set_rate = phy_clk_set_rate,
	.get_rate = phy_clk_recalc_rate,
};

static const struct udevice_id phy_clk_ids[] = {
	{ .compatible = "fsl,samsung-hdmi-phy-clk" },
	{ }
};

U_BOOT_DRIVER(samsung_hdmi_phy_clk) = {
	.name = "samsung-hdmi-phy-clk",
	.id = UCLASS_CLK,
	.of_match = phy_clk_ids,
	.ops = &phy_clk_ops,
	.probe = phy_clk_probe,
	.priv_auto_alloc_size = sizeof(struct phy_clk_priv),
};
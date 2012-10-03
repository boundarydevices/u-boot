/*
 * Copyright (C) 2012 Boundary Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */
#ifndef __ASM_ARCH_IMX_MKIMAGE_H__
#define __ASM_ARCH_IMX_MKIMAGE_H__

#define IRAM_FREE_START		0x00907000
#define MMDC_P0			0x021b0000
#define MMDC_P1			0x021b4000
#define IOMUXC_BASE		0x020e0000
#define CCM_BASE		0x020c4000

#define IOMUXC_GPR4		(IOMUXC_BASE + 0x010)
#define IOMUXC_GPR6		(IOMUXC_BASE + 0x018)
#define IOMUXC_GPR7		(IOMUXC_BASE + 0x01c)

/* mx6 duallite and solo have same offsets */

#define IOM_DRAM_DQM0		MA(0x5ac, 0x470, 0x0)
#define IOM_DRAM_DQM1		MA(0x5b4, 0x474, 0x0)
#define IOM_DRAM_DQM2		MA(0x528, 0x478, 0x0)
#define IOM_DRAM_DQM3		MA(0x520, 0x47c, 0x0)
#define IOM_DRAM_DQM4		MA(0x514, 0x480, 0x0)
#define IOM_DRAM_DQM5		MA(0x510, 0x484, 0x0)
#define IOM_DRAM_DQM6		MA(0x5bc, 0x488, 0x0)
#define IOM_DRAM_DQM7		MA(0x5c4, 0x48c, 0x0)

#define IOM_DRAM_CAS		MA(0x56c, 0x464, 0x0)
#define IOM_DRAM_RAS		MA(0x578, 0x490, 0x0)
#define IOM_DRAM_RESET		MA(0x57c, 0x494, 0x0)
#define IOM_DRAM_SDCLK_0	MA(0x588, 0x4ac, 0x0)
#define IOM_DRAM_SDCLK_1	MA(0x594, 0x4b0, 0x0)
#define IOM_DRAM_SDBA2		MA(0x58c, 0x4a0, 0x0)
#define IOM_DRAM_SDCKE0		MA(0x590, 0x4a4, 0x0)
#define IOM_DRAM_SDCKE1		MA(0x598, 0x4a8, 0x0)
#define IOM_DRAM_SDODT0		MA(0x59c, 0x4b4, 0x0)
#define IOM_DRAM_SDODT1		MA(0x5a0, 0x4b8, 0x0)

#define IOM_DRAM_SDQS0		MA(0x5a8, 0x4bc, 0x0)
#define IOM_DRAM_SDQS1		MA(0x5b0, 0x4c0, 0x0)
#define IOM_DRAM_SDQS2		MA(0x524, 0x4c4, 0x0)
#define IOM_DRAM_SDQS3		MA(0x51c, 0x4c8, 0x0)
#define IOM_DRAM_SDQS4		MA(0x518, 0x4cc, 0x0)
#define IOM_DRAM_SDQS5		MA(0x50c, 0x4d0, 0x0)
#define IOM_DRAM_SDQS6		MA(0x5b8, 0x4d4, 0x0)
#define IOM_DRAM_SDQS7		MA(0x5c0, 0x4d8, 0x0)

#define IOM_GRP_B0DS		MA(0x784, 0x764, 0x0)
#define IOM_GRP_B1DS		MA(0x788, 0x770, 0x0)
#define IOM_GRP_B2DS		MA(0x794, 0x778, 0x0)
#define IOM_GRP_B3DS		MA(0x79c, 0x77c, 0x0)
#define IOM_GRP_B4DS		MA(0x7a0, 0x780, 0x0)
#define IOM_GRP_B5DS		MA(0x7a4, 0x784, 0x0)
#define IOM_GRP_B6DS		MA(0x7a8, 0x78c, 0x0)
#define IOM_GRP_B7DS		MA(0x748, 0x748, 0x0)
#define IOM_GRP_ADDDS		MA(0x74c, 0x74c, 0x0)
#define IOM_DDRMODE_CTL		MA(0x750, 0x750, 0x0)
#define IOM_GRP_DDRPKE		MA(0x758, 0x754, 0x0)
#define IOM_GRP_DDRMODE		MA(0x774, 0x760, 0x0)
#define IOM_GRP_CTLDS		MA(0x78c, 0x76c, 0x0)
#define IOM_GRP_DDR_TYPE	MA(0x798, 0x774, 0x0)

#define MMDC_MDCTL		0x000
#define MMDC_MDPDC		0x004
#define MMDC_MDOTC		0x008
#define MMDC_MDCFG0		0x00c
#define MMDC_MDCFG1		0x010
#define MMDC_MDCFG2		0x014
#define MMDC_MDMISC		0x018
#define MMDC_MDSCR		0x01c
#define MMDC_MDREF		0x020
#define MMDC_MDRWD		0x02c
#define MMDC_MDOR		0x030
#define MMDC_MDASP		0x040
#define MMDC_MAPSR		0x404
#define MMDC_MPZQHWCTRL		0x800
#define MMDC_MPWLDECTRL0	0x80c
#define MMDC_MPWLDECTRL1	0x810
#define MMDC_MPODTCTRL		0x818
#define MMDC_MPRDDQBY0DL	0x81c
#define MMDC_MPRDDQBY1DL	0x820
#define MMDC_MPRDDQBY2DL	0x824
#define MMDC_MPRDDQBY3DL	0x828
#define MMDC_MPDGCTRL0		0x83c
#define MMDC_MPDGCTRL1		0x840
#define MMDC_MPRDDLCTL		0x848
#define MMDC_MPWRDLCTL		0x850
#define MMDC_MPMUR0		0x8b8

#define CCM_CCGR0		(CCM_BASE + 0x068)
#define CCM_CCGR1		(CCM_BASE + 0x06c)
#define CCM_CCGR2		(CCM_BASE + 0x070)
#define CCM_CCGR3		(CCM_BASE + 0x074)
#define CCM_CCGR4		(CCM_BASE + 0x078)
#define CCM_CCGR5		(CCM_BASE + 0x07c)
#define CCM_CCGR6		(CCM_BASE + 0x080)


#define WRITE_ENTRY1(addr, q)		DATA 4, addr, q
#ifdef CONFIG_MX6Q
#define MA(mx6q, mx6dl_solo, mx6sololite)	(IOMUXC_BASE + mx6q)
#define WRITE_ENTRY2(addr, q, dl)		WRITE_ENTRY1(addr, q)
#define WRITE_ENTRY3(addr, q, dl, solo)		WRITE_ENTRY1(addr, q)
#define WRITE_ENTRY4(addr, q, dl, solo, sl)	WRITE_ENTRY1(addr, q)
#else

#define WRITE_ENTRY2(addr, q, dl)		WRITE_ENTRY1(addr, dl)
#ifdef CONFIG_MX6DL
#define MA(mx6q, mx6dl_solo, mx6sololite)	(IOMUXC_BASE + mx6dl_solo)
#define WRITE_ENTRY3(addr, q, dl, solo)		WRITE_ENTRY1(addr, dl)
#define WRITE_ENTRY4(addr, q, dl, solo, sl)	WRITE_ENTRY1(addr, dl)
#else

#define WRITE_ENTRY3(addr, q, dl, solo)		WRITE_ENTRY1(addr, solo)
#ifdef CONFIG_MX6S
#define MA(mx6q, mx6dl_solo, mx6sololite)	(IOMUXC_BASE + mx6dl_solo)
#define WRITE_ENTRY4(addr, q, dl, solo, sl)	WRITE_ENTRY1(addr, solo)
#else

#define WRITE_ENTRY4(addr, q, dl, solo, sl)	WRITE_ENTRY1(addr, sl)
#ifdef CONFIG_MX6SL
#define MA(mx6q, mx6dl_solo, mx6sololite)	(IOMUXC_BASE + mx6sololite)
#else

#error "Please select cpu"
#endif	/* CONFIG_MX6SL */
#endif	/* CONFIG_MX6S */
#endif	/* CONFIG_MX6DL */
#endif	/* CONFIG_MX6Q */

#endif	/*__ASM_ARCH_IMX_MKIMAGE_H__ */

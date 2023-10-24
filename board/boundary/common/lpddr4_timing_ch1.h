/*
 * Copyright Boundary Devices
 *
 * SPDX-License-Identifier: GPL-2.0+
 */
#include <config.h>
#include <linux/kernel.h>
#include <asm/arch/ddr.h>
#include <asm/arch/lpddr4_define.h>

#ifdef WR_POST_EXT_3200
#define WR_POST_EXT 0x00020000
#else
#define WR_POST_EXT 0
#endif

#define CH1_VAL_INIT4	((LPDDR4_MR3 << 16) | WR_POST_EXT)
#define CH1_VAL_FREQ1_INIT4	((LPDDR4_MR3 << 16) | WR_POST_EXT)
#define CH1_VAL_FREQ2_INIT4	((LPDDR4_MR3 << 16) | WR_POST_EXT)

#ifdef CONFIG_DRATE_BYTE
#define DRATE_BYTE	(CONFIG_DRATE_BYTE << 4)
#else
#define DRATE_BYTE	(0 << 4)
#endif

#ifdef CONFIG_IMX8MN
#define IBASE_DIFF	1
#else
#define IBASE_DIFF	0
#endif

#if CONFIG_DDR_MB == 1024
	/* Address map is from MSB 28: r15, r14, r13-r0, b2-b0, c9-c0 */
#define CH1_VAL_DDRC_ADDRMAP0_R0	0x0000001F
#define CH1_VAL_DDRC_ADDRMAP6_R0	(0x06060606 + IBASE_DIFF * 0x01010101)
#define CH1_VAL_DDRC_ADDRMAP7_R0	0x0f0f
	/* Address map is from MSB 28: cs, r14, r13-r0, b2-b0, c9-c0 */
#define CH1_VAL_DDRC_ADDRMAP0_R1	(0x00000015 + IBASE_DIFF * 0x01)
#define CH1_VAL_DDRC_ADDRMAP6_R1	(0x0F060606 + IBASE_DIFF * 0x010101)
#define CH1_VAL_DDRC_ADDRMAP7_R1	0x0f0f

#elif CONFIG_DDR_MB == 1536
	/* Address map is from MSB 29: r15, r14, cs, r13-r0, b2-b0, c9-c0 */
#define CH1_VAL_DDRC_ADDRMAP0_R1	(0x00000014 + IBASE_DIFF * 0x01)
#define CH1_VAL_DDRC_ADDRMAP6_R1	(0x47070606 + IBASE_DIFF * 0x01010101)
#define CH1_VAL_DDRC_ADDRMAP7_R1	0x0f0f

#elif CONFIG_DDR_MB == 2048
	/* Address map is from MSB 29: cs, r15, r14, r13-r0, b2-b0, c9-c0 */
#define CH1_VAL_DDRC_ADDRMAP0_R1	(0x00000016 + IBASE_DIFF * 0x01)
#define CH1_VAL_DDRC_ADDRMAP6_R1	(0x06060606 + IBASE_DIFF * 0x01010101)
#define CH1_VAL_DDRC_ADDRMAP7_R1	0x0f0f
#else
#error unsupported memory size
#endif

#define LPDDR4_CS_R0	0x1	/* 0 rank bits, 1 chip select */
#define LPDDR4_CS_R1	0x3	/* 1 rank bit, 2 chip selects */

#if (CONFIG_DDR_RANK_BITS == 0) || !defined(CH1_VAL_DDRC_ADDRMAP0_R1)
#ifdef CH1_VAL_DDRC_ADDRMAP0_R0
#define CH1_LPDDR4_CS			LPDDR4_CS_R0
#define CH1_VAL_DDRC_ADDRMAP0		CH1_VAL_DDRC_ADDRMAP0_R0
#define CH1_VAL_DDRC_ADDRMAP6		CH1_VAL_DDRC_ADDRMAP6_R0
#define CH1_VAL_DDRC_ADDRMAP7		CH1_VAL_DDRC_ADDRMAP7_R0
#else
#error unsupported memory rank/size
#endif
/*
 * rank0 will succeed, even if really rank 1, so we need
 * to probe memory if rank0 succeeds
 */
#if defined(CH1_VAL_DDRC_ADDRMAP0_R0) && defined(CH1_VAL_DDRC_ADDRMAP0_R1)
#define CH1_LPDDR4_CS_NEW		LPDDR4_CS_R1
#define CH1_VAL_DDRC_ADDRMAP0_NEW	CH1_VAL_DDRC_ADDRMAP0_R1
#define CH1_VAL_DDRC_ADDRMAP6_NEW	CH1_VAL_DDRC_ADDRMAP6_R1
#define CH1_VAL_DDRC_ADDRMAP7_NEW	CH1_VAL_DDRC_ADDRMAP7_R1
#endif

#elif (CONFIG_DDR_RANK_BITS == 1) || !defined(CH1_VAL_DDRC_ADDRMAP0_R0)
#ifdef CH1_VAL_DDRC_ADDRMAP0_R1
#define CH1_LPDDR4_CS			LPDDR4_CS_R1
#define CH1_VAL_DDRC_ADDRMAP0		CH1_VAL_DDRC_ADDRMAP0_R1
#define CH1_VAL_DDRC_ADDRMAP6		CH1_VAL_DDRC_ADDRMAP6_R1
#define CH1_VAL_DDRC_ADDRMAP7		CH1_VAL_DDRC_ADDRMAP7_R1
#else
#error unsupported memory rank/size
#endif

#if defined(CH1_VAL_DDRC_ADDRMAP0_R0) && defined(CH1_VAL_DDRC_ADDRMAP0_R1)
#define CH1_LPDDR4_CS_NEW		LPDDR4_CS_R0
#define CH1_VAL_DDRC_ADDRMAP0_NEW	CH1_VAL_DDRC_ADDRMAP0_R0
#define CH1_VAL_DDRC_ADDRMAP6_NEW	CH1_VAL_DDRC_ADDRMAP6_R0
#define CH1_VAL_DDRC_ADDRMAP7_NEW	CH1_VAL_DDRC_ADDRMAP7_R0
#endif

#else
#error unsupported rank bits
#endif

#if (CONFIG_DDR_CHANNEL_CNT == 1)
#if (CONFIG_DDR_RANK_BITS == 0) && !defined(CH1_VAL_DDRC_ADDRMAP0_R0)
#error unsupported options
#endif
#if (CONFIG_DDR_RANK_BITS == 1) && !defined(CH1_VAL_DDRC_ADDRMAP0_R1)
#error unsupported options
#endif
#endif

/*
 * (C) Copyright 2009
 * Stefano Babic, DENX Software Engineering, sbabic@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _SYS_PROTO_H_
#define _SYS_PROTO_H_

#include <asm/io.h>
#include <asm/imx-common/regs-common.h>
#include <common.h>
#include "../arch-imx/cpu.h"

#define soc_rev() (get_cpu_rev() & 0xFF)
#define is_soc_rev(rev) (soc_rev() == rev)

/* returns MXC_CPU_ value */
#define cpu_type(rev) (((rev) >> 12) & 0xff)
#define soc_type(rev) (((rev) >> 12) & 0xf0)
/* both macros return/take MXC_CPU_ constants */
#define get_cpu_type() (cpu_type(get_cpu_rev()))
#define get_soc_type() (soc_type(get_cpu_rev()))
#define is_cpu_type(cpu) (get_cpu_type() == cpu)
#define is_soc_type(soc) (get_soc_type() == soc)

#define is_mx6() (is_soc_type(MXC_SOC_MX6))
#define is_mx7() (is_soc_type(MXC_SOC_MX7))

#define is_mx6dqp() (is_cpu_type(MXC_CPU_MX6QP) || is_cpu_type(MXC_CPU_MX6DP))
#define is_mx6dq() (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
#define is_mx6sdl() (is_cpu_type(MXC_CPU_MX6SOLO) || is_cpu_type(MXC_CPU_MX6DL))
#define is_mx6dl() (is_cpu_type(MXC_CPU_MX6DL))
#define is_mx6sx() (is_cpu_type(MXC_CPU_MX6SX))
#define is_mx6sl() (is_cpu_type(MXC_CPU_MX6SL))
#define is_mx6solo() (is_cpu_type(MXC_CPU_MX6SOLO))
#define is_mx6ul() (is_cpu_type(MXC_CPU_MX6UL))
#define is_mx6ull() (is_cpu_type(MXC_CPU_MX6ULL))
#define is_mx6sll() (is_cpu_type(MXC_CPU_MX6SLL))

#define is_mx7ulp() (is_cpu_type(MXC_CPU_MX7ULP))

#ifdef CONFIG_MX6
#define IMX6_SRC_GPR10_BMODE		BIT(28)

#define IMX6_BMODE_MASK			GENMASK(7, 0)
#define	IMX6_BMODE_SHIFT		4
#define IMX6_BMODE_EMI_MASK		BIT(3)
#define IMX6_BMODE_EMI_SHIFT		3
#define IMX6_BMODE_SERIAL_ROM_MASK	GENMASK(26, 24)
#define IMX6_BMODE_SERIAL_ROM_SHIFT	24

enum imx6_bmode_serial_rom {
	IMX6_BMODE_ECSPI1,
	IMX6_BMODE_ECSPI2,
	IMX6_BMODE_ECSPI3,
	IMX6_BMODE_ECSPI4,
	IMX6_BMODE_ECSPI5,
	IMX6_BMODE_I2C1,
	IMX6_BMODE_I2C2,
	IMX6_BMODE_I2C3,
};

enum imx6_bmode_emi {
	IMX6_BMODE_ONENAND,
	IMX6_BMODE_NOR,
};

enum imx6_bmode {
	IMX6_BMODE_EMI,
	IMX6_BMODE_UART,
	IMX6_BMODE_SATA,
	IMX6_BMODE_SERIAL_ROM,
	IMX6_BMODE_SD,
	IMX6_BMODE_ESD,
	IMX6_BMODE_MMC,
	IMX6_BMODE_EMMC,
	IMX6_BMODE_NAND,
};

static inline u8 imx6_is_bmode_from_gpr9(void)
{
	return readl(&src_base->gpr10) & IMX6_SRC_GPR10_BMODE;
}

u32 imx6_src_get_boot_mode(void);
#endif /* CONFIG_MX6 */

u32 get_nr_cpus(void);
u32 get_cpu_rev(void);
u32 get_cpu_speed_grade_hz(void);
u32 get_cpu_temp_grade(int *minc, int *maxc);
const char *get_imx_type(u32 imxtype);
u32 imx_ddr_size(void);
void sdelay(unsigned long);
void set_chipselect_size(int const);

void init_aips(void);
void init_src(void);
void imx_set_wdog_powerdown(bool enable);

/*
 * Initializes on-chip ethernet controllers.
 * to override, implement board_eth_init()
 */
int fecmxc_initialize(bd_t *bis);
u32 get_ahb_clk(void);
u32 get_periph_clk(void);

void lcdif_power_down(void);
struct fb_videomode;
int mxsfb_init(struct fb_videomode const *mode, uint32_t pixfmt);
void *mxsfb_init2(void);


int mxs_reset_block(struct mxs_register_32 *reg);
int mxs_wait_mask_set(struct mxs_register_32 *reg, u32 mask, u32 timeout);
int mxs_wait_mask_clr(struct mxs_register_32 *reg, u32 mask, u32 timeout);
#endif

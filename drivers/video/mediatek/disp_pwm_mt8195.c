// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek mt8195 display pwm setting
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Tommy Chen <tommyyl.chen@mediatek.com>
 */

#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include "mtk_panel.h"
#include "disp_reg_mt8195.h"

#define DISP_PWM_EN		0x00
#define ENABLE_MASK		BIT(0)

#define PWM_CLKDIV_SHIFT	16
#define PWM_CLKDIV_MAX		0x3ff
#define PWM_CLKDIV_MASK		(PWM_CLKDIV_MAX << PWM_CLKDIV_SHIFT)

#define PWM_PERIOD_BIT_WIDTH	12
#define PWM_PERIOD_MASK		((1 << PWM_PERIOD_BIT_WIDTH) - 1)

#define PWM_HIGH_WIDTH_SHIFT	16
#define PWM_HIGH_WIDTH_MASK	(0x1fff << PWM_HIGH_WIDTH_SHIFT)
#define PWM_HIGH_WIDTH_FULL_SHIFT	28

#define NSEC_PER_SEC 1000000000

#define DISP_PWM_CON_0 0x18
#define DISP_PWM_CON_1 0x1c
#define DISP_PWM_DEBUG 0x80
#define FORCE_COMMIT 3

#define PWM_REG_READ(addr) readl(addr)
#define PWM_REG_WRITE(addr, val) writel(val, addr)
#define PWM_REG_SET_BITS(addr, val) setbits_le32(addr, val)
#define PWM_REG_CLRSET_BITS(addr, clear, val) clrsetbits_le32(addr, clear, val)

/* use MTK_DISP_PWM_DEBUG to enable disp pwm debug logs
 * #define MTK_DISP_PWM_DEBUG
 */

#ifdef MTK_DISP_PWM_DEBUG
#define disp_pwm_printf(string, args...) printf("[DISP_PWM]"string, ##args)
#else
#define disp_pwm_printf(string, args...)
#endif

struct mtk_disp_pwm_ipc_config {
	u64 rate;
	u64 duty_ns;
	u64 period_ns;
};

static void mtk_disp_pwm_update_bits(u32 offset, u32 mask, u32 data)
{
	u32 value;

	value = PWM_REG_READ(DISP_PWM0_BASE + offset);

	value &= ~mask;
	value |= data;

	PWM_REG_WRITE(DISP_PWM0_BASE + offset, value);
}

static void mtk_disp_pwm_config(u64 rate, u64 period_ns, u64 duty_ns)
{
	u64 clk_div, period, high_width, value;
	u64 div;

	/*
	 * Find period, high_width and clk_div to suit duty_ns and period_ns.
	 * Calculate proper div value to keep period value in the bound.
	 *
	 * period_ns = 10^9 * (clk_div + 1) * (period + 1) / PWM_CLK_RATE
	 * duty_ns = 10^9 * (clk_div + 1) * high_width / PWM_CLK_RATE
	 *
	 * period = (PWM_CLK_RATE * period_ns) / (10^9 * (clk_div + 1)) - 1
	 * high_width = (PWM_CLK_RATE * duty_ns) / (10^9 * (clk_div + 1))
	 */

	disp_pwm_printf("rate: %lld\n", rate);
	disp_pwm_printf("period_ns: %lld\n", period_ns);
	disp_pwm_printf("duty_ns: %lld\n", duty_ns);

	clk_div = DIV_ROUND_UP(rate * period_ns, NSEC_PER_SEC) >> PWM_PERIOD_BIT_WIDTH;
	if (clk_div > PWM_CLKDIV_MAX)
		return;

	div = 1000000000 * (clk_div + 1);
	period = DIV_ROUND_UP(rate * period_ns, div);
	if (period > 0)
		period--;

	high_width = DIV_ROUND_UP(rate * duty_ns, div);
	value = period | (high_width << PWM_HIGH_WIDTH_SHIFT) | (1 << PWM_HIGH_WIDTH_FULL_SHIFT);

	disp_pwm_printf("period: %d\n", period);
	disp_pwm_printf("high_width: %d\n", high_width);
	disp_pwm_printf("clk_div: %d, value %d\n", clk_div, value);

	mtk_disp_pwm_update_bits(DISP_PWM_CON_0, PWM_CLKDIV_MASK, clk_div << PWM_CLKDIV_SHIFT);
	mtk_disp_pwm_update_bits(DISP_PWM_CON_1, PWM_PERIOD_MASK | PWM_HIGH_WIDTH_MASK, value);

	mtk_disp_pwm_update_bits(DISP_PWM_DEBUG, FORCE_COMMIT, FORCE_COMMIT);
}

void mtk_disp_pwm_dump(void)
{
	int k;

	printf("- DSI REGS -\n");
	for (k = 0; k < 0xc0; k += 16) {
		printf("0x%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n", k,
		       PWM_REG_READ(DISP_PWM0_BASE + k),
		       PWM_REG_READ(DISP_PWM0_BASE + k + 0x4),
		       PWM_REG_READ(DISP_PWM0_BASE + k + 0x8),
		       PWM_REG_READ(DISP_PWM0_BASE + k + 0xc));
	}
}

u32 lcm_if_set_backlight(u32 level, u32 level_max, bool use_default_level)
{
	u64 rate, period_ns, duty_ns;
	struct panel_description *panel_desc = NULL;
	int ret;

	panel_get_desc(&panel_desc);

	if (use_default_level) {
		level = 10;
		level_max = 15;
	}

	rate = 104000000;
	period_ns = 1000000;
	duty_ns = level * period_ns / (level_max + 1);

	/* enable pwm_main clock */
	ret = clk_enable(&panel_desc->pwm_main);
	if (ret < 0)
		printf("Could not enable pwm_main clk, panel may not work. ret: %d\n", ret);

	/* enable pwm_mm clock */
	ret = clk_enable(&panel_desc->pwm_mm);
	if (ret < 0)
		printf("Could not enable pwm_mm clk, panel may not work. ret: %d\n", ret);

	mtk_disp_pwm_config(rate, period_ns, duty_ns);
	mtk_disp_pwm_update_bits(DISP_PWM_EN, ENABLE_MASK, ENABLE_MASK);

#ifdef MTK_DISP_PWM_DEBUG
	mtk_disp_pwm_dump();
#endif

	return 0;
}

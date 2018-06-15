/*
 * (C) Copyright 2016
 * Vikas Manocha, ST Micoelectronics, vikas.manocha@st.com.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/stm32.h>
#include <asm/arch/stm32_defs.h>
#include <asm/arch/gpt.h>

#define READ_TIMER()	(readl(&gpt1_regs_ptr->cnt) & GPT_FREE_RUNNING)
#define GPT_RESOLUTION	(CONFIG_SYS_HZ_CLOCK/CONFIG_STM32_HZ)

DECLARE_GLOBAL_DATA_PTR;

#define timestamp gd->arch.tbl
#define lastdec gd->arch.lastinc

int timer_init(void)
{
	/* Timer2 clock configuration */
	clock_setup(TIMER2_CLOCK_CFG);
	/* Stop the timer */
	writel(readl(&gpt1_regs_ptr->cr1) & ~GPT_CR1_CEN, &gpt1_regs_ptr->cr1);

	writel((CONFIG_SYS_CLK_FREQ/CONFIG_SYS_HZ_CLOCK) - 1,
						&gpt1_regs_ptr->psc);

	/* Configure timer for auto-reload */
	writel(readl(&gpt1_regs_ptr->cr1) | GPT_MODE_AUTO_RELOAD,
						&gpt1_regs_ptr->cr1);

	/* load value for free running */
	writel(GPT_FREE_RUNNING, &gpt1_regs_ptr->arr);

	/* start timer */
	writel(readl(&gpt1_regs_ptr->cr1) | GPT_CR1_CEN, &gpt1_regs_ptr->cr1);

	writel(readl(&gpt1_regs_ptr->egr) | TIM_EGR_UG, &gpt1_regs_ptr->egr);

	/* Reset the timer */
	lastdec = READ_TIMER();
	timestamp = 0;

	return 0;
}

/*
 * timer without interrupts
 */
ulong get_timer(ulong base)
{
	return (get_timer_masked() / GPT_RESOLUTION) - base;
}

void __udelay(unsigned long usec)
{
	ulong tmo;
	ulong start = get_timer_masked();
	ulong tenudelcnt = CONFIG_SYS_HZ_CLOCK / (1000 * 100);
	ulong rndoff;

	rndoff = (usec % 10) ? 1 : 0;

	/* tenudelcnt timer tick gives 10 microsecconds delay */
	tmo = ((usec / 10) + rndoff) * tenudelcnt;

	while ((ulong) (get_timer_masked() - start) < tmo)
		;
}

ulong get_timer_masked(void)
{
	ulong now = READ_TIMER();

	if (now >= lastdec) {
		/* normal mode */
		timestamp += now - lastdec;
	} else {
		/* we have an overflow ... */
		timestamp += now + GPT_FREE_RUNNING - lastdec;
	}
	lastdec = now;

	return timestamp;
}

void udelay_masked(unsigned long usec)
{
	return udelay(usec);
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return CONFIG_STM32_HZ;
}

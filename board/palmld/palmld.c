/*
 * Palm LifeDrive Support
 *
 * Copyright (C) 2010 Marek Vasut <marek.vasut@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <serial.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxa.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Miscelaneous platform dependent initialisations
 */

int board_init(void)
{
	/* We have RAM, disable cache */
	dcache_disable();
	icache_disable();

	/* arch number of PalmLD */
	gd->bd->bi_arch_number = MACH_TYPE_PALMLD;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = 0xa0000100;

	/* Set PWM for LCD */
	writel(0x7, PWM_CTRL0);
	writel(0x16c, PWM_PERVAL0);
	writel(0x11a, PWM_PWDUTY0);

	return 0;
}

int dram_init(void)
{
	pxa2xx_dram_init();
	gd->ram_size = PHYS_SDRAM_1_SIZE;
	return 0;
}

void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
}

ulong board_flash_get_legacy(ulong base, int banknum, flash_info_t *info)
{
	info->portwidth = FLASH_CFI_16BIT;
	info->chipwidth = FLASH_CFI_BY16;
	info->interface = FLASH_CFI_X16;
	return 1;
}

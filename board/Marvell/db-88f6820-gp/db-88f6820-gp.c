/*
 * Copyright (C) 2015 Stefan Roese <sr@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <miiphy.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/soc.h>

DECLARE_GLOBAL_DATA_PTR;

#define BIT(nr)				(1UL << (nr))

#define ETH_PHY_CTRL_REG		0
#define ETH_PHY_CTRL_POWER_DOWN_BIT	11
#define ETH_PHY_CTRL_POWER_DOWN_MASK	(1 << ETH_PHY_CTRL_POWER_DOWN_BIT)

/*
 * Those values and defines are taken from the Marvell U-Boot version
 * "u-boot-2013.01-2014_T3.0"
 */
#define DB_GP_88F68XX_GPP_OUT_ENA_LOW					\
	(~(BIT(1)  | BIT(4)  | BIT(6)  | BIT(7)  | BIT(8)  | BIT(9)  |	\
	   BIT(10) | BIT(11) | BIT(19) | BIT(22) | BIT(23) | BIT(25) |	\
	   BIT(26) | BIT(27) | BIT(29) | BIT(30) | BIT(31)))
#define DB_GP_88F68XX_GPP_OUT_ENA_MID					\
	(~(BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(15) |	\
	   BIT(16) | BIT(17) | BIT(18)))

#define DB_GP_88F68XX_GPP_OUT_VAL_LOW	0x0
#define DB_GP_88F68XX_GPP_OUT_VAL_MID	0x0
#define DB_GP_88F68XX_GPP_POL_LOW	0x0
#define DB_GP_88F68XX_GPP_POL_MID	0x0

/* IO expander on Marvell GP board includes e.g. fan enabling */
struct marvell_io_exp {
	u8 chip;
	u8 addr;
	u8 val;
};

static struct marvell_io_exp io_exp[] = {
	{ 0x20, 6, 0x20 }, /* Configuration registers: Bit on --> Input bits */
	{ 0x20, 7, 0xC3 }, /* Configuration registers: Bit on --> Input bits */
	{ 0x20, 2, 0x1D }, /* Output Data, register#0 */
	{ 0x20, 3, 0x18 }, /* Output Data, register#1 */
	{ 0x21, 6, 0xC3 }, /* Configuration registers: Bit on --> Input bits  */
	{ 0x21, 7, 0x31 }, /* Configuration registers: Bit on --> Input bits  */
	{ 0x21, 2, 0x08 }, /* Output Data, register#0 */
	{ 0x21, 3, 0xC0 }  /* Output Data, register#1 */
};

int board_early_init_f(void)
{
	/* Configure MPP */
	writel(0x11111111, MVEBU_MPP_BASE + 0x00);
	writel(0x11111111, MVEBU_MPP_BASE + 0x04);
	writel(0x11244011, MVEBU_MPP_BASE + 0x08);
	writel(0x22222111, MVEBU_MPP_BASE + 0x0c);
	writel(0x22200002, MVEBU_MPP_BASE + 0x10);
	writel(0x30042022, MVEBU_MPP_BASE + 0x14);
	writel(0x55550555, MVEBU_MPP_BASE + 0x18);
	writel(0x00005550, MVEBU_MPP_BASE + 0x1c);

	/* Set GPP Out value */
	writel(DB_GP_88F68XX_GPP_OUT_VAL_LOW, MVEBU_GPIO0_BASE + 0x00);
	writel(DB_GP_88F68XX_GPP_OUT_VAL_MID, MVEBU_GPIO1_BASE + 0x00);

	/* Set GPP Polarity */
	writel(DB_GP_88F68XX_GPP_POL_LOW, MVEBU_GPIO0_BASE + 0x0c);
	writel(DB_GP_88F68XX_GPP_POL_MID, MVEBU_GPIO1_BASE + 0x0c);

	/* Set GPP Out Enable */
	writel(DB_GP_88F68XX_GPP_OUT_ENA_LOW, MVEBU_GPIO0_BASE + 0x04);
	writel(DB_GP_88F68XX_GPP_OUT_ENA_MID, MVEBU_GPIO1_BASE + 0x04);

	return 0;
}

int board_init(void)
{
	int i;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = mvebu_sdram_bar(0) + 0x100;

	/* Init I2C IO expanders */
	for (i = 0; i < ARRAY_SIZE(io_exp); i++)
		i2c_write(io_exp[i].chip, io_exp[i].addr, 1, &io_exp[i].val, 1);

	return 0;
}

int checkboard(void)
{
	puts("Board: Marvell DB-88F6820-GP\n");

	return 0;
}

/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/clock.h>
#if defined(CONFIG_IMX8MM)
#include <asm/arch/imx8mm_pins.h>
#elif defined(CONFIG_IMX8MN)
#include <asm/arch/imx8mn_pins.h>
#endif
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/dma.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/video.h>
#include <dm.h>
#include <env.h>
#include <i2c.h>
#include <linux/delay.h>
#include <spl.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

static iomux_v3_cfg_t const init_pads[] = {
	IOMUX_PAD_CTRL(GPIO1_IO02__WDOG1_WDOG_B, WDOG_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_RXD__UART2_DCE_RX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_TXD__UART2_DCE_TX, UART_PAD_CTRL),

/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO1_IO03__GPIO1_IO3, 0x116),

#define GPIRQ_GT911 			IMX_GPIO_NR(3, 6)
	IOMUX_PAD_CTRL(NAND_DATA00__GPIO3_IO6, 0x1d6),
#define GP_GT911_RESET			IMX_GPIO_NR(3, 7)
	IOMUX_PAD_CTRL(NAND_DATA01__GPIO3_IO7, 0x149),


#define GP_CSI1_1MIPI_PWDN	IMX_GPIO_NR(3, 9)
	IOMUX_PAD_CTRL(NAND_DATA03__GPIO3_IO9, 0x141),
#define GP_CSI1_1MIPI_RESET	IMX_GPIO_NR(3, 1)
	IOMUX_PAD_CTRL(NAND_CE0_B__GPIO3_IO1, 0x101),

#define GP_CSI1_2MIPI_PWDN	IMX_GPIO_NR(3, 16)
	IOMUX_PAD_CTRL(NAND_READY_B__GPIO3_IO16, 0x141),
#define GP_CSI1_2MIPI_RESET	IMX_GPIO_NR(3, 0)
	IOMUX_PAD_CTRL(NAND_ALE__GPIO3_IO0, 0x101),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(3, 22)
	IOMUX_PAD_CTRL(SAI5_RXD1__GPIO3_IO22, 0x09),
#define GP_WM8960_HP_DET	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x80),

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x41),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(GP_EMMC_RESET, 1);
	set_wdog_reset(wdog);
	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
int board_detect_gt911(struct display_info_t const *di)
{
	int ret;
	struct udevice *dev, *chip;

#ifdef CONFIG_DM_VIDEO
	if (di->bus_gp)
		gpio_request(di->bus_gp, "bus_gp");
#endif
	if (di->bus_gp)
		gpio_direction_output(di->bus_gp, 1);
	gpio_set_value(GP_GT911_RESET, 0);
	mdelay(20);
	gpio_direction_output(GPIRQ_GT911, di->addr_num == 0x14 ? 1 : 0);
	udelay(100);
	gpio_set_value(GP_GT911_RESET, 1);
	mdelay(6);
	gpio_set_value(GPIRQ_GT911, 0);
	mdelay(50);
	gpio_direction_input(GPIRQ_GT911);
	ret = uclass_get_device(UCLASS_I2C, di->bus_num, &dev);
	if (!ret) {
		ret = dm_i2c_probe(dev, di->addr_num, 0x0, &chip);
		if (ret && di->bus_gp)
			gpio_direction_input(di->bus_gp);
	}
#ifdef CONFIG_DM_VIDEO
	if (di->bus_gp)
		gpio_free(di->bus_gp);
#endif
	return (ret == 0);
}

static const struct display_info_t displays[] = {
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(3, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
#endif
	gpio_request(GP_CSI1_1MIPI_PWDN, "csi1_1mipi_pwdn");
	gpio_request(GP_CSI1_1MIPI_RESET, "csi1_1mipi_reset");
	gpio_request(GP_CSI1_2MIPI_PWDN, "csi1_2mipi_pwdn");
	gpio_request(GP_CSI1_2MIPI_RESET, "csi1_2mipi_reset");
	gpio_direction_output(GP_GT911_RESET, 0);
	gpio_direction_output(GP_CSI1_1MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI1_1MIPI_RESET, 0);
	gpio_direction_output(GP_CSI1_2MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI1_2MIPI_RESET, 0);
#if defined(CONFIG_MXC_SPI) && !defined(CONFIG_DM_SPI)
	setup_spi();
#endif

#ifdef CONFIG_FSL_FSPI
	board_qspi_init();
#endif

#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}

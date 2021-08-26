/*
 * Copyright 2021 Boundary Devices LLC
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
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

#define GP_I2C3_PCA9546_RESET		IMX_GPIO_NR(4, 4)
	IOMUX_PAD_CTRL(SAI1_RXD2__GPIO4_IO4, 0x140),

#define GPIRQ_SN65DSI83			IMX_GPIO_NR(3, 20)
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(3, 20)
	IOMUX_PAD_CTRL(SAI5_RXC__GPIO3_IO20, 0x100),

	/* Touch screen on J8 and J22, same pins used */
#define GPIRQ_GT911 			IMX_GPIO_NR(4, 1)
	IOMUX_PAD_CTRL(SAI1_RXC__GPIO4_IO1, 0xd6),
#define GP_GT911_RESET			IMX_GPIO_NR(4, 0)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(4, 0)
	IOMUX_PAD_CTRL(SAI1_RXFS__GPIO4_IO0, 0x109),

#define GP_SN65DSI83_EN		IMX_GPIO_NR(5, 28)
#define GP_MIPI_RESET		IMX_GPIO_NR(5, 28)
	IOMUX_PAD_CTRL(UART4_RXD__GPIO5_IO28, 0x100),

#define GP_BACKLIGHT_MIPI		IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(SPDIF_TX__GPIO5_IO3, 0x116),

#define GP_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(1, 11)
	IOMUX_PAD_CTRL(GPIO1_IO11__GPIO1_IO11, 0x141),
#define GP_OV5640_MIPI_RESET	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO10, 0x101),

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IOMUX_PAD_CTRL(SD1_RESET_B__GPIO2_IO10, 0x41),

#define GP_FASTBOOT_KEY		IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO1_IO07__GPIO1_IO7, WEAK_PULLUP),
};

#if defined(CONFIG_CMD_FASTBOOT) || defined(CONFIG_CMD_DFU)
int board_fastboot_key_pressed(void)
{
	gpio_request(GP_FASTBOOT_KEY, "fastboot_key");
	gpio_direction_input(GP_FASTBOOT_KEY);
	return !gpio_get_value(GP_FASTBOOT_KEY);
}
#endif

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	gpio_direction_output(GP_BACKLIGHT_MIPI, 0);
	gpio_request(GP_SN65DSI83_EN, "sn65en");
	gpio_direction_output(GP_SN65DSI83_EN, 0);
	gpio_direction_output(GP_EMMC_RESET, 1);
	set_wdog_reset(wdog);

	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
int board_detect_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di,  1 << (di->bus_num >> 4), 0, GP_GT911_RESET, GPIRQ_GT911);
}

void board_enable_mipi(const struct display_info_t *di, int enable)
{
#ifndef CONFIG_DM_VIDEO
	if (di->enable_alias[0] == FBP_BACKLIGHT_MIPI) {
		gpio_direction_output(GP_BACKLIGHT_MIPI, enable);
	}
#endif
}

static const struct display_info_t displays[] = {
	VD_LTK080A60A004T_2(MIPI, NULL, fbp_bus_gp(2 | (2 << 4), GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0),
	VD_MIPI_M101NWWB(MIPI, board_detect_pca9546, fbp_bus_gp(2 | (2 << 4), GP_SN65DSI83_EN, 0, 0), 0x2c, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(2 | (2 << 4), GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
	gpio_request(GP_BACKLIGHT_MIPI, "backlight_pwm");
#endif
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
	gpio_request(GP_OV5640_MIPI_RESET, "csi1_mipi_reset");
	gpio_request(GP_I2C3_PCA9546_RESET, "i2c3-pca9546-reset");
	gpio_direction_output(GP_OV5640_MIPI_RESET, 0);
	gpio_direction_output(GP_GT911_RESET, 0);
	gpio_direction_output(GP_I2C3_PCA9546_RESET, 1);
	gpio_free(GP_I2C3_PCA9546_RESET);

#if defined(CONFIG_MXC_SPI) && !defined(CONFIG_DM_SPI)
	setup_spi();
#endif

#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}

void board_poweroff(void)
{
	struct snvs_regs *snvs = (struct snvs_regs *)(SNVS_BASE_ADDR);

	writel(0x60, &snvs->lpcr);
	mdelay(500);
}

static int _do_poweroff(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	board_poweroff();
	return 0;
}

U_BOOT_CMD(
	poweroff, 70, 0, _do_poweroff,
	"power down board",
	""
);

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

#define GPIRQ_SN65DSI83			IMX_GPIO_NR(1, 1)
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x146),

#define GP_DLC0350_ENABLE		IMX_GPIO_NR(1, 3)
	IOMUX_PAD_CTRL(GPIO1_IO03__GPIO1_IO3, 0x100),

	/* Touch screen on J8 and J22, same pins used */
#define GPIRQ_GT911 			IMX_GPIO_NR(5, 22)
	IOMUX_PAD_CTRL(UART1_RXD__GPIO5_IO22, 0xd6),
#define GP_GT911_RESET			IMX_GPIO_NR(5, 23)
	IOMUX_PAD_CTRL(UART1_TXD__GPIO5_IO23, 0x109),

#define GP_SN65DSI83_EN		IMX_GPIO_NR(1, 9)
#define GP_MIPI_RESET		IMX_GPIO_NR(1, 9)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, 0x06),
	IOMUX_PAD_CTRL(SAI2_RXC__GPIO4_IO22, PAD_CTL_DSE1 | PAD_CTL_ODE),

#define GP_BACKLIGHT_MIPI_ALT		IMX_GPIO_NR(5, 3)
	IOMUX_PAD_CTRL(SPDIF_TX__GPIO5_IO3, 0x116),

#define GP_CSI1_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO8, 0x141),
#define GP_CSI1_OV5640_MIPI_RESET	IMX_GPIO_NR(3, 5)
	IOMUX_PAD_CTRL(NAND_CLE__GPIO3_IO5, 0x101),

#ifdef CONFIG_IMX8MM
	IOMUX_PAD_CTRL(GPIO1_IO14__USB2_OTG_PWR, 0x16),
	IOMUX_PAD_CTRL(GPIO1_IO15__USB2_OTG_OC, WEAK_PULLUP),
#endif

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
	gpio_direction_output(GP_BACKLIGHT_MIPI_ALT, 0);
	gpio_request(GP_SN65DSI83_EN, "sn65en");
	gpio_direction_output(GP_SN65DSI83_EN, 0);
	gpio_direction_output(GP_EMMC_RESET, 1);
	set_wdog_reset(wdog);

	return 0;
}

#ifdef CONFIG_CMD_FBPANEL
int board_detect_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, (di->bus_num >> 4) | 4, 0, GP_GT911_RESET, GPIRQ_GT911);
}

void board_enable_mipi(const struct display_info_t *di, int enable)
{
#ifndef CONFIG_DM_VIDEO
	if (di->enable_alias[0] == FBP_BACKLIGHT_MIPI_ALT) {
		gpio_direction_output(GP_BACKLIGHT_MIPI_ALT, enable);
	}
#endif
}

static const struct display_info_t displays[] = {
	VD_MIPI_DLC0350GEV06(MIPI, board_detect_gt911, fbp_bus_gp(2, 0, GP_DLC0350_ENABLE, 0), fbp_addr_gp(0x5d, 0, 0, 0), FBTS_GOODIX),
	VD_LTK080A60A004T(MIPI, NULL, fbp_bus_gp(2, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBP_BACKLIGHT_MIPI_ALT, FBTS_GOODIX),
	VD_MIPI_M101NWWB_x("m101nwwb-1", B, MIPI, fbp_detect_i2c, fbp_bus_gp(2, GP_SN65DSI83_EN, 0, 0), 0x2c, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_MIPI_1280_720M_60(MIPI, NULL, fbp_bus_gp((2 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_1920_1080M_60(MIPI, board_detect_pca9546, fbp_bus_gp((2 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
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
	gpio_request(GP_BACKLIGHT_MIPI_ALT, "backlight_mipi_alt");
#endif
	gpio_request(GP_DLC0350_ENABLE, "dlc0350 reset");
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
	gpio_request(GP_CSI1_OV5640_MIPI_RESET, "csi1_mipi_reset");
	gpio_direction_output(GP_DLC0350_ENABLE, 0);
	gpio_direction_output(GP_CSI1_OV5640_MIPI_RESET, 0);
	gpio_direction_output(GP_GT911_RESET, 0);
	gpio_free(GP_DLC0350_ENABLE);

#ifdef CONFIG_MAX77823
	max77823_init();
	max77823_boost_power(1);
#endif

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

#ifdef CONFIG_MAX77823
	max77823_otg_power(0);
	max77823_boost_power(0);
#endif
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

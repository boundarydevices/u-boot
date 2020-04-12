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
#include <i2c.h>
#include <spl.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

static iomux_v3_cfg_t const init_pads[] = {
	IOMUX_PAD_CTRL(GPIO1_IO02__WDOG1_WDOG_B, WDOG_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_RXD__UART2_DCE_RX, UART_PAD_CTRL),
	IOMUX_PAD_CTRL(UART2_TXD__UART2_DCE_TX, UART_PAD_CTRL),
#define GP_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(3, 9)
	IOMUX_PAD_CTRL(NAND_DATA03__GPIO3_IO9, 0x141),
#define GP_OV5640_MIPI_RESET	IMX_GPIO_NR(3, 5)
	IOMUX_PAD_CTRL(NAND_CLE__GPIO3_IO5, 0x101),

#define GP_LVDS_PWM		IMX_GPIO_NR(5, 2)
	IOMUX_PAD_CTRL(SAI3_MCLK__GPIO5_IO2, WEAK_PULLDN_OUTPUT),
#define GP_LVDS_BKL_EN		IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO8, WEAK_PULLDN_OUTPUT),

#ifdef CONFIG_FEC_MXC
	/* PHY - AR8035 */
#ifdef CONFIG_FEC_PHY_BITBANG
#define GP_MII_MDIO	IMX_GPIO_NR(1, 17)
	IOMUX_PAD_CTRL(ENET_MDIO__GPIO1_IO17, 0),
#define GP_MII_MDC	IMX_GPIO_NR(1, 16)
	IOMUX_PAD_CTRL(ENET_MDC__GPIO1_IO16, 0),
#else
	IOMUX_PAD_CTRL(ENET_MDIO__ENET1_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET1_MDC, PAD_CTRL_ENET_MDC),
#endif
	IOMUX_PAD_CTRL(ENET_TX_CTL__ENET1_RGMII_TX_CTL, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD0__ENET1_RGMII_TD0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD1__ENET1_RGMII_TD1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD2__ENET1_RGMII_TD2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD3__ENET1_RGMII_TD3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TXC__ENET1_RGMII_TXC, PAD_CTRL_ENET_TX),
#endif

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
static const struct display_info_t displays[] = {
	VD_MIPI_TM070JDHG30(MIPI, fbp_detect_i2c, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38, FBTS_FT5X06),
	VD_MIPI_640_480M_60(MIPI, fbp_detect_i2c, 1, 0x70),
	VD_MIPI_1280_720M_60(MIPI, NULL, 1, 0x70),
	VD_MIPI_1920_1080M_60(MIPI, NULL, 1, 0x70),
	VD_MIPI_1024_768M_60(MIPI, NULL, 1, 0x70),
	VD_MIPI_800_600MR_60(MIPI, NULL, 1, 0x70),
	VD_MIPI_720_480M_60(MIPI, NULL, 1, 0x70),
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
#ifdef CONFIG_FEC_PHY_BITBANG
	gpio_request(GP_MII_MDC, "mii_mdc");
	gpio_request(GP_MII_MDIO, "mii_mdio");
#endif
	gpio_request(GP_OV5640_MIPI_POWER_DOWN, "csi1_mipi_pwdn");
	gpio_request(GP_OV5640_MIPI_RESET, "csi1_mipi_reset");
	gpio_direction_output(GP_OV5640_MIPI_POWER_DOWN, 1);
	gpio_direction_output(GP_OV5640_MIPI_RESET, 0);
	gpio_request(GP_LVDS_PWM, "lvds_pwm");
	gpio_request(GP_LVDS_BKL_EN, "lvds backlight enable");
	gpio_direction_output(GP_LVDS_PWM, 0);
	gpio_direction_output(GP_LVDS_BKL_EN, 0);
#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#ifdef CONFIG_FSL_FSPI
	board_qspi_init();
#endif

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand(); /* SPL will call the board_early_init_f */
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}

/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/video.h>
#include <linux/delay.h>
#include <video_fb.h>
#include <spl.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <dm.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

static iomux_v3_cfg_t const init_pads[] = {
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
	IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
#define GP_I2C4_SN65DSI83_IRQ		IMX_GPIO_NR(1, 1)
#define GP_LCM_JM430_BKL_EN		IMX_GPIO_NR(1, 1)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
#define GP_LCD133_070_RESET		IMX_GPIO_NR(1, 1)
#define GP_TCXD070_BKL_EN		IMX_GPIO_NR(1, 1)
	IMX8MQ_PAD_GPIO1_IO01__GPIO1_IO1 | MUX_PAD_CTRL(0x16),

#define GPIRQ_TS_GT911 			IMX_GPIO_NR(3, 12)
#define GPIRQ_LCD133_TOUCH		IMX_GPIO_NR(3, 12)
	IMX8MQ_PAD_NAND_DATA06__GPIO3_IO12 | MUX_PAD_CTRL(0xc3),
#define GP_TS_GT911_RESET		IMX_GPIO_NR(3, 13)
#define GP_ST1633_RESET			IMX_GPIO_NR(3, 13)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(3, 13)
#define GP_TS_FT7250_RESET		IMX_GPIO_NR(3, 13)
#define GP_TS_ILI251X_RESET		IMX_GPIO_NR(3, 13)
	IMX8MQ_PAD_NAND_DATA07__GPIO3_IO13 | MUX_PAD_CTRL(0x43),

#define GP_ARM_DRAM_VSEL		IMX_GPIO_NR(3, 24)
	IMX8MQ_PAD_SAI5_RXD3__GPIO3_IO24 | MUX_PAD_CTRL(0x16),
#define GP_DRAM_1P1_VSEL		IMX_GPIO_NR(2, 11)
	IMX8MQ_PAD_SD1_STROBE__GPIO2_IO11 | MUX_PAD_CTRL(0x16),
#define GP_SOC_GPU_VPU_VSEL		IMX_GPIO_NR(2, 20)
	IMX8MQ_PAD_SD2_WP__GPIO2_IO20 | MUX_PAD_CTRL(0x16),

#define GP_BACKLIGHT_MIPI		IMX_GPIO_NR(5, 3)
	IMX8MQ_PAD_SPDIF_TX__GPIO5_IO3 | MUX_PAD_CTRL(0x4),

#define GP_FASTBOOT_KEY			IMX_GPIO_NR(1, 7)
	IMX8MQ_PAD_GPIO1_IO07__GPIO1_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),

#define GP_I2C1_PCA9546_RESET		IMX_GPIO_NR(1, 8)
	IMX8MQ_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(0x49),

#define GP_TC358762_EN			IMX_GPIO_NR(3, 15)
#define GP_SC18IS602B_RESET		IMX_GPIO_NR(3, 15)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(3, 15)
#define GP_MIPI_RESET			IMX_GPIO_NR(3, 15)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(3, 15)
	IMX8MQ_PAD_NAND_RE_B__GPIO3_IO15 | MUX_PAD_CTRL(0x6),


#define GP_EMMC_RESET			IMX_GPIO_NR(2, 10)
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(0x41),

#define GPIRQ_CSI1_TC3587		IMX_GPIO_NR(3, 3)
#define GP_CSI1_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(3, 3)
	IMX8MQ_PAD_NAND_CE2_B__GPIO3_IO3 | MUX_PAD_CTRL(0x45),
#define GP_CSI1_OV5640_MIPI_RESET	IMX_GPIO_NR(3, 17)
	IMX8MQ_PAD_NAND_WE_B__GPIO3_IO17 | MUX_PAD_CTRL(0x05),

#define GPIRQ_CSI2_TC3587		IMX_GPIO_NR(3, 2)
#define GP_CSI2_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(3, 2)
	IMX8MQ_PAD_NAND_CE1_B__GPIO3_IO2 | MUX_PAD_CTRL(0x45),
#define GP_CSI2_OV5640_MIPI_RESET	IMX_GPIO_NR(2, 19)
	IMX8MQ_PAD_SD2_RESET_B__GPIO2_IO19 |MUX_PAD_CTRL(0x05),
#ifdef CONFIG_FEC_MXC
	/* PHY - AR8035 */
	IOMUX_PAD_CTRL(ENET_MDIO__ENET_MDIO, PAD_CTRL_ENET_MDIO),
	IOMUX_PAD_CTRL(ENET_MDC__ENET_MDC, PAD_CTRL_ENET_MDC),
	IOMUX_PAD_CTRL(ENET_TX_CTL__ENET_RGMII_TX_CTL, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD0__ENET_RGMII_TD0, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD1__ENET_RGMII_TD1, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD2__ENET_RGMII_TD2, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TD3__ENET_RGMII_TD3, PAD_CTRL_ENET_TX),
	IOMUX_PAD_CTRL(ENET_TXC__ENET_RGMII_TXC, PAD_CTRL_ENET_TX),
#endif
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, WEAK_PULLUP),
#define GPIRQ_ENET_PHY		IMX_GPIO_NR(1, 11)
	IOMUX_PAD_CTRL(GPIO1_IO11__GPIO1_IO11, WEAK_PULLUP),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	set_wdog_reset(wdog);

	gpio_request(GP_BACKLIGHT_MIPI, "backlight_mipi");
	gpio_request(GP_ARM_DRAM_VSEL, "arm_vsel");
	gpio_request(GP_DRAM_1P1_VSEL, "dram_vsel");
	gpio_request(GP_SOC_GPU_VPU_VSEL, "soc_vsel");
	gpio_request(GP_EMMC_RESET, "emmc_reset");
	gpio_request(GP_I2C1_PCA9546_RESET, "pca9546_reset");
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
#endif
	gpio_request(GPIRQ_CSI1_TC3587, "csi1_tc3587");
	gpio_request(GP_CSI1_OV5640_MIPI_RESET, "csi1_ov5640_reset");
	gpio_request(GPIRQ_CSI2_TC3587, "csi2_tc3587");
	gpio_request(GP_CSI2_OV5640_MIPI_RESET, "csi2_ov5640_reset");

	gpio_direction_output(GP_BACKLIGHT_MIPI, 0);
	gpio_direction_output(GP_ARM_DRAM_VSEL, 0);
	gpio_direction_output(GP_DRAM_1P1_VSEL, 0);
	gpio_direction_output(GP_SOC_GPU_VPU_VSEL, 0);
	gpio_direction_output(GP_EMMC_RESET, 1);
	gpio_direction_output(GP_I2C1_PCA9546_RESET, 0);
	gpio_direction_output(GP_SN65DSI83_EN, 0);
	gpio_direction_input(GPIRQ_CSI1_TC3587);
	gpio_direction_output(GP_CSI1_OV5640_MIPI_RESET, 0);
	gpio_direction_input(GPIRQ_CSI2_TC3587);
	gpio_direction_output(GP_CSI2_OV5640_MIPI_RESET, 0);

	return 0;
}

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int board_usb_hub_gpio_init(void)
{
#define GP_USB1_HUB_RESET	IMX_GPIO_NR(1, 14)
	imx_iomux_v3_setup_pad(IMX8MQ_PAD_GPIO1_IO14__GPIO1_IO14 |
			MUX_PAD_CTRL(WEAK_PULLUP));
	return GP_USB1_HUB_RESET;
}
#endif

#ifdef CONFIG_CMD_FBPANEL

#ifdef CONFIG_VIDEO_IMX8M_HDMI
int board_detect_hdmi(struct display_info_t const *di)
{
	return hdmi_hpd_status() ? 1 : 0;
}
#endif

int board_detect_gt911(struct display_info_t const *di)
{
	return board_detect_gt911_common(di, 0, 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
}

int board_detect_gt911_sn65(struct display_info_t const *di)
{
	return board_detect_gt911_sn65_common(di, 0, 0, GP_TS_GT911_RESET, GPIRQ_TS_GT911);
}

static const struct display_info_t displays[] = {
#ifdef CONFIG_VIDEO_IMX8M_HDMI
	/* hdmi */
	VD_1920_1080M_60(HDMI, board_detect_hdmi, 0, 0x50),
	VD_1280_720M_60(HDMI, NULL, 0, 0x50),
#endif
	VD_MIPI_M101NWWB_2(MIPI, board_detect_gt911_sn65, fbp_bus_gp(3, GP_SN65DSI83_EN, 0, 0), 0x5d, FBP_MIPI_TO_LVDS, FBTS_GOODIX),
	VD_MIPI_M101NWWB(MIPI, board_detect_sn65_and_ts, fbp_bus_gp(3, GP_SN65DSI83_EN, GP_TS_FT5X06_RESET, 0), 0x38, FBP_MIPI_TO_LVDS, FBTS_FT5X06),
	VD_DMT050WVNXCMI(MIPI, fbp_detect_i2c, fbp_bus_gp(3, GP_SC18IS602B_RESET, 0, 30), fbp_addr_gp(0x2f, 0, 6, 0), FBP_SPI_LCD, FBTS_GOODIX),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(3, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d, FBTS_GOODIX),	/* Goodix touchscreen */
	VD_LCM_JM430(MIPI, fbp_detect_i2c, fbp_bus_gp(3, GP_ST1633_RESET, GP_TC358762_EN, 30), fbp_addr_gp(0x55, GP_LCM_JM430_BKL_EN, 0, 0), FBTS_ST1633I),		/* Sitronix touch */
	VD_LTK0680YTMDB(MIPI, NULL, fbp_bus_gp(3, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d, FBTS_GOODIX),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp(3, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x00),
	VD_MIPI_TCXD070IBLMAT77(MIPI, fbp_detect_i2c, fbp_bus_gp(3, GP_TS_ILI251X_RESET, GP_MIPI_RESET, 0), fbp_addr_gp(0x41, GP_TCXD070_BKL_EN, 0, 0), FBTS_ILI251X),
	/* 0x3e is the TPS65132 power chip on our adapter board */
	VD_MIPI_LCD133_070(MIPI, board_detect_lcd133, fbp_bus_gp(3, GP_LCD133_070_ENABLE, GP_LCD133_070_ENABLE, 1), fbp_addr_gp(0x3e, 0, 0, 0), FBTS_FT7250),
	VD_MIPI_MQ_1920_1080M_60(MIPI, board_detect_pca9546, fbp_bus_gp((3 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_MQ_1280_800M_60(MIPI, NULL, fbp_bus_gp((3 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_MQ_1280_720M_60(MIPI, NULL, fbp_bus_gp((3 | (3 << 4)), 0, 0, 0), fbp_addr_gp(0x68,0,8,0), FBP_PCA9546),
	VD_MIPI_MQ_1024_768M_60(MIPI, NULL, fbp_bus_gp((3 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_MQ_800_600MR_60(MIPI, NULL, fbp_bus_gp((3 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_MQ_720_480M_60(MIPI, NULL, fbp_bus_gp((3 | (3 << 4)), 0, 0, 0), 0x68, FBP_PCA9546),
	VD_MIPI_MQ_640_480M_60(MIPI, NULL, fbp_bus_gp((3 | (3 << 4)), 0, 0, 0), fbp_addr_gp(0x68,0,16,0), FBP_PCA9546),
	VD_MIPI_MQ_VTFT101RPFT20(MIPI, fbp_detect_i2c, 3, 0x70, FBP_PCA9540),
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
#endif
	gpio_request(GP_TS_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_TS_GT911, "gt911_irq");
	gpio_direction_output(GP_TS_GT911_RESET, 0);
#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif
	board_usb_reset(0, USB_INIT_DEVICE);

	return 0;
}

int board_fastboot_key_pressed(void)
{
	gpio_request(GP_FASTBOOT_KEY, "fastboot_key");
	gpio_direction_input(GP_FASTBOOT_KEY);
	return !gpio_get_value(GP_FASTBOOT_KEY);
}

void board_env_init(void)
{
#ifdef CONFIG_DM_VIDEO
	int ret = gpio_request(GP_LCD133_070_RESET, "bus_gp");

	if (!ret) {
		/*
		 * A mipi panel may have requested, only modify if not owned by
		 * sn65/ltk08
		 */
		/* An unmodified panel has reset connected directly to 1.8V, so make input */
		gpio_direction_input(GP_LCD133_070_RESET);
		gpio_free(GP_LCD133_070_RESET);
	}
#else
	/* An unmodified panel has reset connected directly to 1.8V, so make input */
	gpio_direction_input(GP_LCD133_070_RESET);
#endif
	/*
	 * If touchscreen reset is low, display will not initialize, but runs fine
	 * after init independent of gpio level
	 */
	gpio_direction_output(GP_TS_FT7250_RESET, 1); /* GP_TS_GT911_RESET */
}

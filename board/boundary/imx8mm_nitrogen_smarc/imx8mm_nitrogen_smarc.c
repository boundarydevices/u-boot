// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Ezurio
 */

#include <efi_loader.h>
#include <env.h>
#include <init.h>
#include <linux/delay.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <i2c.h>
#include <asm/io.h>
#include <usb.h>
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

static iomux_v3_cfg_t const init_pads[] = {
	IMX8MM_PAD_SAI5_RXD3_GPIO3_IO24 | MUX_PAD_CTRL(PAD_CTL_PE),	/* MIPI_IRQ - SM_CARRIER_STANDBY */
	IMX8MM_PAD_GPIO1_IO01_GPIO1_IO1 | MUX_PAD_CTRL(PAD_CTL_PE | PAD_CTL_HYS),	/* MIPI_TS_IRQ  - SM_LCD0_BKLT_EN */
#define GP_SM_LCD0_BKLT_EN IMX_GPIO_NR(1, 1)
	IMX8MM_PAD_SAI1_RXD3_GPIO4_IO5 | MUX_PAD_CTRL(PAD_CTL_PE),	/* MIPI_TS_RESET - SM_GPIO7 */
	IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* MIPI_ENABLE - SM_LCD0_VDD_EN */
	IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* SM_ESPI_ALERT0 */
#define GP_TS_LVDS_GT911_RESET  IMX_GPIO_NR(1, 10)
#define GP_TS_LVDS_FT5X06_RESET IMX_GPIO_NR(1, 10)
	IMX8MM_PAD_SAI5_RXD1_GPIO3_IO22 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* SM_SMB_ALERT */
#define GPIRQ_TS_LVDS_GT911     IMX_GPIO_NR(3, 22)
	IMX8MM_PAD_SAI5_MCLK_GPIO3_IO25 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* SN65DSI83 on som  */
#define GP_SN65DSI83_LVDS_EN    IMX_GPIO_NR(3, 25)
	IMX8MM_PAD_SAI5_RXD2_GPIO3_IO23 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* RV3028 - SM_GPIO12 */
	IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2 | MUX_PAD_CTRL(PAD_CTL_PE),	/* SM_PCIE_A_RST */
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(PAD_CTL_PUE | PAD_CTL_DSE1),	/* EMMC_RESET */
};

#if IS_ENABLED(CONFIG_FEC_MXC)
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
	    (struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

#ifdef CONFIG_CMD_FBPANEL
int board_detect_lvds_gt911(struct display_info_t const *di)
{
	/*						TS RESET	TS IRQ	*/
	return board_detect_gt911_common(di, 0, 0, GP_TS_LVDS_GT911_RESET, GPIRQ_TS_LVDS_GT911);
}

static const struct display_info_t displays[] = {
        /* on som mipi-to-lvds coverter */
        VD_MIPI_TM070JDHG30_x("tm070jdhg30-5", UT, MIPI, board_detect_lvds_gt911, fbp_bus_gp(3, 0, GP_SN65DSI83_LVDS_EN, 0), 0x14, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_GOODIX),
        VD_MIPI_TM070JDHG30_x("tm070jdhg30-6", BT, MIPI, NULL, fbp_bus_gp(3, 0, GP_SN65DSI83_LVDS_EN, 0), 0x14, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_GOODIX),
        VD_MIPI_TM070JDHG30_x("tm070jdhg30-7", ET, MIPI, NULL, fbp_bus_gp(3, 0, GP_SN65DSI83_LVDS_EN, 0), 0x14, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_GOODIX),
        VD_MIPI_TM070JDHG30_x("tm070jdhg30-8", UT, MIPI, board_detect_lvds_gt911, fbp_bus_gp(3, 0, GP_SN65DSI83_LVDS_EN, 0), 0x5d, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_GOODIX2),
        VD_MIPI_TM070JDHG30_x("tm070jdhg30-9", BT, MIPI, fbp_detect_i2c, fbp_bus_gp(3, GP_TS_LVDS_FT5X06_RESET, GP_SN65DSI83_LVDS_EN, 0), 0x38, FBP_BACKLIGHT_MIPI_ALT, FBTS_LVDS_FT5X06),
};
#define display_cnt     ARRAY_SIZE(displays)
#else
#define displays        NULL
#define display_cnt     0

#endif

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_request(GP_SN65DSI83_LVDS_EN, "sn65en");
	gpio_direction_output(GP_SN65DSI83_LVDS_EN, 1);

	gpio_request(GP_SM_LCD0_BKLT_EN, "lcd0_bklt_en");
	gpio_direction_output(GP_SM_LCD0_BKLT_EN, 1);

	gpio_request(IMX_GPIO_NR(2, 10), "emmc reset");
	gpio_direction_output(IMX_GPIO_NR(2, 10), 1);
	return 0;
}

int board_phys_sdram_size(phys_size_t *size)
{
	if (!size)
		return -EINVAL;

	*size = get_ram_size((void *)CFG_SYS_SDRAM_BASE, SZ_4G);	/* Maximum 4G */

	return 0;
}

int board_init(void)
{
	gpio_request(IMX_GPIO_NR(3, 23), "rv3028_irq");
	gpio_request(IMX_GPIO_NR(4, 5), "gt911_reset");
	gpio_request(GP_TS_LVDS_GT911_RESET, "lvds_gt911_reset");
	gpio_request(GPIRQ_TS_LVDS_GT911, "lvds_gt911_irq");
#ifndef CONFIG_VIDEO
	gpio_request(IMX_GPIO_NR(1, 3), "sn65dsi83_enable");
	/* This enables 5V power on LTK080A60A004T mipi display */
	gpio_request(IMX_GPIO_NR(3, 24), "lkt08_mipi_en");
#endif
	gpio_direction_output(IMX_GPIO_NR(4, 5), 0);
	gpio_direction_output(IMX_GPIO_NR(1, 10), 0);

	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();
#ifdef CONFIG_CMD_FBPANEL
        fbp_setup_display(displays, display_cnt);
#endif
	return 0;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

int board_late_init(void)
{
	if (!env_get("serial#")) {
		unsigned char mac_address[8];
		char serialbuf[20];

		imx_get_mac_from_fuse(0, mac_address);
		snprintf(serialbuf, sizeof(serialbuf),
			 "%02x%02x%02x%02x%02x%02x", mac_address[0],
			 mac_address[1], mac_address[2], mac_address[3],
			 mac_address[4], mac_address[5]);
		printf("serial: %s\n", serialbuf);
		env_set("serial#", serialbuf);
	}

	return 0;
}

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void)
{
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0;		/* TODO */
}
#endif /* CONFIG_ANDROID_RECOVERY */
#endif /* CONFIG_FSL_FASTBOOT */

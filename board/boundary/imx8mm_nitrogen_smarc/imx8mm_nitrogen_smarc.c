// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */
#include <common.h>
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
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <i2c.h>
#include <asm/io.h>
#include <usb.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#if 0
static iomux_v3_cfg_t const eth_strap_pads[] = {
	IMX8MM_PAD_ENET_RXC_GPIO1_IO25 | MUX_PAD_CTRL(PAD_CTL_PUE),
	IMX8MM_PAD_NAND_CLE_GPIO3_IO5	| MUX_PAD_CTRL(PAD_CTL_PUE),
};

static iomux_v3_cfg_t const eth_normal_pads[] = {
	IMX8MM_PAD_ENET_RXC_ENET1_RGMII_RXC | MUX_PAD_CTRL(PAD_CTL_FSEL2),
};
#endif

#define GP_ETH_STRAP			IMX_GPIO_NR(1, 25)
#define GP_RGMII_PHY_RESET		IMX_GPIO_NR(3, 15)

static iomux_v3_cfg_t const init_pads[] = {
#define GPIRQ_I2C2_SN65DSI83		IMX_GPIO_NR(3, 24)
#define GP_CS005_0004_03_DISPLAY_EN	IMX_GPIO_NR(3, 24)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(3, 24)
#define GP_LS050T1SX12_EN		IMX_GPIO_NR(3, 24)
#define GP_LT8912_DISPLAY_EN		IMX_GPIO_NR(3, 24)
	IMX8MM_PAD_SAI5_RXD3_GPIO3_IO24 | MUX_PAD_CTRL(PAD_CTL_PE), /* MIPI_IRQ - SM_CARRIER_STANDBY */

#define GP_LVDS_BACKLIGHT_EN		IMX_GPIO_NR(1, 1)
#define GPIRQ_TS_GT911 			IMX_GPIO_NR(1, 1)
	IMX8MM_PAD_GPIO1_IO01_GPIO1_IO1 | MUX_PAD_CTRL(PAD_CTL_PE | PAD_CTL_HYS), /* MIPI_TS_IRQ  - SM_LCD0_BKLT_EN */

#define GP_TS_ATMEL_RESET		IMX_GPIO_NR(4, 5)
#define GP_TS_GT911_RESET		IMX_GPIO_NR(4, 5)
#define GP_ST1633_RESET			IMX_GPIO_NR(4, 5)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(4, 5)
#define GP_TS_ILI251X_RESET		IMX_GPIO_NR(4, 5)
	IMX8MM_PAD_SAI1_RXD3_GPIO4_IO5 | MUX_PAD_CTRL(PAD_CTL_PE), /* MIPI_TS_RESET - SM_GPIO7 */

#define GP_TC358762_EN		IMX_GPIO_NR(1, 3)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(1, 3)
#define GP_DMT055FHNMCMI_EN	IMX_GPIO_NR(1, 3)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(1, 3)
#define GP_MIPI_ENABLE		IMX_GPIO_NR(1, 3)
#define	GP_LT8912_RESET		IMX_GPIO_NR(1, 3)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(1, 3)
	IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4), /* MIPI_ENABLE - SM_LCD0_VDD_EN */

#define GP_TS_LVDS_GT911_RESET	IMX_GPIO_NR(1, 10)
#define GP_TS_LVDS_FT5X06_RESET	IMX_GPIO_NR(1, 10)
	IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4), /* SM_ESPI_ALERT0 */
#define GPIRQ_TS_LVDS_GT911	IMX_GPIO_NR(3, 22)
	IMX8MM_PAD_SAI5_RXD1_GPIO3_IO22 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4), /* SM_SMB_ALERT */

#define GP_SN65DSI83_LVDS_EN	IMX_GPIO_NR(3, 25)
	IMX8MM_PAD_SAI5_MCLK_GPIO3_IO25 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4), /* on som  */

#define GPIRQ_RV3028		IMX_GPIO_NR(3, 23)
	IMX8MM_PAD_SAI5_RXD2_GPIO3_IO23 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4), /* SM_GPIO12 */

	/* pcie */
#define GP_PCIE0_RESET		IMX_GPIO_NR(3, 2)
	IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2 | MUX_PAD_CTRL(PAD_CTL_PE), /* SM_PCIE_A_RST */

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(PAD_CTL_PUE | PAD_CTL_DSE1),
};



#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX8MM-NIT-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 2=flash-bin raw 0x42 0x2000 mmcpart 1",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};

#endif /* EFI_HAVE_CAPSULE_SUPPORT */

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

#ifndef CONFIG_DM_ETH
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);
#endif

	return 0;
}
#endif

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

        imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

        set_wdog_reset(wdog);

        imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

        init_uart_clk(1);

	gpio_request(GP_SN65DSI83_EN, "sn65en");
	gpio_direction_output(GP_SN65DSI83_EN, 0);
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(GP_EMMC_RESET, 1);
	return 0;
}

int board_phys_sdram_size(phys_size_t *size)
{
	if (!size)
		return -EINVAL;

	*size = get_ram_size((void *)CFG_SYS_SDRAM_BASE, SZ_4G); /* Maximum 4G */

	return 0;
}

int board_init(void)
{
	gpio_request(GPIRQ_RV3028, "rv3028_irq");
	gpio_request(GP_TS_GT911_RESET, "gt911_reset");
	gpio_request(GP_TS_LVDS_GT911_RESET, "lvds_gt911_reset");
	gpio_request(GPIRQ_TS_LVDS_GT911, "lvds_gt911_irq");
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
#endif
	gpio_direction_output(GP_TS_GT911_RESET, 0);
	gpio_direction_output(GP_TS_LVDS_GT911_RESET, 0);

	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /* TODO */
}
#endif /* CONFIG_ANDROID_RECOVERY */
#endif /* CONFIG_FSL_FASTBOOT */

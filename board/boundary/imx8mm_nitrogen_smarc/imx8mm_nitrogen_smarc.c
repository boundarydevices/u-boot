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
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

static iomux_v3_cfg_t const init_pads[] = {
	IMX8MM_PAD_SAI5_RXD3_GPIO3_IO24 | MUX_PAD_CTRL(PAD_CTL_PE),			/* MIPI_IRQ - SM_CARRIER_STANDBY */
	IMX8MM_PAD_GPIO1_IO01_GPIO1_IO1 | MUX_PAD_CTRL(PAD_CTL_PE | PAD_CTL_HYS),	/* MIPI_TS_IRQ  - SM_LCD0_BKLT_EN */
	IMX8MM_PAD_SAI1_RXD3_GPIO4_IO5 | MUX_PAD_CTRL(PAD_CTL_PE),			/* MIPI_TS_RESET - SM_GPIO7 */
	IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* MIPI_ENABLE - SM_LCD0_VDD_EN */
	IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* SM_ESPI_ALERT0 */
	IMX8MM_PAD_SAI5_RXD1_GPIO3_IO22 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* SM_SMB_ALERT */
	IMX8MM_PAD_SAI5_MCLK_GPIO3_IO25 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* SN65DSI83 on som  */
	IMX8MM_PAD_SAI5_RXD2_GPIO3_IO23 | MUX_PAD_CTRL(PAD_CTL_DSE2 | PAD_CTL_DSE4),	/* RV3028 - SM_GPIO12 */
	IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2 | MUX_PAD_CTRL(PAD_CTL_PE),			/* SM_PCIE_A_RST */
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

//#ifndef CONFIG_DM_ETH
#if 0
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

	gpio_request(IMX_GPIO_NR(1, 3), "sn65en");
	gpio_direction_output(IMX_GPIO_NR(1, 3), 0);
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(IMX_GPIO_NR(2, 10), 1);
	return 0;
}

int board_phys_sdram_size(phys_size_t * size)
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
	gpio_request(IMX_GPIO_NR(1, 10), "lvds_gt911_reset");
	gpio_request(IMX_GPIO_NR(3, 22), "lvds_gt911_irq");
#ifndef CONFIG_DM_VIDEO
	gpio_request(IMX_GPIO_NR(1, 3), "sn65dsi83_enable");
	/* This enables 5V power on LTK080A60A004T mipi display */
	gpio_request(IMX_GPIO_NR(3, 24), "lkt08_mipi_en");
#endif
	gpio_direction_output(IMX_GPIO_NR(4, 5), 0);
	gpio_direction_output(IMX_GPIO_NR(1, 10), 0);

	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	return 0;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

int board_late_init(void)
{
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

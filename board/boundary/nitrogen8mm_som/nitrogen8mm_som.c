/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <asm/io.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/dma.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <env.h>
#include <dm.h>
#include <i2c.h>
#include <linux/delay.h>
#include <spl.h>

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
#define GPIRQ_I2C2_SN65DSI83		IMX_GPIO_NR(1, 1)
#define GP_CS005_0004_03_DISPLAY_EN	IMX_GPIO_NR(1, 1)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
#define GP_LS050T1SX12_EN		IMX_GPIO_NR(1, 1)
#define GP_LT8912_DISPLAY_EN		IMX_GPIO_NR(1, 1)
	IMX8MM_PAD_GPIO1_IO01_GPIO1_IO1 | MUX_PAD_CTRL(PAD_CTL_PE |
						       PAD_CTL_DSE6),
#define GPIRQ_TS_GT911 			IMX_GPIO_NR(1, 6)
	IMX8MM_PAD_GPIO1_IO06_GPIO1_IO6 | MUX_PAD_CTRL(PAD_CTL_PE |
						       PAD_CTL_HYS),
#define GP_TS_ATMEL_RESET		IMX_GPIO_NR(1, 7)
#define GP_TS_GT911_RESET		IMX_GPIO_NR(1, 7)
#define GP_ST1633_RESET			IMX_GPIO_NR(1, 7)
#define GP_TS_FT5X06_RESET		IMX_GPIO_NR(1, 7)
#define GP_TS_ILI251X_RESET		IMX_GPIO_NR(1, 7)
	IMX8MM_PAD_GPIO1_IO07_GPIO1_IO7 | MUX_PAD_CTRL(PAD_CTL_PE),

#define GP_CS005_0004_03_BKL_EN	IMX_GPIO_NR(5, 0)
#define GP_TC358762_EN		IMX_GPIO_NR(5, 0)
#define GP_SC18IS602B_RESET	IMX_GPIO_NR(5, 0)
#define GP_DMT055FHNMCMI_EN	IMX_GPIO_NR(5, 0)
#define GP_SN65DSI83_EN		IMX_GPIO_NR(5, 0)
#define GP_MIPI_RESET		IMX_GPIO_NR(5, 0)
#define	GP_LT8912_RESET		IMX_GPIO_NR(5, 0)
/* enable for TPS65132 Single Inductor - Dual Output Power Supply */
#define GP_LCD133_070_ENABLE		IMX_GPIO_NR(5, 0)
	IMX8MM_PAD_SAI3_TXC_GPIO5_IO0 | MUX_PAD_CTRL(PAD_CTL_DSE6),

#define GPIRQ_RV4162		<&gpio4 22 IRQ_TYPE_LEVEL_LOW>
	IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3 | MUX_PAD_CTRL(PAD_CTL_PE | PAD_CTL_HYS
						       | PAD_CTL_PUE),

#define GP_CSI1_MIPI_PWDN	IMX_GPIO_NR(1, 11)
#define GP_5P5_EN		IMX_GPIO_NR(1, 11)
	IMX8MM_PAD_GPIO1_IO11_GPIO1_IO11 | MUX_PAD_CTRL(PAD_CTL_PE |
							PAD_CTL_PUE),
#define GPIRQ_TC358743		IMX_GPIO_NR(1, 9)	/* TG carrier board */
#define GP_CSI1_MIPI_RESET	IMX_GPIO_NR(1, 9)
	IMX8MM_PAD_GPIO1_IO09_GPIO1_IO9 | MUX_PAD_CTRL(PAD_CTL_PE),

	/* pcie */
#define GP_PCIE0_RESET		IMX_GPIO_NR(4, 31)
	IMX8MM_PAD_SAI3_TXFS_GPIO4_IO31 | MUX_PAD_CTRL(PAD_CTL_PE),
#define GP_PCIE0_DISABLE	IMX_GPIO_NR(1, 4)
	IMX8MM_PAD_GPIO1_IO04_GPIO1_IO4 | MUX_PAD_CTRL(PAD_CTL_PE),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(1, 10)
	IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(PAD_CTL_HYS),
#define GP_WM8960_HP_DET	IMX_GPIO_NR(4, 28)
	IMX8MM_PAD_SAI3_RXFS_GPIO4_IO28 | MUX_PAD_CTRL(PAD_CTL_HYS),
#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(PAD_CTL_PUE |
							 PAD_CTL_DSE1),

	IMX8MM_PAD_GPIO1_IO14_USB2_OTG_PWR | MUX_PAD_CTRL(PAD_CTL_FSEL1 |
							  PAD_CTL_DSE6),
	/* GPIO15 is used for CCM_CLKO2, GPIO1_IO08 is overcurrent */
	IMX8MM_PAD_GPIO1_IO08_GPIO1_IO8 | MUX_PAD_CTRL(PAD_CTL_PUE |
						       PAD_CTL_PE),

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
	set_wdog_reset(wdog);
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
	gpio_request(GP_TS_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_TS_GT911, "gt911_irq");
#ifndef CONFIG_DM_VIDEO
	gpio_request(GP_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
#endif
//      gpio_request(GP_CSI1_MIPI_PWDN, "csi1_mipi_pwdn");
	gpio_request(GP_CSI1_MIPI_RESET, "csi1_mipi_reset");
	gpio_direction_output(GP_TS_GT911_RESET, 0);
	/* Rely on pull up only, the toshiba hdmi input uses as IRQ */
//      gpio_direction_output(GP_CSI1_MIPI_PWDN, 1);
	/* Rely on pull down only, the TG carrier toshiba hdmi input uses as IRQ */
//      gpio_direction_output(GP_CSI1_MIPI_RESET, 0);
#if defined(CONFIG_MXC_SPI) && !defined(CONFIG_DM_SPI)
	setup_spi();
#endif
	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();
#ifdef CONFIG_FSL_FSPI
	board_qspi_init();
#endif

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

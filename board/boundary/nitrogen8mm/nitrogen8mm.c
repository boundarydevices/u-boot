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
	IMX8MM_PAD_GPIO1_IO01_GPIO1_IO1 | MUX_PAD_CTRL(PAD_CTL_PE |
						       PAD_CTL_DSE6),
	IMX8MM_PAD_GPIO1_IO06_GPIO1_IO6 | MUX_PAD_CTRL(PAD_CTL_PE |
						       PAD_CTL_HYS),
	IMX8MM_PAD_GPIO1_IO07_GPIO1_IO7 | MUX_PAD_CTRL(PAD_CTL_PE),
	IMX8MM_PAD_SAI3_TXC_GPIO5_IO0 | MUX_PAD_CTRL(PAD_CTL_DSE6),
	IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3 | MUX_PAD_CTRL(PAD_CTL_PE | PAD_CTL_HYS
						       | PAD_CTL_PUE),

	IMX8MM_PAD_GPIO1_IO11_GPIO1_IO11 | MUX_PAD_CTRL(PAD_CTL_PE |
							PAD_CTL_PUE),
	IMX8MM_PAD_GPIO1_IO09_GPIO1_IO9 | MUX_PAD_CTRL(PAD_CTL_PE),

	IMX8MM_PAD_SAI3_TXFS_GPIO4_IO31 | MUX_PAD_CTRL(PAD_CTL_PE),
	IMX8MM_PAD_GPIO1_IO04_GPIO1_IO4 | MUX_PAD_CTRL(PAD_CTL_PE),

	IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(PAD_CTL_HYS),
	IMX8MM_PAD_SAI3_RXFS_GPIO4_IO28 | MUX_PAD_CTRL(PAD_CTL_HYS),
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(PAD_CTL_PUE |
							 PAD_CTL_DSE1),

	IMX8MM_PAD_GPIO1_IO14_USB2_OTG_PWR | MUX_PAD_CTRL(PAD_CTL_FSEL1 |
							  PAD_CTL_DSE6),
#ifdef CONFIG_TARGET_NITROGEN8MM_SOM
	/* GPIO15 is used for CCM_CLKO2, GPIO1_IO08 is overcurrent */
	IMX8MM_PAD_GPIO1_IO08_GPIO1_IO8 | MUX_PAD_CTRL(PAD_CTL_PUE |
						       PAD_CTL_PE),
#else
	/* SBC */
	IMX8MM_PAD_GPIO1_IO15_USB2_OTG_OC | MUX_PAD_CTRL(PAD_CTL_PUE |
							PAD_CTL_PE),
#endif

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

	gpio_request(IMX_GPIO_NR(5, 0), "sn65en");
	gpio_direction_output(IMX_GPIO_NR(5, 0), 0);
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(IMX_GPIO_NR(2, 10), 1);
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
	gpio_request(IMX_GPIO_NR(1, 7), "gt911_reset");
	gpio_request(IMX_GPIO_NR(1, 6), "gt911_irq");
#ifndef CONFIG_DM_VIDEO
	gpio_request(IMX_GPIO_NR(5, 0), "sn65dsi83_enable");
	gpio_request(IMX_GPIO_NR(1, 1), "lkt08_mipi_en");
#endif
	gpio_request(IMX_GPIO_NR(1, 9), "csi1_mipi_reset");
	gpio_direction_output(IMX_GPIO_NR(1, 7), 0);
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
	if (IS_ENABLED(CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG)) {
#ifdef CONFIG_TARGET_NITROGEN8MM_SOM
		env_set("board", "nitrogen8mm_som");
		env_set("board_name", "nitrogen8mm_som");
#elif CONFIG_TARGET_NITROGEN8MM
		env_set("board", "nitrogen8mm_rev2");
		env_set("board_name", "nitrogen8mm_rev2");
#endif
	}
	if (!env_get("serial#")) {
		unsigned char mac_address[8];
		char serialbuf[20];

		imx_get_mac_from_fuse(0, mac_address);
		snprintf(serialbuf, sizeof(serialbuf), "%02x%02x%02x%02x%02x%02x",
				mac_address[0], mac_address[1], mac_address[2],
				mac_address[3], mac_address[4], mac_address[5]);
		printf("serial: %s\n", serialbuf);
		env_set("serial#", serialbuf);
	}

	return 0;
}

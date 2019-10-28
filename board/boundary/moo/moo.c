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
#include <asm/arch/imx8mm_pins.h>
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
#include <usb.h>
#include "../common/padctrl.h"
#include "../common/bd_common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

static iomux_v3_cfg_t const init_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
	IMX8MM_PAD_UART1_RXD_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART1_TXD_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),

	/* sound - wm8960 */
#define GP_WM8960_AMP_STDBY	IMX_GPIO_NR(3, 22)	/* Low is standby */
	IMX8MM_PAD_SAI5_RXD1_GPIO3_IO22 | MUX_PAD_CTRL(0x100),
#define GP_WM8960_AMP_MUTE	IMX_GPIO_NR(3, 21)	/* Low is muted */
	IMX8MM_PAD_SAI5_RXD0_GPIO3_IO21 | MUX_PAD_CTRL(0x100),
#define GP_WM8960_AMP_G0	IMX_GPIO_NR(4, 8)
	IMX8MM_PAD_SAI1_RXD6_GPIO4_IO8 | MUX_PAD_CTRL(0x100),
#define GP_WM8960_AMP_G1	IMX_GPIO_NR(4, 9)
	IMX8MM_PAD_SAI1_RXD7_GPIO4_IO9 | MUX_PAD_CTRL(0x100),
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(1, 10)
	IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(0x1c0),
#define GP_WM8960_HP_DET	IMX_GPIO_NR(4, 28)
	IMX8MM_PAD_SAI3_RXFS_GPIO4_IO28 | MUX_PAD_CTRL(0x1c0),

#define GP_OTG2_HUB_RESET	IMX_GPIO_NR(4, 19)
	IMX8MM_PAD_SAI1_TXD7_GPIO4_IO19 | MUX_PAD_CTRL(0x100),
#define GP_MODEM_ON		IMX_GPIO_NR(3, 5)
	IMX8MM_PAD_NAND_CLE_GPIO3_IO5 | MUX_PAD_CTRL(0x100),
#define GP_MODEM_RESET		IMX_GPIO_NR(3, 14)
	IMX8MM_PAD_NAND_DQS_GPIO3_IO14 | MUX_PAD_CTRL(0x100),

#ifdef CONFIG_FEC_MXC
	/* PHY - AR8035 */
#ifdef CONFIG_FEC_PHY_BITBANG
#define GP_MII_MDIO	IMX_GPIO_NR(1, 17)
	IMX8MM_PAD_ENET_MDIO_GPIO1_IO17 | MUX_PAD_CTRL(0),
#define GP_MII_MDC	IMX_GPIO_NR(1, 16)
	IMX8MM_PAD_ENET_MDC_GPIO1_IO16 | MUX_PAD_CTRL(0),
#else
	IMX8MM_PAD_ENET_MDIO_ENET1_MDIO | MUX_PAD_CTRL(PAD_CTRL_ENET_MDIO),
	IMX8MM_PAD_ENET_MDC_ENET1_MDC | MUX_PAD_CTRL(PAD_CTRL_ENET_MDC),
#endif
	IMX8MM_PAD_ENET_TX_CTL_ENET1_RGMII_TX_CTL | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MM_PAD_ENET_TD0_ENET1_RGMII_TD0 | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MM_PAD_ENET_TD1_ENET1_RGMII_TD1 | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MM_PAD_ENET_TD2_ENET1_RGMII_TD2 | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MM_PAD_ENET_TD3_ENET1_RGMII_TD3 | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
	IMX8MM_PAD_ENET_TXC_ENET1_RGMII_TXC | MUX_PAD_CTRL(PAD_CTRL_ENET_TX),
#endif

#define GP_EMMC_RESET	IMX_GPIO_NR(2, 10)
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(0x100),
};


int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(GP_EMMC_RESET, 1);
	set_wdog_reset(wdog);
	return 0;
}

#ifdef CONFIG_BOARD_POSTCLK_INIT
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif

int dram_init(void)
{
	/* rom_pointer[1] contains the size of TEE occupies */
	if (rom_pointer[1])
		gd->ram_size = PHYS_SDRAM_SIZE - rom_pointer[1];
	else
		gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return 0;
}
#endif

int board_init(void)
{
#ifdef CONFIG_FEC_PHY_BITBANG
	gpio_request(GP_MII_MDC, "mii_mdc");
	gpio_request(GP_MII_MDIO, "mii_mdio");
#endif
#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#ifdef CONFIG_FSL_FSPI
	board_qspi_init();
#endif

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand(); /* SPL will call the board_early_init_f */
#endif
	return 0;
}

static void set_env_vars(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (!env_get("board"))
		env_set("board", "moo");
	env_set("soc", "imx8mm");
	env_set("imx_cpu", get_imx_type((get_cpu_rev() & 0xFF000) >> 12));
	env_set("uboot_defconfig", CONFIG_DEFCONFIG);
#endif
}

void board_set_default_env(void)
{
	set_env_vars();
	board_eth_addresses();
}

int board_usb_init(int index, enum usb_init_type init)
{
	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, false);
	return 0;
}

int board_late_init(void)
{
	set_env_vars();
#ifdef CONFIG_ENV_IS_IN_MMC
	env_set_ulong("mmcdev", 0);
	run_command("mmc dev 0", 0);
#endif
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

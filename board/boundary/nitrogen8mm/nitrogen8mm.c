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
#include <fsl_esdhc.h>
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
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),

#define GP_LCM_JM430_BKL_EN		IMX_GPIO_NR(1, 1)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
	IMX8MM_PAD_GPIO1_IO01_GPIO1_IO1 | MUX_PAD_CTRL(0x16),

#define GPIRQ_GT911 			IMX_GPIO_NR(1, 6)
	IMX8MM_PAD_GPIO1_IO06_GPIO1_IO6 | MUX_PAD_CTRL(0xd6),
#define GP_GT911_RESET			IMX_GPIO_NR(1, 7)
#define GP_ST1633_RESET			IMX_GPIO_NR(1, 7)
	IMX8MM_PAD_GPIO1_IO07_GPIO1_IO7 | MUX_PAD_CTRL(0x49),

#define GP_TC358762_EN		IMX_GPIO_NR(1, 9)
#define GP_I2C2_SN65DSI83_EN	IMX_GPIO_NR(1, 9)
#define GP_MIPI_RESET		IMX_GPIO_NR(1, 9)
	IMX8MM_PAD_GPIO1_IO09_GPIO1_IO9 | MUX_PAD_CTRL(0x06),
	IMX8MM_PAD_SAI2_RXC_GPIO4_IO22 | MUX_PAD_CTRL(PAD_CTL_DSE1 | PAD_CTL_ODE),

#define GP_CSI1_MIPI_PWDN	IMX_GPIO_NR(1, 3)
	IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3 | MUX_PAD_CTRL(0x141),
#define GP_CSI1_MIPI_RESET	IMX_GPIO_NR(1, 4)
	IMX8MM_PAD_GPIO1_IO04_GPIO1_IO4 | MUX_PAD_CTRL(0x101),

	/* pcie */
#define GP_PCIE0_RESET		IMX_GPIO_NR(4, 31)
	IMX8MM_PAD_SAI3_TXFS_GPIO4_IO31 | MUX_PAD_CTRL(0x100),
#define GP_PCIE0_DISABLE	IMX_GPIO_NR(1, 5)
	IMX8MM_PAD_GPIO1_IO05_GPIO1_IO5 | MUX_PAD_CTRL(0x100),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(1, 10)
	IMX8MM_PAD_GPIO1_IO10_GPIO1_IO10 | MUX_PAD_CTRL(0x80),
#define GP_WM8960_HP_DET	IMX_GPIO_NR(4, 28)
	IMX8MM_PAD_SAI3_RXFS_GPIO4_IO28 | MUX_PAD_CTRL(0x80),

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
	IMX8MM_PAD_SD1_RESET_B_GPIO2_IO10 | MUX_PAD_CTRL(0x41),
};


int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	gpio_request(GP_I2C2_SN65DSI83_EN, "sn65en");
	gpio_direction_output(GP_I2C2_SN65DSI83_EN, 0);
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	gpio_direction_output(GP_EMMC_RESET, 1);
	set_wdog_reset(wdog);
	gpio_direction_output(GP_I2C2_SN65DSI83_EN, 0);
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

#ifdef CONFIG_CMD_FBPANEL

int board_detect_gt911(struct display_info_t const *di)
{
	int ret;
	struct udevice *dev, *chip;

	if (di->bus_gp)
		gpio_direction_output(di->bus_gp, 1);
	gpio_set_value(GP_GT911_RESET, 0);
	mdelay(20);
	gpio_direction_output(GPIRQ_GT911, di->addr_num == 0x14 ? 1 : 0);
	udelay(100);
	gpio_set_value(GP_GT911_RESET, 1);
	mdelay(6);
	gpio_set_value(GPIRQ_GT911, 0);
	mdelay(50);
	gpio_direction_input(GPIRQ_GT911);
	ret = uclass_get_device(UCLASS_I2C, di->bus_num, &dev);
	if (ret)
		return 0;

	ret = dm_i2c_probe(dev, di->addr_num, 0x0, &chip);
	if (ret && di->bus_gp)
		gpio_direction_input(di->bus_gp);
	return (ret == 0);
}

static const struct display_info_t displays[] = {
	VD_MIPI_M101NWWB_NO_CMDS(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_I2C2_SN65DSI83_EN, 0, 0), 0x2c),
	VD_MIPI_M101NWWB(MIPI, NULL, fbp_bus_gp(1, GP_I2C2_SN65DSI83_EN, 0, 0), 0x2c),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(1, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d),	/* Goodix touchscreen */
	VD_LCM_JM430(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_ST1633_RESET, GP_TC358762_EN, 30), fbp_addr_gp(0x55, GP_LCM_JM430_BKL_EN, 0, 0)),		/* Sitronix touch */
	VD_LTK0680YTMDB(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d),
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
	gpio_request(GP_I2C2_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
	gpio_request(GP_CSI1_MIPI_PWDN, "csi1_mipi_pwdn");
	gpio_request(GP_CSI1_MIPI_RESET, "csi1_mipi_reset");
	gpio_direction_output(GP_GT911_RESET, 0);
	gpio_direction_output(GP_CSI1_MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI1_MIPI_RESET, 0);
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

static void set_env_vars(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (!env_get("board"))
		env_set("board", "nitrogen8mm");
	env_set("soc", "imx8mm");
	env_set("imx_cpu", get_imx_type((get_cpu_rev() & 0xFF000) >> 12));
	env_set("uboot_defconfig", CONFIG_DEFCONFIG);
#endif
}

void board_set_default_env(void)
{
	set_env_vars();
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_env_cmds();
#endif
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
	/* No display yet for mini so need to setup cmd_... */
#ifdef CONFIG_CMD_FBPANEL
	board_video_skip();
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

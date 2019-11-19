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
#include <usb.h>
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

#ifdef CONFIG_BOARD_POSTCLK_INIT
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif

#define MAX_LOW_SIZE	(0x100000000ULL - CONFIG_SYS_SDRAM_BASE)
#define SDRAM_SIZE	((1ULL * CONFIG_DDR_MB) << 20)

#if SDRAM_SIZE > MAX_LOW_SIZE
#define MEM_SIZE	MAX_LOW_SIZE
#else
#define MEM_SIZE	SDRAM_SIZE
#endif

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = SDRAM_SIZE;
	return 0;
}

int dram_init(void)
{
	/* rom_pointer[1] contains the size of TEE occupies */
	gd->ram_size = MEM_SIZE - rom_pointer[1];
	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return 0;
}
#endif

#ifdef CONFIG_CMD_FBPANEL


static const struct display_info_t displays[] = {
	VD_MIPI_TM070JDHG30(MIPI, fbp_detect_i2c, fbp_bus_gp(2, 0, GP_LVDS_BKL_EN, 0), 0x38),
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

static void set_env_vars(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (!env_get("board"))
		env_set("board", "hl8mm");
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

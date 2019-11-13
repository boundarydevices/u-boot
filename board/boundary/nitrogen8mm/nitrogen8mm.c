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

#define GP_LCM_JM430_BKL_EN		IMX_GPIO_NR(1, 1)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
	IOMUX_PAD_CTRL(GPIO1_IO01__GPIO1_IO1, 0x16),

#define GPIRQ_GT911 			IMX_GPIO_NR(1, 6)
	IOMUX_PAD_CTRL(GPIO1_IO06__GPIO1_IO6, 0xd6),
#define GP_GT911_RESET			IMX_GPIO_NR(1, 7)
#define GP_ST1633_RESET			IMX_GPIO_NR(1, 7)
	IOMUX_PAD_CTRL(GPIO1_IO07__GPIO1_IO7, 0x49),

#define GP_TC358762_EN		IMX_GPIO_NR(1, 9)
#define GP_I2C2_SN65DSI83_EN	IMX_GPIO_NR(1, 9)
#define GP_MIPI_RESET		IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, 0x06),
	IOMUX_PAD_CTRL(SAI2_RXC__GPIO4_IO22, PAD_CTL_DSE1 | PAD_CTL_ODE),

#define GPIRQ_CSI1_TC3587		IMX_GPIO_NR(1, 8)
#define GP_CSI1_OV5640_MIPI_POWER_DOWN	IMX_GPIO_NR(1, 8)
	IOMUX_PAD_CTRL(GPIO1_IO08__GPIO1_IO8, 0x141),
#define GP_CSI1_OV5640_MIPI_RESET	IMX_GPIO_NR(3, 5)
	IOMUX_PAD_CTRL(NAND_CLE__GPIO3_IO5, 0x101),

	/* pcie */
#define GP_PCIE0_RESET		IMX_GPIO_NR(4, 31)
	IOMUX_PAD_CTRL(SAI3_TXFS__GPIO4_IO31, 0x100),
#define GP_PCIE0_DISABLE	IMX_GPIO_NR(1, 5)
	IOMUX_PAD_CTRL(GPIO1_IO05__GPIO1_IO5, 0x100),

	/* sound - wm8960 */
#define GP_WM8960_MIC_DET	IMX_GPIO_NR(1, 10)
	IOMUX_PAD_CTRL(GPIO1_IO10__GPIO1_IO10, 0x80),
#define GP_WM8960_HP_DET	IMX_GPIO_NR(4, 28)
	IOMUX_PAD_CTRL(SAI3_RXFS__GPIO4_IO28, 0x80),

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
	VD_MIPI_M101NWWB(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_I2C2_SN65DSI83_EN, 0, 0), 0x2c),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, fbp_bus_gp(1, GP_LTK08_MIPI_EN, GP_LTK08_MIPI_EN, 0), 0x5d),	/* Goodix touchscreen */
	VD_LCM_JM430(MIPI, fbp_detect_i2c, fbp_bus_gp(1, GP_ST1633_RESET, GP_TC358762_EN, 30), fbp_addr_gp(0x55, GP_LCM_JM430_BKL_EN, 0, 0)),		/* Sitronix touch */
	VD_LTK0680YTMDB(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x5d),
	VD_MIPI_COM50H5N03ULC(MIPI, NULL, fbp_bus_gp(1, GP_MIPI_RESET, GP_MIPI_RESET, 0), 0x00),
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
	gpio_request(GP_I2C2_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
	gpio_request(GPIRQ_CSI1_TC3587, "csi1_mipi_pwdn");
	gpio_request(GP_CSI1_OV5640_MIPI_RESET, "csi1_mipi_reset");
	gpio_direction_output(GP_GT911_RESET, 0);
	gpio_direction_input(GPIRQ_CSI1_TC3587);
	gpio_direction_output(GP_CSI1_OV5640_MIPI_RESET, 0);
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

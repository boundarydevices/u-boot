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
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/fbpanel.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/mx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/video.h>
#include <video_fb.h>
#include <spl.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <dm.h>
#include <usb.h>
#include <dwc3-uboot.h>

DECLARE_GLOBAL_DATA_PTR;

#define QSPI_PAD_CTRL	(PAD_CTL_DSE2 | PAD_CTL_HYS)

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)
#define WEAK_PULLUP	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

static iomux_v3_cfg_t const init_pads[] = {
#if 0
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
#else
#define GP_WATCHDOG			IMX_GPIO_NR(1, 2)
	IMX8MQ_PAD_GPIO1_IO02__GPIO1_IO2 | MUX_PAD_CTRL(WDOG_PAD_CTRL),
#endif
	IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN		IMX_GPIO_NR(1, 1)
	IMX8MQ_PAD_GPIO1_IO01__GPIO1_IO1 | MUX_PAD_CTRL(0x16),

#define GPIRQ_GT911 			IMX_GPIO_NR(3, 12)
	IMX8MQ_PAD_NAND_DATA06__GPIO3_IO12 | MUX_PAD_CTRL(0xd6),
#define GP_GT911_RESET			IMX_GPIO_NR(3, 13)
	IMX8MQ_PAD_NAND_DATA07__GPIO3_IO13 | MUX_PAD_CTRL(0x49),

#define GP_ARM_DRAM_VSEL		IMX_GPIO_NR(3, 24)
	IMX8MQ_PAD_SAI5_RXD3__GPIO3_IO24 | MUX_PAD_CTRL(0x16),
#define GP_DRAM_1P1_VSEL		IMX_GPIO_NR(2, 11)
	IMX8MQ_PAD_SD1_STROBE__GPIO2_IO11 | MUX_PAD_CTRL(0x16),
#define GP_SOC_GPU_VPU_VSEL		IMX_GPIO_NR(2, 20)
	IMX8MQ_PAD_SD2_WP__GPIO2_IO20 | MUX_PAD_CTRL(0x16),

#define GP_FASTBOOT_KEY			IMX_GPIO_NR(1, 7)
	IMX8MQ_PAD_GPIO1_IO07__GPIO1_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),

#define GP_I2C1_PCA9546_RESET		IMX_GPIO_NR(1, 8)
	IMX8MQ_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(0x49),

#define GP_I2C4_SN65DSI83_EN		IMX_GPIO_NR(3, 15)
	IMX8MQ_PAD_NAND_RE_B__GPIO3_IO15 | MUX_PAD_CTRL(WEAK_PULLUP),

#define GP_EMMC_RESET			IMX_GPIO_NR(2, 10)
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(0x41),

#define GP_CSI1_MIPI_PWDN		IMX_GPIO_NR(3, 3)
	IMX8MQ_PAD_NAND_CE2_B__GPIO3_IO3 | MUX_PAD_CTRL(0x61),
#define GP_CSI1_MIPI_RESET		IMX_GPIO_NR(3, 17)
	IMX8MQ_PAD_NAND_WE_B__GPIO3_IO17 | MUX_PAD_CTRL(0x61),

#define GP_CSI2_MIPI_PWDN		IMX_GPIO_NR(3, 2)
	IMX8MQ_PAD_NAND_CE1_B__GPIO3_IO2 | MUX_PAD_CTRL(0x61),
#define GP_CSI2_MIPI_RESET		IMX_GPIO_NR(2, 19)
	IMX8MQ_PAD_SD2_RESET_B__GPIO2_IO19 |MUX_PAD_CTRL(0x61),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	set_wdog_reset(wdog);

	gpio_direction_output(GP_ARM_DRAM_VSEL, 0);
	gpio_direction_output(GP_DRAM_1P1_VSEL, 0);
	gpio_direction_output(GP_SOC_GPU_VPU_VSEL, 0);
	gpio_direction_output(GP_EMMC_RESET, 1);
	gpio_direction_output(GP_I2C1_PCA9546_RESET, 0);
	gpio_direction_output(GP_I2C4_SN65DSI83_EN, 0);
	gpio_direction_output(GP_CSI1_MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI1_MIPI_RESET, 0);
	gpio_direction_output(GP_CSI2_MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI2_MIPI_RESET, 0);

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

#ifdef CONFIG_PHY_ATHEROS
#define WEAK_PULLDN_OUTPUT	0x91
#define WEAK_PULLUP_OUTPUT	0xd1

#define PULL_GP(a, bit)		(((a >> bit) & 1) ? WEAK_PULLUP_OUTPUT : WEAK_PULLDN_OUTPUT)
#define PULL_ENET(a, bit)	0x91

#define GP_PHY_RX_CTL	IMX_GPIO_NR(1, 24)
#define GP_PHY_RXC	IMX_GPIO_NR(1, 25)
#define GP_PHY_RD0	IMX_GPIO_NR(1, 26)
#define GP_PHY_RD1	IMX_GPIO_NR(1, 27)
#define GP_PHY_RD2	IMX_GPIO_NR(1, 28)
#define GP_PHY_RD3	IMX_GPIO_NR(1, 29)

#ifndef STRAP_AR8035
#define STRAP_AR8035	((0x28 | (CONFIG_FEC_MXC_PHYADDR & 3)) | ((0x28 | ((CONFIG_FEC_MXC_PHYADDR + 1) & 3)) << 6))
#endif

static const iomux_v3_cfg_t enet_ar8035_gpio_pads[] = {
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 9)
	IOMUX_PAD_CTRL(GPIO1_IO09__GPIO1_IO9, PAD_CTL_DSE6),
	IOMUX_PAD_CTRL(ENET_RD0__GPIO1_IO26, PULL_GP(STRAP_AR8035, 0)),
	IOMUX_PAD_CTRL(ENET_RD1__GPIO1_IO27, PULL_GP(STRAP_AR8035, 1)),
	IOMUX_PAD_CTRL(ENET_RD2__GPIO1_IO28, PULL_GP(STRAP_AR8035, 2)),
	IOMUX_PAD_CTRL(ENET_RD3__GPIO1_IO29, PULL_GP(STRAP_AR8035, 3)),
	IOMUX_PAD_CTRL(ENET_RX_CTL__GPIO1_IO24, PULL_GP(STRAP_AR8035, 4)),
	/* 1.8V(1)/1.5V select(0) */
	IOMUX_PAD_CTRL(ENET_RXC__GPIO1_IO25, PULL_GP(STRAP_AR8035, 5)),
};

static const iomux_v3_cfg_t enet_ar8035_pads[] = {
	IOMUX_PAD_CTRL(ENET_RD0__ENET_RGMII_RD0, PULL_ENET(STRAP_AR8035, 0)),
	IOMUX_PAD_CTRL(ENET_RD1__ENET_RGMII_RD1, PULL_ENET(STRAP_AR8035, 1)),
	IOMUX_PAD_CTRL(ENET_RD2__ENET_RGMII_RD2, PULL_ENET(STRAP_AR8035, 2)),
	IOMUX_PAD_CTRL(ENET_RD3__ENET_RGMII_RD3, PULL_ENET(STRAP_AR8035, 3)),
	IOMUX_PAD_CTRL(ENET_RX_CTL__ENET_RGMII_RX_CTL, PULL_ENET(STRAP_AR8035, 4)),
	IOMUX_PAD_CTRL(ENET_RXC__ENET_RGMII_RXC, PULL_ENET(STRAP_AR8035, 5)),
};

static unsigned char strap_gpios[] = {
	GP_PHY_RD0,
	GP_PHY_RD1,
	GP_PHY_RD2,	/* 0 */
	GP_PHY_RD3,	/* 1 */
	GP_PHY_RX_CTL,	/* 0 */
	GP_PHY_RXC,	/* 1  with LED_1000 pulled high, yields mode 0xc (RGMII, PLLOFF,INT) */
};

static void set_strap_pins(unsigned strap)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(strap_gpios); i++) {
		gpio_direction_output(strap_gpios[i], strap & 1);
		strap >>= 1;
	}
}

static void setup_gpio_ar8035(void)
{
	set_strap_pins(STRAP_AR8035);
	SETUP_IOMUX_PADS(enet_ar8035_gpio_pads);
}

static void setup_enet_ar8035(void)
{
	SETUP_IOMUX_PADS(enet_ar8035_pads);
}

static void setup_iomux_enet(void)
{
	gpio_direction_output(GP_RGMII_PHY_RESET, 0); /* PHY rst */
	setup_gpio_ar8035();

	/* Need delay 10ms according to KSZ9021 spec */
	/* 1 ms minimum reset pulse for ar8035 */
	udelay(1000 * 10);
	gpio_set_value(GP_RGMII_PHY_RESET, 1); /* PHY reset */

	/* strap hold time for AR8035, 5 fails, 6 works, so 12 should be safe */
	udelay(24);

	setup_enet_ar8035();
}

static void phy_ar8031_config(struct phy_device *phydev)
{
	int val;

	/* Select 125MHz clk from local PLL on CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x0007);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= ~0x1c;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, (val|0x0018));

#if 0 //done in ar8031_config
	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, (val|0x0100));
#endif
}

static void phy_ar8035_config(struct phy_device *phydev)
{
	int val;

	/*
	 * Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x3);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x805d);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4003);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val & ~(1 << 8));

#if 0 //done in ar8035_config
	/* rgmii tx clock delay enable */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, (val|0x0100));

	phydev->supported = phydev->drv->features;
#endif
}

#define PHY_ID_AR8031	0x004dd074
#define PHY_ID_AR8035	0x004dd072

int board_phy_config(struct phy_device *phydev)
{
	if (((phydev->drv->uid ^ PHY_ID_AR8031) & 0xffffffef) == 0)
		phy_ar8031_config(phydev);
	else if (((phydev->drv->uid ^ PHY_ID_AR8035) & 0xffffffef) == 0)
		phy_ar8035_config(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC

#define IOMUXC_GPR1		(IOMUXC_GPR_BASE_ADDR + 0x04)

static int setup_fec(void)
{
	gpio_request(GP_RGMII_PHY_RESET, "fec_rst");
	gpio_request(GP_PHY_RD0, "fec_rd0");
	gpio_request(GP_PHY_RD1, "fec_rd1");
	gpio_request(GP_PHY_RD2, "fec_rd2");
	gpio_request(GP_PHY_RD3, "fec_rd3");
	gpio_request(GP_PHY_RX_CTL, "fec_rx_ctl");
	gpio_request(GP_PHY_RXC, "fec_rxc");
	setup_iomux_enet();

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(IOMUXC_GPR1,
			BIT(13) | BIT(17), 0);
	return set_clk_enet(ENET_125MHZ);
}
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)

#define USB_PHY_CTRL0			0xF0040
#define USB_PHY_CTRL0_REF_SSP_EN	BIT(2)

#define USB_PHY_CTRL1			0xF0044
#define USB_PHY_CTRL1_RESET		BIT(0)
#define USB_PHY_CTRL1_COMMONONN		BIT(1)
#define USB_PHY_CTRL1_ATERESET		BIT(3)
#define USB_PHY_CTRL1_VDATSRCENB0	BIT(19)
#define USB_PHY_CTRL1_VDATDETENB0	BIT(20)

#define USB_PHY_CTRL2			0xF0048
#define USB_PHY_CTRL2_TXENABLEN0	BIT(8)

static struct dwc3_device dwc3_device_data = {
	.maximum_speed = USB_SPEED_SUPER,
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
//	.power_down_scale = 2,
};

int usb_gadget_handle_interrupts(void)
{
	dwc3_uboot_handle_interrupt(0);
	return 0;
}

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 RegData;

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_VDATSRCENB0 | USB_PHY_CTRL1_VDATDETENB0 |
			USB_PHY_CTRL1_COMMONONN);
	RegData |= USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET;
	writel(RegData, dwc3->base + USB_PHY_CTRL1);

	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData |= USB_PHY_CTRL0_REF_SSP_EN;
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL2);
	RegData |= USB_PHY_CTRL2_TXENABLEN0;
	writel(RegData, dwc3->base + USB_PHY_CTRL2);

	RegData = readl(dwc3->base + USB_PHY_CTRL1);
	RegData &= ~(USB_PHY_CTRL1_RESET | USB_PHY_CTRL1_ATERESET);
	writel(RegData, dwc3->base + USB_PHY_CTRL1);
}
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;
	imx8m_usb_power(index, true);

	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
		return ret;
	}

	if (index == 1) {
		/* Release HUB reset */
#define GP_USB1_HUB_RESET	IMX_GPIO_NR(1, 14)
		imx_iomux_v3_setup_pad(IMX8MQ_PAD_GPIO1_IO14__GPIO1_IO14 |
				       MUX_PAD_CTRL(WEAK_PULLUP));
		gpio_request(GP_USB1_HUB_RESET, "usb1_rst");
		gpio_direction_output(GP_USB1_HUB_RESET, 1);
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE)
		dwc3_uboot_exit(index);

	imx8m_usb_power(index, false);

	return ret;
}
#endif

#ifdef CONFIG_CMD_FBPANEL

int board_detect_hdmi(struct display_info_t const *di)
{
	return 1;
}

int board_detect_gt911(struct display_info_t const *di)
{
	int ret;
	struct udevice *dev, *chip;

	if (di->bus_gp)
		gpio_direction_output(di->bus_gp, 1);
	gpio_set_value(GP_GT911_RESET, 0);
	mdelay(20);
	gpio_direction_output(GPIRQ_GT911, di->addr == 0x14 ? 1 : 0);
	udelay(100);
	gpio_set_value(GP_GT911_RESET, 1);
	mdelay(6);
	gpio_set_value(GPIRQ_GT911, 0);
	mdelay(50);
	gpio_direction_input(GPIRQ_GT911);
	ret = uclass_get_device(UCLASS_I2C, di->bus_num, &dev);
	if (ret)
		return 0;

	ret = dm_i2c_probe(dev, di->addr, 0x0, &chip);
	if (ret && di->bus_gp)
		gpio_direction_input(di->bus_gp);
	return (ret == 0);
}

static const struct display_info_t displays[] = {
	/* hdmi */
	VD_1920_1080M_60(HDMI, board_detect_hdmi, 0, 0x50),
	VD_1280_720M_60(HDMI, NULL, 0, 0x50),
	VD_MIPI_M101NWWB(MIPI, fbp_detect_i2c, 3 | (GP_I2C4_SN65DSI83_EN << 8), 0x2c),
	VD_LTK080A60A004T(MIPI, board_detect_gt911, 3 | (GP_LTK08_MIPI_EN << 8), 0x5d),	/* Goodix touchscreen */
};
#define display_cnt	ARRAY_SIZE(displays)
#else
#define displays	NULL
#define display_cnt	0
#endif

int board_init(void)
{
	gpio_request(GP_I2C4_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
	gpio_request(GP_WATCHDOG, "watchdog");
	gpio_direction_output(GP_GT911_RESET, 0);
	gpio_direction_output(GP_WATCHDOG, 1);
	#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return 0;
}

#if defined(CONFIG_CMD_FASTBOOT) || defined(CONFIG_CMD_DFU)
extern void imx_get_mac_from_fuse(int dev_id, unsigned char *mac);
static void addserial_env(const char* env_var)
{
	unsigned char mac_address[8];
	char serialbuf[20];

	if (!env_get(env_var)) {
		imx_get_mac_from_fuse(0, mac_address);
		snprintf(serialbuf, sizeof(serialbuf), "%02x%02x%02x%02x%02x%02x",
			 mac_address[0], mac_address[1], mac_address[2],
			 mac_address[3], mac_address[4], mac_address[5]);
		env_set(env_var, serialbuf);
	}
}
#endif

#ifdef CONFIG_CMD_BMODE
const struct boot_mode board_boot_modes[] = {
        /* 4 bit bus width */
	{"emmc0",	MAKE_CFGVAL(0x22, 0x20, 0x00, 0x10)},
        {NULL,          0},
};
#endif

static int fastboot_key_pressed(void)
{
	gpio_request(GP_FASTBOOT_KEY, "fastboot_key");
	gpio_direction_input(GP_FASTBOOT_KEY);
	return !gpio_get_value(GP_FASTBOOT_KEY);
}

void board_late_mmc_env_init(void);
void init_usb_clk(int usbno);

static void set_env_vars(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board", "nitrogen8m");
	env_set("soc", "imx8mq");
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
}

int board_late_init(void)
{
	set_env_vars();
#if defined(CONFIG_USB_FUNCTION_FASTBOOT) || defined(CONFIG_CMD_DFU)
	addserial_env("serial#");
	if (fastboot_key_pressed()) {
		printf("Starting fastboot...\n");
		env_set("preboot", "fastboot 0");
	}
#endif
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	init_usb_clk(0);
	init_usb_clk(1);
	return 0;
}

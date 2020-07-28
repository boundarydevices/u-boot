// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 NXP
 * Copyright 2020 Boundary Devices
 */

#include <common.h>
#include <errno.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/dma.h>
#include <asm/arch/clock.h>
#include <mmc.h>
#include <spl.h>
#include <power/pmic.h>
#include <usb.h>
#include <dwc3-uboot.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	MX8MP_PAD_UART2_RXD__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX8MP_PAD_UART2_TXD__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	MX8MP_PAD_GPIO1_IO02__WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#ifdef CONFIG_NAND_MXS

static void setup_gpmi_nand(void)
{
	init_nand_clk();
}
#endif

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
#ifdef CONFIG_IMX8M_DRAM_INLINE_ECC
	int rc;
	phys_addr_t ecc0_start = 0xb0000000;
	phys_addr_t ecc1_start = 0x130000000;
	phys_addr_t ecc2_start = 0x1b0000000;
	size_t ecc_size = 0x10000000;

	rc = add_res_mem_dt_node(blob, "ecc", ecc0_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc0 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc1_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc1 reserved-memory node.\n");
		return rc;
	}

	rc = add_res_mem_dt_node(blob, "ecc", ecc2_start, ecc_size);
	if (rc < 0) {
		printf("Could not create ecc2 reserved-memory node.\n");
		return rc;
	}
#endif

	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC
#define FEC_RST_PAD IMX_GPIO_NR(4, 2)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	MX8MP_PAD_SAI1_RXD0__GPIO4_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads,
					 ARRAY_SIZE(fec1_rst_pads));

	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 0);
	mdelay(15);
	gpio_direction_output(FEC_RST_PAD, 1);
	mdelay(100);
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	setup_iomux_fec();

	/* Enable RGMII TX clk output */
	setbits_le32(&gpr->gpr[1], BIT(22));

	//return set_clk_enet(ENET_125MHZ);
	return 0;
}
#endif

#ifdef CONFIG_DWC_ETH_QOS

#define EQOS_RST_PAD IMX_GPIO_NR(4, 22)
static iomux_v3_cfg_t const eqos_rst_pads[] = {
	MX8MP_PAD_SAI2_RXC__GPIO4_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_eqos(void)
{
	imx_iomux_v3_setup_multiple_pads(eqos_rst_pads,
					 ARRAY_SIZE(eqos_rst_pads));

	gpio_request(EQOS_RST_PAD, "eqos_rst");
	gpio_direction_output(EQOS_RST_PAD, 0);
	mdelay(15);
	gpio_direction_output(EQOS_RST_PAD, 1);
	mdelay(100);
}

static int setup_eqos(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	setup_iomux_eqos();

	/* set INTF as RGMII, enable RGMII TXC clock */
	clrsetbits_le32(&gpr->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET_QOS_INTF_SEL_MASK, BIT(16));
	setbits_le32(&gpr->gpr[1], BIT(19) | BIT(21));

	return set_clk_eqos(ENET_125MHZ);
}
#endif

#ifdef CONFIG_USB_DWC3

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

#define USB_PHY_CTRL6			0xF0058

#define HSIO_GPR_BASE                               (0x32F10000U)
#define HSIO_GPR_REG_0                              (HSIO_GPR_BASE)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT    (1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN          (0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)


static struct dwc3_device dwc3_device_data = {
#ifdef CONFIG_SPL_BUILD
	.maximum_speed = USB_SPEED_HIGH,
#else
	.maximum_speed = USB_SPEED_SUPER,
#endif
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

int usb_gadget_handle_interrupts(void)
{
	dwc3_uboot_handle_interrupt(0);
	return 0;
}

static void dwc3_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 RegData;

	/* enable usb clock via hsio gpr */
	RegData = readl(HSIO_GPR_REG_0);
	RegData |= HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN;
	writel(RegData, HSIO_GPR_REG_0);

	/* USB3.0 PHY signal fsel for 100M ref */
	RegData = readl(dwc3->base + USB_PHY_CTRL0);
	RegData = (RegData & 0xfffff81f) | (0x2a<<5);
	writel(RegData, dwc3->base + USB_PHY_CTRL0);

	RegData = readl(dwc3->base + USB_PHY_CTRL6);
	RegData &=~0x1;
	writel(RegData, dwc3->base + USB_PHY_CTRL6);

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
#define USB2_PWR_EN IMX_GPIO_NR(1, 14)
int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;
	imx8m_usb_power(index, true);

	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
	} else if (index == 0 && init == USB_INIT_HOST) {
		return ret;
	} else if (index == 1 && init == USB_INIT_HOST) {
		/* Enable GPIO1_IO14 for 5V VBUS */
		gpio_request(USB2_PWR_EN, "usb2_pwr");
		gpio_direction_output(USB2_PWR_EN, 1);
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE) {
		dwc3_uboot_exit(index);
	} else if (index == 0 && init == USB_INIT_HOST) {
	} else if (index == 1 && init == USB_INIT_HOST) {
		/* Disable GPIO1_IO14 for 5V VBUS */
		gpio_direction_output(USB2_PWR_EN, 0);
	}

	imx8m_usb_power(index, false);

	return ret;
}

#ifdef CONFIG_USB_TCPC
/* Not used so far */
int board_typec_get_mode(int index)
{
	if (index == 0) {
		return USB_INIT_DEVICE;
	} else {
		return USB_INIT_HOST;
	}
}
#endif
#endif

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				13
#define MIPI				15

int board_init(void)
{
#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_DWC_ETH_QOS
	/* clock, pin, gpr */
	setup_eqos();
#endif

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand();
#endif

#if defined(CONFIG_USB_DWC3) || defined(CONFIG_USB_XHCI_IMX8M)
	init_usb_clk();
#endif

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	return 0;
}

static int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

/* This should be defined for each board */
__weak int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_dev();

	if (!check_mmc_autodetect())
		return;

	env_set_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	env_set("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

void board_env_set_default(void)
{
	u32 cpurev = get_cpu_rev();

	env_set("imx_cpu", get_imx_type(cpurev >> 12));
	env_set("uboot_defconfig", CONFIG_DEFCONFIG);
	env_set("board", "nitrogen8mp");
}

static int board_eth_initialize(void);

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "nitrogen8mp");
#endif
	board_eth_initialize();
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

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

#ifdef CONFIG_SPL_MMC_SUPPORT

#define UBOOT_RAW_SECTOR_OFFSET 0x40
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc)
{
	u32 boot_dev = spl_boot_device();
	switch (boot_dev) {
		case BOOT_DEVICE_MMC2:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR - UBOOT_RAW_SECTOR_OFFSET;
		default:
			return CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
	}
}
#endif

#ifdef CONFIG_PHY_ATHEROS
static void init_fec_clocks(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			BIT(13) | BIT(17), 0);
	set_clk_enet(ENET_125MHZ);
	udelay(100);	/* Wait 100 us before using mii interface */
}

#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(3, 16)
#define GP_PHY_RD0		IMX_GPIO_NR(1, 26)
#define GP_PHY_RD1		IMX_GPIO_NR(1, 27)
#define GP_PHY_RD2		IMX_GPIO_NR(1, 28)
#define GP_PHY_RD3		IMX_GPIO_NR(1, 29)
#define GP_PHY_RX_CTL		IMX_GPIO_NR(1, 24)
#define GP_PHY_RXC		IMX_GPIO_NR(1, 25)

#define GP_RGMII2_PHY_RESET	IMX_GPIO_NR(5, 8)
#define GP_PHY2_RD0		IMX_GPIO_NR(4, 6)
#define GP_PHY2_RD1		IMX_GPIO_NR(4, 7)
#define GP_PHY2_RD2		IMX_GPIO_NR(4, 8)
#define GP_PHY2_RD3		IMX_GPIO_NR(4, 9)
#define GP_PHY2_RX_CTL		IMX_GPIO_NR(4, 10)
#define GP_PHY2_RXC		IMX_GPIO_NR(4, 11)

#define WEAK_PULLUP_OUTPUT	0x140
#define WEAK_PULLDN_OUTPUT	0x100
#define ATHEROS_RESET_PINCTRL	WEAK_PULLDN_OUTPUT

#define STRAP_AR8035		(0x28 | (CONFIG_FEC_MXC_PHYADDR & 3))
#define PULL_GP(a, bit)		(((a >> bit) & 1) ? WEAK_PULLUP_OUTPUT : WEAK_PULLDN_OUTPUT)

#define IOMUX_PAD_CTRL(a, b) NEW_PAD_CTRL(MX8MP_PAD_##a, b)
#define PAD_CTRL_ENET_RX	0x91

static const iomux_v3_cfg_t enet_ar8035_gpio_pads[] = {
       IOMUX_PAD_CTRL(NAND_READY_B__GPIO3_IO16, ATHEROS_RESET_PINCTRL),
       IOMUX_PAD_CTRL(ECSPI1_MISO__GPIO5_IO08, ATHEROS_RESET_PINCTRL),
       IOMUX_PAD_CTRL(ENET_RD0__GPIO1_IO26, PULL_GP(STRAP_AR8035, 0)),
       IOMUX_PAD_CTRL(ENET_RD1__GPIO1_IO27, PULL_GP(STRAP_AR8035, 1)),
       IOMUX_PAD_CTRL(ENET_RD2__GPIO1_IO28, PULL_GP(STRAP_AR8035, 2)),
       IOMUX_PAD_CTRL(ENET_RD3__GPIO1_IO29, PULL_GP(STRAP_AR8035, 3)),
       IOMUX_PAD_CTRL(ENET_RX_CTL__GPIO1_IO24, PULL_GP(STRAP_AR8035, 4)),
       IOMUX_PAD_CTRL(ENET_RXC__GPIO1_IO25, PULL_GP(STRAP_AR8035, 5)),

       IOMUX_PAD_CTRL(SAI1_RXD4__GPIO4_IO06, PULL_GP(STRAP_AR8035, 0)),
       IOMUX_PAD_CTRL(SAI1_RXD5__GPIO4_IO07, PULL_GP(STRAP_AR8035, 1)),
       IOMUX_PAD_CTRL(SAI1_RXD6__GPIO4_IO08, PULL_GP(STRAP_AR8035, 2)),
       IOMUX_PAD_CTRL(SAI1_RXD7__GPIO4_IO09, PULL_GP(STRAP_AR8035, 3)),
       IOMUX_PAD_CTRL(SAI1_TXFS__GPIO4_IO10, PULL_GP(STRAP_AR8035, 4)),
       IOMUX_PAD_CTRL(SAI1_TXC__GPIO4_IO11, PULL_GP(STRAP_AR8035, 5)),
};

static const iomux_v3_cfg_t enet_ar8035_pads[] = {
       IOMUX_PAD_CTRL(ENET_RD0__ENET_QOS_RGMII_RD0, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(ENET_RD1__ENET_QOS_RGMII_RD1, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(ENET_RD2__ENET_QOS_RGMII_RD2, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(ENET_RD3__ENET_QOS_RGMII_RD3, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(ENET_RX_CTL__ENET_QOS_RGMII_RX_CTL, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(ENET_RXC__CCM_ENET_QOS_CLOCK_GENERATE_RX_CLK, PAD_CTRL_ENET_RX),

       IOMUX_PAD_CTRL(SAI1_RXD4__ENET1_RGMII_RD0, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(SAI1_RXD5__ENET1_RGMII_RD1, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(SAI1_RXD6__ENET1_RGMII_RD2, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(SAI1_RXD7__ENET1_RGMII_RD3, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(SAI1_TXFS__ENET1_RGMII_RX_CTL, PAD_CTRL_ENET_RX),
       IOMUX_PAD_CTRL(SAI1_TXC__ENET1_RGMII_RXC, PAD_CTRL_ENET_RX),
};

static unsigned char strap_gpios[] = {
	GP_PHY_RD0,
	GP_PHY_RD1,
	GP_PHY_RD2,
	GP_PHY_RD3,
	GP_PHY_RX_CTL,
	GP_PHY_RXC,

	GP_PHY2_RD0,
	GP_PHY2_RD1,
	GP_PHY2_RD2,
	GP_PHY2_RD3,
	GP_PHY2_RX_CTL,
	GP_PHY2_RXC,
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
	set_strap_pins(STRAP_AR8035 | (STRAP_AR8035 << 6));
	SETUP_IOMUX_PADS(enet_ar8035_gpio_pads);
}

static void setup_enet_ar8035(void)
{
	SETUP_IOMUX_PADS(enet_ar8035_pads);
}
#define setup_gpio_eth(kz) setup_gpio_ar8035()
#define setup_enet_eth(kz) setup_enet_ar8035()

static void release_phy_reset(int gp)
{
	gpio_set_value(gp, 1);
#ifdef CONFIG_FEC_RESET_PULLUP
	udelay(20);
	gpio_direction_input(gp);
	/* Let external pull have time to pull to guaranteed high */
	udelay(200);
#endif
}

static void setup_iomux_enet(int kz)
{
	gpio_direction_output(GP_RGMII2_PHY_RESET, 0); /* PHY rst */
	gpio_direction_output(GP_RGMII_PHY_RESET, 0); /* PHY rst */
	setup_gpio_eth(kz);

	init_fec_clocks();

	/* Need delay 10ms according to KSZ9021 spec */
	/* 1 ms minimum reset pulse for ar8035 */
	udelay(1000 * 10);

	release_phy_reset(GP_RGMII_PHY_RESET);
	release_phy_reset(GP_RGMII2_PHY_RESET);


	/* strap hold time for AR8031, 18 fails, 19 works, so 40 should be safe */
	/* strap hold time for AR8035, 5 fails, 6 works, so 12 should be safe */
	udelay(40);

	setup_enet_eth(kz);
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

#define PHY_ID_AR8035	0x004dd072

int board_phy_config(struct phy_device *phydev)
{
	phy_ar8035_config(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif



void board_eth_addresses(void)
{
#if defined(CONFIG_USB_ETHER)
#if defined(CONFIG_FEC_MXC) && defined(CONFIG_FEC_ENET1) \
	&& defined(CONFIG_FEC_ENET2)
#define USB_ETH "eth2addr"
#elif defined(CONFIG_FEC_MXC)
#define USB_ETH "eth1addr"
#else
#define USB_ETH "ethaddr"
#endif
	/* For otg ethernet*/
#ifndef CONFIG_FEC_MXC
	/* ethaddr should be set from fuses */
	if (!env_get(USB_ETH)) {
		unsigned char mac[8];

		imx_get_mac_from_fuse(0, mac);
		if (is_valid_ethaddr(mac))
			eth_env_set_enetaddr(USB_ETH, mac);
		else
			env_set(USB_ETH, env_get("usbnet_devaddr"));
	}
#else
	if (!env_get(USB_ETH))
		env_set(USB_ETH, env_get("usbnet_devaddr"));
#endif
#endif
}

static int board_eth_initialize(void)
{
	gpio_request(GP_RGMII_PHY_RESET, "fec_rst");
	gpio_request(GP_PHY_RD0, "fec_rd0");
	gpio_request(GP_PHY_RD1, "fec_rd1");
	gpio_request(GP_PHY_RD2, "fec_rd2");
	gpio_request(GP_PHY_RD3, "fec_rd3");
	gpio_request(GP_PHY_RX_CTL, "fec_rx_ctl");
	gpio_request(GP_PHY_RXC, "fec_rxc");

	gpio_request(GP_RGMII2_PHY_RESET, "fec2_rst");
	gpio_request(GP_PHY2_RD0, "fec2_rd0");
	gpio_request(GP_PHY2_RD1, "fec2_rd1");
	gpio_request(GP_PHY2_RD2, "fec2_rd2");
	gpio_request(GP_PHY2_RD3, "fec2_rd3");
	gpio_request(GP_PHY2_RX_CTL, "fec2_rx_ctl");
	gpio_request(GP_PHY2_RXC, "fec2_rxc");

	setup_iomux_enet(0);

	board_eth_addresses();
#if defined(CONFIG_USB_ETHER)
	{
		int ret = usb_ether_init();

		printf("usb_ether_init (%d)\n", ret);
		if (ret) {
			printf("usb_ether_init failed(%d)\n", ret);
		}
	}
#endif
	return 0;
}

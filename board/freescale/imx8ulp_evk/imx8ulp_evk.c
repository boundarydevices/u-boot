// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#include <common.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/imx8ulp-pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/pcc.h>
#include <asm/arch/sys_proto.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <malloc.h>
#include <micrel.h>
#include <i2c.h>
#include <power-domain.h>
#include <dt-bindings/power/imx8ulp-power.h>

DECLARE_GLOBAL_DATA_PTR;
#if defined(CONFIG_NXP_FSPI) || defined(CONFIG_FSL_FSPI_NAND)
#define FSPI_PAD_CTRL	(PAD_CTL_PUS_UP | PAD_CTL_DSE)
static iomux_cfg_t const flexspi0_pads[] = {
	IMX8ULP_PAD_PTC5__FLEXSPI0_A_SS0_b | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC6__FLEXSPI0_A_SCLK | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC10__FLEXSPI0_A_DATA0 | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC9__FLEXSPI0_A_DATA1 | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC8__FLEXSPI0_A_DATA2 | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC7__FLEXSPI0_A_DATA3 | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC4__FLEXSPI0_A_DATA4 | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC3__FLEXSPI0_A_DATA5 | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC2__FLEXSPI0_A_DATA6 | MUX_PAD_CTRL(FSPI_PAD_CTRL),
	IMX8ULP_PAD_PTC1__FLEXSPI0_A_DATA7 | MUX_PAD_CTRL(FSPI_PAD_CTRL),
};

static void setup_flexspi(void)
{
	init_clk_fspi(0);
}

static void setup_rtd_flexspi0(void)
{
	imx8ulp_iomux_setup_multiple_pads(flexspi0_pads, ARRAY_SIZE(flexspi0_pads));

	/* Set PCC of flexspi0, 192Mhz % 4 = 48Mhz */
	writel(0xD6000003, 0x280300e4);
}

#endif

#if IS_ENABLED(CONFIG_FEC_MXC)
#define ENET_CLK_PAD_CTRL	(PAD_CTL_PUS_UP | PAD_CTL_DSE | PAD_CTL_IBE_ENABLE)
static iomux_cfg_t const enet_clk_pads[] = {
	IMX8ULP_PAD_PTE19__ENET0_REFCLK | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	IMX8ULP_PAD_PTF10__ENET0_1588_CLKIN | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
};

static int setup_fec(void)
{
	/*
	 * Since ref clock and timestamp clock are from external,
	 * set the iomux prior the clock enablement
	 */
	imx8ulp_iomux_setup_multiple_pads(enet_clk_pads, ARRAY_SIZE(enet_clk_pads));

	/* Select enet time stamp clock: 001 - External Timestamp Clock */
	cgc1_enet_stamp_sel(1);

	/* enable FEC PCC */
	pcc_clock_enable(4, ENET_PCC4_SLOT, true);
	pcc_reset_peripheral(4, ENET_PCC4_SLOT, false);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#define I2C_PAD_CTRL	(PAD_CTL_ODE)
static const iomux_cfg_t lpi2c0_pads[] = {
	IMX8ULP_PAD_PTA8__LPI2C0_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
	IMX8ULP_PAD_PTA9__LPI2C0_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
};

#define TPM_PAD_CTRL	(PAD_CTL_DSE)
static const iomux_cfg_t tpm0_pads[] = {
	IMX8ULP_PAD_PTA3__TPM0_CH2 | MUX_PAD_CTRL(TPM_PAD_CTRL),
};

void mipi_dsi_mux_panel(void)
{
	int ret;
	struct gpio_desc desc;

	/* It is temp solution to directly access i2c, need change to rpmsg later */

	/* enable lpi2c0 clock and iomux */
	imx8ulp_iomux_setup_multiple_pads(lpi2c0_pads, ARRAY_SIZE(lpi2c0_pads));
	writel(0xD2000000, 0x28091060);

	ret = dm_gpio_lookup_name("gpio@20_9", &desc);
	if (ret) {
		printf("%s lookup gpio@20_9 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "dsi_mux");
	if (ret) {
		printf("%s request dsi_mux failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
}

void mipi_dsi_panel_backlight(void)
{
	/* It is temp solution to directly access pwm, need change to rpmsg later */
	imx8ulp_iomux_setup_multiple_pads(tpm0_pads, ARRAY_SIZE(tpm0_pads));
	writel(0xD4000001, 0x28091054);

	/* Use center-aligned PWM mode, CPWMS=1, MSnB:MSnA = 10, ELSnB:ELSnA = 00 */
	writel(1000, 0x28095018);
	writel(1000, 0x28095034); /* MOD = CV, full duty */
	writel(0x28, 0x28095010);
	writel(0x20, 0x28095030);
}

void reset_lsm6dsx(uint8_t i2c_bus, uint8_t addr)
{
	struct udevice *bus;
	struct udevice *i2c_dev = NULL;
	int ret;
	struct i2c_msg msg;
	u8 i2c_buf[2] = { 0x12, 0x1 };

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return;
	}

	ret = dm_i2c_probe(bus, addr, 0, &i2c_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x\n",
			__func__, addr);
		return;
	}

	msg.addr = addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = i2c_buf;

	ret = dm_i2c_xfer(i2c_dev, &msg, 1);
	if (!ret)
		printf("%s: Reset device 0x%x successfully.\n", __func__, addr);
}

int board_init(void)
{
#if defined(CONFIG_NXP_FSPI) || defined(CONFIG_FSL_FSPI_NAND)
	setup_flexspi();

	if (get_boot_mode() == SINGLE_BOOT) {
		setup_rtd_flexspi0();
	}
#endif
#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif

#if defined(CONFIG_FEC_MXC)
	setup_fec();
#endif

	/* When sync with M33 is failed, use local driver to set for video */
	if (!is_m33_handshake_necessary() && IS_ENABLED(CONFIG_DM_VIDEO)) {
		mipi_dsi_mux_panel();
		mipi_dsi_panel_backlight();
	}

	return 0;
}

int board_early_init_f(void)
{
	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

#ifdef CONFIG_SYS_I2C_IMX_I3C
	reset_lsm6dsx(8, 0x9);
#endif

	return 0;
}

#define PAD_CTRL_ENET_MDC	(0)
/* PAD_CTL_ODE is screwy on MDIO, it behaves like a strong hysteresis */
#define PAD_CTRL_ENET_MDIO	(0)

#define PAD_CTRL_ENET_RX	(PAD_CTL_DSE)
#define PAD_CTRL_ENET_RX_DN	(PAD_CTRL_ENET_RX | PAD_CTL_PE)
#define PAD_CTRL_ENET_RX_UP	(PAD_CTRL_ENET_RX | PAD_CTL_PE | PAD_CTL_PUE)

#define PAD_CTRL_ENET_TX	0x1f

#define WEAK_PULLUP		(PAD_CTL_PUS_UP)
#define WEAK_PULLUP_OUTPUT	(PAD_CTL_PUS_UP)
#define WEAK_PULLDN_OUTPUT	(PAD_CTL_PUS_DOWN)

#define QSPI_PAD_CTRL	(PAD_CTL_DSE)

#define UART_PAD_CTRL	(PAD_CTL_PUS_UP)

#define WDOG_PAD_CTRL	(PAD_CTL_ODE | PAD_CTL_PUS_UP)

#if defined(CONFIG_FEC_MXC) && !defined(CONFIG_FEC_MXC_PHYADDR)
#error CONFIG_FEC_MXC_PHYADDR missing
#endif

#define ATHEROS_MASK(a) 0
#define KSZ9021_MASK(a) 0
#define KSZ8XXX_MASK(a) 2

#define ALL_ATH_PHY_MASK(a) (ATHEROS_MASK(a))
#define ALL_KSZ_PHY_MASK(a) (KSZ9021_MASK(a) | KSZ8XXX_MASK(a))
#define COMBINED_ATH_MASK(a, b) ALL_ATH_PHY_MASK(a)
#define COMBINED_KSZ_MASK(a, b) ALL_KSZ_PHY_MASK(a)

#ifndef CONFIG_FEC_ENET1
#define CONFIG_FEC_MXC_PHYADDR2		CONFIG_FEC_MXC_PHYADDR
#else
#define CONFIG_FEC_MXC_PHYADDR2		(CONFIG_FEC_MXC_PHYADDR + 1)
#endif

#ifndef CONFIG_FEC_MXC_KSZ_PHYADDR
#define CONFIG_FEC_MXC_KSZ_PHYADDR	CONFIG_FEC_MXC_PHYADDR
#endif
#define CONFIG_FEC_MXC_KSZ_PHYADDR2	(CONFIG_FEC_MXC_KSZ_PHYADDR + 1)

#ifndef ETH_PHY_MASK
#define ETH_PHY_MASK_ATH	COMBINED_ATH_MASK(CONFIG_FEC_MXC_PHYADDR, CONFIG_FEC_MXC_PHYADDR2)
#define ETH_PHY_MASK_KSZ	COMBINED_KSZ_MASK(CONFIG_FEC_MXC_KSZ_PHYADDR, CONFIG_FEC_MXC_KSZ_PHYADDR2)
#else
#define ETH_PHY_MASK_ATH	ETH_PHY_MASK
#define ETH_PHY_MASK_KSZ	ETH_PHY_MASK

#if ((ETH_PHY_MASK >> CONFIG_FEC_MXC_PHYADDR) & 1) == 0
#error CONFIG_FEC_MXC_PHYADDR is not part of ETH_PHY_MASK
#endif
#endif

#define PULL_GP(a, bit)		(((a >> (bit)) & 1) ? WEAK_PULLUP_OUTPUT : WEAK_PULLDN_OUTPUT)
#if 0	//defined(CONFIG_MX7D)
#define PULL_ENET(a, bit)	(((a >> (bit)) & 1) ? PAD_CTRL_ENET_RX_UP : PAD_CTRL_ENET_RX_DN)
#else
#define PULL_ENET(a, bit)	PAD_CTRL_ENET_RX
#endif

#define IMX_GPIO_NR(port, index)	((((port)-1)*32)+((index)&31))
#define GP_PHY_RD0	IMX_GPIO_NR(3, 1)
#define GP_PHY_RD1	IMX_GPIO_NR(3, 0)
#define GP_PHY_ER	IMX_GPIO_NR(3, 5)
#define GP_PHY_CRS	IMX_GPIO_NR(3, 6)

#ifndef STRAP_AR8035
#define STRAP_AR8035	(0x28 | (CONFIG_FEC_MXC_PHYADDR & 3))
#endif

#ifndef GP_RGMII_PHY_RESET
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(3, 15)
#endif

#define IOMUX_PAD_CTRL(name, ctrl)	NEW_PAD_CTRL(IMX8ULP_PAD_##name, ctrl)
#define SETUP_IOMUX_PAD(def)                                    \
        imx8ulp_iomux_setup_pad(IMX8ULP_PAD_##def);
#define SETUP_IOMUX_PADS(x)                                     \
        imx8ulp_iomux_setup_multiple_pads(x, ARRAY_SIZE(x))

#define STRAP_KSZ_CLK125_EN	0x10
#define STRAP_KSZ_BASE		0x0f
#define STRAP_KSZ8863	(STRAP_KSZ_BASE | ((CONFIG_FEC_MXC_PHYADDR & 4) ? 0x20 : 0))

#ifdef CONFIG_PHY_MICREL
static const iomux_cfg_t enet_ksz8863_gpio_pads[] = {
	IOMUX_PAD_CTRL(PTF1__PTF1, PULL_GP(STRAP_KSZ8863, 0)),
	IOMUX_PAD_CTRL(PTF0__PTF0, PULL_GP(STRAP_KSZ8863, 1)),
	IOMUX_PAD_CTRL(PTF5__PTF5, PULL_GP(STRAP_KSZ8863, 2)),
	IOMUX_PAD_CTRL(PTF6__PTF6, PULL_GP(STRAP_KSZ8863, 3)),
};

static const iomux_cfg_t enet_ksz8863_pads[] = {
	IOMUX_PAD_CTRL(PTF1__ENET0_RXD0, PULL_ENET(STRAP_KSZ8863, 0)),
	IOMUX_PAD_CTRL(PTF0__ENET0_RXD1, PULL_ENET(STRAP_KSZ8863, 1)),
	IOMUX_PAD_CTRL(PTF5__ENET0_RXER, PULL_ENET(STRAP_KSZ8863, 2)),
	IOMUX_PAD_CTRL(PTF6__ENET0_CRS_DV, PULL_ENET(STRAP_KSZ8863, 3)),
};
#endif


#if !defined(CONFIG_FEC_ENET1) && !defined(CONFIG_FEC_ENET2) && defined(CONFIG_FEC_MXC)
#define CONFIG_FEC_ENET1
#endif
#if !defined(CONFIG_FEC_ENET2) && defined(CONFIG_DWC_ETH_QOS)
#define CONFIG_FEC_ENET2
#endif

#if defined(CONFIG_PHY_ATHEROS) || defined(CONFIG_PHY_MICREL)
#ifdef CONFIG_FEC_ENET1
static unsigned char strap_gpios[] = {
	GP_PHY_RD0,
	GP_PHY_RD1,
#ifdef GP_PHY_RD2
	GP_PHY_RD2,
#endif
#ifdef GP_PHY_RD3
	GP_PHY_RD3,
#endif
#ifdef GP_PHY_RX_CTL
	GP_PHY_RX_CTL,
#endif
#ifdef GP_PHY_RXC
	GP_PHY_RXC,
#endif
};
#endif

#if defined(CONFIG_FEC_ENET1) || defined(CONFIG_FEC_ENET2)
static void set_strap_pins(unsigned char* pgpio, int cnt, unsigned strap)
{
	int i = 0;

	for (i = 0; i < cnt; i++) {
		gpio_direction_output(pgpio[i], strap & 1);
		strap >>= 1;
	}
}
#endif
#endif

#ifdef CONFIG_PHY_MICREL_KSZ8XXX
static void setup_gpio_ksz8863(void)
{
	set_strap_pins(strap_gpios, ARRAY_SIZE(strap_gpios), STRAP_KSZ8863);
	SETUP_IOMUX_PADS(enet_ksz8863_gpio_pads);
}

static void setup_enet_ksz8863(void)
{
	SETUP_IOMUX_PADS(enet_ksz8863_pads);
}
#define setup_gpio_micrel setup_gpio_ksz8863
#define setup_enet_micrel setup_enet_ksz8863
#endif

#ifdef CONFIG_FEC_ENET1
#define setup_gpio_eth(kz) setup_gpio_micrel();
#define setup_enet_eth(kz) setup_enet_micrel();
#endif

#if defined(CONFIG_FEC_MXC) || defined(CONFIG_DWC_ETH_QOS)
#if !defined(CONFIG_MX5)
static void init_fec_clocks(void)
{
	udelay(100);	/* Wait 100 us before using mii interface */
}
#endif
#endif

#define FEC_ENET1_ID	-1	/* just plain FEC */
#define FEC_ENET2_ID	1

#define PHY_MODE PHY_INTERFACE_MODE_RMII

#if defined(CONFIG_PHY_ATHEROS) || defined(CONFIG_PHY_MICREL)
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

#ifdef CONFIG_FEC_ENET1
static char iomux_selection = -1;
#endif

static void setup_iomux_enet(int kz, int net_mask)
{
#ifdef CONFIG_FEC_ENET1
	if (net_mask & 1) {
		if (iomux_selection == kz)
			net_mask &= ~1;
		else
			iomux_selection = kz;
	}
#endif
	if (!net_mask)
		return;

#ifdef GP_RGMII_PHY_RESET
	if (net_mask & 1)
		gpio_direction_output(GP_RGMII_PHY_RESET, 0); /* PHY rst */
#endif

#ifdef CONFIG_FEC_ENET1
	if (net_mask & 1) {
		setup_gpio_eth(kz);
	}
#endif

#if defined(CONFIG_FEC_MXC) || defined(CONFIG_DWC_ETH_QOS)
	init_fec_clocks();
#endif
	/* Need delay 10ms according to KSZ9021 spec */
	/* 1 ms minimum reset pulse for ar8035 */
	udelay(1000 * 10);
#ifdef GP_RGMII_PHY_RESET
	if (net_mask & 1)
		release_phy_reset(GP_RGMII_PHY_RESET);
#endif

	/* strap hold time for AR8031, 18 fails, 19 works, so 40 should be safe */
	/* strap hold time for AR8035, 5 fails, 6 works, so 12 should be safe */
	udelay(40);

#ifdef CONFIG_FEC_ENET1
	if (net_mask & 1) {
		setup_enet_eth(kz);
	}
#endif
	/*
	 * KSZ9031 needs 100 us before starting programming,
	 * we've already waited 40us, let's wait 100 more
	 */
	udelay(100);
}
#endif

#define PHY_ID_KSZ9021	0x221610

static inline bool is_micrel_part(struct phy_device *phydev)
{
	if ((((phydev->drv->uid ^ PHY_ID_KSZ9021) & 0xfffffff0) == 0) ||
	    (((phydev->drv->uid ^ PHY_ID_KSZ9031) & 0xfffffff0) == 0))
		return true;
	return false;
}


#if !defined(CONFIG_DM_ETH) && defined(CONFIG_FEC_MXC)
#define MDIO_BASE	IMX_FEC_BASE
#define BASE		IMX_FEC_BASE

static void init_fec(struct bd_info *bis, unsigned phy_mask_ath, unsigned phy_mask_ksz)
{
	struct mii_dev *bus = NULL;
	uint32_t mdio_base = MDIO_BASE;
#if defined(CONFIG_FEC_ENET1)
	uint32_t base = BASE;
	struct phy_device *phydev1 = NULL;
#endif
#if defined(CONFIG_FEC_ENET1) || defined(CONFIG_FEC_ENET2)
	int ret;
#endif

	bus = fec_get_miibus(mdio_base, -1);
	if (!bus)
		return;
#if defined(CONFIG_FEC_ENET1)
	if (phy_mask_ath & 0xffff)
		phydev1 = phy_find_by_mask(bus, phy_mask_ath & 0xffff, PHY_MODE);
	if (!phydev1 && (phy_mask_ksz & 0xffff)) {
		phydev1 = phy_find_by_mask(bus, phy_mask_ksz & 0xffff, PHY_MODE);
	}
	if (!phydev1) {
		printf("%s: phy not found\n", __func__);
	} else {
		printf("%s at %d\n", phydev1->drv->name, phydev1->addr);
		ret  = fec_probe(bis, FEC_ENET1_ID, base, bus, phydev1);
		if (ret) {
			printf("FEC MXC: %s:failed\n", __func__);
			free(phydev1);
		}
		phy_mask_ksz &= ~(1 << phydev1->addr);
		phy_mask_ath &= ~(1 << phydev1->addr);
	}
#endif

	return;
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
#if defined(CONFIG_FEC) && defined(CONFIG_FEC_ENET1)
	fec_env_set_ethaddr(0);
#endif
#if defined(CONFIG_FEC) && defined(CONFIG_FEC_ENET2)
	fec_env_set_ethaddr(1);
#endif
}

int board_eth_init(struct bd_info *bis)
{
#if defined(CONFIG_PHY_ATHEROS) || defined(CONFIG_PHY_MICREL)
#ifdef CONFIG_FEC_ENET1
	gpio_request(GP_RGMII_PHY_RESET, "fec_rst");
#if defined(GP_RGMII2_PHY_RESET) && !defined(CONFIG_FEC_ENET2)
	gpio_request(GP_RGMII2_PHY_RESET, "fec2_rst");
#endif
	gpio_request(GP_PHY_RD0, "fec_rd0");
	gpio_request(GP_PHY_RD1, "fec_rd1");
#ifdef GP_PHY_RD2
	gpio_request(GP_PHY_RD2, "fec_rd2");
#endif
#ifdef GP_PHY_RD3
	gpio_request(GP_PHY_RD3, "fec_rd3");
#endif
#ifdef GP_PHY_RX_CTL
	gpio_request(GP_PHY_RX_CTL, "fec_rx_ctl");
#endif
#ifdef GP_PHY_RXC
	gpio_request(GP_PHY_RXC, "fec_rxc");
#endif
#ifdef CONFIG_PHY_MICREL_KSZ8XXX
#ifdef GP_PHY_RX_EN
	gpio_request(GP_PHY_RX_EN, "fec_rx_en");
#endif
	gpio_request(GP_PHY_ER, "fec_er");
	gpio_request(GP_PHY_CRS, "fec_crs");
#ifdef GP_PHY_COL
	gpio_request(GP_PHY_COL, "fec_col");
#endif
#endif
#endif

	setup_iomux_enet(0, 3);
#endif
#if !defined(CONFIG_DM_ETH) && defined(CONFIG_FEC_MXC)
	init_fec(bis, ETH_PHY_MASK_ATH, ETH_PHY_MASK_KSZ);
#endif

	board_eth_addresses();
#if defined(CONFIG_USB_ETHER)
#if defined(CONFIG_DM_ETH)
	{
		int ret = usb_ether_init();

		if (ret) {
			printf("usb_ether_init failed(%d)\n", ret);
		}
	}
#else
	usb_eth_initialize(bis);
#endif
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


void board_quiesce_devices(void)
{
	/* Disable the power domains may used in u-boot before entering kernel */
#if CONFIG_IS_ENABLED(POWER_DOMAIN)
	struct udevice *scmi_devpd;
	int ret, i;
	struct power_domain pd;
	ulong ids[] = {
		IMX8ULP_PD_FLEXSPI2, IMX8ULP_PD_USB0, IMX8ULP_PD_USDHC0,
		IMX8ULP_PD_USDHC1, IMX8ULP_PD_USDHC2_USB1, IMX8ULP_PD_DCNANO,
		IMX8ULP_PD_MIPI_DSI};

	ret = uclass_get_device(UCLASS_POWER_DOMAIN, 0, &scmi_devpd);
	if (ret) {
		printf("Cannot get scmi devpd: err=%d\n", ret);
		return;
	}

	pd.dev = scmi_devpd;

	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		pd.id = ids[i];
		ret = power_domain_off(&pd);
		if (ret)
			printf("power_domain_off %lu failed: err=%d\n", ids[i], ret);
	}
#endif
}

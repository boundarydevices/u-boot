/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#if defined(CONFIG_MX51)
#include <asm/arch/iomux-mx51.h>
#elif defined(CONFIG_MX7D) || defined(CONFIG_IMX8M)
#else
#include <asm/arch/iomux.h>
#endif
#include <asm/arch/sys_proto.h>
#if defined(CONFIG_MX7D)
#include <asm/arch/mx7-pins.h>
#elif defined(CONFIG_IMX8MM)
#include <asm/arch/imx8mm_pins.h>
#elif defined(CONFIG_IMX8MN)
#include <asm/arch/imx8mn_pins.h>
#elif defined(CONFIG_IMX8MP)
#include <asm/arch/imx8mp_pins.h>
#elif defined(CONFIG_IMX8MQ)
#include <asm/arch/imx8mq_pins.h>
#elif !defined(CONFIG_MX51)
#include <asm/arch/mx6-pins.h>
#endif
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/spi.h>
#include <env.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <malloc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <spi.h>
#include "../common/padctrl.h"

#if defined(CONFIG_FEC_MXC) && !defined(CONFIG_FEC_MXC_PHYADDR)
#error CONFIG_FEC_MXC_PHYADDR missing
#endif

#ifdef CONFIG_FEC_MXC_PHYADDR
#ifdef CONFIG_PHY_ATHEROS
#define ATHEROS_MASK(a) ((1 << (a)) | ((1 << ((a) ^ 4))))
#else
#define ATHEROS_MASK(a) 0
#endif

#if defined(CONFIG_PHY_MICREL) && !defined(CONFIG_PHY_MICREL_KSZ8XXX)
#define KSZ9021_MASK(a) (0xf << ((a) & 4))
#else
#define KSZ9021_MASK(a) 0
#endif

#ifdef CONFIG_PHY_MICREL_KSZ8XXX
#define KSZ8XXX_MASK(a) 2
#else
#define KSZ8XXX_MASK(a) 0
#endif

#define ALL_PHY_MASK(a) (ATHEROS_MASK(a) | KSZ9021_MASK(a) | KSZ8XXX_MASK(a))
#ifdef CONFIG_FEC_ENET2
#define COMBINED_MASK(a) (ALL_PHY_MASK(a) | (ALL_PHY_MASK(a + 1) << 16))
#else
#define COMBINED_MASK(a) ALL_PHY_MASK(a)
#endif

#ifndef ETH_PHY_MASK
#define ETH_PHY_MASK	COMBINED_MASK(CONFIG_FEC_MXC_PHYADDR)
#endif

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

#ifdef CONFIG_MX6SX
#include "eth-mx6sx.c"
#elif defined(CONFIG_MX6ULL)
#include "eth-mx6ull.c"
#elif defined(CONFIG_MX7D)
#include "eth-mx7d.c"
#elif defined(CONFIG_MX51)
#include "eth-mx51.c"
#elif defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN)
#include "eth-imx8mm.c"
#elif defined(CONFIG_IMX8MP)
#include "eth-imx8mp.c"
#elif defined(CONFIG_IMX8MQ)
#include "eth-imx8mq.c"
#else
#include "eth-mx6.c"
#endif

#if !defined(CONFIG_FEC_ENET1) && !defined(CONFIG_FEC_ENET2) && defined(CONFIG_FEC_MXC)
#define CONFIG_FEC_ENET1
#endif

#if defined(CONFIG_PHY_ATHEROS) || defined(CONFIG_PHY_MICREL)
static unsigned char strap_gpios[] = {
#ifdef CONFIG_FEC_ENET1
	GP_PHY_RD0,
	GP_PHY_RD1,
	GP_PHY_RD2,
	GP_PHY_RD3,
#ifdef GP_PHY_RX_CTL
	GP_PHY_RX_CTL,
#endif
#ifdef GP_PHY_RXC
	GP_PHY_RXC,
#endif
#endif

#ifdef CONFIG_FEC_ENET2
	GP_PHY2_RD0,
	GP_PHY2_RD1,
	GP_PHY2_RD2,
	GP_PHY2_RD3,
#ifdef GP_PHY2_RX_CTL
	GP_PHY2_RX_CTL,
#endif
#ifdef GP_PHY2_RXC
	GP_PHY2_RXC,
#endif
#ifdef CONFIG_PHY_MICREL_KSZ8XXX
	GP_PHY2_RX_EN,
	GP_PHY2_ER,
	GP_PHY2_CRS,
	GP_PHY2_COL,
	GP_PHY2_TD0,
	GP_PHY2_TD1,
	GP_PHY2_TD2,
	GP_PHY2_TD3,
#endif
#endif
};

#ifdef CONFIG_MX6ULL
static unsigned char strap_input[] = {
#ifdef CONFIG_FEC_ENET1
	GP_PHY_RD2,
	GP_PHY_RD3,
	GP_PHY_TD2,
	GP_PHY_TD3,
#endif
#ifdef CONFIG_FEC_ENET2
	GP_PHY2_RD2,
	GP_PHY2_RD3,
	GP_PHY2_TD2,
	GP_PHY2_TD3,
#endif
};
#endif

static void set_strap_pins(unsigned strap)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(strap_gpios); i++) {
		gpio_direction_output(strap_gpios[i], strap & 1);
		strap >>= 1;
	}
}

#ifdef CONFIG_MX6ULL
static void set_strap_input(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(strap_input); i++) {
		gpio_direction_input(strap_input[i]);
	}
}
#endif
#endif

#if defined(CONFIG_FEC_MXC) || defined(CONFIG_DWC_ETH_QOS)
#if !defined(CONFIG_MX5)
static void init_fec_clocks(void)
{
#ifdef CONFIG_MX6SX
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	int reg;
	int i;

	/* Use 125MHz anatop loopback REF_CLK1 for ENET1 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK | IOMUX_GPR1_FEC2_MASK, 0);

	reg = readl(&anatop->pll_enet);
	reg |= BM_ANADIG_PLL_ENET_REF_25M_ENABLE;
	writel(reg, &anatop->pll_enet);

	for (i = 0; i < 2; i++) {
		int ret = enable_fec_anatop_clock(i, ENET_125MHZ);
		if (ret) {
			printf("Failed to enable clock (FEC%d): %d\n", i, ret);
			return;
		}
	}
#endif
#ifdef CONFIG_MX6ULL
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	/* Use 50MHz anatop loopback REF_CLK2 for ENET2 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
			IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);


	ret = enable_fec_anatop_clock(1, ENET_50MHZ);
	if (ret) {
		printf("Failed to enable clock (FEC%d): %d\n", 1, ret);
		return;
	}
	enable_enet_clk(1);
#endif
#ifdef CONFIG_MX7D
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
		(IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK |
		 IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK), 0);

	set_clk_enet(ENET_125MHZ);
#endif
#ifdef CONFIG_IMX8M
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

#ifdef CONFIG_FEC_MXC
	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			BIT(13) | BIT(17), 0);
	/* Enable RGMII TX clk output */
	setbits_le32(&iomuxc_gpr_regs->gpr[1], BIT(22));
	set_clk_enet(ENET_125MHZ);
#endif

#ifdef CONFIG_DWC_ETH_QOS
	/* set INTF as RGMII, enable RGMII TXC clock */
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET_QOS_INTF_SEL_MASK, BIT(16));
	setbits_le32(&iomuxc_gpr_regs->gpr[1], BIT(19) | BIT(21));
	set_clk_eqos(ENET_125MHZ);
#endif
#endif
	udelay(100);	/* Wait 100 us before using mii interface */
}
#endif

#if defined(CONFIG_FEC_ENET1) && defined(CONFIG_FEC_ENET2)
#define FEC_ENET1_ID	0	/* FEC0 for imx6sx */
#else
#define FEC_ENET1_ID	-1	/* just plain FEC */
#endif

#if defined(CONFIG_FEC_ENET1)
#define FEC_ENET2_ID	1
#else
#define FEC_ENET2_ID	-1	/* just plain FEC */
#endif

#ifdef CONFIG_MX6ULL
#define PHY_MODE PHY_INTERFACE_MODE_RMII
#else
#define PHY_MODE PHY_INTERFACE_MODE_RGMII
#endif

#ifndef CONFIG_DM_ETH
static void init_fec(struct bd_info *bis, unsigned phy_mask)
{
#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6ULL)
	uint32_t mdio_base = ENET_MDIO_BASE;
#if defined(CONFIG_FEC_ENET1)
	uint32_t base = ENET_BASE_ADDR;
#endif
#else
	uint32_t mdio_base = IMX_FEC_BASE;
	uint32_t base = IMX_FEC_BASE;
#endif
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	bus = fec_get_miibus(mdio_base, -1);
	if (!bus)
		return;
#if defined(CONFIG_FEC_ENET1)
	phydev = phy_find_by_mask(bus, phy_mask & 0xffff, PHY_MODE);
	if (!phydev) {
		printf("%s: phy not found\n", __func__);
		goto error;
	}
	printf("%s at %d\n", phydev->drv->name, phydev->addr);
	ret  = fec_probe(bis, FEC_ENET1_ID, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		goto error;
	}
#endif
#if defined(CONFIG_FEC_ENET2)
	phydev = phy_find_by_mask(bus, phy_mask >> 16, PHY_MODE);
	if (!phydev) {
		printf("%s: phy2 not found\n", __func__);
		goto error;
	}
	printf("%s at %d\n", phydev->drv->name, phydev->addr);
	ret  = fec_probe(bis, FEC_ENET2_ID, ENET2_BASE_ADDR, bus, phydev);
	if (ret) {
		printf("FEC1 MXC: %s:failed\n", __func__);
		free(phydev);
		goto error;
	}
#endif
	return;
error:
	;
	/* Let's leave "mii read" in working state for debug */
#if 0
	mdio_unregister(bus);
	mdio_free(bus);
#endif
}
#endif
#endif

#ifdef CONFIG_PHY_ATHEROS
static void setup_gpio_ar8035(void)
{
	set_strap_pins(STRAP_AR8035);
	SETUP_IOMUX_PADS(enet_ar8035_gpio_pads);
}

static void setup_enet_ar8035(void)
{
	SETUP_IOMUX_PADS(enet_ar8035_pads);
}
#ifndef CONFIG_PHY_MICREL
#define setup_gpio_eth(kz) setup_gpio_ar8035()
#define setup_enet_eth(kz) setup_enet_ar8035()
#endif
#endif

#ifdef CONFIG_PHY_MICREL
#ifdef CONFIG_PHY_MICREL_KSZ8XXX
static void setup_gpio_ksz8863(void)
{
	set_strap_pins(STRAP_KSZ8863);
	SETUP_IOMUX_PADS(enet_ksz8863_gpio_pads);
}

static void setup_enet_ksz8863(void)
{
	SETUP_IOMUX_PADS(enet_ksz8863_pads);
}
#define setup_gpio_micrel setup_gpio_ksz8863
#define setup_enet_micrel setup_enet_ksz8863
#else
static void setup_gpio_ksz9021(void)
{
	set_strap_pins(STRAP_KSZ9021);
	SETUP_IOMUX_PADS(enet_ksz9021_gpio_pads);
}

static void setup_enet_ksz9021(void)
{
	SETUP_IOMUX_PADS(enet_ksz9021_pads);
}
#define setup_gpio_micrel setup_gpio_ksz9021
#define setup_enet_micrel setup_enet_ksz9021
#endif
#ifndef CONFIG_PHY_ATHEROS
#define setup_gpio_eth(kz) setup_gpio_micrel();
#define setup_enet_eth(kz) setup_enet_micrel();
#else
#define setup_gpio_eth(kz) if (kz) setup_gpio_micrel(); else setup_gpio_ar8035();
#define setup_enet_eth(kz) if (kz) setup_enet_micrel(); else setup_enet_ar8035();
#endif
#endif

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

static void setup_iomux_enet(int kz)
{
#ifdef GP_KS8995_RESET
	gpio_direction_output(GP_KS8995_RESET, 0);
#endif
#ifdef GP_RGMII2_PHY_RESET
	gpio_direction_output(GP_RGMII2_PHY_RESET, 0); /* PHY rst */
#endif
#ifdef GP_RGMII_PHY_RESET
	gpio_direction_output(GP_RGMII_PHY_RESET, 0); /* PHY rst */
#endif
	setup_gpio_eth(kz);

#ifdef CONFIG_FEC_MXC
	init_fec_clocks();
#endif
	/* Need delay 10ms according to KSZ9021 spec */
	/* 1 ms minimum reset pulse for ar8035 */
	udelay(1000 * 10);
#ifdef GP_RGMII2_PHY_RESET
	release_phy_reset(GP_RGMII2_PHY_RESET);
#endif

#ifdef GP_RGMII_PHY_RESET
	release_phy_reset(GP_RGMII_PHY_RESET);
#endif

#ifdef GP_KS8995_POWER_DOWN
	gpio_direction_output(GP_KS8995_POWER_DOWN, 1);
#endif

	/* strap hold time for AR8031, 18 fails, 19 works, so 40 should be safe */
	/* strap hold time for AR8035, 5 fails, 6 works, so 12 should be safe */
	udelay(40);
#ifdef GP_KS8995_RESET
	gpio_direction_output(GP_KS8995_RESET, 1);
#endif

	setup_enet_eth(kz);
#ifdef CONFIG_MX6ULL
	set_strap_input();
#endif
}
#endif

#ifdef CONFIG_PHY_ATHEROS
static void phy_ar8031_config(struct phy_device *phydev)
{
	int regval;
	ulong freq;

	/* Select 125MHz clk from local PLL on CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x0007);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);
	regval = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	regval &= ~0x11c;
	regval |= 0x80;	/* 1/2 drive strength */
	freq = env_get_ulong("phy_clock_out", 10, 125000000);
	if (freq >= 125000000) {
		regval |= 0x18;
	} else if (freq >= 62500000) {
		regval |= 0x10;
	} else if (freq >= 50000000) {
		regval |= 0x08;
	} else if (freq == 0) {
		/* 1/4 drive strength since off was requested */
		regval |= 0x180;
	}
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, regval);

#if 0 //done in ar8031_config
	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	regval = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, (regval|0x0100));
#endif
}

static void phy_ar8035_config(struct phy_device *phydev)
{
	int val;
	ulong freq;

	/*
	 * Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x3);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x805d);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4003);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val & ~(1 << 8));

	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x0007);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= ~0x11c;
	val |= 0x80; /* 1/2 drive strength */
	freq = env_get_ulong("phy_clock_out", 10, 125000000);
	if (freq >= 125000000) {
		val |= 0x18;
	} else if (freq >= 62500000) {
		val |= 0x10;
	} else if (freq >= 50000000) {
		val |= 0x08;
	} else if (freq == 0) {
		/* 1/4 drive strength since off was requested */
		val |= 0x180;
	}
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* rgmii tx clock delay enable */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, (val|0x0100));

	phydev->supported = phydev->drv->features;
}

#define PHY_ID_AR8031	0x004dd074
#define PHY_ID_AR8035	0x004dd072

#ifndef CONFIG_PHY_MICREL
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
#endif

#ifdef CONFIG_PHY_MICREL
#define PHY_ID_KSZ9021	0x221610

int board_phy_config(struct phy_device *phydev)
{
#ifdef CONFIG_PHY_ATHEROS
	if (((phydev->drv->uid ^ PHY_ID_AR8031) & 0xffffffef) == 0) {
		phy_ar8031_config(phydev);
	} else if (((phydev->drv->uid ^ PHY_ID_AR8035) & 0xffffffef) == 0) {
		phy_ar8035_config(phydev);
	} else if (((phydev->drv->uid ^ PHY_ID_KSZ9021) & 0xfffffff0) == 0) {
		/* found KSZ, reinit phy for KSZ */
		setup_iomux_enet(1);
#else
	{
#endif
#ifndef CONFIG_PHY_MICREL_KSZ8XXX
		/* min rx data delay */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
		/* min tx data delay */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
		/* max rx/tx clock delay, min rx/tx control */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
#endif
	}
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef GP_KS8995_RESET
static int ks8995_write_rtn(struct spi_slave *spi, u8 *cmds)
{
	int ret = 0;

	pr_debug("%s\n", __func__);
	while (1) {
		uint len = *cmds++;

		if (!len)
			break;

		ret = spi_xfer(spi, len * 8, cmds, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret) {
			pr_debug("%s: Failed spi %02x,%02x,%02x %d\n", __func__, cmds[0], cmds[1], cmds[2], ret);
			return ret;
		}
		pr_debug("spi: len=%02x cmds= %02x,%02x,%02x\n", len, cmds[0], cmds[1], cmds[2]);
		cmds += len;
	}
	return ret;
}

#define KS8995_REG_ID1		0x01    /* Chip ID1 */
#define KS8995_RESET_DELAY	10	/* usec */

static u8 stop_cmds[] = {3, 2, KS8995_REG_ID1, 0, 0};
static u8 start_cmds[] = {3, 2, KS8995_REG_ID1, 1, 0};

static int ks8995_reset(void)
{
	struct spi_slave *spi;
	int ret;

	enable_spi_clk(1, 1);

	/* Setup spi_slave */
	spi = spi_setup_slave(1, 1, 4000000, SPI_MODE_0);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return -EINVAL;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		pr_debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto free_slave;
	}

	ret = ks8995_write_rtn(spi, stop_cmds);
	if (ret)
		return ret;

	udelay(KS8995_RESET_DELAY);

	ret = ks8995_write_rtn(spi, start_cmds);

	/* Release spi bus */
	spi_release_bus(spi);
free_slave:
	spi_free_slave(spi);
	enable_spi_clk(0, 1);

	return ret;
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
#if defined(CONFIG_FEC_ENET1)
	fec_env_set_ethaddr(0);
#endif
#if defined(CONFIG_FEC_ENET2)
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
	gpio_request(GP_PHY_RD2, "fec_rd2");
	gpio_request(GP_PHY_RD3, "fec_rd3");
#ifdef GP_PHY_RX_CTL
	gpio_request(GP_PHY_RX_CTL, "fec_rx_ctl");
#endif
#ifdef GP_PHY_RXC
	gpio_request(GP_PHY_RXC, "fec_rxc");
#endif
#ifdef CONFIG_PHY_MICREL_KSZ8XXX
	gpio_request(GP_PHY_RX_EN, "fec_rx_en");
	gpio_request(GP_PHY_ER, "fec_er");
	gpio_request(GP_PHY_CRS, "fec_crs");
	gpio_request(GP_PHY_COL, "fec_col");
#endif
#endif

#ifdef CONFIG_FEC_ENET2
	gpio_request(GP_RGMII2_PHY_RESET, "fec2_rst");
	gpio_request(GP_PHY2_RD0, "fec2_rd0");
	gpio_request(GP_PHY2_RD1, "fec2_rd1");
	gpio_request(GP_PHY2_RD2, "fec2_rd2");
	gpio_request(GP_PHY2_RD3, "fec2_rd3");
#ifdef GP_PHY2_RX_CTL
	gpio_request(GP_PHY2_RX_CTL, "fec2_rx_ctl");
#endif
#ifdef GP_PHY2_RXC
	gpio_request(GP_PHY2_RXC, "fec2_rxc");
#endif
#ifdef CONFIG_PHY_MICREL_KSZ8XXX
	gpio_request(GP_PHY2_RX_EN, "fec2_rx_en");
	gpio_request(GP_PHY2_ER, "fec2_er");
	gpio_request(GP_PHY2_CRS, "fec2_crs");
	gpio_request(GP_PHY2_COL, "fec2_col");
#endif
#endif
	setup_iomux_enet(0);
#endif
#ifdef GP_KS8995_RESET
	ks8995_reset();
#endif
#ifdef CONFIG_FEC_MXC
#ifndef CONFIG_DM_ETH
	init_fec(bis, ETH_PHY_MASK);
#endif
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

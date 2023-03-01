/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/arch/clock.h>
#ifndef CONFIG_IMX8ULP
#include <asm/arch/crm_regs.h>
#endif
#include <asm/arch/imx-regs.h>
#if defined(CONFIG_MX51)
#include <asm/arch/iomux-mx51.h>
#elif defined(CONFIG_MX7D) || defined(CONFIG_IMX8M) || defined(CONFIG_IMX8ULP)
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
#elif defined(CONFIG_IMX8ULP)
#include <asm/arch/imx8ulp-pins.h>
#elif !defined(CONFIG_MX51)
#include <asm/arch/mx6-pins.h>
#endif
#include <asm/gpio.h>
#include <asm/io.h>
#if !defined(CONFIG_IMX8ULP)
#include <asm/mach-imx/iomux-v3.h>
#endif
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
#if defined(CONFIG_PHY_MICREL) && defined(CONFIG_PHY_ATHEROS)
#define ATHEROS_MASK(a) (1 << (a))
#elif defined(CONFIG_PHY_ATHEROS)
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

#define ALL_ATH_PHY_MASK(a) (ATHEROS_MASK(a))
#define ALL_KSZ_PHY_MASK(a) (KSZ9021_MASK(a) | KSZ8XXX_MASK(a))
#ifdef CONFIG_FEC_ENET2
#define COMBINED_ATH_MASK(a, b) (ALL_ATH_PHY_MASK(a) | (ALL_ATH_PHY_MASK(b) << 16))
#define COMBINED_KSZ_MASK(a, b) (ALL_KSZ_PHY_MASK(a) | (ALL_KSZ_PHY_MASK(b) << 16))
#else
#define COMBINED_ATH_MASK(a, b) ALL_ATH_PHY_MASK(a)
#define COMBINED_KSZ_MASK(a, b) ALL_KSZ_PHY_MASK(a)
#endif

#ifndef CONFIG_FEC_MXC_PHYADDR2
#ifndef CONFIG_FEC_ENET1
#define CONFIG_FEC_MXC_PHYADDR2		CONFIG_FEC_MXC_PHYADDR
#else
#define CONFIG_FEC_MXC_PHYADDR2		(CONFIG_FEC_MXC_PHYADDR + 1)
#endif
#endif

#ifndef CONFIG_FEC_MXC_KSZ_PHYADDR
#define CONFIG_FEC_MXC_KSZ_PHYADDR	CONFIG_FEC_MXC_PHYADDR
#endif

#ifndef CONFIG_FEC_MXC_KSZ_PHYADDR2
#ifndef CONFIG_FEC_ENET1
#define CONFIG_FEC_MXC_KSZ_PHYADDR2	CONFIG_FEC_MXC_KSZ_PHYADDR
#else
#define CONFIG_FEC_MXC_KSZ_PHYADDR2	(CONFIG_FEC_MXC_KSZ_PHYADDR + 1)
#endif
#endif

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
#elif defined(CONFIG_IMX8ULP)
#include "eth-imx8ulp.c"
#else
#include "eth-mx6.c"
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
#ifdef GP_PHY_REF_CLK
	GP_PHY_REF_CLK,
#endif
};
#endif

#ifdef CONFIG_FEC_ENET2
static unsigned char strap2_gpios[] = {
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
#ifdef GP_PHY2_REF_CLK
	GP_PHY2_REF_CLK,
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
};
#endif

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

#ifdef CONFIG_PHY_ATHEROS
#ifdef CONFIG_FEC_ENET1
static void setup_gpio_ar8035(void)
{
	set_strap_pins(strap_gpios, ARRAY_SIZE(strap_gpios), STRAP_AR8035);
	SETUP_IOMUX_PADS(enet_ar8035_gpio_pads);
}

static void setup_enet_ar8035(void)
{
	SETUP_IOMUX_PADS(enet_ar8035_pads);
}
#endif

#ifdef CONFIG_FEC_ENET2
static void setup2_gpio_ar8035(void)
{
	set_strap_pins(strap2_gpios, ARRAY_SIZE(strap2_gpios), STRAP2_AR8035);
	SETUP_IOMUX_PADS(enet2_ar8035_gpio_pads);
}

static void setup2_enet_ar8035(void)
{
	SETUP_IOMUX_PADS(enet2_ar8035_pads);
}
#endif

#ifndef CONFIG_PHY_MICREL
#ifdef CONFIG_FEC_ENET1
#define setup_gpio_eth(kz) setup_gpio_ar8035()
#define setup_enet_eth(kz) setup_enet_ar8035()
#endif
#ifdef CONFIG_FEC_ENET2
#define setup2_gpio_eth(kz) setup2_gpio_ar8035()
#define setup2_enet_eth(kz) setup2_enet_ar8035()
#endif
#endif
#endif

#ifdef CONFIG_PHY_MICREL
#ifdef CONFIG_PHY_MICREL_KSZ8XXX
#ifdef CONFIG_FEC_ENET1
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

#ifdef CONFIG_FEC_ENET2
static void setup2_gpio_ksz8863(void)
{
	set_strap_pins(strap2_gpios, ARRAY_SIZE(strap2_gpios), STRAP2_KSZ8863);
	SETUP_IOMUX_PADS(enet2_ksz8863_gpio_pads);
}

static void setup2_enet_ksz8863(void)
{
	SETUP_IOMUX_PADS(enet2_ksz8863_pads);
}
#define setup2_gpio_micrel setup2_gpio_ksz8863
#define setup2_enet_micrel setup2_enet_ksz8863
#endif

#else
#ifdef CONFIG_FEC_ENET1
static void setup_gpio_ksz9021(void)
{
	set_strap_pins(strap_gpios, ARRAY_SIZE(strap_gpios), STRAP_KSZ9021);
	SETUP_IOMUX_PADS(enet_ksz9021_gpio_pads);
}

static void setup_enet_ksz9021(void)
{
	SETUP_IOMUX_PADS(enet_ksz9021_pads);
}
#define setup_gpio_micrel setup_gpio_ksz9021
#define setup_enet_micrel setup_enet_ksz9021
#endif

#ifdef CONFIG_FEC_ENET2
static void setup2_gpio_ksz9021(void)
{
	set_strap_pins(strap2_gpios, ARRAY_SIZE(strap2_gpios), STRAP2_KSZ9021);
	SETUP_IOMUX_PADS(enet2_ksz9021_gpio_pads);
}

static void setup2_enet_ksz9021(void)
{
	SETUP_IOMUX_PADS(enet2_ksz9021_pads);
}
#define setup2_gpio_micrel setup2_gpio_ksz9021
#define setup2_enet_micrel setup2_enet_ksz9021
#endif
#endif

#ifndef CONFIG_PHY_ATHEROS
#ifdef CONFIG_FEC_ENET1
#define setup_gpio_eth(kz) setup_gpio_micrel();
#define setup_enet_eth(kz) setup_enet_micrel();
#endif
#ifdef CONFIG_FEC_ENET2
#define setup2_gpio_eth(kz) setup2_gpio_micrel();
#define setup2_enet_eth(kz) setup2_enet_micrel();
#endif
#else
#ifdef CONFIG_FEC_ENET1
#define setup_gpio_eth(kz) if (kz) setup_gpio_micrel(); else setup_gpio_ar8035();
#define setup_enet_eth(kz) if (kz) setup_enet_micrel(); else setup_enet_ar8035();
#endif
#ifdef CONFIG_FEC_ENET2
#define setup2_gpio_eth(kz) if (kz) setup2_gpio_micrel(); else setup2_gpio_ar8035();
#define setup2_enet_eth(kz) if (kz) setup2_enet_micrel(); else setup2_enet_ar8035();
#endif
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

#if defined(CONFIG_PHY_ATHEROS) || defined(CONFIG_PHY_MICREL)
#ifdef CONFIG_FEC_ENET1
static char iomux_selection = -1;
#endif
#ifdef CONFIG_FEC_ENET2
static char iomux_selection2 = -1;
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
#ifdef CONFIG_FEC_ENET2
	if (net_mask & 2) {
		if (iomux_selection2 == kz)
			net_mask &= ~2;
		else
			iomux_selection2 = kz;
	}
#endif
	if (!net_mask)
		return;

#ifdef GP_KS8995_RESET
	gpio_direction_output(GP_KS8995_RESET, 0);
#endif
#ifdef GP_RGMII_PHY_RESET
	if (net_mask & 1) {
		gpio_direction_output(GP_RGMII_PHY_RESET, 0); /* PHY rst */
#ifdef GP_RGMII_PHY_RESET2
		gpio_direction_output(GP_RGMII_PHY_RESET2, 0); /* PHY rst */
#endif
	}
#endif
#ifdef GP_RGMII2_PHY_RESET
#ifdef CONFIG_FEC_ENET2
	if (net_mask & 2)
#endif
		gpio_direction_output(GP_RGMII2_PHY_RESET, 0); /* PHY rst */
#endif

#ifdef CONFIG_FEC_ENET1
	if (net_mask & 1) {
		setup_gpio_eth(kz);
	}
#endif
#ifdef CONFIG_FEC_ENET2
	if (net_mask & 2) {
		setup2_gpio_eth(kz);
	}
#endif

#if defined(CONFIG_FEC_MXC) || defined(CONFIG_DWC_ETH_QOS)
	init_fec_clocks();
#endif
	/* Need delay 10ms according to KSZ9021 spec */
	/* 1 ms minimum reset pulse for ar8035 */
	udelay(1000 * 10);
#ifdef GP_RGMII_PHY_RESET
	if (net_mask & 1) {
		gpio_set_value(GP_RGMII_PHY_RESET, 1);
#ifdef GP_RGMII_PHY_RESET2
		gpio_set_value(GP_RGMII_PHY_RESET2, 1);
#endif
	}
#endif
#ifdef GP_RGMII2_PHY_RESET
#ifdef CONFIG_FEC_ENET2
	if (net_mask & 2)
#endif
		gpio_set_value(GP_RGMII2_PHY_RESET, 1);
#endif

#ifdef CONFIG_FEC_RESET_PULLUP
	udelay(20);
#ifdef GP_RGMII_PHY_RESET
	gpio_direction_input(GP_RGMII_PHY_RESET);
#endif
#ifdef GP_RGMII_PHY_RESET2
	gpio_direction_input(GP_RGMII_PHY_RESET2);
#endif
#ifdef GP_RGMII2_PHY_RESET
	gpio_direction_input(GP_RGMII2_PHY_RESET);
#endif
	/* Let external pull have time to pull to guaranteed high */
	udelay(200);
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

#ifdef CONFIG_FEC_ENET1
	if (net_mask & 1) {
		setup_enet_eth(kz);
	}
#endif
#ifdef CONFIG_FEC_ENET2
	if (net_mask & 2) {
		setup2_enet_eth(kz);
	}
#endif
#ifdef CONFIG_MX6ULL
	set_strap_input();
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
	    (((phydev->drv->uid ^ PHY_ID_KSZ9031) & 0xfffffff0) == 0) ||
	    (((phydev->drv->uid ^ PHY_ID_KSZ9131) & 0xfffffff0) == 0))
		return true;
	return false;
}


#if !defined(CONFIG_DM_ETH) && defined(CONFIG_FEC_MXC)
#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6ULL)
#define MDIO_BASE	ENET_MDIO_BASE
#define BASE		ENET_BASE_ADDR
#else
#define MDIO_BASE	IMX_FEC_BASE
#define BASE		IMX_FEC_BASE
#endif

static void init_fec(struct bd_info *bis, unsigned phy_mask_ath, unsigned phy_mask_ksz)
{
	struct mii_dev *bus = NULL;
	uint32_t mdio_base = MDIO_BASE;
#if defined(CONFIG_FEC_ENET1)
	uint32_t base = BASE;
	struct phy_device *phydev1 = NULL;
#endif
#if defined(CONFIG_FEC_ENET2) && !defined(CONFIG_DWC_ETH_QOS)
	struct phy_device *phydev2 = NULL;
#endif
#if defined(CONFIG_FEC_ENET1) || defined(CONFIG_FEC_ENET2)
	int ret;
#endif
#if defined(CONFIG_PHY_ATHEROS) && defined(CONFIG_PHY_MICREL)
	int ksz = 0;
#endif

	bus = fec_get_miibus(mdio_base, -1);
	if (!bus)
		return;
#if defined(CONFIG_FEC_ENET1)
	if (phy_mask_ath & 0xffff)
		phydev1 = phy_find_by_mask(bus, phy_mask_ath & 0xffff, PHY_MODE);
	if (!phydev1 && (phy_mask_ksz & 0xffff)) {
#if defined(CONFIG_PHY_ATHEROS) && defined(CONFIG_PHY_MICREL)
		setup_iomux_enet(1, 3);
		ksz = 1;
#endif
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

#if defined(CONFIG_FEC_ENET2) && !defined(CONFIG_DWC_ETH_QOS)
#if defined(CONFIG_PHY_ATHEROS) && defined(CONFIG_PHY_MICREL)
	if (ksz && (phy_mask_ksz >> 16)) {
		phydev2 = phy_find_by_mask(bus, phy_mask_ksz >> 16, PHY_MODE);
	}
#endif
	if (!phydev2 && (phy_mask_ath >> 16)) {
#if defined(CONFIG_PHY_ATHEROS) && defined(CONFIG_PHY_MICREL)
		setup_iomux_enet(0, 2);
#endif
		phydev2 = phy_find_by_mask(bus, phy_mask_ath >> 16, PHY_MODE);
	}
	if (!phydev2 && (phy_mask_ksz >> 16)) {
#if defined(CONFIG_PHY_ATHEROS) && defined(CONFIG_PHY_MICREL)
		setup_iomux_enet(1, 2);
#endif
		phydev2 = phy_find_by_mask(bus, phy_mask_ksz >> 16, PHY_MODE);
	}
	if (!phydev2) {
		printf("%s: phy2 not found\n", __func__);
		goto error;
	}
	printf("%s at %d\n", phydev2->drv->name, phydev2->addr);
	ret  = fec_probe(bis, FEC_ENET2_ID, ENET2_BASE_ADDR, bus, phydev2);
	if (ret) {
		printf("FEC1 MXC: %s:failed\n", __func__);
		free(phydev2);
		goto error;
	}
#endif
	return;
#if defined(CONFIG_FEC_ENET2) && !defined(CONFIG_DWC_ETH_QOS)
error:
	;
	/* Let's leave "mii read" in working state for debug */
#if 0
	mdio_unregister(bus);
	mdio_free(bus);
#endif
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
#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN) || defined(CONFIG_IMX8MP) || defined(CONFIG_IMX8MQ)
	freq = env_get_ulong("phy_clock_out", 10, 0);
#else
	freq = env_get_ulong("phy_clock_out", 10, 125000000);
#endif
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
#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN) || defined(CONFIG_IMX8MP) || defined(CONFIG_IMX8MQ)
	freq = env_get_ulong("phy_clock_out", 10, 0);
#else
	freq = env_get_ulong("phy_clock_out", 10, 125000000);
#endif
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
#define MII_KSZ9031_EXT_RGMII_COMMON_CTRL	0
#define KSZ9031_LED_MODE_SINGLE			0x10
#define KSZ9031_LED_MODE_TRI_COLOR		0
#define KSZ9031_CLK125MHZ_EN			0x02
#if defined(CONFIG_IMX8MM) || defined(CONFIG_IMX8MN) || defined(CONFIG_IMX8MP) || defined(CONFIG_IMX8MQ) || defined(CONFIG_IMX8ULP) || defined(CONFIG_MX6SX) || defined(CONFIG_MX6ULL)
#define KSZ_CLK_DEFAULT	0		/* Disable 125 Mhz output */
#else
#define KSZ_CLK_DEFAULT	125000000
#endif

#ifndef CONFIG_PHY_MICREL_KSZ8XXX
static void phy_micrel_config(struct phy_device *phydev)
{
	if (((phydev->drv->uid ^ PHY_ID_KSZ9131) & 0xfffffff0) == 0) {
		u32 tmp;

		/* Make leds blink normally, and separately */
		phy_write(phydev, MDIO_DEVAD_NONE, 0x16, 0xa);
		phy_write(phydev, MDIO_DEVAD_NONE, 0x17, 0x8863);
		phy_write(phydev, MDIO_DEVAD_NONE, 0x1a, 0);

		/* read rxc dll control - devaddr = 0x2, register = 0x4c */
		tmp = ksz9031_phy_extended_read(phydev, 0x02,
					MII_KSZ9131_EXT_RGMII_2NS_SKEW_RXDLL,
					MII_KSZ9031_MOD_DATA_NO_POST_INC);
		/* disable rxdll bypass (enable 2ns skew delay on RXC) */
		tmp &= ~MII_KSZ9131_RXTXDLL_BYPASS;
		/* rxc data pad skew 2ns - devaddr = 0x02, register = 0x4c */
		tmp = ksz9031_phy_extended_write(phydev, 0x02,
					MII_KSZ9131_EXT_RGMII_2NS_SKEW_RXDLL,
					MII_KSZ9031_MOD_DATA_NO_POST_INC, tmp);
		/* read txc dll control - devaddr = 0x02, register = 0x4d */
		tmp = ksz9031_phy_extended_read(phydev, 0x02,
					MII_KSZ9131_EXT_RGMII_2NS_SKEW_TXDLL,
					MII_KSZ9031_MOD_DATA_NO_POST_INC);
		/* disable txdll bypass (enable 2ns skew delay on TXC) */
		tmp &= ~MII_KSZ9131_RXTXDLL_BYPASS;
		/* rxc data pad skew 2ns - devaddr = 0x02, register = 0x4d */
		tmp = ksz9031_phy_extended_write(phydev, 0x02,
					MII_KSZ9131_EXT_RGMII_2NS_SKEW_TXDLL,
					MII_KSZ9031_MOD_DATA_NO_POST_INC, tmp);
	}
	if ((((phydev->drv->uid ^ PHY_ID_KSZ9031) & 0xfffffff0) == 0) ||
	    (((phydev->drv->uid ^ PHY_ID_KSZ9131) & 0xfffffff0) == 0)) {
		u32 freq = env_get_ulong("phy_clock_out", 10, KSZ_CLK_DEFAULT);
		u32 led_mod = env_get_ulong("phy_led_mode", 10, 1);
		u32 common_ctrl = led_mod ? KSZ9031_LED_MODE_SINGLE:
				KSZ9031_LED_MODE_TRI_COLOR;

		if (freq)
			common_ctrl |= KSZ9031_CLK125MHZ_EN;
		/* common ctrl - devaddr = 0x02, register = 0x00, tri-color led mode */
		ksz9031_phy_extended_write(phydev, 0x02,
			MII_KSZ9031_EXT_RGMII_COMMON_CTRL,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, common_ctrl);
		/* control data pad skew - devaddr = 0x02, register = 0x04 */
		ksz9031_phy_extended_write(phydev, 0x02,
			MII_KSZ9031_EXT_RGMII_CTRL_SIG_SKEW,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0);
		/* rx data pad skew - devaddr = 0x02, register = 0x05 */
		ksz9031_phy_extended_write(phydev, 0x02,
			MII_KSZ9031_EXT_RGMII_RX_DATA_SKEW,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0);
		/* tx data pad skew - devaddr = 0x02, register = 0x06 */
		ksz9031_phy_extended_write(phydev, 0x02,
			MII_KSZ9031_EXT_RGMII_TX_DATA_SKEW,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0);
		/* gtx and rx clock pad skew - devaddr = 0x02, register = 0x08 */
		ksz9031_phy_extended_write(phydev, 0x02,
			MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x03FF);
	} else {
		/* min rx data delay */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
		/* min tx data delay */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
		/* max rx/tx clock delay, min rx/tx control */
		ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
	}
}
#endif

void __weak board_eth_type(int index, int ksz) {}

int board_phy_config(struct phy_device *phydev)
{
#if defined(CONFIG_FEC_ENET2) && defined(CONFIG_DM_ETH)
#define PHY_INDEX	dev_seq(phydev->dev)
#elif defined(CONFIG_FEC_ENET2) && !defined(CONFIG_DM_ETH)
#define PHY_INDEX	phydev->dev->index
#else
#define PHY_INDEX	0
#endif

	debug("%s: seq=%d, addr = %d\n", __func__, PHY_INDEX, phydev->addr);
#ifdef CONFIG_PHY_ATHEROS
	if (((phydev->drv->uid ^ PHY_ID_AR8031) & 0xffffffef) == 0) {
		phy_ar8031_config(phydev);
		board_eth_type(PHY_INDEX, 0);
	} else if (((phydev->drv->uid ^ PHY_ID_AR8035) & 0xffffffef) == 0) {
		phy_ar8035_config(phydev);
		board_eth_type(PHY_INDEX, 0);
	} else if (is_micrel_part(phydev)) {
		/* found KSZ, reinit phy for KSZ */
		setup_iomux_enet(1, (PHY_INDEX ? 2 : 1));
#else
	{
#endif
#ifndef CONFIG_PHY_MICREL_KSZ8XXX
		phy_micrel_config(phydev);
		board_eth_type(PHY_INDEX, 1);
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
#if defined(CONFIG_FEC_MXC) || defined(CONFIG_DWC_ETH_QOS)
#if defined(CONFIG_FEC_ENET1) && defined(CONFIG_FEC_ENET2)
#define USB_ETH "eth2addr"
#else
#define USB_ETH "eth1addr"
#endif
#else
#define USB_ETH "ethaddr"
#endif
	/* For otg ethernet*/
#if !defined(CONFIG_FEC_MXC) && !defined(CONFIG_DWC_ETH_QOS)
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
#ifdef GP_RGMII_PHY_RESET
	gpio_request(GP_RGMII_PHY_RESET, "fec_rst");
#endif
#ifdef GP_RGMII_PHY_RESET2
	gpio_request(GP_RGMII_PHY_RESET2, "fec_rst2");
#endif
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
#ifdef GP_PHY_REF_CLK
	gpio_request(GP_PHY_REF_CLK, "fec_ref_clk");
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
#ifdef GP_PHY2_REF_CLK
	gpio_request(GP_PHY2_REF_CLK, "fec2_ref_clk");
#endif
#ifdef CONFIG_PHY_MICREL_KSZ8XXX
	gpio_request(GP_PHY2_RX_EN, "fec2_rx_en");
	gpio_request(GP_PHY2_ER, "fec2_er");
	gpio_request(GP_PHY2_CRS, "fec2_crs");
	gpio_request(GP_PHY2_COL, "fec2_col");
#endif
#endif
	setup_iomux_enet(0, 3);
#endif
#ifdef GP_KS8995_RESET
	ks8995_reset();
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

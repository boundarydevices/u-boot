// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <common.h>
#include <asm/arch/boot.h>
#include <asm/arch/eth.h>
#include <asm/arch/sm.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm.h>
#include <env_internal.h>
#include <i2c.h>
#include <init.h>
#include <linux/delay.h>
#include <net.h>
#include <phy.h>

int mmc_get_env_dev(void)
{
	switch (meson_get_boot_device()) {
	case BOOT_DEVICE_EMMC:
		return 2;
	case BOOT_DEVICE_SD:
		return 1;
	default:
		/* boot device is not EMMC|SD */
		return -1;
	}
}

int meson_ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}

#define EFUSE_MAC_OFFSET	0
#define EFUSE_MAC_SIZE		12
#define MAC_ADDR_LEN		6

int misc_init_r(void)
{

#if defined CONFIG_POWER_FUSB302
#include <fusb302.h>
	fusb302_init();
#endif

	u8 mac_addr[MAC_ADDR_LEN];
	char efuse_mac_addr[EFUSE_MAC_SIZE], tmp[3];
	ssize_t len;

	if (!eth_env_get_enetaddr("ethaddr", mac_addr)) {
		len = meson_sm_read_efuse(EFUSE_MAC_OFFSET,
					  efuse_mac_addr, EFUSE_MAC_SIZE);
		if (len != EFUSE_MAC_SIZE)
			return 0;

		/* MAC is stored in ASCII format, 1bytes = 2characters */
		for (int i = 0; i < 6; i++) {
			tmp[0] = efuse_mac_addr[i * 2];
			tmp[1] = efuse_mac_addr[i * 2 + 1];
			tmp[2] = '\0';
			mac_addr[i] = hextoul(tmp, NULL);
		}

		if (is_valid_ethaddr(mac_addr))
			eth_env_set_enetaddr("ethaddr", mac_addr);
		else
			meson_generate_serial_ethaddr();

		eth_env_get_enetaddr("ethaddr", mac_addr);
	}

	return 0;
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

#define PHY_ID_AR8035	0x004dd072

int board_phy_config(struct phy_device *phydev)
{
	if (((phydev->drv->uid ^ PHY_ID_AR8035) & 0xffffffef) == 0)
		phy_ar8035_config(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}

#define GP_RGMII_PHY_RESET	15
#define GP_PHY_RD0		4
#define GP_PHY_RD1		5
#define GP_PHY_RD2		6
#define GP_PHY_RD3		7
#define GP_PHY_RX_CTL		3
#define GP_PHY_RXC		2

static unsigned char strap_gpios[] = {
	GP_PHY_RD0,
	GP_PHY_RD1,
	GP_PHY_RD2,
	GP_PHY_RD3,
	GP_PHY_RX_CTL,
	GP_PHY_RXC,
};

static void set_strap_pins(unsigned strap)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(strap_gpios); i++) {
		gpio_direction_output(strap_gpios[i], strap & 1);
		strap >>= 1;
	}
}

#define CONFIG_ETH_PHYADDR	4
#define STRAP_AR8035	(0x28 | (CONFIG_ETH_PHYADDR & 3))

static void setup_gpio_ar8035(void)
{
	set_strap_pins(STRAP_AR8035);
//	SETUP_IOMUX_PADS(enet_ar8035_gpio_pads);
}

static void setup_enet_ar8035(void)
{
//	SETUP_IOMUX_PADS(enet_ar8035_pads);
}

static void setup_iomux_enet(int kz)
{
	gpio_direction_output(GP_RGMII_PHY_RESET, 0); /* PHY rst */
	setup_gpio_ar8035();

	/* Need delay 10ms according to KSZ9021 spec */
	/* 1 ms minimum reset pulse for ar8035 */
	udelay(1000 * 10);
	gpio_direction_input(GP_RGMII_PHY_RESET);
	/* Let external pull have time to pull to guaranteed high */
	udelay(200);


	/* strap hold time for AR8031, 18 fails, 19 works, so 40 should be safe */
	/* strap hold time for AR8035, 5 fails, 6 works, so 12 should be safe */
	udelay(40);

	setup_enet_ar8035();
}

int board_eth_init(void)
{
	gpio_request(GP_RGMII_PHY_RESET, "eth_rst");
	gpio_request(GP_PHY_RD0, "eth_rd0");
	gpio_request(GP_PHY_RD1, "eth_rd1");
	gpio_request(GP_PHY_RD2, "eth_rd2");
	gpio_request(GP_PHY_RD3, "eth_rd3");
	gpio_request(GP_PHY_RX_CTL, "eth_rx_ctl");
	gpio_request(GP_PHY_RXC, "eth_rxc");

	setup_iomux_enet(0);
	return 0;
}

int meson_board_late_init(void)
{
	board_eth_init();
	return 0;
}

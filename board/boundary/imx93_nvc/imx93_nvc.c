// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 Ezurio
 */

#include <env.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/arch-imx9/ccm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch-imx9/imx93_pins.h>
#include <asm/arch/clock.h>
#include <power/pmic.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <asm/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_FSEL2)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	MX93_PAD_UART1_RXD__LPUART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX93_PAD_UART1_TXD__LPUART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(LPUART1_CLK_ROOT);

	return 0;
}

static void board_gpio_init(void)
{
	struct gpio_desc desc;
	int ret;

	/* Enable nRESET_BLE output-high for BL654 */
	ret = dm_gpio_lookup_name("GPIO2_26", &desc);
	if (ret)
		return;

	ret = dm_gpio_request(&desc, "nRESET_BLE_HIGH");
	if (ret)
		return;

	dm_gpio_set_value(&desc, 1);
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
};

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

static int setup_eqos(void)
{
	struct blk_ctrl_wakeupmix_regs *bctrl =
	    (struct blk_ctrl_wakeupmix_regs *)BLK_CTRL_WAKEUPMIX_BASE_ADDR;

	/* set INTF as RGMII, enable RGMII TXC clock */
	clrsetbits_le32(&bctrl->eqos_gpr,
			BCTRL_GPR_ENET_QOS_INTF_MODE_MASK,
			BCTRL_GPR_ENET_QOS_INTF_SEL_RGMII |
			BCTRL_GPR_ENET_QOS_CLK_GEN_EN);

	return set_clk_eqos(ENET_125MHZ);
}

int board_init(void)
{
	if (CONFIG_IS_ENABLED(DWC_ETH_QOS))
		setup_eqos();

	board_gpio_init();

	return 0;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

int board_late_init(void)
{
	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

	if (!env_get("serial#")) {
		unsigned char mac_address[8];
		char serialbuf[20];

		imx_get_mac_from_fuse(0, mac_address);
		snprintf(serialbuf, sizeof(serialbuf),
			 "%02x%02x%02x%02x%02x%02x", mac_address[0],
			 mac_address[1], mac_address[2], mac_address[3],
			 mac_address[4], mac_address[5]);
		printf("serial %s\n", serialbuf);
		env_set("serial#", serialbuf);
	}

	return 0;
}

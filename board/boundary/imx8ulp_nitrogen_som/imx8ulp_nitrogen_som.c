// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Ezurio LLC
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
#include <power-domain.h>
#include <dt-bindings/power/imx8ulp-power.h>

DECLARE_GLOBAL_DATA_PTR;

#define PMIC_I2C_PAD_CTRL	(PAD_CTL_PUS_UP | PAD_CTL_SRE_SLOW | PAD_CTL_ODE)
#define PMIC_MODE_PAD_CTRL	(PAD_CTL_PUS_UP)
#define PAD_CTRL_ENET_RX_UP     (PAD_CTL_DSE | PAD_CTL_PUS_UP)

static iomux_cfg_t const init_pads[] = {
	IMX8ULP_PAD_PTE20__LPI2C5_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTE21__LPI2C5_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTE15__PTE15 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTF26__PTF26 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTE11__PTE11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTF16__LPI2C6_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTF17__LPI2C6_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTD11__PTD11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTD16__PTD16 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTD17__PTD17 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTD18__PTD18 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTD19__PTD19 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE0__PTE0 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE2__PTE2 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE3__PTE3 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE6__PTE6 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE7__PTE7 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTE8__PTE8 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTE9__PTE9 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTE10__PTE10 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTE11__PTE11 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE14__PTE14 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTE16__PTE16 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE17__PTE17 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE18__PTE18 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE19__PTE19 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
#if IS_ENABLED(CONFIG_FEC_MXC)
	IMX8ULP_PAD_PTF8__ENET0_MDIO | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF9__ENET0_MDC | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF6__ENET0_CRS_DV | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF5__ENET0_RXER | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF1__ENET0_RXD0 | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF0__ENET0_RXD1 | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF4__ENET0_TXEN | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF3__ENET0_TXD0 | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF2__ENET0_TXD1 | MUX_PAD_CTRL(PAD_CTRL_ENET_RX_UP),
	IMX8ULP_PAD_PTF7__ENET0_REFCLK | MUX_PAD_CTRL((PAD_CTRL_ENET_RX_UP | PAD_CTL_IBE_ENABLE)),
	IMX8ULP_PAD_PTF11__PTF11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTF10__PTF10 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
#endif
	IMX8ULP_PAD_PTF13__PTF13 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTF18__PTF18 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTF19__PTF19 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTF20__PTF20 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTF21__PTF21 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTF25__PTF25 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTF27__PTF27 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTF28__PTF28 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
};

#if IS_ENABLED(CONFIG_FEC_MXC)
static int setup_fec(void)
{
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

int board_init(void)
{
#if defined(CONFIG_FEC_MXC)
	setup_fec();
#endif

	return 0;
}

int board_early_init_f(void)
{
	imx8ulp_iomux_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));

	return 0;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

int board_late_init(void)
{
	ulong addr;

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

	/* clear fdtaddr to avoid obsolete data */
	addr = env_get_hex("fdt_addr_r", 0);
	if (addr)
		memset((void *)addr, 0, 0x400);

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

void board_quiesce_devices(void)
{
	/* Disable the power domains may used in u-boot before entering kernel */
#if CONFIG_IS_ENABLED(POWER_DOMAIN)
	struct udevice *scmi_devpd;
	int ret, i;
	struct power_domain pd;
	ulong ids[] = {
		IMX8ULP_PD_USB0, IMX8ULP_PD_USDHC0, IMX8ULP_PD_USDHC1,
		IMX8ULP_PD_USDHC2_USB1, IMX8ULP_PD_DCNANO, IMX8ULP_PD_MIPI_DSI};

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

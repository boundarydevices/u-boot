/*
 * Copyright (C) 2018, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
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

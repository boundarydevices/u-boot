/*
 * Copyright (C) 2018, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#define GPIOD	4	/* From dts aliases gpio4 = &gpiod */
#define GPIOE	5	/* From dts aliases gpio5 = &gpioe */
#define GPIOF	6	/* From dts aliases gpio6 = &gpiof */

#define GPIO_NR(port, index)	(((GPIO##port)*32)+((index)&31))

#define GP_PHY_RD0	GPIO_NR(F, 1)
#define GP_PHY_RD1	GPIO_NR(F, 0)
#define GP_PHY_ER	GPIO_NR(F, 5)
#define GP_PHY_CRS	GPIO_NR(F, 6)

#ifndef STRAP_AR8035
#define STRAP_AR8035	(0x28 | (CONFIG_FEC_MXC_PHYADDR & 3))
#endif

#ifndef GP_RGMII_PHY_RESET
#define GP_RGMII_PHY_RESET	GPIO_NR(F, 11)
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
#if GP_RGMII_PHY_RESET == GPIO_NR(F, 11)
	IOMUX_PAD_CTRL(PTF11__PTF11, WEAK_PULLDN_OUTPUT),
#endif
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

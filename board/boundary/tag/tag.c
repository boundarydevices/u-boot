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
#include <asm/mach-imx/fbpanel.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/gpio.h>

#define IMX_GPIO_NR(port, index)	((((port)-1)*32)+((index)&31))

DECLARE_GLOBAL_DATA_PTR;

#if IS_ENABLED(CONFIG_FEC_MXC)
#define ENET_CLK_PAD_CTRL	(PAD_CTL_PUS_UP | PAD_CTL_DSE | PAD_CTL_IBE_ENABLE)
static iomux_cfg_t const init_pads[] = {
	IMX8ULP_PAD_PTF7__ENET0_REFCLK | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	IMX8ULP_PAD_PTE13__ENET0_1588_CLKIN | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),

	IMX8ULP_PAD_PTE4__LPI2C5_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTE5__LPI2C5_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
#define GPIRQ_TS_ATMEL		IMX_GPIO_NR(2, 15)
	IMX8ULP_PAD_PTE15__PTE15 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
#define GP_TS_ATMEL_RESET	IMX_GPIO_NR(2, 11)
	IMX8ULP_PAD_PTE11__PTE11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#if 0
	IMX8ULP_PAD_PTE16__LPI2C4_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTE17__LPI2C4_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
#endif
	IMX8ULP_PAD_PTB8__PTB8,
	IMX8ULP_PAD_PTB9__PTB9,
	IMX8ULP_PAD_PTF16__LPI2C6_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTF17__LPI2C6_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
#define GP_I2C6_HAPTICS_EN	IMX_GPIO_NR(1, 11)
	IMX8ULP_PAD_PTD11__PTD11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#if 0
	IMX8ULP_PAD_PTF20__LPI2C7_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTF21__LPI2C7_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
#endif
#if 0
#define GP_PTE16	IMX_GPIO_NR(2, 16)
	IMX8ULP_PAD_PTF16__PTF16 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#define GP_PTE17	IMX_GPIO_NR(2, 17)
	IMX8ULP_PAD_PTF17__PTF17 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#define GP_PTF16	IMX_GPIO_NR(3, 16)
	IMX8ULP_PAD_PTF16__PTF16 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#define GP_PTF17	IMX_GPIO_NR(3, 17)
	IMX8ULP_PAD_PTF17__PTF17 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#define GP_PTF20	IMX_GPIO_NR(3, 20)
	IMX8ULP_PAD_PTF20__PTF20 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#define GP_PTF21	IMX_GPIO_NR(3, 21)
	IMX8ULP_PAD_PTF21__PTF21 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#endif
};

static int setup_fec(void)
{
	/* Select enet time stamp clock: 001 - External Timestamp Clock */
	cgc1_enet_stamp_sel(1);

	/* enable FEC PCC */
	pcc_clock_enable(4, ENET_PCC4_SLOT, true);
	pcc_reset_peripheral(4, ENET_PCC4_SLOT, false);

	return 0;
}
#endif

static const iomux_cfg_t mipi_pads[] = {
	IMX8ULP_PAD_PTA6__PTA6 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),	/* mipi display reset */
	IMX8ULP_PAD_PTC14__PTC14 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),	/* mipi backlight enable */
#if 0
	IMX8ULP_PAD_PTC15__TPM3_CH3 | MUX_PAD_CTRL(PAD_CTL_DSE),	/* mipi backlight pwm */
#else
	IMX8ULP_PAD_PTC15__PTC15 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),	/* mipi backlight pwm */
#endif
};

static const struct display_info_t displays[] = {
	VD_Q035_014(MIPI, NULL, fbp_bus_gp(0, 0, 0, 0), 0x0),
};
#define display_cnt	ARRAY_SIZE(displays)

static void mipi_dsi_panel_backlight(void)
{
	/* It is temp solution to directly access pwm, need change to rpmsg later */
	imx8ulp_iomux_setup_multiple_pads(mipi_pads, ARRAY_SIZE(mipi_pads));
	writel(0xD4000001, 0x28102018);	/* PCC_TPM3, (option 4)/2 */

	/* Use center-aligned PWM mode, CPWMS=1, MSnB:MSnA = 10, ELSnB:ELSnA = 00 */
	writel(1000, 0x28106018);
	writel(1000, 0x2810603c); /* MOD = CV, full duty */
	writel(0x28, 0x28106010);
	writel(0x20, 0x28106038);
}

int board_init(void)
{
	gpio_request(GP_TS_ATMEL_RESET, "atmel_reset");
	gpio_request(GP_I2C6_HAPTICS_EN, "haptic_en");
	gpio_direction_output(GP_TS_ATMEL_RESET, 0);
	gpio_direction_output(GP_I2C6_HAPTICS_EN, 0);
#ifdef GP_PTE16
	gpio_request(GP_PTE16, "pte16");
	gpio_request(GP_PTE17, "pte17");
	gpio_request(GP_PTF16, "ptf16");
	gpio_request(GP_PTF17, "ptf17");
	gpio_request(GP_PTF20, "ptf20");
	gpio_request(GP_PTF21, "ptf21");
	gpio_direction_output(GP_PTE16, 0);
	gpio_direction_output(GP_PTE17, 0);
	gpio_direction_output(GP_PTF16, 0);
	gpio_direction_output(GP_PTF17, 0);
	gpio_direction_output(GP_PTF20, 0);
	gpio_direction_output(GP_PTF21, 0);
#endif

#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	mipi_dsi_panel_backlight();
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif

	return 0;
}

int board_early_init_f(void)
{
	imx8ulp_iomux_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	return 0;
}

// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Boundary Devices
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
#include <power-domain.h>
#include <dt-bindings/power/imx8ulp-power.h>
#include "../common/padctrl.h"

#define GPIOD	4	/* From dts aliases gpio4 = &gpiod */
#define GPIOE	5	/* From dts aliases gpio5 = &gpioe */
#define GPIOF	6	/* From dts aliases gpio6 = &gpiof */

#define GPIO_NR(port, index)	(((GPIO##port)*32)+((index)&31))

DECLARE_GLOBAL_DATA_PTR;

#define PMIC_I2C_PAD_CTRL	(PAD_CTL_PUS_UP | PAD_CTL_SRE_SLOW | PAD_CTL_ODE)
#define PMIC_MODE_PAD_CTRL	(PAD_CTL_PUS_UP)

static iomux_cfg_t const init_pads_m33[] = {
	IMX8ULP_PAD_PTB11__PMIC0_SCL | MUX_PAD_CTRL(PMIC_I2C_PAD_CTRL),
	IMX8ULP_PAD_PTB10__PMIC0_SDA | MUX_PAD_CTRL(PMIC_I2C_PAD_CTRL),
	IMX8ULP_PAD_PTB8__PTB8,
	IMX8ULP_PAD_PTB9__PTB9,
	IMX8ULP_PAD_PTA0__PTA0 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTA16__PTA16 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTA21__PTA21 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTA22__PTA22 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTA23__PTA23 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTB0__LPI2C2_SCL | MUX_PAD_CTRL(PMIC_I2C_PAD_CTRL),
	IMX8ULP_PAD_PTB1__LPI2C2_SDA | MUX_PAD_CTRL(PMIC_I2C_PAD_CTRL),
	IMX8ULP_PAD_PTB7__PTB7 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTB13__PTB13 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTB14__PTB14 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC0__PTC0 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC3__PTC3 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC6__PTC6 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC7__PTC7 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC8__PTC8 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC9__PTC9 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC10__PTC10 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC11__PTC11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTC13__PTC13 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC16__PTC16 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTC17__PTC17 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC20__PTC20 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC21__PTC21 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTC22__PTC22 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
};

static iomux_cfg_t const init_pads[] = {
#if IS_ENABLED(CONFIG_FEC_MXC)
	IMX8ULP_PAD_PTE20__LPI2C5_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTE21__LPI2C5_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
#else
	IMX8ULP_PAD_PTE4__LPI2C5_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTE5__LPI2C5_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
#endif
#define GPIRQ_TS_ATMEL		GPIO_NR(E, 15)
	IMX8ULP_PAD_PTE15__PTE15 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
#if IS_ENABLED(CONFIG_FEC_MXC)
#define GP_TS_ATMEL_RESET	GPIO_NR(F, 26)
#else
#define GP_TS_ATMEL_RESET	GPIO_NR(E, 11)
#endif
	IMX8ULP_PAD_PTF26__PTF26 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTE11__PTE11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTF16__LPI2C6_SCL | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
	IMX8ULP_PAD_PTF17__LPI2C6_SDA | MUX_PAD_CTRL(PAD_CTL_PUS_UP | PAD_CTL_ODE),
#define GP_I2C6_HAPTICS_EN	GPIO_NR(D, 11)
	IMX8ULP_PAD_PTD11__PTD11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTD16__PTD16 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTD17__PTD17 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTD18__PTD18 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTD19__PTD19 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE0__PTE0 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE2__PTE2 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE3__PTE3 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTE6__PTE6 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
#define GP_LCD_RESET		GPIO_NR(E, 7)
	IMX8ULP_PAD_PTE7__PTE7 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#define GP_BACKLIGHT_PWM	GPIO_NR(E, 8)
	IMX8ULP_PAD_PTE8__PTE8 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#define GP_BACKLIGHT_EN		GPIO_NR(E, 9)
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
#define GP_FEC_RESET	GPIO_NR(F, 11)
	IMX8ULP_PAD_PTF11__PTF11 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
#define GPIRQ_FEC_PHY	GPIO_NR(F, 10)
	IMX8ULP_PAD_PTF10__PTF10 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
#else
	IMX8ULP_PAD_PTF8__PTF8 | MUX_PAD_CTRL(PAD_CTL_PUS_UP),
	IMX8ULP_PAD_PTF9__PTF9 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTF6__PTF6 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTF4__PTF4 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
	IMX8ULP_PAD_PTF7__PTF7 | MUX_PAD_CTRL(PAD_CTL_PUS_DOWN),
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
	VD_Q035_014(MIPI, fbp_detect_i2c, fbp_bus_gp(5, GP_TS_ATMEL_RESET, GP_LCD_RESET, 50), 0x4a, FBTS_ATMEL_MT),
	VD_MIPI_COM35H3R04ULY(MIPI, NULL, fbp_bus_gp(0, 0, GP_LCD_RESET, 0), 0x0),
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
	gpio_request(GP_BACKLIGHT_PWM, "backlight_pwm");
	gpio_request(GP_BACKLIGHT_EN, "backlight_en");
	gpio_direction_output(GP_BACKLIGHT_PWM, 1);
	gpio_direction_output(GP_BACKLIGHT_EN, 1);

#ifdef CONFIG_DM_ETH
	board_eth_init(gd->bd);
#endif
#if defined(CONFIG_FEC_MXC)
	setup_fec();
#endif

	/* When sync with M33 is failed, use local driver to set for video */
	if (!is_m33_handshake_necessary() && IS_ENABLED(CONFIG_DM_VIDEO)) {
		mipi_dsi_panel_backlight();
	}
#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, display_cnt);
#endif

	return 0;
}

int board_early_init_f(void)
{
	imx8ulp_iomux_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	if (!m33_image_booted())
		imx8ulp_iomux_setup_multiple_pads(init_pads_m33, ARRAY_SIZE(init_pads_m33));
	gpio_request(GP_LCD_RESET, "lcd_rst");
	gpio_direction_output(GP_LCD_RESET, 0);
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
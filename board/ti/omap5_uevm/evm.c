/*
 * (C) Copyright 2010
 * Texas Instruments Incorporated, <www.ti.com>
 * Aneesh V       <aneesh@ti.com>
 * Steve Sakoman  <steve@sakoman.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <palmas.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <tca642x.h>

#include "mux_data.h"

#if defined(CONFIG_USB_EHCI) || defined(CONFIG_USB_XHCI_OMAP)
#include <sata.h>
#include <usb.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/ehci.h>
#include <asm/ehci-omap.h>
#include <asm/arch/sata.h>

#define DIE_ID_REG_BASE     (OMAP54XX_L4_CORE_BASE + 0x2000)
#define DIE_ID_REG_OFFSET	0x200

#endif

DECLARE_GLOBAL_DATA_PTR;

const struct omap_sysinfo sysinfo = {
	"Board: OMAP5432 uEVM\n"
};

/**
 * @brief tca642x_init - uEVM default values for the GPIO expander
 * input reg, output reg, polarity reg, configuration reg
 */
struct tca642x_bank_info tca642x_init[] = {
	{ .input_reg = 0x00,
	  .output_reg = 0x04,
	  .polarity_reg = 0x00,
	  .configuration_reg = 0x80 },
	{ .input_reg = 0x00,
	  .output_reg = 0x00,
	  .polarity_reg = 0x00,
	  .configuration_reg = 0xff },
	{ .input_reg = 0x00,
	  .output_reg = 0x00,
	  .polarity_reg = 0x00,
	  .configuration_reg = 0x40 },
};

/**
 * @brief board_init
 *
 * @return 0
 */
int board_init(void)
{
	gpmc_init();
	gd->bd->bi_arch_number = MACH_TYPE_OMAP5_SEVM;
	gd->bd->bi_boot_params = (0x80000000 + 0x100); /* boot param addr */

	tca642x_set_inital_state(CONFIG_SYS_I2C_TCA642X_ADDR, tca642x_init);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	return 0;
}

#if defined(CONFIG_USB_EHCI) || defined(CONFIG_USB_XHCI_OMAP)
static void enable_host_clocks(void)
{
	int auxclk;
	int hs_clk_ctrl_val = (OPTFCLKEN_HSIC60M_P3_CLK |
				OPTFCLKEN_HSIC480M_P3_CLK |
				OPTFCLKEN_HSIC60M_P2_CLK |
				OPTFCLKEN_HSIC480M_P2_CLK |
				OPTFCLKEN_UTMI_P3_CLK | OPTFCLKEN_UTMI_P2_CLK);

	/* Enable port 2 and 3 clocks*/
	setbits_le32((*prcm)->cm_l3init_hsusbhost_clkctrl, hs_clk_ctrl_val);

	/* Enable port 2 and 3 usb host ports tll clocks*/
	setbits_le32((*prcm)->cm_l3init_hsusbtll_clkctrl,
			(OPTFCLKEN_USB_CH1_CLK_ENABLE | OPTFCLKEN_USB_CH2_CLK_ENABLE));
#ifdef CONFIG_USB_XHCI_OMAP
	/* Enable the USB OTG Super speed clocks */
	setbits_le32((*prcm)->cm_l3init_usb_otg_ss_clkctrl,
			(OPTFCLKEN_REFCLK960M | OTG_SS_CLKCTRL_MODULEMODE_HW));
#endif

	auxclk = readl((*prcm)->scrm_auxclk1);
	/* Request auxilary clock */
	auxclk |= AUXCLK_ENABLE_MASK;
	writel(auxclk, (*prcm)->scrm_auxclk1);
}
#endif

/**
 * @brief misc_init_r - Configure EVM board specific configurations
 * such as power configurations, ethernet initialization as phase2 of
 * boot sequence
 *
 * @return 0
 */
int misc_init_r(void)
{
	int reg;
	u32 id[4];

#ifdef CONFIG_PALMAS_POWER
	palmas_init_settings();
#endif

	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;

	id[0] = readl(reg);
	id[1] = readl(reg + 0x8);
	id[2] = readl(reg + 0xC);
	id[3] = readl(reg + 0x10);
	usb_fake_mac_from_die_id(id);

	return 0;
}

void set_muxconf_regs_essential(void)
{
	do_set_mux((*ctrl)->control_padconf_core_base,
		   core_padconf_array_essential,
		   sizeof(core_padconf_array_essential) /
		   sizeof(struct pad_conf_entry));

	do_set_mux((*ctrl)->control_padconf_wkup_base,
		   wkup_padconf_array_essential,
		   sizeof(wkup_padconf_array_essential) /
		   sizeof(struct pad_conf_entry));
}

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_GENERIC_MMC)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0, -1, -1);
	omap_mmc_init(1, 0, 0, -1, -1);
	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI
static struct omap_usbhs_board_data usbhs_bdata = {
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_HSIC,
	.port_mode[2] = OMAP_EHCI_PORT_MODE_HSIC,
};

int ehci_hcd_init(int index, enum usb_init_type init,
		struct ehci_hccr **hccr, struct ehci_hcor **hcor)
{
	int ret;

	enable_host_clocks();

	ret = omap_ehci_hcd_init(index, &usbhs_bdata, hccr, hcor);
	if (ret < 0) {
		puts("Failed to initialize ehci\n");
		return ret;
	}

	return 0;
}

int ehci_hcd_stop(void)
{
	int ret;

	ret = omap_ehci_hcd_stop();
	return ret;
}

void usb_hub_reset_devices(int port)
{
	/* The LAN9730 needs to be reset after the port power has been set. */
	if (port == 3) {
		gpio_direction_output(CONFIG_OMAP_EHCI_PHY3_RESET_GPIO, 0);
		udelay(10);
		gpio_direction_output(CONFIG_OMAP_EHCI_PHY3_RESET_GPIO, 1);
	}
}
#endif

#ifdef CONFIG_USB_XHCI_OMAP
/**
 * @brief board_usb_init - Configure EVM board specific configurations
 * for the LDO's and clocks for the USB blocks.
 *
 * @return 0
 */
int board_usb_init(int index, enum usb_init_type init)
{
	int ret;
#ifdef CONFIG_PALMAS_USB_SS_PWR
	ret = palmas_enable_ss_ldo();
#endif

	enable_host_clocks();

	return 0;
}
#endif

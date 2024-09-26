// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Ezurio LLC
 */

#include <common.h>
#include <env.h>
#include <init.h>
#include <asm/global_data.h>
#include <asm/arch-imx9/ccm_regs.h>
#include <asm/arch/clock.h>
#include <fdt_support.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <asm/io.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/gpio.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <dm/uclass.h>
#include <dm/uclass-internal.h>

#ifdef CONFIG_SCMI_FIRMWARE
#include <scmi_agent.h>
#include <scmi_protocols.h>
#include <dt-bindings/clock/fsl,imx95-clock.h>
#include <dt-bindings/power/fsl,imx95-power.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

int board_early_init_f(void)
{
	/* UART1: A55, UART2: M33, UART3: M7 */
	init_uart_clk(0);

	return 0;
}

static int imx9_scmi_power_domain_enable(u32 domain, bool enable)
{
	return scmi_pwd_state_set(gd->arch.scmi_dev, 0, domain, enable ? 0 : BIT(30));
}

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	if (init == USB_INIT_DEVICE) {
		ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true);
		if (ret) {
			printf("SCMI_POWWER_STATE_SET Failed for USB\n");
			return ret;
		}
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE) {
#ifdef CONFIG_USB_DWC3
		dwc3_uboot_exit(index);
#endif
	} else if (index == 0 && init == USB_INIT_HOST) {
	}

	return ret;
}

static void netc_phy_rst(const char *gpio_name, const char *label)
{
	int ret;
	struct gpio_desc desc;

	/* ENET_RST_B */
	ret = dm_gpio_lookup_name(gpio_name, &desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, gpio_name, ret);
		return;
	}

	ret = dm_gpio_request(&desc, label);
	if (ret) {
		printf("%s request %s failed ret = %d\n", __func__, label, ret);
		return;
	}

	/* assert the ENET_RST_B */
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);
	udelay(10000);
	dm_gpio_set_value(&desc, 0); /* deassert the ENET_RST_B */
	udelay(80000);

}

void netc_init(void)
{
	int ret;

	/* Power up the NETC MIX. */
	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, true);
	if (ret) {
		printf("SCMI_POWWER_STATE_SET Failed for NETC MIX\n");
		return;
	}

	set_clk_netc(ENET_125MHZ);

	netc_phy_rst("GPIO2_29", "ENET1_RST_B");
	netc_phy_rst("GPIO2_18", "ENET2_RST_B");

	pci_init();
}

#if CONFIG_IS_ENABLED(NET)
int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

int board_init(void)
{
	int ret;

	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true);
	if (ret) {
		printf("SCMI_POWWER_STATE_SET Failed for USB\n");
		return ret;
	}

	imx9_scmi_power_domain_enable(IMX95_PD_DISPLAY, false);
	imx9_scmi_power_domain_enable(IMX95_PD_CAMERA, false);

	netc_init();

	return 0;
}

int board_late_init(void)
{
	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif
	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}
#endif

int board_phys_sdram_size(phys_size_t *size)
{
	*size = PHYS_SDRAM_SIZE + PHYS_SDRAM_2_SIZE;

	return 0;
}

void board_quiesce_devices(void)
{
	int ret;
	struct uclass *uc_dev;

	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, false);
	if (ret) {
		printf("%s: Failed for HSIO MIX: %d\n", __func__, ret);
		return;
	}

	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, false);
	if (ret) {
		printf("%s: Failed for NETC MIX: %d\n", __func__, ret);
		return;
	}

	ret = uclass_get(UCLASS_SPI_FLASH, &uc_dev);
	if (uc_dev)
		ret = uclass_destroy(uc_dev);
	if (ret)
		printf("couldn't remove SPI FLASH devices\n");
}

#if IS_ENABLED(CONFIG_OF_BOARD_FIXUP)
static int imx9_scmi_misc_cfginfo(u32 *msel, char *cfgname)
{
	struct scmi_cfg_info_out out;
	struct scmi_msg msg = SCMI_MSG(SCMI_PROTOCOL_ID_MISC, SCMI_MISC_CFG_INFO, out);
	int ret;

	ret = devm_scmi_process_msg(gd->arch.scmi_dev, &msg);
	if(ret == 0 && out.status == 0) {
		strcpy(cfgname, (const char *)out.cfgname);
	} else {
		printf("Failed to get cfg name, scmi_err = %d\n",
		       out.status);
		return -EINVAL;
	}

	return 0;
}

static void disable_fdt_resources(void *fdt)
{
	int i = 0;
	int nodeoff, ret;
	const char *status = "disabled";
	static const char * const dsi_nodes[] = {
		"/soc@0/bus@42000000/i2c@426b0000",
		"/soc@0/bus@42000000/i2c@426d0000",
		"/pcie@4ca00000",
		"/pcie@4cb00000"
	};

	for (i = 0; i < ARRAY_SIZE(dsi_nodes); i++) {
		nodeoff = fdt_path_offset(fdt, dsi_nodes[i]);
		if (nodeoff > 0) {
set_status:
			ret = fdt_setprop(fdt, nodeoff, "status", status,
					  strlen(status) + 1);
			if (ret == -FDT_ERR_NOSPACE) {
				ret = fdt_increase_size(fdt, 512);
				if (!ret)
					goto set_status;
			}
		}
	}
}

static int board_fix_nitrogen_smarc(void *fdt)
{
	char cfgname[SCMI_MISC_MAX_CFGNAME];
	u32 msel;
	int ret;
	const char *netcfg = "mx95netc";

	ret = imx9_scmi_misc_cfginfo(&msel, cfgname);
	if (!ret) {
		debug("SM: %s\n", cfgname);
		if (!strcmp(netcfg, cfgname))
			disable_fdt_resources(fdt);
	}

	return 0;
}

int board_fix_fdt(void *fdt)
{
	return board_fix_nitrogen_smarc(fdt);
}
#endif

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

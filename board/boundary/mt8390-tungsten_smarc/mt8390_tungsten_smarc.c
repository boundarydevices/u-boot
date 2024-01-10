// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 MediaTek Inc.
 * Author: Chris-QJ Chen <chris-qj.chen@mediatek.com>
 */

#include <common.h>
#include <dm.h>
#include <efi_loader.h>
#include <net.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/arm-smccc.h>
#include <power/regulator.h>

#define MT8390_UPDATABLE_IMAGES	5

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
static struct efi_fw_image fw_images[MT8390_UPDATABLE_IMAGES] = {0};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=bl2.img raw 0x0 0x400000 mmcpart 1;"
			"fip.bin part 0 1;firmware.vfat part 0 3;u-boot-env.bin raw 0x0 0x400000 mmcpart 2",
	.images = fw_images,
};

u8 num_image_type_guids = MT8390_UPDATABLE_IMAGES;
#endif

#if defined(CONFIG_EFI_HAVE_CAPSULE_SUPPORT) && defined(CONFIG_EFI_PARTITION)
enum mt8390_updatable_images {
	MT8390_BL2_IMAGE = 1,
	MT8390_FIP_IMAGE,
	MT8390_FW_IMAGE,
	MT8390_ENV_IMAGE,
	MT8390_FIT_IMAGE,
};

void mediatek_capsule_update_board_setup(void)
{
	fw_images[0].image_index = MT8390_FIT_IMAGE;
	fw_images[1].image_index = MT8390_FIP_IMAGE;
	fw_images[2].image_index = MT8390_BL2_IMAGE;
	fw_images[3].image_index = MT8390_FW_IMAGE;
	fw_images[4].image_index = MT8390_ENV_IMAGE;

	efi_guid_t image_type_guid = MT8390_TUNGSTEN_SMARC_FIT_IMAGE_GUID;
	efi_guid_t uboot_image_type_guid = MT8390_TUNGSTEN_SMARC_FIP_IMAGE_GUID;
	efi_guid_t bl2_image_type_guid = MT8390_TUNGSTEN_SMARC_BL2_IMAGE_GUID;
	efi_guid_t fw_image_type_guid = MT8390_TUNGSTEN_SMARC_FW_IMAGE_GUID;
	efi_guid_t env_image_type_guid = MT8390_TUNGSTEN_SMARC_ENV_IMAGE_GUID;

	guidcpy(&fw_images[0].image_type_id, &image_type_guid);
	guidcpy(&fw_images[1].image_type_id, &uboot_image_type_guid);
	guidcpy(&fw_images[2].image_type_id, &bl2_image_type_guid);
	guidcpy(&fw_images[3].image_type_id, &fw_image_type_guid);
	guidcpy(&fw_images[4].image_type_id, &env_image_type_guid);

	fw_images[0].fw_name = u"GENIO-700-EVK-FIT";
	fw_images[1].fw_name = u"GENIO-700-EVK-FIP";
	fw_images[2].fw_name = u"GENIO-700-EVK-BL2";
	fw_images[3].fw_name = u"GENIO-700-EVK-FW";
	fw_images[4].fw_name = u"GENIO-700-EVK-ENV";
}
#endif /* CONFIG_EFI_HAVE_CAPSULE_SUPPORT && CONFIG_EFI_PARTITION */

#ifdef CONFIG_DM_REGULATOR
struct regulator_value {
	const char *name;
	unsigned val;
};
const struct regulator_value reg_vals[] = {
	{ "buck_vs2", 1600000},
	{ "ldo_vcn33_2_bt", 3300000},
	{ "ldo_vcn18", 1800000},
#if 0
	{ "ldo_vcn33_1_bt", 3300000},
	{ "ldo_vcn33_1_wifi", 3300000},
	{ "ldo_vcn33_2_wifi", 3300000},
#endif
};
void set_regulators(void)
{
	const struct regulator_value* p = reg_vals;
	struct udevice *dev;
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(reg_vals); i++, p++) {
		ret = regulator_get_by_devname(p->name, &dev);
		if (ret) {
			printf("failed to get %s reg %d\n", p->name, ret);
		} else {
			debug("%s: %s %d\n", __func__, p->name, regulator_get_value(dev));
			ret = regulator_set_value(dev, p->val);
			if (ret)
				printf("failed to set %s voltage to %d(%d)\n", p->name, p->val, ret);
			ret = regulator_set_enable(dev, true);
			if (ret)
				printf("failed to enable %s(%d)\n", p->name, ret);
		}
	}
}
#endif

static void clear_dp_hpd(void)
{
	int ret;
	struct gpio_desc desc;

	ret = dm_gpio_lookup_name("46", &desc);
        if (ret) {
		printf("ERROR: Could NOT lookup DP HP_DET\n");
		return;
	}

	ret = dm_gpio_request(&desc, "dp_hp_det");
	if (ret) {
		printf("ERROR: Could NOT request DP HP_DET\n");
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	udelay(500);
	dm_gpio_set_value(&desc, 0);

	ret = dm_gpio_lookup_name("26", &desc);
        if (ret) {
		printf("ERROR: Could NOT lookup eDP HP_DET\n");
		return;
	}

	ret = dm_gpio_request(&desc, "edp_hp_det");
	if (ret) {
		printf("ERROR: Could NOT request eDP HP_DET\n");
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	udelay(500);
	dm_gpio_set_value(&desc, 0);

}

int board_init(void)
{
	struct udevice *dev;
	int ret;

#ifdef CONFIG_DM_REGULATOR
	ret = uclass_get_device_by_driver(UCLASS_MISC,
					  DM_DRIVER_GET(pmic_chip),
					  &dev);
	if (ret) {
		printf("Can't find pmic_chip driver\n");
	}
	ret = uclass_get_device_by_driver(UCLASS_MISC,
					  DM_DRIVER_GET(mt6359_regulator),
					  &dev);
	if (ret) {
		printf("Can't find mt6359_regulator driver\n");
	}

	set_regulators();
#endif

	if (CONFIG_IS_ENABLED(USB_GADGET)) {
		ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 0, &dev);
		if (ret) {
			pr_err("%s: Cannot find USB device\n", __func__);
		}
	}

	if (CONFIG_IS_ENABLED(USB_ETHER))
		usb_ether_init();

	printf("Disabling WDT\n");
	writel(0, 0x10007000);

	printf("Clearing (e)DP HP_DET\n");
	clear_dp_hpd();

	printf("Enabling SCP SRAM\n");
	for (unsigned int val = 0xFFFFFFFF; val != 0U;) {
		val = val >> 1;
		writel(val, 0x1072102C);
	}

#if defined(CONFIG_EFI_HAVE_CAPSULE_SUPPORT) && defined(CONFIG_EFI_PARTITION)
	mediatek_capsule_update_board_setup();
#endif
	return 0;
}

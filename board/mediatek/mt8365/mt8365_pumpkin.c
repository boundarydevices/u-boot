// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 BayLibre SAS
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <common.h>
#include <dm.h>
#include <efi_loader.h>
#include <net.h>
#include <asm/io.h>
#include <linux/kernel.h>

#define MT8365_UPDATABLE_IMAGES	2

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
static struct efi_fw_image fw_images[MT8365_UPDATABLE_IMAGES] = {0};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=bl2.img raw 0x0 0x100 mmcpart 1;"
			"fip.bin part 0 1",
	.images = fw_images,
};

u8 num_image_type_guids = MT8365_UPDATABLE_IMAGES;
#endif

#if defined(CONFIG_EFI_HAVE_CAPSULE_SUPPORT) && defined(CONFIG_EFI_PARTITION)
static bool board_is_genio_350_evk(void)
{
	return CONFIG_IS_ENABLED(TARGET_MT8365) &&
		of_machine_is_compatible("mediatek,genio-350-evk");
}

void mediatek_capsule_update_board_setup(void)
{
	if (board_is_genio_350_evk()) {
		efi_guid_t image_type_guid =
			GENIO_350_EVK_FIT_IMAGE_GUID;
		efi_guid_t uboot_image_type_guid = GENIO_350_EVK_FIP_IMAGE_GUID;

		guidcpy(&fw_images[0].image_type_id, &image_type_guid);
		guidcpy(&fw_images[1].image_type_id, &uboot_image_type_guid);

		fw_images[0].fw_name = u"GENIO-350-EVK-FIT";
		fw_images[1].fw_name = u"GENIO-350-EVK-FIP";
	}
}
#endif /* CONFIG_EFI_HAVE_CAPSULE_SUPPORT && CONFIG_EFI_PARTITION */

int board_init(void)
{
	struct udevice *dev;
	int ret;

	if (CONFIG_IS_ENABLED(USB_GADGET)) {
		ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 0, &dev);
		if (ret) {
			pr_err("%s: Cannot find USB device\n", __func__);
			return ret;
		}
	}

	if (CONFIG_IS_ENABLED(USB_ETHER))
		usb_ether_init();

	return 0;
}

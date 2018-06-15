/*
 * (C) Copyright 2016
 * Xilinx, Inc.
 *
 * (C) Copyright 2016
 * Toradex AG
 *
 * Michal Simek <michal.simek@xilinx.com>
 * Stefan Agner <stefan.agner@toradex.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <spl.h>
#include <libfdt.h>

#ifndef CONFIG_SPL_LOAD_FIT_ADDRESS
# define CONFIG_SPL_LOAD_FIT_ADDRESS	0
#endif

static ulong spl_ram_load_read(struct spl_load_info *load, ulong sector,
			       ulong count, void *buf)
{
	debug("%s: sector %lx, count %lx, buf %lx\n",
	      __func__, sector, count, (ulong)buf);
	memcpy(buf, (void *)(CONFIG_SPL_LOAD_FIT_ADDRESS + sector), count);
	return count;
}

static int spl_ram_load_image(struct spl_image_info *spl_image,
			      struct spl_boot_device *bootdev)
{
	struct image_header *header;

	header = (struct image_header *)CONFIG_SPL_LOAD_FIT_ADDRESS;

#if defined(CONFIG_SPL_DFU_SUPPORT)
	if (bootdev->boot_device == BOOT_DEVICE_DFU)
		spl_dfu_cmd(0, "dfu_alt_info_ram", "ram", "0");
#endif

	if (IS_ENABLED(CONFIG_SPL_LOAD_FIT) &&
	    image_get_magic(header) == FDT_MAGIC) {
		struct spl_load_info load;

		debug("Found FIT\n");
		load.bl_len = 1;
		load.read = spl_ram_load_read;
		spl_load_simple_fit(spl_image, &load, 0, header);
	} else {
		debug("Legacy image\n");
		/*
		 * Get the header.  It will point to an address defined by
		 * handoff which will tell where the image located inside
		 * the flash. For now, it will temporary fixed to address
		 * pointed by U-Boot.
		 */
		header = (struct image_header *)
			(CONFIG_SYS_TEXT_BASE -	sizeof(struct image_header));

		spl_parse_image_header(spl_image, header);
	}

	return 0;
}
#if defined(CONFIG_SPL_RAM_DEVICE)
SPL_LOAD_IMAGE_METHOD("RAM", 0, BOOT_DEVICE_RAM, spl_ram_load_image);
#endif
#if defined(CONFIG_SPL_DFU_SUPPORT)
SPL_LOAD_IMAGE_METHOD("DFU", 0, BOOT_DEVICE_DFU, spl_ram_load_image);
#endif



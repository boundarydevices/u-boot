// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <image.h>
#include <spl.h>

static uintptr_t tee_entry;

void board_fit_image_post_process(const void *fit, int node, void **p_image,
				  size_t *p_size)
{
	uint8_t os;
	int ret;

	ret = fit_image_get_os(fit, node, &os);
	if (ret || (os != IH_OS_TEE))
		return;

	ret = fit_image_get_entry(fit, node, &tee_entry);
	if (ret)
		printf("Cannot get entry from tee found\n");
}

struct bl31_params *bl2_plat_get_bl31_params(uintptr_t bl32_entry,
					     uintptr_t bl33_entry,
					     uintptr_t fdt_addr)
{
	return bl2_plat_get_bl31_params_default(tee_entry, bl33_entry,
						fdt_addr);
}

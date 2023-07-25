// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: Flora Fu, MediaTek
 */

#include <common.h>
#include <dm/device.h>
#include <dm/lists.h>
#include <dm/device_compat.h>
#include <linux/ioport.h>
#include <regmap.h>

#define DEFINE_RES_IRQ(_irq)  { .start = _irq, .flags = IORESOURCE_IRQ}
#define DEFINE_RES_IRQ_NAMED(_irq, _name) { .start = _irq, .name = _name, .flags = IORESOURCE_IRQ}
#define DEFINE_RES_MEM(_base, _size) { .start = _base, .end = _base + _size, .flags = IORESOURCE_MEM}

struct mfd_cell {
	const char *name;
	int	num_resources;
	const struct resource *resources;
	const char *of_compatible;
};

struct pmic_chip {
        struct udevice *dev;
        struct regmap *regmap;
        u16 chip_id;
};

struct chip_data {
	u32 cid_addr;
	u32 cid_shift;
	const struct mfd_cell *cells;
	int cell_size;
};

#include "mt6323.c"
#include "mt6357.c"
#include "mt6358.c"
#include "mt6359.c"
#include "mt6397.c"

static int pmic_chip_probe(struct udevice *dev)
{
	int i;
	int ret;
	unsigned int id = 0;
	struct pmic_chip *pmic = dev_get_priv(dev);
	const struct chip_data *pmic_core;
	const struct mfd_cell *cells;

	pmic->dev = dev;
	debug("%s: %s\n", __func__, dev->name);
	/*
	 * MFD is child device of soc pmic wrapper.
	 * Regmap is set from its parent.
	 */
	pmic->regmap = dev_get_regmap(dev->parent, NULL);
	if (!pmic->regmap)
		return -ENODEV;

	pmic_core = (const struct chip_data *)dev_get_driver_data(dev);
	if (!pmic_core)
		return -ENODEV;

	ret = regmap_read(pmic->regmap, pmic_core->cid_addr, &id);
	if (ret) {
		dev_err(dev, "Failed to read chip id: %d\n", ret);
		return ret;
	}

	pmic->chip_id = (id >> pmic_core->cid_shift) & 0xff;

	cells = pmic_core->cells;
	for (i = 0; i < pmic_core->cell_size; i++, cells++) {
		debug("%s: %s %s\n", __func__, dev->name, cells->name);
		device_bind_driver_to_node(dev, cells->name, cells->name, dev_ofnode(dev), NULL);
	}
	return ret;
}

static const struct udevice_id pmic_chip_of_match[] = {
	PMIC_MT6323_COMPAT
	PMIC_MT6357_COMPAT
	PMIC_MT6358_COMPAT
	PMIC_MT6359_COMPAT
	PMIC_MT6397_COMPAT
	{
		/* sentinel */
	}
};

U_BOOT_DRIVER(pmic_chip) = {
	.name = "pmic_chip",
	.id = UCLASS_MISC,
	.of_match = pmic_chip_of_match,
	.probe = pmic_chip_probe,
	.priv_auto = sizeof(struct pmic_chip),
};


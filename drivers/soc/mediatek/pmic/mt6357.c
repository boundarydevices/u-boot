#ifdef CONFIG_MT6357
#include <linux/mfd/mt6357/core.h>
#include <linux/mfd/mt6357/registers.h>

#define MT6357_RTC_BASE		0x0588
#define MT6357_RTC_SIZE		0x3c

static const struct resource mt6357_rtc_resources[] = {
	DEFINE_RES_MEM(MT6357_RTC_BASE, MT6357_RTC_SIZE),
	DEFINE_RES_IRQ(MT6357_IRQ_RTC),
};

static const struct resource mt6357_keys_resources[] = {
	DEFINE_RES_IRQ_NAMED(MT6357_IRQ_PWRKEY, "powerkey"),
	DEFINE_RES_IRQ_NAMED(MT6357_IRQ_HOMEKEY, "homekey"),
	DEFINE_RES_IRQ_NAMED(MT6357_IRQ_PWRKEY_R, "powerkey_r"),
	DEFINE_RES_IRQ_NAMED(MT6357_IRQ_HOMEKEY_R, "homekey_r"),
};

static const struct mfd_cell mt6357_devs[] = {
	{
		.name = "mt6357-regulator",
	}, {
		.name = "mt6357-rtc",
		.num_resources = ARRAY_SIZE(mt6357_rtc_resources),
		.resources = mt6357_rtc_resources,
		.of_compatible = "mediatek,mt6357-rtc",
	}, {
		.name = "mt6357-sound",
		.of_compatible = "mediatek,mt6357-sound"
	}, {
		.name = "mtk-pmic-keys",
		.num_resources = ARRAY_SIZE(mt6357_keys_resources),
		.resources = mt6357_keys_resources,
		.of_compatible = "mediatek,mt6357-keys"
	},
};

static const struct chip_data mt6357_core = {
	.cid_addr = MT6357_SWCID,
	.cid_shift = 8,
	.cells = mt6357_devs,
	.cell_size = ARRAY_SIZE(mt6357_devs),
};

#define PMIC_MT6357_COMPAT	{ .compatible = "mediatek,mt6357", .data = (ulong)&mt6357_core, },
#else
#define PMIC_MT6357_COMPAT
#endif

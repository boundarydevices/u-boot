#ifdef CONFIG_MT6359
#include <linux/mfd/mt6359/core.h>
#include <linux/mfd/mt6359/registers.h>

#define MT6359_RTC_BASE		0x0588
#define MT6359_RTC_SIZE		0x3c

static const struct resource mt6359_rtc_resources[] = {
	DEFINE_RES_MEM(MT6359_RTC_BASE, MT6359_RTC_SIZE),
	DEFINE_RES_IRQ(MT6359_IRQ_RTC),
};

static const struct resource mt6359_keys_resources[] = {
	DEFINE_RES_IRQ_NAMED(MT6359_IRQ_PWRKEY, "powerkey"),
	DEFINE_RES_IRQ_NAMED(MT6359_IRQ_HOMEKEY, "homekey"),
	DEFINE_RES_IRQ_NAMED(MT6359_IRQ_PWRKEY_R, "powerkey_r"),
	DEFINE_RES_IRQ_NAMED(MT6359_IRQ_HOMEKEY_R, "homekey_r"),
};

static const struct mfd_cell mt6359_devs[] = {
	{ .name = "mt6359-regulator", },
	{
		.name = "mt6359-rtc",
		.num_resources = ARRAY_SIZE(mt6359_rtc_resources),
		.resources = mt6359_rtc_resources,
		.of_compatible = "mediatek,mt6358-rtc",
	},
	{ .name = "mt6359-sound", },
	{
		.name = "mtk-pmic-keys",
		.num_resources = ARRAY_SIZE(mt6359_keys_resources),
		.resources = mt6359_keys_resources,
		.of_compatible = "mediatek,mt6359-keys"
	},
};

static const struct chip_data mt6359_core = {
	.cid_addr = MT6359_SWCID,
	.cid_shift = 8,
	.cells = mt6359_devs,
	.cell_size = ARRAY_SIZE(mt6359_devs),
};

#define PMIC_MT6359_COMPAT	{ .compatible = "mediatek,mt6359", .data = (ulong)&mt6359_core, },
#else
#define PMIC_MT6359_COMPAT
#endif

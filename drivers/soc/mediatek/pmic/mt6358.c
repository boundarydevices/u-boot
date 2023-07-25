#ifdef CONFIG_MT6358
#include <linux/mfd/mt6358/core.h>
#include <linux/mfd/mt6358/registers.h>

#define MT6358_RTC_BASE		0x0588
#define MT6358_RTC_SIZE		0x3c

static const struct resource mt6358_rtc_resources[] = {
	DEFINE_RES_MEM(MT6358_RTC_BASE, MT6358_RTC_SIZE),
	DEFINE_RES_IRQ(MT6358_IRQ_RTC),
};

static const struct resource mt6358_keys_resources[] = {
	DEFINE_RES_IRQ_NAMED(MT6358_IRQ_PWRKEY, "powerkey"),
	DEFINE_RES_IRQ_NAMED(MT6358_IRQ_HOMEKEY, "homekey"),
	DEFINE_RES_IRQ_NAMED(MT6358_IRQ_PWRKEY_R, "powerkey_r"),
	DEFINE_RES_IRQ_NAMED(MT6358_IRQ_HOMEKEY_R, "homekey_r"),
};

static const struct mfd_cell mt6358_devs[] = {
	{
		.name = "mt6358-regulator",
		.of_compatible = "mediatek,mt6358-regulator"
	}, {
		.name = "mt6358-rtc",
		.num_resources = ARRAY_SIZE(mt6358_rtc_resources),
		.resources = mt6358_rtc_resources,
		.of_compatible = "mediatek,mt6358-rtc",
	}, {
		.name = "mt6358-sound",
		.of_compatible = "mediatek,mt6358-sound"
	}, {
		.name = "mt6358-keys",
		.num_resources = ARRAY_SIZE(mt6358_keys_resources),
		.resources = mt6358_keys_resources,
		.of_compatible = "mediatek,mt6358-keys"
	},
};

static const struct chip_data mt6358_core = {
	.cid_addr = MT6358_SWCID,
	.cid_shift = 8,
	.cells = mt6358_devs,
	.cell_size = ARRAY_SIZE(mt6358_devs),
};

#define PMIC_MT6358_COMPAT	{ .compatible = "mediatek,mt6358", .data = (ulong)&mt6358_core, },
#else
#define PMIC_MT6358_COMPAT
#endif

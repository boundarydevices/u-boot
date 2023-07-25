#ifdef CONFIG_MT6351
static const u32 mt6351_regs[] = {
	[PWRAP_DEW_DIO_EN] =		0x02F2,
	[PWRAP_DEW_READ_TEST] =		0x02F4,
	[PWRAP_DEW_WRITE_TEST] =	0x02F6,
	[PWRAP_DEW_CRC_EN] =		0x02FA,
	[PWRAP_DEW_CRC_VAL] =		0x02FC,
	[PWRAP_DEW_CIPHER_KEY_SEL] =	0x0300,
	[PWRAP_DEW_CIPHER_IV_SEL] =	0x0302,
	[PWRAP_DEW_CIPHER_EN] =		0x0304,
	[PWRAP_DEW_CIPHER_RDY] =	0x0306,
	[PWRAP_DEW_CIPHER_MODE] =	0x0308,
	[PWRAP_DEW_CIPHER_SWRST] =	0x030A,
	[PWRAP_DEW_RDDMY_NO] =		0x030C,
};

#define NEED_PWRAP_REGMAP16
static int pwrap_read16(struct pmic_wrapper *wrp, u32 adr, u32 *rdata);
static int pwrap_write16(struct pmic_wrapper *wrp, u32 adr, u32 wdata);
static const struct regmap_config pwrap_regmap_config16;

static const struct pwrap_slv_type pmic_mt6351 = {
	.dew_regs = mt6351_regs,
	.type = PMIC_MT6351,
	.regmap = &pwrap_regmap_config16,
	.caps = 0,
	.pwrap_read = pwrap_read16,
	.pwrap_write = pwrap_write16,
};

#define MT6351_COMPAT	{ .compatible = "mediatek,mt6351", .data = (ulong)&pmic_mt6351, },
#else
#define MT6351_COMPAT
#endif

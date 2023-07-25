#ifdef CONFIG_MT6397
static const u32 mt6397_regs[] = {
	[PWRAP_DEW_BASE] =		0xbc00,
	[PWRAP_DEW_EVENT_OUT_EN] =	0xbc00,
	[PWRAP_DEW_DIO_EN] =		0xbc02,
	[PWRAP_DEW_EVENT_SRC_EN] =	0xbc04,
	[PWRAP_DEW_EVENT_SRC] =		0xbc06,
	[PWRAP_DEW_EVENT_FLAG] =	0xbc08,
	[PWRAP_DEW_READ_TEST] =		0xbc0a,
	[PWRAP_DEW_WRITE_TEST] =	0xbc0c,
	[PWRAP_DEW_CRC_EN] =		0xbc0e,
	[PWRAP_DEW_CRC_VAL] =		0xbc10,
	[PWRAP_DEW_MON_GRP_SEL] =	0xbc12,
	[PWRAP_DEW_MON_FLAG_SEL] =	0xbc14,
	[PWRAP_DEW_EVENT_TEST] =	0xbc16,
	[PWRAP_DEW_CIPHER_KEY_SEL] =	0xbc18,
	[PWRAP_DEW_CIPHER_IV_SEL] =	0xbc1a,
	[PWRAP_DEW_CIPHER_LOAD] =	0xbc1c,
	[PWRAP_DEW_CIPHER_START] =	0xbc1e,
	[PWRAP_DEW_CIPHER_RDY] =	0xbc20,
	[PWRAP_DEW_CIPHER_MODE] =	0xbc22,
	[PWRAP_DEW_CIPHER_SWRST] =	0xbc24,
};

#define NEED_PWRAP_REGMAP16
static int pwrap_read16(struct pmic_wrapper *wrp, u32 adr, u32 *rdata);
static int pwrap_write16(struct pmic_wrapper *wrp, u32 adr, u32 wdata);
static const struct regmap_config pwrap_regmap_config16;

static const struct pwrap_slv_type pmic_mt6397 = {
	.dew_regs = mt6397_regs,
	.type = PMIC_MT6397,
	.regmap = &pwrap_regmap_config16,
	.caps = PWRAP_SLV_CAP_SPI | PWRAP_SLV_CAP_DUALIO |
		PWRAP_SLV_CAP_SECURITY,
	.pwrap_read = pwrap_read16,
	.pwrap_write = pwrap_write16,
};

#define MT6397_COMPAT	{ .compatible = "mediatek,mt6397", .data = (ulong)&pmic_mt6397, },
#else
#define MT6397_COMPAT
#endif

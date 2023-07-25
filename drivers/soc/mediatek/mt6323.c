#ifdef CONFIG_MT6323
static const u32 mt6323_regs[] = {
	[PWRAP_DEW_BASE] =		0x0000,
	[PWRAP_DEW_DIO_EN] =		0x018a,
	[PWRAP_DEW_READ_TEST] =		0x018c,
	[PWRAP_DEW_WRITE_TEST] =	0x018e,
	[PWRAP_DEW_CRC_EN] =		0x0192,
	[PWRAP_DEW_CRC_VAL] =		0x0194,
	[PWRAP_DEW_MON_GRP_SEL] =	0x0196,
	[PWRAP_DEW_CIPHER_KEY_SEL] =	0x0198,
	[PWRAP_DEW_CIPHER_IV_SEL] =	0x019a,
	[PWRAP_DEW_CIPHER_EN] =		0x019c,
	[PWRAP_DEW_CIPHER_RDY] =	0x019e,
	[PWRAP_DEW_CIPHER_MODE] =	0x01a0,
	[PWRAP_DEW_CIPHER_SWRST] =	0x01a2,
	[PWRAP_DEW_RDDMY_NO] =		0x01a4,
};

#define NEED_PWRAP_REGMAP16
static int pwrap_read16(struct pmic_wrapper *wrp, u32 adr, u32 *rdata);
static int pwrap_write16(struct pmic_wrapper *wrp, u32 adr, u32 wdata);
static const struct regmap_config pwrap_regmap_config16;

static const struct pwrap_slv_type pmic_mt6323 = {
	.dew_regs = mt6323_regs,
	.type = PMIC_MT6323,
	.regmap = &pwrap_regmap_config16,
	.caps = PWRAP_SLV_CAP_SPI | PWRAP_SLV_CAP_DUALIO |
		PWRAP_SLV_CAP_SECURITY,
	.pwrap_read = pwrap_read16,
	.pwrap_write = pwrap_write16,
};

#define MT6323_COMPAT	{ .compatible = "mediatek,mt6323", .data = (ulong)&pmic_mt6323, },
#else
#define MT6323_COMPAT
#endif

#ifdef CONFIG_MT8183
static int mt8183_regs[] = {
	[PWRAP_MUX_SEL] =			0x0,
	[PWRAP_WRAP_EN] =			0x4,
	[PWRAP_DIO_EN] =			0x8,
	[PWRAP_SI_SAMPLE_CTRL] =		0xC,
	[PWRAP_RDDMY] =				0x14,
	[PWRAP_CSHEXT_WRITE] =			0x18,
	[PWRAP_CSHEXT_READ] =			0x1C,
	[PWRAP_CSLEXT_WRITE] =			0x20,
	[PWRAP_CSLEXT_READ] =			0x24,
	[PWRAP_EXT_CK_WRITE] =			0x28,
	[PWRAP_STAUPD_CTRL] =			0x30,
	[PWRAP_STAUPD_GRPEN] =			0x34,
	[PWRAP_EINT_STA0_ADR] =			0x38,
	[PWRAP_HARB_HPRIO] =			0x5C,
	[PWRAP_HIPRIO_ARB_EN] =			0x60,
	[PWRAP_MAN_EN] =			0x70,
	[PWRAP_MAN_CMD] =			0x74,
	[PWRAP_WACS0_EN] =			0x80,
	[PWRAP_INIT_DONE0] =			0x84,
	[PWRAP_WACS1_EN] =			0x88,
	[PWRAP_INIT_DONE1] =			0x8C,
	[PWRAP_WACS2_EN] =			0x90,
	[PWRAP_INIT_DONE2] =			0x94,
	[PWRAP_WACS_P2P_EN] =			0xA0,
	[PWRAP_INIT_DONE_P2P] =			0xA4,
	[PWRAP_WACS_MD32_EN] =			0xA8,
	[PWRAP_INIT_DONE_MD32] =		0xAC,
	[PWRAP_INT_EN] =			0xB0,
	[PWRAP_INT_FLG] =			0xB8,
	[PWRAP_INT_CLR] =			0xBC,
	[PWRAP_INT1_EN] =			0xC0,
	[PWRAP_INT1_FLG] =			0xC8,
	[PWRAP_INT1_CLR] =			0xCC,
	[PWRAP_SIG_ADR] =			0xD0,
	[PWRAP_CRC_EN] =			0xE0,
	[PWRAP_TIMER_EN] =			0xE4,
	[PWRAP_WDT_UNIT] =			0xEC,
	[PWRAP_WDT_SRC_EN] =			0xF0,
	[PWRAP_WDT_SRC_EN_1] =			0xF4,
	[PWRAP_INT_GPS_AUXADC_CMD_ADDR] =	0x1DC,
	[PWRAP_INT_GPS_AUXADC_CMD] =		0x1E0,
	[PWRAP_INT_GPS_AUXADC_RDATA_ADDR] =	0x1E4,
	[PWRAP_EXT_GPS_AUXADC_RDATA_ADDR] =	0x1E8,
	[PWRAP_GPSINF_0_STA] =			0x1EC,
	[PWRAP_GPSINF_1_STA] =			0x1F0,
	[PWRAP_WACS2_CMD] =			0xC20,
	[PWRAP_WACS2_RDATA] =			0xC24,
	[PWRAP_WACS2_VLDCLR] =			0xC28,
};

static int pwrap_mt8183_init_soc_specific(struct pmic_wrapper *wrp)
{
	pwrap_writel(wrp, 0xf5, PWRAP_STAUPD_GRPEN);

	pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_CRC_EN], 0x1);
	pwrap_writel(wrp, 1, PWRAP_CRC_EN);
	pwrap_writel(wrp, 0x416, PWRAP_SIG_ADR);
	pwrap_writel(wrp, 0x42e, PWRAP_EINT_STA0_ADR);

	pwrap_writel(wrp, 1, PWRAP_WACS_P2P_EN);
	pwrap_writel(wrp, 1, PWRAP_WACS_MD32_EN);
	pwrap_writel(wrp, 1, PWRAP_INIT_DONE_P2P);
	pwrap_writel(wrp, 1, PWRAP_INIT_DONE_MD32);

	return 0;
}

static const struct pmic_wrapper_type pwrap_mt8183 = {
	.regs = mt8183_regs,
	.type = PWRAP_MT8183,
	.arb_en_all = 0x3fa75,
	.int_en_all = 0xffffffff,
	.int1_en_all = 0xeef7ffff,
	.spi_w = PWRAP_MAN_CMD_SPI_WRITE,
	.wdt_src = PWRAP_WDT_SRC_MASK_ALL,
	.caps = PWRAP_CAP_INT1_EN | PWRAP_CAP_WDT_SRC1,
	.init_reg_clock = pwrap_common_init_reg_clock,
	.init_soc_specific = pwrap_mt8183_init_soc_specific,
};

#define WRAP_MT8183_COMPAT	{ .compatible = "mediatek,mt8183-pwrap", .data = (ulong)&pwrap_mt8183, },
#else
#define WRAP_MT8183_COMPAT
#endif

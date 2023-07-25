#ifdef CONFIG_MT6765
static int mt6765_regs[] = {
	[PWRAP_MUX_SEL] =		0x0,
	[PWRAP_WRAP_EN] =		0x4,
	[PWRAP_DIO_EN] =		0x8,
	[PWRAP_RDDMY] =			0x20,
	[PWRAP_CSHEXT_WRITE] =		0x24,
	[PWRAP_CSHEXT_READ] =		0x28,
	[PWRAP_CSLEXT_START] =		0x2C,
	[PWRAP_CSLEXT_END] =		0x30,
	[PWRAP_STAUPD_PRD] =		0x3C,
	[PWRAP_HARB_HPRIO] =		0x68,
	[PWRAP_HIPRIO_ARB_EN] =		0x6C,
	[PWRAP_MAN_EN] =		0x7C,
	[PWRAP_MAN_CMD] =		0x80,
	[PWRAP_WACS0_EN] =		0x8C,
	[PWRAP_WACS1_EN] =		0x94,
	[PWRAP_WACS2_EN] =		0x9C,
	[PWRAP_INIT_DONE2] =		0xA0,
	[PWRAP_WACS2_CMD] =		0xC20,
	[PWRAP_WACS2_RDATA] =		0xC24,
	[PWRAP_WACS2_VLDCLR] =		0xC28,
	[PWRAP_INT_EN] =		0xB4,
	[PWRAP_INT_FLG_RAW] =		0xB8,
	[PWRAP_INT_FLG] =		0xBC,
	[PWRAP_INT_CLR] =		0xC0,
	[PWRAP_TIMER_EN] =		0xE8,
	[PWRAP_WDT_UNIT] =		0xF0,
	[PWRAP_WDT_SRC_EN] =		0xF4,
	[PWRAP_DCM_EN] =		0x1DC,
	[PWRAP_DCM_DBC_PRD] =		0x1E0,
};

static const struct pmic_wrapper_type pwrap_mt6765 = {
	.regs = mt6765_regs,
	.type = PWRAP_MT6765,
	.arb_en_all = 0x3fd35,
	.int_en_all = 0xffffffff,
	.spi_w = PWRAP_MAN_CMD_SPI_WRITE,
	.wdt_src = PWRAP_WDT_SRC_MASK_ALL,
	.caps = PWRAP_CAP_RESET | PWRAP_CAP_DCM,
	.init_reg_clock = pwrap_common_init_reg_clock,
	.init_soc_specific = NULL,
};

#define WRAP_MT6765_COMPAT	{ .compatible = "mediatek,mt6765-pwrap", .data = (ulong)&pwrap_mt6765, },
#else
#define WRAP_MT6765_COMPAT
#endif

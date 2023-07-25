#ifdef CONFIG_MT6797
static int mt6797_regs[] = {
	[PWRAP_MUX_SEL] =		0x0,
	[PWRAP_WRAP_EN] =		0x4,
	[PWRAP_DIO_EN] =		0x8,
	[PWRAP_SIDLY] =			0xC,
	[PWRAP_RDDMY] =			0x10,
	[PWRAP_CSHEXT_WRITE] =		0x18,
	[PWRAP_CSHEXT_READ] =		0x1C,
	[PWRAP_CSLEXT_START] =		0x20,
	[PWRAP_CSLEXT_END] =		0x24,
	[PWRAP_STAUPD_PRD] =		0x28,
	[PWRAP_HARB_HPRIO] =		0x50,
	[PWRAP_HIPRIO_ARB_EN] =		0x54,
	[PWRAP_MAN_EN] =		0x60,
	[PWRAP_MAN_CMD] =		0x64,
	[PWRAP_WACS0_EN] =		0x70,
	[PWRAP_WACS1_EN] =		0x84,
	[PWRAP_WACS2_EN] =		0x98,
	[PWRAP_INIT_DONE2] =		0x9C,
	[PWRAP_WACS2_CMD] =		0xA0,
	[PWRAP_WACS2_RDATA] =		0xA4,
	[PWRAP_WACS2_VLDCLR] =		0xA8,
	[PWRAP_INT_EN] =		0xC0,
	[PWRAP_INT_FLG_RAW] =		0xC4,
	[PWRAP_INT_FLG] =		0xC8,
	[PWRAP_INT_CLR] =		0xCC,
	[PWRAP_TIMER_EN] =		0xF4,
	[PWRAP_WDT_UNIT] =		0xFC,
	[PWRAP_WDT_SRC_EN] =		0x100,
	[PWRAP_DCM_EN] =		0x1CC,
	[PWRAP_DCM_DBC_PRD] =		0x1D4,
};

static const struct pmic_wrapper_type pwrap_mt6797 = {
	.regs = mt6797_regs,
	.type = PWRAP_MT6797,
	.arb_en_all = 0x01fff,
	.int_en_all = 0xffffffc6,
	.int1_en_all = 0,
	.spi_w = PWRAP_MAN_CMD_SPI_WRITE,
	.wdt_src = PWRAP_WDT_SRC_MASK_ALL,
	.caps = PWRAP_CAP_RESET | PWRAP_CAP_DCM,
	.init_reg_clock = pwrap_common_init_reg_clock,
	.init_soc_specific = NULL,
};

#define WRAP_MT6797_COMPAT	{ .compatible = "mediatek,mt6797-pwrap", .data = (ulong)&pwrap_mt6797, },
#else
#define WRAP_MT6797_COMPAT
#endif

#ifdef CONFIG_MT2701
const static int mt2701_regs[] = {
	[PWRAP_MUX_SEL] =		0x0,
	[PWRAP_WRAP_EN] =		0x4,
	[PWRAP_DIO_EN] =		0x8,
	[PWRAP_SIDLY] =			0xc,
	[PWRAP_RDDMY] =			0x18,
	[PWRAP_SI_CK_CON] =		0x1c,
	[PWRAP_CSHEXT_WRITE] =		0x20,
	[PWRAP_CSHEXT_READ] =		0x24,
	[PWRAP_CSLEXT_START] =		0x28,
	[PWRAP_CSLEXT_END] =		0x2c,
	[PWRAP_STAUPD_PRD] =		0x30,
	[PWRAP_STAUPD_GRPEN] =		0x34,
	[PWRAP_STAUPD_MAN_TRIG] =	0x38,
	[PWRAP_STAUPD_STA] =		0x3c,
	[PWRAP_WRAP_STA] =		0x44,
	[PWRAP_HARB_INIT] =		0x48,
	[PWRAP_HARB_HPRIO] =		0x4c,
	[PWRAP_HIPRIO_ARB_EN] =		0x50,
	[PWRAP_HARB_STA0] =		0x54,
	[PWRAP_HARB_STA1] =		0x58,
	[PWRAP_MAN_EN] =		0x5c,
	[PWRAP_MAN_CMD] =		0x60,
	[PWRAP_MAN_RDATA] =		0x64,
	[PWRAP_MAN_VLDCLR] =		0x68,
	[PWRAP_WACS0_EN] =		0x6c,
	[PWRAP_INIT_DONE0] =		0x70,
	[PWRAP_WACS0_CMD] =		0x74,
	[PWRAP_WACS0_RDATA] =		0x78,
	[PWRAP_WACS0_VLDCLR] =		0x7c,
	[PWRAP_WACS1_EN] =		0x80,
	[PWRAP_INIT_DONE1] =		0x84,
	[PWRAP_WACS1_CMD] =		0x88,
	[PWRAP_WACS1_RDATA] =		0x8c,
	[PWRAP_WACS1_VLDCLR] =		0x90,
	[PWRAP_WACS2_EN] =		0x94,
	[PWRAP_INIT_DONE2] =		0x98,
	[PWRAP_WACS2_CMD] =		0x9c,
	[PWRAP_WACS2_RDATA] =		0xa0,
	[PWRAP_WACS2_VLDCLR] =		0xa4,
	[PWRAP_INT_EN] =		0xa8,
	[PWRAP_INT_FLG_RAW] =		0xac,
	[PWRAP_INT_FLG] =		0xb0,
	[PWRAP_INT_CLR] =		0xb4,
	[PWRAP_SIG_ADR] =		0xb8,
	[PWRAP_SIG_MODE] =		0xbc,
	[PWRAP_SIG_VALUE] =		0xc0,
	[PWRAP_SIG_ERRVAL] =		0xc4,
	[PWRAP_CRC_EN] =		0xc8,
	[PWRAP_TIMER_EN] =		0xcc,
	[PWRAP_TIMER_STA] =		0xd0,
	[PWRAP_WDT_UNIT] =		0xd4,
	[PWRAP_WDT_SRC_EN] =		0xd8,
	[PWRAP_WDT_FLG] =		0xdc,
	[PWRAP_DEBUG_INT_SEL] =		0xe0,
	[PWRAP_DVFS_ADR0] =		0xe4,
	[PWRAP_DVFS_WDATA0] =		0xe8,
	[PWRAP_DVFS_ADR1] =		0xec,
	[PWRAP_DVFS_WDATA1] =		0xf0,
	[PWRAP_DVFS_ADR2] =		0xf4,
	[PWRAP_DVFS_WDATA2] =		0xf8,
	[PWRAP_DVFS_ADR3] =		0xfc,
	[PWRAP_DVFS_WDATA3] =		0x100,
	[PWRAP_DVFS_ADR4] =		0x104,
	[PWRAP_DVFS_WDATA4] =		0x108,
	[PWRAP_DVFS_ADR5] =		0x10c,
	[PWRAP_DVFS_WDATA5] =		0x110,
	[PWRAP_DVFS_ADR6] =		0x114,
	[PWRAP_DVFS_WDATA6] =		0x118,
	[PWRAP_DVFS_ADR7] =		0x11c,
	[PWRAP_DVFS_WDATA7] =		0x120,
	[PWRAP_CIPHER_KEY_SEL] =	0x124,
	[PWRAP_CIPHER_IV_SEL] =		0x128,
	[PWRAP_CIPHER_EN] =		0x12c,
	[PWRAP_CIPHER_RDY] =		0x130,
	[PWRAP_CIPHER_MODE] =		0x134,
	[PWRAP_CIPHER_SWRST] =		0x138,
	[PWRAP_DCM_EN] =		0x13c,
	[PWRAP_DCM_DBC_PRD] =		0x140,
	[PWRAP_ADC_CMD_ADDR] =		0x144,
	[PWRAP_PWRAP_ADC_CMD] =		0x148,
	[PWRAP_ADC_RDY_ADDR] =		0x14c,
	[PWRAP_ADC_RDATA_ADDR1] =	0x150,
	[PWRAP_ADC_RDATA_ADDR2] =	0x154,
};

static int pwrap_mt2701_init_soc_specific(struct pmic_wrapper *wrp)
{
	/* GPS_INTF initialization */
	switch (wrp->slave->type) {
	case PMIC_MT6323:
		pwrap_writel(wrp, 0x076c, PWRAP_ADC_CMD_ADDR);
		pwrap_writel(wrp, 0x8000, PWRAP_PWRAP_ADC_CMD);
		pwrap_writel(wrp, 0x072c, PWRAP_ADC_RDY_ADDR);
		pwrap_writel(wrp, 0x072e, PWRAP_ADC_RDATA_ADDR1);
		pwrap_writel(wrp, 0x0730, PWRAP_ADC_RDATA_ADDR2);
		break;
	default:
		break;
	}

	return 0;
}

static int pwrap_mt2701_init_reg_clock(struct pmic_wrapper *wrp)
{
	switch (wrp->slave->type) {
	case PMIC_MT6397:
		pwrap_writel(wrp, 0xc, PWRAP_RDDMY);
		pwrap_init_chip_select_ext(wrp, 4, 0, 2, 2);
		break;

	case PMIC_MT6323:
		pwrap_writel(wrp, 0x8, PWRAP_RDDMY);
		pwrap_write(wrp, wrp->slave->dew_regs[PWRAP_DEW_RDDMY_NO],
			    0x8);
		pwrap_init_chip_select_ext(wrp, 5, 0, 2, 2);
		break;
	default:
		break;
	}

	return 0;
}

static const struct pmic_wrapper_type pwrap_mt2701 = {
	.regs = mt2701_regs,
	.type = PWRAP_MT2701,
	.arb_en_all = 0x3f,
	.int_en_all = ~(u32)(BIT(31) | BIT(2)),
	.int1_en_all = 0,
	.spi_w = PWRAP_MAN_CMD_SPI_WRITE_NEW,
	.wdt_src = PWRAP_WDT_SRC_MASK_ALL,
	.caps = PWRAP_CAP_RESET | PWRAP_CAP_DCM,
	.init_reg_clock = pwrap_mt2701_init_reg_clock,
	.init_soc_specific = pwrap_mt2701_init_soc_specific,
};

#define WRAP_MT2701_COMPAT	{ .compatible = "mediatek,mt2701-pwrap", .data = (ulong)&pwrap_mt2701, },
#else
#define WRAP_MT2701_COMPAT
#endif

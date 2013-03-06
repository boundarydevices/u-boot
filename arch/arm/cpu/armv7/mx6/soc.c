/*
 * (C) Copyright 2007
 * Sascha Hauer, Pengutronix
 *
 * (C) Copyright 2009 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/boot_mode.h>
#include <div64.h>
#include <stdbool.h>

#define DEFAULT_RAW_25C		1440
#define DEFAULT_RAW_HOT		1240
#define DEFAULT_TEMP_HOT	125

struct scu_regs {
	u32	ctrl;
	u32	config;
	u32	status;
	u32	invalidate;
	u32	fpga_rev;
};

u32 get_cpu_rev(void)
{
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	u32 reg = readl(&anatop->digprog_sololite);
	u32 type = ((reg >> 16) & 0xff);

	if (type != MXC_CPU_MX6SL) {
		reg = readl(&anatop->digprog);
		type = ((reg >> 16) & 0xff);
		if (type == MXC_CPU_MX6DL) {
			struct scu_regs *scu = (struct scu_regs *)SCU_BASE_ADDR;
			u32 cfg = readl(&scu->config) & 3;

			if (!cfg)
				type = MXC_CPU_MX6SOLO;
		}
	}
	reg &= 0xff;		/* mx6 silicon revision */
	return (type << 12) | (reg + 0x10);
}

void init_aips(void)
{
	struct aipstz_regs *aips1, *aips2;

	aips1 = (struct aipstz_regs *)AIPS1_BASE_ADDR;
	aips2 = (struct aipstz_regs *)AIPS2_BASE_ADDR;

	/*
	 * Set all MPROTx to be non-bufferable, trusted for R/W,
	 * not forced to user-mode.
	 */
	writel(0x77777777, &aips1->mprot0);
	writel(0x77777777, &aips1->mprot1);
	writel(0x77777777, &aips2->mprot0);
	writel(0x77777777, &aips2->mprot1);

	/*
	 * Set all OPACRx to be non-bufferable, not require
	 * supervisor privilege level for access,allow for
	 * write access and untrusted master access.
	 */
	writel(0x00000000, &aips1->opacr0);
	writel(0x00000000, &aips1->opacr1);
	writel(0x00000000, &aips1->opacr2);
	writel(0x00000000, &aips1->opacr3);
	writel(0x00000000, &aips1->opacr4);
	writel(0x00000000, &aips2->opacr0);
	writel(0x00000000, &aips2->opacr1);
	writel(0x00000000, &aips2->opacr2);
	writel(0x00000000, &aips2->opacr3);
	writel(0x00000000, &aips2->opacr4);
}

/*
 * Set the VDDSOC
 *
 * Mask out the REG_CORE[22:18] bits (REG2_TRIG) and set
 * them to the specified millivolt level.
 * Possible values are from 0.725V to 1.450V in steps of
 * 0.025V (25mV).
 */
void set_vddsoc(u32 mv)
{
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	u32 val, reg = readl(&anatop->reg_core);

	if (mv < 725)
		val = 0x00;	/* Power gated off */
	else if (mv > 1450)
		val = 0x1F;	/* Power FET switched full on. No regulation */
	else
		val = (mv - 700) / 25;

	/*
	 * Mask out the REG_CORE[22:18] bits (REG2_TRIG)
	 * and set them to the calculated value (0.7V + val * 0.25V)
	 */
	reg = (reg & ~(0x1F << 18)) | (val << 18);
	writel(reg, &anatop->reg_core);
}

static void imx_set_wdog_powerdown(bool enable)
{
	struct wdog_regs *wdog1 = (struct wdog_regs *)WDOG1_BASE_ADDR;
	struct wdog_regs *wdog2 = (struct wdog_regs *)WDOG2_BASE_ADDR;

	/* Write to the PDE (Power Down Enable) bit */
	writew(enable, &wdog1->wmcr);
	writew(enable, &wdog2->wmcr);
}

int arch_cpu_init(void)
{
	init_aips();

	set_vddsoc(1200);	/* Set VDDSOC to 1.2V */

	imx_set_wdog_powerdown(false); /* Disable PDE bit of WMCR register */
	return 0;
}

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif

#if defined(CONFIG_FEC_MXC)
void imx_get_mac_from_fuse(int dev_id, unsigned char *mac)
{
	struct iim_regs *iim = (struct iim_regs *)IMX_IIM_BASE;
	struct fuse_bank *bank = &iim->bank[4];
	struct fuse_bank4_regs *fuse =
			(struct fuse_bank4_regs *)bank->fuse_regs;

	u32 value = readl(&fuse->mac_addr_high);
	mac[0] = (value >> 8);
	mac[1] = value ;

	value = readl(&fuse->mac_addr_low);
	mac[2] = value >> 24 ;
	mac[3] = value >> 16 ;
	mac[4] = value >> 8 ;
	mac[5] = value ;

}
#endif

void boot_mode_apply(unsigned cfg_val)
{
	unsigned reg;
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	writel(cfg_val, &psrc->gpr9);
	reg = readl(&psrc->gpr10);
	if (cfg_val)
		reg |= 1 << 28;
	else
		reg &= ~(1 << 28);
	writel(reg, &psrc->gpr10);
}
/*
 * cfg_val will be used for
 * Boot_cfg4[7:0]:Boot_cfg3[7:0]:Boot_cfg2[7:0]:Boot_cfg1[7:0]
 * After reset, if GPR10[28] is 1, ROM will copy GPR9[25:0]
 * to SBMR1, which will determine the boot device.
 */
const struct boot_mode soc_boot_modes[] = {
	{"normal",	MAKE_CFGVAL(0x00, 0x00, 0x00, 0x00)},
	/* reserved value should start rom usb */
	{"usb",		MAKE_CFGVAL(0x01, 0x00, 0x00, 0x00)},
	{"sata",	MAKE_CFGVAL(0x20, 0x00, 0x00, 0x00)},
	{"escpi1:0",	MAKE_CFGVAL(0x30, 0x00, 0x00, 0x08)},
	{"escpi1:1",	MAKE_CFGVAL(0x30, 0x00, 0x00, 0x18)},
	{"escpi1:2",	MAKE_CFGVAL(0x30, 0x00, 0x00, 0x28)},
	{"escpi1:3",	MAKE_CFGVAL(0x30, 0x00, 0x00, 0x38)},
	/* 4 bit bus width */
	{"esdhc1",	MAKE_CFGVAL(0x40, 0x20, 0x00, 0x00)},
	{"esdhc2",	MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"esdhc3",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"esdhc4",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};

int cvt_raw_to_celius(unsigned raw, unsigned raw_25c,
		unsigned long long cvt_to_celsius, unsigned *hundredths)
{
	int change = (raw_25c - raw);
	long long scale = (change * cvt_to_celsius);

	change =  scale >> 32;
	scale = (scale & 0xffffffff) * 100;
	*hundredths = scale >> 32;
	return 25 + change;
}

static int get_temperature_fuse(void)
{
	u32 ccm_ccgr2;
	u32 fuse;
	struct mxc_ccm_reg *imx_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct ocotp_regs *ocotp = (struct ocotp_regs *)OCOTP_BASE_ADDR;

	ccm_ccgr2 = readl(&imx_ccm->CCGR2);
	writel(ccm_ccgr2 | (0x3 << MXC_CCM_CCGR2_OCOTP_CTRL_OFFSET), &imx_ccm->CCGR2);

	fuse = readl(&ocotp->thermal_calibration_data);
	writel(ccm_ccgr2, &imx_ccm->CCGR2);
	return fuse;
}

static void get_temperature_scale(u32 fuse, unsigned *praw_25c, unsigned long long *pcvt_to_celsius)
{
	unsigned raw_25c;
	unsigned raw_hot;
	unsigned hot_temp;
	unsigned long long cvt_to_celsius;
	unsigned scale_int, scale_frac;

	/*
	 * Fuse data layout:
	 * Bits[31:20] raw value @ 25C
	 * Bits[19:8] raw value @hot temp
	 * Bits[7:0] hot temperature value
	 */
	raw_25c = fuse >> 20;
	raw_hot = (fuse >> 8) & 0xfff;
	hot_temp = fuse & 0xff;
	if ((hot_temp <= 25) || (raw_25c <= raw_hot)) {
		/* Use default settings */
		printf("Invalid temperature fuse data: use defaults\n");
		raw_25c = DEFAULT_RAW_25C;
		raw_hot = DEFAULT_RAW_HOT;
		hot_temp = DEFAULT_TEMP_HOT;
	}
	cvt_to_celsius = hot_temp - 25;
	cvt_to_celsius <<= 32;
	do_div(cvt_to_celsius, raw_25c - raw_hot);	/* raw25c > raw_hot */
	*praw_25c = raw_25c;
	*pcvt_to_celsius = cvt_to_celsius;
	scale_int = (cvt_to_celsius >> 32);
	scale_frac = ((cvt_to_celsius & 0xffffffff) * 100) >> 32;
	printf("Thermal fuse is 0x%x, raw_25c=%d raw_hot=%d"
			" hot_temp=%d C scale=%d.%02d\n",
			fuse, raw_25c, raw_hot,
			hot_temp, scale_int, scale_frac);
}

#define MEASURE_FREQ	3276  /* 3276 RTC clocks delay, 10ms */
#define BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF	0x8
#define BM_ANADIG_TEMPSENSE0_POWER_DOWN		0x1
#define BM_ANADIG_TEMPSENSE0_MEASURE_TEMP	0x2
#define BM_ANADIG_TEMPSENSE0_FINISHED		0x4
#define BM_ANADIG_TEMPSENSE1_MEASURE_FREQ	0x0000FFFF

static unsigned read_cpu_temperature_raw(void)
{
	u32 reg;
	unsigned raw;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;

	/*
	 * every time we measure the temperature, we will power up/down
	 * the anadig module
	 */
	writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN, &anatop->tempsense0_clr);
	writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF, &anatop->ana_misc0_set);

	/* write measure freq */
	reg = readl(&anatop->tempsense1) & ~BM_ANADIG_TEMPSENSE1_MEASURE_FREQ;
	reg |= MEASURE_FREQ;
	writel(reg, &anatop->tempsense1);

	writel(BM_ANADIG_TEMPSENSE0_MEASURE_TEMP, &anatop->tempsense0_clr);
	writel(BM_ANADIG_TEMPSENSE0_FINISHED, &anatop->tempsense0_clr);
	writel(BM_ANADIG_TEMPSENSE0_MEASURE_TEMP, &anatop->tempsense0_set);

	for (;;) {
		reg = readl(&anatop->tempsense0);
		if (reg & BM_ANADIG_TEMPSENSE0_FINISHED)
			break;
		udelay(1000);
	}
	raw = (reg >> 8) & 0xfff;
	writel(BM_ANADIG_TEMPSENSE0_FINISHED, &anatop->tempsense0_clr);

	/* power down thermal sensor */
	writel(BM_ANADIG_TEMPSENSE0_POWER_DOWN, &anatop->tempsense0_set);
	writel(BM_ANADIG_ANA_MISC0_REFTOP_SELBIASOFF, &anatop->ana_misc0_clr);
	return raw;
}

static int print_cpu_temperature(unsigned raw, unsigned raw_25c, unsigned long long cvt_to_celsius)
{
	int temperature, temp;
	unsigned hundredths;
	int negative = 0;

	temp = temperature = cvt_raw_to_celius(raw, raw_25c, cvt_to_celsius, &hundredths);
	if (temp < 0) {
		negative = 1;
		temp = -temp;
		if (hundredths) {
			temp--;
			hundredths = 100 - hundredths;
		}
	}
	printf("Temperature raw=%d temperature=%c%d.%02d C\n", raw,
			negative ? '-' : ' ', temp, hundredths);
	return temperature;
}

#define TEMP_MIN	-25
#define TEMP_HOT	80
#define TEMP_MAX	125

#define TEMPFUSE "tempfuse"

static u32 get_fuse_data(void)
{
	u32 fuse;
	char const *fuse_env = getenv(TEMPFUSE);
	if (fuse_env) {
		printf("use tempfuse(%s) from environment\n", fuse_env);
		fuse = simple_strtoul(fuse_env, 0, 16);
	} else
		fuse = get_temperature_fuse();
	return fuse;
}

int check_cpu_temperature(void)
{
	int cpu_temp;
	unsigned raw_25c;
	unsigned long long cvt_to_celsius;
	u32 fuse = get_fuse_data();

	get_temperature_scale(fuse, &raw_25c, &cvt_to_celsius);

	for (;;) {
		unsigned raw = read_cpu_temperature_raw();
		cpu_temp = print_cpu_temperature(raw, raw_25c, cvt_to_celsius);
		if ((cpu_temp > TEMP_MAX) || (cpu_temp < TEMP_MIN)) {
			printf("Invalid temperature reading\n");
			break;
		}
		if (cpu_temp < TEMP_HOT)
			break;
		printf("CPU is %d C, too hot to boot, waiting for below %d C...\n",
				cpu_temp, TEMP_HOT);
		udelay(5000000);
	}
	return cpu_temp;
}

int do_temp(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int cpu_temp;
	unsigned raw_25c;
	u32 raw;
	unsigned long long cvt_to_celsius;
	u32 fuse = get_fuse_data();

	get_temperature_scale(fuse, &raw_25c, &cvt_to_celsius);

	raw = read_cpu_temperature_raw();
	cpu_temp = print_cpu_temperature(raw, raw_25c, cvt_to_celsius);
	if ((cpu_temp > TEMP_MAX) || (cpu_temp < TEMP_MIN)) {
		printf("Invalid temperature reading\n");
		return -EINVAL;
	} else if (1 < argc)
		return cpu_temp > simple_strtoul(argv[1], 0, 10);
	else
		return 0;
}

U_BOOT_CMD(
	temp,	1, 0, do_temp,
	"display temperature",
	""
);

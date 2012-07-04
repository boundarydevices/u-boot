/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
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
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>

#define BF(value, field)	(((value) << BP_##field) & BM_##field)
#define DEF_RELAX 20

#ifdef CONFIG_IMX_OTP_DEBUG
#define log(a, ...) printf("[%s,%3d]:"a"\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define log(a, ...)
#endif

static int otp_wait_busy(u32 flags)
{
	int count;
	u32 c;
	struct iim_regs *otp = (struct iim_regs *)IMX_OTP_BASE;

	for (count = 10000; count >= 0; count--) {
		c = readl(&otp->ctrl);
		if (!(c & (BM_OCOTP_CTRL_BUSY | BM_OCOTP_CTRL_ERROR | flags)))
			break;
	}

	if (count < 0) {
		printf("ERROR: otp_wait_busy timeout. 0x%X\n", c);
		/* clear ERROR bit, busy bit will be cleared by controller */
		writel(BM_OCOTP_CTRL_ERROR, &otp->ctrl_clr);
		return -1;
	}

	log("wait busy successful.");
	return 0;
}

static int set_otp_timing(void)
{
	u32 clk_rate;
	u32 relax, strobe_read, strobe_prog;
	u32 timing;
	struct iim_regs *otp = (struct iim_regs *)IMX_OTP_BASE;

	/* get clock */
	clk_rate = mxc_get_clock(MXC_IPG_CLK);
	if (clk_rate == -1) {
		printf("ERROR: mxc_get_clock failed\n");
		return -1;
	}

	log("clk_rate: %d.", clk_rate);

	relax = clk_rate / (1000000000 / DEF_RELAX) - 1;
	strobe_prog = clk_rate / (1000000000 / 10000) + 2 * (DEF_RELAX + 1) - 1;
	strobe_read = clk_rate / (1000000000 / 40) + 2 * (DEF_RELAX + 1) - 1;

	timing = BF(relax, OCOTP_TIMING_RELAX);
	timing |= BF(strobe_read, OCOTP_TIMING_STROBE_READ);
	timing |= BF(strobe_prog, OCOTP_TIMING_STROBE_PROG);
	log("timing: 0x%X", timing);

	writel(timing, &otp->timing);

	return 0;
}

static int otp_read_prep(void)
{
	return  (!set_otp_timing()) ? otp_wait_busy(0) : -1;
}

#ifdef IMX_OTPWRITE_ENABLED
static int otp_blow_prep(void)
{
	return  (!set_otp_timing()) ? otp_wait_busy(0) : -1;
}

static int otp_blow_post(void)
{
	struct iim_regs *otp = (struct iim_regs *)IMX_OTP_BASE;

	printf("Reloading shadow registers...\n");
	/* reload all the shadow registers */
	writel(BM_OCOTP_CTRL_RELOAD_SHADOWS, &otp->ctrl_set);
	udelay(1);

	return otp_wait_busy(BM_OCOTP_CTRL_RELOAD_SHADOWS);
}
#endif

static int fuse_read_addr(u32 addr, u32 *pdata)
{
	u32 ctrl_reg;
	struct iim_regs *otp = (struct iim_regs *)IMX_OTP_BASE;

	ctrl_reg = readl(&otp->ctrl);
	ctrl_reg &= ~BM_OCOTP_CTRL_ADDR;
	ctrl_reg &= ~BM_OCOTP_CTRL_WR_UNLOCK;
	ctrl_reg |= BF(addr, OCOTP_CTRL_ADDR);
	writel(ctrl_reg, &otp->ctrl);

	writel(BM_OCOTP_READ_CTRL_READ_FUSE, &otp->read_ctrl);
	if (otp_wait_busy(0))
		return -1;

	*pdata = readl(&otp->fuse_data);
	return 0;
}

#ifdef IMX_OTPWRITE_ENABLED
static int fuse_blow_addr(u32 addr, u32 value)
{
	u32 ctrl_reg;
	struct iim_regs *otp = (struct iim_regs *)IMX_OTP_BASE;

	log("blowing...");

	/* control register */
	ctrl_reg = readl(&otp->ctrl);
	ctrl_reg &= ~BM_OCOTP_CTRL_ADDR;
	ctrl_reg |= BF(addr, OCOTP_CTRL_ADDR);
	ctrl_reg |= BF(BV_OCOTP_CTRL_WR_UNLOCK__KEY, OCOTP_CTRL_WR_UNLOCK);
	writel(ctrl_reg, &otp->ctrl);

	writel(value, &otp->data);
	if (otp_wait_busy(0))
		return -1;

	/* write postamble */
	udelay(2000);
	return 0;
}
#endif

/*
 * read one u32 to indexed fuse
 */
int imx_otp_read_one_u32(u32 index, u32 *pdata)
{
	u32 ctrl_reg;
	int ret = 0;
	struct iim_regs *otp = (struct iim_regs *)IMX_OTP_BASE;

	log("index: 0x%X", index);

	if (index > IMX_OTP_ADDR_MAX) {
		printf("ERROR: invalid address.\n");
		ret = -1;
		goto exit_nop;
	}

	enable_otp_clk(1);

	if (otp_read_prep()) {
		ret = -1;
		printf("ERROR: read preparation failed\n");
		goto exit_cleanup;
	}

	if (fuse_read_addr(index, pdata)) {
		ret = -1;
		printf("ERROR: read failed\n");
		goto exit_cleanup;
	}

	if (*pdata == IMX_OTP_DATA_ERROR_VAL) {
		ctrl_reg = readl(&otp->ctrl);
		if (ctrl_reg & BM_OCOTP_CTRL_ERROR) {
			printf("ERROR: read fuse failed\n");
			ret = -1;
		}
	}

exit_cleanup:
	enable_otp_clk(0);
exit_nop:
	return ret;
}

#ifdef IMX_OTPWRITE_ENABLED
/*
 * blow one u32 to indexed fuse
 */
int imx_otp_blow_one_u32(u32 index, u32 data, u32 *pfused_value)
{
	int ret = 0;
	struct iim_regs *otp = (struct iim_regs *)IMX_OTP_BASE;

	enable_otp_clk(1);

	if (otp_blow_prep()) {
		ret = -1;
		printf("ERROR: blow preparation failed\n");
		goto exit_cleanup;
	}

	if (fuse_blow_addr(index, data)) {
		ret = -1;
		printf("ERROR: blow fuse failed\n");
		goto exit_cleanup;
	}

	if (otp_blow_post()) {
		ret = -1;
		printf("ERROR: blow post operation failed\n");
		goto exit_cleanup;
	}

	if (readl(&otp->ctrl) & BM_OCOTP_CTRL_ERROR) {
		ret = -1;
		printf("ERROR: OTP control\n");
		goto exit_cleanup;
	}

	if (imx_otp_read_one_u32(index, pfused_value)) {
		ret = -1;
		printf("ERROR: OTP read\n");
	}

exit_cleanup:
	enable_otp_clk(0);

	return ret;
}
#endif

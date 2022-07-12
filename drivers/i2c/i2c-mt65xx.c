// SPDX-License-Identifier: GPL-2.0-only
/*
 * Mediatek i2c driver
 *
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <i2c.h>
#include <linux/delay.h>

#define I2C_RS_TRANSFER			(1 << 4)
#define I2C_HS_NACKERR			(1 << 2)
#define I2C_ACKERR			(1 << 1)
#define I2C_TRANSAC_COMP		(1 << 0)
#define I2C_TRANSAC_START		(1 << 0)
#define I2C_RS_MUL_CNFG			(1 << 15)
#define I2C_RS_MUL_TRIG			(1 << 14)
#define I2C_DCM_DISABLE			0x0000
#define I2C_IO_CONFIG_OPEN_DRAIN	0x0003
#define I2C_SOFT_RST			0x0001
#define I2C_FIFO_ADDR_CLR		0x0001
#define I2C_DELAY_LEN			0x0002
#define I2C_ST_START_CON		0x8001
#define I2C_FS_START_CON		0x1800
#define I2C_TIME_CLR_VALUE		0x0000
#define I2C_TIME_DEFAULT_VALUE		0x0003
#define I2C_WRRD_TRANAC_VALUE		0x0002
#define I2C_CHN_CLR_FLAG		0x0000

#define I2C_DEFAULT_CLK_DIV		5
#define I2C_DEFAULT_SPEED		100000	/* hz */
#define MAX_FS_MODE_SPEED		400000
#define MAX_HS_MODE_SPEED		3400000
#define MAX_SAMPLE_CNT_DIV		8
#define MAX_STEP_CNT_DIV		64
#define MAX_HS_STEP_CNT_DIV		8

#define I2C_CONTROL_RS			(0x1 << 1)
#define I2C_CONTROL_CLK_EXT_EN		(0x1 << 3)
#define I2C_CONTROL_DIR_CHANGE		(0x1 << 4)
#define I2C_CONTROL_ACKERR_DET_EN	(0x1 << 5)

enum mtk_trans_op {
	I2C_MASTER_WR = 1,
	I2C_MASTER_RD,
	I2C_MASTER_WRRD,
};

enum I2C_REGS_OFFSET {
	OFFSET_DATA_PORT,
	OFFSET_SLAVE_ADDR,
	OFFSET_INTR_MASK,
	OFFSET_INTR_STAT,
	OFFSET_CONTROL,
	OFFSET_TRANSFER_LEN,
	OFFSET_TRANSAC_LEN,
	OFFSET_DELAY_LEN,
	OFFSET_TIMING,
	OFFSET_START,
	OFFSET_EXT_CONF,
	OFFSET_FIFO_STAT,
	OFFSET_FIFO_ADDR_CLR,
	OFFSET_IO_CONFIG,
	OFFSET_HS,
	OFFSET_SOFTRESET,
	OFFSET_DCM_EN,
	OFFSET_TRANSFER_LEN_AUX,
	OFFSET_CLOCK_DIV,
	OFFSET_LTIMING,
};

static const u16 mt_i2c_regs_v1[] = {
	[OFFSET_DATA_PORT] = 0x0,
	[OFFSET_SLAVE_ADDR] = 0x4,
	[OFFSET_INTR_MASK] = 0x8,
	[OFFSET_INTR_STAT] = 0xc,
	[OFFSET_CONTROL] = 0x10,
	[OFFSET_TRANSFER_LEN] = 0x14,
	[OFFSET_TRANSAC_LEN] = 0x18,
	[OFFSET_DELAY_LEN] = 0x1c,
	[OFFSET_TIMING] = 0x20,
	[OFFSET_START] = 0x24,
	[OFFSET_EXT_CONF] = 0x28,
	[OFFSET_FIFO_STAT] = 0x30,
	[OFFSET_FIFO_ADDR_CLR] = 0x38,
	[OFFSET_IO_CONFIG] = 0x40,
	[OFFSET_HS] = 0x48,
	[OFFSET_SOFTRESET] = 0x50,
	[OFFSET_DCM_EN] = 0x54,
	[OFFSET_TRANSFER_LEN_AUX] = 0x6c,
	[OFFSET_CLOCK_DIV] = 0x70,
};

static const u16 mt_i2c_regs_v2[] = {
	[OFFSET_DATA_PORT] = 0x0,
	[OFFSET_SLAVE_ADDR] = 0x4,
	[OFFSET_INTR_MASK] = 0x8,
	[OFFSET_INTR_STAT] = 0xc,
	[OFFSET_CONTROL] = 0x10,
	[OFFSET_TRANSFER_LEN] = 0x14,
	[OFFSET_TRANSAC_LEN] = 0x18,
	[OFFSET_DELAY_LEN] = 0x1c,
	[OFFSET_TIMING] = 0x20,
	[OFFSET_START] = 0x24,
	[OFFSET_EXT_CONF] = 0x28,
	[OFFSET_FIFO_STAT] = 0xf4,
	[OFFSET_FIFO_ADDR_CLR] = 0x38,
	[OFFSET_IO_CONFIG] = 0x34,
	[OFFSET_HS] = 0x30,
	[OFFSET_SOFTRESET] = 0x50,
	[OFFSET_DCM_EN] = 0xf88,
	[OFFSET_TRANSFER_LEN_AUX] = 0x44,
	[OFFSET_CLOCK_DIV] = 0x48,
	[OFFSET_LTIMING] = 0x2c,
};

struct mtk_i2c_compatible {
	const u16 *regs;
	unsigned char pmic_i2c: 1;
	unsigned char dcm: 1;
	unsigned char auto_restart: 1;
	unsigned char aux_len_reg: 1;
	unsigned char timing_adjust: 1;
	unsigned char ltiming_adjust: 1;
};

struct mtk_i2c {
	struct udevice *bus;
	void __iomem *base;
	struct clk *main_clk;
	unsigned int speed_hz;
	unsigned int clk_src_div;
	const struct mtk_i2c_compatible *dev_comp;
	enum mtk_trans_op op;
	u16 timing_reg;
	u16 high_speed_reg;
	u16 ltiming_reg;
	unsigned char auto_restart;
};

static const struct mtk_i2c_compatible mt8183_compat = {
	.regs = mt_i2c_regs_v2,
	.pmic_i2c = 0,
	.dcm = 0,
	.auto_restart = 1,
	.aux_len_reg = 1,
	.timing_adjust = 1,
	.ltiming_adjust = 1,
};

static const struct mtk_i2c_compatible mt8365_compat = {
	.regs = mt_i2c_regs_v1,
	.pmic_i2c = 0,
	.dcm = 1,
	.auto_restart = 1,
	.aux_len_reg = 1,
	.timing_adjust = 1,
	.ltiming_adjust = 0,
};

static u16 mtk_i2c_readw(struct mtk_i2c *i2c, enum I2C_REGS_OFFSET reg)
{
	return readw(i2c->base + i2c->dev_comp->regs[reg]);
}

static void mtk_i2c_writew(struct mtk_i2c *i2c, u16 val,
			   enum I2C_REGS_OFFSET reg)
{
	writew(val, i2c->base + i2c->dev_comp->regs[reg]);
}

static int mtk_i2c_clock_enable(struct mtk_i2c *i2c)
{
	return clk_enable(i2c->main_clk);
}

static void mtk_i2c_clock_disable(struct mtk_i2c *i2c)
{
	clk_disable(i2c->main_clk);
}

static void mtk_i2c_init_hw(struct mtk_i2c *i2c)
{
	u16 control_reg;
	u16 intr_stat_reg;
	u16 ext_conf_val;

	mtk_i2c_writew(i2c, I2C_CHN_CLR_FLAG, OFFSET_START);
	intr_stat_reg = mtk_i2c_readw(i2c, OFFSET_INTR_STAT);
	mtk_i2c_writew(i2c, intr_stat_reg, OFFSET_INTR_STAT);

	mtk_i2c_writew(i2c, I2C_SOFT_RST, OFFSET_SOFTRESET);

	mtk_i2c_writew(i2c, I2C_IO_CONFIG_OPEN_DRAIN, OFFSET_IO_CONFIG);

	if (i2c->dev_comp->dcm)
		mtk_i2c_writew(i2c, I2C_DCM_DISABLE, OFFSET_DCM_EN);

	if (i2c->dev_comp->timing_adjust)
		mtk_i2c_writew(i2c, I2C_DEFAULT_CLK_DIV - 1, OFFSET_CLOCK_DIV);

	mtk_i2c_writew(i2c, i2c->timing_reg, OFFSET_TIMING);
	mtk_i2c_writew(i2c, i2c->high_speed_reg, OFFSET_HS);
	if (i2c->dev_comp->ltiming_adjust)
		mtk_i2c_writew(i2c, i2c->ltiming_reg, OFFSET_LTIMING);

	if (i2c->speed_hz <= I2C_DEFAULT_SPEED)
		ext_conf_val = I2C_ST_START_CON;
	else
		ext_conf_val = I2C_FS_START_CON;
	mtk_i2c_writew(i2c, ext_conf_val, OFFSET_EXT_CONF);

	control_reg = I2C_CONTROL_ACKERR_DET_EN | I2C_CONTROL_CLK_EXT_EN;

	mtk_i2c_writew(i2c, control_reg, OFFSET_CONTROL);
	mtk_i2c_writew(i2c, I2C_DELAY_LEN, OFFSET_DELAY_LEN);
}

/*
 * Calculate i2c port speed
 *
 * Hardware design:
 * i2c_bus_freq = parent_clk / (clock_div * 2 * sample_cnt * step_cnt)
 * clock_div: fixed in hardware, but may be various in different SoCs
 *
 * The calculation want to pick the highest bus frequency that is still
 * less than or equal to i2c->speed_hz. The calculation try to get
 * sample_cnt and step_cn
 */
static int mtk_i2c_calculate_speed(struct mtk_i2c *i2c, unsigned int clk_src,
				   unsigned int target_speed,
				   unsigned int *timing_step_cnt,
				   unsigned int *timing_sample_cnt)
{
	unsigned int step_cnt;
	unsigned int sample_cnt;
	unsigned int max_step_cnt;
	unsigned int base_sample_cnt = MAX_SAMPLE_CNT_DIV;
	unsigned int base_step_cnt;
	unsigned int opt_div;
	unsigned int best_mul;
	unsigned int cnt_mul;

	if (target_speed > MAX_HS_MODE_SPEED)
		target_speed = MAX_HS_MODE_SPEED;

	if (target_speed > MAX_FS_MODE_SPEED)
		max_step_cnt = MAX_HS_STEP_CNT_DIV;
	else
		max_step_cnt = MAX_STEP_CNT_DIV;

	base_step_cnt = max_step_cnt;
	/* Find the best combination */
	opt_div = DIV_ROUND_UP(clk_src >> 1, target_speed);
	best_mul = MAX_SAMPLE_CNT_DIV * max_step_cnt;

	/* Search for the best pair (sample_cnt, step_cnt) with
	 * 0 < sample_cnt < MAX_SAMPLE_CNT_DIV
	 * 0 < step_cnt < max_step_cnt
	 * sample_cnt * step_cnt >= opt_div
	 * optimizing for sample_cnt * step_cnt being minimal
	 */
	for (sample_cnt = 1; sample_cnt <= MAX_SAMPLE_CNT_DIV; sample_cnt++) {
		step_cnt = DIV_ROUND_UP(opt_div, sample_cnt);
		cnt_mul = step_cnt * sample_cnt;
		if (step_cnt > max_step_cnt)
			continue;

		if (cnt_mul < best_mul) {
			best_mul = cnt_mul;
			base_sample_cnt = sample_cnt;
			base_step_cnt = step_cnt;
			if (best_mul == opt_div)
				break;
		}
	}

	sample_cnt = base_sample_cnt;
	step_cnt = base_step_cnt;

	if ((clk_src / (2 * sample_cnt * step_cnt)) > target_speed) {
		/* In this case, hardware can't support such
		 * low i2c_bus_freq
		 */
		dev_err(i2c->bus, "Unsupported speed (%uhz)\n", target_speed);
		return -EINVAL;
	}

	*timing_step_cnt = step_cnt - 1;
	*timing_sample_cnt = sample_cnt - 1;

	return 0;
}

static int mtk_i2c_set_speed(struct mtk_i2c *i2c, unsigned int parent_clk)
{
	unsigned int clk_src;
	unsigned int step_cnt;
	unsigned int sample_cnt;
	unsigned int l_step_cnt;
	unsigned int l_sample_cnt;
	unsigned int target_speed;
	int ret;

	clk_src = parent_clk / i2c->clk_src_div;
	target_speed = i2c->speed_hz;

	if (target_speed > MAX_FS_MODE_SPEED) {
		/* Set master code speed register */
		ret = mtk_i2c_calculate_speed(i2c, clk_src, MAX_FS_MODE_SPEED,
					      &l_step_cnt, &l_sample_cnt);
		if (ret < 0)
			return ret;

		i2c->timing_reg = (l_sample_cnt << 8) | l_step_cnt;

		/* Set the high speed mode register */
		ret = mtk_i2c_calculate_speed(i2c, clk_src, target_speed,
					      &step_cnt, &sample_cnt);
		if (ret < 0)
			return ret;

		i2c->high_speed_reg = I2C_TIME_DEFAULT_VALUE | (sample_cnt << 12) |
				      (step_cnt << 8);

		if (i2c->dev_comp->ltiming_adjust)
			i2c->ltiming_reg = (l_sample_cnt << 6) | l_step_cnt |
					   (sample_cnt << 12) | (step_cnt << 9);
	} else {
		ret = mtk_i2c_calculate_speed(i2c, clk_src, target_speed,
					      &step_cnt, &sample_cnt);
		if (ret < 0)
			return ret;

		i2c->timing_reg = (sample_cnt << 8) | step_cnt;

		/* Disable the high speed transaction */
		i2c->high_speed_reg = I2C_TIME_CLR_VALUE;

		if (i2c->dev_comp->ltiming_adjust)
			i2c->ltiming_reg = (sample_cnt << 6) | step_cnt;
	}

	return 0;
}

static bool mtk_i2c_wait_for_completion(struct mtk_i2c *i2c, u16 restart_flag)
{
	u64 timeout_us = UL(100000);
	u64 delay = timeout_us / 10;
	u16 intr_stat;
	bool complete = false;

	do {
		intr_stat = mtk_i2c_readw(i2c, OFFSET_INTR_STAT);
		mtk_i2c_writew(i2c, intr_stat, OFFSET_INTR_STAT);

		if (intr_stat & (I2C_TRANSAC_COMP | restart_flag))
			complete = true;

		udelay(delay);
		timeout_us -= delay;
	} while (!complete && timeout_us);

	return complete;
}

static int mtk_i2c_do_transfer(struct mtk_i2c *i2c, struct i2c_msg *msgs,
			       int num, int left_num)
{
	u16 control_reg;
	u16 addr_reg;
	u16 start_reg;
	u16 restart_flag = 0;
	bool complete;
	u8 *ptr = msgs->buf;
	u16 data_size = msgs->len;

	if (i2c->auto_restart)
		restart_flag = I2C_RS_TRANSFER;

	control_reg = mtk_i2c_readw(i2c, OFFSET_CONTROL) &
		      ~(I2C_CONTROL_DIR_CHANGE | I2C_CONTROL_RS);
	if ((i2c->speed_hz > MAX_FS_MODE_SPEED) || (num > 1))
		control_reg |= I2C_CONTROL_RS;

	if (i2c->op == I2C_MASTER_WRRD)
		control_reg |= I2C_CONTROL_DIR_CHANGE | I2C_CONTROL_RS;

	mtk_i2c_writew(i2c, control_reg, OFFSET_CONTROL);

	addr_reg = (msgs->addr << 1) | (msgs->flags & I2C_M_RD ? 1 : 0);
	mtk_i2c_writew(i2c, addr_reg, OFFSET_SLAVE_ADDR);

	/* Clear interrupt status */
	mtk_i2c_writew(i2c, restart_flag | I2C_HS_NACKERR | I2C_ACKERR |
		       I2C_TRANSAC_COMP, OFFSET_INTR_STAT);

	mtk_i2c_writew(i2c, I2C_FIFO_ADDR_CLR, OFFSET_FIFO_ADDR_CLR);

	/* Enable interrupt */
	mtk_i2c_writew(i2c, restart_flag | I2C_HS_NACKERR | I2C_ACKERR |
		       I2C_TRANSAC_COMP, OFFSET_INTR_MASK);

	/* Set transfer and transaction len */
	if (i2c->op == I2C_MASTER_WRRD) {
		if (i2c->dev_comp->aux_len_reg) {
			mtk_i2c_writew(i2c, msgs->len, OFFSET_TRANSFER_LEN);
			mtk_i2c_writew(i2c, (msgs + 1)->len, OFFSET_TRANSFER_LEN_AUX);
		} else {
			mtk_i2c_writew(i2c, msgs->len | ((msgs + 1)->len) << 8,
				       OFFSET_TRANSFER_LEN);
		}
		mtk_i2c_writew(i2c, I2C_WRRD_TRANAC_VALUE, OFFSET_TRANSAC_LEN);
	} else {
		mtk_i2c_writew(i2c, msgs->len, OFFSET_TRANSFER_LEN);
		mtk_i2c_writew(i2c, num, OFFSET_TRANSAC_LEN);
	}

	if (i2c->op != I2C_MASTER_RD) {
		while (data_size--) {
			mtk_i2c_writew(i2c, *ptr, OFFSET_DATA_PORT);
			ptr++;
		}
	}

	if (!i2c->auto_restart) {
		start_reg = I2C_TRANSAC_START;
	} else {
		start_reg = I2C_TRANSAC_START | I2C_RS_MUL_TRIG;
		if (left_num >= 1)
			start_reg |= I2C_RS_MUL_CNFG;
	}
	mtk_i2c_writew(i2c, start_reg, OFFSET_START);

	complete = mtk_i2c_wait_for_completion(i2c, restart_flag);

	/* Clear interrupt mask */
	mtk_i2c_writew(i2c, ~(restart_flag | I2C_HS_NACKERR | I2C_ACKERR |
			      I2C_TRANSAC_COMP), OFFSET_INTR_MASK);

	if (complete) {
		/* transfer success */
		if (i2c->op != I2C_MASTER_WR) {
			ptr = (i2c->op == I2C_MASTER_RD) ? msgs->buf : (msgs + 1)->buf;
			data_size = (mtk_i2c_readw(i2c, OFFSET_FIFO_STAT) >> 4) & 0x000F;

			while (data_size--) {
				*ptr = mtk_i2c_readw(i2c, OFFSET_DATA_PORT);
				ptr++;
			}
		}
	} else {
		mtk_i2c_init_hw(i2c);
		return -ETIMEDOUT;
	}

	return 0;
}

static int mtk_i2c_transfer(struct udevice *bus, struct i2c_msg *msgs, int num)
{
	struct mtk_i2c *i2c = dev_get_priv(bus);
	int left_num = num;
	int num_cnt;
	int ret;

	ret = mtk_i2c_clock_enable(i2c);
	if (ret)
		return ret;

	i2c->auto_restart = i2c->dev_comp->auto_restart;

	for (num_cnt = 0; num_cnt < num; num_cnt++) {
		if (((msgs + num_cnt)->addr) > 0x7f) {
			dev_err(bus, "i2c addr: msgs[%d]->addr(%x) > 0x7f, error!\n",
				num_cnt, ((msgs + num_cnt)->addr));
			return -EINVAL;
		}

		if (((msgs + num_cnt)->len) > 8) {
			dev_err(bus, "FIFO MODE: msgs[%d]->len(%d) > 8, error!\n",
				num_cnt, ((msgs + num_cnt)->len));
			return -EINVAL;
		}
	}

	while (left_num--) {
		if (!msgs->buf) {
			dev_err(bus, "data buffer is NULL\n");
			ret = -EINVAL;
			goto err_exit;
		}

		if (msgs->flags & I2C_M_RD)
			i2c->op = I2C_MASTER_RD;
		else
			i2c->op = I2C_MASTER_WR;

		if (!i2c->auto_restart) {
			if (num > 1) {
				/* combined two messages into one transaction */
				i2c->op = I2C_MASTER_WRRD;
				left_num--;
			}
		}

		ret = mtk_i2c_do_transfer(i2c, msgs, num, left_num);
		if (ret < 0)
			goto err_exit;

		msgs++;
	}

err_exit:
	mtk_i2c_clock_disable(i2c);
	return ret;
}

static int mtk_i2c_probe_chip(struct udevice *bus, u32 chip_addr, u32 chip_flags)
{
	struct i2c_msg msg[1];
	u8 buf = 0;

	msg->addr = chip_addr;
	msg->len = 1;
	msg->buf = &buf;
	msg->flags = chip_flags & DM_I2C_CHIP_10BIT ? I2C_M_TEN : 0;

	return mtk_i2c_transfer(bus, msg, 1);
}

static int mtk_i2c_probe(struct udevice *bus)
{
	struct mtk_i2c *i2c = dev_get_priv(bus);
	int ret = 0;

	if (i2c->dev_comp->timing_adjust)
		i2c->clk_src_div *= I2C_DEFAULT_CLK_DIV;

	ret = mtk_i2c_set_speed(i2c, clk_get_rate(i2c->main_clk));
	if (ret) {
		dev_err(bus, "Failed to set the speed.\n");
		return -EINVAL;
	}

	ret = mtk_i2c_clock_enable(i2c);
	if (ret) {
		dev_err(bus, "clock enable failed!\n");
		return ret;
	}
	mtk_i2c_init_hw(i2c);
	mtk_i2c_clock_disable(i2c);

	return 0;
}

static int mtk_i2c_ofdata_to_platdata(struct udevice *bus)
{
	struct mtk_i2c *i2c = dev_get_priv(bus);
	int ret;

	i2c->bus = bus;

	i2c->base = dev_remap_addr(bus);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	i2c->main_clk = devm_clk_get(bus, "main");
	if (IS_ERR(i2c->main_clk))
		return PTR_ERR(i2c->main_clk);

	i2c->speed_hz = dev_read_u32_default(bus, "clock-frequency", I2C_DEFAULT_SPEED);

	ret = dev_read_u32(bus, "clock-div", &i2c->clk_src_div);
	if (ret) {
		dev_err(bus, "Missing clock-div value in device tree!");
		return -EINVAL;
	}

	i2c->dev_comp = (const struct mtk_i2c_compatible *)dev_get_driver_data(bus);

	return 0;
}

static const struct dm_i2c_ops mtk_i2c_ops = {
	.xfer	    = mtk_i2c_transfer,
	.probe_chip = mtk_i2c_probe_chip,
};

static const struct udevice_id mtk_i2c_ids[] = {
	{ .compatible = "mediatek,mt8183-i2c", .data = (ulong)&mt8183_compat },
	{ .compatible = "mediatek,mt8365-i2c", .data = (ulong)&mt8365_compat },
	{ }
};

U_BOOT_DRIVER(i2c_mt65xx) = {
	.name	    = "i2c_mt65xx",
	.id	    = UCLASS_I2C,
	.of_match   = mtk_i2c_ids,
	.probe	    = mtk_i2c_probe,
	.priv_auto  = sizeof(struct mtk_i2c),
	.of_to_plat = mtk_i2c_ofdata_to_platdata,
	.ops	    = &mtk_i2c_ops,
};

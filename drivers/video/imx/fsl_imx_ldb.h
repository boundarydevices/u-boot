// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#ifndef __FSL_IMX_LDB__
#define __FSL_IMX_LDB__

#define LDB_CH_NUM	2

struct ldb_channel {
	struct ldb *ldb;
	struct udevice *panel;
	ofnode child;
	int chno;
	u32 bus_format;
	bool is_valid;
};

struct ldb {
	struct regmap *regmap;
	struct udevice *dev;
	struct ldb_channel *channel[LDB_CH_NUM];
	unsigned int ctrl_reg;
	u32 ldb_ctrl;
	int output_port;
	bool dual;
};

void ldb_bridge_mode_set(struct ldb_channel *ldb_ch, enum display_flags flags);
void ldb_bridge_enable(struct ldb_channel *ldb_ch);
void ldb_bridge_disable(struct ldb_channel *ldb_ch);

int ldb_bind(struct ldb *ldb);

#endif /* __FSL_IMX_LDB__ */

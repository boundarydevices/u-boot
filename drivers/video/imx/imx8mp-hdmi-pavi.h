// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020 NXP
 *
 * PAI/PVI Head file
 *
 */
#ifndef _IMX8MP_HDMI_AV_CTL_H_
#define _IMX8MP_HDMI_AV_CTL_H_

struct imx8mp_hdmi_pavi {
	struct udevice *dev;

	void __iomem *base;
	int rpm_suspended;

	struct clk *clk_pai;
	struct clk *clk_pvi;
	struct reset_ctl *reset_pai;
	struct reset_ctl *reset_pvi;
	enum display_flags flags;
	int channel;
	int width;
	int rate;
	int non_pcm;
};
#endif /* _IMX8MP_HDMI_PAVI_H_ */

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/mach-imx/sys_proto.h>
#include <clk.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/device.h>
#include <imx_sip.h>
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <malloc.h>
#include <power/regulator.h>
#include <power-domain-uclass.h>

#define MAX_CLK_NUM	6

struct imx8m_pm_domain_platdata {
	u32 domain_index;
	struct udevice *regulator;
#define GENPD_FLAG_ACTIVE_WAKEUP	BIT(0)
#define GENPD_FLAG_RPM_ALWAYS_ON	BIT(1)
	u32 flags;
	const char *name;
        struct power_domain parent_pd;
	unsigned int num_clks;
	struct clk clk[MAX_CLK_NUM];
	int enable_count;
};

enum imx8m_pm_domain_state {
	PD_STATE_OFF,
	PD_STATE_ON,
};

static int imx8m_pm_domain_request(struct power_domain *power_domain)
{
	return 0;
}

static int imx8m_pm_domain_free(struct power_domain *power_domain)
{
	return 0;
}

static int imx8m_pm_domain_on(struct power_domain *power_domain)
{
	struct udevice *dev = power_domain->dev;
	struct imx8m_pm_domain_platdata *pd = dev_get_platdata(dev);
	struct arm_smccc_res res;
	int index, ret = 0;

	if (pd->enable_count++)
		return 0;
	if (pd->parent_pd.dev)
		power_domain_on(&pd->parent_pd);

	/* power on the external supply */
	if (pd->regulator) {
		ret = regulator_set_enable(pd->regulator, true);
		if (ret) {
			debug("failed to power up the regulator%d\n", ret);
			return ret;
		}
	}
	/* enable the necessary clks needed by the power pd */
	for (index = 0; index < pd->num_clks; index++)
		clk_prepare_enable(&pd->clk[index]);

	debug("%s: %d\n", __func__, pd->domain_index);
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN, pd->domain_index,
		      PD_STATE_ON, 0, 0, 0, 0, &res);

	return 0;
}

static int imx8m_pm_domain_off(struct power_domain *power_domain)
{
	struct udevice *dev = power_domain->dev;
	struct imx8m_pm_domain_platdata *pd = dev_get_platdata(dev);
	struct arm_smccc_res res;
	int index, ret = 0;

	if (--pd->enable_count)
		return 0;
	debug("%s: %d\n", __func__, pd->domain_index);
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN, pd->domain_index,
		      PD_STATE_OFF, 0, 0, 0, 0, &res);

	/* disable clks when power pd is off */
	for (index = 0; index < pd->num_clks; index++)
		clk_disable_unprepare(&pd->clk[index]);

	/* power off the external supply */
	if (pd->regulator) {
		ret = regulator_set_enable(pd->regulator, false);
		if (ret) {
			debug("failed to power off the regulator%d\n", ret);
			return ret;
		}
	}

	if (pd->parent_pd.dev)
		power_domain_off(&pd->parent_pd);

	return ret;
};

static int imx8m_pm_domain_of_xlate(struct power_domain *power_domain,
				      struct ofnode_phandle_args *args)
{
	return 0;
}


static int imx8m_pd_get_clocks(struct imx8m_pm_domain_platdata *pd, struct udevice *dev)
{
	int i, ret;

	for (i = 0; ; i++) {
		ret = clk_get_by_index(dev, i, &pd->clk[i]);
		if (ret)
			break;
		if (i >= MAX_CLK_NUM) {
			debug("more than %d clocks\n", MAX_CLK_NUM);
			return -EINVAL;
		}
	}
	pd->num_clks = i;
	return 0;
}

static int imx8m_pm_domain_probe(struct udevice *dev)
{
	return 0;
}

static int imx8m_pm_domain_ofdata_to_platdata(struct udevice *dev)
{
	struct imx8m_pm_domain_platdata *pd = dev_get_platdata(dev);
	ofnode np = dev_ofnode(dev);
	struct ofnode_phandle_args parent;
	int ret;

	pd->name = ofnode_read_string(np, "domain-name");
	if (!pd->name) {
		debug("failed to get the domain name\n");
		return -EINVAL;
	}

	ret = ofnode_read_u32(np, "domain-index", &pd->domain_index);
	if (ret) {
		debug("failed to get the domain index\n");
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
						   "power-supply", &pd->regulator);
		if (ret) {
			if (ret != -ENOENT) {
				debug("%s: cannot get power regulator: ret=%d\n",
				      __func__, ret);
				return ret;
			}
		}
	}

	ret = imx8m_pd_get_clocks(pd, dev);
	if (ret) {
		debug("failed to get domain's clocks\n");
		return ret;
	}

	if (ofnode_read_bool(np, "active-wakeup"))
		pd->flags |= GENPD_FLAG_ACTIVE_WAKEUP;
	if (ofnode_read_bool(np, "rpm-always-on"))
		pd->flags |= GENPD_FLAG_RPM_ALWAYS_ON;

	/* add it as subdomain if necessary */
	if (!ofnode_parse_phandle_with_args(np, "parent-domains",
			"#power-domain-cells", 0, 0, &parent)) {
		ret = power_domain_get_by_args(&pd->parent_pd, &parent);
	        if (ret) {
	                debug("power_domain_request_by_args failed: %d\n", ret);
	                return ret;
	        }
	}
	return 0;
}

static const struct udevice_id imx8m_pm_domain_ids[] = {
	{.compatible = "fsl,imx8m-pm-domain"},
	{},
};

static struct power_domain_ops imx8m_pm_domain_ops = {
	.request = imx8m_pm_domain_request,
	.rfree = imx8m_pm_domain_free,
	.on = imx8m_pm_domain_on,
	.off = imx8m_pm_domain_off,
	.of_xlate = imx8m_pm_domain_of_xlate,
};

U_BOOT_DRIVER(imx8m_pm_domain) = {
	.name = "imx8m_pm_domain",
	.id = UCLASS_POWER_DOMAIN,
	.of_match = imx8m_pm_domain_ids,
	.probe = imx8m_pm_domain_probe,
	.ofdata_to_platdata = imx8m_pm_domain_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct imx8m_pm_domain_platdata),
	.ops = &imx8m_pm_domain_ops,
};

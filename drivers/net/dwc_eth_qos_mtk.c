// SPDX-License-Identifier: GPL-2.0

#include <common.h>
#include <clk.h>
#include <cpu_func.h>
#include <dm.h>
#include <errno.h>
#include <eth_phy.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <miiphy.h>
#include <net.h>
#include <netdev.h>
#include <phy.h>
#include <reset.h>
#include <regmap.h>
#include <syscon.h>
#include <wait_bit.h>
#include <asm/cache.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/sys_proto.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>

#include "dwc_eth_qos.h"

/* Peri Configuration register for MTK-specific */
#define EQOS_MTK_PERI_ETH_CTRL0		0xFD0
#define EQOS_MTK_RMII_CLK_SRC_INTERNAL	BIT(28)
#define EQOS_MTK_RMII_CLK_SRC_RXC		BIT(27)
#define EQOS_MTK_ETH_INTF_SEL		GENMASK(26, 24)
#define EQOS_MTK_PHY_INTF_MII		0
#define EQOS_MTK_PHY_INTF_RGMII		1
#define EQOS_MTK_PHY_INTF_RMII		4
#define EQOS_MTK_RGMII_TXC_PHASE_CTRL	BIT(22)
#define EQOS_MTK_EXT_PHY_MODE		BIT(21)
#define EQOS_MTK_DLY_GTXC_INV		BIT(12)
#define EQOS_MTK_DLY_GTXC_ENABLE		BIT(5)
#define EQOS_MTK_DLY_GTXC_STAGES		GENMASK(4, 0)

#define EQOS_MTK_PERI_ETH_CTRL1		0xFD4
#define EQOS_MTK_DLY_RXC_INV		BIT(25)
#define EQOS_MTK_DLY_RXC_ENABLE		BIT(18)
#define EQOS_MTK_DLY_RXC_STAGES		GENMASK(17, 13)
#define EQOS_MTK_DLY_TXC_INV		BIT(12)
#define EQOS_MTK_DLY_TXC_ENABLE		BIT(5)
#define EQOS_MTK_DLY_TXC_STAGES		GENMASK(4, 0)

#define EQOS_MTK_PERI_ETH_CTRL2		0xFD8
#define EQOS_MTK_DLY_RMII_RXC_INV		BIT(25)
#define EQOS_MTK_DLY_RMII_RXC_ENABLE	BIT(18)
#define EQOS_MTK_DLY_RMII_RXC_STAGES	GENMASK(17, 13)
#define EQOS_MTK_DLY_RMII_TXC_INV		BIT(12)
#define EQOS_MTK_DLY_RMII_TXC_ENABLE	BIT(5)
#define EQOS_MTK_DLY_RMII_TXC_STAGES	GENMASK(4, 0)

/* These variables are mtk-specific */
struct eqos_mtk_priv {
	int (*eqos_mtk_config_dt)(struct udevice *dev);
	int (*eqos_mtk_clk_init)(struct udevice *dev);
	int (*eqos_mtk_set_phy_interface)(struct udevice *dev);
	int (*eqos_mtk_delay_stage2ps)(struct udevice *dev);
	int (*eqos_mtk_delay_ps2stage)(struct udevice *dev);
	int (*eqos_mtk_set_delay)(struct udevice *dev);
	struct regmap *peri_regmap;
	phy_interface_t interface;
	bool rmii_clk_from_mac;
	bool rmii_rxc;
	u32 rx_delay_max;
	u32 tx_delay_max;
	u32 tx_delay;
	u32 rx_delay;
	bool tx_inv;
	bool rx_inv;
	/* TODO: remove it when power driver is ready*/
	fdt_addr_t power_regs;
};

static int eqos_start_clks_mtk(struct udevice *dev)
{
#ifdef CONFIG_CLK
	struct eqos_priv *eqos = dev_get_priv(dev);
	int ret;

	debug("%s(dev=%p):\n", __func__, dev);

	ret = clk_enable(&eqos->clk_slave_bus);
	if (ret < 0) {
		pr_err("clk_enable(apb) failed: %d", ret);
		goto err;
	}

	ret = clk_enable(&eqos->clk_master_bus);
	if (ret < 0) {
		pr_err("clk_enable(axi) failed: %d", ret);
		goto err_disable_clk_apb;
	}

	ret = clk_enable(&eqos->clk_rx);
	if (ret < 0) {
		pr_err("clk_enable(rmii_internal) failed: %d", ret);
		goto err_disable_clk_axi;
	}

	ret = clk_enable(&eqos->clk_ptp_ref);
	if (ret < 0) {
		pr_err("clk_enable(ptp_ref) failed: %d", ret);
		goto err_disable_clk_rmii_internal;
	}

	ret = clk_enable(&eqos->clk_tx);
	if (ret < 0) {
		pr_err("clk_enable(mac_main) failed: %d", ret);
		goto err_disable_clk_ptp_ref;
	}

	ret = clk_enable(&eqos->clk_ck);
	if (ret < 0) {
		pr_err("clk_enable(cg) failed: %d", ret);
		goto err_disable_clk_mac_main;
	}

#endif

	debug("%s: OK\n", __func__);
	return 0;

#ifdef CONFIG_CLK
err_disable_clk_mac_main:
	clk_disable(&eqos->clk_tx);
err_disable_clk_ptp_ref:
	clk_disable(&eqos->clk_ptp_ref);
err_disable_clk_rmii_internal:
	clk_disable(&eqos->clk_rx);
err_disable_clk_axi:
	clk_disable(&eqos->clk_master_bus);
err_disable_clk_apb:
	clk_disable(&eqos->clk_slave_bus);
err:
	debug("%s: FAILED: %d\n", __func__, ret);
	return ret;
#endif
}

static int eqos_stop_clks_mtk(struct udevice *dev)
{
#ifdef CONFIG_CLK
	struct eqos_priv *eqos = dev_get_priv(dev);

	debug("%s(dev=%p):\n", __func__, dev);

	clk_disable(&eqos->clk_tx);
	clk_disable(&eqos->clk_ptp_ref);
	clk_disable(&eqos->clk_rx);
	clk_disable(&eqos->clk_master_bus);
	clk_disable(&eqos->clk_slave_bus);
	clk_disable(&eqos->clk_ck);
#endif

	debug("%s: OK\n", __func__);
	return 0;
}

static int eqos_fix_mac_speed_mtk(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct eqos_mtk_priv *mtk_pdata = pdata->priv_pdata;

	switch (mtk_pdata->interface) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_ID:
		if (eqos->phy->speed == SPEED_1000)
			regmap_update_bits(mtk_pdata->peri_regmap,
					   EQOS_MTK_PERI_ETH_CTRL0,
					   EQOS_MTK_RGMII_TXC_PHASE_CTRL |
					   EQOS_MTK_DLY_GTXC_ENABLE |
					   EQOS_MTK_DLY_GTXC_INV |
					   EQOS_MTK_DLY_GTXC_STAGES,
					   EQOS_MTK_RGMII_TXC_PHASE_CTRL);
		else
			mtk_pdata->eqos_mtk_set_delay(dev);
		break;
	default:
		pr_err("%s: dev=%p no need to adjust mac delay\n", __func__, dev);
		break;
	}

	return 0;
}

static int mtk_config_dt(struct udevice *dev)
{	struct eth_pdata *pdata = dev_get_plat(dev);
	struct eqos_mtk_priv *mtk_pdata = pdata->priv_pdata;
	struct ofnode_phandle_args args;
	int ret;

	if (!dev_read_u32(dev, "mediatek,tx-delay-ps", &mtk_pdata->tx_delay)) {
		if (mtk_pdata->tx_delay > mtk_pdata->tx_delay_max) {
			pr_err("%s: dev=%p Invalid TX clock delay: %dps\n",
			       __func__, dev, mtk_pdata->tx_delay);
			return -EINVAL;
		}
	}

	if (!dev_read_u32(dev, "mediatek,rx-delay-ps", &mtk_pdata->rx_delay)) {
		if (mtk_pdata->rx_delay > mtk_pdata->rx_delay_max) {
			pr_err("%s: dev=%p Invalid RX clock delay: %dps\n",
			       __func__, dev, mtk_pdata->rx_delay);
			return -EINVAL;
		}
	}

	mtk_pdata->tx_inv = dev_read_bool(dev, "mediatek,txc-inverse");
	mtk_pdata->rx_inv = dev_read_bool(dev, "mediatek,rxc-inverse");
	mtk_pdata->rmii_clk_from_mac = dev_read_bool(dev, "mediatek,rmii-clk-from-mac");
	mtk_pdata->rmii_rxc = dev_read_bool(dev, "mediatek,rmii-rxc");

	ret = dev_read_phandle_with_args(dev, "mediatek,pericfg", NULL, 0, 0, &args);
	if (ret) {
		pr_err("Failed to get mediatek,pericfg property: %d\n", ret);
		return ret;
	}

	mtk_pdata->peri_regmap = syscon_node_to_regmap(args.node);
	if (IS_ERR(mtk_pdata->peri_regmap)) {
		pr_err("%s: dev=%p Invalid perif_cfg reg\n", __func__, dev);
		return -EINVAL;
	}

	mtk_pdata->interface = dev_read_phy_mode(dev);
	if (mtk_pdata->interface == PHY_INTERFACE_MODE_NA) {
		printf("error: phy-mode is not set\n");
		return -EINVAL;
	}

	return 0;
}

static int mtk_clk_init(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	int ret = 0;

#ifdef CONFIG_CLK
	ret = clk_get_by_name(dev, "axi", &eqos->clk_master_bus);
	if (ret) {
		pr_err("clk_get_by_name(axi) failed: %d", ret);
		goto err_clk_init;
	}

	ret = clk_get_by_name(dev, "rmii_internal", &eqos->clk_rx);
	if (ret) {
		pr_err("clk_get_by_name(rmii_internal) failed: %d", ret);
		goto err_free_clk_axi;
	}

	ret = clk_get_by_name(dev, "mac_main", &eqos->clk_tx);
	if (ret) {
		pr_err("clk_get_by_name(mac_main) failed: %d", ret);
		goto err_free_clk_rmii_internal;
	}

	ret = clk_get_by_name(dev, "mac_cg", &eqos->clk_ck);
	if (ret) {
		pr_err("clk_get_by_name(mac_cg) failed: %d", ret);
		goto err_free_clk_mac_main;
	}

	ret = clk_get_by_name(dev, "apb", &eqos->clk_slave_bus);
	if (ret) {
		pr_err("clk_get_by_name(apb) failed: %d", ret);
		goto err_free_clk_mac_cg;
	}

	ret = clk_get_by_name(dev, "ptp_ref", &eqos->clk_ptp_ref);
	if (ret) {
		pr_err("clk_get_by_name(ptp_ref) failed: %d", ret);
		goto err_free_clk_apb;
	}

	return ret;

err_free_clk_apb:
	clk_free(&eqos->clk_slave_bus);
err_free_clk_mac_cg:
	clk_free(&eqos->clk_ck);
err_free_clk_mac_main:
	clk_free(&eqos->clk_tx);
err_free_clk_rmii_internal:
	clk_free(&eqos->clk_rx);
err_free_clk_axi:
	clk_free(&eqos->clk_master_bus);
err_clk_init:
	pr_err("%s: dev=%p clock init fail\n", __func__, dev);
#endif

	return ret;
}

static int mtk_set_interface(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct eqos_mtk_priv *mtk_pdata = pdata->priv_pdata;
	int rmii_clk_from_mac = mtk_pdata->rmii_clk_from_mac ? EQOS_MTK_RMII_CLK_SRC_INTERNAL : 0;
	int rmii_rxc = mtk_pdata->rmii_rxc ? EQOS_MTK_RMII_CLK_SRC_RXC : 0;
	u32 intf_val = 0;

	/* select phy interface in top control domain */
	switch (mtk_pdata->interface) {
	case PHY_INTERFACE_MODE_MII:
		intf_val |= FIELD_PREP(EQOS_MTK_ETH_INTF_SEL, EQOS_MTK_PHY_INTF_MII);
		break;
	case PHY_INTERFACE_MODE_RMII:
		intf_val |= (rmii_rxc | rmii_clk_from_mac);
		intf_val |= FIELD_PREP(EQOS_MTK_ETH_INTF_SEL, EQOS_MTK_PHY_INTF_RMII);
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_ID:
		intf_val |= FIELD_PREP(EQOS_MTK_ETH_INTF_SEL, EQOS_MTK_PHY_INTF_RGMII);
		break;
	default:
		pr_err("%s: dev=%p phy interface not supported\n", __func__, dev);
		return -EINVAL;
	}

	/* only support external PHY */
	intf_val |= EQOS_MTK_EXT_PHY_MODE;

	regmap_write(mtk_pdata->peri_regmap, EQOS_MTK_PERI_ETH_CTRL0, intf_val);

	return 0;
}

static int mtk_delay_ps2stage(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct eqos_mtk_priv *mtk_pdata = pdata->priv_pdata;

	/* 290ps per stage */
	mtk_pdata->tx_delay /= 290;
	mtk_pdata->rx_delay /= 290;
	return 0;
}

static int mtk_delay_stage2ps(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct eqos_mtk_priv *mtk_pdata = pdata->priv_pdata;

	/* 290ps per stage */
	mtk_pdata->tx_delay *= 290;
	mtk_pdata->rx_delay *= 290;
	return 0;
}

static int mtk_set_delay(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct eqos_mtk_priv *mtk_pdata = pdata->priv_pdata;
	u32 gtxc_delay_val = 0, delay_val = 0, rmii_delay_val = 0;

	mtk_pdata->eqos_mtk_delay_ps2stage(dev);

	switch (mtk_pdata->interface) {
	case PHY_INTERFACE_MODE_MII:
		delay_val |= FIELD_PREP(EQOS_MTK_DLY_TXC_ENABLE, !!mtk_pdata->tx_delay);
		delay_val |= FIELD_PREP(EQOS_MTK_DLY_TXC_STAGES, mtk_pdata->tx_delay);
		delay_val |= FIELD_PREP(EQOS_MTK_DLY_TXC_INV, mtk_pdata->tx_inv);

		delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_ENABLE, !!mtk_pdata->rx_delay);
		delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_STAGES, mtk_pdata->rx_delay);
		delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_INV, mtk_pdata->rx_inv);
		break;
	case PHY_INTERFACE_MODE_RMII:
		if (mtk_pdata->rmii_clk_from_mac) {
			/* case 1: mac provides the rmii reference clock,
			 * and the clock output to TXC pin.
			 * The egress timing can be adjusted by RMII_TXC delay macro circuit.
			 * The ingress timing can be adjusted by RMII_RXC delay macro circuit.
			 */
			rmii_delay_val |= FIELD_PREP(EQOS_MTK_DLY_RMII_TXC_ENABLE,
						     !!mtk_pdata->tx_delay);
			rmii_delay_val |= FIELD_PREP(EQOS_MTK_DLY_RMII_TXC_STAGES,
						     mtk_pdata->tx_delay);
			rmii_delay_val |= FIELD_PREP(EQOS_MTK_DLY_RMII_TXC_INV,
						     mtk_pdata->tx_inv);

			rmii_delay_val |= FIELD_PREP(EQOS_MTK_DLY_RMII_RXC_ENABLE,
						     !!mtk_pdata->rx_delay);
			rmii_delay_val |= FIELD_PREP(EQOS_MTK_DLY_RMII_RXC_STAGES,
						     mtk_pdata->rx_delay);
			rmii_delay_val |= FIELD_PREP(EQOS_MTK_DLY_RMII_RXC_INV,
						     mtk_pdata->rx_inv);
		} else {
			/* case 2: the rmii reference clock is from external phy,
			 * and the property "rmii_rxc" indicates which pin(TXC/RXC)
			 * the reference clk is connected to. The reference clock is a
			 * received signal, so rx_delay/rx_inv are used to indicate
			 * the reference clock timing adjustment
			 */
			if (mtk_pdata->rmii_rxc) {
				/* the rmii reference clock from outside is connected
				 * to RXC pin, the reference clock will be adjusted
				 * by RXC delay macro circuit.
				 */
				delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_ENABLE,
							!!mtk_pdata->rx_delay);
				delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_STAGES,
							mtk_pdata->rx_delay);
				delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_INV,
							mtk_pdata->rx_inv);
			} else {
				/* the rmii reference clock from outside is connected
				 * to TXC pin, the reference clock will be adjusted
				 * by TXC delay macro circuit.
				 */
				delay_val |= FIELD_PREP(EQOS_MTK_DLY_TXC_ENABLE,
							!!mtk_pdata->rx_delay);
				delay_val |= FIELD_PREP(EQOS_MTK_DLY_TXC_STAGES,
							mtk_pdata->rx_delay);
				delay_val |= FIELD_PREP(EQOS_MTK_DLY_TXC_INV,
							mtk_pdata->rx_inv);
			}
		}
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_ID:
		gtxc_delay_val |= FIELD_PREP(EQOS_MTK_DLY_GTXC_ENABLE, !!mtk_pdata->tx_delay);
		gtxc_delay_val |= FIELD_PREP(EQOS_MTK_DLY_GTXC_STAGES, mtk_pdata->tx_delay);
		gtxc_delay_val |= FIELD_PREP(EQOS_MTK_DLY_GTXC_INV, mtk_pdata->tx_inv);

		delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_ENABLE, !!mtk_pdata->rx_delay);
		delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_STAGES, mtk_pdata->rx_delay);
		delay_val |= FIELD_PREP(EQOS_MTK_DLY_RXC_INV, mtk_pdata->rx_inv);

		break;
	default:
		pr_err("%s: dev=%p phy interface not supported\n", __func__, dev);
		return -EINVAL;
	}

	regmap_update_bits(mtk_pdata->peri_regmap,
			   EQOS_MTK_PERI_ETH_CTRL0,
			   EQOS_MTK_RGMII_TXC_PHASE_CTRL |
			   EQOS_MTK_DLY_GTXC_INV |
			   EQOS_MTK_DLY_GTXC_ENABLE |
			   EQOS_MTK_DLY_GTXC_STAGES,
			   gtxc_delay_val);
	regmap_write(mtk_pdata->peri_regmap, EQOS_MTK_PERI_ETH_CTRL1, delay_val);
	regmap_write(mtk_pdata->peri_regmap, EQOS_MTK_PERI_ETH_CTRL2, rmii_delay_val);

	mtk_pdata->eqos_mtk_delay_stage2ps(dev);

	return 0;
}

static struct eqos_mtk_priv mtk_priv_data = {
	.eqos_mtk_config_dt = mtk_config_dt,
	.eqos_mtk_clk_init = mtk_clk_init,
	.eqos_mtk_set_phy_interface = mtk_set_interface,
	.eqos_mtk_delay_stage2ps = mtk_delay_stage2ps,
	.eqos_mtk_delay_ps2stage = mtk_delay_ps2stage,
	.eqos_mtk_set_delay = mtk_set_delay,
	.rx_delay_max = 9800,
	.tx_delay_max = 9800
};

static int eqos_probe_resources_mtk(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct eqos_mtk_priv *mtk_pdata;
	int ret;

	pdata->priv_pdata = (void *)(&mtk_priv_data);
	mtk_pdata = &mtk_priv_data;

	debug("%s(dev=%p):\n", __func__, dev);

	ret = mtk_pdata->eqos_mtk_config_dt(dev);
	if (ret) {
		pr_err("%s: dev=%p mtk config dt fail\n", __func__, dev);
		return ret;
	}

	ret = mtk_pdata->eqos_mtk_clk_init(dev);
	if (ret) {
		pr_err("%s: dev=%p mtk config dt fail\n", __func__, dev);
		return ret;
	}

	ret = mtk_pdata->eqos_mtk_set_phy_interface(dev);
	if (ret) {
		pr_err("%s: dev=%p mtk set interface fail\n", __func__, dev);
		return ret;
	}

	ret = mtk_pdata->eqos_mtk_set_delay(dev);
	if (ret) {
		pr_err("%s: dev=%p mtk set delay fail\n", __func__, dev);
		return ret;
	}

	/* TODO: remove it when power driver is ready
	 * enable power of ethernet module by write register directly
	 */
	mtk_pdata->power_regs = dev_read_addr_index(dev, 1);
	if (mtk_pdata->power_regs == FDT_ADDR_T_NONE) {
		pr_err("dev_read_addr() gpio_regs failed");
		return -ENODEV;
	}
#ifdef CONFIG_MT8195
	writel(0xd, mtk_pdata->power_regs + 0x344);
#elif defined(CONFIG_MT8188)
	writel(0xd, mtk_pdata->power_regs + 0x338);
#endif
	debug("%s(dev=%p): OK\n", __func__, dev);

	return 0;
}

static int eqos_remove_resources_mtk(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);

	debug("%s(dev=%p):\n", __func__, dev);

#ifdef CONFIG_CLK
	clk_free(&eqos->clk_tx);
	clk_free(&eqos->clk_ptp_ref);
	clk_free(&eqos->clk_rx);
	clk_free(&eqos->clk_slave_bus);
	clk_free(&eqos->clk_master_bus);
	clk_free(&eqos->clk_ck);
#endif

	debug("%s: OK\n", __func__);
	return 0;
}

static phy_interface_t eqos_get_interface_mtk(const struct udevice *dev)
{
	phy_interface_t interface = PHY_INTERFACE_MODE_NA;

	debug("%s(dev=%p):\n", __func__, dev);

	interface = dev_read_phy_mode(dev);

	return interface;
}

static ulong eqos_get_tick_clk_rate_mtk(struct udevice *dev)
{
#ifdef CONFIG_CLK
	struct eqos_priv *eqos = dev_get_priv(dev);

	return clk_get_rate(&eqos->clk_master_bus);
#else
	return 0;
#endif
}

static struct eqos_ops eqos_mtk_ops = {
	.eqos_inval_desc = eqos_inval_desc_generic,
	.eqos_flush_desc = eqos_flush_desc_generic,
	.eqos_inval_buffer = eqos_inval_buffer_generic,
	.eqos_flush_buffer = eqos_flush_buffer_generic,
	.eqos_probe_resources = eqos_probe_resources_mtk,
	.eqos_remove_resources = eqos_remove_resources_mtk,
	.eqos_stop_resets = eqos_null_ops,
	.eqos_start_resets = eqos_null_ops,
	.eqos_stop_clks = eqos_stop_clks_mtk,
	.eqos_start_clks = eqos_start_clks_mtk,
	.eqos_calibrate_pads = eqos_null_ops,
	.eqos_disable_calibration = eqos_null_ops,
	.eqos_set_tx_clk_speed = eqos_null_ops,
	.eqos_get_enetaddr = eqos_null_ops,
	.eqos_get_tick_clk_rate = eqos_get_tick_clk_rate_mtk,
	.eqos_fix_mac_speed = eqos_fix_mac_speed_mtk,
};

struct eqos_config eqos_mtk_config = {
	.reg_access_always_ok = false,
	.mdio_wait = 10000,
	.swr_wait = 10,
	.config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB,
	.config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_60_100,
	.axi_bus_width = EQOS_AXI_WIDTH_64,
	.interface = eqos_get_interface_mtk,
	.ops = &eqos_mtk_ops
};

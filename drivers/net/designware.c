/*
 * (C) Copyright 2010
 * Vipin Kumar, ST Micoelectronics, vipin.kumar@st.com.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * Designware ethernet IP driver for u-boot
 */

#include <common.h>
#include <miiphy.h>
#include <malloc.h>
#include <linux/compiler.h>
#include <linux/err.h>
#include <asm/io.h>
#include <asm/arch/io.h>
#include "designware.h"
#include <asm/arch/secure_apb.h>
#include <amlogic/keyunify.h>
#include <version.h>

#if !defined(CONFIG_PHYLIB)
# error "DesignWare Ether MAC requires PHYLIB - missing CONFIG_PHYLIB"
#endif

struct eth_device *dev = NULL;
struct dw_eth_dev *priv = NULL;
int do_cali_process = 0;
int do_cali_timeout = 0;

static int dw_mdio_read(struct mii_dev *bus, int addr, int devad, int reg)
{
	struct eth_mac_regs *mac_p = bus->priv;
	ulong start;
	u16 miiaddr;
	int timeout = CONFIG_MDIO_TIMEOUT;

	miiaddr = ((addr << MIIADDRSHIFT) & MII_ADDRMSK) |
		  ((reg << MIIREGSHIFT) & MII_REGMSK);

	writel(miiaddr | MII_CLKRANGE_150_250M | MII_BUSY, &mac_p->miiaddr);

	start = get_timer(0);
	while (get_timer(start) < timeout) {
		if (!(readl(&mac_p->miiaddr) & MII_BUSY))
			return readl(&mac_p->miidata);
		udelay(10);
	};

	return -1;
}

static int dw_mdio_write(struct mii_dev *bus, int addr, int devad, int reg,
			u16 val)
{
	struct eth_mac_regs *mac_p = bus->priv;
	ulong start;
	u16 miiaddr;
	int ret = -1, timeout = CONFIG_MDIO_TIMEOUT;

	writel(val, &mac_p->miidata);
	miiaddr = ((addr << MIIADDRSHIFT) & MII_ADDRMSK) |
		  ((reg << MIIREGSHIFT) & MII_REGMSK) | MII_WRITE;

	writel(miiaddr | MII_CLKRANGE_150_250M | MII_BUSY, &mac_p->miiaddr);

	start = get_timer(0);
	while (get_timer(start) < timeout) {
		if (!(readl(&mac_p->miiaddr) & MII_BUSY)) {
			ret = 0;
			break;
		}
		udelay(10);
	};

	return ret;
}

static int dw_mdio_init(char *name, struct eth_mac_regs *mac_regs_p)
{
	struct mii_dev *bus = mdio_alloc();

	if (!bus) {
		printf("Failed to allocate MDIO bus\n");
		return -1;
	}

	bus->read = dw_mdio_read;
	bus->write = dw_mdio_write;
	sprintf(bus->name, name);

	bus->priv = (void *)mac_regs_p;

	return mdio_register(bus);
}

static void tx_descs_init(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	struct dmamacdescr *desc_table_p = &priv->tx_mac_descrtable[0];
	char *txbuffs = &priv->txbuffs[0];
	struct dmamacdescr *desc_p;
	u32 idx;

	for (idx = 0; idx < CONFIG_TX_DESCR_NUM; idx++) {
		desc_p = &desc_table_p[idx];
		desc_p->dmamac_addr = (u32)(phys_addr_t)&txbuffs[idx * CONFIG_ETH_BUFSIZE];
		desc_p->dmamac_next = (u32)(phys_addr_t)&desc_table_p[idx + 1];

#if defined(CONFIG_DW_ALTDESCRIPTOR)
		desc_p->txrx_status &= ~(DESC_TXSTS_TXINT | DESC_TXSTS_TXLAST |
				DESC_TXSTS_TXFIRST | DESC_TXSTS_TXCRCDIS | \
				DESC_TXSTS_TXCHECKINSCTRL | \
				DESC_TXSTS_TXRINGEND | DESC_TXSTS_TXPADDIS);

		desc_p->txrx_status |= DESC_TXSTS_TXCHAIN;
		desc_p->dmamac_cntl = 0;
		desc_p->txrx_status &= ~(DESC_TXSTS_MSK | DESC_TXSTS_OWNBYDMA);
#else
		desc_p->dmamac_cntl = DESC_TXCTRL_TXCHAIN;
		desc_p->txrx_status = 0;
#endif
	}

	/* Correcting the last pointer of the chain */
	desc_p->dmamac_next = (u32)(phys_addr_t)&desc_table_p[0];

	/* Flush all Tx buffer descriptors at once */
	flush_dcache_range((phys_addr_t)priv->tx_mac_descrtable,
			   (phys_addr_t)priv->tx_mac_descrtable +
			   sizeof(priv->tx_mac_descrtable));

	writel((ulong)&desc_table_p[0], &dma_p->txdesclistaddr);
	priv->tx_currdescnum = 0;
}

static void rx_descs_init(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	struct dmamacdescr *desc_table_p = &priv->rx_mac_descrtable[0];
	char *rxbuffs = &priv->rxbuffs[0];
	struct dmamacdescr *desc_p;
	u32 idx;

	/* Before passing buffers to GMAC we need to make sure zeros
	 * written there right after "priv" structure allocation were
	 * flushed into RAM.
	 * Otherwise there's a chance to get some of them flushed in RAM when
	 * GMAC is already pushing data to RAM via DMA. This way incoming from
	 * GMAC data will be corrupted. */
	flush_dcache_range((phys_addr_t)rxbuffs, (phys_addr_t)rxbuffs +
			   RX_TOTAL_BUFSIZE);

	for (idx = 0; idx < CONFIG_RX_DESCR_NUM; idx++) {
		desc_p = &desc_table_p[idx];
		desc_p->dmamac_addr = (u32)(phys_addr_t)&rxbuffs[idx * CONFIG_ETH_BUFSIZE];
		desc_p->dmamac_next = (u32)(phys_addr_t)&desc_table_p[idx + 1];

		desc_p->dmamac_cntl =
			(MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) | \
				      DESC_RXCTRL_RXCHAIN;

		desc_p->txrx_status = DESC_RXSTS_OWNBYDMA;
	}

	/* Correcting the last pointer of the chain */
	desc_p->dmamac_next = (u32)(phys_addr_t)&desc_table_p[0];

	/* Flush all Rx buffer descriptors at once */
	flush_dcache_range((phys_addr_t)priv->rx_mac_descrtable,
			   (phys_addr_t)priv->rx_mac_descrtable +
			   sizeof(priv->rx_mac_descrtable));

	writel((ulong)&desc_table_p[0], &dma_p->rxdesclistaddr);
	priv->rx_currdescnum = 0;
}

static int dw_write_hwaddr(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	u32 macid_lo, macid_hi;
	u8 *mac_id = &dev->enetaddr[0];

	macid_lo = mac_id[0] + (mac_id[1] << 8) + (mac_id[2] << 16) +
		   (mac_id[3] << 24);
	macid_hi = mac_id[4] + (mac_id[5] << 8);

	writel(macid_hi, &mac_p->macaddr0hi);
	writel(macid_lo, &mac_p->macaddr0lo);

	return 0;
}

static void dw_adjust_link(struct eth_mac_regs *mac_p,
			   struct phy_device *phydev)
{
	u32 conf = readl(&mac_p->conf) | FRAMEBURSTENABLE | DISABLERXOWN;

	if (!phydev->link) {
		printf("%s: No link.\n", phydev->dev->name);
		return;
	}

	if (phydev->speed != 1000)
		conf |= MII_PORTSELECT;

	if (phydev->speed == 100)
		conf |= FES_100;

	if (phydev->duplex)
		conf |= FULLDPLXMODE;

	writel(conf, &mac_p->conf);

	if (!do_cali_process) {
		printf("Speed: %d, %s duplex%s\n", phydev->speed,
			(phydev->duplex) ? "full" : "half",
			(phydev->port == PORT_FIBRE) ? ", fiber mode" : "");
	}
}

static void dw_eth_halt(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;

	writel(readl(&mac_p->conf) & ~(RXENABLE | TXENABLE), &mac_p->conf);
	writel(readl(&dma_p->opmode) & ~(RXSTART | TXSTART), &dma_p->opmode);

	phy_shutdown(priv->phydev);
}

static int dw_eth_init(struct eth_device *dev, bd_t *bis)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	unsigned int start;

	writel(readl(&dma_p->busmode) | DMAMAC_SRST, &dma_p->busmode);

	start = get_timer(0);
	while (readl(&dma_p->busmode) & DMAMAC_SRST) {
		if (get_timer(start) >= CONFIG_MACRESET_TIMEOUT)
			return -1;
#ifdef CONFIG_PXP_EMULATOR
		udelay(100);
#else
		mdelay(100);
#endif

	};

	/* Soft reset above clears HW address registers.
	 * So we have to set it here once again */
	dw_write_hwaddr(dev);

	rx_descs_init(dev);
	tx_descs_init(dev);

	writel(FIXEDBURST | PRIORXTX_41 | DMA_PBL, &dma_p->busmode);

	writel(readl(&dma_p->opmode) | FLUSHTXFIFO | STOREFORWARD,
	       &dma_p->opmode);

	writel(readl(&dma_p->opmode) | RXSTART | TXSTART, &dma_p->opmode);

	/* Start up the PHY */
	if (phy_startup(priv->phydev)) {
		printf("Could not initialize PHY %s\n",
		       priv->phydev->dev->name);
		return -1;
	}
#ifdef CONFIG_PXP_EMULATOR
	priv->phydev->link = 1;
	priv->phydev->speed = 100;
	priv->phydev->duplex = 1;
#endif
	dw_adjust_link(mac_p, priv->phydev);

	if (!priv->phydev->link)
		return -1;

	writel(readl(&mac_p->conf) | RXENABLE | TXENABLE, &mac_p->conf);

	return 0;
}

static int dw_eth_send(struct eth_device *dev, void *packet, int length)
{
	struct dw_eth_dev *priv = dev->priv;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	u32 desc_num = priv->tx_currdescnum;
	struct dmamacdescr *desc_p = &priv->tx_mac_descrtable[desc_num];
	phys_addr_t desc_start = (phys_addr_t)desc_p;
	phys_addr_t desc_end = desc_start +
		roundup(sizeof(*desc_p), ARCH_DMA_MINALIGN);
	/*
	 * Strictly we only need to invalidate the "txrx_status" field
	 * for the following check, but on some platforms we cannot
	 * invalidate only 4 bytes, so we flush the entire descriptor,
	 * which is 16 bytes in total. This is safe because the
	 * individual descriptors in the array are each aligned to
	 * ARCH_DMA_MINALIGN and padded appropriately.
	 */
	invalidate_dcache_range(desc_start, desc_end);

	/* Check if the descriptor is owned by CPU */
	if (desc_p->txrx_status & DESC_TXSTS_OWNBYDMA) {
		printf("CPU not owner of tx frame\n");
		return -1;
	}

	memcpy((void *)(phys_addr_t)desc_p->dmamac_addr, packet, length);
	/* Flush data to be sent */
	flush_dcache_range((phys_addr_t)priv->txbuffs, (phys_addr_t)priv->txbuffs + TX_TOTAL_BUFSIZE);

#if defined(CONFIG_DW_ALTDESCRIPTOR)
	desc_p->txrx_status |= DESC_TXSTS_TXFIRST | DESC_TXSTS_TXLAST;
	desc_p->dmamac_cntl &= (~DESC_TXCTRL_SIZE1MASK);
	desc_p->dmamac_cntl |= (length << DESC_TXCTRL_SIZE1SHFT) & \
			       DESC_TXCTRL_SIZE1MASK;

	desc_p->txrx_status &= ~(DESC_TXSTS_MSK);
	desc_p->txrx_status |= DESC_TXSTS_OWNBYDMA;
#else
	desc_p->dmamac_cntl &= (~DESC_TXCTRL_SIZE1MASK);
	desc_p->dmamac_cntl |= ((length << DESC_TXCTRL_SIZE1SHFT) & \
			       DESC_TXCTRL_SIZE1MASK) | DESC_TXCTRL_TXLAST | \
			       DESC_TXCTRL_TXFIRST;

	desc_p->txrx_status = DESC_TXSTS_OWNBYDMA;
#endif

	/* Flush modified buffer descriptor */
	flush_dcache_range(desc_start, desc_end);

	/* Test the wrap-around condition. */
	if (++desc_num >= CONFIG_TX_DESCR_NUM)
		desc_num = 0;

	priv->tx_currdescnum = desc_num;

	/* Start the transmission */
	writel(POLL_DATA, &dma_p->txpolldemand);

	return 0;
}

static int dw_eth_recv(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	u32 status, desc_num = priv->rx_currdescnum;
	struct dmamacdescr *desc_p = &priv->rx_mac_descrtable[desc_num];
	int length = 0;
	phys_addr_t desc_start = (phys_addr_t)desc_p;
	phys_addr_t desc_end = desc_start +
	roundup(sizeof(*desc_p), ARCH_DMA_MINALIGN);
	/* Invalidate entire buffer descriptor */
	invalidate_dcache_range(desc_start, desc_end);

	status = desc_p->txrx_status;

	/* Check  if the owner is the CPU */
	if (!(status & DESC_RXSTS_OWNBYDMA)) {
		length = (status & DESC_RXSTS_FRMLENMSK) >> \
			 DESC_RXSTS_FRMLENSHFT;

		/* Invalidate received data */
		invalidate_dcache_range((phys_addr_t)priv->rxbuffs, (phys_addr_t)priv->rxbuffs + RX_TOTAL_BUFSIZE);

		NetReceive((uchar *)(phys_addr_t)desc_p->dmamac_addr, length);
		/*
		 * Make the current descriptor valid again and go to
		 * the next one
		 */
		desc_p->txrx_status |= DESC_RXSTS_OWNBYDMA;

		/* Flush only status field - others weren't changed */
		flush_dcache_range(desc_start, desc_end);

		/* Test the wrap-around condition. */
		if (++desc_num >= CONFIG_RX_DESCR_NUM)
			desc_num = 0;
	}

	priv->rx_currdescnum = desc_num;

	return length;
}

static int dw_phy_init(struct eth_device *dev)
{
	struct dw_eth_dev *priv = dev->priv;
	struct phy_device *phydev;
	int mask = 0xffffffff;

#ifdef CONFIG_PHY_ADDR
	mask = 1 << CONFIG_PHY_ADDR;
#endif

	phydev = phy_find_by_mask(priv->bus, mask, priv->interface);
	if (!phydev)
		return -1;

	phy_connect_dev(phydev, dev);

	phydev->supported &= PHY_GBIT_FEATURES;
	phydev->advertising = phydev->supported;

	priv->phydev = phydev;
	phy_config(phydev);

	return 1;
}

#ifdef ETHERNET_EXTERNAL_PHY
int check_eth_para(void);
char bestwindow = -1;
char cmd[64];
#endif
int designware_initialize(ulong base_addr, u32 interface)
{
	int ret;
	dev = (struct eth_device *) malloc(sizeof(struct eth_device));
	if (!dev)
		return -ENOMEM;

	/*
	 * Since the priv structure contains the descriptors which need a strict
	 * buswidth alignment, memalign is used to allocate memory
	 */
	priv = (struct dw_eth_dev *) memalign(ARCH_DMA_MINALIGN,
					      sizeof(struct dw_eth_dev));
	if (!priv) {
		free(dev);
		return -ENOMEM;
	}

	memset(dev, 0, sizeof(struct eth_device));
	memset(priv, 0, sizeof(struct dw_eth_dev));

	sprintf(dev->name, "dwmac.%lx", base_addr);
	dev->iobase = (int)base_addr;
	dev->priv = priv;

	priv->dev = dev;
	priv->mac_regs_p = (struct eth_mac_regs *)base_addr;
	priv->dma_regs_p = (struct eth_dma_regs *)(base_addr +
			DW_DMA_BASE_OFFSET);

	dev->init = dw_eth_init;
	dev->send = dw_eth_send;
	dev->recv = dw_eth_recv;
	dev->halt = dw_eth_halt;
	dev->write_hwaddr = dw_write_hwaddr;

	eth_register(dev);

	priv->interface = interface;

	dw_mdio_init(dev->name, priv->mac_regs_p);
	priv->bus = miiphy_get_dev_by_name(dev->name);
	ret = dw_phy_init(dev);

#ifdef ETHERNET_EXTERNAL_PHY
	if (check_eth_para()) {
		run_command("autocali 5 1 1 0", 0);
	}
	if (bestwindow >= 0) {
		sprintf(cmd, "fdt set /ethernet@%08x auto_cali_idx <%d>", (unsigned int)base_addr, bestwindow);
		run_command("fdt addr $dtb_mem_addr", 0);
		run_command(cmd, 0);
	}
#endif
	return ret;
}

/* amlogic debug cmd start */
static int do_phyreg(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int reg, value;
	unsigned char *cmd = NULL;
	unsigned int i;

	if (argc < 2) {
		return cmd_usage(cmdtp);
	}

	if (priv == NULL || priv->phydev == NULL) {
		return -1;
	}

	cmd = (unsigned char *)argv[1];
	switch (*cmd) {
		case 'd':
			printf("=== ethernet phy register dump:\n");
			for (i = 0; i < 32; i++)
				printf("[reg_%d] 0x%x\n", i, phy_read(priv->phydev, MDIO_DEVAD_NONE, i));
			break;
		case 'r':
			if (argc != 3) {
				return cmd_usage(cmdtp);
			}
			printf("=== ethernet phy register read:\n");
			reg = simple_strtoul(argv[2], NULL, 10);
			printf("[reg_%d] 0x%x\n", reg, phy_read(priv->phydev, MDIO_DEVAD_NONE, reg));

			break;
		case 'w':
			if (argc != 4) {
				return cmd_usage(cmdtp);
			}
			printf("=== ethernet phy register write:\n");
			reg = simple_strtoul(argv[2], NULL, 10);
			value = simple_strtoul(argv[3], NULL, 16);
			phy_write(priv->phydev, MDIO_DEVAD_NONE, reg, value);
			printf("[reg_%d] 0x%x\n", reg, phy_read(priv->phydev, MDIO_DEVAD_NONE, reg));
			break;

		default:
			return cmd_usage(cmdtp);
	}

	return 0;
}

static int do_macreg(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int reg, value;
	unsigned char *cmd = NULL;
	unsigned int i = 0;

	if (argc  < 2) {
		return cmd_usage(cmdtp);
	}

	cmd = (unsigned char *)argv[1];
	switch (*cmd) {
		case 'd':
			printf("=== ETH_MAC register dump:\n");
			for (i = 0x0000; i <= 0x004C; i += 0x4)
				printf("[0x%04x] 0x%lx\n", i, (unsigned long)readl((unsigned long)priv->mac_regs_p + i));

			printf("=== ETH_DMA register dump:\n");
			for (i = 0x0000; i <= 0x0054; i += 0x4)
				printf("[0x%04x] 0x%x\n", i, (unsigned int)readl((unsigned long)priv->dma_regs_p + i));

			break;
		case 'r':
			if (argc != 3) {
				return cmd_usage(cmdtp);
			}
			printf("=== ethernet mac register read:\n");
			reg = simple_strtoul(argv[2], NULL, 10);
			printf("[0x%04x] 0x%x\n", i, (unsigned int)readl((unsigned long)priv->mac_regs_p + reg));

			break;
		case 'w':
			if (argc != 4) {
				return cmd_usage(cmdtp);
			}
			printf("=== ethernet mac register write:\n");
			reg = simple_strtoul(argv[2], NULL, 10);
			value = simple_strtoul(argv[3], NULL, 16);
			writel(value, (unsigned long)priv->mac_regs_p + reg);
			printf("[0x%04x] 0x%x\n", reg, value);
			break;

		default:
			return cmd_usage(cmdtp);
	}

	return 0;
}

static int do_cbusreg(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int reg, value;
	char *cmd = NULL;


	if (argc < 3) {
		return cmd_usage(cmdtp);
	}

	cmd = argv[1];
	switch (*cmd) {
		case 'r':
			if (argc != 3) {
				return cmd_usage(cmdtp);
			}
			printf("=== cbus register read:\n");
			reg = simple_strtoul(argv[2], NULL, 16);
			printf("[0x%04x] 0x%x\n", reg, READ_CBUS_REG(reg));

			break;
		case 'w':
			if (argc != 4) {
				return cmd_usage(cmdtp);
			}
			printf("=== cbus register write:\n");
			reg = simple_strtoul(argv[2], NULL, 16);
			value = simple_strtoul(argv[3], NULL, 16);
			WRITE_CBUS_REG(reg, value);
			printf("[0x%04x] 0x%x\n", reg, READ_CBUS_REG(reg));
			break;

		default:
			return cmd_usage(cmdtp);
	}

	return 0;
}
#ifdef ETHERNET_EXTERNAL_PHY
int print_flag = 0;

#define ETH_MAGIC "exphy:"
struct unify_eth_info {
	char magic[6];
	char index;
	char chk;
	char ver[24];
};

int check_eth_para(void)
{
	struct unify_eth_info unify_eth;
	int ret;

	ret = key_unify_read("eth_exphy_para", &unify_eth, sizeof(struct unify_eth_info));
	if (ret)
		return ret;

	if (strncmp(unify_eth.magic, ETH_MAGIC, 6) != 0)
		return -1;

	if (unify_eth.index + unify_eth.chk !=0xff)
		return -1;

	if (strcmp(unify_eth.ver, PLAIN_VERSION) != 0)
		return -1;

	bestwindow = unify_eth.index;
	return 0;
}

int calc_result(u16* calilist)
{
	int best_window = -1;
	int start_window = -1;
	int end_window = -1;
	int cur_test_start_window = -1;
	int cur_window;
	int i,j;
	u16 cali;

	for (i=0; i<4; i++) {
		cali = calilist[i];
		cur_test_start_window = -1;
		for (j=0; j<16; j++) {
			cur_window = (i * 16) +j;
			if (cali & (1<<j)) {
				if (cur_test_start_window == -1)
					cur_test_start_window = cur_window;

				if (j == 15) {
					if (cur_window - cur_test_start_window >= end_window - start_window) {
						start_window = cur_test_start_window;
						end_window = cur_window;
						if (print_flag)
							printf("cur: %d, start:%d, end:%d\n", cur_window,start_window,end_window);
					}
				}
			} else {
				if (cur_test_start_window != -1) {
					if (cur_window-1 - cur_test_start_window >= end_window - start_window) {
						start_window = cur_test_start_window;
						end_window = cur_window-1;
						if (print_flag)
							printf("cur: %d, start:%d, end:%d\n", cur_window,start_window,end_window);
					}
					cur_test_start_window = -1;
				}
			}
		}
	}

	if (print_flag)
		printf("final start:%d, end:%d\n", start_window,end_window);
	if ((start_window == -1) || (end_window == -1))
		best_window = -1;
	else {
		best_window = (start_window + end_window)/2;
		if (((end_window & 0xf) == 0xf) && ((best_window & 0xf) != 0xf))
			best_window += 1;
	}
	return best_window;
}

static int do_autocali(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[])
{
	unsigned int i,j /*, valid = 0*/;
	unsigned int loop_num;
	unsigned int loop_type;
	u16 reg = 0;
	u16 calilist[4];
	int cali_window;
	struct unify_eth_info unify_eth;

	if (argc < 5) {
		return cmd_usage(cmdtp);
	}
	loop_num = simple_strtoul(argv[1], NULL, 10);
	loop_type = simple_strtoul(argv[2], NULL, 10);
	print_flag = simple_strtoul(argv[4], NULL, 10);
	do_cali_timeout = simple_strtoul(argv[3], NULL, 10);
	do_cali_process = 1;
	for (i=0; i<4; i++) {
		calilist[i] = 0;
	}
	if (loop_type) {
		phy_write(priv->phydev, MDIO_DEVAD_NONE, 0, 0x5040);
		mdelay(2);
	} else {
		//MDI 1000M loopback
		phy_write(priv->phydev, MDIO_DEVAD_NONE, 31, 0x0a43);
		phy_write(priv->phydev, MDIO_DEVAD_NONE, 0, 0x8000);
		mdelay(40);
		phy_write(priv->phydev, MDIO_DEVAD_NONE, 0, 0x1140);
		phy_write(priv->phydev, MDIO_DEVAD_NONE, 24, 0x2d18);
		mdelay(400);
	}
	if (print_flag)
		printf("----------normal----------\n");

	phy_write(priv->phydev, MDIO_DEVAD_NONE,31, 0xd08);
	reg = phy_read(priv->phydev, MDIO_DEVAD_NONE,0x11);
	reg = phy_write(priv->phydev, MDIO_DEVAD_NONE, 0x11, reg & (~0x100));
	reg = phy_read(priv->phydev, MDIO_DEVAD_NONE,0x15);
	reg = phy_write(priv->phydev, MDIO_DEVAD_NONE, 0x15, reg & (~0x8));
	phy_write(priv->phydev, MDIO_DEVAD_NONE, 31, 0x0);
	writel(0x1621, 0xff634540);
	for (i = 0; i < 16; i++) {
		writel(i << 16, 0xff634544);
		if (print_flag)
			printf("0x%05x\n", i << 16);
		//mdelay(20);
		for (j=0; j<loop_num; j++) {
			if (NetLoop(ETHLOOP) < 0) {
				if (print_flag)
					printf("-------------failed\n\n");
				calilist[0] &=(~(1<<i));
				break;
			} else {
				if (print_flag)
					printf("-------------success\n\n");
				calilist[0] |=(1<<i);
			}
			mdelay(1);
		}
	}
	if (print_flag)
		printf("----------invert----------\n");
	writel(0x1629, 0xff634540);
	for (i = 0; i < 16; i++) {
		writel(i << 16, 0xff634544);
		if (print_flag)
			printf("0x%05x\n", i << 16);
		//mdelay(20);
		for (j=0; j<loop_num; j++) {
			if (NetLoop(ETHLOOP) < 0) {
				if (print_flag)
					printf("-------------failed\n\n");
				calilist[1] &=(~(1<<i));
				break;
			} else {
				if (print_flag)
					printf("-------------success\n\n");
				calilist[1] |=(1<<i);
			}
			mdelay(1);
		}
	}

	/*add 2ns delay and invert clk*/
	/*reset exphy to add 2ns delay*/
	phy_write(priv->phydev, MDIO_DEVAD_NONE,31, 0xd08);
	reg = phy_read(priv->phydev, MDIO_DEVAD_NONE,0x15);
	reg = phy_write(priv->phydev, MDIO_DEVAD_NONE, 0x15, reg | 0x8);
	phy_write(priv->phydev, MDIO_DEVAD_NONE, 31, 0x0);

	/*inverte clk*/
	writel(0x1629, 0xff634540);

	if (print_flag)
		printf("----invert && add 2ns-----\n");
	for (i = 0; i < 16; i++) {
		writel(i << 16, 0xff634544);
		if (print_flag)
			printf("0x%05x\n", i << 16);
		//mdelay(20);
		for (j=0;j<loop_num;j++) {
			if (NetLoop(ETHLOOP) < 0) {
				if (print_flag)
					printf("-------------failed\n\n");
				calilist[2] &=(~(1<<i));
				break;
			} else {
				if (print_flag)
					printf("-------------success\n\n");
				calilist[2] |=(1<<i);
			}
			mdelay(1);
		}
	}

	if (print_flag)
		printf("----normal && add 2ns-----\n");
	writel(0x1621, 0xff634540);
	for (i = 0; i < 16; i++) {
		writel(i << 16, 0xff634544);
		if (print_flag)
			printf("0x%05x\n", i << 16);
		//mdelay(20);
		for (j=0; j<loop_num; j++) {
			if (NetLoop(ETHLOOP) < 0) {
				if (print_flag)
					printf("-------------failed\n\n");
				calilist[3] &=(~(1<<i));
				break;
			} else {
				if (print_flag)
					printf("-------------success\n\n");
				calilist[3] |=(1<<i);
			}
			mdelay(1);
		}
	}

	if (print_flag)
		printf("result:	%04x\t%04x\t%04x\t%04x\n", calilist[0], calilist[1], calilist[2], calilist[3]);

	cali_window = calc_result(calilist);
	printf("The Best Window is index %d\n", cali_window);

	strncpy(unify_eth.magic, ETH_MAGIC, 6);
	unify_eth.index = cali_window;
	unify_eth.chk = 0xff - unify_eth.index;
	strcpy(unify_eth.ver, PLAIN_VERSION);

	key_unify_write("eth_exphy_para", &unify_eth, sizeof(struct unify_eth_info));
	bestwindow = unify_eth.index;
	do_cali_process = 0;
	return 0;
}

#endif

U_BOOT_CMD(
		phyreg, 4, 1, do_phyreg,
		"ethernet phy register read/write/dump",
		"d            - dump phy registers\n"
		"       r reg        - read phy register\n"
		"       w reg val    - write phy register"
		);

U_BOOT_CMD(
		macreg, 4, 1, do_macreg,
		"ethernet mac register read/write/dump",
		"d            - dump mac registers\n"
		"       r reg        - read mac register\n"
		"       w reg val    - write mac register"
		);

U_BOOT_CMD(
		cbusreg, 4, 1, do_cbusreg,
		"cbus register read/write",
		"r reg        - read cbus register\n"
		"        w reg val    - write cbus register"
		);
#ifdef ETHERNET_EXTERNAL_PHY
U_BOOT_CMD(
	autocali,	5,	1,	do_autocali,
	"auto cali\t- auto set cali value for exphy\n",
	"loopcnt type timeout flag\n"
	"		loopcnt	- loop package count\n"
	"		type	- 0: pcs loop 1: phy loop\n"
	"		timeout	- timeout (ms)\n"
	"		flag	- print flag\n"
);
#endif
/* amlogic debug cmd end */

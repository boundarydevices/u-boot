// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2013-2015 Freescale Semiconductor, Inc.
 *
 * Freescale Quad Serial Peripheral Interface (QSPI) driver
 */

#include <asm/arch/clock.h>
#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <dm.h>
#include <errno.h>
#include <watchdog.h>
#include <wait_bit.h>
#include "fsl_qspi.h"
#include <asm/arch/sys_proto.h>

DECLARE_GLOBAL_DATA_PTR;

#define RX_BUFFER_SIZE		0x80
#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6UL) || \
	defined(CONFIG_MX6ULL) || defined(CONFIG_MX7D)
#define TX_BUFFER_SIZE		0x200
#else
#define TX_BUFFER_SIZE		0x40
#endif

#define OFFSET_BITS_MASK	GENMASK(23, 0)

#define FLASH_STATUS_BUSY	0x01
#define FLASH_STATUS_WEL	0x02

/* SEQID */
#define SEQID_WREN		1
#define SEQID_FAST_READ		2
#define SEQID_RDSR		3
#define SEQID_SE		4
#define SEQID_CHIP_ERASE	5
#define SEQID_PP		6
#define SEQID_RDID		7
#define SEQID_BE_4K		8

#ifdef CONFIG_SPI_FLASH_BAR
#define SEQID_BRRD		9
#define SEQID_BRWR		10
#define SEQID_RDEAR		11
#define SEQID_WREAR		12
#else
#define SEQID_WRITE_STATUS	9
#define SEQID_SST_AAI_WP	10
#define SEQID_SST_AAI_WP_CONT	11
#define SEQID_WRITE_DISABLE	12
#endif
#define SEQID_WRAR		13
#define SEQID_RDAR		14

/* QSPI CMD */
#define QSPI_CMD_WRITE_STATUS	0x01	/* Write status register */
#define QSPI_CMD_WRITE_DISABLE	0x04	/* exit autoinc write */
#define QSPI_CMD_SST_AAI_WP	0xad	/* Auto Address Incr Word Program(sst specific) */
#define QSPI_CMD_PP		0x02	/* Page program (up to 256 bytes) */
#define QSPI_CMD_RDSR		0x05	/* Read status register */
#define QSPI_CMD_WREN		0x06	/* Write enable */
#define QSPI_CMD_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define QSPI_CMD_BE_4K		0x20    /* 4K erase */
#define QSPI_CMD_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define QSPI_CMD_SE		0xd8	/* Sector erase (usually 64KiB) */
#define QSPI_CMD_RDID		0x9f	/* Read JEDEC ID */

/* Used for Micron, winbond and Macronix flashes */
#define	QSPI_CMD_WREAR		0xc5	/* EAR register write */
#define	QSPI_CMD_RDEAR		0xc8	/* EAR reigster read */

/* Used for Spansion flashes only. */
#define	QSPI_CMD_BRRD		0x16	/* Bank register read */
#define	QSPI_CMD_BRWR		0x17	/* Bank register write */

/* Used for Spansion S25FS-S family flash only. */
#define QSPI_CMD_RDAR		0x65	/* Read any device register */
#define QSPI_CMD_WRAR		0x71	/* Write any device register */

/* 4-byte address QSPI CMD - used on Spansion and some Macronix flashes */
#define QSPI_CMD_FAST_READ_4B	0x0c    /* Read data bytes (high frequency) */
#define QSPI_CMD_PP_4B		0x12    /* Page program (up to 256 bytes) */
#define QSPI_CMD_SE_4B		0xdc    /* Sector erase (usually 64KiB) */

/* fsl_qspi_platdata flags */
#define QSPI_FLAG_REGMAP_ENDIAN_BIG	BIT(0)

/* default SCK frequency, unit: HZ */
#define FSL_QSPI_DEFAULT_SCK_FREQ	50000000

/* QSPI max chipselect signals number */
#define FSL_QSPI_MAX_CHIPSELECT_NUM     4

#ifdef CONFIG_DM_SPI
/**
 * struct fsl_qspi_platdata - platform data for Freescale QSPI
 *
 * @flags: Flags for QSPI QSPI_FLAG_...
 * @speed_hz: Default SCK frequency
 * @reg_base: Base address of QSPI registers
 * @amba_base: Base address of QSPI memory mapping
 * @amba_total_size: size of QSPI memory mapping
 * @flash_num: Number of active slave devices
 * @num_chipselect: Number of QSPI chipselect signals
 */
struct fsl_qspi_platdata {
	u32 flags;
	u32 speed_hz;
	fdt_addr_t reg_base;
	fdt_addr_t amba_base;
	fdt_size_t amba_total_size;
	u32 flash_num;
	u32 num_chipselect;
};
#endif

/**
 * struct fsl_qspi_priv - private data for Freescale QSPI
 *
 * @flags: Flags for QSPI QSPI_FLAG_...
 * @bus_clk: QSPI input clk frequency
 * @speed_hz: Default SCK frequency
 * @cur_cmd: current command
 * @sf_addr: flash access offset
 * @amba_base: Base address of QSPI memory mapping of every CS
 * @amba_total_size: size of QSPI memory mapping
 * @cur_amba_base: Base address of QSPI memory mapping of current CS
 * @flash_num: Number of active slave devices
 * @num_chipselect: Number of QSPI chipselect signals
 * @regs: Point to QSPI register structure for I/O access
 */
struct fsl_qspi_priv {
	u32 flags;
	u32 bus_clk;
	u32 speed_hz;
	u32 cur_cmd;
	u32 prev_cmd;
	u32 sf_addr;
	u32 amba_base[FSL_QSPI_MAX_CHIPSELECT_NUM];
	u32 amba_total_size;
	u32 cur_amba_base;
	u32 flash_num;
	u32 num_chipselect;
	u32 aai_mode;
	struct fsl_qspi_regs *regs;
};

#ifndef CONFIG_DM_SPI
struct fsl_qspi {
	struct spi_slave slave;
	struct fsl_qspi_priv priv;
};
#endif

static u32 qspi_read32(u32 flags, u32 *addr)
{
	return flags & QSPI_FLAG_REGMAP_ENDIAN_BIG ?
		in_be32(addr) : in_le32(addr);
}

static void qspi_write32(u32 flags, u32 *addr, u32 val)
{
	flags & QSPI_FLAG_REGMAP_ENDIAN_BIG ?
		out_be32(addr, val) : out_le32(addr, val);
}

static inline int is_controller_busy(const struct fsl_qspi_priv *priv)
{
	u32 val;
	const u32 mask = QSPI_SR_BUSY_MASK | QSPI_SR_AHB_ACC_MASK |
			 QSPI_SR_IP_ACC_MASK;
	unsigned int retry = 5;

	do {
		val = qspi_read32(priv->flags, &priv->regs->sr);

		if ((~val & mask) == mask)
			return 0;

		udelay(1);
	} while (--retry);

	return -ETIMEDOUT;
}

/* QSPI support swapping the flash read/write data
 * in hardware for LS102xA, but not for VF610 */
static inline u32 qspi_endian_xchg(u32 data)
{
#ifdef CONFIG_VF610
	return swab32(data);
#else
	return data;
#endif
}

struct lut_entry
{
	u32 entry[4];
};
static struct lut_entry lut_table[] = {
/* Write Enable */
[SEQID_WREN] = {
		{ LUT0(LUT_CMD, QSPI_CMD_WREN, LUT_PAD1),
		0, 0, 0 }
	},
/* Fast Read */
[SEQID_FAST_READ] = {
		{
#if defined(CONFIG_SPI_FLASH_BAR) || (FSL_QSPI_FLASH_SIZE <= SZ_16M)
		LUT0(LUT_CMD, QSPI_CMD_FAST_READ, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR24BIT, LUT_PAD1),
#else
		LUT0(LUT_CMD, QSPI_CMD_FAST_READ_4B, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR32BIT, LUT_PAD1),
#endif
		LUT0(LUT_DUMMY, 8, LUT_PAD1) |
		LUT1(LUT_READ, RX_BUFFER_SIZE, LUT_PAD1),
		0, 0 }
	},
/* Read Status */
[SEQID_RDSR] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_RDSR, LUT_PAD1) |
		LUT1(LUT_READ, 1, LUT_PAD1),
		0, 0, 0 }
	},
/* Erase a sector */
[SEQID_SE] = {
		{
#if defined(CONFIG_SPI_FLASH_BAR) || (FSL_QSPI_FLASH_SIZE <= SZ_16M)
		LUT0(LUT_CMD, QSPI_CMD_SE, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR24BIT, LUT_PAD1),
#else
		LUT0(LUT_CMD, QSPI_CMD_SE_4B, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR32BIT, LUT_PAD1),
#endif
		0, 0, 0 }
	},
/* Erase the whole chip */
[SEQID_CHIP_ERASE] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_CHIP_ERASE, LUT_PAD1),
		0, 0, 0 }
	},
/* Page Program */
[SEQID_PP] = {
		{
#if defined(CONFIG_SPI_FLASH_BAR) || (FSL_QSPI_FLASH_SIZE <= SZ_16M)
		LUT0(LUT_CMD, QSPI_CMD_PP, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR24BIT, LUT_PAD1),
#else
		LUT0(LUT_CMD, QSPI_CMD_PP_4B, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR32BIT, LUT_PAD1),
#endif
#if defined(CONFIG_MX6SX) || defined(CONFIG_MX6UL) || \
	defined(CONFIG_MX6ULL) || defined(CONFIG_MX7D)
		/*
		 * For MX6SX, TX_BUFFER_SIZE (0x200 is > 0xff) can not work correctly.
		 * So, Use IDATSZ in IPCR to determine the size and here set 0.
		 */
		LUT0(LUT_WRITE, 0, LUT_PAD1),
#else
		LUT0(LUT_WRITE, TX_BUFFER_SIZE, LUT_PAD1),
#endif
		0, 0 }
	},
/* READ ID */
[SEQID_RDID] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_RDID, LUT_PAD1) |
		LUT1(LUT_READ, 8, LUT_PAD1),
		0, 0, 0 }
	},
/* SUB SECTOR 4K ERASE */
[SEQID_BE_4K] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_BE_4K, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR24BIT, LUT_PAD1),
		0, 0, 0 }
	},
#ifdef CONFIG_SPI_FLASH_BAR
	/*
	 * BRRD BRWR RDEAR WREAR are all supported, because it is hard to
	 * dynamically check whether to set BRRD BRWR or RDEAR WREAR during
	 * initialization.
	 */
[SEQID_BRRD] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_BRRD, LUT_PAD1) |
		LUT1(LUT_READ, 1, LUT_PAD1),
		0, 0, 0 }
	},
[SEQID_BRWR] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_BRWR, LUT_PAD1) |
		LUT1(LUT_WRITE, 1, LUT_PAD1),
		0, 0, 0 }
	},
[SEQID_RDEAR] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_RDEAR, LUT_PAD1) |
		LUT1(LUT_READ, 1, LUT_PAD1),
		0, 0, 0 }
	},
[SEQID_WREAR] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_WREAR, LUT_PAD1) |
		LUT1(LUT_WRITE, 1, LUT_PAD1),
		0, 0, 0 }
},
#else
/* Write Status */
[SEQID_WRITE_STATUS] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_WRITE_STATUS, LUT_PAD1) |
		LUT1(LUT_WRITE, 1, LUT_PAD1),
		0, 0, 0 }
	},
/* SST_AAI_WP start */
[SEQID_SST_AAI_WP] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_SST_AAI_WP, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR24BIT, LUT_PAD1),
		LUT0(LUT_WRITE, 2, LUT_PAD1),
		0, 0 }
	},
/* SST_AAI_WP continue */
[SEQID_SST_AAI_WP_CONT] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_SST_AAI_WP, LUT_PAD1) |
		LUT1(LUT_WRITE, 2, LUT_PAD1),
		0, 0, 0 }
	},
/* WRITE_DISABLE */
[SEQID_WRITE_DISABLE] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_WRITE_DISABLE, LUT_PAD1),
		0, 0, 0 }
	},
#endif
[SEQID_WRAR] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_WRAR, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR24BIT, LUT_PAD1),
		LUT0(LUT_WRITE, 1, LUT_PAD1),
		0, 0 }
	},
[SEQID_RDAR] = {
		{
		LUT0(LUT_CMD, QSPI_CMD_RDAR, LUT_PAD1) |
		LUT1(LUT_ADDR, ADDR24BIT, LUT_PAD1),
		LUT0(LUT_DUMMY, 8, LUT_PAD1) |
		LUT1(LUT_READ, 1, LUT_PAD1),
		0, 0 }
	},
};

static void qspi_set_lut(struct fsl_qspi_priv *priv)
{
	struct fsl_qspi_regs *regs = priv->regs;
	struct lut_entry *e = lut_table;
	int i;

	/* Unlock the LUT */
	qspi_write32(priv->flags, &regs->lutkey, LUT_KEY_VALUE);
	qspi_write32(priv->flags, &regs->lckcr, QSPI_LCKCR_UNLOCK);

	for (i = 0; i < ARRAY_SIZE(lut_table); i++, e++) {
		u32 *p = &regs->lut[i * 4];

		qspi_write32(priv->flags, &p[0], e->entry[0]);
		qspi_write32(priv->flags, &p[1], e->entry[1]);
		qspi_write32(priv->flags, &p[2], e->entry[2]);
		qspi_write32(priv->flags, &p[3], e->entry[3]);
	}

	/* Lock the LUT */
	qspi_write32(priv->flags, &regs->lutkey, LUT_KEY_VALUE);
	qspi_write32(priv->flags, &regs->lckcr, QSPI_LCKCR_LOCK);
}

#if defined(CONFIG_SYS_FSL_QSPI_AHB)
/*
 * If we have changed the content of the flash by writing or erasing,
 * we need to invalidate the AHB buffer. If we do not do so, we may read out
 * the wrong data. The spec tells us reset the AHB domain and Serial Flash
 * domain at the same time.
 */
static inline void qspi_ahb_invalid(struct fsl_qspi_priv *priv)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 reg;

	reg = qspi_read32(priv->flags, &regs->mcr);
	reg |= QSPI_MCR_SWRSTHD_MASK | QSPI_MCR_SWRSTSD_MASK;
	qspi_write32(priv->flags, &regs->mcr, reg);

	/*
	 * The minimum delay : 1 AHB + 2 SFCK clocks.
	 * Delay 1 us is enough.
	 */
	udelay(1);

	reg &= ~(QSPI_MCR_SWRSTHD_MASK | QSPI_MCR_SWRSTSD_MASK);
	qspi_write32(priv->flags, &regs->mcr, reg);
}

/* Read out the data from the AHB buffer. */
static inline void qspi_ahb_read(struct fsl_qspi_priv *priv, u8 *rxbuf, int len)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 mcr_reg;
	void *rx_addr;

	mcr_reg = qspi_read32(priv->flags, &regs->mcr);

	qspi_write32(priv->flags, &regs->mcr,
		     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
		     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);

	rx_addr = (void *)(uintptr_t)(priv->cur_amba_base + priv->sf_addr);
	/* Read out the data directly from the AHB buffer. */
	memcpy(rxbuf, rx_addr, len);

	qspi_write32(priv->flags, &regs->mcr, mcr_reg);
}

static void qspi_enable_ddr_mode(struct fsl_qspi_priv *priv)
{
	u32 reg, reg2;
	struct fsl_qspi_regs *regs = priv->regs;

	reg = qspi_read32(priv->flags, &regs->mcr);
	/* Disable the module */
	qspi_write32(priv->flags, &regs->mcr, reg | QSPI_MCR_MDIS_MASK);

	/* Set the Sampling Register for DDR */
	reg2 = qspi_read32(priv->flags, &regs->smpr);
	reg2 &= ~QSPI_SMPR_DDRSMP_MASK;
	reg2 |= (2 << QSPI_SMPR_DDRSMP_SHIFT);
	qspi_write32(priv->flags, &regs->smpr, reg2);

	/* Enable the module again (enable the DDR too) */
	reg |= QSPI_MCR_DDR_EN_MASK;
	/* Enable bit 29 for imx6sx */
	reg |= BIT(29);

	qspi_write32(priv->flags, &regs->mcr, reg);
}

/*
 * There are two different ways to read out the data from the flash:
 *  the "IP Command Read" and the "AHB Command Read".
 *
 * The IC guy suggests we use the "AHB Command Read" which is faster
 * then the "IP Command Read". (What's more is that there is a bug in
 * the "IP Command Read" in the Vybrid.)
 *
 * After we set up the registers for the "AHB Command Read", we can use
 * the memcpy to read the data directly. A "missed" access to the buffer
 * causes the controller to clear the buffer, and use the sequence pointed
 * by the QUADSPI_BFGENCR[SEQID] to initiate a read from the flash.
 */
static void qspi_init_ahb_read(struct fsl_qspi_priv *priv)
{
	struct fsl_qspi_regs *regs = priv->regs;

	/* AHB configuration for access buffer 0/1/2 .*/
	qspi_write32(priv->flags, &regs->buf0cr, QSPI_BUFXCR_INVALID_MSTRID);
	qspi_write32(priv->flags, &regs->buf1cr, QSPI_BUFXCR_INVALID_MSTRID);
	qspi_write32(priv->flags, &regs->buf2cr, QSPI_BUFXCR_INVALID_MSTRID);
	qspi_write32(priv->flags, &regs->buf3cr, QSPI_BUF3CR_ALLMST_MASK |
		     (0x80 << QSPI_BUF3CR_ADATSZ_SHIFT));

	/* We only use the buffer3 */
	qspi_write32(priv->flags, &regs->buf0ind, 0);
	qspi_write32(priv->flags, &regs->buf1ind, 0);
	qspi_write32(priv->flags, &regs->buf2ind, 0);

	/*
	 * Set the default lut sequence for AHB Read.
	 * Parallel mode is disabled.
	 */
	qspi_write32(priv->flags, &regs->bfgencr,
		     SEQID_FAST_READ << QSPI_BFGENCR_SEQID_SHIFT);

	/*Enable DDR Mode*/
	qspi_enable_ddr_mode(priv);
}
#endif

static void qspi_start_transaction_tx(struct fsl_qspi_priv *priv, int seqid, int len)
{
	struct fsl_qspi_regs *regs = priv->regs;

//	if ((seqid != SEQID_SST_AAI_WP_CONT) || (len != 2))
//		if ((seqid != SEQID_RDSR) || (len != 0))
			debug("%s:cmd=%02x, seqid=%02x, len=%x sfar=%x\n", __func__,
				priv->cur_cmd, seqid, len,
				priv->sf_addr + priv->cur_amba_base);

	qspi_write32(priv->flags, &regs->sfar,
			priv->sf_addr + priv->cur_amba_base);

	qspi_write32(priv->flags, &regs->ipcr,
			(seqid << QSPI_IPCR_SEQID_SHIFT) | len);

	/* Wait previous command complete */
	while (qspi_read32(priv->flags, &regs->sr) & QSPI_SR_BUSY_MASK)
		;
}

static void qspi_start_transaction(struct fsl_qspi_priv *priv, int seqid, int len)
{
	struct fsl_qspi_regs *regs = priv->regs;

	qspi_write32(priv->flags, &regs->mcr,
		     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
		     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);
	qspi_start_transaction_tx(priv, seqid, len);
}

static u32 qspi_rx_single_entry(struct fsl_qspi_priv *priv)
{
	u32 data;
	struct fsl_qspi_regs *regs = priv->regs;

	do {
		WATCHDOG_RESET();
		data = qspi_read32(priv->flags, &regs->rbsr);
	} while (!(data & QSPI_RBSR_RDBFL_MASK));

	data = qspi_read32(priv->flags, &regs->rbdr[0]);
	data = qspi_endian_xchg(data);
	return data;
}

static u32 wait_for_idle(struct fsl_qspi_priv *priv)
{
	u32 status_reg;

	do {
		qspi_start_transaction(priv, SEQID_RDSR, 1);
		status_reg = qspi_rx_single_entry(priv);
	} while ((status_reg & FLASH_STATUS_BUSY));

	return status_reg;
}

static void qspi_send_cmd(struct fsl_qspi_priv *priv, int seq)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 mcr_reg = qspi_read32(priv->flags, &regs->mcr);

	qspi_write32(priv->flags, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);

	qspi_start_transaction(priv, seq, 0);

	qspi_write32(priv->flags, &regs->mcr, mcr_reg);
}

#ifdef CONFIG_SPI_FLASH_BAR
/* Bank register read/write, EAR register read/write */
static void qspi_op_rdbank(struct fsl_qspi_priv *priv, u8 *rxbuf, u32 len)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 reg, mcr_reg, data, seqid;

	mcr_reg = qspi_read32(priv->flags, &regs->mcr);
	qspi_write32(priv->flags, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);


	if (priv->cur_cmd == QSPI_CMD_BRRD)
		seqid = SEQID_BRRD;
	else
		seqid = SEQID_RDEAR;

	qspi_start_transaction(priv, seqid, len);
	data = qspi_rx_single_entry(priv);
	memcpy(rxbuf, &data, len);

	qspi_write32(priv->flags, &regs->mcr, mcr_reg);
}
#endif

static void qspi_op_rdid(struct fsl_qspi_priv *priv, u32 *rxbuf, u32 len)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 mcr_reg, rbsr_reg, data, size;
	int i;

	mcr_reg = qspi_read32(priv->flags, &regs->mcr);
	qspi_write32(priv->flags, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);

	qspi_start_transaction(priv, SEQID_RDID, len);

	i = 0;
	while ((RX_BUFFER_SIZE >= len) && (len > 0)) {
		WATCHDOG_RESET();

		rbsr_reg = qspi_read32(priv->flags, &regs->rbsr);
		if (rbsr_reg & QSPI_RBSR_RDBFL_MASK) {
			data = qspi_read32(priv->flags, &regs->rbdr[i]);
			data = qspi_endian_xchg(data);
			size = (len < 4) ? len : 4;
			memcpy(rxbuf, &data, size);
			len -= size;
			rxbuf++;
			i++;
		}
	}

	qspi_write32(priv->flags, &regs->mcr, mcr_reg);
}

/* If not use AHB read, read data from ip interface */
static void qspi_op_read(struct fsl_qspi_priv *priv, u32 *rxbuf, u32 len)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 mcr_reg, data;
	int i, size;
	u32 seqid;

	if (priv->cur_cmd == QSPI_CMD_RDAR)
		seqid = SEQID_RDAR;
	else
		seqid = SEQID_FAST_READ;

	mcr_reg = qspi_read32(priv->flags, &regs->mcr);
	qspi_write32(priv->flags, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);

	while (len > 0) {
		WATCHDOG_RESET();
		size = (len > RX_BUFFER_SIZE) ?
			RX_BUFFER_SIZE : len;

		qspi_start_transaction(priv, seqid, size);

		priv->sf_addr += size;
		len -= size;

		i = 0;
		while ((RX_BUFFER_SIZE >= size) && (size > 0)) {
			data = qspi_read32(priv->flags, &regs->rbsr);
			if (data & QSPI_RBSR_RDBFL_MASK) {
				data = qspi_read32(priv->flags, &regs->rbdr[i]);
				data = qspi_endian_xchg(data);
				if (size < 4)
					memcpy(rxbuf, &data, size);
				else
					memcpy(rxbuf, &data, 4);
				rxbuf++;
				size -= 4;
				i++;
			}
		}
	}

	qspi_write32(priv->flags, &regs->mcr, mcr_reg);
}

static void qspi_op_write(struct fsl_qspi_priv *priv, u8 *txbuf, u32 len)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 mcr_reg, data, status_reg, seqid;
	int i, size, tx_size;

	mcr_reg = qspi_read32(priv->flags, &regs->mcr);
	qspi_write32(priv->flags, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);
	WATCHDOG_RESET();

	/* Default is page programming */
	seqid = SEQID_PP;
	if (priv->cur_cmd == QSPI_CMD_WRAR)
		seqid = SEQID_WRAR;
#ifdef CONFIG_SPI_FLASH_BAR
	if (priv->cur_cmd == QSPI_CMD_BRWR)
		seqid = SEQID_BRWR;
	else if (priv->cur_cmd == QSPI_CMD_WREAR)
		seqid = SEQID_WREAR;
#else
	if (priv->cur_cmd == QSPI_CMD_SST_AAI_WP) {
		seqid = priv->aai_mode ? SEQID_SST_AAI_WP_CONT : SEQID_SST_AAI_WP;
		priv->aai_mode = 1;
	} else if (priv->cur_cmd == QSPI_CMD_WRITE_STATUS) {
		seqid = SEQID_WRITE_STATUS;
	}
#endif

	if ((seqid != SEQID_SST_AAI_WP_CONT)) {
		do {
			qspi_start_transaction(priv, SEQID_WREN, 0);
			status_reg = wait_for_idle(priv);
			/* Try until write is enabled */
		} while (!(status_reg & FLASH_STATUS_WEL));
	}

	qspi_write32(priv->flags, &regs->mcr,
		     QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK |
		     QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFD_LE);
	tx_size = (len > TX_BUFFER_SIZE) ? TX_BUFFER_SIZE : len;

	i = tx_size;
	while (i) {
		size = 4;
		if (size > i) {
			data = 0;
			size = i;
		}
		memcpy(&data, txbuf, size);
		data = qspi_endian_xchg(data);
		qspi_write32(priv->flags, &regs->tbdr, data);
		txbuf += size;
		i -= size;
	}
	i = tx_size;
	/*
	 * There must be atleast 128bit data
	 * available in TX FIFO for any pop operation
	 */
	while (i <= 12) {
		qspi_write32(priv->flags, &regs->tbdr, data);
		i += 4;
	}

	qspi_start_transaction_tx(priv, seqid, tx_size);

	qspi_write32(priv->flags, &regs->mcr, mcr_reg);
}

static void qspi_op_rdsr(struct fsl_qspi_priv *priv, void *rxbuf, u32 len)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 mcr_reg, data;

	mcr_reg = qspi_read32(priv->flags, &regs->mcr);
	qspi_write32(priv->flags, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);

	qspi_start_transaction(priv, SEQID_RDSR, 1);
	data = qspi_rx_single_entry(priv);
	memcpy(rxbuf, &data, len);

	qspi_write32(priv->flags, &regs->mcr, mcr_reg);
}

static void qspi_op_erase(struct fsl_qspi_priv *priv)
{
	struct fsl_qspi_regs *regs = priv->regs;
	u32 mcr_reg;

	mcr_reg = qspi_read32(priv->flags, &regs->mcr);
	qspi_write32(priv->flags, &regs->rbct, QSPI_RBCT_RXBRD_USEIPS);

	qspi_start_transaction(priv, SEQID_WREN, 0);


	if (priv->cur_cmd == QSPI_CMD_SE) {
		qspi_start_transaction(priv, SEQID_SE, 0);
	} else if (priv->cur_cmd == QSPI_CMD_BE_4K) {
		qspi_start_transaction(priv, SEQID_BE_4K, 0);
	}
	qspi_write32(priv->flags, &regs->mcr, mcr_reg);
}

int qspi_xfer(struct fsl_qspi_priv *priv, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	u32 bytes = DIV_ROUND_UP(bitlen, 8);

	WATCHDOG_RESET();

	if (dout) {
		if (flags & SPI_XFER_BEGIN) {
			priv->prev_cmd = priv->cur_cmd;
			priv->cur_cmd = *(u8 *)dout;
			debug("%s: cur_cmd=%02x\n", __func__, priv->cur_cmd);
			switch (priv->cur_cmd) {
#ifdef CONFIG_SPI_FLASH_BAR
			case QSPI_CMD_BRWR:
			case QSPI_CMD_WREAR:
			case QSPI_CMD_BRRD:
			case QSPI_CMD_RDEAR:
				break;
#else
			case QSPI_CMD_WRITE_STATUS:
			case QSPI_CMD_WRITE_DISABLE:
				break;
			case QSPI_CMD_SST_AAI_WP:
#endif
			case QSPI_CMD_FAST_READ:
			case QSPI_CMD_RDAR:
			case QSPI_CMD_SE:
			case QSPI_CMD_PP:
			case QSPI_CMD_WRAR:
			case QSPI_CMD_BE_4K:
				if (bitlen >= 16) {
					u32 txbuf = 0;
					int len = (bitlen <= 32) ?
							bitlen >> 3 : 4;

					memcpy(&txbuf, dout, len);
					priv->sf_addr = swab32(txbuf)
							& OFFSET_BITS_MASK;
				}
			case QSPI_CMD_RDID:
			case QSPI_CMD_RDSR:
			case QSPI_CMD_WREN:
				break;
			default:
				printf("%s: unexpected cmd=%02x\n", __func__, priv->cur_cmd);
			}
		}

		if (flags == SPI_XFER_END) {
			qspi_op_write(priv, (u8 *)dout, bytes);
			if ((priv->cur_cmd) == QSPI_CMD_WRITE_STATUS) {
				u32 wanted = *(u8 *)dout;
				u32 status = wait_for_idle(priv);

				if ((status ^ wanted) & 0x3c)
					printf("!!write_status: wanted %x, got=%x\n",
						wanted, status);
			}
			return 0;
		}

		switch (priv->cur_cmd) {
		case QSPI_CMD_WREN:
			/* This is done in the write command */
			break;
		case QSPI_CMD_SE:
		case QSPI_CMD_BE_4K:
			qspi_op_erase(priv);
			break;
#ifndef CONFIG_SPI_FLASH_BAR
		case QSPI_CMD_WRITE_DISABLE:
			qspi_send_cmd(priv, SEQID_WRITE_DISABLE);
			priv->aai_mode = 0;
			break;
#endif
		}
	}

	if (din) {
		switch (priv->cur_cmd) {
		case QSPI_CMD_FAST_READ:
#ifdef CONFIG_SYS_FSL_QSPI_AHB
			qspi_ahb_read(priv, din, bytes);
#else
			qspi_op_read(priv, din, bytes);
#endif
			break;
		case QSPI_CMD_RDAR:
			qspi_op_read(priv, din, bytes);
			break;
		case QSPI_CMD_RDID:
			qspi_op_rdid(priv, din, bytes);
			break;
		case QSPI_CMD_RDSR:
			qspi_op_rdsr(priv, din, bytes);
			break;
#ifdef CONFIG_SPI_FLASH_BAR
		case QSPI_CMD_BRRD:
		case QSPI_CMD_RDEAR:
			priv->sf_addr = 0;
			qspi_op_rdbank(priv, din, bytes);
			break;
#endif
		}
	}

#ifdef CONFIG_SYS_FSL_QSPI_AHB
	if ((priv->cur_cmd == QSPI_CMD_SE) ||
	    (priv->cur_cmd == QSPI_CMD_PP) ||
	    (priv->cur_cmd == QSPI_CMD_WRITE_DISABLE) ||
	    (priv->cur_cmd == QSPI_CMD_BE_4K) ||
	    (priv->cur_cmd == QSPI_CMD_WREAR) ||
	    (priv->cur_cmd == QSPI_CMD_BRWR))
		qspi_ahb_invalid(priv);
#endif

	return 0;
}

void qspi_module_disable(struct fsl_qspi_priv *priv, u8 disable)
{
	u32 mcr_val;

	mcr_val = qspi_read32(priv->flags, &priv->regs->mcr);
	if (disable)
		mcr_val |= QSPI_MCR_MDIS_MASK;
	else
		mcr_val &= ~QSPI_MCR_MDIS_MASK;
	qspi_write32(priv->flags, &priv->regs->mcr, mcr_val);
}

void qspi_cfg_smpr(struct fsl_qspi_priv *priv, u32 clear_bits, u32 set_bits)
{
	u32 smpr_val;

	smpr_val = qspi_read32(priv->flags, &priv->regs->smpr);
	smpr_val &= ~clear_bits;
	smpr_val |= set_bits;
	qspi_write32(priv->flags, &priv->regs->smpr, smpr_val);
}
#ifndef CONFIG_DM_SPI
static unsigned long spi_bases[] = {
	QSPI0_BASE_ADDR,
#ifdef CONFIG_MX6SX
	QSPI1_BASE_ADDR,
#endif
};

static unsigned long amba_bases[] = {
	QSPI0_AMBA_BASE,
#ifdef CONFIG_MX6SX
	QSPI1_AMBA_BASE,
#endif
};

static inline struct fsl_qspi *to_qspi_spi(struct spi_slave *slave)
{
	return container_of(slave, struct fsl_qspi, slave);
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	u32 mcr_val;
	struct fsl_qspi *qspi;
	struct fsl_qspi_regs *regs;
	u32 total_size;

	if (bus >= ARRAY_SIZE(spi_bases))
		return NULL;

#ifdef CONFIG_MX6
	if (mx6_qspi_fused(spi_bases[bus])) {
		printf("QSPI@0x%lx is fused, disable it\n", spi_bases[bus]);
		return NULL;
	}
#endif

	if (cs >= FSL_QSPI_FLASH_NUM)
		return NULL;

	qspi = spi_alloc_slave(struct fsl_qspi, bus, cs);
	if (!qspi)
		return NULL;

#ifdef CONFIG_SYS_FSL_QSPI_BE
	qspi->priv.flags |= QSPI_FLAG_REGMAP_ENDIAN_BIG;
#endif

	regs = (struct fsl_qspi_regs *)spi_bases[bus];
	qspi->priv.regs = regs;
	qspi->priv.speed_hz = max_hz;
	/*
	 * According cs, use different amba_base to choose the
	 * corresponding flash devices.
	 *
	 * If not, only one flash device is used even if passing
	 * different cs using `sf probe`
	 */
	qspi->priv.cur_amba_base = amba_bases[bus] + cs * FSL_QSPI_FLASH_SIZE;

	qspi->slave.max_write_size = TX_BUFFER_SIZE;

	set_clk_qspi(qspi->priv.speed_hz);
	mcr_val = qspi_read32(qspi->priv.flags, &regs->mcr);

	/* Set endianness to LE for i.mx */
	if (IS_ENABLED(CONFIG_MX6) || IS_ENABLED(CONFIG_MX7))
		mcr_val = QSPI_MCR_END_CFD_LE;

	qspi_write32(qspi->priv.flags, &regs->mcr,
		     QSPI_MCR_RESERVED_MASK | QSPI_MCR_MDIS_MASK |
		     (mcr_val & QSPI_MCR_END_CFD_MASK));

	qspi_cfg_smpr(&qspi->priv,
		      ~(QSPI_SMPR_FSDLY_MASK | QSPI_SMPR_DDRSMP_MASK |
		      QSPI_SMPR_FSPHS_MASK | QSPI_SMPR_HSENA_MASK), 0);

	total_size = FSL_QSPI_FLASH_SIZE * FSL_QSPI_FLASH_NUM;
	/*
	 * Any read access to non-implemented addresses will provide
	 * undefined results.
	 *
	 * In case single die flash devices, TOP_ADDR_MEMA2 and
	 * TOP_ADDR_MEMB2 should be initialized/programmed to
	 * TOP_ADDR_MEMA1 and TOP_ADDR_MEMB1 respectively - in effect,
	 * setting the size of these devices to 0.  This would ensure
	 * that the complete memory map is assigned to only one flash device.
	 */
	qspi_write32(qspi->priv.flags, &regs->sfa1ad,
		     FSL_QSPI_FLASH_SIZE | amba_bases[bus]);
	qspi_write32(qspi->priv.flags, &regs->sfa2ad,
		     FSL_QSPI_FLASH_SIZE | amba_bases[bus]);
	qspi_write32(qspi->priv.flags, &regs->sfb1ad,
		     total_size | amba_bases[bus]);
	qspi_write32(qspi->priv.flags, &regs->sfb2ad,
		     total_size | amba_bases[bus]);

	qspi_set_lut(&qspi->priv);

#ifdef CONFIG_SYS_FSL_QSPI_AHB
	qspi_init_ahb_read(&qspi->priv);
#endif

	qspi_module_disable(&qspi->priv, 0);

	return &qspi->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct fsl_qspi *qspi = to_qspi_spi(slave);

	free(qspi);
}

int spi_claim_bus(struct spi_slave *slave)
{
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	/* Nothing to do */
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	struct fsl_qspi *qspi = to_qspi_spi(slave);

	return qspi_xfer(&qspi->priv, bitlen, dout, din, flags);
}

void spi_init(void)
{
	/* Nothing to do */
}

#else
static int fsl_qspi_child_pre_probe(struct udevice *dev)
{
	struct spi_slave *slave = dev_get_parent_priv(dev);

	slave->max_write_size = TX_BUFFER_SIZE;

	return 0;
}

static int fsl_qspi_probe(struct udevice *bus)
{
	u32 mcr_val;
	u32 amba_size_per_chip;
	struct fsl_qspi_platdata *plat = dev_get_platdata(bus);
	struct fsl_qspi_priv *priv = dev_get_priv(bus);
	struct dm_spi_bus *dm_spi_bus;
	int i, ret;

#ifdef CONFIG_MX6
	if (mx6_qspi_fused(plat->reg_base)) {
		printf("QSPI@0x%lx is fused, disable it\n", plat->reg_base);
		return -ENODEV;
	}
#endif	

	dm_spi_bus = bus->uclass_priv;

	dm_spi_bus->max_hz = plat->speed_hz;

	priv->regs = (struct fsl_qspi_regs *)(uintptr_t)plat->reg_base;
	priv->flags = plat->flags;

	priv->speed_hz = plat->speed_hz;
	/*
	 * QSPI SFADR width is 32bits, the max dest addr is 4GB-1.
	 * AMBA memory zone should be located on the 0~4GB space
	 * even on a 64bits cpu.
	 */
	priv->amba_base[0] = (u32)plat->amba_base;
	priv->amba_total_size = (u32)plat->amba_total_size;
	priv->flash_num = plat->flash_num;
	priv->num_chipselect = plat->num_chipselect;

	/* make sure controller is not busy anywhere */
	ret = is_controller_busy(priv);

	if (ret) {
		debug("ERROR : The controller is busy\n");
		return ret;
	}

	mcr_val = qspi_read32(priv->flags, &priv->regs->mcr);

	/* Set endianness to LE for i.mx */
	if (IS_ENABLED(CONFIG_MX6) || IS_ENABLED(CONFIG_MX7))
		mcr_val = QSPI_MCR_END_CFD_LE;

	qspi_write32(priv->flags, &priv->regs->mcr,
		     QSPI_MCR_RESERVED_MASK | QSPI_MCR_MDIS_MASK |
		     (mcr_val & QSPI_MCR_END_CFD_MASK));

	qspi_cfg_smpr(priv, ~(QSPI_SMPR_FSDLY_MASK | QSPI_SMPR_DDRSMP_MASK |
		QSPI_SMPR_FSPHS_MASK | QSPI_SMPR_HSENA_MASK), 0);

	/*
	 * Assign AMBA memory zone for every chipselect
	 * QuadSPI has two channels, every channel has two chipselects.
	 * If the property 'num-cs' in dts is 2, the AMBA memory will be divided
	 * into two parts and assign to every channel. This indicate that every
	 * channel only has one valid chipselect.
	 * If the property 'num-cs' in dts is 4, the AMBA memory will be divided
	 * into four parts and assign to every chipselect.
	 * Every channel will has two valid chipselects.
	 */
	amba_size_per_chip = priv->amba_total_size >>
			     (priv->num_chipselect >> 1);
	for (i = 1 ; i < priv->num_chipselect ; i++)
		priv->amba_base[i] =
			amba_size_per_chip + priv->amba_base[i - 1];

	/*
	 * Any read access to non-implemented addresses will provide
	 * undefined results.
	 *
	 * In case single die flash devices, TOP_ADDR_MEMA2 and
	 * TOP_ADDR_MEMB2 should be initialized/programmed to
	 * TOP_ADDR_MEMA1 and TOP_ADDR_MEMB1 respectively - in effect,
	 * setting the size of these devices to 0.  This would ensure
	 * that the complete memory map is assigned to only one flash device.
	 */
	qspi_write32(priv->flags, &priv->regs->sfa1ad,
		     priv->amba_base[0] + amba_size_per_chip);
	switch (priv->num_chipselect) {
	case 1:
		break;
	case 2:
		qspi_write32(priv->flags, &priv->regs->sfa2ad,
			     priv->amba_base[1]);
		qspi_write32(priv->flags, &priv->regs->sfb1ad,
			     priv->amba_base[1] + amba_size_per_chip);
		qspi_write32(priv->flags, &priv->regs->sfb2ad,
			     priv->amba_base[1] + amba_size_per_chip);
		break;
	case 4:
		qspi_write32(priv->flags, &priv->regs->sfa2ad,
			     priv->amba_base[2]);
		qspi_write32(priv->flags, &priv->regs->sfb1ad,
			     priv->amba_base[3]);
		qspi_write32(priv->flags, &priv->regs->sfb2ad,
			     priv->amba_base[3] + amba_size_per_chip);
		break;
	default:
		debug("Error: Unsupported chipselect number %u!\n",
		      priv->num_chipselect);
		qspi_module_disable(priv, 1);
		return -EINVAL;
	}

	qspi_set_lut(priv);

#ifdef CONFIG_SYS_FSL_QSPI_AHB
	qspi_init_ahb_read(priv);
#endif

	qspi_module_disable(priv, 0);

	return 0;
}

static int fsl_qspi_ofdata_to_platdata(struct udevice *bus)
{
	struct fdt_resource res_regs, res_mem;
	struct fsl_qspi_platdata *plat = bus->platdata;
	const void *blob = gd->fdt_blob;
	int node = dev_of_offset(bus);
	int ret, flash_num = 0, subnode;

	if (fdtdec_get_bool(blob, node, "big-endian"))
		plat->flags |= QSPI_FLAG_REGMAP_ENDIAN_BIG;

	ret = fdt_get_named_resource(blob, node, "reg", "reg-names",
				     "QuadSPI", &res_regs);
	if (ret) {
		debug("Error: can't get regs base addresses(ret = %d)!\n", ret);
		return -ENOMEM;
	}
	ret = fdt_get_named_resource(blob, node, "reg", "reg-names",
				     "QuadSPI-memory", &res_mem);
	if (ret) {
		debug("Error: can't get AMBA base addresses(ret = %d)!\n", ret);
		return -ENOMEM;
	}

	/* Count flash numbers */
	fdt_for_each_subnode(subnode, blob, node)
		++flash_num;

	if (flash_num == 0) {
		debug("Error: Missing flashes!\n");
		return -ENODEV;
	}

	plat->speed_hz = fdtdec_get_int(blob, node, "spi-max-frequency",
					FSL_QSPI_DEFAULT_SCK_FREQ);
	plat->num_chipselect = fdtdec_get_int(blob, node, "num-cs",
					      FSL_QSPI_MAX_CHIPSELECT_NUM);

	plat->reg_base = res_regs.start;
	plat->amba_base = res_mem.start;
	plat->amba_total_size = res_mem.end - res_mem.start + 1;
	plat->flash_num = flash_num;

	debug("%s: regs=<0x%llx> <0x%llx, 0x%llx>, max-frequency=%d, endianess=%s\n",
	      __func__,
	      (u64)plat->reg_base,
	      (u64)plat->amba_base,
	      (u64)plat->amba_total_size,
	      plat->speed_hz,
	      plat->flags & QSPI_FLAG_REGMAP_ENDIAN_BIG ? "be" : "le"
	      );

	return 0;
}

static int fsl_qspi_xfer(struct udevice *dev, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	struct fsl_qspi_priv *priv;
	struct udevice *bus;

	bus = dev->parent;
	priv = dev_get_priv(bus);

	return qspi_xfer(priv, bitlen, dout, din, flags);
}

static int fsl_qspi_claim_bus(struct udevice *dev)
{
	struct fsl_qspi_priv *priv;
	struct udevice *bus;
	struct dm_spi_slave_platdata *slave_plat = dev_get_parent_platdata(dev);
	int ret;

	bus = dev->parent;
	priv = dev_get_priv(bus);

	/* make sure controller is not busy anywhere */
	ret = is_controller_busy(priv);

	if (ret) {
		debug("ERROR : The controller is busy\n");
		return ret;
	}

	priv->cur_amba_base = priv->amba_base[slave_plat->cs];

	qspi_module_disable(priv, 0);

	return 0;
}

static int fsl_qspi_release_bus(struct udevice *dev)
{
	struct fsl_qspi_priv *priv;
	struct udevice *bus;

	bus = dev->parent;
	priv = dev_get_priv(bus);

	qspi_module_disable(priv, 1);

	return 0;
}

static int fsl_qspi_set_speed(struct udevice *bus, uint speed)
{
	/* Nothing to do */
	return 0;
}

static int fsl_qspi_set_mode(struct udevice *bus, uint mode)
{
	/* Nothing to do */
	return 0;
}

static const struct dm_spi_ops fsl_qspi_ops = {
	.claim_bus	= fsl_qspi_claim_bus,
	.release_bus	= fsl_qspi_release_bus,
	.xfer		= fsl_qspi_xfer,
	.set_speed	= fsl_qspi_set_speed,
	.set_mode	= fsl_qspi_set_mode,
};

static const struct udevice_id fsl_qspi_ids[] = {
	{ .compatible = "fsl,vf610-qspi" },
	{ .compatible = "fsl,imx6sx-qspi" },
	{ .compatible = "fsl,imx6ul-qspi" },
	{ .compatible = "fsl,imx7d-qspi" },
	{ }
};

U_BOOT_DRIVER(fsl_qspi) = {
	.name	= "fsl_qspi",
	.id	= UCLASS_SPI,
	.of_match = fsl_qspi_ids,
	.ops	= &fsl_qspi_ops,
	.ofdata_to_platdata = fsl_qspi_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct fsl_qspi_platdata),
	.priv_auto_alloc_size = sizeof(struct fsl_qspi_priv),
	.probe	= fsl_qspi_probe,
	.child_pre_probe = fsl_qspi_child_pre_probe,
};
#endif

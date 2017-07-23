/*
 * Copyright (C) 2018, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#define PAD_CTRL_ENET_MDC	(PAD_CTL_DSE3)
#define PAD_CTRL_ENET_MDIO	(PAD_CTL_DSE3 | PAD_CTL_ODE)

#define PAD_CTRL_ENET_RX	0x91
#define PAD_CTRL_ENET_RX_DN	(PAD_CTRL_ENET_RX)
#define PAD_CTRL_ENET_RX_UP	(PAD_CTL_PUE | PAD_CTRL_ENET_RX)

#define PAD_CTRL_ENET_TX	0x1f

#define WEAK_PULLUP		(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)
#define WEAK_PULLUP_OUTPUT	0xd1
#define WEAK_PULLDN_OUTPUT	0x91	/* Cannot pull down */

#define QSPI_PAD_CTRL	(PAD_CTL_DSE2 | PAD_CTL_HYS)

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)

#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

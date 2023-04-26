/*
 * Copyright (C) 2018, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#define PAD_CTRL_ENET_MDC	(0)
/* PAD_CTL_ODE is screwy on MDIO, it behaves like a strong hysteresis */
#define PAD_CTRL_ENET_MDIO	(0)

#define PAD_CTRL_ENET_RX	(PAD_CTL_DSE)
#define PAD_CTRL_ENET_RX_DN	(PAD_CTRL_ENET_RX | PAD_CTL_PUS_UP)
#define PAD_CTRL_ENET_RX_UP	(PAD_CTRL_ENET_RX | PAD_CTL_PUS_DOWN)

#define PAD_CTRL_ENET_TX	0x1f

#define WEAK_PULLUP		(PAD_CTL_PUS_UP)
#define WEAK_PULLUP_OUTPUT	(PAD_CTL_PUS_UP)
#define WEAK_PULLDN_OUTPUT	(PAD_CTL_PUS_DOWN)

#define QSPI_PAD_CTRL	(PAD_CTL_DSE)

#define UART_PAD_CTRL	(PAD_CTL_PUS_UP)

#define WDOG_PAD_CTRL	(PAD_CTL_ODE | PAD_CTL_PUS_UP)

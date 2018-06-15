/*
 * bur_cfg_common.h
 *
 * common parts used over all B&R boards
 *
 * Copyright (C) 2016 Hannes Schmelzer <oe5hpm@oevsv.at> -
 * Bernecker & Rainer Industrieelektronik GmbH - http://www.br-automation.com
 *
 * SPDX-License-Identifier:        GPL-2.0+
 */

#ifndef __BUR_CFG_COMMON_H__
#define __BUR_CFG_COMMON_H__
/* ------------------------------------------------------------------------- */
#define BUR_COMMON_ENV \
"usbscript=usb start && fatload usb 0 ${scradr} usbscript.img &&" \
" source ${scradr}\0" \
"brdefaultip=if test -r ${ipaddr}; then; else" \
" setenv ipaddr 192.168.60.1; setenv serverip 192.168.60.254;" \
" setenv gatewayip 192.168.60.254; setenv netmask 255.255.255.0; fi;\0" \
"netconsole=echo switching to network console ...; " \
"if dhcp; then; else run brdefaultip; fi; setenv ncip ${serverip}; " \
"setcurs 1 9; lcdputs myip; setcurs 10 9; lcdputs ${ipaddr};" \
"setcurs 1 10;lcdputs serverip; setcurs 10 10; lcdputs ${serverip};" \
"setenv stdout nc;setenv stdin nc;setenv stderr nc\0"

#define CONFIG_PREBOOT			"run cfgscr; run brdefaultip"

/* Network defines */
#define CONFIG_BOOTP_DNS		/* Configurable parts of CMD_DHCP */
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_NET_RETRY_COUNT		10

/* Network console */
#define CONFIG_NETCONSOLE		1
#define CONFIG_BOOTP_MAY_FAIL		/* if we don't have DHCP environment */

#define CONFIG_ENV_OVERWRITE		/* Overwrite ethaddr / serial# */

/* As stated above, the following choices are optional. */
#define CONFIG_SYS_LONGHELP
#define CONFIG_AUTO_COMPLETE
#define CONFIG_CMDLINE_EDITING

/* We set the max number of command args high to avoid HUSH bugs. */
#define CONFIG_SYS_MAXARGS		64

/* Console I/O Buffer Size */
#define CONFIG_SYS_CBSIZE		512
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE +\
					sizeof(CONFIG_SYS_PROMPT) + 16)
/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

#endif	/* __BUR_CFG_COMMON_H__ */

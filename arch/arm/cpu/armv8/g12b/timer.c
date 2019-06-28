
/*
 * arch/arm/cpu/armv8/txl/timer.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <asm/arch/secure_apb.h>
#include <asm/arch/timer.h>
#include <asm/types.h>
#include <serial.h>

#define P_EE_TIMER_E		P_ISA_TIMERE

uint32_t get_time(void)
{
	return readl(P_EE_TIMER_E);
}

void _udelay(unsigned int us)
{
#ifndef CONFIG_PXP_EMULATOR
	unsigned int t0 = get_time();

	while (get_time() - t0 <= us)
		;
#endif
}

#ifdef BL33_BOOT_TIME_PROBE
struct{
const char *pInfo;
unsigned int nTM;
}g_TMArray[100];

int g_nTMCnt = 0;


void TE_time(const char *szInfo)
{
	int i = 0;
	int nFoundFlag = 0;

	for (i = 0;i< sizeof(g_TMArray)/sizeof(g_TMArray[0]);++i)
	{
		if (szInfo == g_TMArray[i].pInfo)
		{
			nFoundFlag = 1;
			break;
		}
	}

	if (!nFoundFlag)
	{
		g_TMArray[g_nTMCnt].pInfo =(void*)szInfo;
		g_TMArray[g_nTMCnt++].nTM   = get_time();
	}
	else
	{
		int nNow = get_time();
		int nUsed = nNow - g_TMArray[i].nTM;
		if (nUsed/1000 >= 50)
			printf("\nTE: %d : %s : used %d\n",nNow,szInfo,nUsed);
		g_nTMCnt--;

		for ( ; i< sizeof(g_TMArray)/sizeof(g_TMArray[0]);++i)
		{
			g_TMArray[i] = g_TMArray[i+1];
			g_TMArray[i+1].pInfo= 0;
			g_TMArray[i+1].nTM = 0;
		}
	}
	printf("\n");
}
#endif

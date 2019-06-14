/*
* Copyright (C) 2017 Amlogic, Inc. All rights reserved.
* *
This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* *
This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
* *
You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
* *
Description:
*/

#include <common.h>
#include <asm/arch/bl31_apis.h>
#include <asm/reboot.h>
#include <asm/arch/secure_apb.h>

#define DEBUG_RAMDUMP	0
unsigned int get_reboot_mode(void)
{
	uint32_t reboot_mode_val = ((readl(AO_SEC_SD_CFG15) >> 12) & 0xf);

	return reboot_mode_val;
}

void check_ramdump(void)
{
	unsigned long size = 0;
	unsigned long addr = 0;
	char *env;
	int reboot_mode;
	char env_cmd[128] = {0};

	env = getenv("ramdump_disable");
	if (env) {
		printf("%s,%s\n", __func__, env);
		if (!strcmp(env, "1")) {
			sprintf(env_cmd,
				"setenv bootargs ${bootargs} ramdump=%s",
				"disabled");
			run_command(env_cmd, 1);
			aml_set_reboot_reason(SET_REBOOT_REASON,
					      AMLOGIC_NORMAL_BOOT, 0, 0);
		#if DEBUG_RAMDUMP
			run_command("printenv bootargs", 1);
		#endif
			return;
		}
	}

	reboot_mode = get_reboot_mode();
	if ((reboot_mode == AMLOGIC_WATCHDOG_REBOOT ||
	     reboot_mode == AMLOGIC_KERNEL_PANIC)) {
		addr = readl(P_AO_SEC_GP_CFG12);
		size = readl(P_AO_SEC_GP_CFG13);
		printf("%s, addr:%lx, size:%lx\n", __func__, addr, size);
		if (addr && size) {
			/*
			 * TODO: Make sure address for fdt_high and initrd_high
			 * are suitable for all boards
			 *
			 * usually kernel load address is 0x010800000
			 * Make sure:
			 * (kernel image size + ramdisk size) <
			 * (initrd_high - 0x010800000)
			 * dts file size < (fdt_high - initrd_high)
			 */
			setenv("initrd_high", "0x04400000");
			setenv("fdt_high",    "0x04E00000");
			sprintf(env_cmd,
				"setenv bootargs ${bootargs} ramdump=%lx,%lx",
				addr, size);
			run_command(env_cmd, 1);
		#if DEBUG_RAMDUMP
			run_command("printenv bootargs", 1);
		#endif
		}
	}
	aml_set_reboot_reason(SET_REBOOT_REASON, AMLOGIC_KERNEL_PANIC, 0, 0);
}

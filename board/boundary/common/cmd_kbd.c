/*
 * Copyright (C) 2017, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <command.h>
#include <asm/arch/imx-regs.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <env.h>
#include <linux/delay.h>
#include "bd_common.h"
#ifdef CONFIG_TAMPER
void tamper_enable(struct snvs_regs *snvs)
{
	writel(0x41736166, &snvs->lppgdr);	/* power glitch magic value */
	writel(0x00800000, &snvs->lptgfcr);	/* Tamper Glitch Filter 1 En */
	writel(0x200, &snvs->lptdcr);		/* External Tampering 1 En */
	writel(0x20, &snvs->hpsvsr);		/* w1c, VIO5 */
	writel(0x40030208, &snvs->lpsr);	/* w1c */
	writel(0x20, &snvs->hpsvcr);		/* gpio1[0] will stop working */
	writel(0x20, &snvs->lpsvcr);		/* Security Violation 5 En */
}

void tamper_clear(struct snvs_regs *snvs)
{
	writel(0x41736166, &snvs->lppgdr);	/* power glitch magic value */
	writel(0, &snvs->lptgfcr);		/* disable Tamper Glitch Filter */
	writel(0, &snvs->lptdcr);		/* disable External Tampering */
	writel(0x20, &snvs->hpsvsr);		/* w1c, VIO5 */
	writel(0, &snvs->hpsvcr);		/* makes gpio1[0] work again */
	writel(0, &snvs->lpsvcr);		/* disable Security Vio */
	writel(0x40030208, &snvs->lpsr);	/* w1c */
}

static void tamper_assert(struct snvs_regs *snvs)
{
	tamper_enable(snvs);
	udelay(100);
	tamper_clear(snvs);
}

void check_tamper(void)
{
	struct snvs_regs *snvs = (struct snvs_regs *)SNVS_BASE_ADDR;
	const struct button_key *bb = board_buttons;
	unsigned short gp;

	while (1) {
		if (!bb->name)
			break;
		if (bb->tamper) {
			gp = bb->gpnum;
			if (gpio_get_value(gp) ^ bb->active_low)
				tamper_assert(snvs);
		}
		bb++;
	}
	tamper_clear(snvs);
}
#endif

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
#ifdef CONFIG_TAMPER
	struct snvs_regs *snvs = (struct snvs_regs *)SNVS_BASE_ADDR;
#endif
	int numpressed = 0;
	const struct button_key *bb = board_buttons;
	unsigned short gp;

	while (1) {
		if (!bb->name)
			break;
		gp = bb->gpnum;
#ifdef CONFIG_TAMPER
		if (gp == TAMPER_CHECK) {
			/* Tamper status */
			unsigned lpsr;

			/*
			 * We cannot leave tamper enabled all the time because
			 * it screws up gpio_0
			 */
			tamper_enable(snvs);
			udelay(100);
			lpsr = readl(&snvs->lpsr);

			if (lpsr & BIT(9)) {
				buf[numpressed++] = bb->ident;
			}
			tamper_clear(snvs);
		} else
#endif
		{
			if (gpio_get_value(gp) ^ bb->active_low) {
				buf[numpressed++] = bb->ident;
#ifdef CONFIG_TAMPER
				if (bb->tamper)
					tamper_assert(snvs);
#endif
			}
		}
		bb++;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

static int do_kbd(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[MAX_BUTTONS];
	int numpressed = read_keys(envvalue);

	env_set("keybd", envvalue);
	return numpressed == 0;
}

U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);

#ifdef CONFIG_PREBOOT
static char const kbd_magic_prefix[] = "key_magic";
static char const kbd_command_prefix[] = "key_cmd";

void board_preboot_keys(void)
{
	int numpressed;
	char keypress[MAX_BUTTONS];

	numpressed = read_keys(keypress);
	if (numpressed) {
		char *kbd_magic_keys = env_get("magic_keys");
		char *suffix;
		/*
		 * loop over all magic keys
		 */
		for (suffix = kbd_magic_keys; *suffix; ++suffix) {
			char *keys;
			char magic[sizeof(kbd_magic_prefix) + 1];
			sprintf(magic, "%s%c", kbd_magic_prefix, *suffix);
			keys = env_get(magic);
			if (keys) {
				if (!strcmp(keys, keypress))
					break;
			}
		}
		if (*suffix) {
			char cmd_name[sizeof(kbd_command_prefix) + 1];
			char *cmd;
			sprintf(cmd_name, "%s%c", kbd_command_prefix, *suffix);
			cmd = env_get(cmd_name);
			if (cmd) {
				env_set("preboot", cmd);
				return;
			}
		}
	}
}
#endif

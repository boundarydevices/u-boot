/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6x_pins.h>
#include <asm/arch/mx6dl_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_MX6Q
#include "pads.h"
#endif
#if defined(CONFIG_MX6DL) || defined(CONFIG_MX6S)
#define FOR_DL_SOLO
#include "pads.h"
#endif

int cpu_is_mx6q(void)
{
	return (get_cpu_rev() >> 12) == MXC_CPU_MX6Q;
}

#if defined(CONFIG_MX6DL) || defined(CONFIG_MX6S)
#ifdef CONFIG_MX6Q
#define GET_MX6_REF(ref) (cpu_is_mx6q() ? mx6q_##ref : mx6dl_solo_##ref)
#define IOMUX_SETUP(list)  iomux_setup(mx6q_##list, ARRAY_SIZE(mx6q_##list), \
		mx6dl_solo_##list, ARRAY_SIZE(mx6dl_solo_##list))

int iomux_setup(iomux_v3_cfg_t *mx6q_pad_list, int mx6q_pad_cnt,
		iomux_v3_cfg_t *mx6dl_solo_pad_list, int mx6dl_solo_pad_cnt)
{
	int mx6q = cpu_is_mx6q();
	iomux_v3_cfg_t *p =  mx6q ? mx6q_pad_list : mx6dl_solo_pad_list;
	int cnt = mx6q ? mx6q_pad_cnt : mx6dl_solo_pad_cnt;

	return imx_iomux_v3_setup_multiple_pads(p, cnt);
}

#else
#define GET_MX6_REF(ref) (mx6dl_solo_##ref)
#define IOMUX_SETUP(list)  imx_iomux_v3_setup_multiple_pads(	\
		mx6dl_solo_##list, ARRAY_SIZE(mx6dl_solo_##list))
#endif
#else

#define GET_MX6_REF(ref) (mx6q_##ref)
#define IOMUX_SETUP(list)  imx_iomux_v3_setup_multiple_pads(	\
		mx6q_##list, ARRAY_SIZE(mx6q_##list))
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static void setup_iomux_enet(void)
{
	gpio_direction_output(IMX_GPIO_NR(3, 23), 0);
	gpio_direction_output(IMX_GPIO_NR(6, 30), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 29), 1);
	IOMUX_SETUP(enet_pads1);
	gpio_direction_output(IMX_GPIO_NR(6, 24), 1);

	/* Need delay 10ms according to KSZ9021 spec */
	udelay(1000 * 10);
	gpio_set_value(IMX_GPIO_NR(3, 23), 1);

	IOMUX_SETUP(enet_pads2);
}

static void setup_iomux_uart(void)
{
	IOMUX_SETUP(uart1_pads);
	IOMUX_SETUP(uart2_pads);
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	IOMUX_SETUP(usb_pads);

	/* Reset USB hub */
	gpio_direction_output(IMX_GPIO_NR(7, 12), 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(7, 12), 1);

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
       {USDHC3_BASE_ADDR},
       {USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
       struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
       int ret;

       if (cfg->esdhc_base == USDHC3_BASE_ADDR) {
		gpio_direction_input(IMX_GPIO_NR(7, 0));
		ret = !gpio_get_value(IMX_GPIO_NR(7, 0));
       } else {
		gpio_direction_input(IMX_GPIO_NR(2, 6));
		ret = !gpio_get_value(IMX_GPIO_NR(2, 6));
       }

       return ret;
}

int board_mmc_init(bd_t *bis)
{
       s32 status = 0;
       u32 index = 0;

       for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
	       switch (index) {
	       case 0:
		       IOMUX_SETUP(usdhc3_pads);
		       break;
	       case 1:
		       IOMUX_SETUP(usdhc4_pads);
		       break;
	       default:
		       printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
		       return status;
	       }

	       status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
       }

       return status;
}
#endif

u32 get_board_rev(void)
{
	return 0x63000 ;
}

#ifdef CONFIG_MXC_SPI
void setup_spi(void)
{
	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
	IOMUX_SETUP(ecspi1_pads);
}
#endif

int board_phy_config(struct phy_device *phydev)
{
	/* min rx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
	/* min tx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
	/* max rx/tx clock delay, min rx/tx control */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_enet();

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return 0;
}

static void setup_buttons(void)
{
	IOMUX_SETUP(button_pads);
}

#ifdef CONFIG_CMD_SATA

int setup_sata(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;

	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
			IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
			|IOMUXC_GPR13_SATA_PHY_7_SATA2M
			|IOMUXC_GPR13_SATA_SPEED_3G
			|(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			|IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			|IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			|IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			|IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			|IOMUXC_GPR13_SATA_PHY_1_SLOW);

	return 0;
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();
	setup_buttons();

	return 0;
}

int board_init(void)
{
	struct i2c_pads_info *p = GET_MX6_REF(i2c_pad_info);

       /* address of boot parameters */
       gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &p[0]);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &p[1]);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &p[2]);

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

       return 0;
}

int checkboard(void)
{
	puts("Board: MX");
	puts(get_imx_type(get_cpu_rev() >> 12));
	puts("-Sabre Lite\n");
	return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"back",	IMX_GPIO_NR(2, 2),	'B'},
	{"home",	IMX_GPIO_NR(2, 4),	'H'},
	{"menu",	IMX_GPIO_NR(2, 1),	'M'},
	{"search",	IMX_GPIO_NR(2, 3),	'S'},
	{"volup",	IMX_GPIO_NR(7, 13),	'V'},
	{"voldown",	IMX_GPIO_NR(4, 5),	'v'},
};

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
	int i, numpressed = 0;
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!gpio_get_value(buttons[i].gpnum))
			buf[numpressed++] = buttons[i].ident;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

static int do_kbd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	int numpressed = read_keys(envvalue);
	setenv("keybd", envvalue);
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

static void preboot_keys(void)
{
	int numpressed;
	char keypress[ARRAY_SIZE(buttons)+1];
	numpressed = read_keys(keypress);
	if (numpressed) {
		char *kbd_magic_keys = getenv("magic_keys");
		char *suffix;
		/*
		 * loop over all magic keys
		 */
		for (suffix = kbd_magic_keys; *suffix; ++suffix) {
			char *keys;
			char magic[sizeof(kbd_magic_prefix) + 1];
			sprintf(magic, "%s%c", kbd_magic_prefix, *suffix);
			keys = getenv(magic);
			if (keys) {
				if (!strcmp(keys, keypress))
					break;
			}
		}
		if (*suffix) {
			char cmd_name[sizeof(kbd_command_prefix) + 1];
			char *cmd;
			sprintf(cmd_name, "%s%c", kbd_command_prefix, *suffix);
			cmd = getenv(cmd_name);
			if (cmd) {
				setenv("preboot", cmd);
				return;
			}
		}
	}
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif

int misc_init_r(void)
{
#ifdef CONFIG_PREBOOT
	preboot_keys();
#endif

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

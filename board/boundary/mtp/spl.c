/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
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
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/imx-common/boot_mode.h>

#include <i2c.h>
#include <spl.h>

#if 0
void board_init_f(ulong dummy)
{
#if 0
	arch_cpu_init();
	board_early_init_f();
	timer_init();
	preloader_console_init();

	print_cpuinfo();
	board_init_r(NULL, 0);
#endif
}
#endif

void spl_board_init(void)
{
#if 0
	int i;
	u32 const *regs ;
	int num_regs;
	unsigned char mac_address[6];
        imx_get_mac_from_fuse(0,mac_address);
	printf("ethaddr: %pM\n", mac_address);

	if (is_cpu_type(MXC_CPU_MX6Q)) {
#if 1
		regs = mx6q_1g;
		num_regs = ARRAY_SIZE(mx6q_1g);
#else
		regs = mx6q_2g;
		num_regs = ARRAY_SIZE(mx6q_2g);
#endif
	} else {
#if CONFIG_DDR_MB == 512
		regs = mx6dl_512m;
		num_regs = ARRAY_SIZE(mx6dl_512m);
printf("Configuring for 512MiB narrow memory bus\n");
#elif CONFIG_DDR_MB == 1024
		regs = mx6dl_1gn;
		num_regs = ARRAY_SIZE(mx6dl_1gn);
printf("Configuring for 1GiB narrow memory bus\n");
#elif CONFIG_DDR_MB == 2048
		regs = mx6dl_2g;
		num_regs = ARRAY_SIZE(mx6dl_2g);
printf("Configuring for 2GiB wide memory bus\n");
#endif
	}
	for (i=0; i < num_regs; i+=2) {
		writel(regs[i+1],regs[i]);
	}
        dram_init();
#endif
	printf("%s\n", __func__);
}

u32 spl_boot_device(void)
{
	printf("%s\n", __func__);
#if 0
	unsigned reg;
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	printf("%s: sbmr1 == 0x%08x\n", __func__, psrc->sbmr1);
	printf("%s: gpr9  == 0x%08x\n", __func__, psrc->gpr9);
	printf("%s: gpr10 == 0x%08x\n", __func__, psrc->gpr10);
	return BOOT_DEVICE_USB;
#endif
#if 1
	return BOOT_DEVICE_SPI;
#endif
}

#if 0
void spl_usb_load_image(void)
{
	boot_mode_apply(MAKE_CFGVAL(0x01, 0x00, 0x00, 0x00));
	reset_cpu(0);
}

#endif

#
# Copyright 2016 Freescale Semiconductor
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += nitrogen8m.o mmc.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_IMX8M_LPDDR4
obj-y += lpddr4_timing.o
else
obj-y += ddr/ddr_init.o ddr/ddrphy_train.o ddr/helper.o
endif
endif

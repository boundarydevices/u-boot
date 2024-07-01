#
# Copyright 2018 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += nitrogen8mm_som.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_IMX8M_4G_LPDDR4
obj-y += lpddr4_timing_4g.o
else ifdef CONFIG_IMX8M_1G_MEMORY
obj-y += lpddr4_timing_1g.o
else
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_2g.o
endif
endif

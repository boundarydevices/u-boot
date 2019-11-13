#
# Copyright 2018 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += nitrogen8mm.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX8MM) += lpddr4_timing.o
obj-$(CONFIG_IMX8MN) += lpddr4_timing_8mn.o
endif

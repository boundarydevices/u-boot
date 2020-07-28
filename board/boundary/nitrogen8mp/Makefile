#
# Copyright 2020 Boundary Devices
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += nitrogen8mp.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
endif

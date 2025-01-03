#
# Copyright 2025 Ezurio
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx93_nvc.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_TARGET_IMX93_NVC) += lpddr4_timing.o
endif

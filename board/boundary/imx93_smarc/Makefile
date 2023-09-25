#
# Copyright 2022 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx93_bd_smarc.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_TARGET_IMX93_BD_SMARC) += lpddr4_timing_2400mts.o
endif

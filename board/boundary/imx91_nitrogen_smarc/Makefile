#
# Copyright 2024 Ezurio LLC
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx91_nitrogen_smarc.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_TARGET_IMX91_NITROGEN_SMARC) += lpddr4_timing.o
endif

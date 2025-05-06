#
# Copyright 2025 Ezurio
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx8mp_nitrogen_smarc.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_IMX8M_8G_LPDDR4
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_8g.o
else ifdef CONFIG_IMX8M_4G_LPDDR4
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_4g.o
else
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_2gr0.o
endif
endif

# SPDX-License-Identifier: GPL-2.0+

obj-y   += imx8ulp_evk.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o lpddr4_timing.o
ifndef CONFIG_IMX8ULP_DRAM
obj-y += ddr_init.o
endif
endif

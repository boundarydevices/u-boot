# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.

MINIMAL=

ifdef CONFIG_SPL_BUILD
ifndef CONFIG_TPL_BUILD
ifdef CONFIG_SPL_INIT_MINIMAL
MINIMAL=y
endif
endif
endif

ifdef MINIMAL
# necessary to create built-in.o
obj- := __dummy__.o
else
# include i2c_common.o once if either VID or FSL_USE_PCA9547_MUX
I2C_COMMON=
ifdef CONFIG_VID
I2C_COMMON=y
endif
ifdef CONFIG_FSL_USE_PCA9547_MUX
I2C_COMMON=y
endif

obj-$(I2C_COMMON)		+= i2c_common.o
obj-$(CONFIG_FSL_USE_PCA9547_MUX)		+= i2c_mux.o
obj-$(CONFIG_$(SPL_)VID)	+= vid.o

ifneq (,$(filter $(SOC), mx6 mx7 mx7ulp imx8 imx8m imx8ulp imx9))
obj-y				+= mmc.o
endif
obj-$(CONFIG_MXC_EPDC)		+= epdc_setup.o
ifdef CONFIG_FSL_FASTBOOT
obj-${CONFIG_ANDROID_RECOVERY} += recovery_keypad.o
endif

obj-$(CONFIG_CHAIN_OF_TRUST) += fsl_chain_of_trust.o

endif

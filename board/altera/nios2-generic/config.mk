#
# (C) Copyright 2005, Psyent Corporation <www.psyent.com>
# Scott McNutt <smcnutt@psyent.com>
#
# SPDX-License-Identifier:	GPL-2.0+
#

PLATFORM_CPPFLAGS += -mno-hw-div -mno-hw-mul

ifeq ($(debug),1)
PLATFORM_CPPFLAGS += -DDEBUG
endif

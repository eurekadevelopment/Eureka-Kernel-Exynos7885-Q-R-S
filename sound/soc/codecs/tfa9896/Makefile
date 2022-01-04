# NXP audio amplifier - TFA9896
# Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
# Copyright 2020 GOODIX 
#
#This program is free software; you can redistribute it and/or modify
#it under the terms of the GNU General Public License version 2 as
#published by the Free Software Foundation.

# make sure that the environment variables ARCH and CROSS_COMPILE
# are set for your architecture and cross compiler.
# And that KDIR points to the kernel to build against.
#
# e.g.:
# export ARCH=arm
# export CROSS_COMPILE=arm-linux-gnueabihf-
# export KDIR=../linux

TFA_VERSION              =tfa9896
TFA_SRC_DIR              =sound/soc/codecs/$(TFA_VERSION)

# In case of out of tree build, build as a module
# (when build inside kernel we will not enter this directory and this will have no effect)
#ifeq ($(CONFIG_SND_SOC_TFA98XX),)
#CONFIG_SND_SOC_TFA98XX := m
#endif

EXTRA_CFLAGS += -I$(TFA_SRC_DIR)/inc

# objs
snd-soc-tfa98xx-objs += tfa98xx.o
snd-soc-tfa98xx-objs += tfa_container.o
snd-soc-tfa98xx-objs += tfa_dsp.o
snd-soc-tfa98xx-objs += tfa_init.o

# built-in driver
obj-y	+= snd-soc-tfa98xx.o

CFLAGS_tfa_dsp.o += $(call cc-disable-warning, enum-conversion)

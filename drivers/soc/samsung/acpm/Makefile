#
# Makefile for ACPM.
#

obj-$(CONFIG_EXYNOS_ACPM)    += acpm.o acpm_ipc.o acpm_mfd.o
KBUILD_CFLAGS   += -Os
CFLAGS_acpm.o += $(DISABLE_LTO_CLANG)

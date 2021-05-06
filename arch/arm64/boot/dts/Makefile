# SPDX-License-Identifier: GPL-2.0
dtb-y += exynos/dtbo/exynos7885.dtb

dtb-$(CONFIG_MACH_EXYNOS7885_A40_OPEN) := \
	exynos/dtbo/exynos7904-a40_eur_open_00.dtbo \
	exynos/dtbo/exynos7904-a40_eur_open_01.dtbo

ifeq ($(CONFIG_MACH_EXYNOS7885_A40_OPEN),y)
always         := $(dtb-y) $(dtbo-y)
clean-files    := *.dtb *.dtbo
endif

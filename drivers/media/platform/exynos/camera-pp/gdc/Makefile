#
# Copyright (c) 2016 Samsung Electronics Co., Ltd.
#		http://www.samsung.com
#
# Licensed under GPLv2
#

gdc-objs := camerapp-gdc-core.o \
		camerapp-hw-api-gdc-v100.o \
		camerapp-gdc-grid.o

obj-$(CONFIG_VIDEO_EXYNOS_CAMREA_POSTPROCESS_GDC)	+= gdc.o

ccflags-y += -Idrivers/media/platform/exynos/camera-pp/
ccflags-y += -Idrivers/media/platform/exynos/fimc-is2/include

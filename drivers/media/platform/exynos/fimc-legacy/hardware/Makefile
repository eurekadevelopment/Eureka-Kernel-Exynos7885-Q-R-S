obj-$(CONFIG_USE_DIRECT_IS_CONTROL) += fimc-is-hw-control.o \
		fimc-is-hw-3aa.o \
		fimc-is-hw-isp.o \
		fimc-is-hw-tpu.o \
		fimc-is-hw-vra.o \
		fimc-is-hw-dm.o

obj-$(CONFIG_CAMERA_FIMC_SCALER_USE) += fimc-is-hw-scp.o
obj-$(CONFIG_CAMERA_MC_SCALER_VER1_USE) += fimc-is-hw-mcscaler-v1.o
obj-$(CONFIG_CAMERA_MC_SCALER_VER2_USE) += fimc-is-hw-mcscaler-v2.o
ifeq ($(CONFIG_CAMERA_MC_SCALER_VER2_USE),y)
obj-$(CONFIG_CAMERA_TDNR_VER1_USE) += fimc-is-hw-tdnr-v1.o
obj-$(CONFIG_CAMERA_TDNR_VER2_USE) += fimc-is-hw-tdnr-v2.o
endif
obj-$(CONFIG_DCP_V1_0) += fimc-is-hw-dcp.o
obj-$(CONFIG_SRDZ_V1_0) += fimc-is-hw-srdz.o

obj-$(CONFIG_USE_DIRECT_IS_CONTROL) += api/
obj-$(CONFIG_EXYNOS_FIMC_IS_HARDWARE) += api/

EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-legacy/
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-legacy/include
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-legacy/interface
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-legacy/hardware
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-legacy/hardware/api
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-legacy/vendor

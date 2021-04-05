#Android makefile to build kernel as a part of Android Build

ifeq ($(KERNEL_DEFCONFIG),)
$(error KERNEL_DEFCONFIG must be set as environment variable)
endif

ifeq ($(KERNEL_DEFCONFIG), universal7885_FHD_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7885-universal7885_FHD_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7905_FHD_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7905-universal7905_FHD_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7905_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7905-universal7905_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7904_FHD_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7904-universal7904_FHD_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7904_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7904-universal7904_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7884_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7884-universal7884_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7884_FHD_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7884-universal7884_FHD_P_Treble.dts
else ifeq ($(KERNEL_DEFCONFIG), universal7884B_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7884b-universal7884b_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7884B_FHD_P_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7884b-universal7884b_FHD_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7885_FHD_Q_MR_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7885-universal7885_FHD_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7885_Q_MR_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7885-universal7885_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7905_FHD_Q_MR_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7905-universal7905_FHD_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7905_Q_MR_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7905-universal7905_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7904_FHD_Q_MR_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7904-universal7904_FHD_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7904_Q_MR_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7904-universal7904_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7884B_FHD_Q_MR_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7884b-universal7884b_FHD_P_Treble.dtb
else ifeq ($(KERNEL_DEFCONFIG), universal7884B_Q_MR_Treble_defconfig)
TARGET_KERNEL_DTB := exynos7884b-universal7884b_P_Treble.dtb
else
TARGET_KERNEL_DTB := exynos7885-universal7885_P_Treble.dtb
endif

ifeq ($(INSTALLED_KERNEL_TARGET),)
INSTALLED_KERNEL_TARGET := $(PRODUCT_OUT)/kernel
INSTALLED_DTBO_TARGET := $(PRODUCT_OUT)/dtbo.img
INSTALLED_DTB_TARGET := $(PRODUCT_OUT)/dtb.img
endif

TARGET_KERNEL_ARCH := $(strip $(TARGET_KERNEL_ARCH))
ifeq ($(TARGET_KERNEL_ARCH),)
KERNEL_ARCH := arm64
else
KERNEL_ARCH := $(TARGET_KERNEL_ARCH)
endif

ifeq ($(CROSS_COMPILE),)
KERNEL_CROSS_COMPILE := aarch64-linux-android-
else
KERNEL_CROSS_COMPILE := $(CROSS_COMPILE)
endif

ifeq ($(TARGET_PREBUILT_KERNEL),)

TARGET_KERNEL_SOURCE := kernel/$(TARGET_KERNEL)
KERNEL_CONFIG := $(TARGET_KERNEL_SOURCE)/.config
KERNEL_BOOT := $(TARGET_KERNEL_SOURCE)/arch/$(KERNEL_ARCH)/boot
KERNEL_BIN := $(KERNEL_BOOT)/Image
KERNEL_DTB_DIR := $(KERNEL_BOOT)/dts/exynos/dtbo
KERNEL_DTB := $(KERNEL_DTB_DIR)/exynos7885.dtb
ifeq ($(KERNEL_DEFCONFIG), universal7885_FHD_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7885_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7905_FHD_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7905_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7905_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7905_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7904_FHD_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7904_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7904_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7904_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7884_FHD_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7884_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7884_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7884_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7884B_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7884B_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7884B_FHD_P_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7884B_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7885_FHD_Q_MR_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7885_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7885_Q_MR_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7885_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7905_FHD_Q_MR_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7905_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7905_Q_MR_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7905_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7904_FHD_Q_MR_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7904_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7904_Q_MR_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7904_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7884B_FHD_Q_MR_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7884B_FHD_dtboimg.cfg
else ifeq ($(KERNEL_DEFCONFIG), universal7884B_Q_MR_Treble_defconfig)
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7884B_dtboimg.cfg
else
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos7885_dtboimg.cfg
endif
MKDTIMG := $(HOST_OUT_EXECUTABLES)/mkdtimg

ifeq ($(KERNEL_DEFCONFIG),)
$(error Kernel configuration not defined, cannot build kernel)
else

ifeq ($(N_KERNEL_BUILD_THREAD),)
N_KERNEL_BUILD_THREAD := 1
endif

TARGET_PREBUILT_KERNEL := $(KERNEL_BIN)

.PHONY: remove-bins
remove-bin:
	$(hide) echo "Clean Up prebuilts"
	rm -f $(KERNEL_CONFIG)
	rm -f $(KERNEL_BIN)
	rm -f $(KERNEL_DTB)
	rm -f $(INSTALLED_KERNEL_TARGET)
	rm -f $(INSTALLED_DTBO_TARGET)
	rm -f $(INSTALLED_DTB_TARGET)

.PHONY: kernel
kernel: $(KERNEL_BIN)

.PHONY: kernel-distclean
kernel-distclean:
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) distclean

$(KERNEL_CONFIG): remove-bins
	$(hide) echo "make $(KERNEL_DEFCONFIG)"
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_DEFCONFIG)

$(KERNEL_BIN): $(KERNEL_CONFIG)
	$(hide) echo "Building kernel..."
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) -j$(N_KERNEL_BUILD_THREAD)

$(INSTALLED_KERNEL_TARGET): $(KERNEL_BIN) $(MKDTIMG)
	cp $(KERNEL_BIN) $(INSTALLED_KERNEL_TARGET)
	cp $(KERNEL_DTB) $(INSTALLED_DTB_TARGET)
	$(hide) echo "Building DTBO..."
	ln -sf $(TARGET_KERNEL_SOURCE)/arch
	$(MKDTIMG) cfg_create $(INSTALLED_DTBO_TARGET) $(KERNEL_DTBO_CFG)
	rm -f arch

endif #TARGET_PREBUILT_KERNEL
endif #KERNEL_DEFCONFIG

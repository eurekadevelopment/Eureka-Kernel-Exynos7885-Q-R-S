# ----------------------------------------------------------------------------
# FILE: Makefile
#
# PURPOSE:
#       Build instructions for SCSC Wlan driver.
#
# Copyright (c) 2016 Samsung Electronics Co., Ltd. All rights reserved
# ----------------------------------------------------------------------------

# Needed since this subdir is symlinked in the main Kernel tree
# without this our samsung subdir is NOT cleaned.
clean-files := *.o *.ko

obj-$(CONFIG_SCSC_WLAN) += scsc_wlan.o
scsc_wlan-y += hip4.o
scsc_wlan-y += mbulk.o

# ----------------------------------------------------------------------------
# HIP4 sampler
# ----------------------------------------------------------------------------
ifeq ($(CONFIG_SCSC_WLAN_HIP4_PROFILING),y)
scsc_wlan-y += hip4_sampler.o
endif

# Upper driver
scsc_wlan-y += dev.o
scsc_wlan-y += cfg80211_ops.o
scsc_wlan-y += netif.o
scsc_wlan-y += rx.o
scsc_wlan-y += tx.o
scsc_wlan-y += ba.o

# SAPs
scsc_wlan-y += sap_mlme.o
scsc_wlan-y += sap_ma.o
scsc_wlan-y += sap_dbg.o
scsc_wlan-y += sap_test.o

# ----------------------------------------------------------------------------
# Common Driver Files
# ----------------------------------------------------------------------------
scsc_wlan-y += mib.o
scsc_wlan-y += mib_text_convert.o
scsc_wlan-y += debug.o
scsc_wlan-y += debug_frame.o
scsc_wlan-y += procfs.o
scsc_wlan-y += mgt.o
scsc_wlan-y += mlme.o
scsc_wlan-y += udi.o
scsc_wlan-y += log_clients.o
scsc_wlan-y += src_sink.o
scsc_wlan-y += fw_test.o
scsc_wlan-y += cac.o
scsc_wlan-y += scsc_wifi_fcq.o
scsc_wlan-y += ioctl.o
scsc_wlan-y += wakelock.o

# ----------------------------------------------------------------------------
# ACM
# ----------------------------------------------------------------------------
ifeq ($(CONFIG_SCSC_WLAN_ACM_API),y)
scsc_wlan-y += acm_api.o
endif

# ----------------------------------------------------------------------------
# Building for Hardware
# ----------------------------------------------------------------------------
scsc_wlan-y += hip.o
scsc_wlan-y += cm_if.o

ifeq ($(CONFIG_SCSC_WLAN_KIC_OPS),y)
scsc_wlan-y += kic.o
endif
ccflags-$(CONFIG_SCSC_WLAN_KIC_OPS) += -DCONFIG_SCSC_WLAN_KIC_OPS

ifeq ($(CONFIG_SCSC_WLAN_GSCAN_ENABLE),y)
scsc_wlan-y += nl80211_vendor.o
ccflags-y += -DCONFIG_SCSC_WLAN_GSCAN_ENABLE
endif
ifeq ($(CONFIG_SCSC_WIFI_NAN_ENABLE),y)
scsc_wlan-$(CONFIG_SCSC_WLAN) += nl80211_vendor_nan.o
scsc_wlan-$(CONFIG_SCSC_WLAN) += mlme_nan.o
endif

ifeq ($(CONFIG_SCSC_WLAN_KEY_MGMT_OFFLOAD),y)
ccflags-y += -DCONFIG_SCSC_WLAN_KEY_MGMT_OFFLOAD
endif
# ---------------------------------------------------------------------------
# Include
# ---------------------------------------------------------------------------
ccflags-y +=  -Idrivers/misc/samsung/scsc
ccflags-y +=  -Iinclude/scsc
ccflags-y += $(CONFIG_SAMSUNG_MAXWELL_EXTRA)
ccflags-y += -I$(src)

# ----------------------------------------------------------------------------
# Wlan configuration
# ----------------------------------------------------------------------------
ccflags-$(CONFIG_SCSC_WLAN_DEBUG)                                         += -DCONFIG_SCSC_WLAN_DEBUG
ccflags-$(CONFIG_SCSC_WLAN_SKB_TRACKING)                                  += -DCONFIG_SCSC_WLAN_SKB_TRACKING
ccflags-$(CONFIG_SCSC_WLAN_OFFLINE_TRACE)                                 += -DCONFIG_SCSC_WLAN_OFFLINE_TRACE
ccflags-$(CONFIG_SCSC_WLAN_OFFLINE_DATA_PLANE_PROFILE_TRACE)              += -DCONFIG_SCSC_WLAN_OFFLINE_DATA_PLANE_PROFILE_TRACE
ccflags-$(CONFIG_SCSC_WLAN_OFFLINE_TX_TRACE)                              += -DCONFIG_SCSC_WLAN_OFFLINE_TX_TRACE
ccflags-$(CONFIG_SCSC_WLAN_HIP_PSCHED_AMSDU)                              += -DCONFIG_SCSC_WLAN_HIP_PSCHED_AMSDU
ccflags-$(CONFIG_SCSC_WLAN_RX_NAPI)                                       += -DCONFIG_SCSC_WLAN_RX_NAPI
ccflags-$(CONFIG_SCSC_WLAN_RX_NAPI_GRO)                                   += -DCONFIG_SCSC_WLAN_RX_NAPI_GRO
ccflags-$(CONFIG_SCSC_WLAN_HIP_SUPPORT_SCATTER_GATHER_API)                += -DCONFIG_SCSC_WLAN_HIP_SUPPORT_SCATTER_GATHER_API
ccflags-$(CONFIG_SCSC_WLAN_WES_NCHO)                                      += -DCONFIG_SCSC_WLAN_WES_NCHO
ccflags-$(CONFIG_SCSC_WLAN_MUTEX_DEBUG)                                   += -DCONFIG_SCSC_WLAN_MUTEX_DEBUG
ccflags-$(CONFIG_SCSC_WLAN_BLOCK_IPV6)					  += -DCONFIG_SCSC_WLAN_BLOCK_IPV6
ccflags-$(CONFIG_SCSC_WLAN_DISABLE_NAT_KA)				  += -DCONFIG_SCSC_WLAN_DISABLE_NAT_KA

ccflags-y += -DCONFIG_SCSC_WLAN_MAX_INTERFACES=$(CONFIG_SCSC_WLAN_MAX_INTERFACES)

# Android specific build options
ccflags-$(CONFIG_SCSC_WLAN_ANDROID) += -DCONFIG_SCSC_WLAN_ANDROID

# Android Platform version
ifneq ($(ANDROID_VERSION),)
ANDROID_PLATFORM_VERSION=$(shell expr $(ANDROID_VERSION) / 10000)
ccflags-y += -DSCSC_SEP_VERSION=$(ANDROID_PLATFORM_VERSION)
$(info ANDROID_VERSION is $(ANDROID_VERSION))
else
ifneq ($(PLATFORM_VERSION),)
ANDROID_PLATFORM_VERSION := $(basename $(PLATFORM_VERSION))
ccflags-y += -DSCSC_SEP_VERSION=$(ANDROID_PLATFORM_VERSION)
$(info PLATFORM_VERSION is $(PLATFORM_VERSION))
endif
endif
$(info ANDROID_PLATFORM_VERSION is $(ANDROID_PLATFORM_VERSION))

## See drivers/misc/samsung/scsc/ Makefile for an explanation of the reasons
## of the following ifeq/else
ifeq ($(CONFIG_SCSC_DEBUG), m)
ccflags-y		 						+= -DCONFIG_SCSC_PRINTK
else
ccflags-$(CONFIG_SCSC_DEBUG) 						+= -DCONFIG_SCSC_PRINTK
endif

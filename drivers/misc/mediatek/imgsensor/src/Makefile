# SPDX-License-Identifier: GPL-2.0

$(info heaven CONFIG_TCT_CAM_DRIVER_SUPPORT=$(CONFIG_TCT_CAM_DRIVER_SUPPORT))
ifeq ($(CONFIG_TCT_CAM_DRIVER_SUPPORT),y)
subdir-ccflags-y += -DTCT_CAM_DRIVER_SUPPORT
endif

subdir-ccflags-y += -I$(srctree)/drivers/i2c/busses/ \
                    -I$(srctree)/drivers/misc/mediatek/ccu/inc

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

MTK_PLATFORM_PATH_ROOT := $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)
MTK_PROJECT_PATH_ROOT := $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)

ifeq ($(CONFIG_MACH_MT6781),y)
CONFIG_MTK_PLATFORM_SENSOR := mt6781
else
CONFIG_MTK_PLATFORM_SENSOR := $(CONFIG_MTK_PLATFORM)
endif
ifeq ($(CONFIG_ARCH_MTK_PROJECT), "odintmo_k419")
subdir-ccflags-y += -DCONFIG_MTK_ODIN5GTMO_K419_CAMERA
endif
subdir-ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/camera/camera

MTK_DRIVER_PATH  := $(srctree)/drivers/misc/mediatek

subdir-ccflags-y += -I$(MTK_DRIVER_PATH)/camera/$(MTK_PLATFORM) \
             -I$(MTK_DRIVER_PATH)/imgsensor/inc \
             -I$(MTK_DRIVER_PATH)/cam_cal/inc \
             -I$(MTK_DRIVER_PATH)/smi \
             -I$(MTK_DRIVER_PATH)/include/mt-plat/$(MTK_PLATFORM)/include/mach \
             -I$(MTK_DRIVER_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM_SENSOR)

CUSTOM_KERNEL_FLAGS := $(addprefix -D, $(shell echo \
             $(CONFIG_CUSTOM_KERNEL_IMGSENSOR) | tr a-z A-Z))
subdir-ccflags-y += $(CUSTOM_KERNEL_FLAGS)

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM_SENSOR))/

# begin modified by tct-hq/yzheng4 in 2022-11-25
#Begin liujunyi for passat 20211126
ifeq ($(CONFIG_ARCH_MTK_PROJECT), "luna84gvzw")
subdir-ccflags-y += -DCONFIG_MTK_LUNA84GVZW_CAMERA
endif
# end modified by tct-hq/yzheng4 in 2022-11-25

#Begin liujunyi for passat 20211126
ifeq ($(CONFIG_ARCH_MTK_PROJECT), "passat")
subdir-ccflags-y += -DCONFIG_MTK_PASSAT_CAMERA
endif
#End liujunyi for passat 20211126
ifeq ($(CONFIG_TCT_CAMERA_PROJECT_BUFFALOTMO),y)
subdir-ccflags-y += -DTCT_CAMERA_PROJECT_BUFFALOTMO
endif
#Begin modified by chengyixuan for CIVICPL-3060 on 2022-07-16
ifeq ($(CONFIG_ARCH_MTK_PROJECT), "civic_s")
subdir-ccflags-y += -DCONFIG_MTK_CIVIC_CAMERA
endif

ifeq ($(CONFIG_ARCH_MTK_PROJECT), "civic_plus")
subdir-ccflags-y += -DCONFIG_MTK_CIVIC_PLUS_CAMERA
endif
#End modified by chengyixuan for CIVICPL-3060 on 2022-07-16
#Begin zihao.li for [Task][BUTF-4165] buffalotf camera bring up on 20220106
ifeq ($(CONFIG_TCT_CAMERA_PROJECT_BUFFALOTF),y)
subdir-ccflags-y += -DTCT_CAMERA_PROJECT_BUFFALOTF
endif
#End   zihao.li for [Task][BUTF-4165] buffalotf camera bring up on 20220106


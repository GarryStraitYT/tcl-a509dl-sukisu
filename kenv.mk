# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2019 MediaTek Inc.

KERNEL_ENV_PATH := $(call my-dir)
KERNEL_ROOT_DIR := $(PWD)

define touch-kernel-image-timestamp
if [ -e $(1) ] && [ -e $(2) ] && cmp -s $(1) $(2); then \
 echo $(2) has no change;\
 mv -f $(1) $(2);\
else \
 rm -f $(1);\
fi
endef

# '\\' in command is wrongly replaced to '\\\\' in kernel/out/arch/arm/boot/compressed/.piggy.xzkern.cmd
define fixup-kernel-cmd-file
if [ -e $(1) ]; then cp $(1) $(1).bak; sed -e 's/\\\\\\\\/\\\\/g' < $(1).bak > $(1); rm -f $(1).bak; fi
endef

ifneq ($(strip $(TARGET_NO_KERNEL)),true)
  KERNEL_DIR := $(KERNEL_ENV_PATH)
  mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
  current_dir := $(notdir $(patsubst %/,%,$(dir $(mkfile_path))))

  kernel_build_config_suffix := .mtk
  ifeq ($(KERNEL_TARGET_ARCH),arm64)
    kernel_build_config_suffix := $(kernel_build_config_suffix).aarch64
    ifeq ($(strip $(TARGET_KERNEL_USE_CLANG)),true)
    else
      kernel_build_config_suffix := $(kernel_build_config_suffix).gcc
    endif
  else
    kernel_build_config_suffix := $(kernel_build_config_suffix).arm
    ifeq ($(strip $(TARGET_KERNEL_USE_CLANG)),true)
    else
      $(error TARGET_KERNEL_USE_CLANG is not set)
    endif
  endif
  ifeq ($(PLATFORM_VERSION),Tiramisu)
    kernel_build_config_suffix := $(kernel_build_config_suffix).tiramisu
  endif
  include $(current_dir)/build.config$(kernel_build_config_suffix)

  ARGS := CROSS_COMPILE=$(CROSS_COMPILE)
  # Begin added by bitao.xiong for task-11412249 on 2021-08-09
  ifeq ($(strip $(TARGET_BUILD_MMITEST)),true)
  ARGS += TARGET_BUILD_MMITEST=$(TARGET_BUILD_MMITEST)
  endif
  ifeq ($(strip $(TARGET_BUILD_CERTIFICATION)),true)
  ARGS += TARGET_BUILD_CERTIFICATION=$(TARGET_BUILD_CERTIFICATION)
  endif
  ifeq ($(strip $(DISABLE_TEMPERATURE_DETECTION_AND_THERMAL_POLICY)),true)
  ARGS += DISABLE_TEMPERATURE_DETECTION_AND_THERMAL_POLICY=$(DISABLE_TEMPERATURE_DETECTION_AND_THERMAL_POLICY)
  endif
  # End added by bitao.xiong for task-11412249 on 2021-08-09
  ifeq ($(strip $(TCL_THERMAL_DEBUG)),true)
  ARGS += TCL_THERMAL_DEBUG=$(TCL_THERMAL_DEBUG)
  endif
  # Begin added by qiaozhen.li for task-11468129 20210901
  ifeq ($(strip $(TARGET_BUILD_ENDURANCE)),true)
  ARGS += TARGET_BUILD_ENDURANCE=$(TARGET_BUILD_ENDURANCE)
  endif
  # End added by qiaozhen.li for task-11468129 20210901

  # Begin added by bing-zhang for task-11582209 20211015
  ifeq ($(strip $(TCL_SECURE_BOOT_FAILURE)),true)
  ARGS += TCL_SECURE_BOOT_FAILURE=$(TCL_SECURE_BOOT_FAILURE)
  endif
  # End added by bing-zhang for task-11582209 20211015

  # Begin added by bitao.xiong for task-11672350 2021-11-15
  ifeq ($(strip $(TARGET_BUILD_IEEE1725)),true)
  ARGS += TARGET_BUILD_IEEE1725=$(TARGET_BUILD_IEEE1725)
  endif
  # End added by bitao.xiong for task-11672350 2021-11-15

  # Begin added by bitao.xiong for ENCORETF-42 on 2022-07-25
  ifeq ($(strip $(TCT_TARGET_GCF)),true)
    ARGS += TCT_TARGET_GCF=$(TCT_TARGET_GCF)
  endif
  # End added by bitao.xiong for ENCORETF-42 on 2022-07-25

  #Add-start by baiwei.peng for ENCOREVZW-8180 on 2022/11/17
  ifeq ($(strip $(TCT_TARGET_OP)),VZW)
  ifeq ($(strip $(TARGET_BUILD_CERTIFICATION)),true)
    ARGS += TARGET_BUILD_USBIF_COMPLIANCE=$(TARGET_BUILD_USBIF_COMPLIANCE)
  endif
  endif
  #Add-start by baiwei.peng for ENCOREVZW-8180 on 2022/11/17

  ifneq ($(LLVM),)
    ARGS += LLVM=1
    ifneq ($(filter-out false,$(USE_CCACHE)),)
      CCACHE_EXEC ?= /usr/bin/ccache
      CCACHE_EXEC := $(abspath $(wildcard $(CCACHE_EXEC)))
    else
      CCACHE_EXEC :=
    endif
    ifneq ($(CCACHE_EXEC),)
      ARGS += CCACHE_CPP2=yes CC='$(CCACHE_EXEC) clang'
    else
      ARGS += CC=clang
    endif
    ifneq ($(LLVM_IAS),)
      ARGS += LLVM_IAS=$(LLVM_IAS)
    endif
    ifeq ($(HOSTCC),)
      ifneq ($(CC),)
        ARGS += HOSTCC=$(CC)
      endif
    else
      ARGS += HOSTCC=$(HOSTCC)
    endif
    ifneq ($(LD),)
      ARGS += LD=$(LD) HOSTLD=$(LD)
      ifneq ($(suffix $(LD)),)
        ARGS += HOSTLDFLAGS=-fuse-ld=$(subst .,,$(suffix $(LD)))
      endif
    endif
    ifneq ($(LD_LIBRARY_PATH),)
      ARGS += LD_LIBRARY_PATH=$(KERNEL_ROOT_DIR)/$(LD_LIBRARY_PATH)
    endif
  endif

  TARGET_KERNEL_CROSS_COMPILE := $(KERNEL_ROOT_DIR)/$(LINUX_GCC_CROSS_COMPILE_PREBUILTS_BIN)/$(CROSS_COMPILE)

  ifeq ($(wildcard $(TARGET_PREBUILT_KERNEL)),)
    KERNEL_OUT ?= $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
    KERNEL_ROOT_OUT := $(if $(filter /% ~%,$(KERNEL_OUT)),,$(KERNEL_ROOT_DIR)/)$(KERNEL_OUT)
    ifeq ($(KERNEL_TARGET_ARCH), arm64)
        KERNEL_ZIMAGE_OUT := $(KERNEL_OUT)/arch/$(KERNEL_TARGET_ARCH)/boot/Image.gz
        KERNEL_DTB_TARGET := $(KERNEL_OUT)/arch/$(KERNEL_TARGET_ARCH)/boot/dts/mediatek/$(TARGET_BOARD_PLATFORM).dtb
    else
        KERNEL_ZIMAGE_OUT := $(KERNEL_OUT)/arch/$(KERNEL_TARGET_ARCH)/boot/zImage
        KERNEL_DTB_TARGET := $(KERNEL_OUT)/arch/$(KERNEL_TARGET_ARCH)/boot/dts/$(TARGET_BOARD_PLATFORM).dtb
    endif

    INSTALLED_MTK_DTB_TARGET := $(BOARD_PREBUILT_DTBIMAGE_DIR)/mtk_dtb
    BUILT_KERNEL_TARGET := $(KERNEL_ZIMAGE_OUT).bin
    INSTALLED_KERNEL_TARGET := $(PRODUCT_OUT)/kernel
    TARGET_KERNEL_CONFIG := $(KERNEL_OUT)/.config
    KERNEL_CONFIG_FILE := $(KERNEL_DIR)/arch/$(KERNEL_TARGET_ARCH)/configs/$(word 1,$(KERNEL_DEFCONFIG))
    KERNEL_MAKE_OPTION := O=$(KERNEL_ROOT_OUT) ARCH=$(KERNEL_TARGET_ARCH) $(ARGS) ROOTDIR=$(KERNEL_ROOT_DIR)
    KERNEL_MAKE_PATH_OPTION := /usr/bin:/bin
    KERNEL_MAKE_OPTION += PATH=$(KERNEL_ROOT_DIR)/$(CLANG_PREBUILT_BIN):$(KERNEL_ROOT_DIR)/$(LINUX_GCC_CROSS_COMPILE_PREBUILTS_BIN):$(KERNEL_MAKE_PATH_OPTION):$$PATH
  else
    BUILT_KERNEL_TARGET := $(TARGET_PREBUILT_KERNEL)
  endif #TARGET_PREBUILT_KERNEL is empty
    KERNEL_MAKE_OPTION += PROJECT_DTB_NAMES='$(PROJECT_DTB_NAMES)'
endif #TARGET_NO_KERNEL

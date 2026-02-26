# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2019 MediaTek Inc.

LOCAL_PATH := $(call my-dir)

ifeq ($(notdir $(LOCAL_PATH)),$(strip $(LINUX_KERNEL_VERSION)))
ifneq ($(strip $(TARGET_NO_KERNEL)),true)

include $(LOCAL_PATH)/kenv.mk

ifeq ($(wildcard $(TARGET_PREBUILT_KERNEL)),)
KERNEL_MAKE_DEPENDENCIES := $(shell find $(KERNEL_DIR) -name .git -prune -o -type f | sort)

$(TARGET_KERNEL_CONFIG): PRIVATE_DIR := $(KERNEL_DIR)
$(TARGET_KERNEL_CONFIG): $(KERNEL_CONFIG_FILE) $(LOCAL_PATH)/Android.mk
$(TARGET_KERNEL_CONFIG): $(KERNEL_MAKE_DEPENDENCIES)
	$(hide) mkdir -p $(dir $@)
	# ifdef VENDOR_EDIT
	# cheng.chang@ARCH, 2020-11-5 add for kernel decoupling
	#$(info "Merge tcl vendor files")
	$(shell $(PRIVATE_DIR)/tcl_compile.sh $(PRIVATE_DIR))
	# endif /*VENDOR_EDIT*/
	# [TCT-SAT][PERF]Begin modified by ziyuanzhao for perf XR11411798 20210906
	## orig: $(PREBUILT_MAKE_PREFIX)$(MAKE) -C $(PRIVATE_DIR) $(KERNEL_MAKE_OPTION) $(KERNEL_DEFCONFIG)
	$(warning TCT kconf files: $(KERNEL_DEFCONFIG))
	cp $(PRIVATE_DIR)/arch/$(KERNEL_TARGET_ARCH)/configs/$(word 1, $(KERNEL_DEFCONFIG)) $(PRIVATE_DIR)/arch/$(KERNEL_TARGET_ARCH)/configs/tmp_defconfig
	if [ ! -z "$(KERNEL_CONFIG_MERGE_SCRIPT)" ]; then $(KERNEL_CONFIG_MERGE_SCRIPT) $(KERNEL_CONFIG_REMOVE) $(KERNEL_CONFIG_ADD) $(PRIVATE_DIR)/arch/$(KERNEL_TARGET_ARCH)/configs/tmp_defconfig; fi
	$(PREBUILT_MAKE_PREFIX)$(MAKE) -C $(PRIVATE_DIR) $(KERNEL_MAKE_OPTION) tmp_defconfig $(filter-out $(word 1, $(KERNEL_DEFCONFIG)),$(KERNEL_DEFCONFIG))
	if [ -f $(PRIVATE_DIR)/arch/$(KERNEL_TARGET_ARCH)/configs/tmp_defconfig ]; then rm -rf $(PRIVATE_DIR)/arch/$(KERNEL_TARGET_ARCH)/configs/tmp_defconfig; fi
	# [TCT-SAT][PERF]End modified by ziyuanzhao for perf XR11411798 20210906

.KATI_RESTAT: $(KERNEL_ZIMAGE_OUT)
$(KERNEL_ZIMAGE_OUT): PRIVATE_DIR := $(KERNEL_DIR)
$(KERNEL_ZIMAGE_OUT): $(TARGET_KERNEL_CONFIG) $(KERNEL_MAKE_DEPENDENCIES)
	$(hide) mkdir -p $(dir $@)
	$(PREBUILT_MAKE_PREFIX)$(MAKE) -C $(PRIVATE_DIR) $(KERNEL_MAKE_OPTION)
	$(hide) $(call fixup-kernel-cmd-file,$(KERNEL_OUT)/arch/$(KERNEL_TARGET_ARCH)/boot/compressed/.piggy.xzkern.cmd)
	# check the kernel image size
	python device/mediatek/build/build/tools/check_kernel_size.py $(KERNEL_OUT) $(KERNEL_DIR) $(PROJECT_DTB_NAMES)

$(BUILT_KERNEL_TARGET): $(KERNEL_ZIMAGE_OUT) $(TARGET_KERNEL_CONFIG) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-target)

$(TARGET_PREBUILT_KERNEL): $(BUILT_KERNEL_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-new-target)

endif #TARGET_PREBUILT_KERNEL is empty

$(INSTALLED_KERNEL_TARGET): $(BUILT_KERNEL_TARGET) $(LOCAL_PATH)/Android.mk | $(ACP)
	$(copy-file-to-target)

.PHONY: kernel save-kernel kernel-savedefconfig kernel-menuconfig menuconfig-kernel savedefconfig-kernel clean-kernel
kernel: $(INSTALLED_KERNEL_TARGET) $(INSTALLED_MTK_DTB_TARGET)
save-kernel: $(TARGET_PREBUILT_KERNEL)

kernel-savedefconfig: $(TARGET_KERNEL_CONFIG)
	cp $(TARGET_KERNEL_CONFIG) $(KERNEL_CONFIG_FILE)

kernel-menuconfig:
	$(hide) mkdir -p $(KERNEL_OUT)
	$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) menuconfig

menuconfig-kernel savedefconfig-kernel:
	$(hide) mkdir -p $(KERNEL_OUT)
	$(MAKE) -C $(KERNEL_DIR) $(KERNEL_MAKE_OPTION) $(patsubst %config-kernel,%config,$@)

clean-kernel:
	$(hide) rm -rf $(KERNEL_OUT) $(INSTALLED_KERNEL_TARGET)

### DTB build template
MTK_DTBIMAGE_DTS := $(addsuffix .dts,$(addprefix $(KERNEL_DIR)/arch/$(KERNEL_TARGET_ARCH)/boot/dts/,$(PLATFORM_DTB_NAME)))
include device/mediatek/build/core/build_dtbimage.mk

MTK_DTBOIMAGE_DTS := $(addsuffix .dts,$(addprefix $(KERNEL_DIR)/arch/$(KERNEL_TARGET_ARCH)/boot/dts/,$(PROJECT_DTB_NAMES)))
include device/mediatek/build/core/build_dtboimage.mk

endif #TARGET_NO_KERNEL
endif #LINUX_KERNEL_VERSION

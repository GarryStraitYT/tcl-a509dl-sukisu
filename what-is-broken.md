[ralsei@ralsei-gwtn14110 bangkok_tf]$ diff stock.config custom.config 
7c7
< # Compiler: Android (6443078 based on r383902) clang version 11.0.1 (https://android.googlesource.com/toolchain/llvm-project b397f81060ce6d701042b782172ed13bee898b79)
---
> # Compiler: Android (5484270 based on r353983c) clang version 9.0.3 (https://android.googlesource.com/toolchain/clang 745b335211bb9eadfa6aa6301f84715cee4b37c5) (https://android.googlesource.com/toolchain/llvm 60cf23e54e46c807513f7a36d0a7b777920b5881) (based on LLVM 9.0.3svn)
11,14c11
< CONFIG_LD_IS_LLD=y
< CONFIG_CLANG_VERSION=110001
< CONFIG_CC_HAS_ASM_GOTO=y
< CONFIG_TOOLS_SUPPORT_RELR=y
---
> CONFIG_CLANG_VERSION=90003
386d382
< CONFIG_MITIGATE_SPECTRE_BRANCH_HISTORY=y
544d539
< # CONFIG_JUMP_LABEL is not set
580d574
< # CONFIG_LTO_CLANG is not set
608d601
< CONFIG_RELR=y
1546c1539
< CONFIG_CUSTOM_KERNEL_LCM="bangkok_coe_gh1001_hd_dsi_vdo bangkok_tdt_gh1001_hd_dsi_vdo bangkok_tdt_ili9881d_hd_dsi_vdo bangkok_coe_gc9702p_hd_dsi_vdo bangkok_ykl_xm96120_hd_dsi_vdo"
---
> CONFIG_CUSTOM_KERNEL_LCM="bangkok_tdt_ili9881d_hd_dsi_vdo bangkok_coe_gc9702p_hd_dsi_vdo"
1639d1631
< CONFIG_BKTF_3RD_MC3416=y
1661d1652
< CONFIG_BKTF_2ND_SPL07=y
2227,2237d2217
< CONFIG_TOUCHSCREEN_MTK_CHSC5XXX_BANGKOKTF=y
< CONFIG_TOUCHSCREEN_MTK_BETTERLIFE_BANGKOKTF=y
< # CONFIG_TOUCHSCREEN_MTK_BTL is not set
< CONFIG_BTL_CHECK_CHIPID=y
< CONFIG_BTL_DEBUG=y
< # CONFIG_TPD_ROTATE_90 is not set
< # CONFIG_TPD_ROTATE_180 is not set
< # CONFIG_TPD_ROTATE_270 is not set
< # CONFIG_TPD_ROTATE_X_180 is not set
< # CONFIG_BL_AUTO_UPGRADE_SUPPORT is not set
< # CONFIG_CUST_BL_APK_DEBUG is not set
=====================================================

What's borked:
- WiFi
- Bluetooth
- Touchscreen
- Modem(?)/Calls

What works:
- Literally everything else

====================================================

If anyone can help, please do so.

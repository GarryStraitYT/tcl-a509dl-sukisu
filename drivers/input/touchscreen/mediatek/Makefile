subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

obj-$(CONFIG_TOUCHSCREEN_MTK_SOLOMON)	+=  SOLOMON/
obj-$(CONFIG_TOUCHSCREEN_FTS)	+=  focaltech_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_GSLX680)	+=  gslX680/
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx)	+=  NT36xxx/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_LM)	+=  GT9XX_LM/
obj-$(CONFIG_TOUCHSCREEN_FT5X26)	+=  FT5X26/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT5688)	+=  GT5688/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_I2C)	+=  hxchipset/
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_FHD)	+=  focaltech_fhd_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9157)    +=  GT9157/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET_8789P1_8185P3)    +=  hxchipset_8789p1_8185p3/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET) += hxchipset_hx83102p/

#Add-begin by baiwei.peng for TP porting on 2020/09/03
obj-$(CONFIG_TOUCHSCREEN_MTK_TOKYOLITETMO_NT36525BH)    +=  tokyolitetmo_nt36525bh_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_TOKYOLITETMO_FT8006S)      +=  tokyolitetmo_ft8006s_touch/
#Add-end by baiwei.peng for TP porting on 2020/09/03

obj-$(CONFIG_TOUCHSCREEN_MTK_FT5446_BANGKOKTF)      +=  bangkoktf_ft5446/
obj-$(CONFIG_TOUCHSCREEN_MTK_CHSC5XXX_JETTA)      +=  jetta_chsc5xxx/
obj-$(CONFIG_TOUCHSCREEN_MTK_CHIPONE_JETTA)      +=  jetta_chipone-ts/
obj-$(CONFIG_TOUCHSCREEN_MTK_CHSC5XXX_BANGKOKTF)      +=  bangkoktf_CHSC5XXX/
obj-$(CONFIG_TOUCHSCREEN_MTK_BETTERLIFE_BANGKOKTF)      +=  bangkoktf_betterlife_ts/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151_BANGKOKTF)      +=  bangkoktf_GT1151/
#begin add by xiongbo.huang for apollo porting on 2020/0307
obj-$(CONFIG_TOUCHSCREEN_MTK_APOLLO_GT912)  +=  apollo_gt912/
#end add by xiongbo.huang for apollo porting on 2020/0307
#Add-start by jiaxin.pi for tp bl7401 porting on 2021/07/06
obj-$(CONFIG_TOUCHSCREEN_MTK_APOLLO_BL7401)  +=  apollo_bl7401/
#Add-end by jiaxin.pi for tp bl7401 porting on 2021/07/06
obj-$(CONFIG_ODIN_CHSC5XXX_TOUCH)	+=  odin_chsc5xxx_touch/
obj-$(CONFIG_TOUCHSCREEN_FTS_ODIN)     +=  odin_focaltech_touch/
obj-$(CONFIG_BUFFALOTMO_GC7202_TOUCH)	+=  buffalotmo_gc7202_touch/
obj-$(CONFIG_TOUCHSCREEN_GOODIX_BRL) += goodix_berlin_driver/

#Begin added by liangjiaqiang for MODEL3-1881 on 2022-09-09
obj-$(CONFIG_TOUCHSCREEN_CHIPONE_XE671) += chipone-tddi_xe671/
#End added by liangjiaqiang for MODEL3-1881 on 2022-09-09
#Begin added by liangjiaqiang for MODEL3-4765 on 2022-11-16
obj-$(CONFIG_TOUCHSCREEN_FT8057_XE671) += Focaltech_Touch_FT8057_xe671/
#End added by liangjiaqiang for MODEL3-4765 on 2022-11-16

#Begin added by liangjiaqiang for CIVIC-3003 on 2022-07-12
obj-$(CONFIG_TOUCHSCREEN_CHIPONE_CIVI) += chipone-tddi/
obj-$(CONFIG_TOUCHSCREEN_FT8057) += Focaltech_Touch_FT8057/
#End added by liangjiaqiang for CIVIC-3003 on 2022-07-12

obj-$(CONFIG_TOUCHSCREEN_LUNA_FOCALTECH_I2C) += luna_focaltech_wxga_i2c_touch/
obj-$(CONFIG_TOUCHSCREEN_LUNA_CHSC5XXX_I2C) += luna_chsc5xxx_wxga_i2c_touch/

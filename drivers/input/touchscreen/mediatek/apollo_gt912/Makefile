# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/apollo_gt912/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/apollo_gt912/include/$(CONFIG_GTP_CONFIG)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-$(CONFIG_APOLLO_GT912)  +=  gt9xx_driver.o
obj-$(CONFIG_APOLLO_GT912)  +=  gt9xx_extents.o
obj-$(CONFIG_APOLLO_GT912)  +=  gt9xx_update.o
obj-$(CONFIG_APOLLO_GT912)  +=  goodix_tool.o

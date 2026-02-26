ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/


obj-$(CONFIG_TOUCHSCREEN_CHIPONE_XE671) += chipone-tddi_xe671.o

chipone-tddi_xe671-objs := cts_tpd_driver.o cts_i2c_driver.o
chipone-tddi_xe671-objs += cts_core.o
chipone-tddi_xe671-objs += cts_sfctrlv2.o
chipone-tddi_xe671-objs += cts_spi_flash.o
chipone-tddi_xe671-objs += cts_firmware.o
chipone-tddi_xe671-objs += cts_test.o
chipone-tddi_xe671-objs += cts_charger_detect.o
chipone-tddi_xe671-objs += cts_earjack_detect.o
chipone-tddi_xe671-objs += cts_hostcomm.o
chipone-tddi_xe671-objs += cts_tcs.o
chipone-tddi_xe671-objs += cts_platform.o
chipone-tddi_xe671-objs += cts_tool.o
chipone-tddi_xe671-objs += cts_sysfs.o
chipone-tddi_xe671-objs += cts_strerror.o
chipone-tddi_xe671-objs += cts_oem.o



ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc/

obj-$(CONFIG_TOUCHSCREEN_CHIPONE_CIVI) += chipone-tddi.o

chipone-tddi-objs := cts_tpd_driver.o cts_i2c_driver.o
chipone-tddi-objs += cts_core.o
chipone-tddi-objs += cts_sfctrlv2.o
chipone-tddi-objs += cts_spi_flash.o
chipone-tddi-objs += cts_firmware.o
chipone-tddi-objs += cts_test.o
chipone-tddi-objs += cts_charger_detect.o
chipone-tddi-objs += cts_earjack_detect.o
chipone-tddi-objs += cts_hostcomm.o
chipone-tddi-objs += cts_tcs.o
chipone-tddi-objs += cts_platform.o
chipone-tddi-objs += cts_tool.o
chipone-tddi-objs += cts_sysfs.o
chipone-tddi-objs += cts_strerror.o
chipone-tddi-objs += cts_oem.o
chipone-tddi-objs += cts_proximity.o


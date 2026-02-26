ccflags-y += -I$(srctree)/drivers/input/touchscreen/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y += chipone-tddi.o

chipone-tddi-objs := cts_driver.o cts_i2c_driver.o cts_spi_driver.o
chipone-tddi-objs += cts_core.o cts_sfctrlv2.o cts_spi_flash.o cts_firmware.o cts_test.o
chipone-tddi-objs += cts_strerror.o
chipone-tddi-objs += cts_charger_detect.o
chipone-tddi-objs += cts_earjack_detect.o
chipone-tddi-objs += cts_platform.o
chipone-tddi-objs += cts_tool.o
chipone-tddi-objs += cts_sysfs.o


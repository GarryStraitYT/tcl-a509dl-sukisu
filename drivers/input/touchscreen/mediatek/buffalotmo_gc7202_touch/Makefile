#
# Makefile for the GalaxyCore touchscreen driver.
#
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-$(CONFIG_BUFFALOTMO_GC7202_TOUCH)	+=  gcore_drv_main.o
obj-$(CONFIG_BUFFALOTMO_GC7202_TOUCH)	+=  gcore_drv_mtk.o
obj-$(CONFIG_BUFFALOTMO_GC7202_TOUCH)	+=  gcore_drv_fw_update.o
obj-$(CONFIG_BUFFALOTMO_GC7202_TOUCH)	+=  gcore_drv_interface.o
obj-$(CONFIG_BUFFALOTMO_GC7202_TOUCH)	+=  gcore_drv_mp_test.o

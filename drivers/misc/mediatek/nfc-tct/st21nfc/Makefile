#
# Makefile for ST21NFC
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
obj-$(CONFIG_NFC_ST21NFCD_TCT) +=st21nfc.o
ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG

subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-$(CONFIG_MACH_MT8167)+= -Werror -I$(srctree)/drivers/misc/mediatek/accdet/$(MTK_PLATFORM)

#subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/accdet/$(MTK_PLATFORM)
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6357)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6357
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6358)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6358
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6359)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6359
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6359P)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6359
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6390)+= -I$(srctree)/drivers/misc/mediatek/accdet/mt6357

obj-$(CONFIG_MTK_ACCDET) := accdet_drv.o

obj-$(CONFIG_MTK_PMIC_CHIP_MT6357) += mt6357/
obj-$(CONFIG_MTK_PMIC_CHIP_MT6358) += mt6358/
obj-$(CONFIG_MTK_PMIC_CHIP_MT6359) += mt6359/
obj-$(CONFIG_MTK_PMIC_CHIP_MT6359P) += mt6359/
obj-$(CONFIG_MTK_PMIC_CHIP_MT6390) += mt6357/
obj-$(CONFIG_MACH_MT8167) += mt8167/
# Begin added by lanying.he for XR porting on 2022/01/05
obj-$(CONFIG_TCT_PROJECT_FAS4480)+= fsa4480/
# End added by lanying.he for XR porting on 2022/01/05

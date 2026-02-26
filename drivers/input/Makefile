# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT)		+= input-core.o
input-core-y := input.o input-compat.o input-mt.o ff-core.o

obj-$(CONFIG_INPUT_FF_MEMLESS)	+= ff-memless.o
obj-$(CONFIG_INPUT_POLLDEV)	+= input-polldev.o
obj-$(CONFIG_INPUT_SPARSEKMAP)	+= sparse-keymap.o
obj-$(CONFIG_INPUT_MATRIXKMAP)	+= matrix-keymap.o

obj-$(CONFIG_INPUT_LEDS)	+= input-leds.o
obj-$(CONFIG_INPUT_MOUSEDEV)	+= mousedev.o
obj-$(CONFIG_INPUT_JOYDEV)	+= joydev.o
obj-$(CONFIG_INPUT_EVDEV)	+= evdev.o
obj-$(CONFIG_INPUT_EVBUG)	+= evbug.o

obj-$(CONFIG_INPUT_KEYBOARD)	+= keyboard/
obj-$(CONFIG_INPUT_MOUSE)	+= mouse/
obj-$(CONFIG_INPUT_JOYSTICK)	+= joystick/
obj-$(CONFIG_INPUT_TABLET)	+= tablet/
obj-$(CONFIG_INPUT_TOUCHSCREEN)	+= touchscreen/
obj-$(CONFIG_INPUT_MISC)	+= misc/

obj-$(CONFIG_INPUT_APMPOWER)	+= apm-power.o

obj-$(CONFIG_RMI4_CORE)		+= rmi4/

# [TCTNB][FINGERPRINT]Added_begin by Jiancong.Huang for F11394572 at 2021/8/12
obj-$(CONFIG_MTK_FINGERPRINT_SUPPORT)		+= fingerprint/
# [TCTNB][FINGERPRINT]Added_end by Jiancong.Huang for F11394572 at 2021/8/12

#Add-start by baiwei.peng for hall & sar sensor porting on 2022/03/16
obj-$(CONFIG_HALL_SWITCH)               += hallsensor/
obj-$(CONFIG_HALL_SWITCH_2)             += hallsensor_2/
obj-$(CONFIG_AW9610X_SAR)              += aw9610x/
#Add-end by baiwei.peng for hall & sar sensor porting on 2022/03/16


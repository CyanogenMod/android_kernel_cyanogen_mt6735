#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include

obj-$(CONFIG_MTK_LEDS) += leds_drv.o

ARCH_ARM_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT)")

ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/leds/$(ARCH_ARM_MTK_PROJECT)/*),)
$(info leds by project)
obj-y += $(ARCH_ARM_MTK_PROJECT)/
else
$(info leds by platform)
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif

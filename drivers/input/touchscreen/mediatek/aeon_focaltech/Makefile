#include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_focaltech/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_focaltech/include/
ifeq ($(AEON_SUPPORT),yes)
ifneq ($(AEON_CUSTOM_MODEL), )
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_focaltech/include/$(AEON_CUSTOM_MODEL)/
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_focaltech/include/firmware_default/
endif
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_focaltech/include/firmware_default/
endif
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  focaltech_ctl.o
obj-y	+=  focaltech_ex_fun.o
obj-y	+=  focaltech_flash.o
obj-y	+=  focaltech_Gesture.o
obj-y	+=  focaltech_core.o





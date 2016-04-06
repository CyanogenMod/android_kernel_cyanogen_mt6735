#include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt9xx/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt9xx/include/
ifeq ($(AEON_SUPPORT),yes)
ifneq ($(AEON_CUSTOM_MODEL), )
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt9xx/include/$(AEON_CUSTOM_MODEL)/
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt9xx/include/config_default/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt9xx/include/firmware_default/
endif
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt9xx/include/config_default/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt9xx/include/firmware_default/
endif
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  gt9xx_driver.o
obj-y	+=  goodix_tool.o
obj-y	+=  gt9xx_update.o


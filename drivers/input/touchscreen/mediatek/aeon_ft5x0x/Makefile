#include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_ft5x0x/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_ft5x0x/include/
ifeq ($(AEON_SUPPORT),yes)
ifneq ($(AEON_CUSTOM_MODEL), )
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_ft5x0x/include/$(AEON_CUSTOM_MODEL)/
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_ft5x0x/include/firmware_default/
endif
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_ft5x0x/include/firmware_default/
endif
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  ft5x0x_driver.o
obj-y	+=  ft5x0x_upgrade.o


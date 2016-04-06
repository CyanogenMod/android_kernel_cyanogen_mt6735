#include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt1x/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt1x/include/
ifeq ($(AEON_SUPPORT),yes)
ifneq ($(AEON_CUSTOM_MODEL), )
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt1x/include/$(AEON_CUSTOM_MODEL)/
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt1x/include/config_default/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt1x/include/firmware_default/
endif
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt1x/include/config_default/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_gt1x/include/firmware_default/
endif
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  gt1x_extents.o
obj-y	+=  gt1x_generic.o
obj-y	+=  gt1x_tools.o
obj-y	+=  gt1x_tpd.o
obj-y	+=  gt1x_update.o


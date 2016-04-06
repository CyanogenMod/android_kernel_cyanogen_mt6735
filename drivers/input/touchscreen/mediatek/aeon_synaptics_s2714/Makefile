#include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_synaptics_s2714/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_synaptics_s2714/include/
ifeq ($(AEON_SUPPORT),yes)
ifneq ($(AEON_CUSTOM_MODEL), )
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_synaptics_s2714/include/$(AEON_CUSTOM_MODEL)/
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_synaptics_s2714/include/firmware_default/
endif
else
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/aeon_synaptics_s2714/include/firmware_default/
endif
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y += synaptics_dsx_fw_update.o
obj-y += synaptics_dsx_i2c.o
obj-y += synaptics_dsx_rmi_dev.o



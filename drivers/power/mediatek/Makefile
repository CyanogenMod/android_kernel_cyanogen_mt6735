
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat

ifeq ($(CONFIG_MTK_HAFG_20),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_fg_20.o battery_common_fg_20.o
else
	obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter.o battery_common.o
endif

# sanford.lin add start on 20160315 for aeon
-include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
ifeq ($(AEON_SUPPORT),yes)
ifneq ($(AEON_CHARGE_IC),)
ifneq ($(AEON_CHARGE_IC),no)
CONFIG_MTK_$(AEON_CHARGE_IC)_SUPPORT=y
endif
endif
endif
# sanford.lin add end on 20160315 for aeon

ifeq ($(CONFIG_MTK_FAN5402_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_FAN5405_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_BQ24158_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_BQ24296_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_NCP1851_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_NCP1854_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_BQ24160_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_BQ24157_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_BQ24250_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_BQ24261_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_RT9536_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_BQ25896_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(CONFIG_MTK_SWCHR_SUPPORT),y)
	obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else
	obj-$(CONFIG_MTK_SMART_BATTERY) += linear_charging.o
endif

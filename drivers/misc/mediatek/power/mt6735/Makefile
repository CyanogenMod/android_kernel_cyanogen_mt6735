#include $(srctree)/drivers/misc/mediatek/Makefile.custom

#obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o pmic_chr_type_det.o mt6311.o
obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o mt6311.o

ifeq ($(CONFIG_MTK_PMIC_DVT_SUPPORT),y)
     obj-$(CONFIG_MTK_PMIC)     += pmic_dvt.o
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o pmic_chr_type_det.o

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

ifeq ($(CONFIG_MTK_BQ24261_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += bq24261.o charging_hw_bq24261.o
else ifeq ($(CONFIG_MTK_NCP1854_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += ncp1854.o charging_hw_ncp1854.o
else ifeq ($(CONFIG_MTK_BQ24296_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += bq24296.o charging_hw_bq24296.o
else ifeq ($(CONFIG_MTK_FAN5405_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += fan5405.o charging_hw_fan5405.o
else ifeq ($(CONFIG_MTK_BQ24158_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += bq24158.o charging_hw_bq24158.o
else ifeq ($(CONFIG_MTK_SWCHR_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_mt_sw_charger.o
else
    obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o
endif

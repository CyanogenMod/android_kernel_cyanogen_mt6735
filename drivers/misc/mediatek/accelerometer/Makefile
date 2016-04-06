# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

obj-y   +=  accel.o accel_factory.o

#sanford.lin 20150205 add for aeon
-include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
ifeq ($(AEON_SUPPORT),yes)

ifneq ($(AEON_CUSTOM_ACCELEROMETER), )
ACCEL_LISTS := $(subst ",,$(AEON_CUSTOM_ACCELEROMETER))
obj-y += $(foreach ACCEL,$(ACCEL_LISTS),$(ACCEL)/)
endif

else
obj-$(CONFIG_MTK_K2DH)   +=  k2dh/
obj-$(CONFIG_MTK_BMA050)   +=  bma050/
obj-$(CONFIG_MTK_BMA2XX)   +=  bma2xx/
obj-$(CONFIG_MTK_BMA050_NEW)   +=  bma050-new/
obj-$(CONFIG_MTK_BMA222E)   +=  bma222E/
obj-$(CONFIG_MTK_BMA222E_NEW)   +=  bma222E-new/
obj-$(CONFIG_MTK_MPU6050G)   +=  mpu6050g/
obj-$(CONFIG_MTK_MPU6050G_NEW)   +=  mpu6050g-new/
obj-$(CONFIG_MTK_BMA250)   +=  bma250/
obj-$(CONFIG_MTK_KXTIK1004)   +=  kxtik1004/
obj-$(CONFIG_MTK_MPU6515A)   +=  mpu6515/
obj-$(CONFIG_MTK_KXTJ2_1009)   +=  KXTJ2_1009/
obj-$(CONFIG_MTK_KXTJ2_1009_NEW)   +=  KXTJ2_1009-new/
obj-$(CONFIG_MTK_BMA250E)   +=  bma250e/
obj-$(CONFIG_MTK_MC3410_NEW)   +=  mc3410-new/
obj-$(CONFIG_MTK_MC3XXX_AUTO)   +=  mc3xxx_auto/
obj-$(CONFIG_MTK_MPU60X0)   +=  mpu60x0/
obj-$(CONFIG_MTK_BMA255_SDO0)   +=  bma255-sdo0/
obj-$(CONFIG_MTK_BMA255_SDO1)   +=  bma255-sdo1/
obj-$(CONFIG_MTK_BMA056)   +=  bma056/
obj-$(CONFIG_MTK_LSM6DS3)   +=  lsm6ds3/
obj-$(CONFIG_MTK_ICM20645G)   +=  icm20645g/
obj-$(CONFIG_MTK_KX023_NEW)   +=  kx023-new/
obj-$(CONFIG_AEON_KX023_NEW)   +=  aeon_kx023-new/
obj-$(CONFIG_AEON_MC3410_NEW)   +=  aeon_mc3410-new/





endif
#sanford.lin 20150205 end

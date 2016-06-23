subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

#sanford.lin 20150205 add for aeon
-include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
ifeq ($(AEON_SUPPORT),yes)

ifneq ($(AEON_KERNEL_TP), )
TP_LISTS := $(subst ",,$(AEON_KERNEL_TP))
obj-y += $(foreach TP,$(TP_LISTS), $(TP)/)
endif

else

obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT910)	+=  GT910/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XXTB_HOTKNOT)	+=  GT9XXTB_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_I2C_RMI4)	+=  synaptics_i2c_rmi4/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT928)	+=  GT928/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_SCP)	+=  GT9XX_hotknot_scp/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5X0X)	+=  ft5x0x/

# eastaeon
obj-$(CONFIG_TOUCHSCREEN_AEON_GT1X)	+=  aeon_gt1x/
obj-$(CONFIG_TOUCHSCREEN_AEON_GT9XX)	+=  aeon_gt9xx/
obj-$(CONFIG_TOUCHSCREEN_AEON_SYNAPTICS_S2714)	+=  aeon_synaptics_s2714/
obj-$(CONFIG_TOUCHSCREEN_AEON_FACTORY_CTP)	+=  aeon_factory_ctp/
obj-$(CONFIG_TOUCHSCREEN_AEON_FT5X0X)	+=  aeon_ft5x0x/
obj-$(CONFIG_TOUCHSCREEN_AEON_FT5X0X_8630)	+=  aeon_ft5x0x_hd720_lide_8630/
obj-$(CONFIG_TOUCHSCREEN_AEON_FOCALTECH)	+=  aeon_focaltech/
obj-$(CONFIG_TOUCHSCREEN_AEON_SYNAPTICS_S2714_8536)   +=  aeon_synaptics_s2714_8536/
obj-$(CONFIG_TOUCHSCREEN_AEON_FT5X0X_8536)   +=  aeon_ft5x0x_hd720_lide_8536/
obj-$(CONFIG_TOUCHSCREEN_MTK_AEON_NT11205_HD720_HLT_8536)	+=  aeon_nt11205_hd720_hlt_8536/
obj-$(CONFIG_TOUCHSCREEN_AEON_MSTAR_8630)	+=  aeon_mstar_8630_v3.14/

endif
#sanford.lin 20150205 end

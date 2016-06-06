/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/mt_gpio.h>
#include <mt-plat/battery_common.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include "bq24158.h"

 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


 // ============================================================ //
 //global variable
 // ============================================================ //
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0
int wireless_charger_gpio_number   = (168 | 0x80000000); 
#endif

#if 0
#include <cust_gpio_usage.h>
int gpio_number   = GPIO_CHR_CE_PIN; 
int gpio_off_mode = GPIO_CHR_CE_PIN_M_GPIO;
int gpio_on_mode  = GPIO_CHR_CE_PIN_M_GPIO;
#else
extern int aeon_gpio_set(const char *name); //sanford.lin
int gpio_number   = (82 | 0x80000000); 
int gpio_off_mode = 0;
int gpio_on_mode  = 0;
#endif
int gpio_off_dir  = GPIO_DIR_OUT;
int gpio_off_out  = GPIO_OUT_ONE;
int gpio_on_dir   = GPIO_DIR_OUT;
int gpio_on_out   = GPIO_OUT_ZERO;

kal_bool charging_type_det_done = KAL_TRUE;

const unsigned int VBAT_CV_VTH[]=
{
	BATTERY_VOLT_03_500000_V,   BATTERY_VOLT_03_520000_V,	BATTERY_VOLT_03_540000_V,   BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V,   BATTERY_VOLT_03_600000_V,	BATTERY_VOLT_03_620000_V,   BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V,	BATTERY_VOLT_03_680000_V,	BATTERY_VOLT_03_700000_V,	BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V,	BATTERY_VOLT_03_760000_V,	BATTERY_VOLT_03_780000_V,	BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V,	BATTERY_VOLT_03_840000_V,	BATTERY_VOLT_03_860000_V,	BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V,	BATTERY_VOLT_03_920000_V,	BATTERY_VOLT_03_940000_V,	BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V,	BATTERY_VOLT_04_000000_V,	BATTERY_VOLT_04_020000_V,	BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V,	BATTERY_VOLT_04_080000_V,	BATTERY_VOLT_04_100000_V,	BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V,   BATTERY_VOLT_04_160000_V,	BATTERY_VOLT_04_180000_V,   BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V,   BATTERY_VOLT_04_240000_V,	BATTERY_VOLT_04_260000_V,   BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_320000_V,	BATTERY_VOLT_04_340000_V,   BATTERY_VOLT_04_360000_V,	
	BATTERY_VOLT_04_380000_V,   BATTERY_VOLT_04_400000_V,	BATTERY_VOLT_04_420000_V,   BATTERY_VOLT_04_440000_V	
	
};

const unsigned int CS_VTH[]=
{
	CHARGE_CURRENT_550_00_MA,   CHARGE_CURRENT_650_00_MA,	CHARGE_CURRENT_750_00_MA, CHARGE_CURRENT_850_00_MA,
	CHARGE_CURRENT_950_00_MA,   CHARGE_CURRENT_1050_00_MA,	CHARGE_CURRENT_1150_00_MA, CHARGE_CURRENT_1250_00_MA
}; 

 const unsigned int INPUT_CS_VTH[]=
 {
	 CHARGE_CURRENT_100_00_MA,	 CHARGE_CURRENT_500_00_MA,	 CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_MAX
 }; 

 const unsigned int VCDT_HV_VTH[]=
 {
	  BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	  BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	  BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	  BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
 };

 // ============================================================ //
 // function prototype
 // ============================================================ //
 
 
 // ============================================================ //
 //extern variable
 // ============================================================ //
 
 // ============================================================ //
 //extern function
 // ============================================================ //
 extern unsigned int upmu_get_reg_value(unsigned int reg);
 extern bool mt_usb_is_device(void);
 extern void Charger_Detect_Init(void);
 extern void Charger_Detect_Release(void);
 extern void mt_power_off(void);
 
 // ============================================================ //
 unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size, const unsigned int val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");	
		return parameter[0];
	}
}

 
 unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size, const unsigned int val)
{
	unsigned int i;

	for(i=0;i<array_size;i++)
	{
		if (val == *(parameter + i))
		{
				return i;
		}
	}

    battery_log(BAT_LOG_CRTI, "NO register value match \r\n");
	//TODO: ASSERT(0);	// not find the value
	return 0;
}


 static unsigned int bmt_find_closest_level(const unsigned int *pList,unsigned int number,unsigned int level)
 {
	 unsigned int i;
	 unsigned int max_value_in_last_element;
 
	 if(pList[0] < pList[1])
		 max_value_in_last_element = KAL_TRUE;
	 else
		 max_value_in_last_element = KAL_FALSE;
 
	 if(max_value_in_last_element == KAL_TRUE)
	 {
		 for(i = (number-1); i != 0; i--)	 //max value in the last element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

 		 battery_log(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		 return pList[0];
		 //return CHARGE_CURRENT_0_00_MA;
	 }
	 else
	 {
		 for(i = 0; i< number; i++)  // max value in the first element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

		 battery_log(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n"); 	 
		 return pList[number -1];
  		 //return CHARGE_CURRENT_0_00_MA;
	 }
 }


static void hw_bc11_dump_register(void)
{
	unsigned int reg_val = 0;
	unsigned int reg_num = MT6328_CHR_CON20;
	unsigned int i = 0;

	for(i=reg_num ; i<=MT6328_CHR_CON21 ; i+=2)
	{
		reg_val = upmu_get_reg_value(i);
		battery_log(BAT_LOG_FULL, "Chr Reg[0x%x]=0x%x \r\n", i, reg_val);
	}
}


static void hw_bc11_init(void)
{
    msleep(200);
    Charger_Detect_Init();
        
    //RG_bc11_BIAS_EN=1
    bc11_set_register_value(PMIC_RG_BC11_BIAS_EN,1);
    //RG_bc11_VSRC_EN[1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0);
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0);
    //bc11_RST=1
    bc11_set_register_value(PMIC_RG_BC11_RST,1);
    //bc11_BB_CTRL=1
    bc11_set_register_value(PMIC_RG_BC11_BB_CTRL,1);

    msleep(50);
    //mdelay(50);
    
    if(Enable_BATDRV_LOG == BAT_LOG_FULL)
    {
        battery_log(BAT_LOG_FULL, "hw_bc11_init() \r\n");
        hw_bc11_dump_register();
    }    

}

static unsigned int hw_bc11_DCD(void)
{
    unsigned int wChargerAvail = 0;

    //RG_bc11_IPU_EN[1.0] = 10
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2);  
    //RG_bc11_IPD_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
    //RG_bc11_VREF_VTH = [1:0]=01
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x1);
    //RG_bc11_CMP_EN[1.0] = 10
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x2);

    msleep(80);
    //mdelay(80);

    wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
    
    if(Enable_BATDRV_LOG == BAT_LOG_FULL)
    {
        battery_log(BAT_LOG_FULL, "hw_bc11_DCD() \r\n");
        hw_bc11_dump_register();
    }
    
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);


    return wChargerAvail;
}
 

static unsigned int hw_bc11_stepA1(void)
{
   unsigned int wChargerAvail = 0;
     
   //RG_bc11_IPD_EN[1.0] = 01
   bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
   //RG_bc11_VREF_VTH = [1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
   //RG_bc11_CMP_EN[1.0] = 01
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);

   msleep(80);
   //mdelay(80);

   wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
				
   if(Enable_BATDRV_LOG == BAT_LOG_FULL)
   {
       battery_log(BAT_LOG_FULL, "hw_bc11_stepA1() \r\n");
       hw_bc11_dump_register();
   }

   //RG_bc11_IPD_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
   //RG_bc11_CMP_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);

   return  wChargerAvail;
}
 
 /*
 static unsigned int hw_bc11_stepB1(void)
 {
	unsigned int wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	//upmu_set_rg_bc11_ipu_en(0x1);
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	//upmu_set_rg_bc11_vref_vth(0x2);
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_log(BAT_LOG_FULL, "hw_bc11_stepB1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static unsigned int hw_bc11_stepC1(void)
 {
	unsigned int wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	upmu_set_rg_bc11_ipu_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_log(BAT_LOG_FULL, "hw_bc11_stepC1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 */

static unsigned int hw_bc11_stepA2(void)
{
   unsigned int wChargerAvail = 0;
     
   //RG_bc11_VSRC_EN[1.0] = 10 
   bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x2);
   //RG_bc11_IPD_EN[1:0] = 01
   bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
   //RG_bc11_VREF_VTH = [1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
   //RG_bc11_CMP_EN[1.0] = 01
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);

   msleep(80);
   //mdelay(80);

   //msleep(80);
   mdelay(80);

   wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

   if(Enable_BATDRV_LOG == BAT_LOG_FULL)
   {
       battery_log(BAT_LOG_FULL, "hw_bc11_stepA2() \r\n");
       hw_bc11_dump_register();
   }

   //RG_bc11_VSRC_EN[1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x0);
   //RG_bc11_IPD_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
   //RG_bc11_CMP_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);

   return  wChargerAvail;
}
 

static unsigned int hw_bc11_stepB2(void)
{
   unsigned int wChargerAvail = 0;

   //RG_bc11_IPU_EN[1:0]=10
   bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2); 
   //RG_bc11_VREF_VTH = [1:0]=01
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x1);
   //RG_bc11_CMP_EN[1.0] = 01
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);

   msleep(80);
   //mdelay(80);

   wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

   if(Enable_BATDRV_LOG == BAT_LOG_FULL)
   {
       battery_log(BAT_LOG_FULL, "hw_bc11_stepB2() \r\n");
       hw_bc11_dump_register();
   }

   
   if (!wChargerAvail) {
       //RG_bc11_VSRC_EN[1.0] = 10 
       //mt6325_upmu_set_rg_bc11_vsrc_en(0x2);
       bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x2); 
   }
   //RG_bc11_IPU_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0); 
   //RG_bc11_CMP_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0); 
   //RG_bc11_VREF_VTH = [1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0); 


   return  wChargerAvail;
}

static void hw_bc11_done(void)
{
   //RG_bc11_VSRC_EN[1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x0);
   //RG_bc11_VREF_VTH = [1:0]=0
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
   //RG_bc11_CMP_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
   //RG_bc11_IPU_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);
   //RG_bc11_IPD_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
   //RG_bc11_BIAS_EN=0
   bc11_set_register_value(PMIC_RG_BC11_BIAS_EN,0x0);


   Charger_Detect_Release();

   if(Enable_BATDRV_LOG == BAT_LOG_FULL)
   {
       battery_log(BAT_LOG_FULL, "hw_bc11_done() \r\n");
       hw_bc11_dump_register();
   }
   
}
 
 static unsigned int charging_hw_init(void *data)
 {
 	unsigned int status = STATUS_OK;
	static bool charging_init_flag = KAL_FALSE;
	
	//mt_set_gpio_mode(gpio_number,gpio_on_mode);  
    //mt_set_gpio_dir(gpio_number,gpio_on_dir);
    //mt_set_gpio_out(gpio_number,gpio_on_out);
	aeon_gpio_set("aeon_chr_ce0"); //sanford.lin
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	mt_set_gpio_mode(wireless_charger_gpio_number,0); // 0:GPIO mode
	mt_set_gpio_dir(wireless_charger_gpio_number,0); // 0: input, 1: output
#endif
	//battery_log(BAT_LOG_FULL, "gpio_number=0x%x,gpio_on_mode=%d,gpio_off_mode=%d\n", gpio_number, gpio_on_mode, gpio_off_mode);

	bc11_set_register_value(PMIC_RG_USBDL_SET,0x0);//force leave USBDL mode
	bc11_set_register_value(PMIC_RG_USBDL_RST,0x1);//force leave USBDL mode

	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
        bq24158_reg_config_interface(0x06,0x47); // ISAFE = 1250mA, VSAFE = 4.34V
    #else
        bq24158_reg_config_interface(0x06,0x70);
	#endif
	    
    bq24158_reg_config_interface(0x00,0x80);	//kick chip watch dog, disable STAT pin funtion (sanford.lin)
    bq24158_reg_config_interface(0x01,0xf8);	//TE=1, CE=0, HZ_MODE=0, OPA_MODE=0
    bq24158_reg_config_interface(0x05,0x04);

	if ( !charging_init_flag ) {
		bq24158_reg_config_interface(0x04,0x19); //100mA
		charging_init_flag = KAL_TRUE;
	}
 	return status;
 }


 static unsigned int charging_dump_register(void *data)
 {
 	unsigned int status = STATUS_OK;

	bq24158_dump_register();
   	
	return status;
 }	


 static unsigned int charging_enable(void *data)
 {
 	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int*)(data);

	if(KAL_TRUE == enable)
	{
		bq24158_set_ce(0);
		bq24158_set_hz_mode(0);
		bq24158_set_opa_mode(0);
	}
	else
	{

#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if(mt_usb_is_device())
#endif 			
    	{
	        //mt_set_gpio_mode(gpio_number,gpio_off_mode);  
	        //mt_set_gpio_dir(gpio_number,gpio_off_dir);
	        //mt_set_gpio_out(gpio_number,gpio_off_out);
			aeon_gpio_set("aeon_chr_ce1"); //sanford.lin
	        bq24158_set_ce(1);
    	}
	}
		
	return status;
 }


 static unsigned int charging_set_cv_voltage(void *data)
 {
 	unsigned int status = STATUS_OK;
	unsigned short register_value;

	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH) ,*(unsigned int *)(data));
	bq24158_set_oreg(register_value); 

	return status;
 } 	


 static unsigned int charging_get_current(void *data)
 {
    unsigned int status = STATUS_OK;
    unsigned int array_size;
    unsigned char reg_value;
	
    //Get current level
    array_size = GETARRAYNUM(CS_VTH);
    bq24158_read_interface(0x1, &reg_value, 0x3, 0x6);	//IINLIM
    *(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
	
    return status;
 }  
  


 static unsigned int charging_set_current(void *data)
 {
 	unsigned int status = STATUS_OK;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	unsigned int current_value = *(unsigned int *)data;

	if(current_value <= CHARGE_CURRENT_350_00_MA)
	{
		bq24158_set_io_level(1);
	}
	else
	{
		bq24158_set_io_level(0);
		array_size = GETARRAYNUM(CS_VTH);
		set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
		register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
		bq24158_set_iocharge(register_value);
	}
	return status;
 } 	
 

 static unsigned int charging_set_input_current(void *data)
 {
 	unsigned int status = STATUS_OK;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

    if(*(unsigned int *)data > CHARGE_CURRENT_500_00_MA)
    {
        register_value = 0x3;
    }
    else
    {
    	array_size = GETARRAYNUM(INPUT_CS_VTH);
    	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, *(unsigned int *)data);
    	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);	
    }
    
    bq24158_set_input_charging_current(register_value);

	return status;
 } 	


 static unsigned int charging_get_charging_status(void *data)
 {
 	unsigned int status = STATUS_OK;
	unsigned int ret_val;

	ret_val = bq24158_get_chip_status();
	
	if(ret_val == 0x2)
		*(unsigned int *)data = KAL_TRUE;
	else
		*(unsigned int *)data = KAL_FALSE;
	
	return status;
 } 	


 static unsigned int charging_reset_watch_dog_timer(void *data)
 {
	 unsigned int status = STATUS_OK;
 
	 bq24158_set_tmr_rst(1);
	 
	 return status;
 }
 
 
  static unsigned int charging_set_hv_threshold(void *data)
  {
	 unsigned int status = STATUS_OK;
 
	 unsigned int set_hv_voltage;
	 unsigned int array_size;
	 unsigned short register_value;
	 unsigned int voltage = *(unsigned int*)(data);
	 
	 array_size = GETARRAYNUM(VCDT_HV_VTH);
	 set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	 register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
        bc11_set_register_value(PMIC_RG_VCDT_HV_VTH,register_value);//upmu_set_rg_vcdt_hv_vth(register_value);
 
	 return status;
  }
 
 
  static unsigned int charging_get_hv_status(void *data)
  {
	   unsigned int status = STATUS_OK;
 
	   *(kal_bool*)(data) = bc11_get_register_value(PMIC_RGS_VCDT_HV_DET);//upmu_get_rgs_vcdt_hv_det();

	   return status;
  }


 static unsigned int charging_get_battery_status(void *data)
 {
	   unsigned int status = STATUS_OK;

	 bc11_set_register_value(PMIC_BATON_TDET_EN,1);
 	 bc11_set_register_value(PMIC_RG_BATON_EN,1); 
	   *(kal_bool*)(data) = bc11_get_register_value(PMIC_RGS_BATON_UNDET);//upmu_get_rgs_baton_undet();
	   
	   return status;
 }


 static unsigned int charging_get_charger_det_status(void *data)
 {
	   unsigned int status = STATUS_OK;
 
	   *(kal_bool*)(data) = bc11_get_register_value(PMIC_RGS_CHRDET);//upmu_get_rgs_chrdet();
	   
	if( bc11_get_register_value(PMIC_RGS_CHRDET)== 0 )
		g_charger_type = CHARGER_UNKNOWN;
       
	   return status;
 }


kal_bool charging_type_detection_done(void)
{
	 return charging_type_det_done;
}


 static unsigned int charging_get_charger_type(void *data)
 {
	 unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT)
	 *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	int wireless_state = 0;
	wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
	if(wireless_state == WIRELESS_CHARGER_EXIST_STATE) {
		*(CHARGER_TYPE*)(data) = WIRELESS_CHARGER;
		battery_log(BAT_LOG_CRTI, "WIRELESS_CHARGER!\r\n");
		return status;
	}
#endif
	if(g_charger_type!=CHARGER_UNKNOWN && g_charger_type!=WIRELESS_CHARGER) {
		*(CHARGER_TYPE*)(data) = g_charger_type;
		battery_log(BAT_LOG_CRTI, "return %d!\r\n", g_charger_type);
		return status;
	}

	charging_type_det_done = KAL_FALSE;

	/********* Step initial  ***************/		 
	hw_bc11_init();
 
	/********* Step DCD ***************/  
	if(1 == hw_bc11_DCD())
	{
		/********* Step A1 ***************/
		if(1 == hw_bc11_stepA1())
		{
			*(CHARGER_TYPE*)(data) = APPLE_2_1A_CHARGER;
			battery_log(BAT_LOG_CRTI, "step A1 : Apple 2.1A CHARGER!\r\n");
		}	 
		else
		{
			*(CHARGER_TYPE*)(data) = NONSTANDARD_CHARGER;
			battery_log(BAT_LOG_CRTI, "step A1 : Non STANDARD CHARGER!\r\n");
		}
	}
	else
	{
		 /********* Step A2 ***************/
		 if(1 == hw_bc11_stepA2())
		 {
			 /********* Step B2 ***************/
			 if(1 == hw_bc11_stepB2())
			 {
				 *(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
				 battery_log(BAT_LOG_CRTI, "step B2 : STANDARD CHARGER!\r\n");
			 }
			 else
			 {
				 *(CHARGER_TYPE*)(data) = CHARGING_HOST;
				 battery_log(BAT_LOG_CRTI, "step B2 :  Charging Host!\r\n");
			 }
		 }
		 else
		 {
			*(CHARGER_TYPE*)(data) = STANDARD_HOST;
			 battery_log(BAT_LOG_CRTI, "step A2 : Standard USB Host!\r\n");
		 }
 
	}
 
	 /********* Finally setting *******************************/
	 hw_bc11_done();

 	charging_type_det_done = KAL_TRUE;

	g_charger_type = *(CHARGER_TYPE*)(data);
#endif
	 return status;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
    unsigned int status = STATUS_OK;

  //  if(slp_get_wake_reason() == WR_PCM_TIMER)
  //      *(kal_bool*)(data) = KAL_TRUE;
  //  else
        *(kal_bool*)(data) = KAL_FALSE;

  //  battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());

    return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
    unsigned int status = STATUS_OK;

    battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");
 
    //arch_reset(0,NULL);
    kernel_restart("battery service reboot system");
        
    return status;
}

static unsigned int charging_get_platfrom_boot_mode(void *data)
{
    unsigned int status = STATUS_OK;
  
    *(unsigned int*)(data) = get_boot_mode();

    battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
         
    return status;
}

static unsigned int charging_set_power_off(void *data)
{
    unsigned int status = STATUS_OK;
  
    battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
    mt_power_off();
         
    return status;
}

static unsigned int charging_get_power_source(void *data)
{
	unsigned int status = STATUS_UNSUPPORTED;

	return status;
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static unsigned int charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static unsigned int charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;	
}

 static unsigned int (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
 {
 	  charging_hw_init
	,charging_dump_register  	
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
 };

 
 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
signed int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
 {
	 signed int status;
	 if(cmd < CHARGING_CMD_NUMBER)
		 status = charging_func[cmd](data);
	 else
		 return STATUS_UNSUPPORTED;
 
	 return status;
 }



/*===========================================================================

                        EDIT HISTORY FOR V11-wgs

when              comment tag        who                  what, where, why                           
----------    ------------     -----------      --------------------------      
 
===========================================================================*/
/* Copyright (c) 2010, Code Aurora Forum. All rights reserved. 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
*/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h> 
#include <asm/atomic.h>
//#include <linux/xlog.h>
//#include <asm/system.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3m2mipi_Sensor.h"


#define S5K3M2_DEBUG
#ifdef S5K3M2_DEBUG
	#define S5K3M2_DB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[S5K3M2_OTP] ",  fmt, ##arg)
#else
	#define S5K3M2_DB(fmt, arg...)
#endif

//S5K3M2 OTP Driver
#define OTP_GROUP_EMPTY                 0x00
#define OTP_GROUP_VALID                 0x01
//#define OTP_GROUP_INVALID               0x10//0x11

#define S5K3M2_I2C_SPEED                300//300 KHz
#define S5K3M2_I2C_WRITE_ID             0x20//0x5a

kal_uint8 MID=0;
#define TRULY_ID 35 //2
kal_uint32 R_Gr_Ratio=0; 
kal_uint32 B_Gr_Ratio=0;
kal_uint32 Gb_Gr_Ratio=0;
static kal_uint32  RG_Ratio_typical = 0;		//goldSample典型值,在read_otp函数中根据custom ID进行赋值
static kal_uint32  BG_Ratio_typical = 0;

#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR   0x020E
#define GAIN_BLUE_ADDR     0x0212
#define GAIN_RED_ADDR      0x0210
#define GAIN_GREEN2_ADDR   0x0214


/*static kal_uint16 otp_read(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);
    
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, S5K3M2_I2C_WRITE_ID);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}*/
static void otp_write(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);
    
    iWriteRegI2C(pusendcmd , 4, S5K3M2_I2C_WRITE_ID);
}
static kal_uint8 otp_read_8(kal_uint32 addr)
{
    kal_uint8 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);
    
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, S5K3M2_I2C_WRITE_ID);
    return get_byte;
}
static void otp_write_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);
    
    iWriteRegI2C(pusendcmd , 3, S5K3M2_I2C_WRITE_ID);
}

/*************************************************************************************************
* Function    :  start_read_otp
* Description :  before read otp , set the reading block setting  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x15
* Return      :  0, reading block setting err
                 1, reading block setting ok 
**************************************************************************************************/
kal_bool start_read_otp(kal_uint8 zone)
{
	//kal_uint8 val = 0;
	//kal_uint16 i;
    
    otp_write(0x0136, 0x1800);//External CLK 24MHz......Follow setting according to diff CLK
    otp_write(0x0304, 0x0006);
    otp_write(0x0306, 0x006E);//1120 0x0064 //1117 //0x0073
    otp_write(0x030c, 0x0004);
    otp_write(0x030e, 0x006A); //1117 //0x0064
    otp_write(0x0302, 0x0001);
    otp_write(0x0300, 0x0004);
    otp_write(0x030a, 0x0001);
    otp_write(0x0308, 0x0008);
    
    otp_write(0x0100, 0x0100);//STREAM ON
    mDELAY(10);
    
	otp_write_8(0x0A02, 0x1F);   //Set page of OTP (0 ~ 0x15) ... page 31(Dec)
	zone=zone;
	otp_write(0x0A00, 0x0100);  //Read Enable
    mDELAY(2);
	printk("--->>>[3m2_otp] start_read_otp\n");
	return 1;
}

/*************************************************************************************************
* Function    :  stop_read_otp
* Description :  after read otp , stop and reset otp block setting  
**************************************************************************************************/
void stop_read_otp(void)
{
    //otp_write(0x0A00, 0x04);//make initial state
	otp_write(0x0A00, 0x00);//Disable NVM controller
	printk("--->>>[3m2_otp] stop_read_otp\n");
}
/*************************************************************************************************
* Function    :  get_otp_AWB_flag
* Description :  get otp AWB_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
kal_uint8 get_otp_AWB_flag(kal_uint8 groupIdx)
{
    kal_uint8 flag_AWB = 0x01;
    
    start_read_otp(groupIdx);
    flag_AWB = otp_read_8(0x0A04 + groupIdx*32);
    stop_read_otp();

    printk("--->>>[3m2_otp] S5K3M2 zcw++: flag_AWB[group %d] := 0x%02x \n", groupIdx, flag_AWB );
	return flag_AWB; //((flag_AWB&0xC0)>>6);
}

/*************************************************************************************************
* Function    :  get_otp_LSC_flag
* Description :  get otp LSC_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
kal_uint8 get_otp_LSC_flag(kal_uint8 groupIdx)
{
    kal_uint8 flag_LSC = 0x01;
    
    start_read_otp(groupIdx);
    flag_LSC = otp_read_8(0x0A04 + groupIdx*32);
    stop_read_otp();

    printk("--->>>[3m2_otp] S5K3M2 zcw++: page[%d] flag_LSC := 0x%02x \n", groupIdx,flag_LSC );
	return ((flag_LSC&0xC0)>>6);
}
/*************************************************************************************************
* Function    :  get_otp_AF_flag
* Description :  get otp AF_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
kal_uint8 get_otp_AF_flag(kal_uint8 groupIdx)
{
    kal_uint8 flag_AF = 0x01;
    
    start_read_otp(groupIdx);
    flag_AF = otp_read_8(0x0A14 + groupIdx*32);
    stop_read_otp();

    printk("--->>>[3m2_otp] S5K3M2 zcw++: Group[%d] flag_AF = 0x%02x \n", groupIdx,flag_AF );
	return flag_AF;
}

/*************************************************************************************************
* Function    :  get_otp_module_info
* Description :  get otp MID ,Lens ID, VCM ID, Driver IC ID... 
* Parameters  :  [BYTE] groupIdx : OTP group index , 0x00~0x01
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : module ID data , TRULY ID is 0x0001            
**************************************************************************************************/
void get_otp_module_info(kal_uint8 groupIdx)
{
	kal_uint8 module_id = 0;
	kal_uint8 year  = 0;
	kal_uint8 month = 0;
	kal_uint8 day   = 0;
	kal_uint8 lens_id = 0;
	kal_uint8 vcm_id = 0;
	kal_uint8 driverIC_id = 0;

	start_read_otp(groupIdx);
	module_id = otp_read_8(0x0A06 + groupIdx*32);
	year = otp_read_8(0x0A07 + groupIdx*32);
	month = otp_read_8(0x0A08 + groupIdx*32);
	day = otp_read_8(0x0A09 + groupIdx*32);
	lens_id = otp_read_8(0x0A0a + groupIdx*32);
	vcm_id = otp_read_8(0x0A0b + groupIdx*32);
	driverIC_id = otp_read_8(0x0A0c + groupIdx*32);
	stop_read_otp();

	printk("--->>>[3m2_otp] S5K3M2 zcw++: Module ID = %d.\n",module_id);
	printk("--->>>[3m2_otp] S5K3M2 zcw++: Date Year = %d.\n",year);
    printk("--->>>[3m2_otp] S5K3M2 zcw++:        month = %d.\n",month);
    printk("--->>>[3m2_otp] S5K3M2 zcw++:        day= %d.\n",day);
    printk("--->>>[3m2_otp] S5K3M2 zcw++: Lens ID = %d.\n",lens_id);
    printk("--->>>[3m2_otp] S5K3M2 zcw++: VCM ID = %d.\n",vcm_id);
    printk("--->>>[3m2_otp] S5K3M2 zcw++: DriverIC ID = %d.\n",driverIC_id);

    MID=module_id;
	
}
/*************************************************************************************************
* Function    :  otp_lsc_update
* Description :  Update lens correction 
* Parameters  :  
* Return      :  [bool] 0 : OTP data fail 
                        1 : success            
**************************************************************************************************/
kal_bool otp_lsc_update(void)
{
    //kal_uint8 flag_LSC_Group0 = 0, flag_LSC_Group1 = 0;
    //kal_bool flag_LSC = 0;
    //flag_LSC_Group0 = get_otp_LSC_flag(0);
    //flag_LSC_Group1 = get_otp_LSC_flag(1);
    //flag_LSC = ((OTP_GROUP_VALID==flag_LSC_Group0) || (OTP_GROUP_VALID==flag_LSC_Group1));

    //if(flag_LSC)
    {
        //LSC correct
        printk("--->>>[3m2_otp] S5K3M2 zcw++: LSC Data Valid...Start Auto Correct\n");
        otp_write_8(0x0B00,0x01);
		otp_write(0x3058,0x0900);
        return 1;
    }
    //printk("S5K3M2 zcw++: OTP LSC Data Empty or Invalid ...Fail!\n");
    //return 0;
}
/*************************************************************************************************
* Function    :  get_otp_wb
* Description :  Get WB data    
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f      
**************************************************************************************************/

kal_bool get_otp_wb(kal_uint8 groupIdx)
{
    start_read_otp(groupIdx);
    R_Gr_Ratio = (otp_read_8(0x0A0E + groupIdx*32)<<8)|otp_read_8(0x0A0F + groupIdx*32);
    B_Gr_Ratio = (otp_read_8(0x0A10 + groupIdx*32)<<8)|otp_read_8(0x0A11 + groupIdx*32);
    Gb_Gr_Ratio = (otp_read_8(0x0A12 + groupIdx*32)<<8)|otp_read_8(0x0A13 + groupIdx*32);
	stop_read_otp();

	RG_Ratio_typical = 547;
    BG_Ratio_typical = 612;
    
    printk("--->>>[3m2_otp] S5K3M2 zcw++:Group[%d] r_Gr=%d,b_Gr=%d,Gb_Gr=%d \n",groupIdx, R_Gr_Ratio, B_Gr_Ratio, Gb_Gr_Ratio);
	return 1;
}
/*************************************************************************************************
* Function    :  otp_wb_update
* Description :  Update WB correction 
* Return      :  [bool] 0 : OTP data fail 
                        1 : otp_WB update success            
**************************************************************************************************/
kal_bool otp_wb_update(void)
{
    kal_uint16 R_GAIN;
    kal_uint16 B_GAIN;
    kal_uint16 G_GAIN;
    kal_uint16 G_gain_R;
    kal_uint16 G_gain_B;

    printk("--->>>[3m2_otp] S5K3M2 zcw++: OTP WB Update Start: \n"); 
    if(!R_Gr_Ratio || !B_Gr_Ratio||!Gb_Gr_Ratio ||!RG_Ratio_typical)
    {
        printk("--->>>[3m2_otp] S5K3M2 zcw++: OTP WB ratio Data Err!\n");
        return 0;
    }
    if(B_Gr_Ratio < BG_Ratio_typical)
    {
        if(R_Gr_Ratio < RG_Ratio_typical)
        {
            G_GAIN = GAIN_DEFAULT;
            B_GAIN = GAIN_DEFAULT * BG_Ratio_typical / B_Gr_Ratio;
            R_GAIN = GAIN_DEFAULT * RG_Ratio_typical / R_Gr_Ratio;
        }
		else
        {
		  	R_GAIN = GAIN_DEFAULT;
            G_GAIN = GAIN_DEFAULT * R_Gr_Ratio / RG_Ratio_typical;
            B_GAIN = G_GAIN * BG_Ratio_typical / B_Gr_Ratio;	        
        }
	}
    else
    {
        if(R_Gr_Ratio < RG_Ratio_typical)
        {
            B_GAIN = GAIN_DEFAULT;
            G_GAIN = GAIN_DEFAULT * B_Gr_Ratio / BG_Ratio_typical;
            R_GAIN = G_GAIN * RG_Ratio_typical / R_Gr_Ratio;
        }
        else
        {
            G_gain_B = GAIN_DEFAULT* B_Gr_Ratio / BG_Ratio_typical;
            G_gain_R = GAIN_DEFAULT* R_Gr_Ratio / RG_Ratio_typical;
            
            if(G_gain_B > G_gain_R)
            {
                B_GAIN = GAIN_DEFAULT;
                G_GAIN = G_gain_B;
                R_GAIN = G_GAIN * RG_Ratio_typical / R_Gr_Ratio;
            }
            else
            {
                R_GAIN = GAIN_DEFAULT;
                G_GAIN = G_gain_R;
                B_GAIN = G_GAIN * BG_Ratio_typical / B_Gr_Ratio;
            }	        
        }		
    }
    
    printk("--->>>[3m2_otp] S5K3M2 zcw++: [R_GAIN=%d],[G_GAIN=%d],[B_GAIN=%d] \n",R_GAIN, G_GAIN, B_GAIN);
    otp_write(GAIN_RED_ADDR, R_GAIN);      
    otp_write(GAIN_BLUE_ADDR, B_GAIN);     
    otp_write(GAIN_GREEN1_ADDR, G_GAIN); //Green 1 default gain 1x     
    otp_write(GAIN_GREEN2_ADDR, G_GAIN); //Green 2 default gain 1x

    otp_write_8(0x3056, 0x01);
    printk("--->>>[3m2_otp] S5K3M2 zcw++: OTP WB Update Finished!  Write 0x3056=1 \n");    
    return 1;
}

/*vivo zcw++ 20140903 Add for OTP AF info*/
int  S5K3M2_vcm_data[2];//[6];
int* get_otp_vcm_data(void)
{
    int i=0;
	start_read_otp(0);
	for(i=0; i<2; i++)
	{
	    //HOR Bit 7:6 = 0x01 ,UP 5:4=0x01, DOWN 3:2=0x01
	    //if((0xFC & get_otp_AF_flag(i))==0x54)
	    //初期Truly 仅烧水平方向的AF 数据
	    if((0xFC & get_otp_AF_flag(i))==0x40)
	    {
	        printk("--->>>[3m2_otp] S5K3M2 zcw++ AF group [%d] is valid!\n",i);
	        break;
	    }
	}
	if(2 <= i)
	{
	    printk("--->>>[3m2_otp] S5K3M2 zcw++ AF Data Empty or Invalid!\n");
	    stop_read_otp();
	    return NULL;
	}
	else
	{
	    S5K3M2_vcm_data[1]= (otp_read_8(0x0A16 + i*32)<<8)+otp_read_8(0x0A15 + i*32);//Hor Macro
	    S5K3M2_vcm_data[0]= (otp_read_8(0x0A18 + i*32)<<8)+otp_read_8(0x0A17 + i*32);//Hor Infinity
        /*
	    S5K3M2_vcm_data[2]= (otp_read_8(0x0A1A + i*32)<<8)+otp_read_8(0x0A19 + i*32);//Up Macro
	    S5K3M2_vcm_data[3]= (otp_read_8(0x0A1C + i*32)<<8)+otp_read_8(0x0A1B + i*32);//Up Infinity
	    S5K3M2_vcm_data[4]= (otp_read_8(0x0A1E + i*32)<<8)+otp_read_8(0x0A1D + i*32);//Down Macro
	    S5K3M2_vcm_data[5]= (otp_read_8(0x0A20 + i*32)<<8)+otp_read_8(0x0A1F + i*32);//Down Infinity
	    */
	    printk("--->>>[3m2_otp] S5K3M2 zcw++ AF vcm[0]Inf=%d vcm[1]Macro=%d\n", S5K3M2_vcm_data[0],S5K3M2_vcm_data[1]);
        stop_read_otp();
	    return S5K3M2_vcm_data;
	}	
}
/*vivo zcw-- 20140903 Add for OTP AF info*/
/*************************************************************************************************
* Function    :  otp_update()
* Description :  update otp data from otp , it otp data is valid, 
                 it include get ID and WB update function  
* Return      :  [bool] 0 : update fail
                        1 : update success
**************************************************************************************************/
kal_bool otp_update(void)
{
	kal_uint8 FLG = 0x00;
	int i;
	
	for(i=0;i<2;i++)
	{
		FLG = get_otp_AWB_flag(i);
		if(FLG == OTP_GROUP_VALID)
			break;
	}
	if(i==2)
	{
		printk("--->>>[3m2_otp] S5K3M2 zcw++: No OTP Data or OTP data is invalid!!\n");
		return 0;
	}
    get_otp_module_info(i);
    if(MID != TRULY_ID)
	{
		printk("--->>>[3m2_otp] S5K3M2 zcw++: Not Truly Module!!!!\n");
		return 0;
	}
    get_otp_wb(i);
    //get_otp_vcm_data();    //Open for OTP DianJian
    return 1;	
    
}

//======================================================================


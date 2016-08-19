/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>   /* proc file use */
#include <linux/dma-mapping.h>
#include <linux/module.h>/*Luke++150701=For 3.18 build pass*/
#include <linux/seq_file.h>
#include <sync_write.h> /*Luke--150701=For 3.18 build pass*/
#include <linux/types.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "s5k3m2_otp.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3m2mipi_Sensor.h"
//#define CAM_CALGETDLT_DEBUG
//#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#include <linux/syslog.h>
#define PFX "s5k3m2otp"

#define CAM_CALINF(fmt, arg...)    //xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " fmt, __FUNCTION__, ##arg)
#define CAM_CALDB(fmt, arg...)    //xlog_printk(ANDROID_LOG_DEBUG   , PFX, "[%s] " fmt, __FUNCTION__, ##arg)
#define CAM_CALERR(fmt, arg...)    //xlog_printk(ANDROID_LOG_ERROR   , PFX, "[%s] " fmt, __FUNCTION__, ##arg)
#else
#define CAM_CALDB(x,...)
#endif
#define PAGE_SIZE_ 256
#define BUFF_SIZE 8

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 0

//extern u8 OTPData[];
u8 OTPData[] = {0};
int otp_flag=0;
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
//static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, S5K3M2OTP_DEVICE_ID>>1)};//A0 for page0 A2 for page 2 and so on for 8 pages

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);

static const struct of_device_id s5k3m2_otp_dt_match[] = {
	{.compatible = "mediatek,camera_otp"},
	{},
};


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
/*
static kal_uint8 eeprom_otp_read_8(kal_uint32 addr, kal_uint16 i2cId)
{
    kal_uint8 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, i2cId);
    return get_byte;
}
*/
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
int afGroupIdx;
int* get_otp_vcm_data(void)
{
    int i=0;
	start_read_otp(0);
	for(i=0; i<2; i++)
	{
	    //HOR Bit 7:6 = 0x01 ,UP 5:4=0x01, DOWN 3:2=0x01
	    //if((0xFC & get_otp_AF_flag(i))==0x54)
	    //初期Truly 仅烧水平方向的AF 数据
	    if((get_otp_AF_flag(i))==0x01) //if((0xFC & get_otp_AF_flag(i))==0x40)
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
	    S5K3M2_vcm_data[1]= (otp_read_8(0x0A16 + i*32)<<8)+otp_read_8(0x0A17 + i*32);//Hor Macro
	    S5K3M2_vcm_data[0]= (otp_read_8(0x0A18 + i*32)<<8)+otp_read_8(0x0A19 + i*32);//Hor Infinity
        /*
	    S5K3M2_vcm_data[2]= (otp_read_8(0x0A1A + i*32)<<8)+otp_read_8(0x0A19 + i*32);//Up Macro
	    S5K3M2_vcm_data[3]= (otp_read_8(0x0A1C + i*32)<<8)+otp_read_8(0x0A1B + i*32);//Up Infinity
	    S5K3M2_vcm_data[4]= (otp_read_8(0x0A1E + i*32)<<8)+otp_read_8(0x0A1D + i*32);//Down Macro
	    S5K3M2_vcm_data[5]= (otp_read_8(0x0A20 + i*32)<<8)+otp_read_8(0x0A1F + i*32);//Down Infinity
	    */
		afGroupIdx = i;
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
	printk("--->>> otp_update: groupIdx = %d\n", i);
    get_otp_module_info(i);
    if(MID != TRULY_ID)
	{
		printk("--->>>[3m2_otp] S5K3M2 zcw++: Not Truly Module!!!!\n");
		return 0;
	}
    get_otp_wb(i);
    get_otp_vcm_data();    //Open for OTP DianJian
    return 1;	
    
}

//======================================================================

/*******************************************************************************
* iReadReg
********************************************************************************/
int iReadCAM_CAL(u16 a_u2Addr, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr>>8), (char)(a_u2Addr & 0xFF) };

    spin_lock(&g_CAM_CALLock); //for SMP
      g_pstI2Cclient->addr = S5K3M2OTP_DEVICE_ID>>1;
	g_pstI2Cclient->timing=400;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
	
    if (i4RetValue != 2)
    {
        printk("--->>>[CAM_CAL] I2C send read address failed!! \n");
	    printk("--->>>[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );
        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
	printk("--->>>[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != 1)
    {
        printk("--->>>[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 

    return 0;
}


static kal_uint8 Read_AF_Otp(kal_uint16 address,unsigned char *iBuffer,unsigned int buffersize)
{	
	/*u8 readbuff, i;

	for(i=0;i<buffersize;i++)
		{
	            iReadCAM_CAL((address+i),&readbuff);
			*(iBuffer+i)=readbuff;
		      CAM_CALDB("read af i = %d data[i]=0x%x\n",i,iBuffer[i]);
		      printk("--->>>read af i = %d data[i]=0x%x\n",i,iBuffer[i]);
		}*/
	/*s5k3m2 af otp*/
	u8 readbuff, i;
	printk("--->>> Read_AF_Otp: address = 0x%x\n", address);
	for(i=0;i<buffersize;i++){
		if(address == 0x0a18 || address == 0x0a16){
			start_read_otp(0);
			*(iBuffer+i) = (unsigned char)otp_read_8(address-i+1 + afGroupIdx*32);
			printk("--->>> Read_AF_Otp: address = 0x%x, iBuffer[%d] = 0x%x\n", address-i+1, i, *((unsigned char*)(iBuffer+i)));
			stop_read_otp();
		}else{
			iReadCAM_CAL((address-i+1),&readbuff);
			*(iBuffer+i)=readbuff;
			printk("read af i = %d data[i]=0x%x\n",i,iBuffer[i]);
		}
	}
	printk("--->>> Read_AF_Otp: af_otp_data = 0x%x, afGroupIdx = %d\n", *((unsigned short*)iBuffer), afGroupIdx);
	return KAL_TRUE;

}


static kal_uint8 Read_AWB_Otp(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{
	u8 readbuff, i, j;
  	printk("Read_AWB_Otp address=%x, buffersize=%d\r\n",address,buffersize);
	for(i=0;i<buffersize;i++)
		{
		if(address == 0x001d || address == 0x0025){
		    j=i*2+1;
	            iReadCAM_CAL((address+j),&readbuff);
			*(iBuffer+i)=readbuff;
		      printk("--->>>read awb j = %d, data[i]=0x%x\n",j,iBuffer[i]);
		}else{
	            iReadCAM_CAL((address+i),&readbuff);
			*(iBuffer+i)=readbuff;
		      //CAM_CALDB("read awb data[i]=0x%x\n",i,iBuffer[i]);
		      printk("--->>>read awb i = %d, data[i]=0x%x\n",i,iBuffer[i]);
		}
	}
	
	return KAL_TRUE;

}



 kal_bool Read_LSC_Otp(kal_uint16 address,unsigned char *iBuffer,unsigned int buffersize)
 {

	u8 readbuff;
	int i = 0;
	printk("--->>>Read_LSC_Otp address=%x, buffersize=%d\r\n", address, buffersize);

	if(otp_flag){
		for(i=0;i<buffersize;i++)
			iBuffer[i]=OTPData[i];
	}
	else{
		for(i=0;i<buffersize;i++)
		{
	       iReadCAM_CAL((address+i),&readbuff);
			*(iBuffer+i)=readbuff;
			printk("--->>>read lsc data[%d]=0x%x\n",i,iBuffer[i]);
		}
	}
	otp_flag=0;
	return KAL_TRUE;
}


 void ReadOtp(kal_uint16 address,unsigned char *iBuffer,unsigned int buffersize)
{
		kal_uint16 i = 0;
		u8 readbuff;
		int ret ;
		printk("ReadOtp buffersize=%d\r\n",buffersize);	
		CAM_CALDB("[CAM_CAL]ENTER address:0x%x buffersize:%d\n ",address, buffersize);
		printk("--->>> ReadOtp-layoutcheck: ENTER address 0x%x buffersize 0x%x\n", address, buffersize);
		if (1)
		{
			for(i = 0; i<buffersize; i++)
			{	
				if(address == 0x00000A06)
				{
					start_read_otp(0);
					*(iBuffer+i) = (unsigned char) otp_read_8(address + i);
					printk("--->>> ReadOtp-layoutcheck: iBuffer[%d] = %d\n", i, *(iBuffer+i));
					stop_read_otp();
				}
				else
				{				
					ret= iReadCAM_CAL(address+i,&readbuff);
					CAM_CALDB("[CAM_CAL]address+i = 0x%x,readbuff = 0x%x\n",(address+i),readbuff );
					*(iBuffer+i) =(unsigned char)readbuff;
				}

			}
		}
}

//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	return 0;
}

//Burst Read Data  iReadData(0x00,932,OTPData);
 int iReadData(kal_uint16 ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   //int  i4RetValue = 0;
   printk("--->>>iReadData address=%x, buffersize=%d\r\n",ui4_offset,ui4_length);
	if(ui4_length == 1) //(ui4_length ==3) // layout check
    {	
		printk("&*&*&*&*&*&*&*&*\n");
	    ReadOtp(ui4_offset, pinputdata, ui4_length);
	   	
    }
	else if(ui4_length ==4)// awb 
    {
		
	    Read_AWB_Otp(ui4_offset, pinputdata, ui4_length);
	   	
    }
	else if(ui4_length ==2)// af
    {
		
	   Read_AF_Otp(ui4_offset, pinputdata, ui4_length);
	   	
    }
      else{  //lsc
		printk("--->>>ui4_offset = %x ******\n", ui4_offset);
		Read_LSC_Otp(ui4_offset, pinputdata, ui4_length);

   }

   return 0;
}


#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    //compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    //err |= get_user(p, &data->pu1Params);
    //err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long s5k3m2otp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	
	CAM_CALDB("[CAMERA SENSOR] COMPAT_CAM_CALIOC_G_READ\n");
	CAM_CALDB("[CAMERA SENSOR] s5k3m2otp_Ioctl_Compat,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            //CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
            printk("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
	CAM_CALDB("[S24CAM_CAL] ioctl\n");

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB(" ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALDB("ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB(" init Working buffer address 0x%p  command is 0x%x\n", pu1Params, a_u4Command);


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    }
	
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            CAM_CALDB("[S24CAM_CAL] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[S24CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            CAM_CALDB("[CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[CAM_CAL] length %d \n", ptempbuf->u4Length);
            //CAM_CALDB("[CAM_CAL] Before read Working buffer address 0x%p \n", pu1Params);
			//otp_flag=1;
            i4RetValue = iReadData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
            CAM_CALDB("[S24CAM_CAL] After read Working buffer data  0x%x \n", *pu1Params);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

            break;
        default :
      	     CAM_CALDB("[S24CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[S24CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[S24CAM_CAL] to user  Working buffer address 0x%p \n", pu1Params);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[S24CAM_CAL] CAM_CAL_Open\n");
    printk("--->>>[S24CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[S24CAM_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
    mdelay(2);
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = s5k3m2otp_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[S24CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[S24CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};



static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,
    .remove = CAM_CAL_i2c_remove,
//   .detect = CAM_CAL_i2c_detect,
    .driver = {
		.name  = CAM_CAL_DRVNAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(s5k3m2_otp_dt_match),
	},
    .id_table = CAM_CAL_i2c_id,
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
int i4RetValue = 0;
	//int ret = 0;
	//u8 buff;
    CAM_CALDB("[S24CAM_CAL] Attach I2C \n");
//    spin_lock_init(&g_CAM_CALLock);
	printk("--->>> enter %s ******\n", __func__);
    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = S5K3M2OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP
	
	//ret = iReadCAM_CAL(0x00, &buff);
	//printk("--->>> iReadCAM_CAL[0x%x] value = %d ret = %d ******\n", 0x00, buff, ret);
    CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[S24CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }


    CAM_CALDB("[S24CAM_CAL] Attached!! \n");
	printk("--->>>[3M2_CAM_CAL] Attached!! \n");
    return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
	int ret = 0;
	printk("--->>> enter %s ******\n", __func__);
	ret = i2c_add_driver(&CAM_CAL_i2c_driver);
	printk("--->>> ret = %d ******\n", ret);
	if(ret != 0)
		printk("--->>> i2c_add_driver error ******\n");
    return ret;
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{

    //i2c_register_board_info(1, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register S24CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");



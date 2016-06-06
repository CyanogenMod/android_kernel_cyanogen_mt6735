#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/semaphore.h>
#include <linux/gpio.h>

#include <mt-plat/mt_pwm.h>
#include <mach/mt_clkmgr.h>
#include <mt-plat/upmu_common.h>
#include <linux/regulator/consumer.h>

//#define SKY81294
//#define LM3642TLX
#define SGM3785
//MTK Internal chip
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(TAG_NAME "%s: " fmt, __func__ , ##arg)

//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG PK_DBG_NONE
#endif

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */

static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);

static struct work_struct workTimeOut;
static void work_timeOutFunc(struct work_struct *data);

extern int aeon_gpio_set(const char *name); //sanford.lin

#if defined(SKY81294)  //SKY81294

#define STROBE_DEVICE_ID 0x6E

//#define FLASH_GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
//#define FLASH_GPIO_ENM GPIO_CAMERA_FLASH_MODE_PIN

static int g_bLtVersion=0;
struct mutex g_strobeLock;
/*****************************************************************************
Functions
*****************************************************************************/
//extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

#define SKY81294_NAME "leds-SKY81294"
static struct i2c_client *SKY81294_i2c_client = NULL;
//static struct i2c_board_info __initdata i2c_SKY81294={I2C_BOARD_INFO(SKY81294_NAME, STROBE_DEVICE_ID>>1)};

//static int readReg(u8 reg)
//{
//	int val=0;

//	mutex_lock(&g_strobeLock);
//	val =  i2c_smbus_read_byte_data(SKY81294_i2c_client, reg);
//	mutex_unlock(&g_strobeLock);

//	return val;
//}

static int writeReg(u8 reg, u8 data)
{
	int ret=0;

	mutex_lock(&g_strobeLock);
	ret =  i2c_smbus_write_byte_data(SKY81294_i2c_client, reg, data);
	mutex_unlock(&g_strobeLock);

	if (ret < 0)
		printk("failed writting at 0x%02x\n", reg);

	return ret;
}

static int SKY81294_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = -1;

	PK_DBG("SKY81294_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "SKY81294 i2c functionality check fail.\n");
		return err;
	}

	mutex_init(&g_strobeLock);
	SKY81294_i2c_client = client;

	PK_DBG("SKY81294 Initializing is done \n");

	return 0;
}

static int SKY81294_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id SKY81294_id[] = {
	{SKY81294_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id SKY81294_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver SKY81294_i2c_driver = {
	.driver = {
		.name  = SKY81294_NAME,
	#ifdef CONFIG_OF
		.of_match_table = SKY81294_of_match,
	#endif
	},
	.probe	= SKY81294_probe,
	.remove   = SKY81294_remove,
	.id_table = SKY81294_id,
};

static int __init SKY81294_init(void)
{
	printk("SKY81294_init\n");

	//i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL, &i2c_SKY81294, 1);

	return i2c_add_driver(&SKY81294_i2c_driver);
}

static void __exit SKY81294_exit(void)
{
	i2c_del_driver(&SKY81294_i2c_driver);
}

module_init(SKY81294_init);
module_exit(SKY81294_exit);


static int FL_Enable(void)
{
	int val;
	char buf[2];

	if(g_duty<0)
		g_duty=0;
	else if(g_duty>16)
		g_duty=16;

	if(g_duty<=2)
	{
		if(g_duty==0)
			val=6;
		else if(g_duty==1)
			val=8;
		else //if(g_duty==2)
			val=10;
	
		buf[0]=2;
		buf[1]=val;
		writeReg(buf[0], buf[1]);
		buf[0]=3;
        buf[1]=0x09;
        writeReg(buf[0], buf[1]);
	}
	else
	{
		buf[0]=0;
		buf[1]=0x0e;//g_duty-1;
		writeReg(buf[0], buf[1]);
		buf[0]=3;
		buf[1]=0x0a;
		writeReg(buf[0], buf[1]);
	}
	PK_DBG(" FL_Enable line=%d\n",__LINE__);

    return 0;
}

static int FL_Disable(void)
{
	char buf[2];

	buf[0]=3;
	buf[1]=0x08;
	writeReg(buf[0], buf[1]);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	g_duty = duty;
    return 0;
}

static int FL_Init(void)
{
	//int regVal0;
	char buf[2];

	buf[0]=0x3;
	buf[1]=0x08;
	writeReg(buf[0], buf[1]);

	g_bLtVersion=0;

	//if(mt_set_gpio_mode(FLASH_GPIO_ENM,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
	//if(mt_set_gpio_dir(FLASH_GPIO_ENM,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	//if(mt_set_gpio_out(FLASH_GPIO_ENM,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	aeon_gpio_set("aeon_flash_enm0");
	//if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
	//if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	//if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	aeon_gpio_set("aeon_flash_enf0");
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}

static int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}

#elif defined(LM3642TLX)  //LM3642TLX-LT/NOPB

#define STROBE_DEVICE_ID 0xC6

//#define FLASH_GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
//#define FLASH_GPIO_ENM GPIO_CAMERA_FLASH_MODE_PIN

static int g_bLtVersion=0;
struct mutex g_strobeLock;
/*****************************************************************************
Functions
*****************************************************************************/
//extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

#define LM3642_NAME "leds-LM3642"
static struct i2c_client *LM3642_i2c_client = NULL;
//static struct i2c_board_info __initdata i2c_LM3642={I2C_BOARD_INFO(LM3642_NAME, STROBE_DEVICE_ID>>1)};

static int readReg(u8 reg)
{
	int val=0;

	mutex_lock(&g_strobeLock);
	val =  i2c_smbus_read_byte_data(LM3642_i2c_client, reg);
	mutex_unlock(&g_strobeLock);

	return val;
}

static int writeReg(u8 reg, u8 data)
{
	int ret=0;

	mutex_lock(&g_strobeLock);
	ret =  i2c_smbus_write_byte_data(LM3642_i2c_client, reg, data);
	mutex_unlock(&g_strobeLock);

	if (ret < 0)
		printk("failed writting at 0x%02x\n", reg);

	return ret;
}

static int LM3642_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = -1;

	PK_DBG("LM3642_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "LM3642 i2c functionality check fail.\n");
		return err;
	}

	mutex_init(&g_strobeLock);
	LM3642_i2c_client = client;

	PK_DBG("LM3642 Initializing is done \n");

	return 0;
}

static int LM3642_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id LM3642_id[] = {
	{LM3642_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id LM3642_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver LM3642_i2c_driver = {
	.driver = {
		.name  = LM3642_NAME,
	#ifdef CONFIG_OF
		.of_match_table = LM3642_of_match,
	#endif
	},
	.probe	= LM3642_probe,
	.remove   = LM3642_remove,
	.id_table = LM3642_id,
};

static int __init LM3642_init(void)
{
	printk("LM3642_init\n");

	//i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL, &i2c_LM3642, 1);

	return i2c_add_driver(&LM3642_i2c_driver);
}

static void __exit LM3642_exit(void)
{
	i2c_del_driver(&LM3642_i2c_driver);
}

module_init(LM3642_init);
module_exit(LM3642_exit);


static int FL_Enable(void)
{
	int val;
	char buf[2];

	if(g_duty<0)
		g_duty=0;
	else if(g_duty>10)
		g_duty=10;

	if(g_duty<=2)
	{
		if(g_bLtVersion==1)
		{
			if(g_duty==0)
				val=3;
			else if(g_duty==1)
				val=5;
			else //if(g_duty==2)
				val=7;
		}
		else
		{
			if(g_duty==0)
				val=1;
			else if(g_duty==1)
				val=3;
			else //if(g_duty==2)
				val=6;
		}

		buf[0]=9;
		buf[1]=(val << 4) & 0x70;
		writeReg(buf[0], buf[1]);
		readReg(9);
		buf[0]=10;
        buf[1]=0x03;
        writeReg(buf[0], buf[1]);
		readReg(10);
        buf[0]=10;
		buf[1]=0x02;
		writeReg(buf[0], buf[1]);
		readReg(10);
	}
	else
	{
		buf[0]=9;
		buf[1]=g_duty-1;
		writeReg(buf[0], buf[1]);
		readReg(9);

		buf[0]=10;
		buf[1]=0x03;
		writeReg(buf[0], buf[1]);
		readReg(10);
	}
	PK_DBG(" FL_Enable line=%d\n",__LINE__);

    readReg(0);
	readReg(1);
	readReg(6);
	readReg(8);
	readReg(9);
	readReg(0xa);
	readReg(0xb);

    return 0;
}

static int FL_Disable(void)
{
	char buf[2];

	buf[0]=10;
	buf[1]=0x00;
	writeReg(buf[0], buf[1]);
	readReg(10);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	g_duty = duty;
    return 0;
}

static int FL_Init(void)
{
	//int regVal0;
	char buf[2];

	buf[0]=0xa;
	buf[1]=0x0;
	writeReg(buf[0], buf[1]);
	readReg(0xa);

	buf[0]=0x8;
	buf[1]=0x47;
	writeReg(buf[0], buf[1]);
	readReg(8);

	buf[0]=9;
	buf[1]=0x35;
	writeReg(buf[0], buf[1]);
	readReg(9);

	//regVal0 = readReg(0);
	//if(regVal0==1)
	//    g_bLtVersion=1;
	//else
	//    g_bLtVersion=0;

	g_bLtVersion=0;

    //PK_DBG(" FL_Init regVal0=%d isLtVer=%d\n",regVal0, g_bLtVersion);

	//if(mt_set_gpio_mode(FLASH_GPIO_ENM,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
	//if(mt_set_gpio_dir(FLASH_GPIO_ENM,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	//if(mt_set_gpio_out(FLASH_GPIO_ENM,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	aeon_gpio_set("aeon_flash_enm0");
	//if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
	//if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	//if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	aeon_gpio_set("aeon_flash_enf0");
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}

static int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}
/*
static int FL_getErr(int* err)
{
    int reg;
    int reg2;
    *err = readReg(0x0b);

    reg = readReg(0x08);
    reg2 = readReg(0x09);
    PK_DBG(" FL_getErr line=%d %d\n",reg,reg2);
    return 0;
}
*/
#elif defined(SGM3785)  //SGM3785

//#define FLASH_GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
//#define FLASH_GPIO_ENM GPIO_CAMERA_FLASH_MODE_PIN
//#define FLASH_GPIO_ENM_M_PWM GPIO_CAMERA_FLASH_MODE_PIN_M_PWM
#define PMW_NUM 3
/*****************************************************************************
Functions
*****************************************************************************/
#if 0
static void gpio_pwm_flash_15(void)
{
	struct pwm_spec_config pwm_setting;
	printk("%s Enter\n",__func__);
	//mt_set_gpio_mode(FLASH_GPIO_ENM,FLASH_GPIO_ENM_M_PWM);
	aeon_gpio_set("aeon_flash_enm_pwm");
	pwm_setting.pwm_no  = PMW_NUM;
	pwm_setting.mode    = PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV8;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0x000003FF;  //15%
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0x00000000;
	pwm_set_spec_config(&pwm_setting);
}
#endif

static void gpio_pwm_flash_50(void)
{
	struct pwm_spec_config pwm_setting;
	printk("%s Enter\n",__func__);
	//mt_set_gpio_mode(FLASH_GPIO_ENM,FLASH_GPIO_ENM_M_PWM);
	//aeon_gpio_set("aeon_flash_enm_pwm");
	pwm_setting.pwm_no  = PMW_NUM;
	pwm_setting.mode    = PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV8;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xffffffff;   //50%
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0x00ffffff;
	pwm_set_spec_config(&pwm_setting);
}

static void gpio_flash_close(void)
{
	PK_DBG(" gpio_flash_close line=%d\n",__LINE__);
	//mt_set_gpio_mode(FLASH_GPIO_ENM, GPIO_MODE_00);
  	//mt_set_gpio_dir(FLASH_GPIO_ENM, GPIO_DIR_OUT);
	//mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ZERO);
	aeon_gpio_set("aeon_flash_enm0");
	//mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
	//mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT);
	//mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
	//aeon_gpio_set("aeon_flash_enf0");
}

static int FL_Enable(void)
{
	//struct pwm_spec_config pwm_setting ;
	PK_DBG(" FL_Enable g_duty = %d\n",g_duty);

	if(g_duty > 1)//flashlight
	{
		aeon_gpio_set("aeon_flash_enm0");
		mdelay(2); 
		//aeon_gpio_set("aeon_flash_enf1");
		gpio_pwm_flash_50();
		//mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
		//mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_OUT);
		//mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ONE);
		//aeon_gpio_set("aeon_flash_enf1");
	}
	else//torch
	{
		//mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00);
		//mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_OUT);
		//mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
		//aeon_gpio_set("aeon_flash_enf0");
		//pwm_setting.pwm_no  = PMW_NUM;
		//mt_pwm_disable(pwm_setting.pwm_no, false);
		//mt_set_gpio_mode(FLASH_GPIO_ENM, GPIO_MODE_00);
		//mt_set_gpio_dir(FLASH_GPIO_ENM, GPIO_DIR_OUT);
		//mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ONE);
		aeon_gpio_set("aeon_flash_enm1");
		//mdelay(10); 
		//gpio_pwm_flash_50();
	}
    return 0;
}

static int FL_Disable(void)
{
	struct pwm_spec_config pwm_setting ;
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	pwm_setting.pwm_no  = PMW_NUM;
	mt_pwm_disable(pwm_setting.pwm_no, false);
    gpio_flash_close();
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    g_duty = duty;
    return 0;
}

static int FL_Init(void)
{
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    gpio_flash_close();
    return 0;
}

static int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}

#else //MTK Internal chip

#if 1 //def GPIO_CAMERA_FLASH_EN_MOS_PIN
//#define FLASH_GPIO_ENM GPIO_CAMERA_FLASH_EN_MOS_PIN
//#define FLASH_GPIO_ENM_M_PWM GPIO_CAMERA_FLASH_EN_MOS_PIN_M_PWM
#define PMW_NUM 3//PWM1
/*****************************************************************************
Functions
*****************************************************************************/
static void gpio_pwm_flash_15(void)
{
	struct pwm_spec_config pwm_setting;
	printk("%s Enter\n",__func__);
	//mt_set_gpio_mode(FLASH_GPIO_ENM,FLASH_GPIO_ENM_M_PWM);
	//aeon_gpio_set("aeon_flash_enm_pwm");
	pwm_setting.pwm_no  = PMW_NUM;
	pwm_setting.mode    = PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV8;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0x000003FF;  //15%
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0x00000000;
	pwm_set_spec_config(&pwm_setting);
}

static void gpio_pwm_flash_50(void)
{
	struct pwm_spec_config pwm_setting;
	printk("%s Enter\n",__func__);
	//mt_set_gpio_mode(FLASH_GPIO_ENM,FLASH_GPIO_ENM_M_PWM);
	//aeon_gpio_set("aeon_flash_enm_pwm");
	pwm_setting.pwm_no  = PMW_NUM;
	pwm_setting.mode    = PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV8;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xffffffff;   //50%
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0x000fffff;
	pwm_set_spec_config(&pwm_setting);
}

static void gpio_flash_close(void)
{
	PK_DBG(" gpio_flash_close line=%d\n",__LINE__);
	//mt_set_gpio_mode(FLASH_GPIO_ENM, GPIO_MODE_00);
  	//mt_set_gpio_dir(FLASH_GPIO_ENM, GPIO_DIR_OUT);
	//mt_set_gpio_out(FLASH_GPIO_ENM, GPIO_OUT_ZERO);
	//aeon_gpio_set("aeon_flash_enm0");
}

static int FL_Enable(void)
{
	PK_DBG(" FL_Enable g_duty = %d\n",g_duty);

	if(g_duty > 0)//flashlight
	{
		gpio_pwm_flash_50();
	}
	else//torch
	{
		gpio_pwm_flash_15();
	}
    return 0;
}

static int FL_Disable(void)
{
	struct pwm_spec_config pwm_setting ;
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	pwm_setting.pwm_no  = PMW_NUM;
	mt_pwm_disable(pwm_setting.pwm_no, false);
    gpio_flash_close();
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    g_duty = duty;
    return 0;
}

static int FL_Init(void)
{
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    gpio_flash_close();
    return 0;
}

static int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}

#else

static int FL_Enable(void)
{
    PK_DBG(" AEON FL_Enable line=%d\n",__LINE__);
    return 0;
}

static int FL_Disable(void)
{
    PK_DBG(" AEON FL_Disable line=%d\n",__LINE__);
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
    PK_DBG(" AEON FL_dim_duty line=%d\n",__LINE__);
    g_duty = duty;
    return 0;
}

static int FL_Init(void)
{
    PK_DBG(" AEON FL_Init line=%d\n",__LINE__);
    return 0;
}

static int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}
#endif

#endif
/*****************************************************************************
User interface
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;
	if (init_flag==0){
		init_flag=1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs=1000; //1s
		hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		g_timeOutTimer.function=ledTimeOutCallback;
	}
}

static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
    switch(cmd)
    {
		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
    		break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;

    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);
    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
			if (arg == 1) {
				int s;
				int ms;

				if (g_timeOutTimeMs > 1000) {
					s = g_timeOutTimeMs / 1000;
					ms = g_timeOutTimeMs - s * 1000;
				} else {
					s = 0;
					ms = g_timeOutTimeMs;
				}

				if (g_timeOutTimeMs != 0) {
					ktime_t ktime;

					ktime = ktime_set(s, ms * 1000000);
					hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
				}
				FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;

		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}

static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
        /* LED On Status */
        g_strobe_On = TRUE;
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);

    if(strobe_Res)
    {
        printk(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }

    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;
}

static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;
}

FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};

MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}

/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{
    return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);

/*************aeon add for factory mode flashlight test*********/
int Flashlight_Switch=0;//aeon add for factory mode  flashlight test
static int flag = 1;

void Flashlight_ON(void)
{
	//hrtimer_cancel( &g_timeOutTimer );
	FL_dim_duty(1);
	if(0 == strobe_Res)
	{	
		FL_Init();
		Flashlight_Switch=0;
	}
	if(flag==1)
	{
		FL_Enable();
		Flashlight_Switch=1;
	}
}

void Flashlight_OFF(void)
{	
	FL_Uninit();
	Flashlight_Switch=0;
}

EXPORT_SYMBOL(Flashlight_ON);
EXPORT_SYMBOL(Flashlight_OFF);
EXPORT_SYMBOL(Flashlight_Switch);
/**************************end**********************/

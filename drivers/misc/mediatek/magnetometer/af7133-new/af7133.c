/* drivers/i2c/chips/af7133.c - AF7133 compass driver
 *
 * Copyright (C) 2013 VTC Technology Inc.
 * Author: Gary Huang <gary.huang@voltafield.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "cust_mag.h"
#include "af7133.h"
#include "mag.h"
#include <mt-plat/battery_common.h>
#include <mt-plat/mt_boot_common.h>
#include <hwmsensor.h>
#include <mt-plat/mt_gpio.h>

/* Add for auto detect feature */
static int  af7133_local_init(void);
static int vtc_remove(void);
static int af7133_init_flag =0;

static struct mag_init_info af7133_init_info = {
    .name = "af7133E",
    .init = af7133_local_init,
    .uninit = vtc_remove,
};

static int af7133_m_enable(int value);

/*----------------------------------------------------------------------------*/
#define DEBUG 1
#define AF7133_DEV_NAME         "af7133E"
#define DRIVER_VERSION          "2.0.3"
#define DRIVER_RELEASE          "20141015"
/*----------------------------------------------------------------------------*/
#define AF7133_DEFAULT_DELAY     40
/*----------------------------------------------------------------------------*/
#define MSE_TAG                  "MSENSOR"
#define MSE_FUN(f)               printk(KERN_INFO MSE_TAG" %s\r\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)    printk(KERN_ERR MSE_TAG" %s %d : \r\n"fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)    printk(KERN_INFO MSE_TAG fmt, ##args)
#define MSE_VER(fmt, args...)   ((void)0)

static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);
/*----------------------------------------------------------------------------*/
static struct i2c_client *af7133_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
#define FIND_SW_OFFSET_LOOP    5
#define FIND_SW_OFFSET_INDEX   2

static int mag_pos[3][FIND_SW_OFFSET_LOOP];
static int mag_neg[3][FIND_SW_OFFSET_LOOP];
static int mag_offset[3];
static int mag_cnt=0;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id af7133_i2c_id[] = {{AF7133_DEV_NAME,0},{}};

/*----------------------------------------------------------------------------*/
static int af7133_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int af7133_i2c_remove(struct i2c_client *client);
static int af7133_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int af7133_suspend(struct i2c_client *client, pm_message_t msg) ;
static int af7133_resume(struct i2c_client *client);
//static int af7133_local_init(void);
//static int af7133_remove(void);
//static struct platform_driver vtc_sensor_driver;

/* Maintain  cust info here */
struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

/* For  driver get cust info */
struct mag_hw *get_cust_mag(void)
{
    return &mag_cust;
}

/*----------------------------------------------------------------------------*/
typedef enum {
    VTC_TRC_DEBUG  = 0x01,
} AMI_TRC;
/*----------------------------------------------------------------------------*/
struct _af7133_data {
    rwlock_t lock;
    int mode;
    int rate;
    volatile int updated;
} af7133_data;
/*----------------------------------------------------------------------------*/
struct _af7133mid_data {
    rwlock_t datalock;
    rwlock_t ctrllock;
    int controldata[10];
    unsigned int debug;
    int yaw;
    int roll;
    int pitch;
    int nmx;
    int nmy;
    int nmz;
    int nax;
    int nay;
    int naz;
    int mag_status;
} af7133mid_data;
/*----------------------------------------------------------------------------*/
struct af7133_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    atomic_t layout;
    atomic_t trace;
    struct hwmsen_convert   cvt;
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
    {.compatible = "mediatek,msensor"},
    {},
};
#endif
static struct i2c_driver af7133_i2c_driver = {
    .driver = {
        .name  = AF7133_DEV_NAME,
#ifdef CONFIG_OF
    .of_match_table = mag_of_match,
#endif
    },
    .probe      = af7133_i2c_probe,
    .remove     = af7133_i2c_remove,
    .detect     = af7133_i2c_detect,
    .suspend    = af7133_suspend,
    .resume     = af7133_resume,
    .id_table   = af7133_i2c_id,
};

static DEFINE_MUTEX(af7133_mutex);
#define I2C_FLAG_WRITE  0
#define I2C_FLAG_READ   1


/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/

//========================================================================
static int VTC_i2c_Rx(struct i2c_client *client, char *rxData, int length)
{
    uint8_t retry;
    struct i2c_msg msgs[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = rxData,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxData,
        },
    };

    for (retry = 0; retry < 3; retry++)
    {
        if (i2c_transfer(client->adapter, msgs, 2) > 0)
            break;
        else
            mdelay(10);
    }

    if (retry >= 3)
    {
        printk(KERN_ERR "%s: retry over 3\n", __func__);
        return -EIO;
    }
    else
        return 0;
}

static int VTC_i2c_Tx(struct i2c_client *client, char *txData, int length)
{
    int retry;
    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length,
            .buf = txData,
        },
    };

    for (retry = 0; retry <= 3; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) > 0)
            break;
        else
            mdelay(10);
    }

    if (retry > 3)
    {
        printk(KERN_ERR "%s: retry over 3\n", __func__);
        return -EIO;
    }
    else
        return 0;
}
//========================================================================
/*------------------------i2c function for 89-------------------------------------*/
int af7133_i2c_master_operate(struct i2c_client *client,  char *buf, int count, int i2c_flag)
{
/*  int res = 0;
    mutex_lock(&af7133e_af8133i_mutex);
    switch(i2c_flag){
    case I2C_FLAG_WRITE:
    client->addr &=I2C_MASK_FLAG;
    res = i2c_master_send(client, buf, count);
    client->addr &=I2C_MASK_FLAG;
    break;

    case I2C_FLAG_READ:
    client->addr &=I2C_MASK_FLAG;
    client->addr |=I2C_WR_FLAG;
    client->addr |=I2C_RS_FLAG;
    res = i2c_master_send(client, buf, count);
    client->addr &=I2C_MASK_FLAG;
    break;
    default:
    MSE_LOG("af7133_i2c_master_operate i2c_flag command not support!\n");
    break;
    }
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    mutex_unlock(&af7133_mutex);
    return res;
    EXIT_ERR:
    mutex_unlock(&af7133_mutex);
    MSE_ERR("af7133_i2c_transfer fail\n");
    return res;
*/

    if(i2c_flag == I2C_FLAG_READ)
    {
        return (VTC_i2c_Rx(client, buf, count>>8));
    }
    else if(i2c_flag == I2C_FLAG_WRITE)
    {
        return (VTC_i2c_Tx(client, buf, count));
    }

    return 0;
}

static void af7133_power(struct mag_hw *hw, unsigned int on)
{
    mt_set_gpio_out(61, on);
}
/*----------------------------------------------------------------------------*/

static int af7133_GetOpenStatus(void)
{
    wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
    return atomic_read(&open_flag);
}

/*----------------------------------------------------------------------------*/
static int AF7133_Chipset_Init(int mode)
{
    u8 databuf[2];
//  u8 vaule;
    int ret=0;

    if((get_boot_mode() == FACTORY_BOOT)){
        rwlock_init(&af7133mid_data.ctrllock);
        rwlock_init(&af7133mid_data.datalock);
        rwlock_init(&af7133_data.lock);
        memset(&af7133mid_data.controldata[0], 0, sizeof(int)*10);
        af7133mid_data.controldata[0] =    AF7133_DEFAULT_DELAY;  // Loop Delay
        af7133mid_data.controldata[1] =     0;  // Run
        af7133mid_data.controldata[2] =     0;  // Disable Start-AccCali
        af7133mid_data.controldata[3] =     1;  // Enable Start-Cali
        af7133mid_data.controldata[4] =   350;  // MW-Timout
        af7133mid_data.controldata[5] =    10;  // MW-IIRStrength_M
        af7133mid_data.controldata[6] =    10;  // MW-IIRStrength_G
        af7133mid_data.controldata[7] =     0;  // Active Sensors
        af7133mid_data.controldata[8] =     0;  // Wait for define
        af7133mid_data.controldata[9] =     0;  // Wait for define
        atomic_set(&dev_open_count, 0);
    }
	
//  af7133_i2c_client->addr = af7133_i2c_client->addr & I2C_MASK_FLAG;

    databuf[0] = 0x00;
    ret=af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x101, I2C_FLAG_READ);
    databuf[0] = 0x00;
    ret=af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x101, I2C_FLAG_READ);

    if(ret<0){
        MSE_ERR("AF7133E PCODE is incorrect: %d\n", databuf[0]);
        return -3;
    }
    else{
        MSE_ERR("AF7133E PCODE is %d\n", databuf[0]);
        printk("%s chip id:%#x\n",__func__,databuf[0]);
    }

    /*if(databuf[0] != 0x50)
    {
        MSE_ERR("AF7133E PCODE is incorrect: %d\n", databuf[0]);
        return -3;
    }
    else{
        printk("%s chip id:%#x\n",__func__,databuf[0]);
    }*/

  /***
    //disable EN_TEMP
    databuf[0] = 0x13;
    databuf[1] = 0x04;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    //skip reset time: (24+1)*4 = 100 ==> ~4s
    databuf[0] = 0x0D;
    databuf[1] = 0x4B;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    if(mode == AF7133_MODE_SINGLE)
    {
        databuf[0] = AF7133_REG_MODE;
        databuf[1] = AF7133_MODE_SINGLE;
        //i2c_master_send(af7133_i2c_client, databuf, 2);
        af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
    }
    else if(mode == AF7133_MODE_WAKE)
    {
        databuf[0] = AF7133_REG_RATE;
        databuf[1] = AF7133_RATE_25HZ;
        //i2c_master_send(af7133_i2c_client, databuf, 2);
        af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

        databuf[0] = AF7133_REG_MODE;
        databuf[1] = AF7133_MODE_WAKE;
        //i2c_master_send(af7133_i2c_client, databuf, 2);
        af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
    }
    else
    {
        databuf[0] = AF7133_REG_MODE;
        databuf[1] = AF7133_MODE_IDLE;
        //i2c_master_send(af7133_i2c_client, databuf, 2);
        af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
    }
  ***/

    databuf[0] = 0x10;
    databuf[1] = 0x55;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x14;
    databuf[1] = 0x34;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x33;
    databuf[1] = 0x16;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x0B;
    databuf[1] = 0x3C;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x13;
    databuf[1] = 0x00;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x0A;
    databuf[1] = 0x01;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    mag_cnt = 0;
    mag_offset[0] = 0;
    mag_offset[1] = 0;
    mag_offset[2] = 0;

//  write_lock(&af7133_data.lock);
    af7133_data.mode = mode;
//  write_unlock(&af7133_data.lock);

    return 0;
}
/*----------------------------------------------------------------------------*/
static int AF7133_SetMode(int newmode)
{
    int mode = 0;

//  read_lock(&af7133_data.lock);
    mode = af7133_data.mode;
//  read_unlock(&af7133_data.lock);

    //if(mode == newmode) return 0;

    return AF7133_Chipset_Init(mode);
}
/*----------------------------------------------------------------------------*/
static int AF7133_Read_Regiser(unsigned char reg, char* value)
{
//  struct af7133_i2c_data *data = i2c_get_clientdata(af7133_i2c_client);
    unsigned char databuf[10];

    databuf[0] = reg;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x101, I2C_FLAG_READ);

    *value = databuf[0];
    return 0;
}
/*----------------------------------------------------------------------------*/
static int AF7133_ReadSensorData(int *buf, int bufsize)
{
//  struct af7133_i2c_data *data = i2c_get_clientdata(af7133_i2c_client);
    unsigned char databuf[10];
    int output[3];
    int i,j,k;

    if(NULL == af7133_i2c_client)
    {
        *buf = 0;
        return -2;
    }

    // We can read all measured data in once
    databuf[0] = 0x03;
    //databuf[0] = AF7133_REG_DATA;
    //i2c_master_send(af7133_i2c_client, databuf, 1);
    //i2c_master_recv(af7133_i2c_client, databuf, 6);
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x601, I2C_FLAG_READ);

    output[0] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
    output[1] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
    output[2] = ((int) databuf[5]) << 8 | ((int) databuf[4]);

    //buf[0] = output[0] > 32767 ? output[0] - 65536 : output[0];
    //buf[1] = output[1] > 32767 ? output[1] - 65536 : output[1];
    //buf[2] = output[2] > 32767 ? output[2] - 65536 : output[2];

    for(i=0;i<3;i++) output[i] = (output[i] > 32767) ? (output[i] - 65536) : output[i];

    if(mag_cnt >= 0 && mag_cnt <= (FIND_SW_OFFSET_LOOP*2))
    {
        if(mag_cnt >= 1 && mag_cnt <= FIND_SW_OFFSET_LOOP)
            for(i=0;i<3;i++) mag_neg[i][mag_cnt-1] = output[i];
        else if(mag_cnt > FIND_SW_OFFSET_LOOP && mag_cnt <= (FIND_SW_OFFSET_LOOP*2))
            for(i=0;i<3;i++) mag_pos[i][mag_cnt-FIND_SW_OFFSET_LOOP-1] = output[i];

        mag_cnt++;

        if(mag_cnt >= 1 && mag_cnt <= FIND_SW_OFFSET_LOOP)
        {
            databuf[0] = 0x14;
            databuf[1] = 0x34;
            af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
        }
        else if(mag_cnt > FIND_SW_OFFSET_LOOP && mag_cnt <= (FIND_SW_OFFSET_LOOP*2))
        {
            databuf[0] = 0x14;
            databuf[1] = 0x38;
            af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
        }

        if(mag_cnt > (FIND_SW_OFFSET_LOOP*2))
        {
            for(i=0;i<3;i++)
            {
                for(j=0;j<(FIND_SW_OFFSET_LOOP-1);j++)
                {
                    for(k=0;k<(FIND_SW_OFFSET_LOOP-1);k++)
                    {
                        if(mag_neg[i][k] < mag_neg[i][k+1])
                        {
                            int tmp = mag_neg[i][k];
                            mag_neg[i][k] = mag_neg[i][k+1];
                            mag_neg[i][k+1] = tmp;
                        }
                        if(mag_pos[i][k] < mag_pos[i][k+1])
                        {
                            int tmp = mag_pos[i][k];
                            mag_pos[i][k] = mag_pos[i][k+1];
                            mag_pos[i][k+1] = tmp;
                        }
                    }
                }
                mag_offset[i] = (mag_pos[i][(FIND_SW_OFFSET_INDEX)] + mag_neg[i][FIND_SW_OFFSET_INDEX]) / 2;
            }
        }
        for(i=0;i<3;i++) buf[i] = 0;
    }
    else
    {
        for(i=0;i<3;i++) buf[i] = output[i] - mag_offset[i];

        databuf[0] = 0x14;
        databuf[1] = 0x38;
        af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
    }

    //Stephen: print out mag_offset[i]
    //MSE_ERR("AF7133 SW offset: %d, %d, %d\n", mag_offset[0], mag_offset[1], mag_offset[2]);

    //Stephen: print out reg 0x0A ~ 0x14
    databuf[0] = 0x00;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x501, I2C_FLAG_READ);
    MSE_ERR("AF7133 reg 0x0A~0x0E: %x, %x, %x, %x, %x\n", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4]);
    databuf[0] = 0x13;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x201, I2C_FLAG_READ);
    MSE_ERR("AF7133 reg 0x13~0x14: %x, %x\n", databuf[0], databuf[1]);
    databuf[0] = 0x33;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x101, I2C_FLAG_READ);
    MSE_ERR("AF7133 reg 0x33: %x\n", databuf[0]);

    //confirm register setting
    databuf[0] = 0x33;
    databuf[1] = 0x16;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x14;
    databuf[1] = 0xB8;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x0B;
    databuf[1] = 0x3C;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x0D;
    databuf[1] = 0x00;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x0E;
    databuf[1] = 0x00;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    databuf[0] = 0x13;
    databuf[1] = 0x00;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    //Next data
    databuf[0] = 0x0A;
    databuf[1] = 0x01;
    af7133_i2c_master_operate(af7133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
    char strbuf[AF7133_BUFSIZE];
    sprintf(strbuf, "af7133d");
    return sprintf(buf, "%s", strbuf);
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    char strbuf[AF7133_BUFSIZE];
    //AF7133_ReadChipInfo(strbuf, AF7133_BUFSIZE);
    return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
    char strbuf[AF7133_BUFSIZE];
//  AF7133_ReadSensorData(strbuf, AF7133_BUFSIZE);
    return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
    char strbuf[AF7133_BUFSIZE];
    //AF7133_ReadPostureData(strbuf, AF7133_BUFSIZE);
    return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
    char strbuf[AF7133_BUFSIZE];
    //AF7133_ReadCaliData(strbuf, AF7133_BUFSIZE);
    return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_midcontrol_value(struct device_driver *ddri, char *buf)
{
    char strbuf[AF7133_BUFSIZE];
    //AF7133_ReadMiddleControl(strbuf, AF7133_BUFSIZE);
    return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_midcontrol_value(struct device_driver *ddri, const char *buf, size_t count)
{
    int p[10];
    if(10 == sscanf(buf, "%d %d %d %d %d %d %d %d %d %d",&p[0], &p[1], &p[2], &p[3], &p[4],
        &p[5], &p[6], &p[7], &p[8], &p[9]))
    {
//      write_lock(&af7133mid_data.ctrllock);
        memcpy(&af7133mid_data.controldata[0], &p, sizeof(int)*10);
//      write_unlock(&af7133mid_data.ctrllock);
    }
    else
    {
        MSE_ERR("invalid format\n");
    }
    return sizeof(int)*10;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_middebug_value(struct device_driver *ddri, char *buf)
{
    ssize_t len;
//  read_lock(&af7133mid_data.ctrllock);
    len = sprintf(buf, "0x%08X\n", af7133mid_data.debug);
//  read_unlock(&af7133mid_data.ctrllock);

    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_middebug_value(struct device_driver *ddri, const char *buf, size_t count)
{
    int debug;
    if(1 == sscanf(buf, "0x%x", &debug))
    {
//      write_lock(&af7133mid_data.ctrllock);
        af7133mid_data.debug = debug;
//      write_unlock(&af7133mid_data.ctrllock);
    }
    else
    {
        MSE_ERR("invalid format\n");
    }
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_mode_value(struct device_driver *ddri, char *buf)
{
    int mode=0;
//  read_lock(&af7133_data.lock);
    mode = af7133_data.mode;
//  read_unlock(&af7133_data.lock);
    return sprintf(buf, "%d\n", mode);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_mode_value(struct device_driver *ddri, const char *buf, size_t count)
{
    int mode = 0;
    sscanf(buf, "%d", &mode);
    AF7133_SetMode(mode);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = af7133_i2c_client;
    struct af7133_i2c_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
        data->hw->direction,atomic_read(&data->layout), data->cvt.sign[0], data->cvt.sign[1],
        data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct i2c_client *client = af7133_i2c_client;
    struct af7133_i2c_data *data = i2c_get_clientdata(client);
    int layout = 0;

    if(1 == sscanf(buf, "%d", &layout))
    {
        atomic_set(&data->layout, layout);
        if(!hwmsen_get_convert(layout, &data->cvt))
        {
            MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
        }
        else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
        {
            MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
        }
        else
        {
            MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
            hwmsen_get_convert(0, &data->cvt);
        }
    }
    else
    {
        MSE_ERR("invalid format = '%s'\n", buf);
    }

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = af7133_i2c_client;
    struct af7133_i2c_data *data = i2c_get_clientdata(client);
    ssize_t len = 0;

    if(data->hw)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
            data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
    return len;
}

static ssize_t store_status_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct i2c_client *client = af7133_i2c_client;
    struct af7133_i2c_data *data = i2c_get_clientdata(client);
    int value = simple_strtol(buf, NULL, 10);

    data->hw->direction = value;
    if(hwmsen_get_convert(value, &data->cvt)<0)
    {
        MSE_ERR("invalid direction: %d\n", value);
    }

    atomic_set(&data->layout, value);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    struct af7133_i2c_data *obj = i2c_get_clientdata(af7133_i2c_client);
    if(NULL == obj)
    {
        MSE_ERR("af7133E_i2c_data is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct af7133_i2c_data *obj = i2c_get_clientdata(af7133_i2c_client);
    int trace;
    if(NULL == obj)
    {
        MSE_ERR("af7133E_i2c_data is null!!\n");
        return 0;
    }

    if(1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&obj->trace, trace);
    }
    else
    {
//      MSE_ERR("invalid content: '%s', length = %d\n", buf, count);
    }

    return count;
}


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(calidata,    S_IRUGO, show_calidata_value, NULL);
static DRIVER_ATTR(midcontrol,  S_IRUGO | S_IWUSR, show_midcontrol_value, store_midcontrol_value );
static DRIVER_ATTR(middebug,    S_IRUGO | S_IWUSR, show_middebug_value, store_middebug_value );
static DRIVER_ATTR(mode,        S_IRUGO | S_IWUSR, show_mode_value, store_mode_value );
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(status,      S_IRUGO | S_IWUSR, show_status_value, store_status_value);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
/*----------------------------------------------------------------------------*/
static struct driver_attribute *af7133_attr_list[] = {
    &driver_attr_daemon,
    &driver_attr_chipinfo,
    &driver_attr_sensordata,
    &driver_attr_posturedata,
    &driver_attr_calidata,
    &driver_attr_midcontrol,
    &driver_attr_middebug,
    &driver_attr_mode,
    &driver_attr_layout,
    &driver_attr_status,
    &driver_attr_trace,
};
/*----------------------------------------------------------------------------*/
static int af7133_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(af7133_attr_list)/sizeof(af7133_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, af7133_attr_list[idx])))
        {
            MSE_ERR("driver_create_file (%s) = %d\n", af7133_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int af7133_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(af7133_attr_list)/sizeof(af7133_attr_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }


    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, af7133_attr_list[idx]);
    }

    return err;
}

/*----------------------------------------------------------------------------*/
static int af7133_open(struct inode *inode, struct file *file)
{
    struct af7133_i2c_data *obj = i2c_get_clientdata(af7133_i2c_client);
    int ret = -1;
    atomic_inc(&dev_open_count);

    if(atomic_read(&obj->trace) & VTC_TRC_DEBUG)
    {
        MSE_LOG("Open device node:af7133E\n");
    }
    ret = nonseekable_open(inode, file);

    return ret;
}
/*----------------------------------------------------------------------------*/
static int af7133_release(struct inode *inode, struct file *file)
{
    struct af7133_i2c_data *obj = i2c_get_clientdata(af7133_i2c_client);
    atomic_dec(&dev_open_count);
    if(atomic_read(&obj->trace) & VTC_TRC_DEBUG)
    {
        MSE_LOG("Release device node:af7133E\n");
    }
    return 0;
}
/*----------------------------------------------------------------------------*/

static long af7133_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int valuebuf[10];
    int magbuf[3];
    char strbuf[AF7133_BUFSIZE];
    void __user *data;
    long retval=0;
    int mode=0;
	int status, value_divide; 				/* for OPEN/CLOSE_STATUS */
    short sensor_status;        /* for Orientation and Msensor status */
	char buff[128];
    unsigned char reg;
    int mag_layout;

    switch (cmd)
    {
        case MSENSOR_IOCTL_INIT:
            AF7133_Chipset_Init(AF7133_MODE_SINGLE);
            break;

        case MSENSOR_IOCTL_SET_POSTURE:
            data = (void __user *) arg;
            if(data == NULL)
            {
                MSE_ERR("IO parameter pointer is NULL!\r\n");
                break;
            }

            if(copy_from_user(valuebuf, data, sizeof(valuebuf)))
            {
                retval = -EFAULT;
                goto err_out;
            }

            MSE_ERR("af7133E driver: Osensor %d %d %d %d\n", valuebuf[0], valuebuf[1], valuebuf[2], valuebuf[9]);
//          write_lock(&af7133mid_data.datalock);
            af7133mid_data.yaw   = valuebuf[0];
            af7133mid_data.pitch = valuebuf[1];
            af7133mid_data.roll  = valuebuf[2];
            af7133mid_data.nmx = valuebuf[3];
            af7133mid_data.nmy = valuebuf[4];
            af7133mid_data.nmz = valuebuf[5];
            af7133mid_data.nax = valuebuf[6];
            af7133mid_data.nay = valuebuf[7];
            af7133mid_data.naz = valuebuf[8];
            af7133mid_data.mag_status = valuebuf[9];
//          write_unlock(&af7133mid_data.datalock);
            break;

        case ECOMPASS_IOC_GET_OFLAG:
            sensor_status = atomic_read(&o_flag);
            if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
            {
                MSE_ERR("copy_to_user failed.");
                return -EFAULT;
            }
            break;

        case ECOMPASS_IOC_GET_MFLAG:
            sensor_status = atomic_read(&m_flag);
            if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
            {
                MSE_ERR("copy_to_user failed.");
                return -EFAULT;
            }
            break;

        case ECOMPASS_IOC_GET_OPEN_STATUS:
            status = af7133_GetOpenStatus();
            if(copy_to_user(argp, &status, sizeof(status)))
            {
                MSE_LOG("copy_to_user failed.");
                return -EFAULT;
            }
            break;

        case ECOMPASS_IOC_GET_DELAY:
            status = af7133mid_data.controldata[0];
            if(copy_to_user(argp, &status, sizeof(status)))
            {
                MSE_LOG("copy_to_user failed.");
                return -EFAULT;
            }
            break;

        case MSENSOR_IOCTL_SET_CALIDATA:
            break;

        case MSENSOR_IOCTL_READ_CHIPINFO:
            break;

        case MSENSOR_IOCTL_SENSOR_ENABLE:
            break;

        case MSENSOR_IOCTL_READ_SENSORDATA:
            data = (void __user *) arg;
            if(data == NULL)
            {
                MSE_ERR("IO parameter pointer is NULL!\r\n");
                break;
            }
            AF7133_ReadSensorData(magbuf, AF7133_BUFSIZE);

            MSE_ERR("af7133E driver: Msensor %d %d %d\n", magbuf[0], magbuf[1], magbuf[2]);

            if(copy_to_user(data, magbuf, sizeof(magbuf)))
            {
                retval = -EFAULT;
                goto err_out;
            }
            break;

        case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				MSE_ERR( "IO parameter pointer is NULL!\r\n");
				break; 
			}
			
			AF7133_ReadSensorData(magbuf, AF7133_BUFSIZE);
			MSE_ERR("FACTORY af7133E driver: Msensor %d %d %d\n", magbuf[0], magbuf[1], magbuf[2]);

			//status = af7133_GetOpenStatus();	
			//MSE_ERR("af7133E driver: Msensor status %d\n", status);

			value_divide = CONVERT_O_DIV;
			MSE_ERR("af7133E driver: Msensor value_divide %d\n", value_divide);

			sprintf(buff, "%d %d %d %d %d", magbuf[0], magbuf[1], magbuf[2], 1, value_divide);
			if(copy_to_user(data, buff, strlen(buff)+1))
			{ 
				return -EFAULT; 
			} 
			break;                

        case MSENSOR_IOCTL_READ_POSTUREDATA:
            break;

        case MSENSOR_IOCTL_READ_CALIDATA:
            break;

        case MSENSOR_IOCTL_READ_CONTROL:
            data = (void __user *) arg;
            if(data == NULL)
            {
              MSE_ERR("IO parameter pointer is NULL!\r\n");
              break;
            }
            if(copy_from_user(strbuf, data, sizeof(strbuf[0])))
            {
                retval = -EFAULT;
                goto err_out;
            }

            reg = (unsigned char)strbuf[0];
            AF7133_Read_Regiser(reg, strbuf);

            if(copy_to_user(data, strbuf, sizeof(strbuf[0])))
            {
                retval = -EFAULT;
                goto err_out;
            }
            break;

        case MSENSOR_IOCTL_SET_CONTROL:
            break;

        case MSENSOR_IOCTL_SET_MODE:
            data = (void __user *) arg;
            if(data == NULL)
            {
                break;
            }
            if(copy_from_user(&mode, data, sizeof(mode)))
            {
                retval = -EFAULT;
                goto err_out;
            }
            if(AF7133_SetMode(mode))
            {
                retval = -EFAULT;
                goto err_out;
            }
            break;

        case ECOMPASS_IOC_GET_LAYOUT:
            mag_layout = hw->direction;
            if(copy_to_user(argp, &mag_layout, sizeof(mag_layout)))
            {
                retval = -EFAULT;
                goto err_out;
            }
            break;

        default:
            MSE_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            retval = -ENOIOCTLCMD;
            break;
    }
err_out:
    return retval;
}
/*----------------------------------------------------------------------------*/
static struct file_operations af7133_fops = {
//  .owner = THIS_MODULE,
    .open = af7133_open,
    .release = af7133_release,
    .unlocked_ioctl = af7133_unlocked_ioctl,//modified
};
/*----------------------------------------------------------------------------*/
static struct miscdevice af7133_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "msensor",
    .fops = &af7133_fops,
};

static int af7133_m_enable(int value)
{
    if(value == 1)
    {
        af7133mid_data.controldata[7] |= SENSOR_MAGNETIC;
        atomic_set(&m_flag, 1);
        atomic_set(&open_flag, 1);
    }
    else
    {
        af7133mid_data.controldata[7] &= ~SENSOR_MAGNETIC;
        atomic_set(&m_flag, 0);
        if(atomic_read(&o_flag) == 0)
        {
            atomic_set(&open_flag, 0);
        }
    }
    wake_up(&open_wq);

    return 0;
}

static int af7133_m_set_delay(u64 ns)
{
    int value,sample_delay;

    value = (int)ns/1000/1000;

    if(value > 100)
        sample_delay = 100;
    else
        sample_delay = value;

    af7133mid_data.controldata[0] = sample_delay;  // Loop Delay

    return 0;
}

static int af7133_m_open_report_data(int open)
{
    return 0;
}

static int af7133_m_get_data(int* x ,int* y,int* z, int* status)
{
    *x = af7133mid_data.nmx * CONVERT_M;
    *y = af7133mid_data.nmy * CONVERT_M;
    *z = af7133mid_data.nmz * CONVERT_M;
    *status = af7133mid_data.mag_status;

    return 0;
}

static int af7133_o_enable(int value)
{
    printk("af7133_o_enable value=%d\n",value);
    if(value == 1)
    {
        af7133mid_data.controldata[7] |= SENSOR_ORIENTATION;
        atomic_set(&o_flag, 1);
        atomic_set(&open_flag, 1);
    }
    else
    {
        af7133mid_data.controldata[7] &= ~SENSOR_ORIENTATION;
        atomic_set(&o_flag, 0);
        if(atomic_read(&m_flag) == 0)
        {
            atomic_set(&open_flag, 0);
        }
    }
    wake_up(&open_wq);
    return 0;
}

static int af7133_o_set_delay(u64 ns)
{
    int value,sample_delay;

    value = (int)ns/1000/1000;

    if(value > 100)
        sample_delay = 100;
    else
        sample_delay = value;

    af7133mid_data.controldata[0] = sample_delay;  // Loop Delay
    return 0;
}

static int af7133_o_open_report_data(int open)
{

    return 0;
}

static int af7133_o_get_data(int* x ,int* y,int* z, int* status)
{
    printk("af7133_o_get_data\n");
    *x = af7133mid_data.yaw;
    *y = af7133mid_data.pitch;
    *z = af7133mid_data.roll;
    *status = af7133mid_data.mag_status;
    return 0;
}

/*----------------------------------------------------------------------------*/
int af7133_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value, sample_delay;
    struct hwm_sensor_data* msensor_data;

    //MSE_FUN(f);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                MSE_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                /*if(value <= AF7133_DEFAULT_DELAY)
                {
                  sample_delay = AF7133_DEFAULT_DELAY;
                }*/

                if(value > 100) sample_delay = 100;
                else sample_delay = value;

                af7133mid_data.controldata[0] = sample_delay;  // Loop Delay
            }
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                MSE_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
//              read_lock(&af7133mid_data.ctrllock);
                if(value == 1)
                {
                    af7133mid_data.controldata[7] |= SENSOR_MAGNETIC;
                    atomic_set(&m_flag, 1);
                    atomic_set(&open_flag, 1);
                }
                else
                {
                    af7133mid_data.controldata[7] &= ~SENSOR_MAGNETIC;
                    atomic_set(&m_flag, 0);
                    if(atomic_read(&o_flag) == 0)
                    {
                        atomic_set(&open_flag, 0);
                    }
                }
                wake_up(&open_wq);
//              read_unlock(&af7133mid_data.ctrllock);
                // TODO: turn device into standby or normal mode
            }
            break;

        case SENSOR_GET_DATA:
            if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
            {
                MSE_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                msensor_data = (struct hwm_sensor_data *)buff_out;
//              read_lock(&af7133mid_data.datalock);
                msensor_data->values[0] = af7133mid_data.nmx;
                msensor_data->values[1] = af7133mid_data.nmy;
                msensor_data->values[2] = af7133mid_data.nmz;
                msensor_data->status = af7133mid_data.mag_status;
//              read_unlock(&af7133mid_data.datalock);

                msensor_data->values[0] = msensor_data->values[0] * CONVERT_M;
                msensor_data->values[1] = msensor_data->values[1] * CONVERT_M;
                msensor_data->values[2] = msensor_data->values[2] * CONVERT_M;
                msensor_data->value_divide = CONVERT_M_DIV;
            }
            break;
        default:
            MSE_ERR("msensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}

/*----------------------------------------------------------------------------*/
int af7133_orientation_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value, sample_delay;
    struct hwm_sensor_data* osensor_data=NULL;

    printk("af7133_orientation_operate command=%d\n",command);
    MSE_FUN(f);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                MSE_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                /*if(value <= AF7133_DEFAULT_DELAY)
                {
                    sample_delay = AF7133_DEFAULT_DELAY;
                }*/

                if(value > 100) sample_delay = 100;
                else sample_delay = value;

                af7133mid_data.controldata[0] = sample_delay;  // Loop Delay
            }
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                MSE_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
//              read_lock(&af7133mid_data.ctrllock);
                if(value == 1)
                {
                    af7133mid_data.controldata[7] |= SENSOR_ORIENTATION;
                    atomic_set(&o_flag, 1);
                    atomic_set(&open_flag, 1);
                }
                else
                {
                    af7133mid_data.controldata[7] &= ~SENSOR_ORIENTATION;
                    atomic_set(&o_flag, 0);
                    if(atomic_read(&m_flag) == 0)
                    {
                        atomic_set(&open_flag, 0);
                    }
                }
                wake_up(&open_wq);
//              read_unlock(&af7133mid_data.ctrllock);
                // Do nothing
            }
            break;

        case SENSOR_GET_DATA:
            if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
            {
                MSE_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                osensor_data = (struct hwm_sensor_data *)buff_out;
//              read_lock(&af7133mid_data.datalock);
                osensor_data->values[0] = af7133mid_data.yaw;
                osensor_data->values[1] = af7133mid_data.pitch;
                osensor_data->values[2] = af7133mid_data.roll;
                osensor_data->status = af7133mid_data.mag_status;
//              read_unlock(&af7133mid_data.datalock);

                osensor_data->value_divide = CONVERT_O_DIV;
            }
            break;

        default:
            MSE_ERR("osensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}

/*----------------------------------------------------------------------------*/
//#ifndef   CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int af7133_suspend(struct i2c_client *client, pm_message_t msg)
{
//  int err;
    struct af7133_i2c_data *obj = i2c_get_clientdata(client);
    MSE_FUN();

    if(msg.event == PM_EVENT_SUSPEND)
    {
        af7133_power(obj->hw, 0);
    }
    return 0;
}

/*----------------------------------------------------------------------------*/
static int af7133_resume(struct i2c_client *client)
{
    struct af7133_i2c_data *obj = i2c_get_clientdata(client);
    MSE_FUN();

    af7133_power(obj->hw, 1);

    return 0;
}

static int af7133_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, AF7133_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int af7133_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct i2c_client *new_client;
    struct af7133_i2c_data *data;
    int err = 0;
//  struct hwmsen_object sobj_m, sobj_o;
    struct mag_control_path ctl={0};
    struct mag_data_path mag_data={0};

    printk("%s start\n",__func__);
    data = kzalloc(sizeof(*data), GFP_KERNEL);
    if (!data)
        {
        err = -ENOMEM;
        goto exit;
    }
    memset(data, 0, sizeof(struct af7133_i2c_data));

    data->hw = hw;

    if((err = hwmsen_get_convert(data->hw->direction, &data->cvt)))
    {
        MSE_ERR("invalid direction: %d\n", data->hw->direction);
        goto exit;
    }

    atomic_set(&data->layout, data->hw->direction);
    atomic_set(&data->trace, 0);
    init_waitqueue_head(&data_ready_wq);
    init_waitqueue_head(&open_wq);

    data->client = client;
    new_client = data->client;
    i2c_set_clientdata(new_client, data);

    af7133_i2c_client = new_client;

    if((err = AF7133_Chipset_Init(AF7133_MODE_IDLE))!=0)
    {
        MSE_ERR("af7133E initial fail: AF7133_Chipset_Init\n");
        goto exit_init_failed;
    }

    /* Register sysfs attribute */
    err = af7133_create_attr(&(af7133_init_info.platform_diver_addr->driver));
    if(err)
    {
        MSE_ERR("create attribute err = %d\n", err);
        goto exit_sysfs_create_group_failed;
    }

    err = misc_register(&af7133_device);
    if(err)
    {
        MSE_ERR("af7133_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    ctl.m_enable = af7133_m_enable;
    ctl.m_set_delay  = af7133_m_set_delay;
    ctl.m_open_report_data = af7133_m_open_report_data;
    ctl.o_enable = af7133_o_enable;
    ctl.o_set_delay  = af7133_o_set_delay;
    ctl.o_open_report_data = af7133_o_open_report_data;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = false;
    ctl.is_use_common_factory= false;

    err = mag_register_control_path(&ctl);
    if(err)
    {
        printk("register mag control path err\n");
        goto exit_kfree;
    }

    mag_data.div_m = 1000; // CONVERT_M_DIV;
    mag_data.div_o = 200; // CONVERT_O_DIV;
    mag_data.get_data_o = af7133_o_get_data;
    mag_data.get_data_m = af7133_m_get_data;

    err = mag_register_data_path(&mag_data);
    if(err)
    {
        printk("register data control path err\n");
        goto exit_kfree;
    }

    MSE_LOG("%s: OK\n", __func__);
    af7133_init_flag=0;
    return 0;

exit_sysfs_create_group_failed:
exit_init_failed:
    //i2c_detach_client(new_client);
exit_misc_device_register_failed:
exit_kfree:
    kfree(data);
exit:
    MSE_ERR("%s: err = %d\n", __func__, err);
    af7133_init_flag=-1;
    return err;
}

/*----------------------------------------------------------------------------*/
static int af7133_i2c_remove(struct i2c_client *client)
{
    int err;

    err = af7133_delete_attr(&(af7133_init_info.platform_diver_addr->driver));

    if(err)
    {
        MSE_ERR("af7133_delete_attr fail: %d\n", err);
    }

    af7133_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    misc_deregister(&af7133_device);
    return 0;
}
/*----------------------------------------------------------------------------*/
//#ifdef MTK_AUTO_DETECT_MAGNETOMETER
/*----------------------------------------------------------------------------*/
static int af7133_local_init(void)
{
    printk("af7133_local_init");
    af7133_power(hw, 1);
    //af7133_force[0] = hw->i2c_num;

    if(i2c_add_driver(&af7133_i2c_driver))
    {
        printk("add driver error\n");
        return -1;
    }
    if(-1 == af7133_init_flag)
    {
        return -1;
    }
    return 0;
}

static int vtc_remove(void)
{
    MSE_FUN();
    af7133_power(hw, 0);
    atomic_set(&dev_open_count, 0);
    i2c_del_driver(&af7133_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/

static int __init af7133_init(void)
{
    const char *name = "mediatek,af7133E";
    printk("af7133_init\n");

    hw = get_mag_dts_func(name, hw);
    if (!hw)
        printk("get dts info fail\n");

    printk("af7133 mag_driver_add\n");
    mag_driver_add(&af7133_init_info);

    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit af7133_exit(void)
{
}
/*----------------------------------------------------------------------------*/

module_init(af7133_init);
module_exit(af7133_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Gary Huang");
MODULE_DESCRIPTION("AF7133E m-Sensor driver");
MODULE_LICENSE("VTC");
MODULE_VERSION(DRIVER_VERSION);


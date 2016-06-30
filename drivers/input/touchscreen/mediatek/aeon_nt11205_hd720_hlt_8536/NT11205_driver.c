/* drivers/input/touchscreen/NVTtouch_205.c
 *
 * Copyright (C) 2010 - 2015 Novatek, Inc.
 *
 * Revision : V2.1 for MTK (2015/6/9)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */


#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>



#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <linux/dma-mapping.h>

#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/completion.h>
#include <asm/uaccess.h>

#include <linux/miscdevice.h>
#include <linux/device.h>

#include "NVTtouch_205.h"
//#include "NVTtouch_11205_mptool.h"

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif
#ifdef MT6577
#include <mach/mt6577_pm_ldo.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#endif
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_boot.h>

#include "tpd.h"
//#include <cust_eint.h>

#ifndef TPD_NO_GPIO
//#include "cust_gpio_usage.h"
#endif

#if TP_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

//#include <mach/mt_pm_ldo.h> 

#if TOUCH_I2C_USE_DMA
#include <linux/dma-mapping.h>

#define MAX_RX_DMA_BUF 250
#define MAX_TX_DMA_BUF 250
#endif

#if BOOT_UPDATE_FIRMWARE
#include "NVT_firmware_205.h"
static struct workqueue_struct *nvt_fwu_wq;
#endif


#if CHARGER_DETECT
static int b_usb_plugin = 0;
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

//extern kal_bool upmu_chr_det(upmu_chr_list_enum chr);
extern struct tpd_device *tpd;

static int tpd_flag = 0;
static int tpd_halt = 0;

unsigned int nt_gesture = 0;
static struct task_struct *thread = NULL;

#if BOOT_UPDATE_FIRMWARE
static int tp_update_flag = 1;
static uint8_t tp_version[1] = {0};
#endif

#ifndef GPIO_CTP_EINT_PIN
#define GPIO_CTP_EINT_PIN (GPIO10 | 0x80000000)
#endif

#ifndef GPIO_CTP_RST_PIN
#define GPIO_CTP_RST_PIN  (GPIO62 | 0x80000000)
#endif

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int __init tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __init tpd_i2c_remove(struct i2c_client *client);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

#ifdef MT6575
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif
#ifdef MT6577
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif


//tpd i2c
//static struct i2c_client *i2c_client = NULL;
struct nvt_ts_data *ts;
static const struct i2c_device_id nt11205_tpd_id[] = {{NVT_I2C_NAME, I2C_BUS_NUMBER},{}};
//static unsigned short force[] = {0, I2C_FW_Address, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short * const forces[] = {force, NULL};
//static struct i2c_client_address_data addr_data = {.forces = forces, };

/*** max-- add ****/
#define GTP_RST_PORT    0
#define GTP_INT_PORT    1

static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc);

unsigned int nt_touch_irq = 0;

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	//GTP_INFO("Device Tree Tpd_irq_registration!");

	node = of_find_matching_node(node, touch_of_match);

	printk("[max--%s@%d]: ss:%s %s %s \n",__func__,__LINE__,node->name,node->full_name,node->type);
	if (node) {
		printk("[max--%s@%d]:   \n",__func__,__LINE__);
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		//gpio_set_debounce(ints[0], ints[1]);

		nt_touch_irq = irq_of_parse_and_map(node, 0);
		//GTP_INFO("Device gt1x_int_type = %d!", gt1x_int_type);
		
		ret = request_irq(nt_touch_irq, (irq_handler_t) tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING,
					"TOUCH_PANEL-eint", NULL);

		printk("[max--%s@%d]: irq:%d  \n",__func__,__LINE__,nt_touch_irq);

		if (ret > 0) {
			ret = -1;
			//GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
		}
		
	} else {
	//	GTP_ERROR("tpd request_irq can not find touch eint device node!.");
		ret = -1;
		printk("[max--%s@%d]:   \n",__func__,__LINE__);
	}
	//GTP_INFO("[%s]irq:%d, debounce:%d-%d:", __func__, nt_touch_irq, ints[0], ints[1]);
	return ret;
}


/***** end ******/



//static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(NVT_I2C_NAME, I2C_FW_Address)};


static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,nt11205_touch"},
	{},
};
static struct i2c_driver tpd_i2c_driver = {
	.driver = 
	{
		.name = NVT_I2C_NAME,
		.of_match_table = tpd_of_match,
		//.owner = THIS_MODULE,
	},
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.id_table = nt11205_tpd_id,
	.detect = tpd_i2c_detect,
	//.address_data = &addr_data,
};


#if TPD_KEY_NUM > 0
void tpd_nt11205_key_init(void)
{
    int i = 0;

    for(i=0;i<TPD_KEY_NUM;i++)
    {
        __set_bit(touch_key_array[i], tpd->dev->keybit);
    }
}
#endif

/* Success: 0(TPD_OK),  Failed: -1(TPD_FAIL)*/
int CTP_I2C_READ_FIFO(struct i2c_client *client,
                unsigned short i2c_addr, unsigned char *buf, unsigned short length)
{
        int retval= -1;
        uint8_t retry = 0;
        int tmp_reg;
        int left_len = length -1;
        struct i2c_msg msgs[2] = {{0}};
        int offset = 0;

        if(buf == NULL) goto exit;

        tmp_reg = buf[0];

        /* for write offset */
        msgs[0].flags = !I2C_M_RD;
        msgs[0].addr  = i2c_addr;
        msgs[0].len   = 1;
        msgs[0].buf   = &buf[0];

        /* for data reading */
        msgs[1].flags = I2C_M_RD;
        msgs[1].addr = i2c_addr;

        mutex_lock(&(ts->i2c_mutex)); /* lock */

        while (left_len > 0) {
                buf[0] = tmp_reg;
                msgs[1].buf = &buf[offset+1];

                for (retry = 0; retry < 5; retry++) {
                        if (left_len > MAX_TR_BYTES) {
                                msgs[1].len = MAX_TR_BYTES;
                                retval = i2c_transfer(client->adapter, msgs, 2);
                        } else {
                                msgs[1].len = left_len;
                                retval = i2c_transfer(client->adapter, msgs, 2);
                        }

                        if (retval ==  2) {
                                //dev_info(&client->dev, "%s: read %d bytes to offset:0x%x from reg:0x%x\n",
                                //                      __func__, msgs[1].len, offset, tmp_reg);
                                break;
                        } else {
                                dev_err(&client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
                                msleep(20);
                        }
                }
                left_len -= MAX_TR_BYTES;
                tmp_reg += MAX_TR_BYTES;
                offset += MAX_TR_BYTES;
        }


        mutex_unlock(&(ts->i2c_mutex)); /* unlock */
exit:
        return ((retval==2)?TPD_OK:TPD_FAIL);
}

/* Success: 0(TPD_OK),  Fail: -1(TPD_FAIL) */
int CTP_I2C_WRITE_FIFO(struct i2c_client *client,
                unsigned short i2c_addr, unsigned char *buf, unsigned short length)
{
        int retval= -1;
        uint8_t retry = 0;
        int left_len = length;
        struct i2c_msg msg = {0};
        int offset = 0;

        if(buf == NULL) goto exit;

        /* for write  */
        msg.flags = !I2C_M_RD;
        msg.addr  = i2c_addr;
        msg.buf   = &buf[0];


        mutex_lock(&(ts->i2c_mutex)); /* lock */

        while (left_len > 0) {
                msg.buf = &buf[offset];

                for (retry = 0; retry < 5; retry++) {
                        if (left_len > MAX_TR_BYTES) {
                                msg.len = MAX_TR_BYTES;
                                retval = i2c_transfer(client->adapter, &msg, 1);
                        } else {
                                msg.len = left_len;
                                retval = i2c_transfer(client->adapter, &msg, 1);
                        }

                        if (retval ==  1) {
                                //dev_info(&client->dev, "%s: write %d bytes from data offset:0x%x\n",
                                //                        __func__, msg.len,  offset);
                                break;
                        } else {
                                dev_err(&client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
                                msleep(20);
                        }
                }

                left_len -= MAX_TR_BYTES;
                offset += MAX_TR_BYTES;
        }

        mutex_unlock(&(ts->i2c_mutex)); /* unlock */
exit:
        return ((retval==1)?TPD_OK:TPD_FAIL);
}

#if TOUCH_I2C_USE_DMA
/* CTP_I2C_READ_DMA:
 * Success: 0(TPD_OK),  Failed: -1(TPD_FAIL)
 */

static u8 *rDMABuf_va = NULL;
static dma_addr_t rDMABuf_pa = 0;
static u8 *wDMABuf_va = NULL;
static dma_addr_t wDMABuf_pa = 0;

int CTP_I2C_READ_DMA(struct i2c_client *client, uint8_t i2c_addr, uint8_t *buf, uint8_t len)
{
        struct i2c_msg msgs[2] = {{0}};
        int ret = -1;
        int retries = 0;

	if((len-1) > MAX_RX_DMA_BUF) goto err;
		
	mutex_lock(&(ts->i2c_mutex)); /* lock */

        msgs[0].flags = !I2C_M_RD;
        msgs[0].addr  = i2c_addr;
        msgs[0].len   = 1;
        msgs[0].buf   = &buf[0];
		msgs[0].timing = I2C_MASTER_CLOCK;

        msgs[1].flags = I2C_M_RD;
        msgs[1].addr  = i2c_addr;
        msgs[1].len   = len-1;
        msgs[1].buf   = (u8*)rDMABuf_pa;  //&buf[1];
	msgs[1].ext_flag = (I2C_ENEXT_FLAG | I2C_DMA_FLAG); /* DMA mode */
	msgs[1].timing = I2C_MASTER_CLOCK;

        while(retries < 5)
        {
                ret = i2c_transfer(client->adapter, msgs, 2);
                if(ret == 2)    break;
                retries++;
        }

	if(ret == 2){
		int i;
		for(i=0; i<(len-1); i++)
			buf[i+1] = rDMABuf_va[i];
	}

	mutex_unlock(&(ts->i2c_mutex)); /* unlock */

err:
        return ((ret==2)?TPD_OK:TPD_FAIL);
}
 
/* CTP_I2C_WRITE_DMA: 
 * Success: 0(TPD_OK),  Failed: -1(TPD_FAIL)
 */
int CTP_I2C_WRITE_DMA (struct i2c_client *client, uint8_t i2c_addr, uint8_t *data, uint8_t len)
{
        struct i2c_msg msg = {0};
        int ret = -1;
        int retries = 0;
	int i;

	if((len) > MAX_TX_DMA_BUF) goto err;

	mutex_lock(&(ts->i2c_mutex)); /* lock */
	
	for(i=0; i<len; i++)
		wDMABuf_va[i] = data[i];

        msg.flags = !I2C_M_RD;
        msg.addr  = i2c_addr;
        msg.len   = len;
        msg.buf   = (u8*)wDMABuf_pa; //data; We need physical addr for DMA. 
	msg.ext_flag = (I2C_ENEXT_FLAG | I2C_DMA_FLAG); /* DMA mode */	
	msg.timing = I2C_MASTER_CLOCK;
	
        while(retries < 5)
        {
                ret = i2c_transfer(client->adapter, &msg, 1);
                if(ret == 1)    break;
                retries++;
        }
	mutex_unlock(&(ts->i2c_mutex)); /* unlock */
err:
        return ((ret==1)?TPD_OK:TPD_FAIL);
}
#endif


int CTP_I2C_READ(struct i2c_client *client, uint8_t i2c_addr, uint8_t *buf, uint8_t len)
{
#if TOUCH_I2C_USE_DMA
//	if((len-1) > 8)
		return CTP_I2C_READ_DMA(client, i2c_addr, buf, len);
//	else
//		return CTP_I2C_READ_FIFO(client, i2c_addr, buf, len);
#else
	return CTP_I2C_READ_FIFO(client, i2c_addr, buf, len);
#endif
}

int CTP_I2C_WRITE(struct i2c_client *client, uint8_t i2c_addr, uint8_t *data, uint8_t len)
{
#if TOUCH_I2C_USE_DMA
	//if(len > 8)
		return CTP_I2C_WRITE_DMA(client, i2c_addr, data, len);
//	else
//		return CTP_I2C_WRITE_FIFO(client, i2c_addr, data, len);
#else
	return CTP_I2C_WRITE_FIFO(client, i2c_addr, data, len);
#endif
}



static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, NVT_I2C_NAME);
	return TPD_OK;
}

static void nvt_hw_reset(void)
{
	//---trigger rst-pin to reset (pull low for 10ms)---	
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(20);
	tpd_gpio_output(GTP_RST_PORT, 0);
	msleep(10);
	tpd_gpio_output(GTP_RST_PORT, 1);
}

/*******************************************************
*******************************************************/
#if TOUCH_WATCHDOG
#define WDG_BARK_TIME msecs_to_jiffies(5000)
static struct timer_list wdg_timer;
static int wdg_init = 0;

static struct workqueue_struct *wdg_wq = NULL;

struct work_struct wdg_work;

static void check_chipid_func(struct work_struct *work)
{
	int retry;
	unsigned char buf[8] = {0};
	int chipID = 0;

	if(ts->fw_updating)
		return;

        for(retry=5; retry>=0; retry--)
        {
                buf[0]=0x86;
                CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

                printk("\n%s: chipid=%d\n", __func__,  buf[1]);

                chipID = buf[1];

                if(chipID == 5)
                {
                        break;
                }
        }

        if(chipID != 5){
                dev_err(&ts->client->dev, "%s: chipID:%d wrong, reset IC.\n", __func__, chipID);
                nvt_hw_reset();
        }
}

void kick_watchdog(unsigned long data)
{
	if(ts->fw_updating)
		goto RESET_TIMER;

	queue_work(wdg_wq, &wdg_work);

RESET_TIMER:
        mod_timer(&wdg_timer, (jiffies+WDG_BARK_TIME));
}

void watchdog_init(void)
{
	if(wdg_init) return;

	init_timer(&wdg_timer);

        wdg_timer.function = kick_watchdog;
        wdg_timer.data = 0;

        //---create workqueue---
        wdg_wq = create_workqueue("wdg_wq");
        if(!wdg_wq)
        {
                dev_err(&ts->client->dev, "%s : wdg_wq create workqueue failed.\n", __func__);
                return;
        }
        INIT_WORK(&wdg_work, check_chipid_func);	


        wdg_init = 1;
}

void watchdog_enable(void)
{
        static int wdg_timer_pending = 0;

        if(!wdg_init) return;

        wdg_timer.expires = jiffies + WDG_BARK_TIME;

        if (wdg_timer_pending)
                mod_timer(&wdg_timer, (jiffies + WDG_BARK_TIME));
        else{
                wdg_timer_pending = 1;
                add_timer(&wdg_timer);
        }
}

void watchdog_disable(void)
{
        if(!wdg_init) return;
        del_timer_sync(&wdg_timer);
	flush_work(&wdg_work);
}
#endif

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_CTRL_DRIVER
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

ssize_t nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
	unsigned char str[64];
	int ret = -1;
	int retries = 0;
	unsigned char i2c_addr;
	int len;
	//TPD_DMESG("tpd nvt_flash_write\n");

	if(count > sizeof(str))
		return -EFAULT;

	if(copy_from_user(str, buff, count))
		return -EFAULT;

	//TPD_DMESG("tpd str[0]=%x, str[1]=%x, str[2]=%x, str[3]=%x\n", str[0], str[1], str[2], str[3]);
	
	i2c_addr = str[0];
	len = str[1];
#if 1
	//---change sw_reset to hw_reset---
	if(str[0]==0x70)
	{
		if(str[2]==0x00 && str[3]==0x5A)
		{
			nvt_hw_reset();
			return TPD_OK;
		}
	}
#endif	
	while(retries < 20)
	{
		ret = CTP_I2C_WRITE(ts->client, i2c_addr, &str[2], len);		

		if(ret == TPD_OK)
			break;
		else
			printk("%s error, retries=%d\n", __func__, retries);

		retries++;
	}	
	
	return ret;
}

ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	unsigned char str[64];
	int ret = -1;
	int retries = 0;
	unsigned char i2c_addr;
	int len;	
	//TPD_DMESG("tpd nvt_flash_read\n");

	if(count > sizeof(str))
		return -EFAULT;

	if(copy_from_user(str, buff, count))
		return -EFAULT;

	//TPD_DMESG("tpd str[0]=%x, str[1]=%x, str[2]=%x, str[3]=%x\n", str[0], str[1], str[2], str[3]);
		
	i2c_addr = str[0];
	len = str[1];

	while(retries < 20)
	{
		ret = CTP_I2C_READ(ts->client, i2c_addr, &str[2], len);
		if(ret == TPD_OK)
			break;
		else
			printk("%s error, retries=%d\n", __func__, retries);

		retries++;
	}

	// copy buff to user if i2c transfer 	
	if(retries < 20)
	{
		if(copy_to_user(buff, str, count))
			return -EFAULT;
	}
	
	return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if(dev == NULL)
		return -ENOMEM;

#if TOUCH_WATCHDOG
	watchdog_disable();
#endif

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if(dev)
		kfree(dev);
	
#if TOUCH_WATCHDOG
	watchdog_enable();
#endif
	return 0;   
}

struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.write = nvt_flash_write,
	.read = nvt_flash_read,
};

static int nvt_flash_proc_init(void)
{
	int ret=0;
	
  	NVT_proc_entry = proc_create(DEVICE_NAME, 0666, NULL,&nvt_flash_fops);
	if(NVT_proc_entry == NULL)
	{
		printk("%s: couldn't create proc entry!\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	else
	{
		printk("%s: create proc entry success!\n", __func__);
		//NVT_proc_entry->proc_fops = &nvt_flash_fops;
	}
	printk("============================================================\n");
	printk("Create /proc/NVTflash\n");
	printk("============================================================\n");
	return 0;
}
#endif


#if TP_PROXIMITY
#define TPD_PROXIMITY_ENABLE_REG	0xA4
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;	//0-->close ; 1--> far away

static s32 tpd_proximity_get_value(void)
{
    return tpd_proximity_detect;
}

static s32 tpd_proximity_enable(s32 enable)
{
    u8 state;
    s32 ret = -1;
    u8 buf[2] = {0};
	TPD_DMESG("tpd_proximity_enable enable=%d\n",enable);
	
    if (enable)
    {
        state = 1;
        tpd_proximity_flag = 1;
        TPD_DMESG("tpd_proximity_enable on.\n");
    }
    else
    {
        state = 0;
        tpd_proximity_flag = 0;
        TPD_DMESG("tpd_proximity_enable off.\n");
    }

    //ret = i2c_write_bytes(i2c_client, TPD_PROXIMITY_ENABLE_REG, &state, 2);
    buf[0] = TPD_PROXIMITY_ENABLE_REG;
    buf[1] = state;
    ret  = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
    if (ret != TPD_OK)
    {
        TPD_DMESG("tpd %s proximity cmd failed.\n", state ? "enable" : "disable");
        return ret;
    }

    TPD_DMESG("tpd proximity function %s success.\n", state ? "enable" : "disable");
    return 0;
}

s32 tpd_proximity_operate(void *self, u32 command, void *buff_in, s32 size_in,
                   void *buff_out, s32 size_out, s32 *actualout)
{
    s32 err = 0;
    s32 value;
    hwm_sensor_data *sensor_data;

    //printk("tpd_proximity_operate enter... ");

    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                TPD_DMESG("tpd Set delay parameter error!");
                err = -EINVAL;
            }

            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                TPD_DMESG("tpd Enable sensor parameter error!");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                err = tpd_proximity_enable(value);
            }

            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data)))
            {
                TPD_DMESG("tpd Get sensor data parameter error!");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = tpd_proximity_get_value();
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
            break;

        default:
            TPD_DMESG("tpd proximy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}

static int tpd_proximity_event(u8 proximity_status)
{
	int ret = 0;
    	hwm_sensor_data sensor_data;

	TPD_DMESG("tpd_proximity_flag = %d, proximity_status = %d\n", tpd_proximity_flag, proximity_status);

	if (tpd_proximity_flag == 1)
	{
		if (proximity_status == 0x03)	//Proximity is far	
		{
			tpd_proximity_detect = 1;
		}
		else if (proximity_status == 0x01)	//Proximity is near
		{
			tpd_proximity_detect = 0;
		}

		TPD_DMESG("tpd PROXIMITY STATUS:0x%02X\n", tpd_proximity_detect);
		
		//map and store data to hwm_sensor_data
		sensor_data.values[0] = tpd_proximity_get_value();
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		
		//report to the up-layerhwmsen	
		ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);
		if (ret)
		{
			TPD_DMESG("tpd Call hwmsen_get_interrupt_data fail = %d\n", ret);
		}
	}

	return ret;
}

int tpd_proximity_init(void)
{
    int err = 0;
	struct hwmsen_object obj_ps;

    //obj_ps.self = cm3623_obj;
    obj_ps.polling = 0;         //0--interrupt mode; 1--polling mode;
    obj_ps.sensor_operate = tpd_proximity_operate;

    if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        TPD_DMESG("tpd hwmsen attach fail, return:%d.", err);
    }

   return err;
}
#endif


#if WAKEUP_GESTURE
#define GESTURE_WORD_C			12
#define GESTURE_WORD_W			13
#define GESTURE_WORD_V			14
#define GESTURE_DOUBLE_CLICK	15
#define GESTURE_WORD_Z			16
#define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18
#define GESTURE_WORD_e			19
#define GESTURE_WORD_S			20
#define GESTURE_SLIDE_UP		21
#define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24

static unsigned char bTouchIsAwake=1;

void nvt_ts_wakeup_gesture_report(unsigned char gesture_id)
{
	unsigned int keycode=0;

	TPD_DMESG("gesture_id = %d\n", gesture_id);

	switch(gesture_id)
	{
		case GESTURE_WORD_C:
			TPD_DMESG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			TPD_DMESG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			TPD_DMESG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			TPD_DMESG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			TPD_DMESG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			TPD_DMESG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			TPD_DMESG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			TPD_DMESG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			TPD_DMESG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			TPD_DMESG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			TPD_DMESG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			TPD_DMESG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			TPD_DMESG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if(keycode > 0)
	{
		input_report_key(tpd->dev, keycode, 1);
		input_sync(tpd->dev);
		input_report_key(tpd->dev, keycode, 0);
		input_sync(tpd->dev);
	}

	msleep(250);
}
#endif


#if CHARGER_DETECT
#define TPD_CHARGER_STATE_REG	0xE8
void tpd_usb_plugin(int plugin)
{
	int ret = -1;
	u8 buf[2] = {0};


	TPD_DMESG("tpd_usb_plugin usb detect: b_usb_plugin=%d, tpd_halt=%d.\n", b_usb_plugin, tpd_halt);
	if(tpd_halt)
        return;
	
	switch(plugin) {
        case 0:
            TPD_DMESG("tpd No charger.\n");
            buf[0] = TPD_CHARGER_STATE_REG;
            buf[1] = 0xF0;
            //ret = i2c_write_bytes(i2c_client, TPD_CHARGER_STATE_REG, &state, 2);
            ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
            if (ret != TPD_OK)
            {
                TPD_DMESG("tpd_usb_plugin 0x%02X cmd failed.\n", buf[1]);
            }
            break;
        case 1:
            TPD_DMESG("tpd VBUS charger.\n");
            buf[0] = TPD_CHARGER_STATE_REG;
            buf[1] = 0xF1;
            //ret = i2c_write_bytes(i2c_client, TPD_CHARGER_STATE_REG, &state, 2);
            ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
            if (ret != TPD_OK)
            {
                TPD_DMESG("tpd_usb_plugin 0x%02X cmd failed.\n", buf[1]);
            }
            break;
        case 2:
            TPD_DMESG("tpd AC charger.\n");
            buf[0] = TPD_CHARGER_STATE_REG;
            buf[1] = 0xF2;
            //ret = i2c_write_bytes(i2c_client, TPD_CHARGER_STATE_REG, &state, 2);
            ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2); 
            if (ret != TPD_OK)
            {
                TPD_DMESG("tpd_usb_plugin 0x%02X cmd failed.\n", buf[1]);
            }
            break;
        default:
            TPD_DMESG("tpd Others.\n");
            buf[0] = TPD_CHARGER_STATE_REG;
            buf[1] = 0xF0;
            //ret = i2c_write_bytes(i2c_client, TPD_CHARGER_STATE_REG, &state, 2);
            ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
            if (ret != TPD_OK)
            {
                TPD_DMESG("tpd_usb_plugin 0x%02X cmd failed.\n", buf[1]);
            }
            break;
    }
}
EXPORT_SYMBOL(tpd_usb_plugin);
#endif

#if 0
static int nvt_i2c_read_bytes(unsigned char addr, unsigned char reg, unsigned char *data, int len)
{
	unsigned char tmpaddr;
	int ret;

	mutex_lock(&ts->i2c_mutex); /* lock */
	
	tmpaddr = i2c_client->addr;
	i2c_client->addr = addr;
	ret = i2c_smbus_read_i2c_block_data(i2c_client, reg, len, data);
	i2c_client->addr = tmpaddr;

	if(ret < 0)
	{
		//TPD_DMESG("nvt_i2c_read_bytes failed.\n");
		mutex_unlock(&ts->i2c_mutex); /* unlock */
	    	return 0;
	}
	else
	{
		mutex_unlock(&ts->i2c_mutex);
	    	return 1;
	}
}

static int nvt_i2c_write_bytes(unsigned char addr, unsigned char reg, unsigned char *data, int len)
{
	unsigned char tmpaddr;
	int ret;
	
	mutex_lock(&ts->i2c_mutex); /* lock */

	tmpaddr = i2c_client->addr;
	i2c_client->addr = addr;
	ret = i2c_smbus_write_i2c_block_data(i2c_client, reg, len, data);
	i2c_client->addr = tmpaddr;

	if(ret < 0)
	{
		//TPD_DMESG("nvt_i2c_write_bytes failed.\n");
		mutex_unlock(&ts->i2c_mutex);
		return 0;
	}
	else
	{
		mutex_unlock(&ts->i2c_mutex);
	    	return 1;
	}
}
#endif

/*******************************************************
  Auto Update FW in Probe
*******************************************************/
#if BOOT_UPDATE_FIRMWARE
int Get_TP_version(void)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0x78;
	CTP_I2C_READ(ts->client, I2C_FW_Address, I2C_Buf, 2);
	TPD_DMESG("IC FW Ver = %d\n", I2C_Buf[1]);
	TPD_DMESG("Bin FW Ver = %d\n", BUFFER_DATA[0x7F00]);
	if(I2C_Buf[1] == BUFFER_DATA[0x7F00])
	{
		tp_version[0] = I2C_Buf[1];
		return 1;
	}
	else
		return 0;
}
int Check_FW_Ver(void)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0x78;
	CTP_I2C_READ(ts->client, I2C_FW_Address, I2C_Buf, 2);
	
	TPD_DMESG("IC FW Ver = %d\n", I2C_Buf[1]);
	TPD_DMESG("Bin FW Ver = %d\n", BUFFER_DATA[0x7F00]);
	tp_version[0] = I2C_Buf[1];

	if(I2C_Buf[1]>BUFFER_DATA[0x7F00])
		return 1;
	else
		return 0;
}

int Check_CheckSum(void)
{
	uint8_t I2C_Buf[64];
	uint8_t buf2[64];
	int i, j, k, Retry_Counter=0;
	int addr=0;
	uint8_t addrH, addrL;
	unsigned short RD_Filechksum, WR_Filechksum;

	WR_Filechksum = 0;


	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x5A;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, I2C_Buf, 2);

	msleep(1000);


	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x3F;
	I2C_Buf[2]=0xE8;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, I2C_Buf, 3);

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0xEA;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, I2C_Buf, 2);

	addr = 0;
	for(i=0;i<(BUFFER_LENGTH)/128;i++)
	{
		for(j=0;j<16;j++)
		{
			unsigned char tmp=0;
			addrH = addr>>8;
			addrL = addr&0xFF;
			for(k=0;k<8;k++)
			{
				tmp+=BUFFER_DATA[i*128+j*8+k];
			}
			tmp = tmp+addrH+addrL+8;
			tmp = (255-tmp)+1;
			WR_Filechksum+=tmp;
			addr+=8;
		}
	}

	msleep(800);

	do
	{
		msleep(10);
		I2C_Buf[0]=0xFF;
		I2C_Buf[1]=0x3F;
		I2C_Buf[2]=0xF8;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, I2C_Buf, 3);

		buf2[0]=0x00;
		buf2[1]=0x00;
		buf2[2]=0x00;
		buf2[3]=0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf2, 4);

		Retry_Counter++;
		msleep(10);

	}while((Retry_Counter<20)&& (buf2[1]!=0xAA));

	//---------------------------------------------------------------------------------------

	if(buf2[1]==0xAA)
	{
		RD_Filechksum=(buf2[2]<<8)+buf2[3];
		if(RD_Filechksum==WR_Filechksum)
		{
			TPD_DMESG("%s : firmware checksum match.\n", __func__);
			return 1;	// checksum match
		}
		else
		{
			TPD_DMESG("%s : firmware checksum not match!!\n", __func__);
			return 0;	// checksum not match
		}
	}
	else
	{
		TPD_DMESG("%s : read firmware checksum timeout!!\n", __func__);
		return -1;	// read checksum failed
	}
}

void Update_Firmware(void)
{
	uint8_t I2C_Buf[16] = {0};
	int i = 0;
	int j = 0;
	unsigned int Flash_Address = 0;
	unsigned int Row_Address = 0;
	//uint8_t CheckSum = 0;	// 128/8 = 16 times ;
	//struct i2c_client *client = ts->client;
	int ret;

	//-------------------------------
	// Step1 --> initial BootLoader
	//-------------------------------
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);

	msleep(2);
  
	// Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);

	msleep(20);

	// Read status
	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);	
	if (I2C_Buf[1] != 0xAA)
	{
		TPD_DMESG("Program: init get status(0x%2X) error.", I2C_Buf[1]);
		return;
	}
	TPD_DMESG("Program: init get status(0x%2X) success.", I2C_Buf[1]);

	//---------------------------------------------------------
 	// Step 2 : Erase 
 	//---------------------------------------------------------
	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x66;
	I2C_Buf[2]=0x00;
	I2C_Buf[3]=0x0E;
	I2C_Buf[4]=0x01;
	I2C_Buf[5]=0xB4;
	I2C_Buf[6]=0x3D;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 7);

	while(1)
	{
		msleep(1);
		CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
		if(I2C_Buf[1]==0xAA)
			break;
	}

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x66;
	I2C_Buf[2]=0x00;
	I2C_Buf[3]=0x0F;
	I2C_Buf[4]=0x01;
	I2C_Buf[5]=0xEF;
	I2C_Buf[6]=0x01;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 7);

	while(1)
	{
		msleep(1);
		CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
		if(I2C_Buf[1]==0xAA)
			break;
	}
	

	for (i = 0 ; i < BUFFER_LENGTH/4096 ; i++)	// 32K = 8 times
	{
		Row_Address = i * 4096; 															

		// Erase Flash	
		I2C_Buf [0] = 0x00;
		I2C_Buf [1] = 0x33;
		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte  
		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte 
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 4);
		msleep(15);	// Delay 15 ms 
			  
		// Read Erase status
		CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
			  
		// check status        
		if (I2C_Buf[1] != 0xAA)
		{
			TPD_DMESG("Program: erase(0x%02X) error.", I2C_Buf[1]);
			return;
		}			
	}

	TPD_DMESG("Program: erase(0x%02X) success.", I2C_Buf[1]);

	Flash_Address = 0;
        		
	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step3. Host write 128 bytes to IC  
	//----------------------------------------
	TPD_DMESG("Program: write begin, please wait...");

    	for(j=0;j<BUFFER_LENGTH/128;j++)
	{
    		Flash_Address=(j)*128;
#if TOUCH_I2C_USE_DMA
                for (i = 0 ; i < 16 ; i++, Flash_Address += 8)        // 128/8 = 16 times for One Row program
                {
                        uint8_t CheckSum = 0;
                // write bin data to IC
                        I2C_Buf[0] = 0x00;
                        I2C_Buf[1] = 0x55;      //Flash write command
                        I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);    //Flash address [15:8]
                        I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);   //Flash address [7:0]
                        I2C_Buf[4] = 0x08;      //Flash write length (byte)
                        I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];    //Binary data 1
                        I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];    //Binary data 2
                        I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];  //Binary data 3
                        I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];  //Binary data 4
                        I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
                        I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5]; //Binary data 6
                        I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6]; //Binary data 7
                        I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7]; //Binary data 8

                        // Calculate a check sum by Host controller.
                        CheckSum = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
                          	I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
                              	I2C_Buf[13]) + 1;

                        // Load check sum to I2C Buffer
                        I2C_Buf[5] = CheckSum;
                        CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 14);
                }

#else
	   	for (i = 0 ; i < 16*4 ; i++, Flash_Address += 2)	// 128/2 = 64 times for One Row program
		{
			uint8_t CheckSum = 0;
    		// write bin data to IC
  			I2C_Buf[0] = 0x00;
			I2C_Buf[1] = 0x55;	//Flash write command
			I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
			I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
			I2C_Buf[4] = 0x02;	//Flash write length (byte)
			I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
			I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
			//I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
			//I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
			//I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
			//I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
			//I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
			//I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8

			// Calculate a check sum by Host controller.
			CheckSum = (~(I2C_Buf[2]+I2C_Buf[3]+I2C_Buf[4]+I2C_Buf[6]+I2C_Buf[7]))+1;
			// Load check sum to I2C Buffer
			I2C_Buf[5] = CheckSum;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 8);
		}
#endif
		msleep(10);
		
		// Read status
		I2C_Buf[0] = 0x00;
		while(1)
		{
		        msleep(1);
			CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);		
			if(I2C_Buf[1]==0xAA)
				break;
		}
    	}

	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step4. Verify  
	//----------------------------------------
	TPD_DMESG("Program: Verify begin, please wait...");
	/* [FIXME] novatek for daewav */
        nvt_hw_reset();

        msleep(500);

	ret=Check_CheckSum();
	if(ret==1)
		TPD_DMESG("Program: Verify Pass!");
	else if(ret==0)
		TPD_DMESG("Program: Verify NG!");
	else if(ret==-1)
		TPD_DMESG("Program: Verify FW not return!");

	//---write i2c command to reset---			
	//I2C_Buf[0] = 0x00;
	//I2C_Buf[1] = 0x5A;
	//CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);
	
	//---trigger rst-pin to reset---
	nvt_hw_reset();

	msleep(500);
	TPD_DMESG("Program: END");
}

void Boot_Update_Firmware(struct work_struct *work)
{
	int ret=0;
	
	ts->fw_updating = 1;
#if TOUCH_WATCHDOG
	watchdog_disable();
#endif
	ret = Check_CheckSum();

	nvt_hw_reset();
	msleep(500);

	if(ret!=1) // checksum not match
	{
		Update_Firmware();
	}
	else if(ret==0&&(Check_FW_Ver()==0))	// (fw checksum not match) && (bin fw version > ic fw version)
	{
		TPD_DMESG("%s : firmware version not match.\n", __func__);
		Update_Firmware();
	}

	Get_TP_version();
	ts->fw_updating = 0;
	tp_update_flag = 1;
#if TOUCH_WATCHDOG
	watchdog_enable();
#endif
}
#endif

/*******   miles add for open/short test *******/
#if 0

//#define FIH_TP_TEST
#ifdef FIH_TP_TEST


static int i2c_read_bytes( struct i2c_client *client, u8 addr, u8 *rxbuf, int len )
{
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    if ( rxbuf == NULL )
        return TPD_FAIL;

    //TPD_DMESG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len );
	
    while ( left > 0 )
    {
        if ( left > MAX_TRANSACTION_LENGTH )
        {
            rxbuf[offset] = ( addr+offset ) & 0xFF;
            ts->client->addr = ts->client->addr | I2C_WR_FLAG | I2C_RS_FLAG;
            retry = 0;
            while ( i2c_master_send(ts->client, &rxbuf[offset], (MAX_TRANSACTION_LENGTH << 8 | 1)) < 0 )
           //while ( i2c_smbus_read_i2c_block_data(i2c_client, offset,8,&rxbuf[offset] )
            {
                retry++;

                if ( retry == 5 )
                {
                    ts->client->addr = ts->client->addr;
                    TPD_DMESG("I2C read 0x%X length=%d failed\n", addr + offset, MAX_TRANSACTION_LENGTH);
                    return -1;
                }
            }
            left -= MAX_TRANSACTION_LENGTH;
            offset += MAX_TRANSACTION_LENGTH;
        }
        else
        {

            //rxbuf[0] = addr;
            rxbuf[offset] = ( addr+offset ) & 0xFF;
            ts->client->addr = ts->client->addr | I2C_WR_FLAG | I2C_RS_FLAG;

            retry = 0;
			//while ( i2c_smbus_read_i2c_block_data(i2c_client, offset,left,&rxbuf[offset] )
            while ( i2c_master_send(ts->client, &rxbuf[offset], (left<< 8 | 1)) < 0 )
            {
                retry++;

                if ( retry == 5 )
                {
                    ts->client->addr = ts->client->addr;
                    TPD_DMESG("I2C read 0x%X length=%d failed\n", addr + offset, left);
                    return TPD_FAIL;
                }
            }
            left = 0;
        }
    }

    ts->client->addr = ts->client->addr;

    return TPD_OK;
}


static int i2c_write_bytes( struct i2c_client *client, u16 addr, u8 *txbuf, int len )
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg =
    {
        .addr = ((client->addr)|(I2C_ENEXT_FLAG )),
        .flags = 0,
        .buf = buffer
    };

    if ( txbuf == NULL )
    {
        return TPD_FAIL;
    }

   	//TPD_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

    while ( left > 0 )
    {
        retry = 0;
        buffer[0] = ( addr+offset ) & 0xFF;

        if ( left > MAX_I2C_TRANSFER_SIZE )
        {
            memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE );
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left );
            msg.len = left + I2C_DEVICE_ADDRESS_LEN;
            left = 0;
        }

        TPD_DEBUG("byte left %d offset %d\n", left, offset );

        while ( i2c_transfer( client->adapter, &msg, 1 ) != 1 )
        {
            retry++;
            if ( retry == 5 )
            {
                TPD_DEBUG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return TPD_FAIL;
            }
            else
        	{
             	TPD_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1]);
        	}
        }
    }

    return TPD_OK;
}

static int nvt_i2c_read_bytes(unsigned char addr, unsigned char reg, unsigned char *data, int len)
{
	unsigned char tmpaddr;
	int ret;
	
	tmpaddr = ts->client->addr;
	ts->client->addr = addr;
	ret = i2c_smbus_read_i2c_block_data(ts->client, reg, len, data);
	ts->client->addr = tmpaddr;

	if(ret < 0)
	{
		printk("jasondz: nvt_i2c_read_bytes failed.\n");
	    return 0;
	}
	else
	{
	    return 1;
	}
}

static int nvt_i2c_write_bytes(unsigned char addr, unsigned char reg, unsigned char *data, int len)
{
	unsigned char tmpaddr;
	int ret;
	
	tmpaddr = ts->client->addr;
	ts->client->addr = addr;
	ret = i2c_smbus_write_i2c_block_data(ts->client, reg, len, data);
	ts->client->addr = tmpaddr;

	if(ret < 0)
	{
		printk("jasondz: nvt_i2c_write_bytes failed.\n");
	    return 0;
	}
	else
	{
	    return 1;
	}
}

static struct kobject *fih_tptest_kobj;

uint8_t buffer[64]={0x00};
int XNum=0;
int YNum=0;
typedef struct{
	unsigned short Tx;
	unsigned short Rx;
	signed short Flag;
}Mapping_t;
Mapping_t MappingTable[45][35];

char FAIL[2048];
char GPIOMSG[2048];

int iAccum;
int fCF;
int fCC;
unsigned char RecordResult[40*40]={0};
int DiffTemp[5000];
unsigned short RawDataTmp[1000];
const int CF_table[8] = {222, 327, 428, 533, 632, 737, 838, 943};
const int CC_table[32] = {	  30,  57,  82, 109, 136, 163, 188, 215,
							 237, 264, 289, 316, 343, 370, 395, 422,
							 856, 883, 908, 935, 962, 989,1014,1041,
							1063,1090,1115,1142,1169,1196,1221,1248};
#define MaxStatisticsBuf 100
int StatisticsNum[MaxStatisticsBuf];
long int StatisticsSum[MaxStatisticsBuf];
int Mutual_GoldenRatio[40][40];
int Mutual_Data[40*40];
int CM_Data[40*40];
int CM_Data_bak[40*40];
static int time  = 0;

int nvt_read_firmware_version(void)
{
	int retry=0;

	for(retry=0; retry<3; retry++)
	{
		buffer[0]=0x78;
		nvt_i2c_read_bytes(I2C_FW_Address, buffer[0], &buffer[1], 5);

		XNum = buffer[3];
		YNum = buffer[4];
		
		if((XNum<=40) && (YNum<=40))
		{
			break;
		}
	}

	if((XNum>40) || (YNum>40))
	{
		TPD_DMESG("%s : read firmware info. error! (XNum=%d, YNum=%d)\n", __func__, XNum, YNum);
		XNum=0;
		YNum=0;
	}

	return buffer[1];
}

int GPIOShort_AllCheck(void)
{
	unsigned short TimeoutCnt0,TimeoutCnt1;
	//------------------------------------
	TimeoutCnt1 = 0;
	
Short_AllCheck:
	TimeoutCnt0 = 0;

	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xE8;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
	msleep(1);
	
	buffer[0]=0x00;
	buffer[1]=0xC5;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 1);	
	msleep(100);

	while(1)
	{
		buffer[0]=0xFF;
		buffer[1]=0x3F;
		buffer[2]=0xE9;
		nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);	
		msleep(1);

		buffer[0]=0x00;
		buffer[1]=0x00;				
		nvt_i2c_read_bytes(I2C_FW_Address, buffer[0], &buffer[1], 8);
		
		if(buffer[1]==0xBB)
			break;
		msleep(1);
		
		TimeoutCnt0++;
		//if(TimeoutCnt0 >MaxTimeoutCnt)
		{
			TimeoutCnt1++;
			if(TimeoutCnt1 > 3)
				return 1;
			else
				goto Short_AllCheck;
		}
	}	

	return 0;
}

char* GPIO2AIN2TxRx(unsigned char GPIO)
{
	if(AIN[GPIO] == -1)
	{
		sprintf(FAIL, "UnusedPin, ");
	}
	else if(AIN[GPIO]>=32)
	{
		sprintf(FAIL, "Rx%d, ", (AIN[GPIO]-32));	
	}
	else
	{
		sprintf(FAIL, "Tx%d, ", (AIN[GPIO]));
	}	
	return FAIL;
}

static int fih_shorttest_exec(void)
{
	//printk("[max--%s@%d]:   \n",__func__,__LINE__);
	unsigned char Port;	
	unsigned char GPIO_ShortList[48];
	unsigned char GPIO_ShortCnt=0;	
	int i,j;
	int ret=0;

	memset(GPIO_ShortList, 0xff, sizeof(GPIO_ShortList));
	memset(GPIOMSG, 0, 2048);

	//-------------------------------------
	//1. Reset IC
	//-------------------------------------
	nvt_hw_reset();
	msleep(500);	

	//---Get XY channel number---
	nvt_read_firmware_version();
	
	//-------------------------------------
	//2. Set Delay
	//-------------------------------------
	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xF4;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
	msleep(1);

	buffer[0]=0x00;
	buffer[1]=255;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 1);
	msleep(1);

	//--------------------------------------------
	//3. All AINs Check in  ShortTest originally 
	//--------------------------------------------
	if(GPIOShort_AllCheck() >0)
	{
		printk("[max--%s@%d]:   \n",__func__,__LINE__);
		sprintf(GPIOMSG,"FW is Timeout!");
		return 1;
	}
	for(i =0 ; i< 6; i++)
	{

		if(buffer[i+ 2] > 0)
		{
		    Port = buffer[i+2];
			for(j = 0; j < 8; j++)
			{
				unsigned char TestPin;
				TestPin = (0x01  << j);
				if((Port & TestPin) >0)
				{
					unsigned char FailAIN;
					FailAIN = (i* 8) + j;
					if(AIN[FailAIN]==-1){
						continue;
					}
					else 
					{
						GPIO_ShortList[GPIO_ShortCnt] = FailAIN;
						GPIO_ShortCnt++;
					}
				}
			}
		}
	}

	//-----------------------------------------
	// 4. Get Result 
	//-----------------------------------------
	ret =0;
	sprintf(GPIOMSG, "{");
	for(j = 0 ; j < GPIO_ShortCnt; j++)
	{
		printk("[max--%s@%d]:   GPIO_ShortList[j] = %d AIN[i] = %d\n",__func__,__LINE__, GPIO_ShortList[j], AIN[i]);
		i = GPIO_ShortList[j];
		if(i >= 0xff)
		{
			continue;
		}
		else
		if((AIN[i]==-1) || (AIN[i]==255))
		{
			continue;
		}
		
		sprintf(GPIOMSG,"%s%s", GPIOMSG, GPIO2AIN2TxRx(i));
		ret++;
	}
printk("[max--%s@%d]:   ret = %d\n",__func__,__LINE__, ret);
	if(ret <= 1)		
	{
		sprintf(GPIOMSG,"%sGND,},", GPIOMSG);
	}
	else
	{
		sprintf(GPIOMSG,"%s},", GPIOMSG);
	}

	//---Reset IC---
	nvt_hw_reset();

	if(ret > 0)
	{
		// FAIL
		printk("Short Test is FAIL=>%s\n", GPIOMSG);
		return -1;
	}
	else
	{
		// PASS
		printk("Short Test is PASS!\n");
		return 0;
	}
}

void GetTP_CMParameter(void)
{
	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xB7;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
	msleep(1);

	buffer[0]=0x00;
	//nvt_i2c_read_bytes(I2C_FW_Address, buffer[0], &buffer[1], 25);
	i2c_read_bytes(ts->client, buffer[0], &buffer[1], 25);
	
	iAccum =  buffer[1];
	fCF = CF_table[buffer[9]& 0x07];
	fCC = CC_table[((buffer[9]& 0xF8)>>4) |(((buffer[9]& 0xF8)& 0x08) <<1)];
}

int CheckFWStatus(void)
{
	int i;

	for(i=0;i<100;i++)
	{
		msleep(1);
		buffer[0]=0xFF;
		buffer[1]=0x3D;
		buffer[2]=0xFB;
		nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
		msleep(1);

		buffer[0]=0x00;
		nvt_i2c_read_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);

		if(buffer[1]==0xAA)
			break;
		msleep(10);
	}

	if(i==100)
	{
		return -1;
	}
	else
		return 0;
}

void ReadRaw_NT11205(void)
{
	unsigned int startAddr;
	int i, j, k, m;
	int bytecount, sec_ct, Residual_1, sec_ct2, Residual_2, temp_cnt, offsetAddr;

	temp_cnt=0;
	bytecount=XNum*YNum*2;
	sec_ct=bytecount/244;
	Residual_1=bytecount%244;
	sec_ct2=Residual_1/61;
	Residual_2=Residual_1%61;
	startAddr=0x0800;
	
	for(m=0; m<sec_ct; m++)
	{
		offsetAddr=0;
		buffer[0]=0xFF;
		buffer[1]=startAddr>>8;
		buffer[2]=startAddr&0xFF;
		nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
		msleep(1);

		for(k=0; k<4; k++)
		{
			buffer[0]=offsetAddr&0xFF;
			//nvt_i2c_read_bytes(I2C_FW_Address, buffer[0], &buffer[1], 62);
			i2c_read_bytes(ts->client, buffer[0], &buffer[1], 62);
			offsetAddr+=61;
			for(i=0; i<61; i++)
			{
				DiffTemp[temp_cnt++]=(int)buffer[1+i];
			}
		}
		startAddr+=offsetAddr;
	}
	
	if(Residual_1>0)
	{
		offsetAddr=0;
		buffer[0]=0xFF;
		buffer[1]=startAddr>>8;
		buffer[2]=startAddr&0xFF;
		nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
		msleep(1);

		for(k=0; k<sec_ct2; k++)
		{
			buffer[0]=offsetAddr&0xFF;
			//nvt_i2c_read_bytes(I2C_FW_Address, buffer[0], &buffer[1], 62);
			i2c_read_bytes(ts->client, buffer[0], &buffer[1], 62);
			offsetAddr+=61;
			for(i=0; i<61; i++)
			{
				DiffTemp[temp_cnt++]=(int)buffer[1+i];
			}
		}
		startAddr+=offsetAddr;
		if(Residual_2>0)
		{
			offsetAddr=0;
			buffer[0]=0xFF;
			buffer[1]=startAddr>>8;
			buffer[2]=startAddr&0xFF;
			nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
			msleep(1);

			buffer[0]=offsetAddr&0xFF;
			//nvt_i2c_read_bytes(I2C_FW_Address, buffer[0], &buffer[1], Residual_2+1);
			i2c_read_bytes(ts->client, buffer[0], &buffer[1], Residual_2+1);
			for(i=0; i<Residual_2; i++)
			{
				DiffTemp[temp_cnt++]=(int)buffer[1+i];
			}
		}
	}

	for(j=0; j<YNum; j++)
	{
		for(i=0; i<XNum; i++)
		{
			Mutual_Data[j*XNum+i] = (unsigned short)((DiffTemp[2*(j*XNum+i)]<<8)|DiffTemp[2*(j*XNum+i)+1]);
		}
	}			
}

void RawDataToCM(void)
{
	int i,j;
	int kk=0;
    int RepeatCnt = 0;
	int temp1;

AgainGetData:
	//---Reset IC---
	nvt_hw_reset();
	msleep(500);

	//---Turn on Single Scan Mode---
	buffer[0]=0xFF;
	buffer[1]=0x3F;
	buffer[2]=0xE8;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
	msleep(1);			
	buffer[0]=0x00;
	buffer[1]=0xCF;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 1);
	msleep(500);	

	//---Enter Debug Mode---
	buffer[0]=0xFF;
	buffer[1]=0x3D;
	buffer[2]=0xFC;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);
	msleep(1);			
	buffer[0]=0x00;
	buffer[1]=0xAA;
	buffer[2]=0x5A;
	nvt_i2c_write_bytes(I2C_FW_Address, buffer[0], &buffer[1], 2);

	//---Get ACCUM, CC, CF---
	GetTP_CMParameter();
	
	//---Backup Data at last time---
	for(j=0; j<YNum; j++)
	{
		for(i=0; i<XNum; i++)
		{
			RawDataTmp[j*XNum + i] = Mutual_Data[j*XNum + i];
		}
	}
	
	//---Check FW ready---
	if(CheckFWStatus()==-1)
	{
		TPD_DMESG("%s : CheckFWStatus() failed.\n", __func__);
		return;
	}

	//---Read Rawdata---
	ReadRaw_NT11205();

	//----------------------------------------------------------
	kk =0;
	for(j=0; j<YNum; j++)
	{
		for(i=0; i<XNum; i++)
		{
			if(abs(RawDataTmp[j*XNum + i] - Mutual_Data[j*XNum + i] > 1000))
			{
				kk++;
				if(kk > 2)
				{
					RepeatCnt++;
					goto AgainGetData;
				}
			}
		}
	}

	//-------------------------------------------
	// Calculate CM
	//-------------------------------------------
	temp1=(int)(((2*(long int)fCC*10/3)*10000/fCF));	
	for(j=0; j<YNum; j++)
	{
		for(i=0; i<XNum; i++)
		{
			CM_Data[XNum*j+i]= (int)((((((((Mutual_Data[XNum*j+i]/((iAccum+1)/2))-1024)*24)+temp1)/2)*3/10)*fCF/100));
			//CM_Data[XNum*j+i]= (int)(((((((((float)Mutual_Data[XNum*j+i]/(float)((iAccum+1)/2))-1024.0f)*0.00244f)+((2.0*fCC*3.3)/fCF))/2.0f)/3.3)*fCF)*10000);
		}
	}
}

static int fih_opentest_exec(void)
{
	//printk("[max--%s@%d]:   \n",__func__,__LINE__);
	int i,j;
	int kk=0;

	//---Reset IC---
	nvt_hw_reset();
	msleep(500);

	//---Get XY channel number---
	nvt_read_firmware_version();

	//--------------------------------------------------------
	// 1. Load Golden FPC CM
	//--------------------------------------------------------


	//--------------------------------------------------------
	// 2. Init RecordResult array
	//--------------------------------------------------------
	for(j =0 ; j < YNum ; j++)
	{
		for(i = 0; i <XNum; i++)
		{
			RecordResult[j*XNum + i] = 0x00;
		}
	}

	//--------------------------------------------------------
	// 3. Get RawData and To CM 
	//--------------------------------------------------------
	RawDataToCM();

	//Dump Firmware Info.
	printk("XNum=%d\n", XNum);
	printk("YNum=%d\n", YNum);
	printk("iAccum=%d\n", iAccum);
	printk("fCC=%d\n", fCC);
	printk("fCF=%d\n", fCF);

	//Dump Mutual Data
	printk("\nMutual_Data\n");
	for(j = 0; j<YNum; j++)
	{
		for(i=0; i< XNum; i++)
		{
			printk("%6d, ", Mutual_Data[XNum*j +i]);
		}
		printk("\n");
	}
	printk("\n");

	//Get & Dump Cm Data
	printk("CM_Data\n");		
	for(j = 0; j<YNum; j++)
	{
		for(i=0; i< XNum; i++)
		{
			CM_Data[XNum*j +i]-= FPC_CM[XNum*j +i];
			printk("%6d, ", CM_Data[XNum*j +i]);
		}
		printk("\n");
	}
	printk("\n");
	printk("[max--%s@%d]:  sizeof(CM_Data) = %lu \n",__func__,__LINE__, sizeof(CM_Data));
	memcpy(CM_Data_bak, CM_Data, sizeof(CM_Data));
	printk("[max--%s@%d]:  \n",__func__,__LINE__);

	printk("CM_Data_bak:\n");		
	for(j = 0; j<YNum; j++)
	{
		for(i=0; i< XNum; i++)
		{
			//CM_Data_bak[XNum*j +i]-= FPC_CM[XNum*j +i];
			printk("%6d, ", CM_Data_bak[XNum*j +i]);
		}
		printk("\n");
	}
	printk("\n");
	//printk("[max--%s@%d]:  sizeof(CM_Data) = %lu \n",__func__,__LINE__, sizeof(CM_Data));	

	for(j = 0; j < YNum ;  j++)
	{
		for(i = 0 ; i<XNum ; i++)
		{
			if(CM_Data[j*XNum + i] == 0)
				 CM_Data[j*XNum + i] = 1;
			else if(CM_Data[j*XNum + i] < 0)
				 CM_Data[j*XNum + 1] = 1;
		}
	}
	
	//--------------------------------------------------------
	// 4 . Get Golden Sensor CM
	//--------------------------------------------------------

	//--------------------------------------------------------
	// 5 . Check abs low boundary
	//--------------------------------------------------------
	for(j =0 ; j < YNum ; j++)
	{
		for(i = 0; i <XNum; i++)
		{		
			kk = (int)((Mutual_AVG[j*XNum+i]*(1000-iTolerance_S))/1000);
			if(CM_Data[j*XNum + i]  < kk )
			{
				RecordResult[j*XNum + i] |= 1;
			}

			kk = (int)((Mutual_AVG[j*XNum+i]*(1000+iPostiveTolerance))/1000);
			if(CM_Data[j*XNum + i]  > kk )
			{
				RecordResult[j*XNum + i] |= 1;
			}
		}
	}

	//--------------------------------------------------------
	// 6 . Record Test Result
	//--------------------------------------------------------
	kk = 0;
	printk("RecordResult:\n");
	for(j=0;j<YNum;j++)
	{		
		for(i=0; i<XNum;i++)
		{
			printk("%6d, ", RecordResult[j*XNum + i]);

			//kk = 0;
			if((RecordResult[j*XNum+ i] & 0x01) > 0)
				kk++;
			if((RecordResult[j*XNum+ i] & 0x02) > 0)
				kk++;
		}
		printk("\n");
	}
printk("\n\n");
for(j=0;j<YNum;j++)
	{		
		for(i=0; i<XNum;i++)
		{
			printk("%d, ", RecordResult[j*XNum + i]);

		}
		printk("\n");
	}


	//---Reset IC---
	nvt_hw_reset();
printk("[max--%s@%d]:  kk = %d iTolerance_S = %d iPostiveTolerance = %d\n",__func__,__LINE__, kk, iTolerance_S, iPostiveTolerance);
	if(kk > 0)
	{
		// FAIL
		 printk("[max--%s@%d]:   \n",__func__,__LINE__);
 		TPD_DMESG("Open Test is FAIL=>%d\n", kk);
		return -1;
	}
	else
	{
		// PASS
		TPD_DMESG("Open Test is PASS!\n");

		return 0;
	}
}

static int fih_sysfs_selftest_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	//printk("[max--%s@%d]:   \n",__func__,__LINE__);
	int ret_short, ret_open;
	ret_short = 0;

	//ret_short = fih_shorttest_exec();
	ret_open = fih_opentest_exec();
printk("[max--%s@%d]:   ret_short = %d  ret_open = %d\n",__func__,__LINE__, ret_short, ret_open);
    if((ret_short==0) && (ret_open==0))
	   // return sprintf(&buf[0], "0\n");
		return sprintf(&buf[0], "0");	//open and short test all PASS
    else if((ret_short!=0) && (ret_open==0))
       // return sprintf(&buf[0], "1\n");	
		return sprintf(&buf[0], "1");//short FAIL, open pass
	else if((ret_short==0) && (ret_open!=0))
		return sprintf(&buf[0], "2");//short pass, open fail
	else if((ret_short!=0) && (ret_open!=0))
		return sprintf(&buf[0], "4");//open and short all FAIL
}

static int fih_sysfs_selftest_fqc(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    return 0;
    //return sprintf(&buf[0], "0\n");
}
/*
static int fih_get_selftest_result(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    int ret;

	ret = fih_selftest_exec();
    if(ret == 0)
	    return sprintf(&buf[0], "0\n");
    else
        return sprintf(&buf[0], "1\n");
}

static int fih_sysfs_tp_rawdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = 0;
    
    //len += sprintf(&buf[len], "Data=2;(D,%d,%d,%d,%d);",rawDefMin,rawDefMax,rawDefMin,rawDefMax);
    //len += sprintf(&buf[len], "(M,%d,%d,%d,%d)",rawTestRow,rawTestColumn,rawTestVal,rawTestVal);
    
	return sprintf(&buf[0], "0\n");
}
*/
//extern char fih_fqc_tpinfo[];
static int fih_sysfs_tp_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "firmware version=%d\n", nvt_read_firmware_version());
}
/*
static int fih_sysfs_tp_ftmcalibration_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    int len = 0;

    len += sprintf(&buf[len], "calibration ok\n");

    return len;
}

static int fih_sysfs_tp_ftmfwupdate_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    int len = 0;

    len += sprintf(&buf[0], "fw update result: %d \n", fts_Update_Firmware());

    return len;
}

*/

/*static int fih_sysfs_selftest_set_tolerance(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
	char (*tol)[20];
	tol =  (char (*) [20])buf;
	//printk("[max--%s@%d]:  buf= %s tol[0] = %s tol[1] = %s\n",__func__,__LINE__, buf, tol[0], tol[1]);
	int tp_up_tol = simple_strtol(tol[0], NULL, 10);
	int tp_low_tol = simple_strtol(tol[1], NULL, 10);
	printk("[max--%s@%d]:  tp_up_tol = %d tp_low_tol = %d\n",__func__,__LINE__, tp_up_tol, tp_low_tol);
	iPostiveTolerance = tp_up_tol;
	iTolerance_S = tp_low_tol;
	printk("[max--%s@%d]:  iPostiveTolerance = %d iTolerance_S = %d\n",__func__,__LINE__, iPostiveTolerance, iTolerance_S);
    return sprintf(&buf[0], "0\n");
}*/

void my_itoa(int value, char *str)
{
    int i,j;
    if (value < 0) //如果是负数,则str[0]='-',并把value取反(变成正整数)

    {
        str[0] = '-';
        value = 0-value;
    }
    for(i=1; value > 0; i++,value/=10) //从value[1]开始存放value的数字字符，不过是逆序，等下再反序过来

        str[i] = value%10+'0'; //将数字加上0的ASCII值(即'0')就得到该数字的ASCII值

    for(j=i-1,i=1; j-i>=1; j--,i++) //将数字字符反序存放

    {
        str[i] = str[i]^str[j];
        str[j] = str[i]^str[j];
        str[i] = str[i]^str[j];
    }
    if(str[0] != '-') //如果不是负数，则需要把数字字符下标左移一位，即减1

    {
        for(i=0; str[i+1]!='\0'; i++)
            str[i] = str[i+1];
        str[i] = '\0';
    }
}

void itoa_mf(int num,char str[])  
{  
    int sign = num;  
    int i = 0;  
    int j = 0;  
    char temp[100];  
    //如果是负数就去掉符号,将-1234转成1234  
    if(sign < 0)  
    {  
        num = -num;  
    }  
    //转成字符串，1234转成"4321"  
    do  
    {  
        temp[i] = num % 10 + '0';  
        num /= 10;  
        i++;  
    }while(num > 0);  
    //如果是负数的话，加个符号在末尾，如："4321-"  
    if(sign < 0)  
    {  
        temp[i++] = '-';  
    }  
    temp[i] = '\0';  
    i--;  
    //将temp数组中逆序输入到str数组中  
    //将"4321-" ====> "-1234"  
    while(i >= 0)  
    {  
        str[j] = temp[i];  
        j++;  
        i--;  
    }  
    //字符串结束标识  
    str[j] = '\0';  
}  

int p;
static int fih_sysfs_open_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	//printk("[max--%s@%d]: time = %d\n",__func__,__LINE__, time);
	int i, j, k = 0;	
	int len = 0;
	char str_temp[5] = {'\0'};
	printk("[max--%s@%d]: time = %d, k = %d \n",__func__,__LINE__, time, k);
if(time == 0)
{	
	printk("str_data1:\n");
	for(i = 0; i < 11; i++)
	{
		for(j = 0; j < 12; j++)
		{
			itoa_mf(CM_Data_bak[XNum * i + j], str_temp); 
			//printk("cm_data_bak[%d]:%d, str_temp:%s, ", (XNum * i + j), CM_Data_bak[XNum * i + j], str_temp);	
			//len += sprintf(&buf[k],  CM_Data_bak[i][j]);
			len += sprintf(&buf[k],  str_temp);
			printk("%s, ", &buf[k]);
			k += 5;		
		}
		printk("\n");
	}
	sprintf(&buf[k],  "\n");
	time++;
} else if(1 == time) {
	printk("str_data2:\n");
	for(i = 8; i < 19; i++)
	//for(i = 0; i < 11; i++)
	{
		for(j = 0; j < 19; j++)
		//for(j = 0; j < 12; j++)
		{
			itoa_mf(CM_Data_bak[XNum * i + j], str_temp); 
			//printk("cm_data_bak[%d]:%d, str_temp:%s, ", (XNum * i + j), CM_Data_bak[XNum * i + j], str_temp);	
			//len += sprintf(&buf[k],  CM_Data_bak[i][j]);
			len += sprintf(&buf[k],  str_temp);
			printk("%s, ", &buf[k]);
			k += 5;		
		}
		printk("\n");
	}
	time++;
}else{
	printk("str_data3:\n");
	for(i = 16; i < 27; i++)
	//for(i = 11; i < 22; i++)
	{
		for(j = 0; j < XNum; j++)
		{
			itoa_mf(CM_Data_bak[XNum * i + j], str_temp); 
			//printk("cm_data_bak[%d]:%d, str_temp:%s, ", (XNum * i + j), CM_Data_bak[XNum * i + j], str_temp);	
			//len += sprintf(&buf[k],  CM_Data_bak[i][j]);
			len += sprintf(&buf[k],  str_temp);
			printk("%s, ", &buf[k]);
			k += 5;		
		}
		printk("\n");
	}
	time = 0;
}
printk("[max--%s@%d]: \n",__func__,__LINE__);
	return len;//sprintf(&buf[0], "2")
}

static DEVICE_ATTR(fts_selftest, S_IRUGO|S_IWUSR, fih_sysfs_selftest_show, fih_sysfs_selftest_fqc);
//static DEVICE_ATTR(fts_selftest_result, S_IRUGO|S_IWUSR, fih_get_selftest_result, NULL);
//static DEVICE_ATTR(tp_rawdata, S_IRUGO|S_IWUSR, fih_sysfs_tp_rawdata_show, NULL);
static DEVICE_ATTR(ftmgetversion, S_IRUGO, fih_sysfs_tp_version_show, NULL);
//static DEVICE_ATTR(ftmcalibration, S_IRUGO|S_IWUSR, fih_sysfs_tp_ftmcalibration_show, NULL);
//static DEVICE_ATTR(ftmfwupdate, S_IRUGO|S_IWUSR, fih_sysfs_tp_ftmfwupdate_show, NULL);
static DEVICE_ATTR(ftm_selftest_tolerance,  S_IRUGO|S_IWUGO, NULL, fih_sysfs_selftest_set_tolerance);
static DEVICE_ATTR(ftm_selftest_get_open_test_data, S_IRUGO|S_IWUGO, fih_sysfs_open_data_show, NULL);


static int fih_tptest_sysfs_init(void)
{
    int ret;
	
    fih_tptest_kobj = kobject_create_and_add("android_touch", NULL);
    if (fih_tptest_kobj == NULL)
    {
        printk("%s: fih selftest subsystem register failed\n", __func__);
        return -1;
    }

    ret = sysfs_create_file(fih_tptest_kobj, &dev_attr_fts_selftest.attr);
    if (ret)
    {
        printk("%s: sysfs_create fts_selftest failed\n", __func__);
        return ret;
    }
#if 0
    ret = sysfs_create_file(fih_tptest_kobj, &dev_attr_fts_selftest_result.attr);
    if (ret) {
        printk("%s: sysfs_create fts_selftest_result failed\n", __func__);
        return ret;
    }

    ret = sysfs_create_file(fih_tptest_kobj, &dev_attr_tp_rawdata.attr);
    if (ret)
    {
        printk("%s: sysfs_create tp_rawdata failed\n", __func__);
        return ret;
    }
#endif
    ret = sysfs_create_file(fih_tptest_kobj, &dev_attr_ftmgetversion.attr);
    if (ret)
    {
        printk("%s: sysfs_create ftmgetversion failed\n", __func__);
        return ret;
    }
#if 0
    ret = sysfs_create_file(fih_tptest_kobj, &dev_attr_ftmcalibration.attr);
    if (ret)
    {
        printk("%s: sysfs_create ftmcalibration failed\n", __func__);
        return ret;
    }

    ret = sysfs_create_file(fih_tptest_kobj, &dev_attr_ftmfwupdate.attr);
    if (ret)
    {
        printk("%s: sysfs_create ftmfwupdate failed\n", __func__);
        return ret;
    }
#endif

	ret = sysfs_create_file(fih_tptest_kobj, &dev_attr_ftm_selftest_tolerance.attr);
    if (ret)
    {
        printk("%s: sysfs_create ftm_selftest_tolerance failed\n", __func__);
        return ret;
    }	


	ret = sysfs_create_file(fih_tptest_kobj, &dev_attr_ftm_selftest_get_open_test_data.attr);
    if (ret)
    {
        printk("%s: sysfs_create ftm_selftest_get_open_test_data failed\n", __func__);
        return ret;
    }	

    printk("FIH selftest subsystem register succeed!\n");
    return 0;
}

static void fih_tptest_sysfs_deinit(void)
{	
    sysfs_remove_file(fih_tptest_kobj, &dev_attr_fts_selftest.attr);
//    sysfs_remove_file(fih_tptest_kobj, &dev_attr_fts_selftest_result.attr);
//    sysfs_remove_file(fih_tptest_kobj, &dev_attr_tp_rawdata.attr);
    sysfs_remove_file(fih_tptest_kobj, &dev_attr_ftmgetversion.attr);
//    sysfs_remove_file(fih_tptest_kobj, &dev_attr_ftmcalibration.attr);
//    sysfs_remove_file(fih_tptest_kobj, &dev_attr_ftmfwupdate.attr);
	kobject_del(fih_tptest_kobj);
}


#define TOUCH_IOC_MAGIC						'A'
#define GTP_START_OPEN_TEST 				_IOR(TOUCH_IOC_MAGIC, 0x03, int)
#define GTP_SET_OPEN_TEST_THD				_IOW(TOUCH_IOC_MAGIC, 0x04, struct goodix_open_test_thd)
#define GTP_GET_RAW_DATA				_IOR(TOUCH_IOC_MAGIC, 0x05, unsigned short[220])
#define GTP_GET_MAX_RAW_DATA				_IOR(TOUCH_IOC_MAGIC, 0x06, gtp_max_raw_data)
#define GTP_RSET_TP_CONFIG				_IOW(TOUCH_IOC_MAGIC, 0x07, int)

#define COMPAT_GTP_GET_RAW_DATA				_IOR(TOUCH_IOC_MAGIC, 0x08, unsigned short[221])

//#define ALSPS 0x84

//#define GTP_GET_RAW_DATA				_IOR(ALSPS, 0x27, int)

//#define ALSPS_GET_DRIVER_NAME					_IOR(ALSPS, 0x17, int)

static int nvt_misc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int nvt_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

#ifdef CONFIG_COMPAT

static long tpd_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
//printk("[max--%s@%d]:   \n",__func__,__LINE__);
	long ret;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
	switch (cmd) {
	case COMPAT_GTP_GET_RAW_DATA:
printk("[max--%s@%d]:   \n",__func__,__LINE__);
		
		if(arg32 == NULL)
		{
			printk("invalid argument.");
			return -EINVAL;
		}
		
		ret = file->f_op->unlocked_ioctl(file, GTP_GET_RAW_DATA,
					   (unsigned long)arg32);
		if (ret){
		   printk("GTP_GET_RAW_DATA unlocked_ioctl failed.");
		   return ret;
		}
		
		break;
		
	default:
printk("[max--%s@%d]:   \n",__func__,__LINE__);
		printk("miles-->tpd: unknown IOCTL: 0x%08x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}	
}
#endif


static long nvt_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
//printk("[max--%s@%d]:   \n",__func__,__LINE__);
	/* char strbuf[256]; */
	void __user *data;

	long err = 0;

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err) {
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GTP_GET_RAW_DATA:
		data = (void __user *)arg;

		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		printk("[max--%s@%d]:   \n",__func__,__LINE__);
#if 1
		if (copy_to_user(data, CM_Data_bak, sizeof(CM_Data_bak))) {
			err = -EFAULT;
printk("[max--%s@%d]:   \n",__func__,__LINE__);
			break;
		}
#endif
		break;


	default:
printk("[max--%s@%d]:   \n",__func__,__LINE__);
		printk("miles-->tpd: unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}
printk("[max--%s@%d]:   \n",__func__,__LINE__);
	return err;
}


static struct file_operations nvt_fops = {
/* .owner = THIS_MODULE, */
	.open = nvt_misc_open,
	.release = nvt_misc_release,
	.unlocked_ioctl = nvt_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	//.compat_ioctl = tpd_compat_ioctl,
#endif
};

static struct miscdevice nvt_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &nvt_fops,
};

#endif

#endif 

static void tpd_down(int id, int x, int y, int w, int p)
{
#if MT_PROTOCOL_B
	input_mt_slot(tpd->dev, id);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);	
#else	// PROTOCOL_A
    	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
    	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);

    	input_report_key(tpd->dev, BTN_TOUCH, 1);
    	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    	input_mt_sync(tpd->dev);
#endif
}

static void tpd_up(int id, int x, int y, int w, int p)
{
#if MT_PROTOCOL_B
	input_mt_slot(tpd->dev, id);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
	//input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
#else	// TPD_REPORT_XY_MODE == MODE_A
	input_report_key(tpd->dev, BTN_TOUCH, 0);

	//input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	//input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 0);
	//input_report_abs(tpd->dev, ABS_MT_PRESSURE,	0);
	input_mt_sync(tpd->dev);
#endif
}

static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
	//printk("========================= %s,%d \n",__func__,__LINE__);
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static void tpd_work_func(void)
{
	unsigned char buf[63] = {0};
	unsigned char point_count = 0;	
	int index, pos, input_id;
	int ret1; 
	//ret2,ret3, ret4, ret5, ret6;
	unsigned int x=0, y=0, w=0, p=0;

	ret1 = CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 63);
/*	
	printk("i2c_read_bytes 0~5   = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
	printk("i2c_read_bytes 6~11  = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
	printk("i2c_read_bytes 12~17 = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[12],buf[13],buf[14],buf[15],buf[16],buf[17]);
	printk("i2c_read_bytes 18~23 = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[18],buf[19],buf[20],buf[21],buf[22],buf[23]);
	printk("i2c_read_bytes 24~29 = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[24],buf[25],buf[26],buf[27],buf[28],buf[29]);
	printk("i2c_read_bytes 30~35 = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[30],buf[31],buf[32],buf[33],buf[34],buf[35]);
*/
	if(ret1 != TPD_OK)
	{
		dev_info(&(ts->client->dev), "%s: CTP_I2C_READ failed.\n", __func__ );
		return;
	}

	input_id = (unsigned int)(buf[1]>>3);

#if WAKEUP_GESTURE
	if(bTouchIsAwake == 0)
	{	
		nvt_ts_wakeup_gesture_report(input_id);
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		enable_irq(nt_touch_irq);
		return;
	}
#endif

#if TP_PROXIMITY
	if(input_id == 30)
		ret = tpd_proximity_event(buf[1]&0x07);
#endif
	
	for(index = 0; index < TPD_MAX_POINTS_NUM; index++)
	{
		pos = 1 + 6*index;
		input_id = (unsigned int)(buf[pos+0]>>3) - 1;

		if(((buf[pos]&0x07) == 0x01) || ((buf[pos]&0x07) == 0x02)) //finger down (enter&moving)
		{	
			x = (unsigned int)(buf[pos+1]<<4) + (unsigned int) (buf[pos+3]>>4);
			y = (unsigned int)(buf[pos+2]<<4) + (unsigned int) (buf[pos+3]&0x0f);
			/*
			w = (unsigned int)(buf[pos+4])+10;

			if(w > 255)
				w = 255;
			*/
			if((x < 0) || (y < 0))
				continue;
			if((x > TPD_MAX_WIDTH)||(y > TPD_MAX_HEIGHT))
				continue;

			//TPD_DMESG("tpd_do---wn, x=%d, y=%d\n", x, y);
			tpd_down(input_id, x, y, w, p);

			point_count++;
		}else{ // finger up (break)
			if(MT_PROTOCOL_B)
				tpd_up(index, 0, 0, 0, 0);
			else
				continue; /* PROTOCOL_A */
		}
	}	
	if(point_count == 0)
	{
		if(MT_PROTOCOL_B)
			input_report_key(tpd->dev, BTN_TOUCH, (point_count>0));
		else
			tpd_up(index, 0, 0, 0, 0); /* PROTOCOL_A */
	}
	input_sync(tpd->dev);
			
#if TPD_KEY_NUM > 0
	if(buf[61]==0xF8)
	{
		for(index=0; index<TPD_KEY_NUM; index++)
		{
			input_report_key(tpd->dev, touch_key_array[index], ((buf[62]>>index)&(0x01)));
		}
	}
	else
	{
		for(index=0; index<TPD_KEY_NUM; index++)
		{
			input_report_key(tpd->dev, touch_key_array[index], 0);
		}
	}
	input_sync(tpd->dev);
#endif
	return;
}


static int tpd_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

    sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE);
		
		while (tpd_halt)
		{
			tpd_flag = 0;
			msleep(20);
		} 
		
		wait_event_interruptible(waiter, tpd_flag != 0);
		
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);

		if ( tpd == NULL || tpd->dev == NULL )
		{
			continue;
		}

		// tpd work process function
		tpd_work_func();
		
    }
	while (!kthread_should_stop());

    return TPD_OK;
}


/*******************************************************
Description:
        get Chip ID.
*******************************************************/
static uint8_t nvt_ts_read_chipid(void)
{
        uint8_t buf[8] = {0};
        int retry=0;
		int res = 0;


        //---check NT11205 for 5 times till success---
        for(retry=5; retry>=0; retry--)
        {
                //---sw reset idle---
                buf[0]=0x00;
                buf[1]=0xA5;
                res = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
				//added by nasri. Avoid rebooting for too many tp i2c error.
				if(res == TPD_FAIL)
				{
					buf[1] = 0x00;
					break;
				}
                msleep(100);

                //---write i2c index---
                buf[0]=0xFF;
                buf[1]=0xF0;
                buf[2]=0x00;
                res = CTP_I2C_WRITE(ts->client, 0x01, buf, 3);
				if(res == TPD_FAIL)
				{
					buf[1] = 0x00;
					break;
				}

                msleep(10);

                //---read hw chip id---
                buf[0]=0x00;
                buf[1]=0x00;
               	res = CTP_I2C_READ(ts->client, 0x01, buf, 3);
				if(res == TPD_FAIL)
				{
					buf[1] = 0x00;
					break;
				}

                printk("\nchipid=%d\n", buf[1]);

                if(buf[1] == 5)
                {
                        break;
                }
        }
        return buf[1];
}

/*
static uint8_t nvt_ts_i2c_test(void)
{
        uint8_t buf[128] = {0};
        //int retry=0;
	int i = 0;
	int ret = TPD_FAIL;

        //---test I2C transfer for 100 times till success---
        for(i=0; i<10; i++)
        {
                //---sw reset idle---
                buf[0]=0x00;
                buf[1]=0xA5;
                ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		if(ret == TPD_FAIL) goto err;
		printk("[%d] 1.\n", i);
                msleep(100);

                //---write i2c index---
                buf[0]=0xFF;
                buf[1]=0xF0;
                buf[2]=0x00;

                ret = CTP_I2C_WRITE(ts->client, 0x01, buf, 3); // test FIFO /
		if(ret == TPD_FAIL) goto err;
		printk("[%d] 2.\n", i);
                msleep(10);

                //---read hw chip id---
                buf[0]=0x00;
                buf[1]=0x00;
		
		if(i%2)
                	ret = CTP_I2C_READ(ts->client, 0x01, buf, 64); // test DMA/
		else
			ret = CTP_I2C_READ(ts->client, 0x01, buf, 3); // test FIFO /
		if(ret == TPD_FAIL) goto err;
		printk("[%d] 3.\n", i);
		printk("\n[%d] chipid=%d\n", i, buf[1]);


		buf[0]=0x02;
		buf[1]=0x00;
		if(i%2)
                        ret = CTP_I2C_WRITE(ts->client, 0x01, buf, 14); // test DMA 
                else
                        ret = CTP_I2C_WRITE(ts->client, 0x01, buf, 3); // test FIFO 
                if(ret == TPD_FAIL) goto err;
		printk("[%d] 4.\n", i);
                msleep(10);
		
err:
		printk("[%d] test end %d\n", i, ret);
        }
	return ret;
}
*/
//added by nasri.NVT TP version is showed in proc/nvt_tp_version;
//static int nvt_tp_version_proc_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
//{
/*	//int len = 0;
	//int count = 60;

    if (*ppos)  // CMD call again
    {
        return 0;
    }

	while(tp_update_flag == 0 && count-- > 0)//normal count is 43 when update is finishing
		msleep(1000);

	TPD_DMESG("tp_version[0] = %d\n",tp_version[0]);

	//printk("[max--%s@%d]: version:%d   \n",__func__,__LINE__,tp_version[0]);

	char *p = page;
	p += sprintf(p,"%d\n",tp_version[0]);
	
    *ppos += p - page;

    return (p - page);*/
//}

/*static const struct file_operations nvt_tp_version_proc_ops = {
    .owner = THIS_MODULE,
    //.read = nvt_tp_version_proc_read,
};

static void nvt_tp_version_proc()
{
    struct proc_dir_entry *nvt_tp_version_prEntry = NULL;	
	nvt_tp_version_prEntry = proc_create("ctp_version", 0, NULL, &nvt_tp_version_proc_ops);
    if (nvt_tp_version_prEntry == NULL) {
        TPD_DMESG("add /proc/ctp_version,entry fail \n");
    }
}*/
static int __init tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;
#if WAKEUP_GESTURE
	int retry;
#endif
	TPD_DMESG("MediaTek touch panel i2c probe\n");
    TPD_DMESG("probe handle -- novatek\n");
	printk("[max--%s@%d]:   \n",__func__,__LINE__);
	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	ts->client = client;
	ts->fw_updating = 0;

	mutex_init(&ts->i2c_mutex);

	TPD_DMESG("Device Tree get regulator!");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	/*set 2.8v*/
	if (ret) {
		TPD_DMESG("regulator_set_voltage(%d) failed!\n", ret);
		return -1;
	}
	ret = regulator_enable(tpd->reg);	/*enable regulator*/
	if (ret)
		TPD_DMESG("regulator_enable() failed!\n");
	msleep(100);

	tpd_gpio_output(GTP_RST_PORT, 0);
        msleep(10);
	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	tpd_gpio_output(GTP_RST_PORT, 1);
	//nvt_hw_reset();

	// set INT mode
	//mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	//mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	
	//mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	//[FIXME] no error handling for interrupt registering in this MTK platform.
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
	msleep(100);
#if 0
	/* For Test */
	if(nvt_ts_i2c_test() != TPD_OK){
		 dev_err(&client->dev,
			"%s: [Error] Test DMA I2C Transfer failed!\n",
			__func__);
		//goto ERROR;
	}
	 nvt_hw_reset();
	/* End of Test */
#endif
	//---check NT11205---
	if(nvt_ts_read_chipid() == 5)
	{
		dev_info(&client->dev, "This IC is NT11205.\n");
	}
	else
	{
		dev_info(&client->dev, "This IC is unknown.\n");
		ret = -ENODEV;
		goto ERROR;
	}
	tpd_gpio_as_int(GTP_INT_PORT);
        msleep(50);
	tpd_irq_registration();


	thread = kthread_run(tpd_event_handler, 0, TPD_DEVICE);
    	if (IS_ERR(thread))
	{
        	ret = PTR_ERR(thread);
        	TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", ret);
		goto ERROR;
    	}
	nvt_hw_reset();

    	msleep(1000);

#if BOOT_UPDATE_FIRMWARE
 	tp_update_flag = Check_FW_Ver();
#endif

	/* Enable INT */
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	enable_irq(nt_touch_irq);

    	tpd_load_status = 1;

#if TP_PROXIMITY
    	tpd_proximity_init();
#endif

#if TOUCH_WATCHDOG
	watchdog_init();
	watchdog_enable();
#endif

#if WAKEUP_GESTURE
	for(retry = 0; retry < (sizeof(gesture_key_array)/sizeof(gesture_key_array[0])); retry++)
	{
		input_set_capability(tpd->dev, EV_KEY, gesture_key_array[retry]);
	}
#endif

#if NVT_TOUCH_CTRL_DRIVER
	nvt_flash_proc_init();
#endif

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if(!nvt_fwu_wq)
	{
		printk("%s : nvt_fwu_wq create workqueue failed.\n", __func__);
		ret =  -ENOMEM; 
		goto ERROR;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(4000));
#endif

	TPD_DMESG("MediaTek touch panel i2c probe success\n");
	
	//added by nasri.NVT TP version is showed in proc/nvt_tp_version;
	//nvt_tp_version_proc();


#ifdef FIH_TP_TEST
	fih_tptest_sysfs_init();
#endif
/*
	if (misc_register(&nvt_misc_device)) {
		printk("NVTtouch test: nvt_misc_device register failed\n");
	}
*/

	return TPD_OK;

ERROR:
	TPD_DMESG("MediaTek touch panel i2c probe Failed.\n");
#if TOUCH_I2C_USE_DMA
	if(rDMABuf_va) dma_free_coherent(&tpd->dev->dev, MAX_RX_DMA_BUF, rDMABuf_va, rDMABuf_pa);
	if(wDMABuf_va) dma_free_coherent(&tpd->dev->dev, MAX_TX_DMA_BUF, wDMABuf_va, wDMABuf_pa);
#endif
	if(ts) kfree(ts);

	return ret;
}

static int __init tpd_i2c_remove(struct i2c_client *client)
{
	TPD_DMESG("call func tpd_i2c_remove\n");
	
#ifdef FIH_TP_TEST
		fih_tptest_sysfs_deinit();
#endif

    return TPD_OK;
}

static ssize_t enable_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
          return snprintf(buf, PAGE_SIZE, "%u\n", nt_gesture);
}

static ssize_t enable_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
          if (sscanf(buf, "%u", &nt_gesture) != 1)
            return -EINVAL;
          nt_gesture = nt_gesture > 0 ? 1 : 0;
          return count;
}
static DEVICE_ATTR(enable_gesture, 0664, enable_gesture_show, enable_gesture_store);

static struct device_attribute *nt11205_attrs[] = {
    &dev_attr_enable_gesture,
};
int tpd_local_init(void)
{
	//TPD_DMESG("tpd_local_init  start 0\n");
	
	int ret;
	TPD_DMESG("Device Tree get regulator!");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	printk("[max--%s@%d]:   \n",__func__,__LINE__);
	ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	/*set 2.8v*/
	if (ret) {
		printk("[max--%s@%d]:   \n",__func__,__LINE__);
		TPD_DMESG("regulator_set_voltage(%d) failed!\n", ret);
		return -1;
	}
	
#if TOUCH_I2C_USE_DMA
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        wDMABuf_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, MAX_TX_DMA_BUF, &wDMABuf_pa, GFP_KERNEL);

	//printk("wDMABuf_pa: %lx, %d\n", (unsigned long int)wDMABuf_pa, virt_addr_valid(wDMABuf_pa));
printk("[max--%s@%d]:   \n",__func__,__LINE__);
        if((!wDMABuf_va)){
                dev_err(&tpd->dev->dev,
			"%s: [Error] Allocate DMA I2C buffer failed!\n",
			__func__);
		return TPD_FAIL;
        }

		printk("[max--%s@%d]:   \n",__func__,__LINE__);
        rDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, MAX_RX_DMA_BUF, &rDMABuf_pa, GFP_KERNEL);
        if (!rDMABuf_va){
                dev_err(&tpd->dev->dev,
			"%s: [Error] Allocate DMA I2C buffer failed!\n",
			__func__);
		return TPD_FAIL;
        }	
#endif
	printk("[max--%s@%d]:   \n",__func__,__LINE__);
    if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		printk("[max--%s@%d]:   \n",__func__,__LINE__);
    	TPD_DMESG("unable to add i2c driver.\n");
    	return TPD_FAIL;
    }
 	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, TPD_MAX_POINTS_NUM, 0, 0);
	printk("[max--%s@%d]:   \n",__func__,__LINE__);
	
#if WAKEUP_GESTURE
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
#endif
	
	TPD_DMESG("tpd_local_init  start 1\n");
    if(tpd_load_status == 0)
    {
    	TPD_DMESG("add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return TPD_FAIL;
    }
	
	TPD_DMESG("tpd_local_init start 2\n");
	
#if TPD_KEY_NUM > 0
	tpd_nt11205_key_init();
#endif

    TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);

    tpd_type_cap = 1;

    return TPD_OK;
}

/* Function to manage low power suspend */
void tpd_suspend(struct device *h)
{
	u8 buf[4]={0};

	
	TPD_DEBUG("call function tpd_suspend\n");

    	tpd_halt = 1;

#if TP_PROXIMITY
	if(tpd_proximity_flag == 1)
	{
		return;
	}
#endif

#if TOUCH_WATCHDOG
	watchdog_disable();
#endif

#if WAKEUP_GESTURE
        if(nt_gesture == 1){
	bTouchIsAwake = 0;
	TPD_DMESG("Enable touch wakeup gesture.\n");

	//---write i2c command to enter "wakeup gesture mode"---
	buf[0]=0x88;
	buf[1]=0x55;
	buf[2]=0xAA;
	buf[3]=0xA6;
	//i2c_smbus_write_i2c_block_data(i2c_client, buf[0], 3, &buf[1]);
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);
        }else{
        disable_irq(nt_touch_irq);

	//---write i2c command to enter "deep sleep mode"---
	buf[0]=0x88;
	buf[1]=0x55;
	buf[2]=0xAA;
	buf[3]=0xA5;
	//i2c_smbus_write_i2c_block_data(i2c_client, buf[0], 3, &buf[1]);
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);

        }
#else
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	disable_irq(nt_touch_irq);

	//---write i2c command to enter "deep sleep mode"---
	buf[0]=0x88;
	buf[1]=0x55;
	buf[2]=0xAA;
	buf[3]=0xA5;
	//i2c_smbus_write_i2c_block_data(i2c_client, buf[0], 3, &buf[1]);
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);
#endif

	return;
}

/* Function to manage power-on resume */
void tpd_resume(struct device *h)
{
    //TPD_DEBUG("call function tpd_resume\n");

#if TP_PROXIMITY
	if(tpd_proximity_flag == 1)
	{
		return;
	}
#endif

	tpd_halt = 0;

#if WAKEUP_GESTURE
        if(nt_gesture == 1){
	nvt_hw_reset();
	bTouchIsAwake = 1;
        }else{
	nvt_hw_reset();
	enable_irq(nt_touch_irq);
        }
#else
	nvt_hw_reset();
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
	enable_irq(nt_touch_irq);
#endif

#if TOUCH_WATCHDOG
	watchdog_enable();
#endif

#if CHARGER_DETECT
	msleep(200);
	tpd_usb_plugin(b_usb_plugin);
#endif
}

static struct tpd_driver_t tpd_device_driver = 
{
	.tpd_device_name = NVT_I2C_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#if TPD_KEY_NUM > 0
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
        .attrs = {
             .attr = nt11205_attrs,
             .num  = ARRAY_SIZE(nt11205_attrs),
        },
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    TPD_DMESG("touch panel driver init tpd_driver_init\n");
	
	//i2c_register_board_info(I2C_BUS_NUMBER, &i2c_tpd, 1);
	tpd_get_dts_info();
    if(tpd_driver_add(&tpd_device_driver) < 0)
    {
    	TPD_DMESG("add generic driver failed\n");
    }

    return TPD_OK;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    TPD_DMESG("touch panel driver exit tpd_driver_exit\n");


#if TOUCH_I2C_USE_DMA
    if(rDMABuf_va) dma_free_coherent(NULL, MAX_RX_DMA_BUF, rDMABuf_va, rDMABuf_pa);
    if(wDMABuf_va) dma_free_coherent(NULL, MAX_TX_DMA_BUF, wDMABuf_va, wDMABuf_pa);
#endif
    if(ts) kfree(ts);

    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);



/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
   BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mt-plat/mt_gpio.h>
#endif

#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#endif

//#define USE_LCM_THREE_LANE 1

#define _LCM_DEBUG_

//#include <cust_gpio_usage.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0XFEFF
#define REGFLAG_END_OF_TABLE      							0xFFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_OTM1287A 0x8B


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(gpio_num,val)    						(lcm_util.set_gpio_out((gpio_num),(val)))


#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


#ifdef _LCM_DEBUG_
  #ifdef BUILD_LK
  #define LCM_PRINT printf
  #else
  #define LCM_PRINT printk
  #endif
#else
	#define LCM_PRINT
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	{0x00,1,{0x00}},
	{0xff,3,{0x12,0x87,0x01}},       //EXTC=1
	{0x00,1,{0x80}},                //Orise mode enable
	{0xff,2,{0x12,0x87}},

	//-------------------- panel setting --------------------//
	{0x00,1,{0x92}},                
	#ifdef USE_LCM_THREE_LANE
	{0xff,2,{0x20, 0x02}}, 					//MIPI 3 Lane
	#else
	{0xff,2,{0x30, 0x02}}, 					//MIPI 4 Lane
	#endif

	{0x00,1,{0x80}},               //TCON Setting
	{0xc0,9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},

	{0x00,1,{0x90}},                //source pre. 
	{0xc0,6,{0x00,0x5c,0x00,0x01,0x00,0x04}},

	{0x00,1,{0xb3}},                //Interval Scan Frame: 0 frame, column inversion
	{0xc0,2,{0x00,0x55}},

	{0x00,1,{0x81}},               //frame rate:60Hz
	{0xc1,1,{0x55}},

	{0x00,1,{0xa0}},               //
	{0xc1,1,{0x02}},

	//-------------------- for Power IC --------------------//
	{0x00,1,{0xa0}},              //dcdc setting
	{0xc4,14,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}},

	{0x00,1,{0xb0}},              //clamp voltage setting
	{0xc4,2,{0x00,0x00}},

	{0x00,1,{0x91}},              //VGH=15V, VGL=-10V, pump ratio:VGH=6x, VGL=-5x
	{0xc5,2,{0x46,0x42}},

	//------------------VGLO1/O2 disable----------------
	{0x00,1,{0x00}},              //GVDD=5.008V, NGVDD=-5.008V
	{0xd8,2,{0xc7,0xc7}},

	{0x00,1,{0x00}},             //VCOMDC=-1.584
	{0xd9,1,{0x7d}},

	{0x00,1,{0xb3}},            //VDD_18V=1.7V, LVDSVDD=1.6V
	{0xc5,1,{0x44}},

	{0x00,1,{0xbb}},            //LVD voltage level setting
	{0xc5,1,{0x8a}},

	{0x00,1,{0x82}},             // chopper 0: frame 2: line 4: disable
	{0xC4,1,{0x0a}},

	{0x00,1,{0xc6}},             // debounce
	{0xB0,1,{0x03}},

	//-------------------- power setting --------------------//
	{0x00,1,{0xb2}},             //dcdc setting
	{0xf5,2,{0x00,0x00}},

	{0x00,1,{0xb6}},             //VGLO2
	{0xf5,2,{0x00,0x00}},

	{0x00,1,{0x94}},            //VCL pump ids	
	{0xF5,2,{0x00,0x00}},

	{0x00,1,{0xd2}},              //VCL reg. en
	{0xf5,2,{0x06,0x15}},

	{0x00,1,{0xb4}},             //VGLO1/2 Pull low setting
	{0xc5,1,{0xcc}},

	{0x00,1, {0x90}},             //Mode-3
	{0xf5,4, {0x02,0x11,0x02,0x11}},

	{0x00,1, {0x90}},             //2xVPNL
	{0xc5,1, {0x50}},

	{0x00,1, {0x94}},             //Freq.
	{0xc5, 1, {0x66}},
	
	//-------------------- panel timing state control --------------------//
	{0x00, 1, {0x80}}, 
	{0xcb, 11, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0x90}},
	{0xcb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xa0}},
	{0xcb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xb0}},
	{0xcb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xcb, 15, {0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x05,0x05,0x00,0x05,0x05}},

	{0x00, 1, {0xd0}},
	{0xcb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05}},

	{0x00, 1, {0xe0}},
	{0xcb, 14, {0x05,0x00,0x05,0x05,0x00,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xf0}},
	{0xcb, 11, {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

	//-------------------- panel pad mapping control --------------------//
	{0x00, 1, {0x80}},
	{0xcc, 15, {0x2E,0x2D,0x29,0x2A,0x0E,0x10,0x0A,0x0C,0x02,0x00,0x04,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0x90}}, 
	{0xcc, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2E,0x2D,0x29,0x2A,0x0D,0x0F,0x09,0x0B}},

	{0x00, 1, {0xa0}},
	{0xcc, 14, {0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xb0}},
	{0xcc, 15, {0x2D,0x2E,0x29,0x2A,0x0B,0x09,0x0F,0x0D,0x03,0x00,0x01,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xcc, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2D,0x2E,0x29,0x2A,0x0C,0x0A,0x10,0x0E}},

	{0x00, 1, {0xd0}},
	{0xcc, 14, {0x04,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	//-------------------- panel timing setting --------------------//
	{0x00, 1, {0x80}},             //panel VST setting
	{0xce, 12, {0x8B,0x03,0x00,0x8A,0x03,0x00,0x89,0x03,0x00,0x88,0x03,0x00}},

	{0x00,1, {0x90}},             //panel VEND setting
	{0xce, 14, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1, {0xa0}},             //panel CLKA1/2 setting
	{0xce,14, {0x38,0x07,0x04,0xFB,0x00,0x00,0x00,0x38,0x06,0x04,0xFC,0x00,0x00,0x00}},

	{0x00,1,{0xb0}},             //panel CLKA3/4 setting
	{0xce,14, {0x38,0x05,0x04,0xFD,0x00,0x00,0x00,0x38,0x04,0x04,0xFE,0x00,0x00,0x00}},

	{0x00,1, {0xc0}},            //panel CLKb1/2 setting
	{0xce,14, {0x38,0x03,0x04,0xFF,0x00,0x00,0x00,0x38,0x02,0x05,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xd0}},            //panel CLKb3/4 setting
	{0xce,14,{0x38,0x01,0x05,0x01,0x00,0x00,0x00,0x38,0x00,0x05,0x02,0x00,0x00,0x00}},

	{0x00,1,{0x80}},            //panel CLKc1/2 setting
	{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x90}},             //panel CLKc3/4 setting
	{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xa0}},             //panel CLKd1/2 setting
	{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xb0}},             //panel CLKd3/4 setting
	{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xc0}},             //panel ECLK setting
	{0xcf,11,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x82,0x10,0x00,0x00}}, //gate pre. ena.

	{0x00,1,{0xb5}},             //TCON_GOA_OUT Setting
	{0xc5,6,{0x00,0x6f,0xfF,0x00,0x6f,0xfF}},

	{0x00,1,{0x00}},                                                                                 ///////////////////////
	{0xE1,20,{0x05,0x0e,0x17,0x20,0x2d,0x39,0x3a,0x65,0x58,0x74,0x8c,0x73,0x80,0x56,0x4d,0x3e,0x2c,0x1f,0x1a,0x10}},

	{0x00,1,{0x00}},
	{0xE2,20, {0x05,0x0f,0x17,0x20,0x2d,0x39,0x3a,0x65,0x58,0x74,0x8c,0x73,0x80,0x56,0x4d,0x3e,0x2c,0x1f,0x1a,0x10}},

	//---------------------------------------------------------//
	{0x00,1,{0x00}},             //Orise mode disable
	{0xff,3,{0xff,0xff,0xff}},
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	LCM_PRINT("%s %d\n", __func__,__LINE__);
    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	LCM_PRINT("%s %d\n", __func__,__LINE__);
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
  	LCM_PRINT("junTang %s %d\n", __func__,__LINE__);
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = BURST_VDO_MODE;
	//params->dsi.mode   = SYNC_PULSE_VDO_MODE;	
#endif
	
	// DSI
	/* Command mode setting */
#ifdef USE_LCM_THREE_LANE
	params->dsi.LANE_NUM				= LCM_THREE_LANE; 
#else
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
#endif
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 8;
	params->dsi.vertical_backporch					= 14;
	params->dsi.vertical_frontporch					= 16;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 8;
	params->dsi.horizontal_backporch				= 74;
	params->dsi.horizontal_frontporch				= 74;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#ifdef USE_LCM_THREE_LANE
    params->dsi.PLL_CLOCK = 250;
#else
	params->dsi.PLL_CLOCK = 182;
#endif

	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0xAC;
	params->dsi.lcm_esd_check_table[0].count =1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x00;

	params->dsi.ssc_disable = 1;
}


static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];
	//int gpio;
	
	LCM_PRINT("%s %d\n", __func__,__LINE__);

	//mt_set_gpio_mode(GPIO_LCD_ID_PIN, GPIO_LCD_ID_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_LCD_ID_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_LCD_ID_PIN, GPIO_PULL_DISABLE);

	SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
	MDELAY(20);//50
    SET_RESET_PIN(1);
	MDELAY(100);//100

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	MDELAY(20);
	read_reg_v2(0xA1, buffer, 2);

	//gpio = mt_get_gpio_in(GPIO_LCD_ID_PIN);

	id = buffer[1]; //we only need ID
	LCM_PRINT("%s,  id otm1287A= 0x%08x\n", __func__, id);
	//LCM_PRINT("%s,  gpio= %d\n", __func__, gpio);


	return (LCM_ID_OTM1287A==id /*&& gpio==0*/)?1:0;
}


static void lcm_init(void)
{
	LCM_PRINT(" %s %d\n", __func__,__LINE__);
	SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
	MDELAY(50);
    SET_RESET_PIN(1);
	MDELAY(100);//100
	push_table(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);
	MDELAY(20);//50
    SET_RESET_PIN(0);
	MDELAY(20);//50
    SET_RESET_PIN(1);
	MDELAY(100);//100
}


static void lcm_resume(void)
{
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	LCM_PRINT("%s %d\n", __func__,__LINE__);
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00000000;
	data_array[4]= 0x00053902;
	data_array[5]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[6]= (y1_LSB);
	data_array[7]= 0x00000000;
	data_array[8]= 0x002c3909;

	dsi_set_cmdq(&data_array, 9, 0);

}
#endif

LCM_DRIVER otm1287a_hd720_dsi_vdo_e539_l521_ld_lcm_drv = 
{
    .name			= "otm1287a_hd720_dsi_vdo_e539_l521_ld",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

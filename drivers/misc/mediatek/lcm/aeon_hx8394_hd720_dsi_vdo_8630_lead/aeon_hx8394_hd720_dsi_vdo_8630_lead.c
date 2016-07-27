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

#define REGFLAG_DELAY             							0xFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(gpio_num,val)    						(lcm_util.set_gpio_out((gpio_num),(val)))


#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))





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
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
{0xB9,3,{ 0xFF, 0x83, 0x94}},				
{0xBA,6,{ 0x63,0x03,0x68,0x6B,0xB2,0xC0}},
{0xB2,6,{ 0x00,0x80,0x64,0x0E,0x0A,0x2F}},
{0xB4,21,{0x1C,0x78,0x1C,0x78,0x1C,0x78,0x01,0x0c,0x86,0x75,0x00,0x3F,0x1C,0x78,0x1C,0x78,0x1C,0x78,0x01,0x0C,0x86}},
{0xD3,33,{0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x32,0x10,0x03,0x00,0x03,0x32,0x13,0xC0,0x00,0x00,0x32,0x10,0x08,0x00,0x00,0x37,0x04,0x05,0x05,0x37,0x05,0x05,0x47,0x0E,0x40}},
{0xD5,44,{0x18,0x18,0x18,0x18,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x19,0x19,0x20,0x21,0x22,0x23}},
{0xD6,44,{0x18,0x18,0x19,0x19,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x18,0x18,0x23,0x22,0x21,0x20}},
{0xE0,58,{0x00,0x02,0x0B,0x10,0x12,0x16,0x19,0x17,0x2F,0x3F,0x4F,0x4E,0x56,0x69,0x6E,0x70,0x7D,0x81,0x7E,0x8D,0xA1,0x51,0x51,0x56,0x5A,0x60,0x69,0x7E,0x7F,0x00,0x02,0x0B,0x10,0x12,0x16,0x19,0x17,0x2F,0x3F,0x4D,0x4D,0x56,0x69,0x6E,0x71,0x7E,0x82,0x80,0x8F,0xA2,0x51,0x51,0x56,0x5B,0x61,0x6A,0x7F,0x7F}},

{0xBD,1,{ 0x00}},
{0xC1,43,{0x01,0x00,0x08,0x10,0x18,0x20,0x28,0x30,0x38,0x40,0x47,0x4F,0x56,0x5E,0x64,0x6D,0x76,0x7E,0x86,0x8D,0x96,0x9D,0xA5,0xAC,0xB4,0xBC,0xC4,0xCD,0xD5,0xDD,0xE6,0xEF,0xF7,0xFF,0x15,0x51,0x62,0x3C,0x8C,0xCA,0x11,0x90,0xC0}},
{0xBD,1,{ 0x01}},
{0xC1,42,{0x00,0x07,0x0F,0x17,0x1E,0x26,0x2E,0x35,0x3D,0x44,0x4B,0x52,0x59,0x60,0x68,0x6F,0x77,0x7F,0x86,0x8D,0x95,0x9C,0xA3,0xAA,0xB1,0xB8,0xC0,0xC8,0xCF,0xD7,0xDF,0xE7,0xEF,0x24,0x90,0x63,0xD3,0xAB,0xB5,0xB0,0xE9,0x80}},
{0xBD,1,{ 0x02}},
{0xC1,42,{0x00,0x07,0x0E,0x15,0x1C,0x24,0x2C,0x33,0x3C,0x43,0x4A,0x52,0x59,0x60,0x68,0x70,0x78,0x80,0x87,0x8E,0x96,0x9E,0xA4,0xAC,0xB3,0xBA,0xC2,0xCA,0xD2,0xDA,0xE2,0xE9,0xF1,0x00,0x2B,0x79,0x80,0x07,0xCC,0xB7,0x46,0x80}},
{0xBD,1,{ 0x00}},

{0xB1,10,{0x50,0x12,0x72,0x09,0x32,0x54,0x71,0x51,0x30,0x50}},
{0xCC,1,{ 0x0b}},
{0xC0,2,{ 0x1F,0x73}},
{0xB6,2,{ 0x51,0x51}},//VCOM
{0xD4,1,{ 0x02}},
{0xbd,1,{ 0x01}},
{0xb1,1,{ 0x00}},
{0xbD,1,{ 0x00}},
{0xBF,7,{ 0x40,0x81,0x50,0x00,0x1A,0xFC,0x01}},
//{0x36,1,{ 0x02}},
{0xc6,1,{ 0xef}},
{0x51,1,{0xff}},
{0x53,1,{0x24}},

    // Sleep Out
	{0x11, 1, {0x00}},
        {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
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

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static struct LCM_setting_table lcm_sleep_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
  
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 86;
	params->dsi.horizontal_backporch				= 55;
	params->dsi.horizontal_frontporch				= 55;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

//#ifndef CONFIG_FPGA_EARLY_PORTING
//#if (LCM_DSI_CMD_MODE)
//	params->dsi.PLL_CLOCK = 350; //this value must be in MTK suggested table
//#else
	params->dsi.PLL_CLOCK = 208; //this value must be in MTK suggested table
//#endif
//#else
//	params->dsi.pll_div1 = 0;
//	params->dsi.pll_div2 = 0;
//	params->dsi.fbk_div = 0x1;
//#endif

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
        params->dsi.lcm_esd_check_table[0].cmd			= 0x09;
	params->dsi.lcm_esd_check_table[0].count		=3;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x04;

//	params->dsi.vertical_vfp_lp = 100;

}


static void lcm_init(void)
{


    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_suspend(void)
{
        push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
        MDELAY(20);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(20);


}


static void lcm_resume(void)
{
	lcm_init();
}



static unsigned int lcm_compare_id(void)
{
	char  buffer;//,lcd_id;
	unsigned int data_array[2];


    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(120);
    SET_RESET_PIN(1);
    MDELAY(50);	



	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= (0x33<<8)|0xba;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xf4, &buffer, 1);

#ifdef BUILD_LK
		printf("%s, LK debug: hx8394d id = 0x%08x\n", __func__, buffer);
#else
		printk("%s, kernel debug: hx8394d id = 0x%08x\n", __func__, buffer);
#endif

	//mt_set_gpio_mode(GPIO_LCD_ID_PIN, GPIO_LCD_ID_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_LCD_ID_PIN, GPIO_DIR_IN);	  
	//mt_set_gpio_pull_enable(GPIO_LCD_ID_PIN, GPIO_PULL_DISABLE);
	
	MDELAY(50);

	//lcd_id = mt_get_gpio_in(GPIO_LCD_ID_PIN);

#ifdef BUILD_LK
	//printf("%s, LK debug: aeon_factory id = %d\n", __func__, lcd_id);
#else
	//printk("%s, kernel debug: aeon_factory id = %d\n", __func__, lcd_id);
#endif



	return (buffer == 0x94 ? 1 : 0);

}




LCM_DRIVER aeon_hx8394_hd720_dsi_vdo_8630_lead_lcm_drv = 
{
    .name			= "aeon_hx8394_hd720_dsi_vdo_8630_lead",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,

};

// SPDX-License-Identifier: GPL-2.0

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

//guozhiliang add TP lcd sleep in PROXIMITY mode 
#if defined(CONFIG_TP_PROXIMITY_SLEEP_CONTROLL_IN_BSP)
extern int  cts_get_prox_en(void);
static int tp_ps_work_skip_lcd_poweroff_flag=0;
#endif
//guozhiliang add TP lcd sleep in PROXIMITY mode 

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

#define LCM_DSI_CMD_MODE 0
#define FRAME_WIDTH		(720)
#define FRAME_HEIGHT		(1600)

#define LCM_PHYSICAL_WIDTH              (70300)
#define LCM_PHYSICAL_HEIGHT             (156200)


#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY		0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

extern int display_bias_enable_VSP(void);
extern int display_bias_enable_VSN(void);

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0xF0,2,{0x5A,0x59}},
	{0xF1,2,{0xA5,0xA6}},
	{0xBB,8,{0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x00}},
	{0x6D,2,{0x25,0x00}},
	{0x28,2,{0x00,0x00}},
	{REGFLAG_DELAY, 10, {} },
	{0x10,2,{0x00,0x00}},
	{REGFLAG_DELAY, 120, {} },
	{0xC9,2,{0x01,0x00}},
	{REGFLAG_DELAY, 10, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table init_setting[] = {
	{0xF0,3,{0x99,0x16,0x0C}},
	//CGOUT FORWARD SCAN		
	{0xC1,32,{0x10,0x20,0x20,0x18,0x04,0x30,0x30,0x04,0x40,0x06,0x22,0x70,0x33,0x31,0x07,0x11,0x84,0x4C,0x00,0x93,0x13,0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xC2,1,{0x00}},
	{0xC3,8,{0x06,0x00,0xFF,0x00,0xFF,0x4D,0x10,0x01}},
	{0xC4,12,{0x04,0x33,0xB8,0x40,0x00,0xBC,0x00,0x00,0x00,0x00,0x00,0xF0}},
	{0xC5,23,{0x03,0x21,0x96,0xC8,0x3E,0x00,0x04,0x01,0x14,0x04,0x0E,0x18,0xC6,0x03,0x64,0xFF,0x01,0x04,0x18,0x22,0x45,0x14,0x38}},
		  
	{0xC6,12,{0x72,0x24,0x13,0x2B,0x2B,0x28,0x3F,0x02,0x16,0x16,0x00,0x01}},
	{0xCA,32,{0x34,0x50,0x04,0x19,0x46,0x94,0x41,0x8F,0x44,0x44,0x36,0x50,0x54,0x54,0x39,0x5A,0x5A,0x5A,0x33,0x00,0x01,0x01,0x0E,0x3F,0xD2,0x00,0x05,0x00,0x00,0x5A,0x5A,0x00}},	  
	


	//{0x36,1,{0x03}},
	{0xB2,24,{0x05,0x04,0x10,0x10,0x44,0x44,0x82,0x88,0x44,0x65,0x59,0x65,0x59,0x65,0x59,0x65,0x59,0x65,0x59,0x65,0x59,0x00,0x00,0x00}},
		  
	{0xB3,15,{0xF4,0x01,0x01,0x11,0x91,0x66,0x00,0x00,0x59,0x00,0x00,0x65,0x4A,0x65,0x4A}},
		  
	{0xB4,28,{0x19,0x0B,0x06,0x0B,0x06,0x26,0x26,0x88,0xA2,0x88,0x44,0x3B,0x26,0x00,0x55,0x3C,0x02,0x08,0x20,0x30,0x00,0x12,0x20,0x40,0x11,0x10,0x20,0x00}},
	
	{0xB5,30,{0x00,0x00,0x08,0x04,0x2E,0x2F,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x28,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0xFF,0xFF,0xFC,0x0C,0x00,0x00,0x3C,0x00}},
	{0xB6,30,{0x00,0x00,0x09,0x05,0x2E,0x2F,0x0D,0x0F,0x11,0x13,0x15,0x17,0x19,0x1B,0x29,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0xFF,0xFF,0xFC,0x0C,0x00,0x00,0x3C,0x00}},	  
	{0xB7,22,{0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55}},
	
	{0xBB,8,{0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x00}},
	
	{0xBC,19,{0x00,0x00,0x00,0x00,0x04,0x00,0xFF,0xF0,0x0B,0x33,0x5C,0x5B,0x43,0x33,0x00,0x5A,0x5A,0x55,0x55}},
	{0xBD,5,{0xA1,0x0A,0x52,0xAE,0x8F}},	  
	{0xBF,9,{0x0C,0x19,0x0C,0x19,0x00,0x11,0x04,0x18,0x50}},

//	{0x89,3,{0xF3,0x00,0x00}},

	//VCOM//已经烧录vcom的模组，请屏蔽0x89
	////{0x89,3,{0x88,0x88,0x11}},
	//GAMMA
	{0xC7,27,{0x77,0x77,0x77,0x77,0x77,0x77,0x77,0x77,0x70,0x77,0x77,0x77,0x77,0x77,0x77,0x77,0x77,0x70,0x31,0x00,0x01,0xFF,0xFF,0x00,0x8A,0x8A,0x40}},
		  
	{0x80,32,{0xFF,0xFA,0xF2,0xEB,0xE5,0xE0,0xDB,0xD6,0xD2,0xC4,0xB9,0xAF,0xA7,0xA0,0x9A,0x8E,0x85,0x7B,0x73,0x72,0x6A,0x62,0x58,0x4E,0x42,0x32,0x27,0x1B,0x17,0x13,0x0F,0x0B}},
		  
	{0x81,32,{0xFF,0xFA,0xF2,0xEB,0xE5,0xE0,0xDB,0xD6,0xD2,0xC4,0xB9,0xAF,0xA7,0xA0,0x9A,0x8E,0x85,0x7B,0x73,0x72,0x6A,0x62,0x58,0x4E,0x42,0x32,0x27,0x1B,0x17,0x13,0x0F,0x0B}},
		  
	{0x82,32,{0xFF,0xFA,0xF2,0xEB,0xE5,0xE0,0xDB,0xD6,0xD2,0xC4,0xB9,0xAF,0xA7,0xA0,0x9A,0x8E,0x85,0x7B,0x73,0x72,0x6A,0x62,0x58,0x4E,0x42,0x32,0x27,0x1B,0x17,0x13,0x0F,0x0B}},
		  
	{0x83,25,{0x01,0x06,0x02,0x00,0x00,0x06,0x02,0x00,0x00,0x06,0x02,0x00,0x00,0x0A,0x06,0x02,0x00,0x0A,0x06,0x02,0x00,0x0A,0x06,0x02,0x00}},
	
	{0x84,27,{0x2B,0x97,0x97,0xF7,0x37,0x4D,0x0D,0xF8,0xA4,0x2B,0x97,0x97,0xF7,0x37,0x4D,0x0D,0xF8,0xA4,0x2B,0x97,0x97,0xF7,0x37,0x4D,0x0D,0xF8,0xA4}},
		  
	{0xF0,3,{0x00,0x00,0x00}},
	{0x35,2,{0x00,0x00}},//用long package的包头（0x29或0x39）
	{0x11,2,{0x00,0x00}},//用long package的包头（0x29或0x39）
	{REGFLAG_DELAY, 120, {} },
	{0x29,2,{0x00,0x00}},//用long package的包头（0x29或0x39）
	{REGFLAG_DELAY, 10, {} },
	{0x6D,2,{0x02,0x00}},//用long package的包头（0x29或0x39）
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
				table[i].para_list, force_update);
		}
	}
}

#ifdef CONFIG_SR_HARDWARE_INFOMATION
extern void sr_add_lcd_ic_name(char *name);
extern void sr_add_lcd_vendor_name(char *name);
extern void sr_add_lcd_size_name(char *name);
#endif
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
#ifdef CONFIG_SR_HARDWARE_INFOMATION
	sr_add_lcd_ic_name("ICNL9916AC");
	sr_add_lcd_vendor_name("HX");
	sr_add_lcd_size_name("6.745");
#endif
}


#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	struct dfps_info *dfps_params = dsi->dfps_params;

	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 9000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/

	/*traversing array must less than DFPS_LEVELS*/
	/*DPFS_LEVEL0*/
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	//dfps_params[0].PLL_CLOCK = 455;
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/
	dfps_params[0].vertical_frontporch = 280;
	// dfps_params[1].vertical_frontporch_for_low_power = 2500;

	/*if need mipi hopping params add here*/
	// dfps_params[1].dynamic_switch_mipi = 0;
	// dfps_params[1].PLL_CLOCK_dyn = 550;
	// dfps_params[1].horizontal_frontporch_dyn = 288;
	// dfps_params[1].vertical_frontporch_dyn = 54;
	// dfps_params[1].vertical_frontporch_for_low_power_dyn = 2500;

	/*DPFS_LEVEL1*/
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	//dfps_params[1].PLL_CLOCK = 300;
	/*dfps_params[0].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[0].horizontal_frontporch = xx;*/
	dfps_params[1].vertical_frontporch = 1270;
	// dfps_params[0].vertical_frontporch_for_low_power = 2500;

	/*if need mipi hopping params add here*/
	// dfps_params[0].dynamic_switch_mipi = 1;
	// dfps_params[0].PLL_CLOCK_dyn = 550;
	// dfps_params[0].horizontal_frontporch_dyn = 288;
	// dfps_params[0].vertical_frontporch_dyn = 1291;
	// dfps_params[0].vertical_frontporch_for_low_power_dyn = 2500;
	dsi->dfps_num = 2;
}
#endif


int lcd_ic_9916=0;

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

  params->physical_width = LCM_PHYSICAL_WIDTH / 1000;
  params->physical_height = LCM_PHYSICAL_HEIGHT / 1000;
  params->physical_width_um = LCM_PHYSICAL_WIDTH;
  params->physical_height_um = LCM_PHYSICAL_HEIGHT;
 

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_EVENT_VDO_MODE;
#endif

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 32;
	params->dsi.vertical_frontporch = 280;//90fps->280 60fps->1270
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 48;
	params->dsi.horizontal_frontporch = 48;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable  = 1;

	params->dsi.PLL_CLOCK = 455;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
#endif
	
	lcd_ic_9916=1;
	printk("icnl9916 hx lcd_ic_9916=1\n");

}

static void lcm_init_power(void)
{
	printk("icnl9916 hx %s enter\n", __func__);
	//display_bias_enable();
}

extern int chipone_gesture_wakeup_flag;
static void lcm_suspend_power(void)
{
	printk("icnl9916 hx %s enter\n", __func__);
	printk("icnl9916 hx lcd wakeup chipone_gesture_wakeup_flag=%d\n", chipone_gesture_wakeup_flag);
	if (chipone_gesture_wakeup_flag == 0) {
		display_bias_disable();
		printk("icnl9916 hx %s disable double wakeup\n", __func__);
	} else if (chipone_gesture_wakeup_flag == 1) {
		printk("icnl9916 hx %s enable double wakeup\n", __func__);
	}
}

static void lcm_resume_power(void)
{
	printk("icnl9916 dzx %s enter\n", __func__);

	
	//SET_RESET_PIN(0);
	//display_bias_enable();
}

static void lcm_init(void)
{
	unsigned int size;
	
	printk("icnl9916 hx %s enter\n", __func__);
	
	display_bias_enable_VSP();	
	MDELAY(5);	
	SET_RESET_PIN(0);	
	MDELAY(2);
	SET_RESET_PIN(1);
	MDELAY(8);
	display_bias_enable_VSN();
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(12);
	SET_RESET_PIN(1);
	MDELAY(12);   
   

	size = sizeof(init_setting) /
		sizeof(struct LCM_setting_table);
		
	push_table(NULL, init_setting, size, 1);

}

static void lcm_suspend(void)
{
	printk("icnl9916 hx %s enter\n", __func__);


	push_table(NULL, lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
		1);
		
	MDELAY(10);
	
	/* SET_RESET_PIN(0); */
}

static void lcm_resume(void)
{
	unsigned int size;
	printk("icnl9916 hx %s enter\n", __func__);
	printk("icnl9916 hx lcd wakeup, chipone_gesture_wakeup_flag=%d\n", chipone_gesture_wakeup_flag);

	if (chipone_gesture_wakeup_flag == 0) {
		display_bias_enable_VSP();
		printk("icnl9916 hx %s disable double wakeup\n", __func__);
	}
	MDELAY(5);	
	SET_RESET_PIN(0);	
	MDELAY(2);
	SET_RESET_PIN(1);
	MDELAY(8);
	if (chipone_gesture_wakeup_flag == 0) {
		display_bias_enable_VSN();
		printk("icnl9916 hx %s disable double wakeup\n", __func__);
	}
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(12);
	SET_RESET_PIN(1);
	MDELAY(12);   
   
	size = sizeof(init_setting) /
		sizeof(struct LCM_setting_table);
		
	push_table(NULL, init_setting, size, 1);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

struct LCM_DRIVER icnl9916_hdplus1600_dsi_vdo_hx_xe671_lcm_drv = {
	.name = "icnl9916_hdplus1600_dsi_vdo_hx_xe671",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};

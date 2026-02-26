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



static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;

//Begin add by bing-zhang for SNTTF-3211 on 2022/07/20
extern int display_bias_vsp_setting(void);
extern int display_bias_vsn_setting(void);
//End add by bing-zhang for SNTTF-3211 on 2022/07/20


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
#define FRAME_WIDTH			(720)
#define FRAME_HEIGHT			(1600)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH (74520)
#define LCM_PHYSICAL_HEIGHT (132480)
#define LCM_DENSITY (480)

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	//Begin add by bing-zhang for SNTTF-3211 on 2022/07/20
	{0xF0, 0x02, {0x5A,0x59} },
	{0xF1, 0x02, {0xA5,0xA6} },
	{0xBB, 0x08, {0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x12} },
	{0x6D, 0x01, {0x25} },
	//End add by bing-zhang for SNTTF-3211 on 2022/07/20
	{0x28, 0x02, {0x00,0x00} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0x02, {0x00,0x00} },
	{REGFLAG_DELAY, 70, {} },
	//Begin add by bing-zhang for SNTTF-3210 on 2022/07/20
	{0xC9, 0x02, {0x01,0x00} },
	//End add by bing-zhang for SNTTF-3210 on 2022/07/20
};

static struct LCM_setting_table init_setting[] = {
    {0xF0,2,{0x5A,0x59}},
    {0xF1,2,{0xA5,0xA6}},
    {0xD4,1,{0x31}},
    {0xC0,4,{0x40,0x93,0x00,0x1F}},
    {0xC1,32,{0x10,0x18,0x25,0x22,0x04,0x28,0x28,0x04,0x40,0x06,0x22,0x70,0x33,0x31,0x07,0x11,0x84,0x4C,0x00,0x93,0x1C,0x21,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xC4,32,{0x04,0x33,0xB8,0x40,0x00,0xBC,0x00,0xA0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0xE0,0x20,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xC5,32,{0x03,0x23,0x96,0xC8,0x32,0x00,0x05,0x02,0x12,0x0C,0x04,0x32,0x3F,0x08,0x01,0x10,0x04,0x0E,0xC8,0x00,0x0A,0x14,0x01,0x14,0x38,0x7F,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xC6,32,{0x89,0x24,0x17,0x2B,0x2B,0x28,0x3F,0x03,0x16,0x16,0x00,0x01,0x40,0x00,0x98,0x98,0x60,0x80,0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xC7,32,{0x76,0x54,0x32,0x22,0x34,0x56,0x77,0x75,0x10,0x76,0x54,0x32,0x22,0x34,0x56,0x77,0x75,0x10,0x31,0x00,0x01,0xFF,0xFF,0x00,0x0E,0x0E,0x43,0x00,0x00,0x00,0x00,0x00}},
	{0xC8,3,{0x47,0x00,0x48}},
    {0xCA,32,{0x00,0x40,0x00,0x19,0x46,0x94,0x41,0x8F,0x44,0x44,0x50,0x50,0x5A,0x5A,0x64,0x64,0x32,0x32,0x11,0x00,0x01,0x01,0x0A,0x06,0x22,0x00,0x05,0x00,0x00,0x64,0x32,0x04}},//VGHO=15V VGLO=-10V
    {0xB2,32,{0x11,0x11,0x10,0x10,0x44,0x44,0x05,0x85,0x44,0x70,0x71,0x70,0x71,0x70,0x71,0x70,0x71,0x70,0x71,0x70,0x71,0x00,0x00,0x00,0x00,0x55,0x55,0x10,0x20,0x11,0x00,0x00}},
    {0xB3,32,{0x73,0x01,0x01,0x01,0x81,0x00,0x78,0x4B,0x6B,0x15,0x13,0x00,0x08,0x44,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xB4,32,{0x73,0x01,0x01,0x01,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xB7,30,{0x00,0x00,0x08,0x08,0x00,0x00,0x12,0x12,0x10,0x10,0x0E,0x0E,0x0C,0x0C,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x3C,0x00}},
    {0xB6,30,{0x00,0x00,0x09,0x09,0x00,0x00,0x13,0x13,0x11,0x11,0x0F,0x0F,0x0D,0x0D,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x3C,0x00}},
    {0xBB,8,{0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x52}},
    {0xBC,15,{0x00,0x00,0x00,0x00,0x04,0x00,0xFF,0xF0,0x0B,0x13,0x50,0x5B,0x33,0x33,0x00}},
    {0xBD,6,{0xA1,0xA2,0x52,0x2E,0x00,0x8F}},
    {0xBE,16,{0x0C,0x88,0x43,0x38,0x33,0x00,0x00,0x38,0x00,0xB2,0xAF,0xB2,0xAF,0x00,0x00,0x33}},
    {0xBF,9,{0x0C,0x19,0x0C,0x19,0x00,0x11,0x04,0x18,0x50}},
    {0xFB,2,{0x5E,0x2C}},//VGH=16V VGL=-11V
    {0xF1,2,{0x5A,0x59}},
    {0xF0,2,{0xA5,0xA6}},
    {0x35,2,{0x00,0x00}},
		{0x11,2,{0x00,0x00}},
	  {REGFLAG_DELAY, 120, {} },
		{0x29,2,{0x00,0x00}},
	  {REGFLAG_DELAY, 50, {} },
	  {0x6D,2,{0x02,0x00}},
	  {REGFLAG_DELAY, 1, {} },

};

static void send_long_packet_data(struct LCM_setting_table *table,unsigned int count)
{
  unsigned int data_array[16];
  unsigned int num;
  unsigned int i;
  unsigned int sendnum,sendrow;
  unsigned char j,n;

  num=count;
  for(i=0;i<num;i++)
  {
   data_array[0] = 0;
   data_array[1] = 0;
   sendnum=table[i].count;
   if(table[i].cmd!=REGFLAG_DELAY)
   {
	 if(sendnum<=1)
	 {
		data_array[0]=0x00023902;
		data_array[1]=table[i].para_list[0]<<8 | table[i].cmd;
		dsi_set_cmdq(data_array, 2, 1);
	 }else if(sendnum>1)
     {
		data_array[0]=0x00003902|(sendnum+1)<<16;
		data_array[1]=table[i].cmd;
	    if((sendnum+1)<=4)
			{
			for(j=0;j<sendnum;j++)
				{
				 data_array[1]|=table[i].para_list[j]<<(8*(j+1));
				}
			dsi_set_cmdq(data_array, 2, 1);
			}else{
			     data_array[1]|=table[i].para_list[0]<<8|table[i].para_list[1]<<16|table[i].para_list[2]<<24;
			     if((sendnum-3)%4)
				 {
				  sendrow=(sendnum-3)/4+1;
				 }else{
				  sendrow=(sendnum-3)/4;
				 }
				 for(j=2;j<sendrow+2;j++)
				  {
					data_array[j]=0;
					for(n=0;n<4;n++)
					{
					data_array[j]|=table[i].para_list[n+4*(j-1)-1]<<(n*8);
					}
					//printk("pige thr data_array[%d]=0x%08x\n",j,data_array[j]);
				  }
				  dsi_set_cmdq(data_array, sendrow+2, 1);
			}
     }
    }else{
	      MDELAY(table[i].count);
	}
  }
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

//Begin add by bing-zhang for SNTTF-3211 on 2022/07/20
static struct LCM_setting_table lcm_init_code_prev_setting[] = {
	{0xF0,0x02,{0x5A,0x59}},
	{0xF1,0x02,{0xA5,0xA6}},
	{0xBB,0x08,{0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x52}},
};

static void lcm_reset()
{
	SET_RESET_PIN(1);
	MDELAY(7);
	SET_RESET_PIN(0);
	MDELAY(7);
	SET_RESET_PIN(1);
	MDELAY(15);
}
//End add by bing-zhang for SNTTF-3211 on 2022/07/20

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
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/
	dfps_params[0].vertical_frontporch = 313;//215
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
	/*dfps_params[0].PLL_CLOCK = xx;*/
	/*dfps_params[0].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[0].horizontal_frontporch = xx;*/
	dfps_params[1].vertical_frontporch = 1240;
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

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;


#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	lcm_dsi_mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
	pr_debug("[LCM]%s lcm_dsi_mode %d\n", __func__, lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

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
	params->dsi.vertical_backporch = 24;
	params->dsi.vertical_frontporch = 313; //215
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 48;
	params->dsi.horizontal_frontporch = 48;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	
	
	params->dsi.PLL_CLOCK = 457;
	params->dsi.ssc_disable = 1; /* ssc will decrease fps */
	params->dsi.ssc_range = 5;
	params->dsi.noncont_clock = 1;
	params->dsi.clk_lp_per_line_enable = 0;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

	params->dsi.LPX = 6;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 0;
	params->corner_pattern_width = 1080;
	params->corner_pattern_height = 32;
	params->corner_pattern_height_bot = 32;
#endif
	
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
#endif
}
#if 0
static void lcm_init_power(void)
{
	display_bias_enable();
}
#endif
static void lcm_suspend_power(void)
{
	MDELAY(10);
	display_bias_disable();
}
#if 0
static void lcm_resume_power(void)
{
	SET_RESET_PIN(0);
	display_bias_enable();
}
#endif

static void lcm_init(void)
{
	//Begin add by bing-zhang for SNTTF-3211 on 2022/07/20
	printk("lcm_init begin\n");
	display_bias_regulator_init();/* vsp/vsn enable */
	display_bias_vsp_setting();/* Set vsp to 5.85v*/

	lcm_reset();
	//push_table(NULL, lcm_init_code_prev_setting, sizeof(lcm_init_code_prev_setting)/sizeof(struct LCM_setting_table), 1);
	send_long_packet_data(lcm_init_code_prev_setting,sizeof(lcm_init_code_prev_setting)/sizeof(struct LCM_setting_table));
	display_bias_vsn_setting();/* Set vsn to -5.85v*/
	lcm_reset();
	printk("lcm_init end\n");
	//End add by bing-zhang for SNTTF-3211 on 2022/07/20
	//push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	send_long_packet_data(init_setting,sizeof(init_setting)/sizeof(struct LCM_setting_table));
}

static void lcm_suspend(void)
{
	//Begin add by bing-zhang for SNTTF-3211 on 2022/07/20
	printk("lcm_suspend begin\n");
	/*push_table(NULL, lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
		1);*/
	send_long_packet_data(lcm_suspend_setting,sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table));
	MDELAY(150);
	SET_RESET_PIN(1);
	lcm_suspend_power();/* Disable vsp/vsn */
	printk("lcm_suspend end\n");
	//End add by bing-zhang for SNTTF-3211 on 2022/07/20
}

static void lcm_resume(void)
{
	lcm_init();
}

struct LCM_DRIVER sonata_icnl9916_hdp_dsi_vdo_lcm_drv = {
	.name = "sonata_icnl9916_hdp_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	//.init_power = lcm_init_power,
	//.resume_power = lcm_resume_power,
	//.suspend_power = lcm_suspend_power,

};

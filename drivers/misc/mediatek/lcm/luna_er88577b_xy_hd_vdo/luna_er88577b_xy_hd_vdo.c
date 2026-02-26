// SPDX-License-Identifier: GPL-2.0

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include "disp_dts_gpio.h"

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
		
#define set_gpio_lcd_enn(cmd) \
	lcm_util.set_gpio_lcd_enn_bias(cmd)
#define set_gpio_lcd_enp(cmd) \
	lcm_util.set_gpio_lcd_enp_bias(cmd)

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
#define FRAME_WIDTH			(800)
#define FRAME_HEIGHT			(1280)

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

#ifdef BUILD_LK
#define LCD_DEBUG(fmt, args...) printf(fmt, ##args)
#else
#define LCD_DEBUG(fmt, args...) printk(fmt, ##args)
#endif

extern int tps65132_write_bytes(unsigned char addr, unsigned char value);

extern int tct_lcm_pmu_bias_set(int min_uV, int max_uV);
extern int tct_lcm_pmu_bias_unset(void);

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table  lcm_initialization_setting_ty[] = {
{0xE0,2,{0xAB,0xBA}},
{0xE1,2,{0xBA,0xAB}},
{0xB1,4,{0x10,0x01,0x47,0xFF}},
{0xB2,6,{0x0C,0x14,0x04,0x50,0x50,0x14}},
{0xB3,3,{0x56,0xD3,0x00}},
{0xB4,3,{0x22,0x30,0x04}},
{0xB6,7,{0x00,0x00,0x00,0x10,0x00,0x10,0x00}},
{0xB8,5,{0x05,0x12,0x29,0x49,0x48}},
{0xB9,38,{0x7C,0x6F,0x63,0x56,0x54,0x47,0x4C,0x36,0x50,0x4F,
		  0x4E,0x6A,0x55,0x5A,0x4A,0x48,0x37,0x24,0x09,0x7C,
		  0x6F,0x63,0x56,0x54,0x47,0x4C,0x36,0x50,0x4F,0x4E,
		  0x6A,0x55,0x5A,0x4A,0x48,0x37,0x24,0x09}},
{0xC0,16,{0x32,0x23,0x67,0x67,0x33,0x33,0x33,0x33,0x10,0x04,0x90,0x04,0x3F,0x00,0x00,0xC0}},
{0xC1,10,{0x13,0x14,0x02,0x8D,0x10,0x04,0x90,0x04,0x54,0x00}},
{0xC2,12,{0x37,0x09,0x08,0x89,0x88,0x21,0x22,0x21,0x44,0xBB,0x18,0x00}},
{0xC3,22,{0x86,0x40,0x00,0x08,0x1F,0x1E,0x02,0x16,0x14,0x02,0x12,0x10,0x02,0x0E,0x0C,0x04,0x02,0x02,0x02,0x02,0x02,0x02}},
{0xC4,22,{0x07,0x00,0x00,0x09,0x1F,0x1E,0x02,0x17,0x15,0x02,0x13,0x11,0x02,0x0F,0x0D,0x05,0x02,0x02,0x02,0x02,0x02,0x02}},
{0xC8,6,{0x61,0x00,0x32,0x40,0x54,0x16}},
{0xCA,2,{0x4B,0x43}},
{0xCD,8,{0x0E,0x64,0x64,0x25,0x1E,0x6B,0x06,0x83}},
{0xD2,4,{0xE3,0x2B,0x38,0x00}},
{0xD4,11,{0x00,0x01,0x00,0x0E,0x04,0x44,0x08,0x10,0x00,0x07,0x00}},
{0xE6,8,{0x00,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
{0xF0,5,{0x12,0x03,0x20,0x00,0xFF}},
{0xF3,1,{0x00}},



        // Sleep Mode On
    {0x11,1,{0x00}},
    {REGFLAG_DELAY,130,{0x00}},
    {0x29,1,{0x00}},
    {REGFLAG_DELAY,30,{0x00}},
    {REGFLAG_END_OF_TABLE,1,{0x00}},
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




static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}





static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
//begin modified by xiongbo.huang for desity
     params->physical_width = 107;
     params->physical_height = 172;
//end modified by xiongbo.huang for desity
    // enable tearing-free
    //params->dbi.te_mode           = LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_edge_polarity      = LCM_POLARITY_RISING;
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;


    params->dsi.mode   = BURST_VDO_MODE;


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

    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    //params->dsi.word_count=480*3;

    //here is for esd protect by legen
    //params->dsi.noncont_clock = true;
    //params->dsi.noncont_clock_period=2;
    params->dsi.lcm_ext_te_enable=false;
    //for esd protest end by legen
    //params->dsi.word_count=FRAME_WIDTH*3;
    params->dsi.vertical_sync_active = 4;
    params->dsi.vertical_backporch = 12;
    params->dsi.vertical_frontporch = 20;
    //params->dsi.vertical_frontporch_for_low_power = 620;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 20;
    params->dsi.horizontal_backporch = 60;
    params->dsi.horizontal_frontporch = 80;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.ssc_disable = 1;
//begin modified by xiongbo.huang fto set fps to 60 temprorily, will change this value according to hardware later
    params->dsi.PLL_CLOCK = 230;
//end add by xiongbo.huang.
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;

    /*params->dsi.lcm_esd_check_table[1].cmd = 0xAC;
    params->dsi.lcm_esd_check_table[1].count = 1;
    params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;*/

    //params->dsi.fbk_div =30;              // fref=26MHz,  fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)   //32

        params->dsi.cont_clock  = 0;//noncontinue mode for dsi clk

        #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        params->round_corner_en = 0;
        params->corner_pattern_width = 720;//Note:ÕâÀïÊÇÆÁÄ»µÄ¿í¶È£¬²»ÊÇÔ­Ê¼Í¼Æ¬¿í¶È
        params->corner_pattern_height = 16;//Ô²½ÇµÄ¸ß¶È
        #endif
//end 20180416 liujunting add for round corner
}


static void lcm_resume_power(void)
{
#ifdef CONFIG_MT6370_PMU_DSV
	//display_bias_enable();
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_VDD_OUT1);
	MDELAY(10);
	tct_lcm_pmu_bias_set(5500000,5500000);
#else
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x11; //vsp 5.7v
	set_gpio_lcd_enp(0);
	set_gpio_lcd_enn(0);
	MDELAY(5);

	set_gpio_lcd_enp(1);
	MDELAY(5);

	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
		LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);

	MDELAY(5);
	//lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	set_gpio_lcd_enn(1);
	MDELAY(5);
	cmd=0x01;
	data=0x11; //vsp 5.7v
	//VPS=0x00;data=0x0A;VSP=5V,
	//         data=0x0E;VSP=5.4V,
	//	   data=0x12;VSP=5.8V,
	//VNG=0x01;data=0x0A;VNG=-5V,
	//         data=0x0E;VNG=-5.4V,
	//	   data=0x12;VSP=-5.8V,

	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
		LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
		LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
}

static void lcm_init_power(void)
{
#ifdef CONFIG_MT6370_PMU_DSV
	//display_bias_enable();

	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_VDD_OUT1);
	MDELAY(10);
	tct_lcm_pmu_bias_set(5500000,5500000);
#else
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x11; //vsp 5.7v
	set_gpio_lcd_enp(0);
	set_gpio_lcd_enn(0);
	MDELAY(5);

	set_gpio_lcd_enp(1);
	MDELAY(5);

	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
		LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);

	MDELAY(5);
	//lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	set_gpio_lcd_enn(1);
	MDELAY(5);
	cmd=0x01;
	data=0x11; //vsp 5.7v
	//VPS=0x00;data=0x0A;VSP=5V,
	//         data=0x0E;VSP=5.4V,
	//	   data=0x12;VSP=5.8V,
	//VNG=0x01;data=0x0A;VNG=-5V,
	//         data=0x0E;VNG=-5.4V,
	//	   data=0x12;VSP=-5.8V,

	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
		LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
		LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
}
static void lcm_init(void)
{
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_VDD_OUT1);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(30);
	
	push_table(NULL,lcm_initialization_setting_ty, sizeof(lcm_initialization_setting_ty) / sizeof(struct LCM_setting_table), 1);
}

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Sleep Mode On
    {0x28,1,{0x00}},
    {REGFLAG_DELAY,20,{0x00}},
    {0x10,1,{0x00}},
    {REGFLAG_DELAY,120,{0x00}},
    {REGFLAG_END_OF_TABLE,1,{0x00}},
};

static void lcm_suspend(void)
{
	push_table(NULL,lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
#ifdef CONFIG_MT6370_PMU_DSV
	printk("#### %s | %d ####\n",__func__,__LINE__);
	tct_lcm_pmu_bias_unset();
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_VDD_OUT0);
#else
	set_gpio_lcd_enn(1);
	set_gpio_lcd_enn(0);
	MDELAY(20);
	set_gpio_lcd_enp(1);
	set_gpio_lcd_enp(0);
    MDELAY(10);
#endif
}

static void lcm_resume(void)
{
	lcm_init();
}

struct LCM_DRIVER luna_er88577b_xy_hd_vdo_lcm_drv = {
	.name = "luna_er88577b_xy_hd_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	//.suspend_power = lcm_suspend_power,

};

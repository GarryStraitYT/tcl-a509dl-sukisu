
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
#ifndef MACH_FPGA
#include <lcm_pmic.h>
#endif
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

//#include <cust_gpio_usage.h>


#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))


/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)

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
#include "disp_dts_gpio.h"
#endif


//static const unsigned char LCD_MODULE_ID = 0x01;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE                                    1
#define FRAME_WIDTH                                     (1080)
#define FRAME_HEIGHT                                    (2400)
#define PHYSICAL_WIDTH      				  (71)
#define PHYSICAL_HIGHT       				  (157)


#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

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

//#define CONFIG_BIAS_TPS65132_I2C
#if defined(CONFIG_BIAS_TPS65132_I2C)
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
/* i2c control start */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
static struct i2c_client *_lcm_i2c_client;
static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int _lcm_i2c_remove(struct i2c_client *client);
struct _lcm_i2c_dev {
	struct i2c_client *client;
};

static const struct i2c_device_id _lcm_i2c_id[] = {
	{ LCM_I2C_ID_NAME, 0 },
	{}
};

static int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = _lcm_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("[LCM][ERROR] _lcm_i2c write data fail !!\n");

	LCM_LOGD("%s()- addr=0x%x, value=0x%x", __func__, addr, value);
	return ret;
}

static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_info("[LCM][I2C] %s\n", __func__);
	pr_info("[LCM][I2C] NT: info==>name=%s addr=0x%x\n", client->name,
		client->addr);
	_lcm_i2c_client = client;
	/* set +- 5.6V for Bias voltage */
	_lcm_i2c_write_bytes(0x01, 0x10);
	_lcm_i2c_write_bytes(0x02, 0x10);
	return 0;
}

static int _lcm_i2c_remove(struct i2c_client *client)
{
	pr_info("[LCM][I2C] %s\n", __func__);
	_lcm_i2c_client = NULL;

	i2c_unregister_device(client);
	return 0;
}

static const struct of_device_id of_leds_i2c_match[] = {
	{ .compatible = "mediatek,i2c_lcd_bias", },
	{},
};
MODULE_DEVICE_TABLE(of, of_leds_i2c_match);

static struct i2c_driver _lcm_i2c_driver = {
	.id_table = _lcm_i2c_id,
	.probe = _lcm_i2c_probe,
	.remove = _lcm_i2c_remove,
	/* .detect		   = _lcm_i2c_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name = LCM_I2C_ID_NAME,
		.of_match_table = of_leds_i2c_match,
	},
};

module_i2c_driver(_lcm_i2c_driver);
#endif

static struct LCM_setting_table lcm_suspend_setting[] = {
    {0x28, 0, {} },
    {REGFLAG_DELAY, 20, {} },
    {0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
    {0xFF, 0x03, {0x78,0x35,0x06}},
    {0xC5, 0x01, {0x55}},//for deep sleep in
    {REGFLAG_DELAY, 100, {} },
    {REGFLAG_END_OF_TABLE,1,{0x00}},
};

static struct LCM_setting_table lcm_resume_setting[] = {
    //----------------------LCD initial code start----------------------//

	
	{0xFF, 0x03, {0x78,0x35,0x06}},
	{0x45, 0x01, {0x4C}},
	{0xC7, 0x01, {0x00}},

	{0xFF, 0x03, {0x78,0x35,0x09}},//ELVSS_120nit_-2.5V
	{0x92, 0x01, {0x02}},
	{0x95, 0x01, {0x53}},

	{0xE1, 0x01, {0x00}},	
	{0xE2, 0x01, {0x07}},
	{0xE3, 0x01, {0x07}},	
	{0xE4, 0x01, {0x07}},
	{0xE5, 0x01, {0x07}},	
	{0xE6, 0x01, {0x00}},
	{0xE7, 0x01, {0x00}},	
	{0xE8, 0x01, {0x00}},
	{0xE9, 0x01, {0x00}},

   	{0xFF, 0x03, {0x78,0x35,0x11}},
	{0x94, 0x01, {0x4F}},//79 PMIC FD ON
	{0x98, 0x01, {0x50}},//80 PMIC FD OFF

  	{0x96, 0x01, {0x1D}},
   	{0x9A, 0x01, {0x1D}},    //ELVSS=-3.2V
	{0xFF, 0x03, {0x78,0x35,0x00}},
	{0x51, 0x02, {0x00,0x00}},
	{0x53, 0x01, {0x24}},
	{0x35, 0x01, {0x00}},

//----------------------LCD initial code End----------------------//
//SLPOUT and DISPON

	{0x11, 0x01, {0x00}},
	{REGFLAG_DELAY, 120, {} },
	{0x29, 0x01, {0x00}},

	{REGFLAG_DELAY, 50, {} },
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

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


static void lcm_get_params(struct LCM_PARAMS *params)
{
    memset(params, 0, sizeof(struct LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
//add by jingqing.yan for physical width
   params->physical_width = PHYSICAL_WIDTH;
   params->physical_height = PHYSICAL_HIGHT;
	/* enable tearing-free */
	//params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	//params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode_enable = 0;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq           = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding             = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 4;
	params->dsi.vertical_frontporch = 8;
	//params->dsi.vertical_frontporch_for_low_power = 840;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 28;
	params->dsi.horizontal_frontporch = 32;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    //params->dsi.customization_esd_check_enable = 1;

    //params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
    //params->dsi.lcm_esd_check_table[0].count        = 1;
    //params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

    /*params->dsi.lcm_esd_check_table[2].cmd          = 0x0F;
    params->dsi.lcm_esd_check_table[2].count        = 1;
    params->dsi.lcm_esd_check_table[2].para_list[0] = 0xC0;*/

    //params->dsi.fbk_div =30;              // fref=26MHz,  fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)   //32

    //begin modify by jingqing.yan for task 8943891 on 2020/02/24
 	params->dsi.PLL_CLOCK = 560;
    //end modify by kun.zheng for task 8943891 on 2020/02/24
	params->dsi.ssc_disable = 1; /* ssc will decrease fps */
	params->dsi.ssc_range = 5;
	params->dsi.noncont_clock = 1;
	params->dsi.clk_lp_per_line_enable = 0;

	/* ESD setting */
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

}






static void lcm_init(void)
{
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENN1);//enable VCI
    MDELAY(10);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
    MDELAY(1);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
    MDELAY(10);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
    MDELAY(37);
    push_table(NULL,lcm_resume_setting, sizeof(lcm_resume_setting) / sizeof(struct LCM_setting_table), 1);

}

static void lcm_suspend(void)
{

    push_table(NULL,lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
   // disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
   // MDELAY(2);
    //disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENN0);//disable VCI
   // SET_RESET_PIN(0);
   // MDELAY(10);
}

static void lcm_resume(void)
{
    lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	data_array[3] = 0x00053902;
	data_array[4] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[5] = (y1_LSB);
	data_array[6] = 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}

static unsigned int lcm_compare_id(void)
{
//         int id_type=0;

// #if 0
//     mt_set_gpio_mode(GPIO_DISP_RST_PIN,GPIO_MODE_00);
//     mt_set_gpio_dir(GPIO_DISP_RST_PIN, GPIO_DIR_OUT);
//     //mt_set_gpio_pull_select(GPIO_LCM_ID1,GPIO_PULL_DOWN);
//     mt_set_gpio_pull_enable(GPIO_DISP_RST_PIN, GPIO_PULL_DISABLE);// def 0
// #endif
//     mt_set_gpio_mode(GPIO_DISP_ID0_PIN,GPIO_MODE_00);
//     mt_set_gpio_dir(GPIO_DISP_ID0_PIN, GPIO_DIR_IN);
//     //mt_set_gpio_pull_select(GPIO_LCM_ID1,GPIO_PULL_DOWN);
//     mt_set_gpio_pull_enable(GPIO_DISP_ID0_PIN, GPIO_PULL_DISABLE);// def 0

//     mt_set_gpio_mode(GPIO_DISP_ID1_PIN,GPIO_MODE_00);
//     mt_set_gpio_dir(GPIO_DISP_ID1_PIN, GPIO_DIR_IN);
//     //mt_set_gpio_pull_select(GPIO_LCM_ID2,GPIO_PULL_DOWN);
//     mt_set_gpio_pull_enable(GPIO_DISP_ID1_PIN, GPIO_PULL_DISABLE);

//     MDELAY(10);
//     id_type = mt_get_gpio_in(GPIO_DISP_ID0_PIN)<<1 | mt_get_gpio_in(GPIO_DISP_ID1_PIN);

//     LCM_LOGI("[LCM] TDT ID_type is: %d\n",id_type);
//   /*  if (id_type == 0)
//             return 1;
//     else
//         return 0;*/
 	return 1;
}
static struct LCM_setting_table bl_level[] = {
	{0xFF, 0x03, {0x78,0x35,0x00}},
	{0x51, 0x02, {0x03,0xFF}},

	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

#if defined(CONFIG_TCT_FEATURE_BACKLIGHT_MAPPING)&& defined(CONFIG_TCT_PROJECT_PASSAT)
static struct LCM_setting_table enable_hbm[] = {

	{0xFF,3,{0x78,0x35,0x00}}, //Page 0
	{0x53,1,{0xEC}},	 //enter HBM
	{0x94,1,{0x2F}},   //ELVSS=3.8V
};

static struct LCM_setting_table disable_hbm[] = {

	{0xFF,3,{0x78,0x35,0x00}},//Page 0
	{0x53,1,{0x24}},     //exit HBM
	{0x94,1,{0x00}},    //ELVSS=-3.2V	

};
#endif

#if defined(CONFIG_TCT_FEATURE_AOD_FUNCTION)
static struct LCM_setting_table enter_idle[] = {
	{0xFF,3,{0x78,0x35,0x02}},
	{0x47,1,{0x3F}},
	{0xFF,3,{0x78,0x35,0x00}}, //Page 0
	{0x53,1,{0x2E}},  //AOD on	
	{REGFLAG_END_OF_TABLE, 0x00, {} }

};

static struct LCM_setting_table leave_idle[] = {

	{0xFF,3,{0x78,0x35,0x00}},//Page 0
	{0x53,1,{0x24}},  //AOD 0ff	
	{REGFLAG_END_OF_TABLE, 0x00, {} }

};

static void set_lcm_aod_mode(int enter,void *qhandle)
{
   printk("AOD_mode 3 enter = %d\n",enter);
   if(enter >= 1)
	{
	printk("AOD_mode enter\n");
       	push_table(qhandle,enter_idle, sizeof(enter_idle) / sizeof(struct LCM_setting_table), 1);	
	} 
   else 
	{
	printk("AOD_mode exit\n");
      	push_table(qhandle,leave_idle, sizeof(leave_idle) / sizeof(struct LCM_setting_table), 1);
	}

}
#endif

#if defined(CONFIG_TCT_FEATURE_BACKLIGHT_MAPPING)&& defined(CONFIG_TCT_PROJECT_PASSAT)
extern int flag_hbm_enable; //flag for hbm
extern int flag_tct_hbm_mode;
#endif

#if defined(CONFIG_TCT_FEATURE_AOD_FUNCTION)
extern int tct_pgc_pm;
extern int tct_pgc_lcm_ps;
#endif
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
    unsigned char mapped_level_MSB = 0x00;
	unsigned char mapped_level_LSB = 0x00;
	#if defined(CONFIG_TCT_FEATURE_AOD_FUNCTION)
	printk("tcl tct_pgc_pm=%d,tct_pgc_lcm_ps=%d\n",tct_pgc_pm,tct_pgc_lcm_ps);
	if (tct_pgc_pm == 3) {
  		if ( tct_pgc_lcm_ps == 2) {
			printk("tcl aod mode enter =%d\n",level);
			push_table(handle,enter_idle, sizeof(enter_idle) / sizeof(struct LCM_setting_table), 1);
			return;
		}
	}
	#endif
		//for LGE backlight IC mapping table
	#if defined(CONFIG_TCT_FEATURE_BACKLIGHT_MAPPING)&& defined(CONFIG_TCT_PROJECT_PASSAT)
	if ((flag_hbm_enable == 1)&&(flag_tct_hbm_mode == 0)) //temp set at 3000
	{
		printk("tcl enter HBM mode\n");
		flag_tct_hbm_mode = 1;
		push_table( handle,enable_hbm,
		sizeof(enable_hbm) / sizeof(struct LCM_setting_table), 1);//enable HBM mode
		return;
	}
	else if ((flag_hbm_enable == 1) &&(flag_tct_hbm_mode == 1))
	{
		printk("tcl in HBM mode,do nothing\n");
		return;
	}
	else if((flag_hbm_enable == 0) &&(flag_tct_hbm_mode == 1))
	{
		printk("tcl exit HBM mode");
		flag_tct_hbm_mode = 0;
		push_table( handle,disable_hbm, sizeof(disable_hbm) / sizeof(struct LCM_setting_table), 1);

		return;
	}
	else {
	#endif
        if (level > 1023)
		{
		 mapped_level_MSB = 0x03;
		 mapped_level_LSB = 0xFF;
		}

	else if (level > 0)
		{
		 mapped_level_MSB = ((level >> 8) & 0xFF);
		 mapped_level_LSB = (level & 0xFF);
		}
	else 
		{
		 mapped_level_MSB = 0x00;
		 mapped_level_LSB = 0x00;
		}
	printk("tcl BBBBv3_no led custom level = %d, mapped_level_MSB = 0x%x,mapped_level_LSB = 0x%x\n",level,mapped_level_MSB,mapped_level_LSB);
	// Refresh value of backlight level.
	bl_level[1].para_list[0] = mapped_level_MSB;
	bl_level[1].para_list[1] = mapped_level_LSB;

	push_table( handle,bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	#if defined(CONFIG_TCT_FEATURE_BACKLIGHT_MAPPING)&& defined(CONFIG_TCT_PROJECT_PASSAT)
	}
	#endif

}


struct LCM_DRIVER model_3_ili7835_hehui_edo_fhdplus_amoled_cmd_lcm_drv = {
    .name = "model_3_ili7835_hehui_edo_fhdplus_amoled_cmd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .compare_id = lcm_compare_id,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.update = lcm_update,
#if defined(CONFIG_TCT_FEATURE_AOD_FUNCTION)
	.aod = set_lcm_aod_mode,
#endif
};

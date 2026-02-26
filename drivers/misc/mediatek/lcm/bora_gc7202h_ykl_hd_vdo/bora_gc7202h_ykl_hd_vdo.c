/*History modified by xiongbo.huang for least_resume time (8423451) on 2019/10/11*/
#ifndef CONFIG_MTK_LCM_DEVICE_TREE_SUPPORT
#ifndef BUILD_LK
    #include <linux/string.h>
    #include <linux/kernel.h>
#else
#include <string.h>
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
#else
    //#include <mt-plat/mt_gpio.h>
    //#include <mach/gpio_const.h>
#endif

//min.luo defect 8249117:[SWD_TEST]ctp resume faster 20190813
//extern void lcd_queue_load_tp_fw(void);

#ifndef  GTP_RST_PORT
#define GTP_RST_PORT    174
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt, args...) printf(fmt, ##args)
#else
#define LCD_DEBUG(fmt, args...) printk(fmt, ##args)
#endif

extern int tps65132_write_bytes(unsigned char addr, unsigned char value);


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1440)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef GPIO_LCD_BIAS_ENP_PIN
#define GPIO_LCD_BIAS_ENP_PIN        			(GPIO173 | 0x80000000)
#endif
#ifndef GPIO_LCD_BIAS_ENN_PIN
#define GPIO_LCD_BIAS_ENN_PIN        			(GPIO171 | 0x80000000)
#endif
#define GPIO_LCD_RST_PIN               		(GPIO45 | 0x80000000)

//min.luo defect 8597127:[SWD_TEST]optimize lcd cmd driver 20191113  start
#define REGFLAG_END_OF_TABLE		(0xFFFD)
#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
//min.luo defect 8597127:[SWD_TEST]optimize lcd cmd driver 20191113  end
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static struct LCM_UTIL_FUNCS lcm_util = {0};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define SET_RESET_PIN(v)    				(lcm_util.set_reset_pin((v)))
#define MDELAY(n)					(lcm_util.mdelay(n))
#define UDELAY(n)					(lcm_util.udelay(n))
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enn(cmd) \
	lcm_util.set_gpio_lcd_enn_bias(cmd)
#define set_gpio_lcd_enp(cmd) \
	lcm_util.set_gpio_lcd_enp_bias(cmd)
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table  lcm_initialization_setting_ty[] = {
 
{0xFF, 3, {0x55,0xAA,0x66 }},

{0xFF, 1, {0xB3 }},
{0x2B, 1, {0x0C }},
{0x29, 1, {0x3F }},
{0x28, 1, {0xC0 }},
{0x2A, 1, {0x03 }},
{0x68, 1, {0x0F }},

{0xFF, 1, {0x10 }}, 
{0xFB, 1, {0x00 }}, 
{0xFF, 1, {0x20 }},  
{0xFB, 1, {0x00 }},  
{0xFF, 1, {0x21 }},  
{0xFB, 1, {0x00 }},  
{0xFF, 1, {0x22 }},  
{0xFB, 1, {0x00 }},  
{0xFF, 1, {0x23 }},  
{0xFB, 1, {0x00 }},  
{0xFF, 1, {0x24 }},  
{0xFB, 1, {0x00 }}, 
{0xFF, 1, {0x27 }},  
{0xFB, 1, {0x00 }},  
{0xFF, 1, {0x26 }},  
{0xFB, 1, {0x00 }},   
{0xFF, 1, {0x28 }},  
{0xFB, 1, {0x00 }},  
{0xFF, 1, {0xB3 }},  
{0xFB, 1, {0x00 }},  

{0xFF, 1, {0x28 }},
{0x53, 1, {0x44 }},
{0x50, 1, {0x4C }},
{0x52, 1, {0x52 }},
{0x4D, 1, {0xA2 }},
{0xFF, 1, {0x20 }},
{0xA5, 1, {0x00 }},
{0xA6, 1, {0xFF }},
{0xA9, 1, {0x00 }},
{0xAA, 1, {0xFF }},
{0xD3, 1, {0x06 }},
{0x2D, 1, {0x1F }},
{0x2E, 1, {0x42 }},
{0x2F, 1, {0x14 }},
{0xFF, 1, {0x22 }},
{0x1f, 1, {0x06 }},
{0xFF, 1, {0xB3 }},
{0x3E, 1, {0x03 }},
{0x58, 1, {0x84 }},
{0x53, 1, {0x1A }},
{0x82, 1, {0x19 }},
{0xFF, 1, {0xB3 }}, 
{0x78, 1, {0x01 }}, 
{0x4A, 1, {0x0F }},
{0x7D, 1, {0x80 }}, 
{0x5B, 1, {0x4B }},
{0x48, 1, {0x26 }},
{0x7C, 1, {0x8C }}, 

{0xFF, 1, {0x20 }}, 
{0xA3, 1, {0x45 }}, 
{0xA7, 1, {0x45 }}, 
{0xFF, 1, {0xB3 }},       
{0x3F, 1, {0x37 }},
{0x5E, 1, {0x10 }},

{0xFF, 1, {0x22 }},
{0xE4, 1, {0x00 }},
{0x01, 1, {0x05 }},
{0x02, 1, {0xA0 }},
{0x25, 1, {0x08 }},
{0x26, 1, {0x00 }},
{0x2E, 1, {0x6F }},
{0x2F, 1, {0x00 }},
{0x36, 1, {0x0D }},
{0x37, 1, {0x00 }},
{0x3F, 1, {0x6F }},
{0x40, 1, {0x00 }},

{0xFF, 1, {0x28 }},
{0x01, 1, {0x00 }},
{0x02, 1, {0x02 }},
{0x03, 1, {0x1A }},
{0x04, 1, {0x1A }},
{0x05, 1, {0x04 }},
{0x06, 1, {0x20 }},
{0x07, 1, {0x21 }},
{0x08, 1, {0x08 }},
{0x09, 1, {0x0A }},
{0x0A, 1, {0x0C }},
{0x0B, 1, {0x0E }},
{0x0C, 1, {0x10 }},
{0x0D, 1, {0x12 }},
{0x0E, 1, {0x25 }},
{0x0F, 1, {0x25 }},
{0x10, 1, {0x25 }},
{0x11, 1, {0x25 }},
{0x12, 1, {0x25 }},
{0x13, 1, {0x25 }},
{0x14, 1, {0x25 }},
{0x15, 1, {0x25 }},
{0x16, 1, {0x25 }},
{0x17, 1, {0x01 }},
{0x18, 1, {0x03 }},
{0x19, 1, {0x1A }},
{0x1A, 1, {0x1A }},
{0x1B, 1, {0x05 }},
{0x1C, 1, {0x20 }},
{0x1D, 1, {0x21 }},
{0x1E, 1, {0x09 }},
{0x1F, 1, {0x0B }},
{0x20, 1, {0x0D }},
{0x21, 1, {0x0F }},
{0x22, 1, {0x11 }},
{0x23, 1, {0x13 }},
{0x24, 1, {0x25 }},
{0x25, 1, {0x25 }},
{0x26, 1, {0x25 }},
{0x27, 1, {0x25 }},
{0x28, 1, {0x25 }},
{0x29, 1, {0x25 }},
{0x2A, 1, {0x25 }},
{0x2B, 1, {0x25 }},
{0x2D, 1, {0x25 }},
{0x30, 1, {0x00 }},
{0x31, 1, {0x00 }},
{0x32, 1, {0x00 }},
{0x33, 1, {0x05 }},
{0x34, 1, {0x00 }},
{0x35, 1, {0x15 }},
{0x38, 1, {0x00 }},
{0x39, 1, {0x00 }},
{0x3A, 1, {0x03 }},
{0x3B, 1, {0x05 }},
{0x2F, 1, {0x1D }},
{0xFF, 1, {0x21 }},
{0x7E, 1, {0x07 }},
{0x7F, 1, {0x24 }},
{0x8B, 1, {0x24 }},
{0x97, 1, {0x24 }},
{0x80, 1, {0x04 }},
{0x8C, 1, {0x00 }},
{0x98, 1, {0x1B }},
{0x81, 1, {0x1B }},
{0x8D, 1, {0x00 }},
{0x99, 1, {0x04 }},
{0xAF, 1, {0x42 }},
{0xB0, 1, {0x42 }},
{0xB1, 1, {0x42 }},
{0x83, 1, {0x05 }},
{0x8F, 1, {0x05 }},
{0x9B, 1, {0x05 }},
{0x84, 1, {0x93 }},
{0x90, 1, {0x93 }},
{0x9C, 1, {0x93 }},
{0x85, 1, {0x93 }},
{0x91, 1, {0x93 }},
{0x9D, 1, {0x93 }},
{0x9D, 1, {0x93 }},
{0x87, 1, {0x04 }},
{0x93, 1, {0x08 }},
{0x9F, 1, {0x0C }},
{0x82, 1, {0x70 }},
{0x8E, 1, {0x70 }},
{0x9A, 1, {0x70 }},
{0x2B, 1, {0x00 }},
{0x2E, 1, {0x00 }},
{0x88, 1, {0xB7 }},
{0x89, 1, {0x20 }},
{0x8A, 1, {0x23 }},
{0x94, 1, {0xB7 }},
{0x95, 1, {0x20 }},
{0x96, 1, {0x23 }},
{0xA0, 1, {0xB7 }},
{0xA1, 1, {0x20 }},
{0xA2, 1, {0x23 }},
{0x45, 1, {0x3F }},
{0x46, 1, {0x74 }},
{0x4C, 1, {0x74 }},
{0x52, 1, {0x74 }},
{0x58, 1, {0x74 }},
{0x5E, 1, {0x74 }},
{0x64, 1, {0x74 }},
{0x47, 1, {0x05 }},
{0x4D, 1, {0x04 }},
{0x53, 1, {0x2B }},
{0x59, 1, {0x2A }},
{0x48, 1, {0x2A }},
{0x4E, 1, {0x2B }},
{0x54, 1, {0x04 }},
{0x5A, 1, {0x05 }},
{0x5F, 1, {0x20 }},
{0x65, 1, {0x21 }},
{0x60, 1, {0x21 }},
{0x66, 1, {0x20 }},
{0x76, 1, {0x44 }},
{0x77, 1, {0x44 }},
{0x78, 1, {0x44 }},
{0x79, 1, {0x44 }},
{0x7A, 1, {0x44 }},
{0x7B, 1, {0x44 }},
{0x49, 1, {0x83 }},
{0x4A, 1, {0x83 }},
{0x4F, 1, {0x83 }},
{0x50, 1, {0x83 }},
{0x55, 1, {0x83 }},
{0x56, 1, {0x83 }},
{0x5B, 1, {0x83 }},
{0x5C, 1, {0x83 }},
{0x61, 1, {0x83 }},
{0x62, 1, {0x83 }},
{0x67, 1, {0x83 }},
{0x68, 1, {0x83 }},
{0xC2, 1, {0x84 }},
{0xC6, 1, {0x46 }},
{0x29, 1, {0x00 }},
{0xCA, 1, {0x03 }},
{0xCB, 1, {0x3C }},
{0xCD, 1, {0x3C }},
{0xCC, 1, {0x20 }},
{0xCE, 1, {0x20 }},
{0xCF, 1, {0x76 }},
{0xD0, 1, {0x72 }},
{0xFF, 1, {0x22 }},
{0x05, 1, {0x10 }},
{0x08, 1, {0x10 }},

{0xFF, 1, {0x20 }},
{0xC3, 1, {0x00 }},
{0xC4, 1, {0xAA }},
{0xC5, 1, {0x00 }},
{0xC6, 1, {0xAA }},
{0xB3, 1, {0x00 }},
{0xB4, 1, {0x20 }},
{0xB5, 1, {0x00 }},
{0xB6, 1, {0xDC }},


{0xFF, 1, {0x28 }}, 
{0x3D, 1, {0x59 }}, 
{0x3E, 1, {0x59 }}, 
{0x3F, 1, {0x46 }}, 
{0x40, 1, {0x46 }}, 
{0x45, 1, {0x45 }}, 
{0x46, 1, {0x45 }}, 
{0x47, 1, {0x43 }}, 
{0x48, 1, {0x43 }}, 
{0x5A, 1, {0x8a }}, 
{0x5B, 1, {0x8a }}, 
{0x62, 1, {0x8D }},

{0xFF, 1, {0x20 }}, 
{0x7E, 1, {0x01 }},
{0x7F, 1, {0x00 }},
{0x80, 1, {0x64 }},
{0x81, 1, {0x00 }},
{0x82, 1, {0x00 }},
{0x83, 1, {0x64 }},
{0x84, 1, {0x64 }},
{0x85, 1, {0x45 }},
{0x86, 1, {0x7B }},
{0x87, 1, {0x45 }},
{0x88, 1, {0x7B }},
{0x8A, 1, {0x0A }},
{0x8B, 1, {0x0A }},

//GAMMA
{0xFF, 1, {0x23 }},
{0x29, 1, {0x03 }},

{0x01, 16, {0x00,0x00,0x00,0x03,0x00,0x2A,0x00,0x4B,0x00,0x69,0x00,0x86,0x00,0xA1,0x00,0xBA }},
{0x02, 16, {0x00,0xCF,0x01,0x15,0x01,0x48,0x01,0x8D,0x01,0xBB,0x01,0xFF,0x02,0x38,0x02,0x39 }},
{0x03, 16, {0x02,0x72,0x02,0xB8,0x02,0xDF,0x03,0x1D,0x03,0x41,0x03,0x6E,0x03,0x7C,0x03,0x8A }},
{0x04, 12, {0x03,0x9A,0x03,0xAC,0x03,0xBF,0x03,0xD6,0x03,0xF1,0x03,0xFF }},

{0x0D, 16, {0x00,0x00,0x00,0x03,0x00,0x2A,0x00,0x4B,0x00,0x69,0x00,0x86,0x00,0xA1,0x00,0xBA }},
{0x0E, 16, {0x00,0xCF,0x01,0x15,0x01,0x48,0x01,0x8D,0x01,0xBB,0x01,0xFF,0x02,0x38,0x02,0x39 }},
{0x0F, 16, {0x02,0x72,0x02,0xB8,0x02,0xDF,0x03,0x1D,0x03,0x41,0x03,0x6E,0x03,0x7C,0x03,0x8A }},
{0x10, 12, {0x03,0x9A,0x03,0xAC,0x03,0xBF,0x03,0xD6,0x03,0xF1,0x03,0xFF }},

/////////////PWM///////////////////////
{0x2D,1, {0x65 }},
{0x2E,1, {0x00 }},

{0x32,1, {0x02 }},//PWM 20K
{0x33,1, {0x18 }},

{0xFF, 1, {0x10 }},
                                                                                                                                                                                                
{0x35,1, {0x00 }},
{0x53,1, {0x2c }},
{0x51,2, {0x00,0x55 }},                                                                                                                                
{0x36,1, {0x08 }},                                                                         
{0x69,1, {0x00 }},
{0x71,2, {0x12,0x46 }},                                                                                                                               

{0xFF,1, {0x24 }},
{0x7D,1, {0x55 }},
{0xFF,3, {0x66,0x99,0x55 }},

{0xFF,1, {0x10 }}, 

	{0x11,1, {0x00}},        // sleep out
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},      // display on
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE,1,{0x00}},
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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

//Begin add by xiongbo.huang for defect 8423473 on 2019/10/23
        params->physical_width = 67;
        params->physical_height = 142;
//End add by xiongbo.huang for defect 8423473 on 2019/10/23

	// enable tearing-free
	//params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
	//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;


	params->dsi.mode   = SYNC_PULSE_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   		= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      		= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;

	// Video mode setting

	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	//params->dsi.word_count=480*3;

	//params->dsi.noncont_clock = true;
	//params->dsi.noncont_clock_period=2;
	params->dsi.lcm_ext_te_enable=false;
	//params->dsi.word_count=FRAME_WIDTH*3;
    params->dsi.vertical_sync_active=6;
	params->dsi.vertical_backporch=28;
	params->dsi.vertical_frontporch=140;
	params->dsi.vertical_active_line=FRAME_HEIGHT;
	params->dsi.horizontal_sync_active=4;
    params->dsi.horizontal_backporch=80;
	params->dsi.horizontal_frontporch=80;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.esd_check_enable = 1;

	params->dsi.customization_esd_check_enable = 1;
#if 1
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	/*params->dsi.lcm_esd_check_table[1].cmd          = 0x0F;
	params->dsi.lcm_esd_check_table[1].count        = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0xC0;*/
#endif

	//params->dsi.fbk_div =30;              // fref=26MHz,  fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)   //32
        //params->dsi.ssc_range = 7;
    params->dsi.PLL_CLOCK=265;
    params->dsi.ssc_disable = 1;
        #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
        params->corner_pattern_width = 720;//Note:这里是屏幕的宽度，不是原始图片宽\E5\BA?
        params->corner_pattern_height = 25;//圆角的高\E5\BA?

	params->corner_pattern_height_bot = 25;
        #endif
}

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;
	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}

}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;

	cmd=0x00;
	data=0x11; //vsp 5.6v
	//VPS=0x00;data=0x0A;VSP=5V,
	//         data=0x0E;VSP=5.4V,
	//	   data=0x12;VSP=5.8V,
	//VNG=0x01;data=0x0A;VNG=-5V,
	//         data=0x0E;VNG=-5.4V,
	//	   data=0x12;VSP=-5.8V,
#ifndef CONFIG_FPGA_EARLY_PORTING
	//enable power
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
	data=0x11; //vsp 5.6v
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
	//LCD_DEBUG("kernel:vsp,vsn on end\n");
	MDELAY(5);
	//reset high to low to high
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	//lcd_queue_load_tp_fw();
	//MDELAY(22);
	push_table(lcm_initialization_setting_ty, sizeof(lcm_initialization_setting_ty) / sizeof(struct LCM_setting_table), 1);
}
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Sleep Mode On
{0xFF,3,{0x55,0xAA,0x66}},
{0xFF,1,{0xB3}},
{0x68,1,{0x00}},
{0x2A,1,{0x00}},
{0x28,1,{0x00}},
{0x29,1,{0x00}},
{0x2B,1,{0x00}},

{0xFF,1,{0x20}},
{0x4A,1,{0x01}},
{0x48,1,{0x10}},
{0x49,1,{0x00}},

{0xFF,1,{0x28}},
{0x2F,1,{0x0D}},

{0xFF,1,{0x10}},
{0x28,1,{0x00}},
{REGFLAG_DELAY, 50, {}},
// Sleep Mode On
{0x10,1,{0x00}},
{REGFLAG_DELAY, 120, {}}, //120ms
{0xFF,1,{0x26}},
{0x1D,2,{0x8A,0x80}},
{0xFF,1,{0xB3}},
{0x04,1,{0x16}},
{0xFF,1,{0xB3}},
{0x01,1,{0x90}},
{0xFF,1,{0x26}},
{0x1F,1,{0x01}},
};
static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	LCD_DEBUG("kernel:vsp,vsn off begin\n");
	SET_RESET_PIN(1);
	MDELAY(10);
	set_gpio_lcd_enn(1);
	set_gpio_lcd_enn(0);
	MDELAY(20);
	set_gpio_lcd_enp(1);
	set_gpio_lcd_enp(0);
    	MDELAY(10);
	LCD_DEBUG("kernel:lcm_suspend\n");

}
static void lcm_resume(void)
{
    lcm_init();

}

static unsigned int lcm_compare_id(void)
{
	return 1;
}
struct LCM_DRIVER bora_gc7202h_ykl_hd_vdo_lcm_drv =
{
    .name           = "bora_gc7202h_ykl_hd_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,/*kd init fun.*/
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
};
#endif


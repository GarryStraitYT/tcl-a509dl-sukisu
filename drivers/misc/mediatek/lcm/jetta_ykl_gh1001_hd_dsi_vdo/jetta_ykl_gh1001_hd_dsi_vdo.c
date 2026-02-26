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
#define GTP_RST_PORT    0
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
#define FRAME_HEIGHT 										(1560)

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
		{0xee,1, {0x50}},			// page 1
		{0xea,2, {0x85,0x55}},
		{0x30,1, {0x00}},
		{0x24,1, {0x20}},			// rgb  TE
		{0x56,1, {0x03}},			// auto lane 
		{0x7a,1, {0x20}},
		{0x7d,1, {0x00}},
		{0x80,1, {0x10}},			// te v width
		{0x81,1, {0x50}},			// tetp en
		{0x90,2, {0x50,0x10}},
		{0x95,1, {0x74}},
		{0x97,1, {0x3A}},			// smrt gip    3A 关smrt gip
		{0x99,1, {0x00}},
		
		{0xee,1, {0x60}},
		{0x28,1, {0x12}},
		{0x2c,1, {0xf9}},
		{0x30,1, {0x01}},			// 4 lane  
		{0x32,1, {0xd9}},			// vrs_tldo
		{0x3a,1, {0x24}},			// GAS diasable 
		{0x3b,1, {0x00}},			// gip_s3s
		{0x3C,1, {0x37}},			// VCOM
		{0x3d,1, {0x01}},			// VGL
		{0x3e,1, {0x83}},			// VGH
		{0x42,2, {0x55,0x55}},		// vsp vsn
		{0x44,1, {0x0a}},			//vgh
		{0x46,1, {0x1e}},			//vgl
		{0x86,1, {0x20}},
		{0x89,1, {0x00}},
		{0x9a,1, {0x05}},
		{0x9b,2, {0x03,0x0c}},
		
		{0x47,5, {0x02,0x28,0x38,0x4a,0x4f}},			//gamma P
		{0x4C,5, {0x60,0x5a,0x71,0x57,0x58}},
		{0x51,5, {0x5a,0x3e,0x52,0x4c,0x59}},
		{0x56,4, {0x5a,0x64,0x6f,0x7f}},  
		
		{0x5a,5, {0x02,0x28,0x38,0x4a,0x4f}},			//gamma n 0.4.8.12.20
		{0x5f,5, {0x60,0x5a,0x71,0x57,0x58}},			//28.44.64.96.128. 
		{0x64,5, {0x5a,0x3e,0x52,0x4c,0x59}},			//159.191.211.227.235  
		{0x69,4, {0x5a,0x64,0x6f,0x7f}},				//243.247.251.255
		
		{0xee,1, {0x70}}, 
		
		//STV0 stv1 stv2
		{0x00,4, {0x10,0x13,0x00,0x01}},  
		{0x04,4, {0x38,0x3B,0x66,0x01}},  
		{0x08,4, {0x3B,0x3E,0x66,0x01}},  
		{0x0c,2, {0x05,0x5d}}, //07 dd
		
		//CYC0 cyc1
		{0x10,5, {0x12,0x15,0x00,0x00,0x00}}, 
		{0x15,4, {0x00,0x41,0x0e,0x0c}},  
		{0x20,5, {0x18,0x1B,0x00,0x00,0x00}}, 
		{0x25,4, {0x00,0x53,0x0e,0x0c}},  
		{0x29,2, {0x05,0x5d}},  //0a dd
		
		{0x46,5, {0xff,0x00,0x00,0x00,0x40}},  
		{0x4b,1, {0x88}}, 
		
		//for FILM0 FILM1
		{0x30,6, {0x45,0x46,0x66,0x7d,0x7d,0x50}},
		{0x36,6, {0x45,0x46,0x66,0x7d,0x7d,0x50}},
		
		//gip0-gip21=gipL1-gipL22   forward SCAN
		{0x60,5, {0x3c,0x3c,0x08,0x04,0x20}},  
		{0x65,5, {0x21,0x10,0x12,0x14,0x16}},  
		{0x6a,5, {0x19,0x1b,0x00,0x3c,0x3c}},  
		{0x6f,5, {0x3c,0x3c,0x3c,0x3c,0x3c}},
		{0x74,2, {0x3c,0x3c}},
		
		//gip22-gip43=gipR1-gipR22 
		{0x80,5, {0x3c,0x3c,0x09,0x05,0x20}},  
		{0x85,5, {0x21,0x11,0x13,0x15,0x17}},  
		{0x8a,5, {0x1a,0x1c,0x01,0x3c,0x3c}},  
		{0x8f,5, {0x3c,0x3c,0x3c,0x3c,0x3c}},
		{0x94,2, {0x3c,0x3c}},
		
		{0xea,2, {0x00,0x00}}, 
		{0xee,1, {0x00}},			// ENTER PAGE0

	{0x11,1,{0x00}},        // sleep out
	{REGFLAG_DELAY, 200, {}},
	{0x29,1,{0x00}},      // display on
	{REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE,1,{0x00}}
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
    params->dsi.vertical_sync_active=8;
	params->dsi.vertical_backporch=12;
	params->dsi.vertical_frontporch=16;
	params->dsi.vertical_active_line=FRAME_HEIGHT;
	params->dsi.horizontal_sync_active=20;
    params->dsi.horizontal_backporch=20;
	params->dsi.horizontal_frontporch=90;
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
    params->dsi.PLL_CLOCK=252;
    params->dsi.ssc_disable = 1;
	//params->dsi.HS_PRPR  = 1;
	//params->dsi.CLK_HS_PRPR = 1;
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
	data=0x12; //vsp 5.8v
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
	data=0x12; //vsp 5.8v
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
	{0x28,0,{0x00}},
	{REGFLAG_DELAY,20,{0x00}},
	{0x10,0,{0x00}},
	{REGFLAG_DELAY,120,{0x00}},
	{REGFLAG_END_OF_TABLE,1,{0x00}},
};
static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	LCD_DEBUG("kernel:vsp,vsn off begin\n");
	SET_RESET_PIN(0);
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
struct LCM_DRIVER jetta_ykl_gh1001_hd_dsi_vdo_lcm_drv =
{
    .name           = "jetta_ykl_gh1001_hd_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,/*kd init fun.*/
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
};
#endif


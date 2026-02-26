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
{0xF0, 2, {0x5A, 0x59}},
{0xF1, 2, {0xA5, 0xA6}},
{0xB0, 29, {0x83, 0x82, 0x00, 0x00, 0x85, 0x86, 0x00, 0x00, 0x33, 0x33, 0x33, 0x33, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x0F, 0x05, 0x04, 0x03, 0x02, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00}},
{0xB1, 32, {0x12, 0x42, 0x89, 0x80, 0x00, 0x00, 0x00, 0x85, 0x00, 0x00, 0x04, 0x08, 0x53, 0x00, 0x00, 0x00, 0x44, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x00, 0x00, 0x00}},
{0xB2, 17, {0x54, 0xC4, 0x82, 0x05, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x05, 0x05, 0x54, 0x0C, 0x0C, 0x0D, 0x0B}},
{0xB3, 31, {0x02, 0x00, 0x00, 0x00, 0x00, 0x26, 0x26, 0x91, 0xA2, 0x33, 0x44, 0x00, 0x26, 0x00, 0x18, 0x01, 0x02, 0x08, 0x20, 0x30, 0x08, 0x09, 0x44, 0x20, 0x40, 0x20, 0x40, 0x08, 0x09, 0x22, 0x33}},
{0xB4, 28, {0x00, 0x00, 0x00, 0x09, 0x1D, 0x1C, 0x13, 0x11, 0x0F, 0x0D, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFC}},
{0xB5, 28, {0x00, 0x00, 0x00, 0x08, 0x1D, 0x1C, 0x12, 0x10, 0x0E, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFC}},

//Backword scan
//R36 03
//RB4 00 00 00 04 1D 1C 0C 0E 10 12 08 00 00 00 00 00 00 00 00 00 00 00 FF FF FC FF FF FC
//RB5 00 00 00 05 1D 1C 0D 0F 11 13 09 00 00 00 00 00 00 00 00 00 00 00 FF FF FC FF FF FC

{0xB8, 24, {0x55, 0x55, 0x55, 0x55, 0x55, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x50, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xA0, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xA0}},
//RBA 82 82 //OTP Vcom请屏蔽
{0xBB, 13, {0x01, 0x05, 0x09, 0x11, 0x0D, 0x19, 0x1D, 0x55, 0x25, 0x69, 0x00, 0x21, 0x25}},
{0xBC, 14, {0x00, 0x00, 0x00, 0x00, 0x02, 0x20, 0xFF, 0x00, 0x03, 0x33, 0x01, 0x73, 0x33, 0x00}},
{0xBD, 10, {0xE9, 0x02, 0x4E, 0xCF, 0x72, 0xA4, 0x08, 0x44, 0xAE, 0x15}},
{0xBE, 10, {0x59, 0x59, 0x50, 0x3C, 0x0C, 0x77, 0x43, 0x07, 0x0E, 0x0E}},//VGMP=5V, VGMN=-5V, VGH=15V, VGL=-11V
{0xBF, 8, {0x07, 0x25, 0x07, 0x25, 0x7F, 0x00, 0x11, 0x04}},
{0xC0, 9, {0x10, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00}},
{0xC1, 19, {0xC0, 0x20, 0x20, 0x96, 0x04, 0x30, 0x30, 0x04, 0x2A, 0xA0, 0x35, 0x00, 0x07, 0xCF, 0xFF, 0xFF, 0xA4, 0x01, 0xC0}},
{0xC2, 1, {0x00}},
{0xC3, 9, {0x06, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x81, 0x01}},
{0xC4, 10, {0x84, 0x01, 0x2B, 0x41, 0x00, 0x3C, 0x00, 0x03, 0x03, 0x2E}},
{0xC5, 11, {0x03, 0x1C, 0xC0, 0xA8, 0x48, 0x10, 0x62, 0x44, 0x0A, 0x09, 0x26}},
{0xC6, 10, {0x7F, 0xA1, 0x22, 0x1F, 0x1D, 0x31, 0x7F, 0x04, 0x08, 0x00}},

//GAMMA2.2
{0xC7, 22, {0xF7, 0xD4, 0xBB, 0xA7, 0x80, 0x64, 0x36, 0x89, 0x51, 0x23, 0xF9, 0xC8, 0x20, 0xF3, 0xD4, 0xA9, 0x8F, 0x6B, 0x42, 0x7F, 0xE4, 0x00}},
{0xC8, 22, {0xF7, 0xD4, 0xBB, 0xA7, 0x80, 0x64, 0x36, 0x89, 0x51, 0x23, 0xF9, 0xC8, 0x20, 0xF3, 0xD4, 0xA9, 0x8F, 0x6B, 0x42, 0x7F, 0xE4, 0x00}},

{0xCB, 1, {0x00}},
{0xD0, 5, {0x80, 0x0D, 0xFF, 0x0F, 0x61}},
{0xD2, 1, {0x42}},
{0xFE, 4, {0xFF, 0xFF, 0xFF, 0x40}},
{0xFA, 3, {0x45, 0x93, 0x01}},
{0xF6, 1, {0x30}},

{0xF1, 2, {0x5A, 0x59}},
{0xF0, 2, {0xA5, 0xA6}},

{0x35, 1, {0x00}},
//=========================================================
// Sleep Out + Display On
//=========================================================
{0x11,1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,20,{}},
{0x26, 1, {0x01}},  //oepn TP
{REGFLAG_END_OF_TABLE, 0x00, {} }
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
    params->dsi.vertical_sync_active=4;
	params->dsi.vertical_backporch=32;
	params->dsi.vertical_frontporch=150;
	params->dsi.vertical_active_line=FRAME_HEIGHT;
	params->dsi.horizontal_sync_active=4;
    params->dsi.horizontal_backporch=48;
	params->dsi.horizontal_frontporch=48;
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
    params->dsi.PLL_CLOCK=253;
    params->dsi.ssc_disable = 1;

	params->dsi.LPX = 4;

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

unsigned long temp_F6 = 0;
int temp_F6_get(char *str)
{
	int ret = 0;
	ret = kstrtoul(str, 0, &temp_F6);//bo_liu, 未检查的返回值
	if (ret == 0){
		printk("kstrtoul str to unsigned long success\n");
	}else{
		printk("kstrtoul str to unsigned long failed\n");
	}
	printk("%s[%d] temp_F6 = 0x%x str = %s\n",__func__,__LINE__,temp_F6,str);

	return 0;
}

__setup("NL9911C_Temp_F6=",temp_F6_get);

//extern int fts_reset_proc(int hdelayms);

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
	//fts_reset_proc(10);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	//lcd_queue_load_tp_fw();
	//MDELAY(22);


	//printk("%s[%d] icnl9911c lcm_initialization_setting_ty[28].para_list[0] = 0x%x\n",__func__,__LINE__,lcm_initialization_setting_ty[28].para_list[0]);
	lcm_initialization_setting_ty[28].para_list[0] = temp_F6;
	printk("%s[%d] icnl9911c lcm_initialization_setting_ty[28].para_list[0] = 0x%x\n",__func__,__LINE__,lcm_initialization_setting_ty[28].para_list[0]);
    MDELAY(5);


	push_table(lcm_initialization_setting_ty, sizeof(lcm_initialization_setting_ty) / sizeof(struct LCM_setting_table), 1);
}
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Sleep Mode On
    {0x26, 1, {0x08} },
    {0x28, 0, {} },
    {REGFLAG_DELAY, 20, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 120, {} },
    {REGFLAG_END_OF_TABLE,1,{0x00}},
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
struct LCM_DRIVER bora_nl9911c_tdt_hd_vdo_lcm_drv =
{
    .name           = "bora_nl9911c_tdt_hd_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,/*kd init fun.*/
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
};
#endif


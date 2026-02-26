
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
#define LCM_DSI_CMD_MODE                                    0
#define FRAME_WIDTH                                     (720)
#define FRAME_HEIGHT                                    (1600)

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

#define CONFIG_BIAS_TPS65132_I2C
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
  //  {0x26,0x01,{0x08}},
    {0x28, 0, {} },
    {REGFLAG_DELAY, 20, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 120, {} },
    {REGFLAG_END_OF_TABLE,1,{0x00}},
};

static struct LCM_setting_table lcm_resume_setting[] = {
    //----------------------LCD initial code start----------------------//
 	{0xF0, 0x02, {0x5A,0x59}},
	{0xF1, 0x02, {0xA5,0xA6}},
	{0xB0, 0x1E, {0x87,0x86,0x85,0x84,0x88,0x89,0x00,0x00,0x33,0x33,0x33,0x33,0x00,0x05,0x05,0x80,0x05,0x00,0x0F,0x05,0x04,0x03,0x02,0x01,0x02,0x03,0x04,0x00,0x00,0x00}},
	{0xB1, 0x1D, {0x53,0x43,0x85,0x00,0x00,0x05,0x05,0x80,0x05,0x00,0x04,0x08,0x54,0x00,0x00,0x00,0x44,0x40,0x02,0x01,0x40,0x02,0x01,0x40,0x02,0x01,0x40,0x02,0x01}},
	{0xB5, 0x1C, {0x08,0x00,0x00,0xc0,0x04,0x06,0xc1,0xc1,0x0C,0x0C,0x0E,0x0E,0x10,0x10,0x12,0x12,0x00,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFC,0x00,0x00,0x00}},
	{0xB4, 0x1C, {0x09,0x00,0x00,0xc0,0x05,0x07,0xc1,0xc1,0x0D,0x0D,0x0F,0x0F,0x11,0x11,0x13,0x13,0x00,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFC,0x00,0x00,0x00}},
	{0xB8, 0x18, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xBA, 0x02, {0x5C,0x5C}},
	{0xBB, 0x0D, {0x01,0x05,0x09,0x11,0x0D,0x19,0x1D,0x55,0x25,0x69,0x00,0x21,0x25}},
	{0xBC, 0x0E, {0x00,0x00,0x00,0x00,0x02,0x20,0xFF,0x00,0x03,0x13,0x01,0x73,0x33,0x00}},
	{0xBD, 0x0A, {0xE9,0x02,0x4F,0xCF,0x72,0xA4,0x08,0x44,0xAE,0x15}},

	{0xBE, 0x0A, {0x7D,0x7D,0x50,0x32,0x0C,0x77,0x43,0x07,0x0E,0x0E}},
	{0xBF, 0x08, {0x07,0x25,0x07,0x25,0x7F,0x00,0x11,0x04}},
	{0xC0, 0x09, {0x10,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0xFF,0x00}},
	{0xC1, 0x13, {0xC0,0x20,0x20,0x7C,0x04,0x32,0x32,0x04,0x2A,0x40,0x36,0x00,0x07,0xCF,0xFF,0xFF,0xC0,0x00,0xC0}},
//	{0xC2, 0x01, {0x00}},
	{0xC3, 0x0A, {0x06,0x00,0xFF,0x00,0xFF,0x00,0x00,0x81,0x01,0x00}},
	{0xC4, 0x0A, {0x84,0x01,0x2B,0x41,0x00,0x3C,0x00,0x03,0x03,0x2E}},
	{0xC5, 0x0B, {0x03,0x1C,0xC0,0xC0,0x40,0x10,0x42,0x44,0x0a,0x09,0x14}},
	{0xC6, 0x0A, {0x88,0x16,0x20,0x28,0x29,0x33,0x64,0x34,0x08,0x04}},
	//{0xC7, 0x16, {0xF7,0x98,0x6B,0x4B,0x17,0xF6,0xC3,0x17,0xE3,0xBB,0x93,0x69,0xC7,0xA3,0x8B,0x69,0x55,0x38,0x1A,0x7C,0x80,0x00}},
//	{0xC8, 0x16, {0xF7,0x98,0x6B,0x4B,0x17,0xF6,0xC3,0x17,0xE3,0xBB,0x93,0x69,0xC7,0xA3,0x8B,0x69,0x55,0x38,0x1A,0x7C,0x80,0x00}},
	{0xCB, 0x01, {0x00}},
	{0xD0, 0x05, {0x80,0x0D,0xFF,0x0F,0x63}},
	{0xD2, 0x01, {0x42}},
	{0xE1, 0x17, {0xEF,0xFE,0xFE,0xFE,0xFE,0xEE,0xF0,0x20,0x33,0xFF,0x00,0x00,0x6A,0x90,0xC0,0x0D,0x6A,0xF0,0x3E,0xFF,0x00,0x07,0xD0}},
	{0xE0, 0x1A, {0x30,0x00,0x80,0x88,0x11,0x3F,0x22,0x62,0xDF,0xA0,0x04,0xCC,0x01,0xFF,0xF6,0xFF,0xF0,0xFD,0xFF,0xFD,0xF8,0xF5,0xFC,0xFC,0xFD,0xFF}},
	{0xF1, 0x02, {0x5A,0x59}},
	{0xF0, 0x02, {0xA5,0xA6}},
	{0x35, 0x01, {0x00}},

	
	{0x11, 0, {}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 0, {}},
	{REGFLAG_DELAY, 50, {}},
	{0x26,0x01,{0x01}},
	{REGFLAG_END_OF_TABLE,1,{0x00}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    for (i = 0; i < count; i++) {
        unsigned cmd;
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
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
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

    // enable tearing-free
    //params->dbi.te_mode           = LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_edge_polarity      = LCM_POLARITY_RISING;
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;


    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq           = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding             = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size = 256;

    // Video mode setting

    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    //params->dsi.word_count=480*3;

    //here is for esd protect by legen
    //params->dsi.noncont_clock = true;
    //params->dsi.noncont_clock_period=2;
    params->dsi.lcm_ext_te_enable=false;
    //for esd protest end by legen
    //params->dsi.word_count=FRAME_WIDTH*3;
    params->dsi.vertical_sync_active = 4;
    params->dsi.vertical_backporch = 32;
    params->dsi.vertical_frontporch = 124;
    params->dsi.vertical_active_line=FRAME_HEIGHT;

    //params->dsi.line_byte=2180;
    params->dsi.horizontal_sync_active = 4;
    params->dsi.horizontal_backporch = 50;
    params->dsi.horizontal_frontporch = 50;//for 286 with 5% spread;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.esd_check_enable = 1;

    params->dsi.customization_esd_check_enable = 1;

    params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

    /*params->dsi.lcm_esd_check_table[2].cmd          = 0x0F;
    params->dsi.lcm_esd_check_table[2].count        = 1;
    params->dsi.lcm_esd_check_table[2].para_list[0] = 0xC0;*/

    //params->dsi.fbk_div =30;              // fref=26MHz,  fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)   //32

    //begin modify by kun.zheng for task 8943891 on 2020/02/24
    params->dsi.PLL_CLOCK = 270;
    //end modify by kun.zheng for task 8943891 on 2020/02/24
    params->dsi.ssc_disable = 1;
        //params->dsi.ssc_range = 5;
#ifdef MTK_ROUND_CORNER_SUPPORT
    params->round_corner_params.round_corner_en = 0;
    params->round_corner_params.w = ROUND_CORNER_W;
    params->round_corner_params.h = ROUND_CORNER_H;
    params->round_corner_params.lt_addr= left_top;
    params->round_corner_params.rt_addr = right_top;
    params->round_corner_params.lb_addr = left_bottom;
    params->round_corner_params.rb_addr = right_bottom;
#endif
}

static void lcm_init_power(void)
{
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP1);
    MDELAY(5);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENN1);
#if defined(CONFIG_BIAS_TPS65132_I2C)
	_lcm_i2c_write_bytes(0x0, 0x12); //+5.6V
	MDELAY(1);
	_lcm_i2c_write_bytes(0x1, 0x12); //-5.6V
	MDELAY(1);
#endif
}

static void lcm_suspend_power(void)
{
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENN0);
    MDELAY(5);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP0);
}

static void lcm_resume_power(void)
{
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP1);
    MDELAY(5);
    disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENN1);
#if defined(CONFIG_BIAS_TPS65132_I2C)
	_lcm_i2c_write_bytes(0x0, 0x12); //+5.8V
	MDELAY(1);
	_lcm_i2c_write_bytes(0x1, 0x12); //-5.8V
	MDELAY(1);
#endif
}

static void lcm_init(void)
{
    MDELAY(8);
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(37);
    push_table(lcm_resume_setting, sizeof(lcm_resume_setting) / sizeof(struct LCM_setting_table), 1);
    //disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BL_EN1);
}

static void lcm_suspend(void)
{
    //disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BL_EN0);
    push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(0);
    MDELAY(10);
}

static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
 	return 1;
}

struct LCM_DRIVER cruze_nl9911c_truly_hd_dsi_vdo_lcm_drv = {
    .name = "cruze_nl9911c_truly_hd_dsi_vdo",
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

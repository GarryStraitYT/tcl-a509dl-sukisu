
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

#else
//#include <mt-plat/mtk_gpio.h>
//#include <mach/gpio_const.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#ifndef GPIO_DISP_ID0_PIN
#define GPIO_DISP_ID0_PIN               (GPIO16 | 0x80000000)
#endif
#ifndef GPIO_DISP_ID1_PIN
#define GPIO_DISP_ID1_PIN               (GPIO23 | 0x80000000)
#endif
#define GPIO_LCD_BIAS_ENN_PIN        			(GPIO171 | 0x80000000)
#define GPIO_LCD_BIAS_ENP_PIN        			(GPIO173 | 0x80000000)//zhangbing add for LCD enp gpio

#define set_gpio_lcd_enp(cmd) \
	lcm_util.set_gpio_lcd_enp_bias(cmd)
static struct LCM_UTIL_FUNCS lcm_util;


#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))

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

/*zhangbing add begin*/
#define set_gpio_lcd_enn(cmd) \
	lcm_util.set_gpio_lcd_enn_bias(cmd)
#define set_gpio_lcd_enp(cmd) \
	lcm_util.set_gpio_lcd_enp_bias(cmd)

extern int tps65132_write_bytes(unsigned char addr, unsigned char value);
/*zhangbing add end*/

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

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

//#include <mach/mtk_6306_gpio.h>
//add end
#endif

#define LCM_DSI_CMD_MODE                                    0
#define FRAME_WIDTH                                     (720)
#define FRAME_HEIGHT                                    (1640)

/* physical size in um */
//#define LCM_PHYSICAL_WIDTH                                    (72040)
//#define LCM_PHYSICAL_HEIGHT                                   (159100)

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

//begin modify by kun.zheng for task 9047204 on 2020/03/27
static struct LCM_setting_table lcm_suspend_setting[] = {
    {0x28, 0, {} },
    {REGFLAG_DELAY, 20, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 120, {} },
    {REGFLAG_END_OF_TABLE,1,{0x00}},
};
//end modify by kun.zheng for task 9047204 on 2020/03/27

//begin modify by kun.zheng for task 8810289 on 2020/02/28
static struct LCM_setting_table lcm_resume_setting[] = {
    //----------------------LCD initial code start----------------------//
 {0xFF, 1, {0x20}},
 {0xFB, 1, {0x01}},
 {0x01, 1, {0x55}},
 {0x03, 1, {0x55}},
 {0x05, 1, {0x99}},
 {0x07, 1, {0x4B}},
 {0x08, 1, {0xDF}},
 {0x0E, 1, {0x69}},
 {0x0F, 1, {0x7D}},
 {0x94, 1, {0x40}},
 {0x95, 1, {0xF5}},
 {0x96, 1, {0x1D}},
 {0x69, 1, {0x91}},
 {0x1F, 1, {0x02}},
 {0xFF, 1, {0x20}},
 {0xFB, 1, {0x01}},
 {0xB0, 16,{0x00,0x08,0x00,0x1A,0x00,0x37,0x00,0x50,0x00,0x66,0x00,0x79,0x00,0x8B,0x00,0x9C}},
 {0xB1, 16,{0x00,0xAB,0x00,0xDF,0x01,0x07,0x01,0x47,0x01,0x76,0x01,0xC5,0x02,0x00,0x02,0x03}},
 {0xB2, 16,{0x02,0x40,0x02,0x82,0x02,0xB1,0x02,0xE9,0x03,0x0E,0x03,0x3B,0x03,0x4A,0x03,0x59}},
 {0xB3, 12,{0x03,0x6A,0x03,0x7E,0x03,0x96,0x03,0xAE,0x03,0xD2,0x03,0xDA}},
 {0xB4, 16,{0x00,0x08,0x00,0x1A,0x00,0x37,0x00,0x50,0x00,0x66,0x00,0x79,0x00,0x8B,0x00,0x9C}},
 {0xB5, 16,{0x00,0xAB,0x00,0xDF,0x01,0x07,0x01,0x47,0x01,0x76,0x01,0xC5,0x02,0x00,0x02,0x03}},
 {0xB6, 16,{0x02,0x40,0x02,0x82,0x02,0xB1,0x02,0xE9,0x03,0x0E,0x03,0x3B,0x03,0x4A,0x03,0x59}},
 {0xB7, 12,{0x03,0x6A,0x03,0x7E,0x03,0x96,0x03,0xAE,0x03,0xD2,0x03,0xDA}},
 {0xB8, 16,{0x00,0x08,0x00,0x1A,0x00,0x37,0x00,0x50,0x00,0x66,0x00,0x79,0x00,0x8B,0x00,0x9C}},
 {0xB9, 16,{0x00,0xAB,0x00,0xDF,0x01,0x07,0x01,0x47,0x01,0x76,0x01,0xC5,0x02,0x00,0x02,0x03}},
 {0xBA, 16,{0x02,0x40,0x02,0x82,0x02,0xB1,0x02,0xE9,0x03,0x0E,0x03,0x3B,0x03,0x4A,0x03,0x59}},
 {0xBB, 16,{0x03,0x6A,0x03,0x7E,0x03,0x96,0x03,0xAE,0x03,0xD2,0x03,0xDA}},
 {0xFF, 1, {0x21}},
 {0xFB, 1, {0x01}},
 {0xB0, 16,{0x00,0x00,0x00,0x12,0x00,0x2F,0x00,0x48,0x00,0x5E,0x00,0x71,0x00,0x83,0x00,0x94}},
 {0xB1, 16,{0x00,0xA3,0x00,0xD7,0x00,0xFF,0x01,0x3F,0x01,0x6E,0x01,0xBD,0x01,0xF8,0x01,0xFB}},
 {0xB2, 16,{0x02,0x38,0x02,0x7A,0x02,0xA9,0x02,0xE1,0x03,0x06,0x03,0x33,0x03,0x42,0x03,0x51}},
 {0xB3, 12,{0x03,0x62,0x03,0x76,0x03,0x8E,0x03,0xA6,0x03,0xCA,0x03,0xD2}},
 {0xB4, 16,{0x00,0x00,0x00,0x12,0x00,0x2F,0x00,0x48,0x00,0x5E,0x00,0x71,0x00,0x83,0x00,0x94}},
 {0xB5, 16,{0x00,0xA3,0x00,0xD7,0x00,0xFF,0x01,0x3F,0x01,0x6E,0x01,0xBD,0x01,0xF8,0x01,0xFB}},
 {0xB6, 16,{0x02,0x38,0x02,0x7A,0x02,0xA9,0x02,0xE1,0x03,0x06,0x03,0x33,0x03,0x42,0x03,0x51}},
 {0xB7, 12,{0x03,0x62,0x03,0x76,0x03,0x8E,0x03,0xA6,0x03,0xCA,0x03,0xD2}},
 {0xB8, 16,{0x00,0x00,0x00,0x12,0x00,0x2F,0x00,0x48,0x00,0x5E,0x00,0x71,0x00,0x83,0x00,0x94}},
 {0xB9, 16,{0x00,0xA3,0x00,0xD7,0x00,0xFF,0x01,0x3F,0x01,0x6E,0x01,0xBD,0x01,0xF8,0x01,0xFB}},
 {0xBA, 16,{0x02,0x38,0x02,0x7A,0x02,0xA9,0x02,0xE1,0x03,0x06,0x03,0x33,0x03,0x42,0x03,0x51}},
 {0xBB, 12,{0x03,0x62,0x03,0x76,0x03,0x8E,0x03,0xA6,0x03,0xCA,0x03,0xD2}},
 {0xFF, 1, {0x23}},
 {0xFB, 1, {0x01}},
 {0x12, 1, {0xB9}},
 {0x15, 1, {0xE3}},
 {0x16, 1, {0x0B}},
 {0xFF, 1, {0x24}},
 {0xFB, 1, {0x01}},
 {0x93, 1, {0x0A}},
 {0x94, 1, {0x12}},
 {0x00, 1, {0x00}},
 {0x01, 1, {0x21}},
 {0x02, 1, {0x1D}},
 {0x03, 1, {0x21}},
 {0x04, 1, {0x04}},
 {0x05, 1, {0x05}},
 {0x06, 1, {0x1C}},
 {0x07, 1, {0x1C}},
 {0x08, 1, {0x0C}},
 {0x09, 1, {0x0D}},
 {0x0A, 1, {0x0E}},
 {0x0B, 1, {0x0F}},
 {0x0C, 1, {0x00}},
 {0x0D, 1, {0x00}},
 {0x0E, 1, {0x00}},
 {0x0F, 1, {0x00}},
 {0x10, 1, {0x00}},
 {0x11, 1, {0x00}},
 {0x12, 1, {0x00}},
 {0x13, 1, {0x00}},
 {0x14, 1, {0x00}},
 {0x15, 1, {0x00}},
 {0x16, 1, {0x00}},
 {0x17, 1, {0x21}},
 {0x18, 1, {0x1D}},
 {0x19, 1, {0x21}},
 {0x1A, 1, {0x04}},
 {0x1B, 1, {0x05}},
 {0x1C, 1, {0x1C}},
 {0x1D, 1, {0x1C}},
 {0x1E, 1, {0x0C}},
 {0x1F, 1, {0x0D}},
 {0x20, 1, {0x0E}},
 {0x21, 1, {0x0F}},
 {0x22, 1, {0x00}},
 {0x23, 1, {0x00}},
 {0x24, 1, {0x00}},
 {0x25, 1, {0x00}},
 {0x26, 1, {0x00}},
 {0x27, 1, {0x00}},
 {0x28, 1, {0x00}},
 {0x29, 1, {0x00}},
 {0x2A, 1, {0x00}},
 {0x2B, 1, {0x00}},
 {0x2D, 1, {0x0C}},
 {0x41, 1, {0x44}},
 {0x42, 1, {0x43}},
 {0x45, 1, {0x43}},
 {0x46, 1, {0x44}},
 {0x47, 1, {0x44}},
 {0x48, 1, {0x44}},
 {0x49, 1, {0x00}},
 {0x4A, 1, {0x21}},
 {0x4B, 1, {0x8A}},
 {0x4C, 1, {0x54}},
 {0x2F, 1, {0x0A}},
 {0x30, 1, {0x08}},
 {0x31, 1, {0x0E}},
 {0x32, 1, {0x0F}},
 {0x33, 1, {0x08}},
 {0x34, 1, {0x0A}},
 {0x35, 1, {0x0F}},
 {0x36, 1, {0x0E}},
 {0x37, 1, {0x44}},
 {0x38, 1, {0x44}},
 {0x39, 1, {0x00}},
 {0x3A, 1, {0x21}},
 {0x3B, 1, {0x8A}},
 {0x3D, 1, {0x54}},
 {0x4D, 1, {0x34}},
 {0x4E, 1, {0x12}},
 {0x4F, 1, {0x00}},
 {0x50, 1, {0x00}},
 {0x51, 1, {0x21}},
 {0x52, 1, {0x43}},
 {0x53, 1, {0x00}},
 {0x54, 1, {0x00}},
 {0x55, 1, {0x47}},
 {0x56, 1, {0x44}},
 {0x58, 1, {0x21}},
 {0x59, 1, {0x00}},
 {0x5A, 1, {0x21}},
 {0x5B, 1, {0x8A}},
 {0x5C, 1, {0x88}},
 {0x5D, 1, {0x08}},
 {0x5E, 1, {0x04}},
 {0x5F, 1, {0x00}},
 {0x60, 1, {0x80}},
 {0x61, 1, {0x9A}},
 {0x64, 1, {0x10}},
 {0x80, 1, {0x0F}},
 {0x81, 1, {0x09}},
 {0x82, 1, {0x0F}},
 {0x83, 1, {0x09}},
 {0x84, 1, {0x80}},
 {0x85, 1, {0x01}},
 {0x86, 1, {0x04}},
 {0x88, 1, {0x40}},
 {0x89, 1, {0xC8}},
 {0x8A, 1, {0x88}},
 {0x8D, 1, {0x08}},
 {0x8E, 1, {0x08}},
 {0x91, 1, {0x40}},
 {0x92, 1, {0xAE}},
 {0x98, 1, {0x80}},
 {0xAB, 1, {0x00}},
 {0xAD, 1, {0x00}},
 {0xAF, 1, {0x04}},
 {0xB0, 1, {0x05}},
 {0xB1, 1, {0xAA}},
 {0xFF, 1, {0x25}},
 {0xFB, 1, {0x01}},
 {0x17, 1, {0x82}},
 {0x18, 1, {0x06}},
 {0x19, 1, {0x0F}},
 {0xC0, 1, {0x07}},
 {0xC2, 1, {0x84}},
 {0xC6, 1, {0x92}},
 {0xCA, 1, {0x1C}},
 {0xCB, 1, {0x1C}},
 {0xCC, 1, {0x1C}},
 {0xCD, 1, {0x00}},
 {0xCE, 1, {0x00}},
 {0xD3, 1, {0x11}},
 {0xD4, 1, {0xCC}},
 {0xD5, 1, {0x11}},
 {0xD6, 1, {0x1C}},
 {0xD7, 1, {0x11}},
 {0x0A, 1, {0x82}},
 {0x0B, 1, {0x17}},
 {0x0C, 1, {0x01}},
 {0xFF, 1, {0x26}},
 {0xFB, 1, {0x01}},
 {0x00, 1, {0xA0}},
 {0xFF, 1, {0x27}},
 {0xFB, 1, {0x01}},
 {0x13, 1, {0x00}},
 {0x15, 1, {0xB4}},
 {0x1F, 1, {0x55}},
 {0x26, 1, {0x0F}},
 {0xC0, 1, {0x18}},
 {0xC1, 1, {0xF8}},
 {0xC7, 1, {0x03}},
 {0xC2, 1, {0x00}},
 {0xC3, 1, {0x00}},
 {0xC4, 1, {0xF8}},
 {0xC5, 1, {0x00}},
 {0xC6, 1, {0x00}},
 {0xFF, 1, {0x10}},
 {0xFB, 1, {0x01}},
 {0xBA, 1, {0x03}},
 {0x35, 1, {0x00}},
    //----------------------LCD initial code End----------------------//
    //SLPOUT and DISPON
    {0x11,1,{0x00}},
    {REGFLAG_DELAY,120,{}},
    {0x29,1,{0x00}},
    
    {REGFLAG_DELAY,20,{}},
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
    unsigned int count, unsigned char force_update)
{
    unsigned int i;
    unsigned cmd;

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
            dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
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
    params->dsi.packet_size=256;
    // Video mode setting

    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    //params->dsi.word_count=480*3;

    //here is for esd protect by legen
    //params->dsi.noncont_clock = true;
    //params->dsi.noncont_clock_period=2;
    params->dsi.lcm_ext_te_enable=false;
    //for esd protest end by legen
    //params->dsi.word_count=FRAME_WIDTH*3;
    params->dsi.vertical_sync_active=2;
    params->dsi.vertical_backporch=264;

    params->dsi.vertical_frontporch=10;

    params->dsi.vertical_active_line=FRAME_HEIGHT;

    //params->dsi.line_byte=2180;
    params->dsi.horizontal_sync_active=20;
    params->dsi.horizontal_backporch=88;
    params->dsi.horizontal_frontporch=100;//for 286 with 5% spread;


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
    params->dsi.PLL_CLOCK=340;
    //end modify by kun.zheng for task 8943891 on 2020/02/24
        params->dsi.ssc_disable = 1;
        //params->dsi.ssc_range = 5;
        #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
    //begin modify by kun.zheng for task 8951789 on 2020/02/24
    params->round_corner_en = 0;
    //end modify by kun.zheng for task 8951789 on 2020/02/24
        params->corner_pattern_width = 720;//Note:这里是屏幕的宽度，不是原始图片宽度
        params->corner_pattern_height = 16;//圆角的高度
        #endif
}

static void lcm_init_power(void)
{
    display_bias_enable();
    MDELAY(10);
}

static void lcm_suspend_power(void)
{
    display_bias_disable();
        MDELAY(10);
}

extern void tpd_gpio_output(int pin, int level);
#define SET_LCD_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
// #define SET_TP_RESET_PIN(v1,v2) (tpd_gpio_output(v1,v2))
static void lcm_resume_power(void)
{
    display_bias_enable();
}

//begin modify by kun.zheng for task 8810289 on 2020/02/28
static void lcm_init(void)
{
	/*zhangbing add begin*/
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;

	cmd=0x00;
	data=0x14; //vsp 6.0v
	//VPS=0x00;data=0x0A;VSP=5V,
	//         data=0x0E;VSP=5.4V,
	//	   data=0x12;VSP=5.8V,
	//	   data=0x13;VSP=5.9V,
	//	   data=0x14;VSP=6.0V,
	//VNG=0x01;data=0x0A;VNG=-5V,
	//         data=0x0E;VNG=-5.4V,
	//	   data=0x12;VSP=-5.8V,
	//	   data=0x13;VSP=-5.9V,
	//	   data=0x14;VSP=-6.0V,
#ifndef CONFIG_FPGA_EARLY_PORTING
	//enable power
	//set_gpio_lcd_enp(0);
	//set_gpio_lcd_enn(0);
	MDELAY(5);

	set_gpio_lcd_enp(1);
	MDELAY(5);

	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
		printk("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);

	MDELAY(5);
	//lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	set_gpio_lcd_enn(1);
	MDELAY(5);
	cmd=0x01;
	data=0x14; //vsp 6.0v
	//VPS=0x00;data=0x0A;VSP=5V,
	//         data=0x0E;VSP=5.4V,
	//	   data=0x12;VSP=5.8V,
	//	   data=0x13;VSP=5.9V,
	//	   data=0x14;VSP=6.0V,
	//VNG=0x01;data=0x0A;VNG=-5V,
	//         data=0x0E;VNG=-5.4V,
	//	   data=0x12;VSP=-5.8V,
	//	   data=0x13;VSP=-5.9V,
	//	   data=0x14;VSP=-6.0V,

	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
		printk("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
		printk("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);


#endif
	/*zhangbing add end*/
	//set_gpio_lcd_enp(1);   
 MDELAY(8);
    //SET_TP_RESET_PIN(0,0);
    //MDELAY(5);
    SET_LCD_RESET_PIN(0);
    MDELAY(5);

    //SET_TP_RESET_PIN(0,1);
    //MDELAY(2);
    SET_LCD_RESET_PIN(1);
    MDELAY(5);
    SET_LCD_RESET_PIN(0);
    MDELAY(5);
    SET_LCD_RESET_PIN(1);
    MDELAY(10);//we need 50ms  to allow flash to write init code.

    push_table(NULL, lcm_resume_setting, sizeof(lcm_resume_setting) / sizeof(struct LCM_setting_table), 1);
}
//end modify by kun.zheng for task 8810289 on 2020/02/28

static void lcm_suspend(void)
{
    push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
}

static void lcm_resume(void)
{
    lcm_init();
}

//static int lcm_id0_gpionum;
//static int lcm_id1_gpionum;


static unsigned int lcm_compare_id(void)
{
    /*int id_type=0;
    int ret =0;
    unsigned int lcd_id0;
    unsigned int lcd_id1;
    struct device_node *node0 = NULL;
    struct device_node *node1 = NULL;
    node0 = of_find_compatible_node(NULL,NULL,"display_lcm_id0");
    lcm_id0_gpionum = of_get_named_gpio(node0,"gpio_num",0);
    ret = gpio_request(lcm_id0_gpionum,"display_lcm_id0");
    gpio_direction_input(lcm_id0_gpionum);
    lcd_id0 = gpio_get_value(lcm_id0_gpionum);

    node1 = of_find_compatible_node(NULL,NULL,"display_lcm_id1");
    lcm_id1_gpionum = of_get_named_gpio(node1,"gpio_num",0);
    ret = gpio_request(lcm_id1_gpionum,"display_lcm_id1");
    gpio_direction_input(lcm_id1_gpionum);
    lcd_id1 = gpio_get_value(lcm_id1_gpionum);

    id_type = lcd_id0 <<1 |lcd_id1;

    LCM_LOGD("[LCM] TDT ID_type is: %d",id_type);
    if (id_type==0)
            return 1;
    else*/
        return 1;
}


/* return TRUE: need recovery */
/* return FALSE: No need recovery */
struct LCM_DRIVER austintf_tdt_nt36525b_hd_dsi_vdo_lcm_drv = {
    .name = "austintf_tdt_nt36525b_hd_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .compare_id = lcm_compare_id,
    .init_power = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
    //.esd_check = lcm_esd_check, //lcm_esd_check
    .set_backlight_cmdq = NULL, //lcm_setbacklight_cmdq,
    .ata_check = NULL, //lcm_ata_check,
    .update = NULL, //lcm_update,
    .switch_mode = NULL, //lcm_switch_mode
};

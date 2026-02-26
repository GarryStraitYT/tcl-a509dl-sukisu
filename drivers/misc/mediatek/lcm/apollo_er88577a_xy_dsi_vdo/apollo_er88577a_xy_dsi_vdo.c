/*history modified by xiongbo.huang on 20200310*/
/*history modified by xiongbo.huang for gpio modify on 20200407 begin*/
#ifndef CONFIG_MTK_LCM_DEVICE_TREE_SUPPORT
#ifndef BUILD_LK
    #include <linux/string.h>
    #include <linux/kernel.h>
#else
#include <string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
    #include <platform/mt_pmic.h>
#else
    //#include <mt-plat/mt_gpio.h>
    //#include <mach/gpio_const.h>
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
#define FRAME_WIDTH                 (800)
#define FRAME_HEIGHT                (1280)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

//#ifndef GPIO_LCD_IOVCC_EN_PIN
//#define GPIO_LCD_IOVCC_EN_PIN                   (GPIO150 | 0x80000000)
//#endif
#ifndef GPIO_LCD_BIAS_ENP_PIN
#define GPIO_LCD_BIAS_ENP_PIN                   (GPIO173 | 0x80000000)
#endif
#ifndef GPIO_LCD_BIAS_ENN_PIN
#define GPIO_LCD_BIAS_ENN_PIN                   (GPIO171 | 0x80000000)
#endif
#ifndef GPIO_LCM_ID0
#define GPIO_LCM_ID0                (GPIO23 | 0x80000000)
#endif
#ifndef GPIO_LCM_ID1
#define GPIO_LCM_ID1                (GPIO16 | 0x80000000)
#endif

#define REGFLAG_END_OF_TABLE        (0xFD)
#define REGFLAG_DELAY               (0xFC)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static struct LCM_UTIL_FUNCS lcm_util = {0};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define SET_RESET_PIN(v)                    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)                   (lcm_util.mdelay(n))
#define UDELAY(n)                   (lcm_util.udelay(n))
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enn(cmd) \
    lcm_util.set_gpio_lcd_enn_bias(cmd)
#define set_gpio_lcd_enp(cmd) \
    lcm_util.set_gpio_lcd_enp_bias(cmd)
/*#ifdef CONFIG_TCT_APOLLO
#define set_gpio_lcd_iovcc(cmd) \
    lcm_util.set_gpio_lcd_iovcc_en(cmd)
#endif*/
struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

//suspend timing: MIPI -> RESET -> VSP/VSN -> IOVCOO
extern int Apollo2_er88577a_qunchuang_kmylar_4nd;

static struct LCM_setting_table  lcm_initialization_setting_ty[] = {
{0xE0,2,{0xAB,0xBA}},
{0xE1,2,{0xBA,0xAB}},
{0xB1,4,{0x10,0x01,0x47,0xFF}},
{0xB2,6,{0x0C,0x14,0x04,0x50,0x50,0x14}},
{0xB3,3,{0x56,0xD3,0x00}},
{0xB4,3,{0x77,0x30,0x04}},
{0xB6,7,{0x00,0x00,0x00,0x10,0x00,0x10,0x00}},
{0xB8,7,{0x05,0x12,0x29,0x49,0x48,0x00,0x00}},
{0xB9,38,{0x7F,0x6F,0x62,0x55,0x53,0x44,0x48,0x31,0x48,0x45,
         0x43,0x5D,0x49,0x4F,0x41,0x3D,0x32,0x20,0x36,0x7F,
		 0x6F,0x62,0x55,0x53,0x44,0x48,0x31,0x48,0x45,0x43,
		 0x5D,0x49,0x4F,0x41,0x3D,0x32,0x20,0x06}},
{0xC0,16,{0x32,0x23,0x67,0x67,0x33,0x33,0x33,0x33,0x10,0x04,0x90,0x04,0x3F,0x00,0x00,0xC0}},
{0xC1,10,{0x13,0x14,0x02,0x8D,0x10,0x04,0x90,0x04,0x54,0x00}},
{0xC2,12,{0x37,0x09,0x08,0x89,0x88,0x21,0x22,0x21,0x44,0xBB,0x18,0x00}},
{0xC3,22,{0x86,0x40,0x00,0x08,0x1F,0x1E,0x02,0x16,0x14,0x02,0x12,0x10,0x02,0x0E,0x0C,0x04,0x02,0x02,0x02,0x02,0x02,0x02}},
{0xC4,22,{0x07,0x00,0x00,0x09,0x1F,0x1E,0x02,0x17,0x15,0x02,0x13,0x11,0x02,0x0F,0x0D,0x05,0x02,0x02,0x02,0x02,0x02,0x02}},
{0xC8,6,{0x61,0x00,0x32,0x40,0x54,0x16}},
{0xCA,2,{0xCB,0x43}},
{0xCD,8,{0x0E,0x6E,0x6E,0x25,0x1E,0x6B,0x06,0xB3}},
{0xD2,4,{0xE3,0x2B,0x38,0x00}},
{0xD4,11,{0x00,0x01,0x00,0x0E,0x04,0x44,0x08,0x10,0x00,0x07,0x00}},
{0xE6,8,{0x00,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
{0xF0,5,{0x12,0x03,0x20,0x00,0xFF}},
{0xF3,1,{0x00}},



        // Sleep Mode On
    {0x11,1,{0x00}},
    {REGFLAG_DELAY_MS_V3,150,{0x00}},
    {0x29,1,{0x00}},
    {REGFLAG_DELAY_MS_V3,50,{0x00}},
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
    //LCD_DEBUG("\t\t ft8006 [lcm_get_params]\n");

    memset(params, 0, sizeof(struct LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

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
    params->dsi.vertical_sync_active = 4;
    params->dsi.vertical_backporch = 12;
    params->dsi.vertical_frontporch = 20;
    //params->dsi.vertical_frontporch_for_low_power = 620;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 20;
    params->dsi.horizontal_backporch = 80;
    params->dsi.horizontal_frontporch = 80;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.ssc_disable = 1;
//begin modified by xiongbo.huang fto set fps to 60 temprorily, will change this value according to hardware later
    params->dsi.PLL_CLOCK = 240;
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
        params->corner_pattern_width = 720;//Note:这里是屏幕的宽度，不是原始图片宽度
        params->corner_pattern_height = 16;//圆角的高度
        #endif
//end 20180416 liujunting add for round corner
}

static void push_table(struct LCM_setting_table *table, unsigned int count,
               unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {
        unsigned cmd;

        cmd = table[i].cmd;

        switch (cmd) {

        case REGFLAG_DELAY_MS_V3:
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

        SET_RESET_PIN(0);
        MDELAY(2);
/*#ifdef CONFIG_TCT_APOLLO
        set_gpio_lcd_iovcc(0);
        MDELAY(5);//set up 1.8v for apollo
        set_gpio_lcd_iovcc(1);
        MDELAY(3);//set up 1.8v for apollo
        printk("hxb iovcc power on\n");
#endif*/

    cmd=0x00;
    //data=0x0A; //vsp 5.0v
    data=0x11;  //vsp 5.7V added by jiaxin.pi for lcd VSP datasheet requirements task-11295409
    //VPS=0x00;data=0x0A;VSP=5V,
    //         data=0x0E;VSP=5.4V,
    //VNG=0x01;data=0x0A;VNG=-5V,
    //         data=0x0E;VNG=-5.4V,
#ifndef CONFIG_FPGA_EARLY_PORTING
    set_gpio_lcd_enp(0);
    set_gpio_lcd_enn(0);
    MDELAY(2);

    set_gpio_lcd_enp(1);
    MDELAY(1);

    ret=tps65132_write_bytes(cmd,data);
    if(ret<0)
    LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
    else
        LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);

    MDELAY(1);
    //lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
    set_gpio_lcd_enn(1);
    MDELAY(1);
    cmd=0x01;
    //data=0x0A; //vsn -5.0v
    data=0x11; //vsn -5.7V modified by jiaxin.pi for lcd VSN datasheet requriements task-11295409
    //VPS=0x00;data=0x0A;VSP=5V,
    //         data=0x0E;VSP=5.4V,
    //VNG=0x01;data=0x0A;VNG=-5V,
    //         data=0x0E;VNG=-5.4V,

    ret=tps65132_write_bytes(cmd,data);
    if(ret<0)
        LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
    else
        LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);


#endif
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);//need at least 50ms to have ic sent init code
    push_table(lcm_initialization_setting_ty, sizeof(lcm_initialization_setting_ty) / sizeof(struct LCM_setting_table), 1);
}
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Sleep Mode On
    {0x28,1,{0x00}},
    {REGFLAG_DELAY_MS_V3,20,{0x00}},
    {0x10,1,{0x00}},
    {REGFLAG_DELAY_MS_V3,120,{0x00}},
    {REGFLAG_END_OF_TABLE,1,{0x00}},
};

void lcm_4nd_power_down(void)
{
#if 1
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(5);

    set_gpio_lcd_enn(1);
    set_gpio_lcd_enn(0);
    MDELAY(5);

    set_gpio_lcd_enp(1);
    set_gpio_lcd_enp(0);
    MDELAY(5);
#endif
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
   
 MDELAY(5);
//   SET_RESET_PIN(0);
//suspend timing: MIPI -> RESET -> VSP/VSN -> IOVCOO
    Apollo2_er88577a_qunchuang_kmylar_4nd = 1;
#if 0
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(5);

    set_gpio_lcd_enn(1);
    set_gpio_lcd_enn(0);
    MDELAY(5);

    set_gpio_lcd_enp(1);
    set_gpio_lcd_enp(0);
        MDELAY(5);
#endif
/*#ifdef CONFIG_TCT_APOLLO
        set_gpio_lcd_iovcc(0);
        printk("hxb iovcc power off\n");
        MDELAY(5);
#endif*/



}
static void lcm_resume(void)
{

   // lcm_init();
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);//need at least 50ms to have ic sent init code
    push_table(lcm_initialization_setting_ty, sizeof(lcm_initialization_setting_ty) / sizeof(struct LCM_setting_table), 1);  
//suspend timing: MIPI -> RESET -> VSP/VSN -> IOVCOO
    Apollo2_er88577a_qunchuang_kmylar_4nd = 0;


}

static unsigned int lcm_compare_id(void)
{
    //int id_type=0;

    /*mt_set_gpio_mode(GPIO_LCM_ID0,GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_ID0, GPIO_DIR_IN);
    //mt_set_gpio_pull_select(GPIO_LCM_ID0,GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO_LCM_ID0, GPIO_PULL_DISABLE);// def 0

    mt_set_gpio_mode(GPIO_LCM_ID1,GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_ID1, GPIO_DIR_IN);
    //mt_set_gpio_pull_select(GPIO_LCM_ID1,GPIO_PULL_DOWN);
    mt_set_gpio_pull_enable(GPIO_LCM_ID1, GPIO_PULL_DISABLE);//def 0

    MDELAY(10);
    id_type = mt_get_gpio_in(GPIO_LCM_ID1)<<1 | mt_get_gpio_in(GPIO_LCM_ID0);
    //if (id_type==1)//ID pin=01;*/
            return 1;
    //else
        //return 0;
}
/*
static void lcm_vspn_power(void)
{
    unsigned char cmd = 0x0;
    unsigned char data = 0xFF;
    int ret=0;

    cmd=0x00;
    //data=0x0A; //vsp 5.0v
    data=0x11;  //vsp 5.7V added by jiaxin.pi for lcd VSP datasheet requirements task-11295409
    //VPS=0x00;data=0x0A;VSP=5V,
    //         data=0x0E;VSP=5.4V,
    //VNG=0x01;data=0x0A;VNG=-5V,
    //         data=0x0E;VNG=-5.4V,
#ifndef CONFIG_FPGA_EARLY_PORTING
    set_gpio_lcd_enp(0);
    set_gpio_lcd_enn(0);
    MDELAY(2);

    set_gpio_lcd_enp(1);
    MDELAY(1);

    ret=tps65132_write_bytes(cmd,data);
    if(ret<0)
    LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
    else
        LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);

    MDELAY(1);
    //lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
    set_gpio_lcd_enn(1);
    MDELAY(1);
    cmd=0x01;
    //data=0x0A; //vsn -5.0v
    data=0x11; //vsn -5.7V modified by jiaxin.pi for lcd VSN datasheet requriements task-11295409
    //VPS=0x00;data=0x0A;VSP=5V,
    //         data=0x0E;VSP=5.4V,
    //VNG=0x01;data=0x0A;VNG=-5V,
    //         data=0x0E;VNG=-5.4V,

    ret=tps65132_write_bytes(cmd,data);
    if(ret<0)
        LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
    else
        LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);


#endif



}
*/
struct LCM_DRIVER apollo_er88577a_xy_dsi_vdo_lcm_drv =
{
    .name           = "apollo_er88577a_xy_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,/*kd init fun.*/
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
   // .id             = 4,
   // .vspn_power     = lcm_vspn_power,

};
#endif
/*history modified by xiongbo.huang for gpio modify on 20200407 end*/

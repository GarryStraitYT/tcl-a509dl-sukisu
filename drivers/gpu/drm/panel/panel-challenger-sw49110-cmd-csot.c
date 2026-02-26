
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/input/touch_notify.h>
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_drm_graphics_base.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_panel_ext.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

#if defined(CONFIG_LEDS_MTK_I2C)
#include "../../../misc/mediatek/leds/leds-mtk-i2c.h"
#endif

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
#endif

#define LOG_TAG "LCM"
#ifdef BUILD_LK
	#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
	#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
	#define LCM_LOGI(fmt, args...)  pr_info("[KERNEL/"LOG_TAG"]"fmt, ##args)
	#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif
//begin add by jiaxin.pi for CHLRTMO-4678 on 20210904
#define PHYSICAL_WIDTH              69500
#define PHYSICAL_HEIGHT             158500
//end add by jiaxin.pi for CHLRTMO-4678 on 20210904
struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
#if defined(CONFIG_LEDS_MTK_I2C)
	struct gpio_desc *pm_enable_gpio;
#endif
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;

	bool prepared;
	bool enabled;
	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})
//begin add by jiaxin.pi for CHLRTMO-4992 on 20210926
static bool double_wakeup_enable = 0;
extern void (*tct_set_double_wakeup_en)(unsigned int enable);
static void double_wakeup(unsigned int enable)
{
	 if(enable)
	  double_wakeup_enable = 1;
         else
          double_wakeup_enable = 0;
}
//end add by jiaxin.pi for CHLRTMO-4992 on 20210926
#if defined(CONFIG_BIAS_TPS65132_I2C)
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


static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	//LCM_LOGD("%s()+ \n", __func__);

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	LCM_LOGD("%s()+ \n", __func__);

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;
	LCM_LOGD("%s()+ \n", __func__);

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;


static int lcm_panel_bias_regulator_init(void)
{
	static int regulator_inited;
	int ret = 0;
	LCM_LOGD("%s()+ \n", __func__);

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_bias_enable(void)
{
	int ret = 0;
	int retval = 0;
	LCM_LOGD("%s()+ \n", __func__);

	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_bias_disable(void)
{
	int ret = 0;
	int retval = 0;
	LCM_LOGD("%s()+ \n", __func__);

	lcm_panel_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{
    unsigned int cur_panel_mode = 3;
	LCM_LOGI("%s()+ \n", __func__);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
    touch_notifier_call_chain(LCD_EVENT_LCD_UNBLANK,NULL);
    //begin add by jiaxin for CHLRTMO-4646 on 20211008
	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

    touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
    mdelay(5);
    touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
	//udelay(10 * 1000);
    mdelay(5);
    //end add by jiaxin for CHLRTMO-4646 on 20211008
	//LCM_LOGD("%s() reset_gpio=%d \n", __func__, gpiod_get_value(ctx->reset_gpio));

	//----------------------LCD initial code start----------------------//
	lcm_dcs_write_seq_static(ctx,0x26,0x01);
    lcm_dcs_write_seq_static(ctx,0x2A,0x00,0x00,0x04,0x37);
    lcm_dcs_write_seq_static(ctx,0x2B,0x00,0x00,0x09,0x9B);
    lcm_dcs_write_seq_static(ctx,0x44,0x05,0xDC);
    lcm_dcs_write_seq_static(ctx,0x51,0xFF);
    lcm_dcs_write_seq_static(ctx,0x53,0x2C);
    lcm_dcs_write_seq_static(ctx,0x55,0x80);
    lcm_dcs_write_seq_static(ctx,0xB0,0xAC);
    lcm_dcs_write_seq_static(ctx,0xB1,0x36,0x00,0x80,0x14,0x85);
    lcm_dcs_write_seq_static(ctx,0xB2,0x77,0x04,0x4C);
    lcm_dcs_write_seq_static(ctx,0xB3,0x02,0x0D,0x0A,0x00,0x5C,0x00,0x02,0x12);
    lcm_dcs_write_seq_static(ctx,0xB4,0x12,0x42,0x78,0x0F,0x1A,0xE0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4E,0x96,0x00,0x00,0x00,0x04,0x24,0x34,0xC8,0x00,0x04,0x04,0x96,0x25,0x02,0x05,0x01,0x64,0x15,0x00);	 
    lcm_dcs_write_seq_static(ctx,0xB5,0x02,0x0F,0x07,0x01,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x05,0x44,0xEE,0x08,0xAB,0x53,0x19,0x08,0xAB,0x53,0x19,0x11,0x00);
    lcm_dcs_write_seq_static(ctx,0xB6,0x00,0x38,0x14,0x08,0x04,0x10,0x08);
    lcm_dcs_write_seq_static(ctx,0xB7,0x00,0x50,0x66,0x02);
    //add by jiaxin.pi for CHLRTMO-7758 on 20211103
	lcm_dcs_write_seq_static(ctx,0xB8,0x07,0x01,0xF4,0x40,0x22,0x00,0x00,0x00,0x00,0x50,0x02,0x04,0x01,0x46,0xCC,0x44,0xB4,0x00,0x00,0x00,0x00,0x10,0x02,0x04,0x01,0x0D,0x40,0x58,0x61,0x31,0x6E);
    //end by jiaxin.pi for CHLRTMO-7758 on 20211103
	lcm_dcs_write_seq_static(ctx,0xB9,0x32,0x32,0x2A,0x37,0x03);
    lcm_dcs_write_seq_static(ctx,0xFA,0xFF,0x22,0x22,0x22,0x20,0x00,0x70,0x84,0x02,0x60,0x09,0x9C,0x42,0x1C);
    lcm_dcs_write_seq_static(ctx,0xC0,0x87,0x0A,0x02);
    lcm_dcs_write_seq_static(ctx,0xC3,0x05,0x06,0x06,0x50,0x66,0x1B);
    lcm_dcs_write_seq_static(ctx,0xC4,0xA2,0x9A,0x90);
    lcm_dcs_write_seq_static(ctx,0xC5,0x94,0x04,0x6C,0x33,0x33);
    lcm_dcs_write_seq_static(ctx,0xCA,0x05,0x0D,0x00,0x14,0xFA,0xFF,0x55,0x55,0x15,0xFE,0x55,0xFE,0x00,0xFE,0x00,0xFE,0x15,0xFE,0x15,0xFE,0x00,0xFE);
    lcm_dcs_write_seq_static(ctx,0xCB,0x7F,0x3B,0xF0,0xA0,0x37);
    lcm_dcs_write_seq_static(ctx,0xCC,0xF3,0x10,0x55,0x3D,0x3D,0x11);
    lcm_dcs_write_seq_static(ctx,0xCD,0x15,0x10,0x50,0x10,0x11,0xF3,0x10,0x61);
    lcm_dcs_write_seq_static(ctx,0xCE,0x48,0x48,0x1A,0x10,0x00,0xAB);
    //begin add by jiaxin.pi for CHLRTMO-10546 on 20211215
    lcm_dcs_write_seq_static(ctx,0xD0,0x0C,0x0C,0x13,0x13,0x1F,0x1F,0x2C,0x2C,0x37,0x37,
                                 0x40,0x40,0x5A,0x5A,0x72,0x72,0x84,0x84,0x93,0x93,
                                 0x61,0x61,0xD0,0xD0,0xBF,0xBF,0xA9,0xA9,0x89,0x89,
                                 0x65,0x65,0x57,0x57,0x44,0x44,0x30,0x30,0x1E,0x1E,
                                 0x00,0x00,0x0C,0x0C,0x13,0x13,0x1F,0x1F,0x2C,0x2C,
                                 0x37,0x37,0x40,0x40,0x5A,0x5A,0x72,0x72,0x84,0x84,
                                 0x93,0x93,0x61,0x61,0xD0,0xD0,0xBF,0xBF,0xA9,0xA9,
                                 0x89,0x89,0x65,0x65,0x57,0x57,0x44,0x44,0x30,0x30,
                                 0x1E,0x1E,0x00,0x00,0x0C,0x0C,0x13,0x13,0x1F,0x1F,
                                 0x2C,0x2C,0x37,0x37,0x40,0x40,0x5A,0x5A,0x72,0x72,
                                 0x84,0x84,0x93,0x93,0x61,0x61,0xD0,0xD0,0xBF,0xBF,
                                 0xA9,0xA9,0x89,0x89,0x65,0x65,0x57,0x57,0x44,0x44,
                                 0x30,0x30,0x1E,0x1E,0x00,0x00);
    //end add by jiaxin.pi for CHLRTMO-10546 on 20211215
    lcm_dcs_write_seq_static(ctx,0xE5,0x11,0x0B,0x0A,0x10,0x06,0x02,0x25,0x28,0x0E,0x25,0x25,0x25);
    lcm_dcs_write_seq_static(ctx,0xE6,0x11,0x0B,0x0A,0x10,0x09,0x05,0x25,0x28,0x0E,0x25,0x25,0x25);
    lcm_dcs_write_seq_static(ctx,0xE7,0x58,0x56,0x54,0x57,0x55,0x53,0x18,0x16,0x14,0x17,0x15,0x13);
    lcm_dcs_write_seq_static(ctx,0xE8,0x5E,0x5C,0x5A,0x5D,0x5B,0x59,0x1E,0x1C,0x1A,0x1D,0x1B,0x19);
	
	//SLPOUT and DISPON
	lcm_dcs_write_seq_static(ctx, 0x11);
//begin add by jiaxin.pi for CHLRTMO-4646 on 20210929
	mdelay(120);
	lcm_dcs_write_seq_static(ctx, 0x35,0x00);
	lcm_dcs_write_seq_static(ctx, 0x29);
	
	mdelay(10);
//end add by jiaxin.pi for CHLRTMO-4646 on 20210929
    touch_notifier_call_chain(LCD_EVENT_LCD_MODE, (void *)&cur_panel_mode);
    printk("pige exit lcm_panel_init\n");
	/* write   display brightness*/
	/* lcm_dcs_write_seq_static(ctx, 0x51, 0xFF);*/
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	LCM_LOGD("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
    unsigned int cur_panel_mode = 0;
	LCM_LOGD("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);

	if (!ctx->prepared)
		return 0;
//begin add by jiaxin.pi for CHLRTMO-4646 on 20210913
	if(double_wakeup_enable){
    touch_notifier_call_chain(LCD_EVENT_LCD_BLANK, NULL);
    udelay(1000);
	}
//end add by jiaxin.pi for CHLRTMO-4646 on 20210913
	lcm_dcs_write_seq_static(ctx, 0x28);
        //begin add by jiaxin for CHLRTMO-4646 on 20211008
	mdelay(10);
	lcm_dcs_write_seq_static(ctx, 0x10);
	mdelay(120);

	lcm_dcs_write_seq_static(ctx, 0xE7,0x18,0x16,0x14,0x17,0x15,0x13,0x58,0x56,0x54,0x57,0x55,0x53);
	lcm_dcs_write_seq_static(ctx, 0xE8,0x1E,0x1C,0x1A,0x1D,0x1B,0x19,0x5E,0x5C,0x5A,0x5D,0x5B,0x59);

	mdelay(5);
//begin add by jiaxin.pi for CHLRTMO-6035 on 20211119
	if(double_wakeup_enable){
	touch_notifier_call_chain(LCD_EVENT_LCD_MODE, (void *)&cur_panel_mode);
    udelay(1000);
	}
//begin add by jiaxin.pi for CHLRTMO-6035 on 20211119
	if(0)
        touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
        //end add by jiaxin for CHLRTMO-4646 on 20211008
	ctx->error = 0;
	ctx->prepared = false;
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#else
#if defined(CONFIG_LEDS_MTK_I2C)
	/*this is rt4831a*/
	mtk_leds_deinit_power();
	lcm_i2c_write_bytes(0x09, 0x18);
	ctx->pm_enable_gpio = devm_gpiod_get(ctx->dev,
		"pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get pm-enable %ld\n",
			__func__, PTR_ERR(ctx->pm_enable_gpio));
		return PTR_ERR(ctx->pm_enable_gpio);
	}
	gpiod_set_value(ctx->pm_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->pm_enable_gpio);
#else
	udelay(1000);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
//begin add by jiaxin.pi for CHLRTMO-4992 on 20210926
        if(0)
//end add by jiaxin.pi for CHLRTMO-4992 on 20210926
	gpiod_set_value(ctx->reset_gpio, 0);
	//gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
//begin add by jiaxin.pi for CHLRTMO-4646 on 20210913
	udelay(5000);
//end add by jiaxin.pi for CHLRTMO-4646 on 20210913
	LCM_LOGD("%s() reset_gpio=%d \n", __func__, gpiod_get_value(ctx->reset_gpio));
	
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
//begin add by jiaxin.pi for CHLRTMO-4992 on 20210926
        if(0)
//end add by jiaxin.pi for CHLRTMO-4992 on 20210926
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(5000);
	LCM_LOGD("%s() bias_neg=%d \n", __func__, gpiod_get_value(ctx->bias_neg));

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
//begin add by jiaxin.pi for CHLRTMO-4992 on 20210926
        if(0)
//end add by jiaxin.pi for CHLRTMO-4992 on 20210926
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(1000);
	LCM_LOGD("%s() bias_pos=%d \n", __func__, gpiod_get_value(ctx->bias_pos)); 
#endif
#endif

	pr_info("%s-\n", __func__);
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	LCM_LOGD("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);

	if (ctx->prepared)
		return 0;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#else
#if defined(CONFIG_LEDS_MTK_I2C)
	/*rt4831a co-work with leds_i2c*/
	ctx->pm_enable_gpio = devm_gpiod_get(ctx->dev,
		"pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get pm-enable %ld\n",
			__func__, PTR_ERR(ctx->pm_enable_gpio));
		return PTR_ERR(ctx->pm_enable_gpio);
	}
	gpiod_set_value(ctx->pm_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->pm_enable_gpio);
	lcm_i2c_write_bytes(0x0a, 0x11);
	lcm_i2c_write_bytes(0x0b, 0x00);
	/*set bias to 5.4v*/
	lcm_i2c_write_bytes(0x0c, 0x24);
	lcm_i2c_write_bytes(0x0d, 0x1c);
	lcm_i2c_write_bytes(0x0e, 0x1c);
	/*bias enable*/
	lcm_i2c_write_bytes(0x09, 0x9e);
	mtk_leds_init_power();
#else
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(5 * 1000); //delay at least 3ms
	//LCM_LOGD("%s() bias_pos=%d \n", __func__, gpiod_get_value(ctx->bias_pos));

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);
	//LCM_LOGD("%s() bias_neg=%d \n", __func__, gpiod_get_value(ctx->bias_neg));
//begin add by jiaxin.pi for CHLRTMO-4646 on 20210929
#if defined(CONFIG_BIAS_TPS65132_I2C)
	_lcm_i2c_write_bytes(0x0, 0x10); //+5.6V
	udelay(5000);
	_lcm_i2c_write_bytes(0x1, 0x10); //-5.6V
	udelay(5000);
#endif
#endif
#endif
//end add by jiaxin.pi for CHLRTMO-4646 on 20210929
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	pr_info("%s-\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	LCM_LOGD("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}
//begin add by jiaxin.pi for CHLRTMO-4711 on 20210908
//end add by jiaxin.pi for CHLRTMO-4711 on 20210908
#define HFP_60HZ (24)
#define HSA_60HZ (4)
#define HBP_60HZ (24)
#define VFP_60HZ (20)
#define VSA_60HZ (4)
#define VBP_60HZ (20)

#define VAC (2460)
#define HAC (1080)
static u32 fake_heigh = 0;
static u32 fake_width = 0;

static bool need_fake_resolution = false;
static int current_fps = 60; //default 60hz
//begin add by jiaxin.pi for CHLRTMO-4711 on 20210908
//end add by jiaxin.pi for CHLRTMO-4711 on 20210908

static struct drm_display_mode lowFPS_mode = {
	.clock = 573000,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP_60HZ,
	.hsync_end = HAC + HFP_60HZ + HSA_60HZ,
	.htotal = HAC + HFP_60HZ + HSA_60HZ + HBP_60HZ,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_60HZ,
	.vsync_end = VAC + VFP_60HZ + VSA_60HZ,
	.vtotal = VAC + VFP_60HZ + VSA_60HZ + VBP_60HZ,
	.vrefresh = 60,
};


#if defined(CONFIG_MTK_PANEL_EXT)
//begin add by jiaxin.pi for CHLRTMO-4711 on 20210908
		/*.vact_timing_fps = 60,
	},
};*/
//end add by jiaxin.pi for CHLRTMO-4711 on 20210908

static struct mtk_panel_params ext_params_60hz = {
	.pll_clk = 573,
	.data_rate = 1146,
	//.vfp_low_power = VFP_90HZ,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
//begin add by jiaxin.pi for CHLRTMO-4678 on 20210904
        .physical_width_um = PHYSICAL_WIDTH,
  	.physical_height_um = PHYSICAL_HEIGHT,
//end add by jiaxin.pi for CHLRTMO-4678 on 20210904
	.dyn_fps = {
		.switch_en = 1,
		/* switch cmd send in mode_switch_to_90()
		//60->90 normal
		//CMD2 ENABLE
		.dfps_cmd_table[0] = {0, 2, {0x00,0x00} },
		.dfps_cmd_table[1] = {0, 4, {0xFF,0x87,0x56,0x01} },
		.dfps_cmd_table[2] = {0, 2, {0x00,0xd1} },
		.dfps_cmd_table[3] = {0, 2, {0xc0,0x83} },
		.dfps_cmd_table[4] = {0, 2, {0x00,0xd2} },
		.dfps_cmd_table[5] = {0, 2, {0xc0,0x00} },
		//CMD2 DISENABLE
		.dfps_cmd_table[6] = {0, 2, {0x00,0x00} },
		.dfps_cmd_table[7] = {0, 4, {0xFF,0xFF,0xFF,0xFF} },*/
		.vact_timing_fps = 60,
	},
};


static int lcm_panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);
	LCM_LOGI("%s()+ on=%d\n", __func__, on);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	udelay(1000);
	LCM_LOGD("%s() reset_gpio=%d \n", __func__, gpiod_get_value(ctx->reset_gpio));

	return 0;
}


static struct drm_display_mode *lcm_get_mode_by_id(struct drm_panel *panel,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &panel->connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}
//begin add by jiaxin.pi for CHLRTMO-4711 on 20210908




	/*	lcm_mode_switch_to_60(panel);
	} else if (m->vrefresh == 90) { /* 60 switch to 90*/
	/*	lcm_mode_switch_to_90(panel);
	} else
		ret = 1;

	return ret;
}*/


static struct mtk_panel_funcs ext_funcs = {
	.reset = lcm_panel_ext_reset,
	/*.ext_param_set = lcm_panel_ext_param_set,
	.mode_switch = lcm_panel_mode_switch,*/
	//.get_virtual_heigh = lcm_get_virtual_heigh,
	//.get_virtual_width = lcm_get_virtual_width,
};
#endif
//end add by jiaxin.pi for CHLRTMO-4711 on 20210908
struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *	   become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *	  display the first valid frame after starting to receive
	 *	  video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *	   turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *		 to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	if (fake_heigh > 0 && fake_heigh < VAC) {
		mode->vsync_start = mode->vsync_start - mode->vdisplay
					+ fake_heigh;
		mode->vsync_end = mode->vsync_end - mode->vdisplay + fake_heigh;
		mode->vtotal = mode->vtotal - mode->vdisplay + fake_heigh;
		mode->vdisplay = fake_heigh;
	}
	if (fake_width > 0 && fake_width < HAC) {
		mode->hsync_start = mode->hsync_start - mode->hdisplay
					+ fake_width;
		mode->hsync_end = mode->hsync_end - mode->hdisplay + fake_width;
		mode->htotal = mode->htotal - mode->hdisplay + fake_width;
		mode->hdisplay = fake_width;
	}
}

//begin add by jiaxin.pi for CHLRTMO-4688 on 20210907
/* Begin add for LCM device info */
#ifdef CONFIG_TCT_PROJECT_CHALLENGER_STYLU
extern char LCM_module_name[256];
#endif
/* End add for LCM device info */

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	//struct drm_display_mode *mode2;

	LCM_LOGD("%s()+ \n", __func__);

	if (need_fake_resolution){
		change_drm_disp_mode_params(&lowFPS_mode);
	//	change_drm_disp_mode_params(&performance_mode);
	}

//add 60hz mode
	mode = drm_mode_duplicate(panel->drm, &lowFPS_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			lowFPS_mode.hdisplay, lowFPS_mode.vdisplay,
			lowFPS_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

//add 90hz mode

	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 129;
//begin add by jiaxin.pi for CHLRTMO-4688 on 20210907
#ifdef CONFIG_TCT_PROJECT_CHALLENGER_STYLU
	sprintf(LCM_module_name, "SW49110:TianMa:1080*2460:AUC0680110C1");
#endif
//end add by jiaxin.pi for CHLRTMO-4688 on 20210907
	return 1;
}
//end add by jiaxin.pi for CHLRTMO-4688 on 20210907
static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static void check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;
	LCM_LOGD("%s()+ \n", __func__);

	ret = of_property_read_u32(dev->of_node, "fake_heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake_width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < VAC)
		need_fake_resolution = false; //true;
	if (fake_width > 0 && fake_width < HAC)
		need_fake_resolution = false; //true;
}

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	pr_info("%s()+ :sw4119 \n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight){
			dev_err(dev, "%s: cannot get backlight %ld\n", __func__, PTR_ERR(ctx->backlight));
			return -EPROBE_DEFER;
		}
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
#ifndef CONFIG_LEDS_MTK_I2C
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);
#endif
	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_60hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	check_is_need_fake_resolution(dev);
	printk("pige %s()-\n", __func__);
//begin add by jiaxin.pi for CHLRTMO-4992 on 20210926
        tct_set_double_wakeup_en = double_wakeup;
//end add by jiaxin.pi for CHLRTMO-4992 on 20210926
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
	LCM_LOGI("%s()+ \n", __func__);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "challenger,sw49110,csot", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-challenger-sw49110-csot",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Tai-Hua Tseng <tai-hua.tseng@mediatek.com>");
MODULE_DESCRIPTION("challenger sw49110 csot LCD Panel Driver");
MODULE_LICENSE("GPL v2");

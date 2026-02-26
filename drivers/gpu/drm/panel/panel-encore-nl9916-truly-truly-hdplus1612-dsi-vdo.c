
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

#include "include/panel-lcd-power.h"

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

static struct drm_display_mode *lcm_get_mode_by_id(struct drm_panel *panel,unsigned int mode);

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
	LCM_LOGI("%s()+ \n", __func__);
	

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return;
	}

	//start set sequence
	//Begin add by bing-zhang for ENCORETF-31 on 2022/06/29
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	udelay(5000);
	_lcm_i2c_write_bytes(0x0, 0x13); //+5.9V

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}

	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	mdelay(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(15);

	lcm_dcs_write_seq_static(ctx,0xF0,0x5A,0x59);
	lcm_dcs_write_seq_static(ctx,0xF1,0xA5,0xA6);
	lcm_dcs_write_seq_static(ctx,0xBB,0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x52);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return;
	}

	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	udelay(5000);
	_lcm_i2c_write_bytes(0x1, 0x13); //-5.9V
	udelay(1000);

	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	mdelay(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	mdelay(15);
	//End add by bing-zhang for ENCORETF-31 on 2022/06/29

#if defined(CONFIG_BIAS_TPS65132_I2C)
	//_lcm_i2c_write_bytes(0x0, 0x12); //+5.8V
	//udelay(1000);
	//_lcm_i2c_write_bytes(0x1, 0x12); //-5.8V
	//udelay(1000);
#endif

	//----------------------LCD initial code start----------------------//
	lcm_dcs_write_seq_static(ctx,0xF0,0x5A,0x59);
	lcm_dcs_write_seq_static(ctx,0xF1,0xA5,0xA6);
	lcm_dcs_write_seq_static(ctx,0xD4,0x31);
	lcm_dcs_write_seq_static(ctx,0xC7,0x76,0x54,0x32,0x22,0x23,0x45,0x67,0x76,0x30,0x76,0x54,0x32,0x22,0x23,0x45,0x67,0x76,0x30,0x31,0x00,0x01,0xFF,0xFF,0x00,0x0E,0x0E,0x43,0x00,0x00,0x00,0x00,0x00);//VOP=5.5V
	lcm_dcs_write_seq_static(ctx,0xC1,0x10,0x20,0x25,0x22,0x04,0x28,0x28,0x04,0x4C,0x06,0x22,0x70,0x33,0x31,0x07,0x11,0x84,0x4C,0x00,0x93,0x1C,0x21,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xC4,0x04,0x33,0xB8,0x40,0x00,0xBC,0x00,0xA0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0xE0,0x20,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xC5,0x03,0x23,0x96,0xC8,0x32,0x00,0x05,0x02,0x12,0x0C,0x04,0x32,0x3F,0x08,0x01,0x10,0x04,0x1C,0xC8,0x00,0x0A,0x14,0x01,0x14,0x38,0x7F,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xC6,0x89,0x24,0x17,0x2B,0x2B,0x28,0x3F,0x03,0x16,0x16,0x00,0x01,0x40,0x00,0x98,0x98,0x60,0x80,0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xCA,0x00,0x40,0x00,0x19,0x46,0x94,0x41,0x8F,0x44,0x44,0x50,0x50,0x5A,0x5A,0x64,0x64,0x32,0x32,0x11,0x00,0x01,0x01,0x0A,0x06,0x22,0x00,0x05,0x00,0x00,0x64,0x32,0x04);//VGHO=15V ,VGLO=-10V
	lcm_dcs_write_seq_static(ctx,0xB2,0x8C,0x8D,0x10,0x10,0x66,0x66,0x0A,0x06,0x66,0x70,0x71,0x70,0x71,0x70,0x71,0x70,0x71,0x70,0x71,0x70,0x71,0x00,0x00,0x00,0x00,0x55,0x55,0x10,0x20,0x11,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xB3,0xB4,0x07,0x01,0x07,0x81,0x80,0x18,0x2C,0x6B,0x0B,0x0B,0x00,0x08,0x44,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xB7,0x00,0x00,0x28,0xC0,0x04,0x06,0x08,0xC1,0x0C,0x0C,0x0E,0x0E,0x10,0x10,0x12,0x12,0x14,0x14,0x16,0x16,0x00,0x02,0xFF,0xFF,0x00,0x00,0x00,0x00,0x3C,0x00);
	lcm_dcs_write_seq_static(ctx,0xB6,0x00,0x00,0x29,0xC0,0x05,0x07,0x09,0xC1,0x0D,0x0D,0x0F,0x0F,0x11,0x11,0x13,0x13,0x15,0x15,0x17,0x17,0x00,0x02,0xFF,0xFF,0x00,0x00,0x00,0x00,0x3C,0x00);
	lcm_dcs_write_seq_static(ctx,0xBB,0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x52);
	lcm_dcs_write_seq_static(ctx,0xBC,0x00,0x00,0x00,0x00,0x04,0x00,0xFF,0xF0,0x0B,0x13,0x50,0x5B,0x33,0x33,0x00);
	lcm_dcs_write_seq_static(ctx,0xBD,0xA1,0xA2,0x52,0x2E,0x00,0x8F);
	lcm_dcs_write_seq_static(ctx,0xBE,0x0C,0x88,0x43,0x38,0x33,0x00,0x00,0x38,0x00,0xB2,0xAF,0xB2,0xAF,0x00,0x00,0x33);
	lcm_dcs_write_seq_static(ctx,0xBF,0x0C,0x19,0x0C,0x19,0x00,0x11,0x04,0x18,0x50);
	lcm_dcs_write_seq_static(ctx,0xFB,0x5E,0x2C);//VGH=16V ,VGL=-11V
	lcm_dcs_write_seq_static(ctx,0xF1,0x5A,0x59);
	lcm_dcs_write_seq_static(ctx,0xF0,0xA5,0xA6);
	lcm_dcs_write_seq_static(ctx,0x35,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0x11,0x00,0x00);
	mdelay(120);
	lcm_dcs_write_seq_static(ctx,0x29,0x00,0x00);
	mdelay(20);
	lcm_dcs_write_seq_static(ctx,0x6D,0x02,0x00);

	LCM_LOGI("%s()- \n", __func__);
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
	LCM_LOGD("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx,0xF0,0x5A,0x59);
	lcm_dcs_write_seq_static(ctx,0xF1,0xA5,0xA6);
	lcm_dcs_write_seq_static(ctx,0x6D,0x25);
	lcm_dcs_write_seq_static(ctx,0xBB,0x01,0x02,0x03,0x0A,0x04,0x13,0x14,0x12);
	
	lcm_dcs_write_seq_static(ctx, 0x28);
	mdelay(20);
	lcm_dcs_write_seq_static(ctx, 0x10,0x00,0x00,0x00);
	mdelay(120);
	
	lcm_dcs_write_seq_static(ctx,0xC9,0x01,0x00,0x00);

	ctx->error = 0;
	ctx->prepared = false;

	//Begin modify by bing-zhang for ENCORETF-5622 on 2022/07/18
	msleep(150);
	//End modify by bing-zhang for ENCORETF-5622 on 2022/07/18
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	//Begin modify by bing-zhang for ENCORETF-5622 on 2022/07/18
	gpiod_set_value(ctx->reset_gpio, 1);
	//End modify by bing-zhang for ENCORETF-5622 on 2022/07/18
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	udelay(5000);

	LCM_LOGD("%s() reset_gpio=%d \n", __func__, gpiod_get_value(ctx->reset_gpio));
	
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}

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

	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(5000);
	LCM_LOGD("%s() bias_pos=%d \n", __func__, gpiod_get_value(ctx->bias_pos)); 


	pr_info("%s-\n", __func__);
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	LCM_LOGI("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);
	if (ctx->prepared)
		return 0;

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




#define HFP (98)//72
#define HSA (4)
#define HBP (98)//72
#define VFP_90HZ (290)
#define VFP_60HZ (1200)
#define VSA (4)
#define VBP (32)

#define VAC (1612)          //1918
#define HAC (720)          //838
static u32 fake_heigh = 0;
static u32 fake_width = 0;

static bool need_fake_resolution = false;
static int current_fps = 90; //default 60hz




static struct drm_display_mode lowFPS_mode = {
	.clock = 597000,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_60HZ,
	.vsync_end = VAC + VFP_60HZ + VSA,
	.vtotal = VAC + VFP_60HZ + VSA + VBP,
	.vrefresh = 60,
};

static struct drm_display_mode highFPS_mode = {
	.clock = 478000,//on used
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_90HZ,
	.vsync_end = VAC + VFP_90HZ + VSA,
	.vtotal = VAC + VFP_90HZ + VSA + VBP,
	.vrefresh = 90,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params_60hz = {
	.pll_clk = 517,
	.data_rate = 1034,//828,    // use data rate to ctl mipi clk  90HZ
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
		.vact_timing_fps = 60,
	},

	.phy_timcon = {
                .lpx = 7,
	},
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 517,  //on used
	.data_rate = 1034,//828,    // use data rate to ctl mipi clk  90HZ
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
		.vact_timing_fps = 90,
	},

	.phy_timcon = {
                .lpx = 7,
	},
};

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = lcm_get_mode_by_id(panel, mode);

	if (m->vrefresh == 60)
		ext->params = &ext_params_60hz;
	else if (m->vrefresh == 90)
		ext->params = &ext_params_90hz;
	else
		ret = 1;
	if (!ret)
		current_fps = m->vrefresh;
	return ret;
}

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







static struct mtk_panel_funcs ext_funcs = {
	.reset = lcm_panel_ext_reset,
	.ext_param_set = mtk_panel_ext_param_set,
	//.get_virtual_heigh = lcm_get_virtual_heigh,
	//.get_virtual_width = lcm_get_virtual_width,
};
#endif

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


/* Begin add for LCM device info */
#ifdef CONFIG_TCT_DEVICEINFO
extern char LCM_module_name[256];
#endif
/* End add for LCM device info */

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
        struct drm_display_mode *mode_1;


	LCM_LOGD("%s()+ \n", __func__);

	if (need_fake_resolution){
		change_drm_disp_mode_params(&lowFPS_mode);

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

//add 60hz mode
	mode_1 = drm_mode_duplicate(panel->drm, &highFPS_mode);
	if (!mode_1) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			highFPS_mode.hdisplay, highFPS_mode.vdisplay,
			highFPS_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode_1);
	mode->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode_1);


	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 129;

#ifdef CONFIG_TCT_DEVICEINFO
	sprintf(LCM_module_name, "ICNL9916:TRULY:1st:AUC0660117C1");
#endif

	return 2;
}

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
	LCM_LOGD("%s()+ \n", __func__);

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
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
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
	ret = mtk_panel_ext_create(dev, &ext_params_90hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	check_is_need_fake_resolution(dev);
	pr_info("%s()-\n", __func__);

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
	{ .compatible = "truly,icnl9916-6f", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-encore-nl9916-truly-truly-hdplus1612-dsi-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Tai-Hua Tseng <tai-hua.tseng@mediatek.com>");
MODULE_DESCRIPTION("ICNL9916 KD AUO FHD VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");

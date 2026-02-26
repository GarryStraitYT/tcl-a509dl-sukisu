
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
	
	//LCM_LOGD("%s() bias_pos=%d \n", __func__, gpiod_get_value(ctx->bias_pos));

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return;
	}
	
	
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}


	//start set sequence
	
	gpiod_set_value(ctx->reset_gpio, 0);
	mdelay(1);
	
	
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	udelay(5 * 1000); //delay at least 3ms
	
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	udelay(1000);
	//LCM_LOGD("%s() bias_neg=%d \n", __func__, gpiod_get_value(ctx->bias_neg));

	_lcm_i2c_write_bytes(0x0, 0x14); //+6V
	udelay(1000);
	_lcm_i2c_write_bytes(0x1, 0x14); //-6V
	udelay(1000);

	

	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
    mdelay(12);
	//udelay(10 * 1000);
	//LCM_LOGD("%s() reset_gpio=%d \n", __func__, gpiod_get_value(ctx->reset_gpio));

	//----------------------LCD initial code start----------------------//
        lcm_dcs_write_seq_static(ctx,0x00,0x00);
		lcm_dcs_write_seq_static(ctx,0xFF,0x80,0x57,0x01); 
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xFF,0x80,0x57);
		lcm_dcs_write_seq_static(ctx,0x00,0xA3);  //Y=1612
		lcm_dcs_write_seq_static(ctx,0xB3,0x06,0x4C,0x00,0x18);
		lcm_dcs_write_seq_static(ctx,0x00,0x93);//VGH_N 16V
		lcm_dcs_write_seq_static(ctx,0xC5,0x61);
		lcm_dcs_write_seq_static(ctx,0x00,0x97);//VGH_I 16V
		lcm_dcs_write_seq_static(ctx,0xC5,0x61);
		lcm_dcs_write_seq_static(ctx,0x00,0x9A);//VGL_N -16V,x3 pump
		lcm_dcs_write_seq_static(ctx,0xC5,0xE9);
		lcm_dcs_write_seq_static(ctx,0x00,0x9C);//VGL_I -16V,x3 pump 
		lcm_dcs_write_seq_static(ctx,0xC5,0xE9);
		lcm_dcs_write_seq_static(ctx,0x00,0xB6); //VGHO1_N_I=14v
		lcm_dcs_write_seq_static(ctx,0xC5,0x43,0x43);
		lcm_dcs_write_seq_static(ctx,0x00,0xB8); //VGLO1_N_I -14V
		lcm_dcs_write_seq_static(ctx,0xC5,0x55,0x55); 
		lcm_dcs_write_seq_static(ctx,0x00,0x00); 
		lcm_dcs_write_seq_static(ctx,0xD8,0x36,0x36);  //GVDDP/N 5.55V  -5.55V
		lcm_dcs_write_seq_static(ctx,0x00,0x82); 
		lcm_dcs_write_seq_static(ctx,0xC5,0x55);  //LVD
		lcm_dcs_write_seq_static(ctx,0x00,0x83); 
		lcm_dcs_write_seq_static(ctx,0xC5,0x07);  //LVD Enable
		lcm_dcs_write_seq_static(ctx,0x00,0x96);
		lcm_dcs_write_seq_static(ctx,0xf5,0x0d);
		lcm_dcs_write_seq_static(ctx,0x00,0x86);
		lcm_dcs_write_seq_static(ctx,0xf5,0x0d);
		lcm_dcs_write_seq_static(ctx,0x00,0x94);
		lcm_dcs_write_seq_static(ctx,0xC5,0x15);
		lcm_dcs_write_seq_static(ctx,0x00,0x9B);
		lcm_dcs_write_seq_static(ctx,0xC5,0x51);
		lcm_dcs_write_seq_static(ctx,0x00,0xA3);  //GVDD_EN
		lcm_dcs_write_seq_static(ctx,0xA5,0x04); 
		lcm_dcs_write_seq_static(ctx,0x00,0x99);  
		lcm_dcs_write_seq_static(ctx,0xCF,0x56); 
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xC0,0x00 ,0xD2 ,0x00 ,0x2A ,0x00 ,0x1C);
		lcm_dcs_write_seq_static(ctx,0x00,0xA0);
		lcm_dcs_write_seq_static(ctx,0xC0,0x00 ,0xD2 ,0x00 ,0x2A ,0x00 ,0x1C);
		lcm_dcs_write_seq_static(ctx,0x00,0xB0);
		lcm_dcs_write_seq_static(ctx,0xC0,0x01 ,0x10 ,0x00 ,0x2A ,0x1C);
		lcm_dcs_write_seq_static(ctx,0x00,0xC1);
		lcm_dcs_write_seq_static(ctx,0xC0,0x01 ,0x33 ,0x01 ,0x0A ,0x00 ,0xCD ,0x01 ,0x90);
		lcm_dcs_write_seq_static(ctx,0x00,0x70);
		lcm_dcs_write_seq_static(ctx,0xC0,0x00 ,0x7F ,0x00 ,0x2A ,0x00 ,0x1C);
		lcm_dcs_write_seq_static(ctx,0x00,0xA3);
		lcm_dcs_write_seq_static(ctx,0xC1,0x00, 0x33, 0x00 ,0x3C ,0x00 ,0x02);
		lcm_dcs_write_seq_static(ctx,0x00,0xB7);
		lcm_dcs_write_seq_static(ctx,0xC1,0x00 ,0x33);
		lcm_dcs_write_seq_static(ctx,0x00,0x73);
		lcm_dcs_write_seq_static(ctx,0xCE,0x09 ,0x09);
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xCE,0x01, 0x81 ,0x09 ,0x09 ,0x00 ,0x78 ,0x00 ,0x96 ,0x00 ,0x78 ,0x00 ,0x96 ,0x00 ,0x78 ,0x00,0x96);
		lcm_dcs_write_seq_static(ctx,0x00,0x90);
		lcm_dcs_write_seq_static(ctx,0xCE,0x00 ,0xA5 ,0x16 ,0x8F ,0x00 ,0xA5 ,0x80 ,0x09 ,0x09 ,0x00 ,0x07 ,0xD0 ,0x16 ,0x16 ,0x17);
		lcm_dcs_write_seq_static(ctx,0x00,0xA0);
		lcm_dcs_write_seq_static(ctx,0xCE,0x20 ,0x00 ,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xB0);
		lcm_dcs_write_seq_static(ctx,0xCE,0x87 ,0x00 ,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xD1);
		lcm_dcs_write_seq_static(ctx,0xCE,0x00 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xE1);
		lcm_dcs_write_seq_static(ctx,0xCE,0x08 ,0x03 ,0xC3 ,0x03 ,0xC3 ,0x02 ,0xB0 ,0x00 ,0x00 ,0x00 ,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xF1);
		lcm_dcs_write_seq_static(ctx,0xCE,0x14 ,0x14 ,0x1E ,0x01 ,0x45 ,0x01 ,0x45 ,0x01 ,0x45);
		lcm_dcs_write_seq_static(ctx,0x00,0xB0);
		lcm_dcs_write_seq_static(ctx,0xCF,0x00 ,0x00 ,0x6D ,0x71);
		lcm_dcs_write_seq_static(ctx,0x00,0xB5);
		lcm_dcs_write_seq_static(ctx,0xCF,0x03 ,0x03 ,0x5B ,0x5F);
		lcm_dcs_write_seq_static(ctx,0x00,0xC0);
		lcm_dcs_write_seq_static(ctx,0xCF,0x06 ,0x06 ,0x47 ,0x4B);
		lcm_dcs_write_seq_static(ctx,0x00,0xC5);
		lcm_dcs_write_seq_static(ctx,0xCF,0x06 ,0x06 ,0x4B ,0x4F);
		lcm_dcs_write_seq_static(ctx,0x00,0x60);
		lcm_dcs_write_seq_static(ctx,0xCF,0x00 ,0x00 ,0x6D ,0x71 ,0x03 ,0x03 ,0x5B ,0x5F);
		lcm_dcs_write_seq_static(ctx,0x00,0x70);
		lcm_dcs_write_seq_static(ctx,0xCF,0x00 ,0x00 ,0x65 ,0x69 ,0x03 ,0x03 ,0x53 ,0x57);
		lcm_dcs_write_seq_static(ctx,0x00,0xAA);
		lcm_dcs_write_seq_static(ctx,0xCF,0x80 ,0x80 ,0x1C ,0x18);
		lcm_dcs_write_seq_static(ctx,0x00,0xD1);
		lcm_dcs_write_seq_static(ctx,0xC1,0x03 ,0xAA ,0x05 ,0x22 ,0x09 ,0x59 ,0x05 ,0x87 ,0x08 ,0x23 ,0x0F ,0xAC);
		lcm_dcs_write_seq_static(ctx,0x00,0xE1);
		lcm_dcs_write_seq_static(ctx,0xC1,0x05 ,0x22);
		lcm_dcs_write_seq_static(ctx,0x00,0xE2);
		lcm_dcs_write_seq_static(ctx,0xCF,0x06 ,0xDA ,0x06 ,0xD9 ,0x06 ,0xD9 ,0x06 ,0xD9 ,0x06 ,0xD9 ,0x06 ,0xD9);
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xC1,0x00 ,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0x90);
		lcm_dcs_write_seq_static(ctx,0xC1,0x01);
		lcm_dcs_write_seq_static(ctx,0x00,0xF5);
		lcm_dcs_write_seq_static(ctx,0xCF,0x01);
		lcm_dcs_write_seq_static(ctx,0x00,0xF6);
		lcm_dcs_write_seq_static(ctx,0xCF,0x5A);
		lcm_dcs_write_seq_static(ctx,0x00,0xF1);
		lcm_dcs_write_seq_static(ctx,0xCF,0x5A);
		lcm_dcs_write_seq_static(ctx,0x00,0xF7);
		lcm_dcs_write_seq_static(ctx,0xCF,0x11);
		lcm_dcs_write_seq_static(ctx,0x00,0xD1);
		lcm_dcs_write_seq_static(ctx,0xCE,0x00 ,0x0A ,0x01 ,0x01 ,0x00 ,0xF0 ,0x01);
		lcm_dcs_write_seq_static(ctx,0x00,0xE8);
		lcm_dcs_write_seq_static(ctx,0xCE,0x00 ,0xF0 ,0x00 ,0xF0);
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xCC,0x26,0x26,0x26,0x2D,0x1A,0x2B,0x16,0x18,0x1C,0x1D,0x06,0x08,0x0A,0x0C,0x0E,0x10);
		lcm_dcs_write_seq_static(ctx,0x00,0x90);
		lcm_dcs_write_seq_static(ctx,0xCC,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xCD,0x26,0x26,0x26,0x2E,0x1B,0x2C,0x17,0x19,0x1C,0x1D,0x07,0x09,0x0B,0x0D,0x0F,0x11);
		lcm_dcs_write_seq_static(ctx,0x00,0x90);
		lcm_dcs_write_seq_static(ctx,0xCD,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xCB,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD,0xCD);
		lcm_dcs_write_seq_static(ctx,0x00,0xED);
		lcm_dcs_write_seq_static(ctx,0xCB,0xCD);
		lcm_dcs_write_seq_static(ctx,0x00,0x90);
		lcm_dcs_write_seq_static(ctx,0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xEE);
		lcm_dcs_write_seq_static(ctx,0xCB,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0x90);
		lcm_dcs_write_seq_static(ctx,0xC3,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xA0);
		lcm_dcs_write_seq_static(ctx,0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xB0);
		lcm_dcs_write_seq_static(ctx,0xCB,0x55,0x55,0x55,0x55);
		lcm_dcs_write_seq_static(ctx,0x00,0xC0);
		lcm_dcs_write_seq_static(ctx,0xCB,0x55,0x55,0x55,0x55);
		lcm_dcs_write_seq_static(ctx,0x00,0xD2);
		lcm_dcs_write_seq_static(ctx,0xCB,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xE0);
		lcm_dcs_write_seq_static(ctx,0xCB,0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83);
		lcm_dcs_write_seq_static(ctx,0x00,0xFA);
		lcm_dcs_write_seq_static(ctx,0xCB,0x83,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xEF);
		lcm_dcs_write_seq_static(ctx,0xCB,0x05);
		lcm_dcs_write_seq_static(ctx,0x00,0x68);
		lcm_dcs_write_seq_static(ctx,0xC2,0x8A,0x06,0x7D,0x7D);
		lcm_dcs_write_seq_static(ctx,0x00,0x6C);
		lcm_dcs_write_seq_static(ctx,0xC2,0x89,0x06,0x7D,0x7D);
		lcm_dcs_write_seq_static(ctx,0x00,0x8C);
		lcm_dcs_write_seq_static(ctx,0xC2,0x85,0x00,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0x91);
		lcm_dcs_write_seq_static(ctx,0xC2,0x84,0x00,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0x96);
		lcm_dcs_write_seq_static(ctx,0xC2,0x83,0x00,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0x9B);
		lcm_dcs_write_seq_static(ctx,0xC2,0x82,0x01,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xA0);
		lcm_dcs_write_seq_static(ctx,0xC2,0x81,0x85,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xA5);
		lcm_dcs_write_seq_static(ctx,0xC2,0x80,0x84,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xAA);
		lcm_dcs_write_seq_static(ctx,0xC2,0x01,0x83,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xAF);
		lcm_dcs_write_seq_static(ctx,0xC2,0x02,0x82,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xB4);
		lcm_dcs_write_seq_static(ctx,0xC2,0x03,0x81,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xB9);
		lcm_dcs_write_seq_static(ctx,0xC2,0x04,0x00,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xBE);
		lcm_dcs_write_seq_static(ctx,0xC2,0x05,0x00,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xC3);
		lcm_dcs_write_seq_static(ctx,0xC2,0x06,0x00,0x26,0x7D,0xB0);
		lcm_dcs_write_seq_static(ctx,0x00,0xdc);
		lcm_dcs_write_seq_static(ctx,0xC2,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0x00,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0x78);
		lcm_dcs_write_seq_static(ctx,0xC2,0x85,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0x7C);
		lcm_dcs_write_seq_static(ctx,0xC2,0x84,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xC2,0x83,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0x84);
		lcm_dcs_write_seq_static(ctx,0xC2,0x82,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0x88);
		lcm_dcs_write_seq_static(ctx,0xC2,0x81,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0xE4);
		lcm_dcs_write_seq_static(ctx,0xC2,0x80,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0xA0);
		lcm_dcs_write_seq_static(ctx,0xC3,0x01,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0xA4);
		lcm_dcs_write_seq_static(ctx,0xC3,0x02,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0xA8);
		lcm_dcs_write_seq_static(ctx,0xC3,0x03,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0xAC);
		lcm_dcs_write_seq_static(ctx,0xC3,0x04,0x06,0x7D,0xC4);
		lcm_dcs_write_seq_static(ctx,0x00,0xFC); //GCH/GCL
		lcm_dcs_write_seq_static(ctx,0xCB,0x0A,0x80);
		lcm_dcs_write_seq_static(ctx,0x00,0xFF); //GCH/GCL frame count
		lcm_dcs_write_seq_static(ctx,0xCB,0x3B);
		lcm_dcs_write_seq_static(ctx,0x00,0xFB); //GCH/GCL start&end point
		lcm_dcs_write_seq_static(ctx,0xC3,0x9A,0x16,0x9A,0x16);
		lcm_dcs_write_seq_static(ctx,0x00,0x98);
		lcm_dcs_write_seq_static(ctx,0xC4,0x08);
		lcm_dcs_write_seq_static(ctx,0x00,0x91);
		lcm_dcs_write_seq_static(ctx,0xE9,0xFF,0xFF,0xFF,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0x85);//Posdmy pch
		lcm_dcs_write_seq_static(ctx,0xC4,0x80);
		lcm_dcs_write_seq_static(ctx,0x00,0x86);//4VB pch
		lcm_dcs_write_seq_static(ctx,0xA4,0xB6);
		lcm_dcs_write_seq_static(ctx,0x00,0x95);//VB pch data
		lcm_dcs_write_seq_static(ctx,0xC4,0x80);
		lcm_dcs_write_seq_static(ctx,0x00,0xCA);//Power on 3 
		lcm_dcs_write_seq_static(ctx,0xC0,0x90);
		lcm_dcs_write_seq_static(ctx,0x00,0xB7);//sd_en_sdpl_sel
		lcm_dcs_write_seq_static(ctx,0xF5,0x1D);
		lcm_dcs_write_seq_static(ctx,0x00,0xB1);//VCOM
		lcm_dcs_write_seq_static(ctx,0xF5,0x1B);
		lcm_dcs_write_seq_static(ctx,0x00,0x83);//VGLO1 power on
		lcm_dcs_write_seq_static(ctx,0xF5,0x11);
		lcm_dcs_write_seq_static(ctx,0x00,0x94);//VGHO1 power on
		lcm_dcs_write_seq_static(ctx,0xF5,0x11);
		lcm_dcs_write_seq_static(ctx,0x00,0xB0); 
		lcm_dcs_write_seq_static(ctx,0xC5,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xB3);
		lcm_dcs_write_seq_static(ctx,0xC5,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xB2);
		lcm_dcs_write_seq_static(ctx,0xC5,0x0D);
		lcm_dcs_write_seq_static(ctx,0x00,0xB5);
		lcm_dcs_write_seq_static(ctx,0xC5,0x02);
		lcm_dcs_write_seq_static(ctx,0x00,0xC2);
		lcm_dcs_write_seq_static(ctx,0xF5,0x42);
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xCE,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xD0);
		lcm_dcs_write_seq_static(ctx,0xCE,0x01);
		lcm_dcs_write_seq_static(ctx,0x00,0xE0);
		lcm_dcs_write_seq_static(ctx,0xCE,0x00);
		lcm_dcs_write_seq_static(ctx,0x00,0xA1);
		lcm_dcs_write_seq_static(ctx,0xC1,0xCC);
		lcm_dcs_write_seq_static(ctx,0x00,0xA6);
		lcm_dcs_write_seq_static(ctx,0xC1,0x10);
		lcm_dcs_write_seq_static(ctx,0x00,0x71);//VFP=255-1
		lcm_dcs_write_seq_static(ctx,0xC0,0xA0,0x00,0xFE,0x00,0x1E);
		lcm_dcs_write_seq_static(ctx,0x00,0x86);//I2C EN
		lcm_dcs_write_seq_static(ctx,0xB7,0x80);
		lcm_dcs_write_seq_static(ctx,0x00,0xA5);  
		lcm_dcs_write_seq_static(ctx,0xB0,0x1D); //RC delay 95ns
		lcm_dcs_write_seq_static(ctx,0x00,0x00);
		lcm_dcs_write_seq_static(ctx,0xFF,0x00,0x00,0x00); 
		lcm_dcs_write_seq_static(ctx,0x00,0x80);
		lcm_dcs_write_seq_static(ctx,0xFF,0x00,0x00);
	//----------------------LCD initial code End----------------------//
	//SLPOUT and DISPON
		lcm_dcs_write_seq_static(ctx, 0x11);
		mdelay(120);
		lcm_dcs_write_seq_static(ctx, 0x29);
		mdelay(50);

		LCM_LOGI("pige %s()- \n", __func__);
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
    lcm_dcs_write_seq_static(ctx, 0x26,0x08);
	
	lcm_dcs_write_seq_static(ctx, 0x28);
	mdelay(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	mdelay(120);
	
	ctx->error = 0;
	ctx->prepared = false;




	udelay(1000);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	gpiod_set_value(ctx->reset_gpio, 1);
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
	LCM_LOGI("pige %s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);
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




#define HFP (170)
#define HSA (16)
#define HBP (18)
#define VFP_90HZ (255)
#define VFP_60HZ (1205)
#define VSA (6)
#define VBP (28)

#define VAC (1612)          //1918
#define HAC (720)          //838
static u32 fake_heigh = 0;
static u32 fake_width = 0;

static bool need_fake_resolution = false;
static int current_fps = 90; //default 60hz




static struct drm_display_mode lowFPS_mode = {
	.clock = 517000,
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
	.clock = 517000,//on used
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
	sprintf(LCM_module_name, "FT8057:TDT:720*1612:AUC0660119C1");
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
	{ .compatible = "tdt,ft8057", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-encore-ft8057-inx-tdt-hdplus1612-dsi-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Tai-Hua Tseng <tai-hua.tseng@mediatek.com>");
MODULE_DESCRIPTION("ft8057 KD AUO FHD VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");

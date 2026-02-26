
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
//#include "include/panel-buffalo-gc7202-dsi-vdo-tft.h"
#include "include/panel-lcd-power.h"

//begin add by jiaxin.pi for 11474010 on 20210903
#define PHYSICAL_WIDTH              68000
#define PHYSICAL_HEIGHT             151000
//end add by jiaxin.pi for 11474010 on 20210903

#define LOG_TAG "LCM"
#ifdef BUILD_LK
	#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
	#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
	#define LCM_LOGI(fmt, args...)  pr_info("[KERNEL/"LOG_TAG"]"fmt, ##args)
	#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

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
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(35);


	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);

	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);

	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(30);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	//udelay(10 * 1000);
	//LCM_LOGD("%s() reset_gpio=%d \n", __func__, gpiod_get_value(ctx->reset_gpio));

	//----------------------LCD initial code start----------------------//
	//CMD2 ENABLE
	lcm_dcs_write_seq_static(ctx, 0xFF,0x55,0xAA,0x66);

	lcm_dcs_write_seq_static(ctx, 0xff,0xc3);// read 0x0A need add
    lcm_dcs_write_seq_static(ctx, 0x0d,0x00);
    lcm_dcs_write_seq_static(ctx, 0x0e,0x8d);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x20);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x21);    
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x22);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x23);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x24);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x26);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x27);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0xA3);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0xB3);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0xC3);
    lcm_dcs_write_seq_static(ctx, 0xFB,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x22);
    lcm_dcs_write_seq_static(ctx, 0xE4,0x01);
    lcm_dcs_write_seq_static(ctx, 0x01,0x06);
    lcm_dcs_write_seq_static(ctx, 0x02,0x40);
    lcm_dcs_write_seq_static(ctx, 0x25,0x08);
    lcm_dcs_write_seq_static(ctx, 0x26,0x00);
    lcm_dcs_write_seq_static(ctx, 0x2E,0xAA);
    lcm_dcs_write_seq_static(ctx, 0x2F,0x00);
    lcm_dcs_write_seq_static(ctx, 0x36,0x09);
    lcm_dcs_write_seq_static(ctx, 0x37,0x00);
    lcm_dcs_write_seq_static(ctx, 0x3F,0xAA);
    lcm_dcs_write_seq_static(ctx, 0x40,0x00);
    lcm_dcs_write_seq_static(ctx, 0x0E,0x33);
    lcm_dcs_write_seq_static(ctx, 0x08,0x40);
    lcm_dcs_write_seq_static(ctx, 0x1f,0x06);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x21);
    lcm_dcs_write_seq_static(ctx, 0x01,0x04);
    lcm_dcs_write_seq_static(ctx, 0x02,0x25);
    lcm_dcs_write_seq_static(ctx, 0x03,0x18);
    lcm_dcs_write_seq_static(ctx, 0x04,0x19);
    lcm_dcs_write_seq_static(ctx, 0x05,0x19);
    lcm_dcs_write_seq_static(ctx, 0x06,0x25);
    lcm_dcs_write_seq_static(ctx, 0x07,0x25);
    lcm_dcs_write_seq_static(ctx, 0x08,0x25);
    lcm_dcs_write_seq_static(ctx, 0x09,0x25);
    lcm_dcs_write_seq_static(ctx, 0x0A,0x0E);
    lcm_dcs_write_seq_static(ctx, 0x0B,0x0C);
    lcm_dcs_write_seq_static(ctx, 0x0C,0x25);
    lcm_dcs_write_seq_static(ctx, 0x0D,0x25);
    lcm_dcs_write_seq_static(ctx, 0x0E,0x0A);
    lcm_dcs_write_seq_static(ctx, 0x0F,0x08);
    lcm_dcs_write_seq_static(ctx, 0x10,0x00);
    lcm_dcs_write_seq_static(ctx, 0x11,0x25);
    lcm_dcs_write_seq_static(ctx, 0x12,0x25);
    lcm_dcs_write_seq_static(ctx, 0x13,0x25);
    lcm_dcs_write_seq_static(ctx, 0x14,0x25);
    lcm_dcs_write_seq_static(ctx, 0x15,0x25);
    lcm_dcs_write_seq_static(ctx, 0x16,0x25);
    lcm_dcs_write_seq_static(ctx, 0x17,0x05);
    lcm_dcs_write_seq_static(ctx, 0x18,0x25);
    lcm_dcs_write_seq_static(ctx, 0x19,0x18);
    lcm_dcs_write_seq_static(ctx, 0x1A,0x19);
    lcm_dcs_write_seq_static(ctx, 0x1B,0x19);
    lcm_dcs_write_seq_static(ctx, 0x1C,0x25);
    lcm_dcs_write_seq_static(ctx, 0x1D,0x25);
    lcm_dcs_write_seq_static(ctx, 0x1E,0x25);
    lcm_dcs_write_seq_static(ctx, 0x1F,0x25);
    lcm_dcs_write_seq_static(ctx, 0x20,0x0F);
    lcm_dcs_write_seq_static(ctx, 0x21,0x0D);
    lcm_dcs_write_seq_static(ctx, 0x22,0x25);
    lcm_dcs_write_seq_static(ctx, 0x23,0x25);
    lcm_dcs_write_seq_static(ctx, 0x24,0x0B);
    lcm_dcs_write_seq_static(ctx, 0x25,0x09);
    lcm_dcs_write_seq_static(ctx, 0x26,0x01);
    lcm_dcs_write_seq_static(ctx, 0x27,0x25);
    lcm_dcs_write_seq_static(ctx, 0x28,0x25);
    lcm_dcs_write_seq_static(ctx, 0x29,0x25);
    lcm_dcs_write_seq_static(ctx, 0x2A,0x25);
    lcm_dcs_write_seq_static(ctx, 0x2B,0x25);
    lcm_dcs_write_seq_static(ctx, 0x2D,0x25);
    lcm_dcs_write_seq_static(ctx, 0x45,0x33);
    lcm_dcs_write_seq_static(ctx, 0x46,0xA6);
    lcm_dcs_write_seq_static(ctx, 0x47,0x06);
    lcm_dcs_write_seq_static(ctx, 0x48,0x45);
    lcm_dcs_write_seq_static(ctx, 0x4C,0xA6);
    lcm_dcs_write_seq_static(ctx, 0x4D,0x05);
    lcm_dcs_write_seq_static(ctx, 0x4E,0x46);
    lcm_dcs_write_seq_static(ctx, 0x5E,0xA3);
    lcm_dcs_write_seq_static(ctx, 0x5F,0x20);
    lcm_dcs_write_seq_static(ctx, 0x60,0x63);
    lcm_dcs_write_seq_static(ctx, 0x64,0xA3);
    lcm_dcs_write_seq_static(ctx, 0x65,0x21);
    lcm_dcs_write_seq_static(ctx, 0x66,0x62);
    lcm_dcs_write_seq_static(ctx, 0xBE,0x03);
    lcm_dcs_write_seq_static(ctx, 0xC0,0x04);
    lcm_dcs_write_seq_static(ctx, 0xC1,0x00);
    lcm_dcs_write_seq_static(ctx, 0x7E,0x0F);
    lcm_dcs_write_seq_static(ctx, 0x7F,0x23);
    lcm_dcs_write_seq_static(ctx, 0x80,0x03);
    lcm_dcs_write_seq_static(ctx, 0x81,0x43);
    lcm_dcs_write_seq_static(ctx, 0x82,0xA0);
    lcm_dcs_write_seq_static(ctx, 0x83,0x02);
    lcm_dcs_write_seq_static(ctx, 0x87,0x18);
    lcm_dcs_write_seq_static(ctx, 0x8B,0x23);
    lcm_dcs_write_seq_static(ctx, 0x8C,0x07);
    lcm_dcs_write_seq_static(ctx, 0x8D,0x43);
    lcm_dcs_write_seq_static(ctx, 0x8E,0xA0);
    lcm_dcs_write_seq_static(ctx, 0x8F,0x02);
    lcm_dcs_write_seq_static(ctx, 0x93,0x19);
    lcm_dcs_write_seq_static(ctx, 0x49,0x02);
    lcm_dcs_write_seq_static(ctx, 0x4A,0x60);
    lcm_dcs_write_seq_static(ctx, 0x4F,0x02);
    lcm_dcs_write_seq_static(ctx, 0x50,0x60);
    lcm_dcs_write_seq_static(ctx, 0x61,0x02);
    lcm_dcs_write_seq_static(ctx, 0x62,0x60);
    lcm_dcs_write_seq_static(ctx, 0x67,0x02);
    lcm_dcs_write_seq_static(ctx, 0x68,0x60);
    lcm_dcs_write_seq_static(ctx, 0x84,0x02);
    lcm_dcs_write_seq_static(ctx, 0x85,0x60);
    lcm_dcs_write_seq_static(ctx, 0x90,0x02);
    lcm_dcs_write_seq_static(ctx, 0x91,0x60);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x20);
    lcm_dcs_write_seq_static(ctx, 0x2D,0x1F);
    lcm_dcs_write_seq_static(ctx, 0x02,0x5F);
    lcm_dcs_write_seq_static(ctx, 0x03,0x5F);
    lcm_dcs_write_seq_static(ctx, 0x04,0x4A);
    lcm_dcs_write_seq_static(ctx, 0x05,0x4A);
    lcm_dcs_write_seq_static(ctx, 0x0A,0x40);
    lcm_dcs_write_seq_static(ctx, 0x0B,0x40);
    lcm_dcs_write_seq_static(ctx, 0x0C,0x4C);
    lcm_dcs_write_seq_static(ctx, 0x0D,0x4C);
    lcm_dcs_write_seq_static(ctx, 0x77,0x74);
    lcm_dcs_write_seq_static(ctx, 0x78,0x75);
   // lcm_dcs_write_seq_static(ctx, 0x1C,0x95);
   // lcm_dcs_write_seq_static(ctx, 0x1D,0x95);
    lcm_dcs_write_seq_static(ctx, 0xA5,0x00);
    lcm_dcs_write_seq_static(ctx, 0xA6,0xFF);
    lcm_dcs_write_seq_static(ctx, 0xA9,0x00);
    lcm_dcs_write_seq_static(ctx, 0xAA,0xFF);
    lcm_dcs_write_seq_static(ctx, 0xAD,0x00);
    lcm_dcs_write_seq_static(ctx, 0xAE,0xFF);
    lcm_dcs_write_seq_static(ctx, 0xB1,0x00);
    lcm_dcs_write_seq_static(ctx, 0xB2,0xFF);
    lcm_dcs_write_seq_static(ctx, 0xC3,0x00);
    lcm_dcs_write_seq_static(ctx, 0xC4,0x65);
    lcm_dcs_write_seq_static(ctx, 0xC5,0x00);
    lcm_dcs_write_seq_static(ctx, 0xC6,0x65);
    lcm_dcs_write_seq_static(ctx, 0xB3,0x00);
    lcm_dcs_write_seq_static(ctx, 0xB4,0x28);
    lcm_dcs_write_seq_static(ctx, 0xB5,0x01);
    lcm_dcs_write_seq_static(ctx, 0xB6,0x04);
    lcm_dcs_write_seq_static(ctx, 0xD3,0x1e);
    lcm_dcs_write_seq_static(ctx, 0x7E,0x01);
    lcm_dcs_write_seq_static(ctx, 0x7F,0x00);
    lcm_dcs_write_seq_static(ctx, 0x80,0x01);
    lcm_dcs_write_seq_static(ctx, 0x81,0x00);
    lcm_dcs_write_seq_static(ctx, 0x82,0x00);
    lcm_dcs_write_seq_static(ctx, 0x83,0x04);
    lcm_dcs_write_seq_static(ctx, 0x84,0x04);
    lcm_dcs_write_seq_static(ctx, 0x85,0x01);
    lcm_dcs_write_seq_static(ctx, 0x86,0xA4);
    lcm_dcs_write_seq_static(ctx, 0x87,0x01);
    lcm_dcs_write_seq_static(ctx, 0x88,0x8B);
    lcm_dcs_write_seq_static(ctx, 0x8A,0x03);
    lcm_dcs_write_seq_static(ctx, 0x8B,0x03);
    lcm_dcs_write_seq_static(ctx, 0x18,0x44);
    lcm_dcs_write_seq_static(ctx, 0x17,0x53);
    lcm_dcs_write_seq_static(ctx, 0x12,0xA2);
    lcm_dcs_write_seq_static(ctx, 0x15,0x4C);
    lcm_dcs_write_seq_static(ctx, 0xA3,0x44);
    lcm_dcs_write_seq_static(ctx, 0xA7,0x44);
    lcm_dcs_write_seq_static(ctx, 0xD3,0x03);
    lcm_dcs_write_seq_static(ctx, 0x24,0x99);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x23);
    lcm_dcs_write_seq_static(ctx, 0x01,0x00,0x00,0x00,0x2E,0x00,0x5D,0x00,0x7D,0x00,0x95,0x00,0xAA,0x00,0xC0,0x00,0xCF);
    lcm_dcs_write_seq_static(ctx, 0x02,0x00,0xDB,0x01,0x09,0x01,0x2C,0x01,0x61,0x01,0x86,0x01,0xCA,0x01,0xFE,0x01,0xFF);
    lcm_dcs_write_seq_static(ctx, 0x03,0x02,0x35,0x02,0x77,0x02,0xA3,0x02,0xDC,0x03,0x00,0x03,0x37,0x03,0x47,0x03,0x57);
    lcm_dcs_write_seq_static(ctx, 0x04,0x03,0x6C,0x03,0x74,0x03,0x89,0x03,0x9F,0x03,0xCD,0x03,0xFF);

    lcm_dcs_write_seq_static(ctx, 0x05,0x00,0x00,0x00,0x2E,0x00,0x5D,0x00,0x7D,0x00,0x95,0x00,0xAA,0x00,0xC0,0x00,0xCF);
    lcm_dcs_write_seq_static(ctx, 0x06,0x00,0xDB,0x01,0x09,0x01,0x2C,0x01,0x61,0x01,0x86,0x01,0xCA,0x01,0xFE,0x01,0xFF);
    lcm_dcs_write_seq_static(ctx, 0x07,0x02,0x35,0x02,0x77,0x02,0xA3,0x02,0xDC,0x03,0x00,0x03,0x37,0x03,0x47,0x03,0x57);
    lcm_dcs_write_seq_static(ctx, 0x08,0x03,0x6C,0x03,0x74,0x03,0x89,0x03,0x9F,0x03,0xCD,0x03,0xFF);

    lcm_dcs_write_seq_static(ctx, 0x09,0x00,0x00,0x00,0x2E,0x00,0x5D,0x00,0x7D,0x00,0x95,0x00,0xAA,0x00,0xC0,0x00,0xCF);
    lcm_dcs_write_seq_static(ctx, 0x0a,0x00,0xDB,0x01,0x09,0x01,0x2C,0x01,0x61,0x01,0x86,0x01,0xCA,0x01,0xFE,0x01,0xFF);
    lcm_dcs_write_seq_static(ctx, 0x0b,0x02,0x35,0x02,0x77,0x02,0xA3,0x02,0xDC,0x03,0x00,0x03,0x37,0x03,0x47,0x03,0x57);
    lcm_dcs_write_seq_static(ctx, 0x0c,0x03,0x6C,0x03,0x74,0x03,0x89,0x03,0x9F,0x03,0xCD,0x03,0xFF);

    lcm_dcs_write_seq_static(ctx, 0x0d,0x00,0x00,0x00,0x2E,0x00,0x5D,0x00,0x7D,0x00,0x95,0x00,0xAA,0x00,0xC0,0x00,0xCF);
    lcm_dcs_write_seq_static(ctx, 0x0e,0x00,0xDB,0x01,0x09,0x01,0x2C,0x01,0x61,0x01,0x86,0x01,0xCA,0x01,0xFE,0x01,0xFF);
    lcm_dcs_write_seq_static(ctx, 0x0f,0x02,0x35,0x02,0x77,0x02,0xA3,0x02,0xDC,0x03,0x00,0x03,0x37,0x03,0x47,0x03,0x57);
    lcm_dcs_write_seq_static(ctx, 0x10,0x03,0x6C,0x03,0x74,0x03,0x89,0x03,0x9F,0x03,0xCD,0x03,0xFF);

    lcm_dcs_write_seq_static(ctx, 0x11,0x00,0x00,0x00,0x2E,0x00,0x5D,0x00,0x7D,0x00,0x95,0x00,0xAA,0x00,0xC0,0x00,0xCF);
    lcm_dcs_write_seq_static(ctx, 0x12,0x00,0xDB,0x01,0x09,0x01,0x2C,0x01,0x61,0x01,0x86,0x01,0xCA,0x01,0xFE,0x01,0xFF);
    lcm_dcs_write_seq_static(ctx, 0x13,0x02,0x35,0x02,0x77,0x02,0xA3,0x02,0xDC,0x03,0x00,0x03,0x37,0x03,0x47,0x03,0x57);
    lcm_dcs_write_seq_static(ctx, 0x14,0x03,0x6C,0x03,0x74,0x03,0x89,0x03,0x9F,0x03,0xCD,0x03,0xFF);

    lcm_dcs_write_seq_static(ctx, 0x15,0x00,0x00,0x00,0x2E,0x00,0x5D,0x00,0x7D,0x00,0x95,0x00,0xAA,0x00,0xC0,0x00,0xCF);
    lcm_dcs_write_seq_static(ctx, 0x16,0x00,0xDB,0x01,0x09,0x01,0x2C,0x01,0x61,0x01,0x86,0x01,0xCA,0x01,0xFE,0x01,0xFF);
    lcm_dcs_write_seq_static(ctx, 0x17,0x02,0x35,0x02,0x77,0x02,0xA3,0x02,0xDC,0x03,0x00,0x03,0x37,0x03,0x47,0x03,0x57);
    lcm_dcs_write_seq_static(ctx, 0x18,0x03,0x6C,0x03,0x74,0x03,0x89,0x03,0x9F,0x03,0xCD,0x03,0xFF);

    lcm_dcs_write_seq_static(ctx, 0xFF,0xA3);
    lcm_dcs_write_seq_static(ctx, 0xFA,0x01);
    lcm_dcs_write_seq_static(ctx, 0x35,0x01);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x26);
    lcm_dcs_write_seq_static(ctx, 0x1D,0x0C,0x00,0x00,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x24);
    lcm_dcs_write_seq_static(ctx, 0x03,0x01,0xdd);
    lcm_dcs_write_seq_static(ctx, 0x50,0xaa);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x20);
    lcm_dcs_write_seq_static(ctx, 0x4A,0x30);
    lcm_dcs_write_seq_static(ctx, 0x48,0x10);
    lcm_dcs_write_seq_static(ctx, 0x49,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x10);
    lcm_dcs_write_seq_static(ctx, 0x51,0x07,0xFF);
    lcm_dcs_write_seq_static(ctx, 0x53,0x2C);
    lcm_dcs_write_seq_static(ctx, 0x36,0x08);
    lcm_dcs_write_seq_static(ctx, 0x69,0x00);
    lcm_dcs_write_seq_static(ctx, 0x35,0x00);
   lcm_dcs_write_seq_static(ctx, 0xFF,0x24);
    lcm_dcs_write_seq_static(ctx, 0x7D,0x00);
    lcm_dcs_write_seq_static(ctx, 0xFF,0x66,0x99,0x55);

	//----------------------LCD initial code End----------------------//
	//SLPOUT and DISPON
	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(120);
	lcm_dcs_write_seq_static(ctx, 0x29);
	//lcm_dcs_write_seq_static(ctx, 0x35,0x00);
	msleep(20);
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
	LCM_LOGD("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);

	if (!ctx->prepared)
		return 0;
	lcm_dcs_write_seq_static(ctx, 0xFF,0x55,0xaa,0x66);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x20);
    lcm_dcs_write_seq_static(ctx, 0x4A,0x01);
	lcm_dcs_write_seq_static(ctx, 0x48,0x10);
	lcm_dcs_write_seq_static(ctx, 0x49,0x00);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x10);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);
	
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
	
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(50);
	
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	udelay(1000);
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

	udelay(2000);
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

	udelay(5* 1000);
	//LCM_LOGD("%s() bias_neg=%d \n", __func__, gpiod_get_value(ctx->bias_neg));

#if defined(CONFIG_BIAS_TPS65132_I2C)
	_lcm_i2c_write_bytes(0x0, 0x14); //+6.0V
	udelay(1000);
	_lcm_i2c_write_bytes(0x1, 0x14); //-6.0V
	msleep(20);
#endif
#endif
#endif
	
    
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


#define HFP_90HZ (32)
#define HSA_90HZ (4)
#define HBP_90HZ (32)

#define VFP_90HZ (150)
#define VSA_90HZ (4)
#define VBP_90HZ (38)

#define HFP_60HZ (32)
#define HSA_60HZ (4)
#define HBP_60HZ (32)

#define VFP_60HZ (990)
#define VSA_60HZ (4)
#define VBP_60HZ (38)

#define VAC (1600)
#define HAC (720)
static u32 fake_heigh = 0;
static u32 fake_width = 0;

static bool need_fake_resolution = false;
static int current_fps = 60; //default 60hz

static struct drm_display_mode performance_mode = {
	.clock = 400000,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP_90HZ,
	.hsync_end = HAC + HFP_90HZ + HSA_90HZ,
	.htotal = HAC + HFP_90HZ + HSA_90HZ + HBP_90HZ,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_90HZ,
	.vsync_end = VAC + VFP_90HZ + VSA_90HZ,
	.vtotal = VAC + VFP_90HZ + VSA_90HZ + VBP_90HZ,
	.vrefresh = 90,
};


static struct drm_display_mode lowFPS_mode = {
	.clock = 400000,
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
static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 400,
	.data_rate = 800,
	//.vfp_low_power = VFP_60HZ,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
//begin add by jiaxin.pi for 11474010 on 20210903
	.physical_width_um = PHYSICAL_WIDTH,
  	.physical_height_um = PHYSICAL_HEIGHT,
//end add by jiaxin.pi for 11474010 on 20210903
	.dyn_fps = {
		.switch_en = 1,
		/* switch cmd send in mode_switch_to_60()
		//90->60 LPF
		//CMD2 ENABLE
		.dfps_cmd_table[0] = {0, 2, {0x00,0x00} },
		.dfps_cmd_table[1] = {0, 4, {0xFF,0x87,0x56,0x01} },
		.dfps_cmd_table[2] = {0, 2, {0x00, 0x80} },
		.dfps_cmd_table[3] = {0, 3, {0xFF,0x87,0x56} },
		.dfps_cmd_table[4] = {0, 2, {0x00,0xd1} },
		.dfps_cmd_table[5] = {0, 2, {0xc0,0x83} },
		.dfps_cmd_table[6] = {0, 2, {0x00,0xd2} },
		.dfps_cmd_table[7] = {0, 2, {0xc0,0x01} },
		//CMD2 DISENABLE
		.dfps_cmd_table[8] = {0, 2, {0x00,0x00} },
		.dfps_cmd_table[9] = {0, 4, {0xFF,0xFF,0xFF,0xFF} },*/
		.vact_timing_fps = 90,
	},
};


static struct mtk_panel_params ext_params_60hz = {
	.pll_clk = 400,
	.data_rate = 800,
	//.vfp_low_power = VFP_90HZ,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
//begin add by jiaxin.pi for 11474010 on 20210903	
	.physical_width_um = PHYSICAL_WIDTH,
  	.physical_height_um = PHYSICAL_HEIGHT,
//end add by jiaxin.pi for 11474010 on 20210903
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

static int lcm_panel_ext_param_set(struct drm_panel *panel, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = lcm_get_mode_by_id(panel, mode);

	LCM_LOGD("%s()+ mode=%d\n", __func__, mode);

	if (m->vrefresh == 60) {
		ext->params = &ext_params_60hz;
		current_fps = 60;
	} else if (m->vrefresh == 90) {
		ext->params = &ext_params_90hz;
		current_fps = 90;
	} else {
		ret = 1;
	}

	LCM_LOGI("%s()- current_fps=%d\n", __func__, current_fps);

	return ret;
}



//extern int gcore_fw_event_notify(enum fw_event_type event);

static void lcm_mode_switch_to_90(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	
	LCM_LOGD("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);

	//60->90 normal
	//CMD2 ENABLE

   // gcore_fw_event_notify(FW_REPORT_RATE_180);
}

static void lcm_mode_switch_to_60(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	LCM_LOGD("%s()+ %d,%d,%d \n", __func__, ctx->prepared, ctx->enabled, ctx->error);

	//90->60 LPF
	//CMD2 ENABLE

  //  gcore_fw_event_notify(FW_REPORT_RATE_120);
}

static int lcm_panel_mode_switch(struct drm_panel *panel, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = lcm_get_mode_by_id(panel, dst_mode);

	LCM_LOGI("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);



	if (m->vrefresh == 60) { /* 90 switch to 60 */
		lcm_mode_switch_to_60(panel);
	} else if (m->vrefresh == 90) { /* 60 switch to 90*/
		lcm_mode_switch_to_90(panel);
	} else
		ret = 1;

	return ret;
}


static struct mtk_panel_funcs ext_funcs = {
	.reset = lcm_panel_ext_reset,
	.ext_param_set = lcm_panel_ext_param_set,
	.mode_switch = lcm_panel_mode_switch,
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
	struct drm_display_mode *mode2;

	LCM_LOGD("%s()+ \n", __func__);

	if (need_fake_resolution){
		change_drm_disp_mode_params(&lowFPS_mode);
		change_drm_disp_mode_params(&performance_mode);
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
	mode2 = drm_mode_duplicate(panel->drm, &performance_mode);
	if (!mode2) {
		dev_info(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode.hdisplay, performance_mode.vdisplay,
			 performance_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(panel->connector, mode2);

	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 129;
//begin add by jiaxin.pi for task 11447839 on 20210819
#ifdef CONFIG_TCT_DEVICEINFO
#ifdef CONFIG_TCT_DEVICEINFO_TMO
	sprintf(LCM_module_name, "GC7202:TDT:720*1600:AUC0650139C1");
#else
        sprintf(LCM_module_name, "FT8756:TDT:720*1600:AUC0650123C1");
#endif
#endif
//end add by jiaxin.pi for task 11447839 on 20210819
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
	ret = mtk_panel_ext_create(dev, &ext_params_60hz, &ext_funcs, &ctx->panel);
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
	{ .compatible = "buffalo,gc7202,tft", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-buffalo-gc7202-tft",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Tai-Hua Tseng <tai-hua.tseng@mediatek.com>");
MODULE_DESCRIPTION("GC7202 TFT VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");

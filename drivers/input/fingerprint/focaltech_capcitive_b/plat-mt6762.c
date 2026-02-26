/**
 * plat-mt6833.c
 *
**/

#include <linux/stddef.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#if !defined(CONFIG_MTK_CLKMGR)
# include <linux/clk.h>
#else
# include <mach/mt_clkmgr.h>
#endif

#include "ff_log.h"

/* TODO: */
/*{{{*/// +++++ +++++ +++++ +++++ +++++ 1. config dts node property +++++ +++++ +++++ +++++ +++++
#define FF_COMPATIBLE_NODE_PIN_CTRL     "mediatek,fingerprint-pinctrl"
#define FF_COMPATIBLE_NODE_INT          "mediatek,fpsensor-eint"
#define FF_COMPATIBLE_NODE_INT_V1          "mediatek,fpsensor-eint-v1"
#define FF_COMPATIBLE_NODE_SPI          "mediatek,mt6765-spi"
#define FF_COMPATIBLE_NODE_POWER        "mediatek,fingerprint-power"
#define FF_SPI_SUB_NODE					"mediatek,fingerprint-fpsensor"

// VDD config
#define FF_DTS_CONFIG_VDD_ALWAYS_ON		0// test only for vdd-always-on
#define SUPPLY_3V3              3300000
#define SUPPLY_2V8              2800000
#define SUPPLY_2V9              2900000
#define SUPPLY_0V0              0		// not accessible, limited by the regulator pmic defined in mt6350.dtsi
#define FF_SUPPLY_VDD_MIN       SUPPLY_2V9
#define FF_SUPPLY_VDD_MAX       SUPPLY_2V9
#define FF_SUPPLY_VDD_DOWN		SUPPLY_2V8

// VDD dts node
#define FF_DTS_NODE_VDD         "finger_vio28"


// pinctrl config
typedef enum {
//    FF_PINCTRL_DEFAULT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_RST_ACT_V1,
    FF_PINCTRL_STATE_RST_CLR_V1,
    FF_PINCTRL_STATE_INT_ACT_V1,
//    FF_PINCTRL_STATE_PINS_SPI,  
    FF_PINCTRL_STATE_SPI_CS,
    FF_PINCTRL_STATE_SPI_MO,
    FF_PINCTRL_STATE_SPI_MI,
    FF_PINCTRL_STATE_SPI_CLK,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} FF_PINCTRL_STATE_E;

typedef enum {
    FF_IRQ_STATE_DISABLE  = 0,
    FF_IRQ_STATE_ENABLE   = 1,
    FF_IRQ_STATE_UNKNOW   = 2
} FF_IRQ_STATE_E;

// pinctrl dts node
static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
	//"default", "fpsensor_rst_low", "fpsensor_rst_high", "fpsensor_eint_low", "spi-default",
	"fpsensor_power_low", "fpsensor_power_high",
	"fpsensor_rst_low", "fpsensor_rst_high", "fpsensor_eint_low",
	"fpsensor_rst_low_v1", "fpsensor_rst_high_v1", "fpsensor_eint_low_v1",
	"fpsensor_spi_cs_high", "fpsensor_spi_mo_low", "fpsensor_spi_mi_low", "fpsensor_spi_clk_low"
};
/*}}}*/// +++++ +++++ +++++ +++++ +++++ 1. config dts node property +++++ +++++ +++++ +++++ +++++






/*{{{*/// +++++ +++++ +++++ +++++ +++++ 2. dts node storage +++++ +++++ +++++ +++++ +++++

/* Native context and its singleton instance. */
typedef struct {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_states[FF_PINCTRL_STATE_MAXIMUM];
#if !defined(CONFIG_MTK_CLKMGR)
	struct clk *spiclk;
#endif
	struct regulator *ff_vdd;
	struct platform_device *p_spi_dev;
	bool b_spiclk_enabled;
} FF_DTS_NODE_CONTEXT_T;
static FF_DTS_NODE_CONTEXT_T ff_mt6833_context = {
	.pinctrl		= NULL,
	.spiclk			= NULL,
	.ff_vdd			= NULL,
	.p_spi_dev		= NULL,
	.b_spiclk_enabled = false
}, *g_context_dts = &ff_mt6833_context;

/*}}}*/// +++++ +++++ +++++ +++++ +++++ 2. dts node storage +++++ +++++ +++++ +++++ +++++
//Begin add by qiaozhen.li for 11672345 on 2021/11/17
#if defined(CONFIG_TCT_BOARD_ID_COMPATIBLE)
extern unsigned int androidboot_dtbo_idx;
#else
unsigned int androidboot_dtbo_idx = 0;
#endif
//End add by qiaozhen.li for 11672345 on 2021/11/17

int ff_ctl_enable_power(bool on);

//Begin add by qiaozhen.li for 11672345 on 2021/11/17
static int fp_gpio_reset_high(void)
{
	int err = 0;
#if defined(CONFIG_TCT_BOARD_ID_COMPATIBLE)
	if (2 == androidboot_dtbo_idx) {
		err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_ACT_V1]);
        }
          else {
		err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_ACT]);
	}
#endif
	return err;
}
static int fp_gpio_reset_low(void)
{
	int err = 0;
#if defined(CONFIG_TCT_BOARD_ID_COMPATIBLE)
	if (2 == androidboot_dtbo_idx) {
		err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_CLR_V1]);
        }
          else {
		err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_CLR]);
	}
#endif
	return err;
}
//End add by qiaozhen.li for 11672345 on 2021/11/17

int ff_ctl_init_pins(int *irq_num)
{
	int err = 0, i, j = 0;
	struct device_node *dev_node = NULL, *spi_node = NULL, *irq_node = NULL;
	struct platform_device *pdev = NULL, *p_spi_dev= NULL, *p_irq_dev  = NULL;
	FF_LOGV("'%s' enter.", __func__);

	/* Find device tree node. */
	dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_PIN_CTRL);
	if (!dev_node) {
		FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_PIN_CTRL);
		return (-ENODEV);
	}

	/* Convert to platform device */
	pdev = of_find_device_by_node(dev_node);
	if (!pdev) {
		FF_LOGE("of_find_device_by_node(..) failed.");
		return (-ENODEV);
	}

	/* Retrieve the pinctrl handler. */
	g_context_dts->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (!g_context_dts->pinctrl) {
		FF_LOGE("devm_pinctrl_get(..pinctrl) failed.");
		return (-ENODEV);
	}

	/* Register all pins. */
	for (i = 0; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
		g_context_dts->pin_states[i] = pinctrl_lookup_state(g_context_dts->pinctrl, g_pinctrl_state_names[i]);
		if (!g_context_dts->pin_states[i]) {
			FF_LOGE("can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
			err = (-ENODEV);
			break;
		}
	}

	if (i < FF_PINCTRL_STATE_MAXIMUM) {
		return (-ENODEV);
	}


	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_SPI_CS]);
	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_SPI_MO]);
	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_SPI_MI]);
	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_SPI_CLK]);

	/* Retrieve the irq number. */
	/* Find device tree node. */
//Begin add by qiaozhen.li for 11672345 on 2021/11/17
#if defined(CONFIG_TCT_BOARD_ID_COMPATIBLE)
	if (2 == androidboot_dtbo_idx) {
		/* Initialize the INT pin. */
		err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_INT_ACT_V1]);
		irq_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_INT_V1);
		if (!irq_node) {
			FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_INT_V1);
			return (-ENODEV);
		}
	}else {
		/* Initialize the INT pin. */
		err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_INT_ACT]);
		irq_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_INT);
		if (!irq_node) {
			FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_INT);
			return (-ENODEV);
		}
	}
#endif
//End add by qiaozhen.li for 11672345 on 2021/11/17

	do {
		u32 irq_gpio = 0;
        int err = of_property_read_u32(irq_node, "interrupts", &irq_gpio);
		FF_LOGD("'%s' irq_gpio = %d, err = %d", __func__, irq_gpio, err);
	} while(0);
	/* Convert to platform device */
	p_irq_dev = of_find_device_by_node(irq_node);
	if (!p_irq_dev) {
		FF_LOGE("of_find_device_by_node(..irq) failed.");
		return (-ENODEV);
	}
	*irq_num = irq_of_parse_and_map(irq_node, 0);
	FF_LOGD("'%s' irq number is %d.", __func__, *irq_num);


	/* Find device tree node. */
	/* get spi0 */
	spi_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_SPI);
	if (!spi_node) {
		FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_SPI);
		return (-ENODEV);
	}

	/* Convert to platform device */
	p_spi_dev = of_find_device_by_node(spi_node);
	if (!p_spi_dev) {
		FF_LOGE("of_find_device_by_node(..spi) failed.");
		return (-ENODEV);
	}

	FF_LOGD("spi controller(#%d) name: %s.", p_spi_dev->id, p_spi_dev->name);

	/* get spi1 */
	for (j = 0; j < 1; j++) {
		/* Find device tree node. */
		spi_node = of_find_compatible_node(spi_node, NULL, FF_COMPATIBLE_NODE_SPI);
		if (!spi_node) {
			FF_LOGE("of_find_compatible_node(.., '%s%d') failed.", FF_COMPATIBLE_NODE_SPI, j + 1);
			return (-ENODEV);
		}

		/* Convert to platform device */
		p_spi_dev = of_find_device_by_node(spi_node);
		if (!p_spi_dev) {
			FF_LOGE("of_find_device_by_node(..spi%d) failed.", j + 1);
			return (-ENODEV);
		}

		FF_LOGD("spi controller(#%d) name: %s.", p_spi_dev->id, p_spi_dev->name);
	}

	g_context_dts->p_spi_dev = p_spi_dev;

	/*get spiclk*/
	g_context_dts->spiclk = devm_clk_get(&p_spi_dev->dev, "spi-clk");
	if (!g_context_dts->spiclk) {
		FF_LOGE("devm_clk_get(..) failed");
		return (-ENODEV);
	} else {
        FF_LOGD("devm_clk_get(..) succeed");
    }

    // [TCTNB][FINGERPRINT]MODIFIED by Jiancong.Huang for sequence at 2021/9/14
    ff_ctl_enable_power(1);

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_free_pins(void)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	if (g_context_dts->ff_vdd != NULL)
	{
		devm_regulator_put(g_context_dts->ff_vdd);
		g_context_dts->ff_vdd = NULL;
	}

	if (g_context_dts->pinctrl != NULL )
	{
                devm_pinctrl_put(g_context_dts->pinctrl);
                FF_LOGD("free pinctrls success");
                g_context_dts->pinctrl = NULL;
	}

	if (g_context_dts->spiclk != NULL )
	{
		devm_clk_put(&g_context_dts->p_spi_dev->dev, g_context_dts->spiclk);
		g_context_dts->spiclk = NULL;
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_enable_spiclk(bool on)
{
	int err = 0;

	FF_LOGV("'%s' enter.", __func__);
	FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");

	do {
		static bool is_spiclk_en = false;
		FF_LOGI("is_spiclk_en = %d.", is_spiclk_en);
		/* Control the clock source. */
		if (on && !is_spiclk_en) {
			err = clk_prepare_enable(g_context_dts->spiclk);
			is_spiclk_en= true;
		} else if (!on && is_spiclk_en) {
			clk_disable_unprepare(g_context_dts->spiclk);
			is_spiclk_en = false;
		}
		FF_LOGD("'%s' is_spiclk_en = %d.", __func__, is_spiclk_en);
	} while(0);


	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_enable_power(bool on)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	FF_LOGD("'%s' is %s", __func__, on ? "on" : "off");
	FF_LOGD("'%s' skip temperarily", __func__);

	if (unlikely(!g_context_dts->pinctrl)) {
		return (-ENOSYS);
	}

	do {
		static bool is_power_on = false;
		FF_LOGI("is_power_on = %d.", is_power_on);
		/* Control the clock source. */
		// [TCTNB][FINGERPRINT]MODIFIED by Jiancong.Huang for sequence at 2021/9/14
		if (on && !is_power_on) {
//Begin add by qiaozhen.li for 11672345 on 2021/11/17
			err = fp_gpio_reset_low();
//End add by qiaozhen.li for 11672345 on 2021/11/17
      	 	mdelay(3);
         	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_PWR_ACT]);
     	    mdelay(5);
//Begin add by qiaozhen.li for 11672345 on 2021/11/17
	    err = fp_gpio_reset_high();
//End add by qiaozhen.li for 11672345 on 2021/11/17
			FF_LOGI("power on err = %d", err);
			is_power_on= true;
		} else if (!on && is_power_on) {
			#if FF_DTS_CONFIG_VDD_ALWAYS_ON	// test only for vdd-always-on
				F_LOGE("'%s' vdd always on = %d", __func__, FF_DTS_CONFIG_VDD_ALWAYS_ON);
			#else
//Begin add by qiaozhen.li for 11672345 on 2021/11/17
				err = fp_gpio_reset_low();
//End add by qiaozhen.li for 11672345 on 2021/11/17
				mdelay(3);
				err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_PWR_CLR]);
				mdelay(5);
				FF_LOGI("power off err = %d", err);
			#endif
			is_power_on = false;
		}
	} while(0);

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_reset_device(void)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	if (unlikely(!g_context_dts->pinctrl)) {
		return (-ENOSYS);
	}

	/* 3-1: Pull down RST pin. */
//Begin add by qiaozhen.li for 11672345 on 2021/11/17
	    err = fp_gpio_reset_high();
//End add by qiaozhen.li for 11672345 on 2021/11/17

	/* 3-2: Delay for 10ms. */
	usleep_range(10000, 10000);

	/* Pull up RST pin. */
//Begin add by qiaozhen.li for 11672345 on 2021/11/17
	err = fp_gpio_reset_low();
//End add by qiaozhen.li for 11672345 on 2021/11/17

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

const char *ff_ctl_arch_str(void)
{
    // TODO:
	return CONFIG_MTK_PLATFORM;
}


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
#include <linux/of_gpio.h>

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

// VDD dts node
#define FF_COMPATIBLE_NODE_POWER        "mediatek,fingerprint-power"
#define FF_DTS_NODE_VDD         "finger_vio28"

// VDD config
#define SUPPLY_3V3              3300000
#define FF_SUPPLY_VDD_MIN       SUPPLY_3V3
#define FF_SUPPLY_VDD_MAX       SUPPLY_3V3
// pinctrl config
typedef enum {
    FF_PINCTRL_STATE_POWER_OFF,
    FF_PINCTRL_STATE_POWER_ON,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} FF_PINCTRL_STATE_E;

typedef enum {
    FF_IRQ_STATE_DISABLE  = 0,
    FF_IRQ_STATE_ENABLE   = 1,
    FF_IRQ_STATE_UNKNOW   = 2
} FF_IRQ_STATE_E;


static int ff_spi_probe(struct spi_device *dev);
static int ff_spi_remove(struct spi_device *spi);
// pinctrl dts node
static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
	"fpsensor_power_low", "fpsensor_power_high",
	"fpsensor_rst_low", "fpsensor_rst_high", 
	"fpsensor_eint_low"
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
	struct spi_device *spidev;
	bool b_spiclk_enabled;
	u32 pinctrl_power;
} FF_DTS_NODE_CONTEXT_T;
static FF_DTS_NODE_CONTEXT_T ff_mt6833_context = {
	.pinctrl		= NULL,
	.spiclk			= NULL,
	.ff_vdd			= NULL,
	.b_spiclk_enabled = false,
	.spidev                 = NULL,
	.pinctrl_power = 2
}, *g_context_dts = &ff_mt6833_context;

extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);

/*}}}*/// +++++ +++++ +++++ +++++ +++++ 2. dts node storage +++++ +++++ +++++ +++++ +++++


static struct of_device_id  ff_spi_of_match[] = {
    { .compatible = "mediatek,fingerprint-spi-dev", },
};

static struct spi_driver ff_spi_driver = {
    .driver = {
        .name = "ff-spi",
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = ff_spi_of_match,
#endif
    },
    .probe  = ff_spi_probe,
    .remove = ff_spi_remove,
};

static int ff_spi_probe(struct spi_device *dev)
{
	printk("%s enter\n", __func__);
	g_context_dts->spidev = dev;
	return 0;
}

static int ff_spi_remove(struct spi_device *spi)
{
	return 0;
}

int ff_ctl_enable_power(bool on);
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifndef CONFIG_FINGERPRINT_REGULATOR_MT6371
#ifdef GPIO_FP_LDO_EN
static int ff_init_fp_ldo(void) {
	int err = 0, id_value = -1;
	unsigned int id_gpio;
	struct device_node *dev_node = NULL;
	struct platform_device *pdev = NULL;

	dev_node = of_find_compatible_node(NULL, NULL, "mediatek,gpio_fp_ldo_en");
	if (!dev_node) {
		FF_LOGE("of_find_compatible_node 'mediatek,gpio_fp_ldo_en' failed.");
		return (-ENODEV);
	}

	id_gpio = of_get_named_gpio(dev_node, "gpio-power-std", 0);
	if(id_gpio < 0) {
		FF_LOGE("get id gpio failed.");
		return (-ENODEV);
	}

	err = gpio_request(id_gpio, "gpio-power");
	if (err) {
		FF_LOGE("id gpio request failed.");
		return err;
	}

	id_value = gpio_get_value(id_gpio);
  	FF_LOGD("id-std value = %d", id_value);

  	gpio_free(id_gpio);
  	return id_value;
}
#endif
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifdef CONFIG_TCT_DEVICEINFO
const char *focaltech_info;
#endif
int ff_ctl_init_pins(int *irq_num)
{
	int err = 0, i = 0, j = 0;
	struct device_node *dev_node = NULL, *irq_node = NULL, *vdd_node = NULL;
	struct platform_device *pdev = NULL, *p_irq_dev  = NULL, *p_vdd_dev  = NULL;
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
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifndef CONFIG_FINGERPRINT_REGULATOR_MT6371
	#ifdef GPIO_FP_LDO_EN
	if(ff_init_fp_ldo() == 1) {
		g_context_dts->pin_states[0] = pinctrl_lookup_state(g_context_dts->pinctrl, "fpsensor_power2_low");
		g_context_dts->pin_states[1] = pinctrl_lookup_state(g_context_dts->pinctrl, "fpsensor_power2_high");
		i = 2;
	}
	#endif
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
	/* Register all pins. */
	for (i ; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
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

	/* Initialize the INT pin. */
	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_INT_ACT]);

	/*err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_SPI_CS]);
	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_SPI_CLK]);
	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_SPI_MO]);
	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_SPI_MI]);*/

	/* Retrieve the irq number. */
	/* Find device tree node. */
	irq_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_INT);
	if (!irq_node) {
		FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_INT);
		return (-ENODEV);
	}
	u32 irq_gpio = 0;
        err = of_property_read_u32(irq_node, "interrupts", &irq_gpio);
	if (err)
		FF_LOGD("'%s' irq_gpio = %d, err = %d", __func__, irq_gpio, err);
	/* Convert to platform device */

	p_irq_dev = of_find_device_by_node(irq_node);
	if (!p_irq_dev) {
		FF_LOGE("of_find_device_by_node(..irq) failed.");
		return (-ENODEV);
	}
	*irq_num = irq_of_parse_and_map(irq_node, 0);
	FF_LOGD("'%s' irq number is %d.", __func__, *irq_num);

	/* Get regulator */
	vdd_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE_POWER);
	if (!vdd_node) {
		FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE_POWER);
		g_context_dts->ff_vdd = NULL;
		return (-ENODEV);
	}

	/* Convert to platform device */
	p_vdd_dev = of_find_device_by_node(vdd_node);
	if (!p_vdd_dev) {
		FF_LOGE("of_find_device_by_node(..vdd) failed.");
		g_context_dts->ff_vdd = NULL;
		return (-ENODEV);
	}
	/* Get flag of the way control power */
	err = of_property_read_u32(vdd_node, "pinctrl_power", &g_context_dts->pinctrl_power);
	FF_LOGE("get pinctrl_power = %d.\n",g_context_dts->pinctrl_power);
	if (err) {
		FF_LOGE("read pinctrl_power fail.\n");
		return err;
	}
#ifdef CONFIG_TCT_DEVICEINFO
	err = of_property_read_string(vdd_node, "focaltech_info", &focaltech_info);
	FF_LOGE("get focaltech_info = %s.\n",focaltech_info);
	if (err) {
		FF_LOGE("read focaltech_info fail.\n");
		return err;
	}
#endif
	if (!g_context_dts->pinctrl_power){
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifdef CONFIG_FINGERPRINT_REGULATOR_MT6371
		FF_LOGD("finger_vio28 Can't use mt_pmic_vcn28_ldo_reg regulator.");
		g_context_dts->ff_vdd = NULL;
#else
		g_context_dts->ff_vdd = devm_regulator_get(&p_vdd_dev->dev, FF_DTS_NODE_VDD);
		if (IS_ERR(g_context_dts->ff_vdd)) {
			FF_LOGE("Can't find vcc_core regulator.");
			g_context_dts->ff_vdd = NULL;
		} else {
			err = regulator_set_voltage(g_context_dts->ff_vdd, FF_SUPPLY_VDD_MIN, FF_SUPPLY_VDD_MAX);
			FF_LOGI("Found vcc_core regulator and set it to : %d", g_context_dts->ff_vdd);
			if (err < 0){
			    FF_LOGE("Can't set voltage.");
			    return err;
			}
		}
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
	}
	/* Register spi driver */
	err = spi_register_driver(&ff_spi_driver);
	if (err < 0)
	{
	        printk("%s register spi driver fail\n", __func__);
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_free_pins(void)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	if (!g_context_dts->pinctrl_power && g_context_dts->ff_vdd != NULL)
	{
		devm_regulator_put(g_context_dts->ff_vdd);
		g_context_dts->ff_vdd = NULL;
	}
	
	if (g_context_dts->pinctrl != NULL )
	{
		devm_pinctrl_put(g_context_dts->pinctrl);
		g_context_dts->pinctrl = NULL;
	}
	if (g_context_dts->spidev != NULL)
	{
		spi_unregister_driver(&ff_spi_driver);
		g_context_dts->spidev = NULL;
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_enable_spiclk(bool on)
{
	int err = 0;

	FF_LOGD("%s enter, clock: '%s\n'.",__func__, on ? "enable" : "disabled");
	FF_LOGI("'%s' before is_spiclk_en = %d.",__func__, g_context_dts->b_spiclk_enabled);

	///* Control the clock source. */
	if (on == g_context_dts->b_spiclk_enabled) {
		FF_LOGE("'%s' skip duplicated spiclk %d", __func__, on);
		return err;
	}
	if (g_context_dts->spidev != NULL){
	/* Control the clock source. */
	if (on && !g_context_dts->b_spiclk_enabled) {
		mt_spi_enable_master_clk(g_context_dts->spidev);
		g_context_dts->b_spiclk_enabled= true;
	} else if (!on && g_context_dts->b_spiclk_enabled) {
		mt_spi_disable_master_clk(g_context_dts->spidev);
		g_context_dts->b_spiclk_enabled = false;
	}
	}
	FF_LOGD("'%s' after is_spiclk_en = %d.", __func__, g_context_dts->b_spiclk_enabled);
	

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

//Begin added by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifdef CONFIG_FINGERPRINT_REGULATOR_MT6371
extern int finger_power_enable(void);
extern int finger_power_disable(void);
#endif
//End added by liangjiaqiang for MODEL3-1890 on 2022-09-15

int ff_ctl_enable_power(bool on)
{
	int err = 0;
	static bool is_power_on = false;
	FF_LOGD("'%s' enter is %s", __func__, on ? "on" : "off");
	FF_LOGI("is_power_on = %d.", is_power_on);

	if (!g_context_dts->pinctrl_power && !g_context_dts->ff_vdd) {
		FF_LOGE("'%s' set power = %d, but vdd is NULL", __func__, on);
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifndef CONFIG_FINGERPRINT_REGULATOR_MT6371
		return err;
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
	}
	err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_CLR]);
	if (err)
	{
		FF_LOGE("select rst _clr fail!\n");
		return err;
	}
      	mdelay(1);
	/* Control the clock source. */
	if (on && !is_power_on) {
//Begin added by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifdef CONFIG_FINGERPRINT_REGULATOR_MT6371
		finger_power_enable();
		is_power_on= true;
//End added by liangjiaqiang for MODEL3-1890 on 2022-09-15
#else
		/* Pull up RST pin. */
		if (!g_context_dts->pinctrl_power)
			err = regulator_enable(g_context_dts->ff_vdd);
		else
			err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_POWER_ON]);
		FF_LOGI("power_enable err = %d", err);
		if (!err) {
			mdelay(2);
			is_power_on= true;
			err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_ACT]);
		}
#endif
	} else if (!on && is_power_on) {
//Begin added by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifdef CONFIG_FINGERPRINT_REGULATOR_MT6371
		 finger_power_disable();
		 is_power_on = false;
//End added by liangjiaqiang for MODEL3-1890 on 2022-09-15
#else
		if (!g_context_dts->pinctrl_power)
			err = regulator_disable(g_context_dts->ff_vdd);
		else
			err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_POWER_OFF]);
		FF_LOGI("power_disable err = %d", err);
		if (!err) {
			is_power_on = false;
		}
#endif
	}	
	//err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_ACT]);
	mdelay(5);
	FF_LOGV("'%s' leave.", __func__);
	return err;
}

int ff_ctl_reset_device(int on)
{
	int err = 0;
	FF_LOGV("'%s' enter.state = %d", __func__,on);

	if (unlikely(!g_context_dts->pinctrl)) {
		return (-ENOSYS);
	}
	/* 3-1: Pull down RST pin. */
	if (!on)
		err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_ACT]);
	else
	/* Pull up RST pin. */
		err = pinctrl_select_state(g_context_dts->pinctrl, g_context_dts->pin_states[FF_PINCTRL_STATE_RST_CLR]);
	/* 3-2: Delay for 10ms. */
	usleep_range(10000, 10000);
	FF_LOGV("'%s' leave.", __func__);
	return err;
}

const char *ff_ctl_arch_str(void)
{
    // TODO:
	return CONFIG_MTK_PLATFORM;
}


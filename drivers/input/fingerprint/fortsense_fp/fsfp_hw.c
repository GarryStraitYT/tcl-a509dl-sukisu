/**
 * The device control driver for Fortsense's fingerprint sensor.
 *
 * Copyright (C) 2018 Fortsense Corporation. <http://www.fortsense.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "fsfp_ctl.h"

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#endif

#define MODULE_NAME "fortsense-fsfp_hw"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME"-%d: "fmt, __LINE__, ##args)


#ifndef CONFIG_OF
#error "error: this driver 'MODULE_NAME' only support dts."
#endif

static struct fsfp_ctl_device *fsfp_ctl_dev = NULL;
typedef enum {
	SF_PIN_STATE_PWR_ON,
	SF_PIN_STATE_PWR_OFF,
    SF_PIN_STATE_RST_HIGH,
    SF_PIN_STATE_RST_LOW,
    SF_PIN_STATE_INT_SET,
	
    /* Array size */
    SF_PIN_STATE_MAX
} fsfp_pin_state_t;


static struct pinctrl *fsfp_pinctrl = NULL;
static struct pinctrl_state *fsfp_pin_states[SF_PIN_STATE_MAX] = {NULL, };
#if defined CONFIG_TCT_DEVICEINFO
extern char fp_module_name[256];
#endif
static const char *fsfp_pinctrl_state_names[SF_PIN_STATE_MAX] = {
    FINGER_POWER_OFF, FINGER_POWER_ON, FINGER_RESET_HIGH, FINGER_RESET_LOW, FINGER_INT_SET,
};

static int fsfp_spi_clock_enable(bool on)
{
    int err = 0;
    static int count;

    if (on && (count == 0)) {
        mt_spi_enable_master_clk(fsfp_ctl_dev->pdev);
        count = 1;
    }
    else if ((count > 0) && (on == 0)) {
        mt_spi_disable_master_clk(fsfp_ctl_dev->pdev);
        count = 0;
    }
    return err;
}

static int fsfp_ctl_device_reset(bool output)
{
    int err = 0;
    xprintk(KERN_ERR, "%s(..) enter.\n", __FUNCTION__);
    if (fsfp_pinctrl == NULL) {
        xprintk(KERN_ERR, "fsfp_pinctrl is NULL.\n");
        return -1;
    }

    if (output) {
        err = pinctrl_select_state(fsfp_pinctrl, fsfp_pin_states[SF_PIN_STATE_RST_HIGH]);
    }
    else {
        err = pinctrl_select_state(fsfp_pinctrl, fsfp_pin_states[SF_PIN_STATE_RST_LOW]);
    }
    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int fsfp_ctl_device_power_by_regulator(bool on)
{
    static bool isPowerOn = false;
    int err = 0;
    xprintk(SF_LOG_LEVEL, "%s onff: %d stat: %d(..) enter.\n", __FUNCTION__, on, isPowerOn);

    if (fsfp_ctl_dev->vdd_reg == NULL) {
        xprintk(KERN_ERR, "ctl_dev->vdd_reg is NULL.\n");
        return (-ENODEV);
    }

    if (on && !isPowerOn) {
        err = regulator_enable(fsfp_ctl_dev->vdd_reg);

        if (err) {
            xprintk(KERN_ERR, "Regulator vdd enable failed err = %d\n", err);
            return err;
        }

        isPowerOn = true;
        msleep(10);
    }
    else if (!on && isPowerOn) {
        err = regulator_disable(fsfp_ctl_dev->vdd_reg);

        if (err) {
            xprintk(KERN_ERR, "Regulator vdd disable failed err = %d\n", err);
            return err;
        }

        isPowerOn = false;
    }

    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int fsfp_ctl_device_power(bool on)
{
    int err = 0;
    err = fsfp_ctl_device_power_by_regulator(on);
    return err;
}

static int fsfp_ctl_device_free_gpio(struct fsfp_ctl_device *ctl_dev)
{
    int err = 0;
    xprintk(KERN_ERR, "%s(..) enter, free resource.\n", __FUNCTION__);

    if (ctl_dev->pdev->dev.of_node) {
        ctl_dev->pdev->dev.of_node = NULL;
    }


    if (fsfp_pinctrl) {
        pinctrl_put(fsfp_pinctrl);
        fsfp_pinctrl = NULL;
    }

    if (ctl_dev->vdd_reg) {
        devm_regulator_put(ctl_dev->vdd_reg);
        ctl_dev->vdd_reg = NULL;
    }

    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int fsfp_ctl_device_init_gpio_pins(struct fsfp_ctl_device *ctl_dev)
{
    int err = 0;
	struct device_node *dev_node = NULL, *vdd_node = NULL, *irq_node = NULL;
    struct platform_device *pdev = NULL, *p_vdd_dev  = NULL, *p_irq_dev  = NULL;
    fsfp_ctl_dev = ctl_dev;
    xprintk(KERN_ERR, "%s(..) enter.\n", __FUNCTION__);
    ctl_dev->pdev->dev.of_node = of_find_compatible_node(NULL, NULL, COMPATIBLE_SW_FP);

    if (!ctl_dev->pdev->dev.of_node) {
        xprintk(KERN_ERR, "of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }
	
    dev_node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint-pinctrl");
	if (!dev_node) {
		xprintk(KERN_ERR, "of_find_compatible_node(.., 'mediatek,fingerprint-pinctrl') failed.");
		return (-ENODEV);
	}

    pdev = of_find_device_by_node(dev_node);
	if (!pdev) {
		xprintk(KERN_ERR, "of_find_device_by_node(..) failed.");
		return (-ENODEV);
	}
    fsfp_pinctrl = pinctrl_get(&pdev->dev);
    if (!fsfp_pinctrl) {
        xprintk(KERN_ERR, "pinctrl_get(..) failed.\n");
        return (-ENODEV);
    }

    /* init irq resources */
    irq_node = of_find_compatible_node(NULL, NULL, "mediatek,fpsensor-eint");
    if (!irq_node) {
		xprintk(KERN_ERR, "of_find_compatible_node(.., 'mediatek,fpsensor-eint') failed.");
		return (-ENODEV);
    }
    u32 irq_gpio = 0;
    err = of_property_read_u32(irq_node, "interrupts", &irq_gpio);
    if (err)
		xprintk(KERN_ERR, "'%s' irq_gpio = %d, err = %d", __func__, irq_gpio, err);
	/* Convert to platform device */

	p_irq_dev = of_find_device_by_node(irq_node);
	if (!p_irq_dev) {
		xprintk(KERN_ERR, "of_find_device_by_node(..irq) failed.");
		return (-ENODEV);
	}
    ctl_dev->irq_num = irq_of_parse_and_map(irq_node, 0);
    xprintk(KERN_INFO, "irq number is %d.\n", ctl_dev->irq_num);
    {
        int i = 0;

        for (i = 0; i < SF_PIN_STATE_MAX; ++i) {
            fsfp_pin_states[i] = pinctrl_lookup_state(fsfp_pinctrl,
                                 fsfp_pinctrl_state_names[i]);

            if (!fsfp_pin_states[i]) {
                xprintk(KERN_ERR, "can't find '%s' pinctrl_state.\n",
                        fsfp_pinctrl_state_names[i]);
                err = (-ENODEV);
                break;
            }
        }

        if (i < SF_PIN_STATE_MAX) {
            xprintk(KERN_ERR, "%s() failed.\n", __FUNCTION__);
        }
         /*init int*/
		 err =pinctrl_select_state(fsfp_pinctrl, fsfp_pin_states[SF_PIN_STATE_INT_SET]);
		 if (err) {
                xprintk(KERN_ERR, "%s() pinctrl_select_state(SF_PIN_STATE_INT_SETHIGH) failed.\n", __FUNCTION__, fsfp_pinctrl_state_names[SF_PIN_STATE_INT_SET]);
                return (-ENODEV);
         }
        /* init spi,sunch as cs clck miso mosi mode, gpio pullup pulldown */
        for (i = SF_PIN_STATE_INT_SET + 1; i < SF_PIN_STATE_MAX; ++i) {
            err = pinctrl_select_state(fsfp_pinctrl, fsfp_pin_states[i]);

            if (err) {
                xprintk(KERN_ERR, "%s() pinctrl_select_state(%s) failed.\n", __FUNCTION__, fsfp_pinctrl_state_names[i]);
                break;
            }

            xprintk(KERN_INFO, "pinctrl_select_state(%s) ok.\n", fsfp_pinctrl_state_names[i]);
        }
    }
	vdd_node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint-power");
	if (!vdd_node) {
		xprintk(KERN_ERR, "of_find_compatible_node(.., 'mediatek,fingerprint-power') failed.");
		ctl_dev->vdd_reg = NULL;
		return (-ENODEV);
	}

	p_vdd_dev = of_find_device_by_node(vdd_node);
	if (!p_vdd_dev) {
		xprintk(KERN_ERR, "of_find_device_by_node(..vdd) failed.");
		ctl_dev->vdd_reg = NULL;
		return (-ENODEV);
	}
    xprintk(KERN_ERR, "%s:regulator get2 \n",__FUNCTION__);
    ctl_dev->vdd_reg = devm_regulator_get(&p_vdd_dev->dev, "finger_vio28");
     
    if (IS_ERR(ctl_dev->vdd_reg)) {
        err = PTR_ERR(ctl_dev->vdd_reg);
        xprintk(KERN_ERR, "Regulator get failed vdd err = %d\n", err);
        return err;
    }
     
    if (regulator_count_voltages(ctl_dev->vdd_reg) > 0) {
        err = regulator_set_voltage(ctl_dev->vdd_reg, SF_VDD_MIN_UV,
                                    SF_VDD_MAX_UV);

        if (err) {
            xprintk(KERN_ERR, "Regulator set_vtg failed vdd err = %d\n", err);
            return err;
        }
    }

    fsfp_ctl_device_power(true);
	#if defined CONFIG_TCT_DEVICEINFO
        sprintf(fp_module_name,"FS9012:FORTSENSE:Sunwin:AYB0000242C1");
    #endif
    xprintk(SF_LOG_LEVEL, "%s(..) ok! exit.\n", __FUNCTION__);
    return err;
}

int fsfp_platform_init(struct fsfp_ctl_device *ctl_dev)
{
    int err = 0;
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);

    if (ctl_dev) {
        ctl_dev->gpio_init  = fsfp_ctl_device_init_gpio_pins;
        ctl_dev->power_on   = fsfp_ctl_device_power;
        ctl_dev->spi_clk_on = fsfp_spi_clock_enable;
        ctl_dev->reset      = fsfp_ctl_device_reset;
        ctl_dev->free_gpio  = fsfp_ctl_device_free_gpio;
    }
    else {
        xprintk(KERN_ERR, "%s() ctl_dev is NULL.\n", __FUNCTION__);
        err = -1;
    }

    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

void fsfp_platform_exit(struct fsfp_ctl_device *ctl_dev)
{
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);

    if (ctl_dev) {
        ctl_dev->gpio_init  = NULL;
        ctl_dev->power_on   = NULL;
        ctl_dev->spi_clk_on = NULL;
        ctl_dev->reset      = NULL;
        ctl_dev->free_gpio  = NULL;
    }
    else {
        xprintk(KERN_ERR, "%s() ctl_dev is NULL.\n", __FUNCTION__);
    }

    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
}

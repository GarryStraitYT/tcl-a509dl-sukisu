/*
 *  Generic DT helper functions for touchscreen devices
 *
 *  Copyright (c) 2014 Sebastian Reichel <sre@kernel.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

/*
 *  History 1.0 added by meng.zahng for task xxx on 20210726
*/


#include <linux/device.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/input/tct_touch.h>
#include <tct_debug.h>


/*****************************************************************************
* Macro definitions using #define
*****************************************************************************/
#define KEY_DOUBLECLICK_WAKEUP 	 0xf9 //kernel report

#define FUNC_ON 1
#define FUNC_OFF 0


/*****************************************************************************
* sys/class define here
*****************************************************************************/
//class
static struct class *tct_touch_class = NULL;
static struct class *tct_display_class = NULL;
static struct class *tct_led_class = NULL;
//device
static struct device *touch_dev = NULL;
static struct device *led_dev = NULL;
#if defined(CONFIG_TCT_FEATURE_AOD_FUNCTION)
static struct device *lcm_dev = NULL;
#endif
struct tct_touch_data *tct_ts_data;
EXPORT_SYMBOL(tct_ts_data);
/*****************************************************************************
* DEBUG function define here
*****************************************************************************/

//gesture:double wakeup
void (*tct_set_double_wakeup_en)(unsigned int enable);
EXPORT_SYMBOL(tct_set_double_wakeup_en);

static bool tct_double_wakeup_status = false; //kernel default set Disable

static ssize_t double_wakeup_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf) {

    if (tct_double_wakeup_status) {
        return snprintf(buf, PAGE_SIZE, "1\n");
    } else {
        return snprintf(buf, PAGE_SIZE, "0\n");
    }
}

static ssize_t double_wakeup_enable_store(struct device* dev,
    struct device_attribute *attr, const char *buf, size_t count) {

    unsigned long state;
    ssize_t ret;

    if(!tct_set_double_wakeup_en){
        goto function_err;
    }

    ret = kstrtoul(buf, 2, &state);
    if (ret){
        TCT_ERROR("read gesture_enable file failed\n");
    }

    if(state == FUNC_ON){
        tct_set_double_wakeup_en(FUNC_ON);
        tct_double_wakeup_status = true;
    }else if(state == FUNC_OFF){
        tct_set_double_wakeup_en(FUNC_OFF);
        tct_double_wakeup_status = false;
    }else{
        TCT_ERROR("invalid cmd for gesture enbale\n");
    }
    return count;
function_err:
    return count;
}

//simulate proximity
void (*tct_set_proximity_en)(unsigned int enable);
EXPORT_SYMBOL(tct_set_proximity_en);

static bool tct_proximity_status = false; //kernel default set Disable

static ssize_t prox_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf) {

    if (tct_proximity_status) {
        return snprintf(buf, PAGE_SIZE, "1\n");
    } else {
        return snprintf(buf, PAGE_SIZE, "0\n");
    }
}

static ssize_t prox_enable_store(struct device* dev,
    struct device_attribute *attr, const char *buf, size_t count) {

    unsigned long state;
    ssize_t ret;

    if(!tct_set_proximity_en){
        goto function_err;
    }

    ret = kstrtoul(buf, 2, &state);
    if (ret){
        TCT_ERROR("read gesture_enable file failed\n");
    }

    if(state == FUNC_ON){
        tct_set_proximity_en(FUNC_ON);
        tct_proximity_status = true;
    }else if(state == FUNC_OFF){
        tct_set_proximity_en(FUNC_OFF);
        tct_proximity_status = false;
    }else{
        TCT_ERROR("invalid cmd for gesture enbale\n");
    }
    return count;
function_err:
    return count;
}

static ssize_t prox_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	TCT_INFO("prox_status_show() proximity_status=%d\n", tct_proximity_status);
	return sprintf(buf, "%u\n",  tct_proximity_status);
}

static ssize_t prox_status_store(struct device *dev, struct device_attribute *attr, const char *buf,
					size_t count)
{
	TCT_INFO("prox_status_store() buf=%s\n", buf);

	return count;
}

//TP edge process
//void (*tct_set_edge_process_en)(unsigned int enable);
void (*tct_grip_mode_set)(u8 screen_mode, u8 level);
EXPORT_SYMBOL(tct_grip_mode_set);

static ssize_t tct_grip_mode_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "screen_mode=%d, grip_level=%d\n",
            tct_ts_data->tct_grip_mode, tct_ts_data->tct_grip_level);
}

static ssize_t tct_grip_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int val[2] = {0};

    //Begin add by bing-zhang for 11700483 on 2022/01/13
    if (!strcmp(CONFIG_ARCH_MTK_PROJECT, "rapidtf")) {
	return 0;
    }else {
	mutex_lock(&tct_ts_data->tct_touch_mutex);
	sscanf(buf, "%d,%d", &val[0], &val[1]);
	pr_info("%s, val=%d,%d previos screen_mode=%d, grip_level=%d",
		__func__, val[0], val[1], tct_ts_data->tct_grip_mode, tct_ts_data->tct_grip_level);
	tct_ts_data->tct_grip_mode = val[0];
	tct_ts_data->tct_grip_level = val[1];
	if (tct_grip_mode_set != NULL)
		tct_grip_mode_set(tct_ts_data->tct_grip_mode, tct_ts_data->tct_grip_level);
	mutex_unlock(&tct_ts_data->tct_touch_mutex);

	return count;
    }
    //End add by bing-zhang for 11700483 on 2022/01/13
}

//TP charger mode process
void (*tct_set_charge_process_en)(unsigned int enable);
EXPORT_SYMBOL(tct_set_charge_process_en);

static bool tct_charge_mode_status = false; //kernel default set Disable

static ssize_t charge_process_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf) {

    if (tct_charge_mode_status) {
        return snprintf(buf, PAGE_SIZE, "1\n");
    } else {
        return snprintf(buf, PAGE_SIZE, "0\n");
    }
}

static ssize_t charge_process_enable_store(struct device* dev,
    struct device_attribute *attr, const char *buf, size_t count) {

    unsigned long state;
    ssize_t ret;

    if (!tct_set_charge_process_en) {
        goto function_err;
    }

    ret = kstrtoul(buf, 2, &state);
    if (ret){
        TCT_ERROR("read charge_process_enable file failed\n");
    }

    if(state == FUNC_ON){
        tct_set_charge_process_en(FUNC_ON);
        tct_charge_mode_status = true;
    }else if(state == FUNC_OFF){
        tct_set_charge_process_en(FUNC_OFF);
        tct_charge_mode_status = false;
    }else{
        TCT_ERROR("invalid cmd for charge_process enbale\n");
    }
    return count;
function_err:
    return count;
}

/* Requst  File Access Rights 0664 */
DEVICE_ATTR(charger_mode, 0664, charge_process_enable_show, charge_process_enable_store);
DEVICE_ATTR(grip_mode, 0664, tct_grip_mode_show, tct_grip_mode_store);
DEVICE_ATTR(double_wakeup_enable, 0664, double_wakeup_enable_show, double_wakeup_enable_store);
DEVICE_ATTR(prox_enable, 0664, prox_enable_show, prox_enable_store);
DEVICE_ATTR(prox_status, 0664, prox_status_show, prox_status_store);

static struct attribute *touchscreen_attributes[] = {
	&dev_attr_charger_mode.attr,
    &dev_attr_grip_mode.attr,
    &dev_attr_double_wakeup_enable.attr,
    &dev_attr_prox_enable.attr,
    &dev_attr_prox_status.attr,
	NULL,
};

static struct attribute_group touchscreen_attr_group = {
	.attrs = touchscreen_attributes,
};

//sunlight mode current boost
void (*tct_sunlight_process_en)(unsigned int enable);
EXPORT_SYMBOL(tct_sunlight_process_en);
bool tct_current_boost_status = false;

static ssize_t current_boost_show(struct device *dev,
    struct device_attribute *attr, char *buf) {

    if (tct_current_boost_status) {
        return snprintf(buf, PAGE_SIZE, "1\n");
    } else {
        return snprintf(buf, PAGE_SIZE, "0\n");
    }
}

static ssize_t current_boost_store(struct device* dev,
    struct device_attribute *attr, const char *buf, size_t count) {

    unsigned long state;
    ssize_t ret;

    if (!tct_sunlight_process_en) {
        goto function_err;
    }

    ret = kstrtoul(buf, 2, &state);
    if (ret){
        pr_err("read current_boost file failed\n");
    }

    pr_info("%s() sunlight boost state=%d", __func__, state);

    if (state == FUNC_ON) {
	tct_current_boost_status = true;
        tct_sunlight_process_en(FUNC_ON);
    } else if(state == FUNC_OFF) {
        tct_current_boost_status = false;
        tct_sunlight_process_en(FUNC_OFF);
    } else {
        pr_err("invalid cmd for current_boost\n");
    }
    return count;

function_err:
    return count;
}

/* Requst  File Access Rights 0664 */
DEVICE_ATTR(current_boost, 0664, current_boost_show, current_boost_store);


//night mode
void (*tct_night_mode_en)(unsigned int enable);
EXPORT_SYMBOL(tct_night_mode_en);

static bool tct_night_mode_status = false;

static ssize_t night_mode_show(struct device *dev,
    struct device_attribute *attr, char *buf) {

    if(tct_night_mode_status){
        return snprintf(buf, PAGE_SIZE, "1\n");
    }else{
        return snprintf(buf, PAGE_SIZE, "0\n");
    }
}

static ssize_t night_mode_store(struct device* dev,
    struct device_attribute *attr, const char *buf, size_t count) {

    unsigned long state;
    ssize_t ret;

    if (!tct_night_mode_en) {
        goto function_err;
    }

    ret = kstrtoul(buf, 2, &state);
    if (ret){
        pr_err("read night_mode file failed\n");
    }

    pr_info("%s() night mode status=%d", __func__, state);

    if(state == 1){
        tct_night_mode_en(FUNC_ON);
        tct_night_mode_status = true;
    }else if(state == 0){
        tct_night_mode_en(FUNC_OFF);
        tct_night_mode_status = false;
    }else{
        pr_err("invalid cmd for current_boost\n");
    }
    return count;
function_err:
    return count;
}

/* Requst  File Access Rights 0664 */
DEVICE_ATTR(night_mode, 0664, night_mode_show, night_mode_store);

#if defined(CONFIG_TCT_FEATURE_AOD_FUNCTION)
//AOD mode process
void (*tct_set_aod_process_en)(unsigned int enable);
EXPORT_SYMBOL(tct_set_aod_process_en);

static bool tct_aod_mode_status = false; //kernel default set Disable

static ssize_t aod_process_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf) {

    if (tct_aod_mode_status) {
        return snprintf(buf, PAGE_SIZE, "1\n");
    } else {
        return snprintf(buf, PAGE_SIZE, "0\n");
    }
}

static ssize_t aod_process_enable_store(struct device* dev,
    struct device_attribute *attr, const char *buf, size_t count) {

    unsigned long state;
    ssize_t ret;

    if (!tct_set_aod_process_en) {
        goto function_err;
    }

    ret = kstrtoul(buf, 2, &state);
    if (ret){
        TCT_ERROR("read aod_process_enable file failed\n");
    }

    if(state == FUNC_ON){
        tct_set_aod_process_en(FUNC_ON);
        tct_aod_mode_status = true;
    }else if(state == FUNC_OFF){
        tct_set_aod_process_en(FUNC_OFF);
        tct_aod_mode_status = false;
    }else{
        TCT_ERROR("invalid cmd for aod_process enbale\n");
    }
    return count;
function_err:
    return count;
}

DEVICE_ATTR(aod_mode, 0664, aod_process_enable_show, aod_process_enable_store);
#endif

/*****************************************************************************
*  Name: tct_bsp_display_init
*  Brief: 1. create bsp node as interface.
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int __init tct_bsp_display_init(void)
{
    int error = 0;
    struct tct_touch_data *ts_data = NULL;

	TCT_FUNC_ENTER();

    /* malloc memory for global struct variable */
    ts_data = (struct tct_touch_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        TCT_ERROR("allocate memory for ts_data fail");
        return -ENOMEM;
    }

    tct_ts_data = ts_data;

    ts_data->tct_grip_mode = VERTICAL;
    ts_data->tct_grip_level = 1;
    mutex_init(&ts_data->tct_touch_mutex);

	if (!tct_touch_class) {
		tct_touch_class = class_create(THIS_MODULE, "tct_touch");
		if (IS_ERR(tct_touch_class)) {
			TCT_ERROR("Failed to create class(tct_touch)!\n");
			return -1;
		}

		if (!touch_dev) {
			touch_dev = device_create(tct_touch_class, NULL, 0, NULL, "tct_touch_dev");
			if (IS_ERR(touch_dev)) {
				TCT_ERROR("Failed to create device(touch)!\n");
				return -1;
			}

           	 error = sysfs_create_group(&touch_dev->kobj, &touchscreen_attr_group);
           	 if (error < 0)
               	 pr_err("Failed to create tct/touch sysfs group!");
		}
   	 }

      if (!tct_display_class) {
		tct_display_class = class_create(THIS_MODULE, "tct_display");
		if (IS_ERR(tct_display_class)) {
			TCT_ERROR("Failed to create class(tct_display)!\n");
			return -1;
		}
#if defined(CONFIG_TCT_FEATURE_AOD_FUNCTION)
		if (!lcm_dev) {
			lcm_dev = device_create(tct_display_class, NULL, 0, NULL, "tct_display_dev");
			if (IS_ERR(lcm_dev))
			{
			    TCT_ERROR("Failed to create device(tct_display_dev)!\n");
			    return -1;
			}			
			if (device_create_file(lcm_dev, &dev_attr_aod_mode) < 0){
				TCT_ERROR("Failed to create /sys/class/tct_display/tct_display_dev/aod_mode file!\n");
				return -1;
		    	 }
		} 
			
#endif
	}
      if (!tct_led_class) {
			tct_led_class = class_create(THIS_MODULE, "tct_led");
			if (IS_ERR(tct_led_class))
			{
			    TCT_ERROR("Failed to create device(tct_led)!\n");
			    return -1;
			}
		if (!led_dev) {
			led_dev = device_create(tct_led_class, NULL, 0, NULL, "tct_led_dev");
			if (IS_ERR(led_dev))
			{
			    TCT_ERROR("Failed to create device(tct_led_dev)!\n");
			    return -1;
			}		

			if (device_create_file(led_dev, &dev_attr_current_boost) < 0){
				TCT_ERROR("Failed to create /sys/class/tct_led/tct_led_dev/current_boost file!\n");
				return -1;
			}

			if (device_create_file(led_dev, &dev_attr_night_mode) < 0){
				TCT_ERROR("Failed to create /sys/class/tct_led/tct_led_dev/night_mode file!\n");
				return -1;
			}
		}
	}

	TCT_FUNC_EXIT();
	return 0;
}

static void __exit tct_bsp_display_exit(void)
{
    TCT_FUNC_ENTER();

    TCT_FUNC_EXIT();
}

module_init(tct_bsp_display_init);
module_exit(tct_bsp_display_exit);

MODULE_AUTHOR("TCT HZ BSP Team");
MODULE_DESCRIPTION("TCT HZ BSP Driver");
MODULE_LICENSE("GPL v2");



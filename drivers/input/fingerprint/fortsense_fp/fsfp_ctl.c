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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include "fsfp_ctl.h"


//---------------------------------------------------------------------------------
#define SF_DRV_VERSION "v2.3.2-2020-03-19"

#define MODULE_NAME "fortsense-fsfp_ctl"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME"-%d: "fmt, __LINE__, ##args)

#define SF_DEFAULT_SPI_SPEED  (1 * 1000 * 1000)
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
static long fsfp_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int fsfp_ctl_init_irq(void);
static int fsfp_ctl_init_input(void);
#ifdef CONFIG_COMPAT
static long fsfp_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif


extern int fsfp_platform_init(struct fsfp_ctl_device *ctl_dev);
extern void fsfp_platform_exit(struct fsfp_ctl_device *ctl_dev);
static bool pointjudge=false;
extern int fb_blank(struct fb_info *info, int blank);

//---------------------------------------------------------------------------------

static struct file_operations fsfp_ctl_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = fsfp_ctl_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = fsfp_ctl_compat_ioctl,
#endif
};

static struct fsfp_ctl_device fsfp_ctl_dev = {
    .miscdev = {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "fortsense_fp",
        .fops   = &fsfp_ctl_fops,
    },
    .rst_num = 0,
    .irq_pin = 0,
    .irq_num = 0,
    .spi_buf_size = 25 * 1024,
};



static int fsfp_remove(fsfp_device_t *pdev);
static int fsfp_probe(fsfp_device_t *pdev);

static struct of_device_id  fsfp_of_match[] = {
    { .compatible = COMPATIBLE_SW_FP, },
    {},
};

static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias = "fortsense-fp",
        .bus_num = 5,
        .chip_select = 0,
        .mode = SPI_MODE_0,
    },
};

static int fsfp_ctl_spi_speed(unsigned int speed)
{
    fsfp_ctl_dev.pdev->max_speed_hz = speed;
    spi_setup(fsfp_ctl_dev.pdev);
    return 0;
}

static fsfp_driver_t fsfp_driver = {
    .driver = {
        .name = "fortsense-fp",
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = fsfp_of_match,
#endif
    },
    .probe  = fsfp_probe,
    .remove = fsfp_remove,
};

static fsfp_version_info_t fsfp_hw_ver;


//---------------------------------------------------------------------------------

static void fsfp_ctl_device_event(struct work_struct *ws)
{
    char *uevent_env[2] = { SF_INT_EVENT_NAME, NULL };
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    kobject_uevent_env(&fsfp_ctl_dev.miscdev.this_device->kobj,
                       KOBJ_CHANGE, uevent_env);
	xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
}

static irqreturn_t fsfp_ctl_device_irq(int irq, void *dev_id)
{
    disable_irq_nosync(irq);
    xprintk(SF_LOG_LEVEL, "%s(irq = %d, ..) toggled.\n", __FUNCTION__, irq);
    schedule_work(&fsfp_ctl_dev.work_queue);
#ifdef CONFIG_PM_WAKELOCKS
    __pm_wakeup_event(&fsfp_ctl_dev.wakelock, msecs_to_jiffies(2*HZ));
#else
    wake_lock_timeout(&fsfp_ctl_dev.wakelock, msecs_to_jiffies(5000));
#endif
    enable_irq(irq);
	xprintk(SF_LOG_LEVEL, "%s(irq = %d, ..) relased.\n", __FUNCTION__, irq);
    return IRQ_HANDLED;
}

static int fsfp_ctl_report_key_event(struct input_dev *input, fsfp_key_event_t *kevent)
{
    int err = 0;
    unsigned int key_code = KEY_UNKNOWN;
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);

    switch (kevent->key) {
        case SF_KEY_HOME:
            key_code = KEY_HOME;
            break;

        case SF_KEY_MENU:
            key_code = KEY_MENU;
            break;

        case SF_KEY_BACK:
            key_code = KEY_BACK;
            break;

        case SF_KEY_F11:
            key_code = 0xFF;
            break;

        case SF_KEY_ENTER:
            key_code = KEY_ENTER;
            break;

        case SF_KEY_UP:
            key_code = KEY_UP;
            break;

        case SF_KEY_LEFT:
            key_code = KEY_LEFT;
            break;

        case SF_KEY_RIGHT:
            key_code = KEY_RIGHT;
            break;

        case SF_KEY_DOWN:
            key_code = KEY_DOWN;
            break;

        case SF_KEY_WAKEUP:
            key_code = 0xFD;
            break;

        default:
            break;
    }

    input_report_key(input, key_code, kevent->value);
    input_sync(input);
    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static const char *fsfp_ctl_get_version(void)
{
    static char version[SF_DRV_VERSION_LEN] = {'\0', };
    strncpy(version, SF_DRV_VERSION, SF_DRV_VERSION_LEN);
    version[SF_DRV_VERSION_LEN - 1] = '\0';
    return (const char *)version;
}
static const char* fs_ctl_cmd_names(unsigned int cmd)
{
    switch (cmd) {
        case SF_IOC_INIT_DRIVER        : return FS_CTL_CMD_NAME(SF_IOC_INIT_DRIVER);
        case SF_IOC_DEINIT_DRIVER        : return FS_CTL_CMD_NAME(SF_IOC_DEINIT_DRIVER);
        case SF_IOC_RESET_DEVICE       : return FS_CTL_CMD_NAME(SF_IOC_RESET_DEVICE);
        case SF_IOC_ENABLE_IRQ         : return FS_CTL_CMD_NAME(SF_IOC_ENABLE_IRQ);
        case SF_IOC_DISABLE_IRQ        : return FS_CTL_CMD_NAME(SF_IOC_DISABLE_IRQ);
        case SF_IOC_REQUEST_IRQ     : return FS_CTL_CMD_NAME(SF_IOC_REQUEST_IRQ);
        case SF_IOC_ENABLE_SPI_CLK    : return FS_CTL_CMD_NAME(SF_IOC_ENABLE_SPI_CLK);
        case SF_IOC_DISABLE_SPI_CLK       : return FS_CTL_CMD_NAME(SF_IOC_DISABLE_SPI_CLK);
        case SF_IOC_ENABLE_POWER      : return FS_CTL_CMD_NAME(SF_IOC_ENABLE_POWER);
        case SF_IOC_DISABLE_POWER   : return FS_CTL_CMD_NAME(SF_IOC_DISABLE_POWER);
        case SF_IOC_REPORT_KEY_EVENT        : return FS_CTL_CMD_NAME(SF_IOC_REPORT_KEY_EVENT);
        case SF_IOC_SYNC_CONFIG        : return FS_CTL_CMD_NAME(SF_IOC_SYNC_CONFIG);
        case SF_IOC_SPI_SPEED : return FS_CTL_CMD_NAME(SF_IOC_SPI_SPEED);
        case SF_IOC_ATTRIBUTE : return FS_CTL_CMD_NAME(SF_IOC_ATTRIBUTE);
        case SF_IOC_GET_VERSION    : return FS_CTL_CMD_NAME(SF_IOC_GET_VERSION);
		case SF_IOC_SET_LIB_VERSION    : return FS_CTL_CMD_NAME(SF_IOC_SET_LIB_VERSION);
        case SF_IOC_GET_LIB_VERSION       : return FS_CTL_CMD_NAME(SF_IOC_GET_LIB_VERSION);
        case SF_IOC_SET_SPI_BUF_SIZE      : return FS_CTL_CMD_NAME(SF_IOC_SET_SPI_BUF_SIZE);
        case SF_IOC_SET_RESET_OUTPUT   : return FS_CTL_CMD_NAME(SF_IOC_SET_RESET_OUTPUT);
        case SPI_IOC_RD_MAX_SPEED_HZ        : return FS_CTL_CMD_NAME(SPI_IOC_RD_MAX_SPEED_HZ);
        case SPI_IOC_WR_MAX_SPEED_HZ        : return FS_CTL_CMD_NAME(SPI_IOC_WR_MAX_SPEED_HZ);
        case SPI_IOC_RST : return FS_CTL_CMD_NAME(SPI_IOC_RST);
        case FORTSENSE_IOC_ATTRIBUTE : return FS_CTL_CMD_NAME(FORTSENSE_IOC_ATTRIBUTE);
        default                        : return "Unknown";
    }
}
////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.

static long fsfp_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    fsfp_key_event_t kevent;
    xprintk(SF_LOG_LEVEL, "%s(...%s...)\n", __FUNCTION__, fs_ctl_cmd_names(cmd));
    if(cmd!=SF_IOC_INIT_DRIVER && !pointjudge){
		return -1;
	}
    switch (cmd) {
        case SF_IOC_INIT_DRIVER: {
			if(!fsfp_ctl_dev.pdev){
	    err = spi_register_driver(&fsfp_driver);
    	    xprintk(KERN_ERR, "%s, spi_register_driver end,error=%d", __FUNCTION__,err);
    	    if (err < 0) {
        	xprintk(KERN_ERR, "%s, Failed to register SPI driver.\n", __FUNCTION__);
			break;
    	    }
//begin by xiakang.chen for task LEVIN-7329 on 20221009
			if(fsfp_ctl_dev.gpio_init==NULL){
				xprintk(KERN_ERR, "%s, fsfp_ctl_dev.gpio_init==NULL\n", __FUNCTION__);
				return -1;
			}
//end by xiakang.chen for task LEVIN-7329 on 20221009
            fsfp_ctl_dev.gpio_init(&fsfp_ctl_dev);
			fsfp_ctl_init_irq();}
			pointjudge=true;
            break;
        }

        case SF_IOC_DEINIT_DRIVER: {
            //fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
			 if(fsfp_ctl_dev.power_on){
			 fsfp_ctl_dev.power_on(false);
			 spi_unregister_driver(&fsfp_driver);
			 pointjudge=false;
			 }
            break;
        }

        case SPI_IOC_RST:
        case SF_IOC_RESET_DEVICE: {
			if(fsfp_ctl_dev.reset){
            fsfp_ctl_dev.reset(false);
            msleep(1);
            fsfp_ctl_dev.reset(true);
            msleep(10);
			}
            break;
        }

        case SF_IOC_ENABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_DISABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_REQUEST_IRQ: {
            //fsfp_ctl_init_irq();
            break;
        }

        case SF_IOC_ENABLE_SPI_CLK: {
			if(fsfp_ctl_dev.spi_clk_on)
            fsfp_ctl_dev.spi_clk_on(true);
            break;
        }

        case SF_IOC_DISABLE_SPI_CLK: {
			if(fsfp_ctl_dev.spi_clk_on)
            fsfp_ctl_dev.spi_clk_on(false);
            break;
        }

        case SF_IOC_ENABLE_POWER: {
			if(fsfp_ctl_dev.power_on)
            fsfp_ctl_dev.power_on(true);
            break;
        }

        case SF_IOC_DISABLE_POWER: {
			if(fsfp_ctl_dev.power_on)
            fsfp_ctl_dev.power_on(false);
            break;
        }

        case SF_IOC_REPORT_KEY_EVENT: {
            if (copy_from_user(&kevent, (fsfp_key_event_t *)arg, sizeof(fsfp_key_event_t))) {
                xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            err = fsfp_ctl_report_key_event(fsfp_ctl_dev.input, &kevent);
            break;
        }

        case SF_IOC_SYNC_CONFIG: {
            // TODO:
            break;
        }

        case SPI_IOC_WR_MAX_SPEED_HZ:
        case SF_IOC_SPI_SPEED: {

            fsfp_ctl_spi_speed(arg);

            break;
        }

        case SPI_IOC_RD_MAX_SPEED_HZ: {
            // TODO:
            break;
        }

        case FORTSENSE_IOC_ATTRIBUTE:
        case SF_IOC_ATTRIBUTE: {
            err = __put_user(fsfp_ctl_dev.attribute, (__u32 __user *)arg);
            break;
        }

        case SF_IOC_GET_VERSION: {
            if (copy_to_user((void *)arg, fsfp_ctl_get_version(), SF_DRV_VERSION_LEN)) {
                xprintk(KERN_ERR, "copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        case SF_IOC_SET_LIB_VERSION: {
            if (copy_from_user((void *)&fsfp_hw_ver, (void *)arg, sizeof(fsfp_version_info_t))) {
                xprintk(KERN_ERR, "fsfp_hw_info_t copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        case SF_IOC_GET_LIB_VERSION: {
            if (copy_to_user((void *)arg, (void *)&fsfp_hw_ver, sizeof(fsfp_version_info_t))) {
                xprintk(KERN_ERR, "fsfp_hw_info_t copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        case SF_IOC_SET_SPI_BUF_SIZE: {
            fsfp_ctl_dev.spi_buf_size = arg;
            break;
        }

        case SF_IOC_SET_RESET_OUTPUT: {
			if(fsfp_ctl_dev.reset){
            if (arg) {
                fsfp_ctl_dev.reset(true);
            }
            else {
                fsfp_ctl_dev.reset(false);
            }
			}
            break;
        }

        default:
            err = (-EINVAL);
            break;
    }

    return err;
}

static ssize_t fortsense_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = 0;
    len += sprintf(buf, "%s\n", fsfp_hw_ver.driver);
    return len;
}
static DEVICE_ATTR(version, S_IRUGO | S_IWUSR, fortsense_version_show, NULL);

static ssize_t fortsense_chip_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = 0;
    len += sprintf((char *)buf, "chip   : %s %s\nid     : 0x0 lib:%s\nvendor : fw:%s\nmore   : fingerprint\n",
                   fsfp_hw_ver.fortsense_id, fsfp_hw_ver.ca_version,
                   fsfp_hw_ver.algorithm,
                   fsfp_hw_ver.firmware);
    return len;
}
static DEVICE_ATTR(chip_info, S_IRUGO | S_IWUSR, fortsense_chip_info_show, NULL);

static ssize_t fsfp_show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    ret += sprintf(buf + ret, "solution:%s\n", fsfp_hw_ver.tee_solution);
    ret += sprintf(buf + ret, "ca      :%s\n", fsfp_hw_ver.ca_version);
    ret += sprintf(buf + ret, "ta      :%s\n", fsfp_hw_ver.ta_version);
    ret += sprintf(buf + ret, "alg     :%s\n", fsfp_hw_ver.algorithm);
    ret += sprintf(buf + ret, "nav     :%s\n", fsfp_hw_ver.algo_nav);
    ret += sprintf(buf + ret, "driver  :%s\n", fsfp_hw_ver.driver);
    ret += sprintf(buf + ret, "firmware:%s\n", fsfp_hw_ver.firmware);
    ret += sprintf(buf + ret, "sensor  :%s\n", fsfp_hw_ver.fortsense_id);
    ret += sprintf(buf + ret, "vendor  :%s\n", fsfp_hw_ver.vendor_id);
    return ret;
}

static DEVICE_ATTR(tee_version, S_IWUSR | S_IRUGO, fsfp_show_version, NULL);

static ssize_t
fsfp_store_set_fun(struct device *d, struct device_attribute *attr,
                   const char *buf, size_t count)
{
    int blank;
    int ret;
    ret = sscanf(buf, "%d", &blank);
    xprintk(KERN_INFO, "fsfp_store_set_fun (..) blank = %d.\n", blank);
    fb_blank(NULL, blank);
    xprintk(KERN_INFO, "fsfp_store_set_fun (..) end.\n");
    return count;
}
static DEVICE_ATTR(set_fun, S_IWUSR | S_IRUGO, NULL, fsfp_store_set_fun);

static struct attribute *fsfp_sysfs_entries[] = {
    &dev_attr_set_fun.attr,
    &dev_attr_tee_version.attr,
    &dev_attr_chip_info.attr,
    &dev_attr_version.attr,
    NULL
};

static struct attribute_group fsfp_attribute_group = {
    .attrs = fsfp_sysfs_entries,
};

#ifdef CONFIG_COMPAT
static long fsfp_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fsfp_ctl_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int fsfp_suspend(void)
{
    char *screen[2] = { "SCREEN_STATUS=OFF", NULL };
    kobject_uevent_env(&fsfp_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen);
    return 0;
}

static int fsfp_resume(void)
{
    char *screen[2] = { "SCREEN_STATUS=ON", NULL };
    kobject_uevent_env(&fsfp_ctl_dev.miscdev.this_device->kobj, KOBJ_CHANGE, screen);
    return 0;
}



static int fsfp_fb_notifier_callback(struct notifier_block *self,
                                     unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    unsigned int blank;
    int retval = 0;

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    blank = *(int *)evdata->data;

    switch (blank) {
        case FB_BLANK_UNBLANK:
            fsfp_resume();
            break;

        case FB_BLANK_POWERDOWN:
            fsfp_suspend();
            break;

        default:
            break;
    }

    return retval;
}

////////////////////////////////////////////////////////////////////////////////
static int fsfp_remove(fsfp_device_t *spi)
{
    int err = 0;
    xprintk(KERN_INFO, "fortsense %s enter\n", __FUNCTION__);
    if (fsfp_ctl_dev.pdev != NULL && fsfp_ctl_dev.gpio_init != NULL) {
		xprintk(KERN_INFO, "fortsense %s enter1\n", __FUNCTION__);
        fb_unregister_client(&fsfp_ctl_dev.notifier);
        if (fsfp_ctl_dev.input) {
            input_unregister_device(fsfp_ctl_dev.input);
        }

        if (fsfp_ctl_dev.irq_num >= 0) {
			xprintk(KERN_INFO, "fortsense %s enter2\n", __FUNCTION__);
            free_irq(fsfp_ctl_dev.irq_num, (void*)&fsfp_ctl_dev);
            fsfp_ctl_dev.irq_num = 0;
        }

/*        misc_deregister(&fsfp_ctl_dev.miscdev);
#ifdef CONFIG_PM_WAKELOCKS
        wakeup_source_trash(&fsfp_ctl_dev.wakelock);
#else
        wake_lock_destroy(&fsfp_ctl_dev.wakelock);
#endif
*/
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        fsfp_ctl_dev.pdev = NULL;
    }

    return err;
}

static int fsfp_probe(fsfp_device_t *dev)
{
    int err = 0;
    xprintk(KERN_INFO, "fortsense %s enter\n", __FUNCTION__);
    fsfp_ctl_dev.pdev = dev;
    /* setup spi config */

    /*fsfp_ctl_dev.pdev->mode            = SPI_MODE_0;
    fsfp_ctl_dev.pdev->bits_per_word   = 8;
    fsfp_ctl_dev.pdev->max_speed_hz    = SF_DEFAULT_SPI_SPEED;
    spi_setup(fsfp_ctl_dev.pdev);*/
	
    /* Initialize the platform config. */
    err = fsfp_platform_init(&fsfp_ctl_dev);

    if (err) {
        fsfp_ctl_dev.pdev = NULL;
        xprintk(KERN_ERR, "fsfp_platform_init failed with %d.\n", err);
        return err;
    }
/*
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_init(&fsfp_ctl_dev.wakelock , "fsfp_wakelock");
#else
    wake_lock_init(&fsfp_ctl_dev.wakelock, WAKE_LOCK_SUSPEND, "fsfp_wakelock");
#endif
*/
    /* Initialize the input subsystem. */
    err = fsfp_ctl_init_input();

    if (err) {
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        fsfp_ctl_dev.pdev = NULL;
        xprintk(KERN_ERR, "fsfp_ctl_init_input failed with %d.\n", err);
        return err;
    }

    /* Register as a miscellaneous device. */
/*
    err = misc_register(&fsfp_ctl_dev.miscdev);

    if (err) {
        fsfp_ctl_dev.free_gpio(&fsfp_ctl_dev);
        fsfp_platform_exit(&fsfp_ctl_dev);
        xprintk(KERN_ERR, "misc_register(..) = %d.\n", err);
        input_unregister_device(fsfp_ctl_dev.input);
        fsfp_ctl_dev.pdev = NULL;
        return err;
    }

    err = sysfs_create_group(&fsfp_ctl_dev.miscdev.this_device->kobj, &fsfp_attribute_group);
*/
    /* Initialize the interrupt callback. */
    //INIT_WORK(&fsfp_ctl_dev.work_queue, fsfp_ctl_device_event);


    fsfp_ctl_dev.notifier.notifier_call = fsfp_fb_notifier_callback;
    fb_register_client(&fsfp_ctl_dev.notifier);
	
    xprintk(KERN_ERR, "%s leave\n", __FUNCTION__);
    return err;
}


////////////////////////////////////////////////////////////////////////////////
static int fsfp_ctl_init_irq(void)
{
    int err = 0;
    unsigned long flags = IRQF_TRIGGER_FALLING; // IRQF_TRIGGER_FALLING or IRQF_TRIGGER_RISING
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    /* Register interrupt callback. */
    err = request_irq(fsfp_ctl_dev.irq_num, fsfp_ctl_device_irq,
                      flags, "sf-irq", (void*)&fsfp_ctl_dev);

    if (err) {
        xprintk(KERN_ERR, "request_irq(..) = %d.\n", err);
    }

    enable_irq_wake(fsfp_ctl_dev.irq_num);
    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int fsfp_ctl_init_input(void)
{
    int err = 0;
    xprintk(SF_LOG_LEVEL, "%s(..) enter.\n", __FUNCTION__);
    fsfp_ctl_dev.input = input_allocate_device();

    if (!fsfp_ctl_dev.input) {
        xprintk(KERN_ERR, "input_allocate_device(..) failed.\n");
        return (-ENOMEM);
    }

    fsfp_ctl_dev.input->name = "sf-keys";
    __set_bit(EV_KEY,     fsfp_ctl_dev.input->evbit );
    __set_bit(KEY_HOME,   fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_MENU,   fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_BACK,   fsfp_ctl_dev.input->keybit);
    __set_bit(0xFF,    fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_ENTER,  fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_UP,     fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_LEFT,   fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_RIGHT,  fsfp_ctl_dev.input->keybit);
    __set_bit(KEY_DOWN,   fsfp_ctl_dev.input->keybit);
    __set_bit(0xFD, fsfp_ctl_dev.input->keybit);
    err = input_register_device(fsfp_ctl_dev.input);

    if (err) {
        xprintk(KERN_ERR, "input_register_device(..) = %d.\n", err);
        input_free_device(fsfp_ctl_dev.input);
        fsfp_ctl_dev.input = NULL;
        return (-ENODEV);
    }

    xprintk(SF_LOG_LEVEL, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int __init fsfp_ctl_driver_init(void)
{
    int err = 0;
    xprintk(KERN_INFO, "'%s' SW_BUS_NAME = %s\n", __FUNCTION__, SW_BUS_NAME);
    /**register SPI device、driver***/
    //spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
    /*
    err = spi_register_driver(&fsfp_driver);
    xprintk(KERN_ERR, "%s, spi_register_driver end,error=%d", __FUNCTION__,err);
    if (err < 0) {
        xprintk(KERN_ERR, "%s, Failed to register SPI driver.\n", __FUNCTION__);
    }
*/
    err = misc_register(&fsfp_ctl_dev.miscdev);

    if (err) {
        xprintk(KERN_ERR, "misc_register(..) = %d.", err);
        return err;
    }
	err = sysfs_create_group(&fsfp_ctl_dev.miscdev.this_device->kobj, &fsfp_attribute_group);
    /* Initialize the interrupt callback. */
    INIT_WORK(&fsfp_ctl_dev.work_queue, fsfp_ctl_device_event);
	#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_init(&fsfp_ctl_dev.wakelock , "fsfp_wakelock");
#else
    wake_lock_init(&fsfp_ctl_dev.wakelock, WAKE_LOCK_SUSPEND, "fsfp_wakelock");
#endif
    xprintk(KERN_INFO, "fortsense fingerprint device control driver registered.\n");
    xprintk(KERN_INFO, "driver version: '%s'.\n", fsfp_ctl_get_version());
    return err;
}

static void __exit fsfp_ctl_driver_exit(void)
{
    //spi_unregister_driver(&fsfp_driver);
#ifdef CONFIG_PM_WAKELOCKS
        wakeup_source_trash(&fsfp_ctl_dev.wakelock);
#else
        wake_lock_destroy(&fsfp_ctl_dev.wakelock);
#endif

    misc_deregister(&fsfp_ctl_dev.miscdev);
    xprintk(KERN_INFO, "fortsense fingerprint device control driver released.\n");
}

module_init(fsfp_ctl_driver_init);
module_exit(fsfp_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for Fortsense's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Fortsense");


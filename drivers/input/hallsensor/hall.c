#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include "hall.h"
#include <linux/interrupt.h>
#include <linux/slab.h>
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/of_irq.h>

#ifdef CONFIG_TCT_PROJECT_CHALLENGER_STYLU
#include <linux/input/touch_notify.h>
#endif

static int prob_flag=0;

static struct of_device_id hall_of_match[] = {
    {.compatible = "mediatek,hall_switch",},
    {},
};

#ifdef CONFIG_PM_WAKELOCKS
static struct wakeup_source hall_wakelock;
#else
static struct wake_lock hall_wakelock;
#endif
//Begin Modified by yan.gong for FR 10682525 on 2021-01-27
#define KEY_HALL_SENSOR_DOWN 587
#define KEY_HALL_SENSOR_UP 586
//End Modified by yan.gong for FR 10682525 on 2021-01-27
struct input_dev *hall_input = NULL;
struct hall_data *hall = NULL;
static struct class *hall_class = NULL;
static struct device *hall_dev = NULL;

static int hall_status = -1;

static void do_hall_work(struct work_struct *work)
{
    unsigned int gpio_status;
#ifdef CONFIG_PM_WAKELOCKS
    __pm_wakeup_event(&hall_wakelock, jiffies_to_msecs(HZ/2));
#else
    wake_lock_timeout(&hall_wakelock, HZ/2);
#endif
    gpio_status = gpio_get_value(hall->irq_gpio);
#ifdef CONFIG_TCT_PROJECT_CHALLENGER_STYLU
	touch_atomic_notifier_call(NOTIFY_PEN_STATUS,(void *)&gpio_status);
#endif
#if 1
//Begin Modified by yan.gong for FR 10682525 on 2021-01-27
    if(!gpio_status)
    {
        input_report_key(hall_input, KEY_HALL_SENSOR_DOWN, 1);
        input_report_key(hall_input, KEY_HALL_SENSOR_DOWN, 0);
        printk("%s,%d,keycode = %d,report key\n",__func__,__LINE__,KEY_HALL_SENSOR_DOWN);
        input_sync(hall_input);
    }
    else
    {
        input_report_key(hall_input, KEY_HALL_SENSOR_UP, 1);
        input_report_key(hall_input, KEY_HALL_SENSOR_UP, 0);
        printk("%s,%d,keycode = %d,report key\n",__func__,__LINE__,KEY_HALL_SENSOR_UP);
        input_sync(hall_input);
    }
#endif
//End Modified by yan.gong for FR 10682525 on 2021-01-27
    enable_irq(hall->irq);
}

static irqreturn_t interrupt_hall_irq(int irq, void *dev)
{
        if (prob_flag==0)
         return IRQ_HANDLED;
    disable_irq_nosync(hall->irq);
    //printk("%s,%d,irq_value = %d\n",__func__,__LINE__, gpio_get_value(hall_eint_gpio));
        queue_work(hall->hall_wq, &hall->hall_work);
    return IRQ_HANDLED;
}

static ssize_t hall_status_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    hall_status = gpio_get_value(hall->irq_gpio);
    sprintf(buf,"%d\n", hall_status);
    return strlen(buf);
}

static DEVICE_ATTR(hall_status, 0444, hall_status_show, NULL);

static int hall_probe(struct platform_device *pdev)
{

    int rc = 0;
    struct device_node *node = pdev->dev.of_node;
    hall = kzalloc( sizeof(struct hall_data), GFP_KERNEL);

    if (hall == NULL)
    {
        printk(KERN_INFO"%s:%d Unable to allocate memory\n", __func__, __LINE__);
        return -ENOMEM;
    }
    //node = of_find_compatible_node(NULL, NULL, hall_of_match);
    if (node)
    {
        hall->irq_gpio = of_get_named_gpio(node, "hall-gpio", 0);
        hall->irq = irq_of_parse_and_map(node, 0);

        rc = request_irq(hall->irq , interrupt_hall_irq, IRQ_TYPE_EDGE_BOTH, "hall-eint", NULL);
        if (rc)
        {
            rc = -1;
            printk("%s : requesting IRQ error\n", __func__);
            return rc;
        }
        else
        {
            printk("%s : requesting IRQ %d\n", __func__, hall->irq);
        }

    } else {
        printk("%s : can not find hall eint compatible node\n",  __func__);
        return -1;
    }
    hall->pdev = pdev;
    dev_set_drvdata(&pdev->dev, hall);

    hall_input = input_allocate_device();
    if (!hall_input)
    {
        printk("hall.c: Not enough memory\n");
        return -ENOMEM;
    }

    hall_input->name = "hall_switch";
    //Begin Modified by yan.gong for FR 10682525 on 2021-01-27
    input_set_capability(hall_input, EV_KEY, KEY_HALL_SENSOR_DOWN);
    input_set_capability(hall_input, EV_KEY, KEY_HALL_SENSOR_UP);
    //End Modified by yan.gong for FR 10682525 on 2021-01-27

    rc = input_register_device(hall_input);
    if (rc)
    {
        printk("hall.c: Failed to register device\n");
        return rc;
    }

    hall->hall_wq = create_singlethread_workqueue("hall_wq");
    if (!hall->hall_wq) {
          printk(KERN_CRIT"%s: create thread error!\n", __func__);
    }

    INIT_WORK(&hall->hall_work, do_hall_work);
    enable_irq_wake(hall->irq);

    hall_class= class_create(THIS_MODULE, "hall_switch");
    hall_dev = device_create(hall_class, NULL, 0, NULL, "hall_switch");
    if (IS_ERR(hall_dev))
            printk( "Failed to create device(hall_dev)!\n");

    if (device_create_file(hall_dev, &dev_attr_hall_status) < 0)
            printk( "Failed to create device(hall_dev)'s node hall_status!\n");
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_init(&hall_wakelock, "hall_wakelock");
#else
    wake_lock_init(&hall_wakelock,WAKE_LOCK_SUSPEND, "hall_wakelock");
#endif

    printk("hall probe completed\n");
    prob_flag=1;
    return 0;
}

int hall_remove(struct platform_device *pdev)
{
    hall = platform_get_drvdata(pdev);

    free_irq(hall->irq, pdev);
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_trash(&hall_wakelock);
#else
    wake_lock_destroy(&hall_wakelock);
#endif

    input_unregister_device(hall_input);
    if (hall_input)
    {
        input_free_device(hall_input);
        hall_input = NULL;
    }

    return 0;
}

static struct platform_driver hall_driver = {
    .probe = hall_probe,
    .remove = hall_remove,
    .driver = {
           .name = "hall_switch",
           .owner = THIS_MODULE,
           .of_match_table = hall_of_match,
    }
};

static int __init hall_init(void)
{
    return platform_driver_register(&hall_driver);
}

static void __init hall_exit(void)
{
    platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_LICENSE("GPL");


#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <net/sock.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>


#include <linux/regulator/consumer.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/workqueue.h>

#include <linux/irq.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif

#include <linux/platform_device.h>

#include "fpsensor_spi_tee.h"
extern void mt_spi_disable_master_clk(struct spi_device *ms);
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
static int fpsensor_probe(struct spi_device *spi);
static int fpsensor_remove(struct spi_device *spi);
struct spi_device *g_fpsensor_spidev = NULL;
#define FPSENSOR_SPI_VERSION              "fpsensor_spi_tee_mtk_v1.23.8"
uint32_t pinctrl_power = -1;
const char *chipone_info;
#ifdef CONFIG_TCT_DEVICEINFO
extern char fp_module_name[256];
#endif
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifndef CONFIG_FINGERPRINT_REGULATOR_MT6371
struct regulator *fp_reg = NULL;
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
/* global variables */
static fpsensor_data_t *g_fpsensor = NULL;
uint32_t g_cmd_sn = 0;
#ifdef CONFIG_OF
static struct of_device_id fpsensor_of_match[] = {
    { .compatible = "mediatek,fingerprint-spi-dev", },
    {}
};
MODULE_DEVICE_TABLE(of, fpsensor_of_match);
#endif

static struct spi_driver fpsensor_spi_driver = {
    .driver = {
        .name = FPSENSOR_DEV_NAME,
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(fpsensor_of_match),
#endif
    },
    .probe = fpsensor_probe,
    .remove = fpsensor_remove,
};

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration                              */
/* -------------------------------------------------------------------- */
static DEFINE_MUTEX(spidev_set_gpio_mutex);
static int fpsensor_irq_gpio_cfg(fpsensor_data_t *fpsensor)
{
    struct device_node *node;
    u32 ints[2] = {0, 0};
    fpsensor_debug(DEBUG_LOG,"%s\n", __func__);

    node = of_find_compatible_node(NULL, NULL, "mediatek,fpsensor-eint");
    if ( node) {
        of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
        fpsensor_debug(DEBUG_LOG,"[fpsensor]ints[0] = %d,is irq_gpio , ints[1] = %d!!\n", ints[0], ints[1]);
        fpsensor->irq_gpio = ints[0];
        fpsensor->irq = irq_of_parse_and_map(node, 0);  // get irq number
        if (!fpsensor->irq) {
            fpsensor_debug(ERR_LOG,"fpsensor irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        fpsensor_debug(DEBUG_LOG," [fpsensor]fpsensor->irq= %d,fpsensor>irq_gpio = %d\n", fpsensor->irq,
                fpsensor->irq_gpio);
    } else {
        fpsensor_debug(ERR_LOG,"fpsensor null irq node!!\n");
        return -EINVAL;
    }

    return 0 ;
}
// null
int fpsensor_gpio_wirte(int gpio, int value)
{
    mutex_lock(&spidev_set_gpio_mutex);
    if (g_fpsensor->pinctrl1) {
    if (gpio == FPSENSOR_RST_PIN) {
        if (value) {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_high);
        } else {
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_low);
        }
    }
    }
    mutex_unlock(&spidev_set_gpio_mutex);
    return 0;
}

int fpsensor_spidev_dts_init(fpsensor_data_t *fpsensor)
{
    struct device_node *node = NULL;
    struct device_node *vdd_node = NULL;
    struct platform_device *pdev = NULL;
    struct platform_device *p_vdd_dev = NULL;
    int ret = 0;
    fpsensor_debug(DEBUG_LOG, "%s\n", __func__);

    node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint-pinctrl");
    if (node) {
        pdev = of_find_device_by_node(node);
        if(pdev) {
            fpsensor->pinctrl1 = devm_pinctrl_get(&pdev->dev);
            if (IS_ERR(fpsensor->pinctrl1)) {
                ret = PTR_ERR(fpsensor->pinctrl1);
                fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl1, ret=%d.\n", ret);
                return ret;
            }
        } else {
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find device.\n");
            return -ENODEV;
        }
        fpsensor->pwr_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_power_low");
        if (IS_ERR(fpsensor->pwr_low)) {
            ret = PTR_ERR(fpsensor->pwr_low);
            fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl fpsensor_power_low!\n");
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifndef CONFIG_FINGERPRINT_REGULATOR_MT6371	
            return ret;
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
        }
        fpsensor->pwr_high = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_power_high");
        if (IS_ERR(fpsensor->pwr_high)) {
            ret = PTR_ERR(fpsensor->pwr_high);
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl fpsensor_power_high!\n");
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifndef CONFIG_FINGERPRINT_REGULATOR_MT6371
            return ret;
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
        }
        fpsensor->eint_as_int = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_eint_low");
        if (IS_ERR(fpsensor->eint_as_int)) {
            ret = PTR_ERR(fpsensor->eint_as_int);
            fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl fpsensor_eint_low!\n");
            return ret;
        }
        fpsensor->fp_rst_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_rst_low");
        if (IS_ERR(fpsensor->fp_rst_low)) {
            ret = PTR_ERR(fpsensor->fp_rst_low);
            fpsensor_debug(ERR_LOG,"fpsensor Cannot find fp pinctrl fpsensor_rst_low!\n");
            return ret;
        }
        fpsensor->fp_rst_high = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_rst_high");
        if (IS_ERR(fpsensor->fp_rst_high)) {
            ret = PTR_ERR(fpsensor->fp_rst_high);
            fpsensor_debug(ERR_LOG, "fpsensor Cannot find fp pinctrl fpsensor_rst_high!\n");
            return ret;
        }
	/*	
	ret = pinctrl_select_state(fpsensor->pinctrl1, fpsensor->pwr_high);
	if (ret) {
        	fpsensor_debug(ERR_LOG, "%s regulator enable failed(%d)\n",__func__, ret);
		return -ENODEV;
        }
	*/
	ret = pinctrl_select_state(fpsensor->pinctrl1, fpsensor->eint_as_int);
	if (ret) {
        	fpsensor_debug(ERR_LOG, "%s set irq state failed(%d)\n",__func__, ret);
		return -ENODEV;
	}
    } else {
        fpsensor_debug(ERR_LOG,"fpsensor Cannot find node!\n");
        return -ENODEV;
    }
    /* Get regulator */
    vdd_node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint-power");
    if (!vdd_node) {
        printk("%s:vdd_node is null",__func__);
    }
    ret = of_property_read_u32(vdd_node, "pinctrl_power", &pinctrl_power);
    fpsensor_debug(ERR_LOG,"get pinctrl_power = %d.\n",pinctrl_power);
    if (ret) {
	fpsensor_debug(ERR_LOG,"read pinctrl_power fail.\n");
	return ret;
    }
    ret = of_property_read_string(vdd_node, "chipone_info", &chipone_info);
    fpsensor_debug(ERR_LOG,"get chipone_info = %s.\n",chipone_info);
    if (ret) {
	fpsensor_debug(ERR_LOG,"read chipone_info fail.\n");
	return ret;
    }
    /* Convert to platform device */
    p_vdd_dev = of_find_device_by_node(vdd_node);
    if (!p_vdd_dev) {
        printk("%s:vdd_node is null",__func__);
    }
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifndef CONFIG_FINGERPRINT_REGULATOR_MT6371
    if (pinctrl_power == 0)
    {
    	fpsensor_debug(DEBUG_LOG, "%s, get regulator from dts\n", __func__);

    	fp_reg = regulator_get(&p_vdd_dev->dev, "vfp");

    	if (IS_ERR(fp_reg)) {
        	fpsensor_debug(ERR_LOG, "%s get regulator failed\n", __func__);
        	return IS_ERR(fp_reg);
    	}

    	ret = regulator_set_voltage(fp_reg, 2800000, 2800000);
    	if (ret) {
        	fpsensor_debug(ERR_LOG, "%s regulator_set_voltage(%d)\n",__func__, ret);
        	return ret;
    		}
    }
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15

    return ret;
}
/* delay us after reset */
static void fpsensor_hw_reset(int delay)
{
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,    1);
    udelay(100);
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  0);
    udelay(1000);
    fpsensor_gpio_wirte(FPSENSOR_RST_PIN,  1);
    if (delay) {
        /* delay is configurable */
#ifdef CONFIG_TCT_PROJECT_MODEL_3
        mdelay(delay);
#else		
        udelay(delay);
#endif
    }
    return;
}
void fpsensor_spi_clk_enable(u8 bonoff)
{
    static int mt_spi_clk_flag = 0;   // 0 mean clock is diasble
    fpsensor_debug(DEBUG_LOG,"%s:enter,now set spiclk as %d.\n",__func__, bonoff);
    if(mt_spi_clk_flag == bonoff) {
        return;
    }
    if (g_fpsensor_spidev){
    mutex_lock(&spidev_set_gpio_mutex);
    if (bonoff == 0) {
        udelay(1000);
        mt_spi_clk_flag = 0;
        mt_spi_disable_master_clk(g_fpsensor_spidev);
    } else {
        mt_spi_clk_flag = 1;
        mt_spi_enable_master_clk(g_fpsensor_spidev);
    }
    mutex_unlock(&spidev_set_gpio_mutex);
    }
    else
	fpsensor_debug(ERR_LOG, "%s: g_fpsensor_spidev is NULL pointer.\n",__func__);
}
EXPORT_SYMBOL_GPL(fpsensor_spi_clk_enable);

static void setRcvIRQ(int val)
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    fpsensor_dev->RcvIRQ = val;
}

static void fpsensor_enable_irq(fpsensor_data_t *fpsensor_dev)
{
    setRcvIRQ(0);
    /* Request that the interrupt should be wakeable */
    if (fpsensor_dev->irq_enabled == 0) {
        if (fpsensor_dev->irq) {
        	enable_irq(fpsensor_dev->irq);
		fpsensor_debug(DEBUG_LOG, "%s enable interrupt!\n", __func__);
        	fpsensor_dev->irq_enabled = 1;
	}
    }
    return;
}

static void fpsensor_disable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();

    if (0 == fpsensor_dev->device_available) {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
        goto out;
    }

    if (0 == fpsensor_dev->irq_enabled) {
        fpsensor_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
        goto out;
    }

    if (fpsensor_dev->irq) {
        disable_irq_nosync(fpsensor_dev->irq);
        fpsensor_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
    }
    fpsensor_dev->irq_enabled = 0;

out:
    setRcvIRQ(0);
    FUNC_EXIT();
    return;
}

static irqreturn_t fpsensor_irq(int irq, void *handle)
{
    fpsensor_data_t *fpsensor_dev = (fpsensor_data_t *)handle;

    /* Make sure 'wakeup_enabled' is updated before using it
     ** since this is interrupt context (other thread...) */
    smp_rmb();

#ifdef CONFIG_TCT_PROJECT_MODEL_3
#ifdef CONFIG_PM_WAKELOCKS
    __pm_wakeup_event(&fpsensor_dev->ttw_wl, jiffies_to_msecs(1*HZ));
#else
    wake_lock_timeout(&fpsensor_dev->ttw_wl, msecs_to_jiffies(1000)); // 2 seconds.
#endif
#endif

    setRcvIRQ(1);
    wake_up_interruptible(&fpsensor_dev->wq_irq_return);

    return IRQ_HANDLED;
}

// release and cleanup fpsensor char device
static void fpsensor_dev_cleanup(fpsensor_data_t *fpsensor)
{
    FUNC_ENTRY();
//Begin added by liangjiaqiang for MODEL3-4506 2022-11-14
#ifdef CONFIG_TCT_PROJECT_MODEL_3
	cdev_del(&fpsensor->cdev);
	unregister_chrdev_region(fpsensor->devno, FPSENSOR_NR_DEVS);
	device_destroy(fpsensor->class, fpsensor->devno);
	class_destroy(fpsensor->class);

	/* Turn off the spi clk. */
	fpsensor_spi_clk_enable(0);
	fpsensor_debug(INFO_LOG, "%s: fpsensor remove(disable spi clk) finished======\n", __func__);

	/* Turn off the fingerprint irq. */
	fpsensor_disable_irq(fpsensor);
	fpsensor_debug(INFO_LOG, "%s: fpsensor remove(disable irq) finished======\n", __func__);

	if (fpsensor->irq)
	free_irq(fpsensor->irq, fpsensor);
	fpsensor_debug(INFO_LOG, "%s: fpsensor remove(free irq) finished======\n", __func__);
	if (g_fpsensor_spidev){
		spi_unregister_driver(&fpsensor_spi_driver);
		g_fpsensor_spidev = NULL;
		fpsensor_debug(INFO_LOG, "%s: fpsensor remove(unregister spi driver) finished======\n", __func__);
	}
    #if FP_NOTIFY
	fb_unregister_client(&fpsensor->notifier);
	fpsensor_debug(INFO_LOG, "%s: fpsensor remove(unregist callback client) finished======\n", __func__);
    #endif
	fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
#endif
//End added by liangjiaqiang for MODEL3-4506 2022-11-14
    	//cdev_del(&fpsensor->cdev);
    	//unregister_chrdev_region(fpsensor->devno, FPSENSOR_NR_DEVS);
    	//device_destroy(fpsensor->class, fpsensor->devno);
    	//class_destroy(fpsensor->class);
    FUNC_EXIT();
}

//Begin added by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifdef CONFIG_FINGERPRINT_REGULATOR_MT6371
extern int finger_power_enable(void);
extern int finger_power_disable(void);
#endif
//End added by liangjiaqiang for MODEL3-1890 on 2022-09-15

static int fpsensor_enable_power(void)
{
//Begin added by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifdef CONFIG_FINGERPRINT_REGULATOR_MT6371
    finger_power_enable();
    return 0;
//End added by liangjiaqiang for MODEL3-1890 on 2022-09-15	
#else	
    int ret = 0;
    
    if (pinctrl_power == 0) {
	if (fp_reg){
    		ret = regulator_enable(fp_reg);
    		if (ret) {
        		fpsensor_debug(ERR_LOG, "%s regulator enable failed(%d)\n",__func__, ret);
        		regulator_put(fp_reg);
    		}
	}
	else
		fpsensor_debug(ERR_LOG, "%s regulator pointer is NULL!",__func__);
    }
    else {
	if (g_fpsensor->pinctrl1){
		ret = pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->pwr_high);
		fpsensor_debug(DEBUG_LOG, "%s:success to disable power.\n",__func__);
    	}
    	else
		fpsensor_debug(ERR_LOG, "%s:pinctrl handle is NULL.\n",__func__);
    	}
    return ret;
#endif
}

static int fpsensor_disable_power(void)
{
//Begin added by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifdef CONFIG_FINGERPRINT_REGULATOR_MT6371
    finger_power_disable();
    return 0;
//End added by liangjiaqiang for MODEL3-1890 on 2022-09-15	
#else
    int ret = 0;
    fpsensor_debug(INFO_LOG,"%s:enter",__func__);
    if (pinctrl_power == 1){
    	if (g_fpsensor->pinctrl1){
		ret = pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->pwr_low);
		fpsensor_debug(DEBUG_LOG, "%s:success to disable power.\n",__func__);
    	}
    	else
		fpsensor_debug(ERR_LOG, "%s:pinctrl handle is NULL.\n",__func__);
    }
    else {
    	if (fp_reg) {
     		ret = regulator_disable(fp_reg);
     		if (ret) {
         		fpsensor_debug(ERR_LOG, "%s regulator disable failed(%d)\n",__func__, ret);
         		regulator_put(fp_reg);
     		}
        	else
        		fpsensor_debug(ERR_LOG, "%s regulator disable success\n",__func__);
    	}
    	else
		fpsensor_debug(ERR_LOG, "%s regulator pointer is NULL!",__func__);
     }
    return ret;
#endif
}

#if FP_NOTIFY
static int fpsensor_fb_notifier_callback(struct notifier_block* self, unsigned long event, void* data);
#endif
static long fpsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    fpsensor_data_t *fpsensor_dev = NULL;
    int retval = 0;
    uint32_t val = 0;
    int irqf;

    fpsensor_debug(INFO_LOG, "[rickon]: fpsensor ioctl cmd : 0x%x \n", cmd );
    fpsensor_dev = (fpsensor_data_t *)filp->private_data;
    fpsensor_dev->cancel = 0 ;
    switch (cmd) {
        case FPSENSOR_IOC_INIT:
            fpsensor_debug(INFO_LOG, "%s: fpsensor init started======\n", __func__);
	    /* Regist spi driver to get spi-clk. */
    	    fpsensor_debug(ERR_LOG, "%s, func enter", "spi_register_driver");
    	    retval = spi_register_driver(&fpsensor_spi_driver);
    	    if (retval < 0) {
        	fpsensor_debug(ERR_LOG, "%s, Failed to register TEE driver.\n", __func__);
    	    }

	    init_waitqueue_head(&fpsensor_dev->wq_irq_return);
	    
	    /* Register screen on/off callback. */
	#if FP_NOTIFY
    	    fpsensor_dev->notifier.notifier_call = fpsensor_fb_notifier_callback;
    	    fb_register_client(&fpsensor_dev->notifier);
	#endif 
	    /* Get pinctrl resources. */
            retval = fpsensor_spidev_dts_init(fpsensor_dev);
            if (retval) {
//Begin modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
#ifndef CONFIG_FINGERPRINT_REGULATOR_MT6371
                break;
#endif
//End modified by liangjiaqiang for MODEL3-1890 on 2022-09-15
            }

	    /* init irq resources. */
            if(fpsensor_irq_gpio_cfg(fpsensor_dev) != 0) {
                break;
            }
            irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
            retval = request_threaded_irq(fpsensor_dev->irq, fpsensor_irq, NULL,
                    irqf, FPSENSOR_DEV_NAME, fpsensor_dev);
            if (retval == 0) {
                fpsensor_debug(INFO_LOG, " irq thread reqquest success!\n");
            } else {
                fpsensor_debug(ERR_LOG, " irq thread request failed , retval =%d \n", retval);
                break;
            }

	    /* Wake up the system while receiving the interrupt. */
            enable_irq_wake(g_fpsensor->irq);
            fpsensor_dev->device_available = 1;
            // fix Unbalanced enable for IRQ, disable irq at first
            fpsensor_dev->irq_enabled = 1;
            fpsensor_disable_irq(fpsensor_dev);
            fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);

            break;

        case FPSENSOR_IOC_EXIT:
            fpsensor_debug(INFO_LOG, "%s: fpsensor exit finished======\n", __func__);
            break;
        case FPSENSOR_IOC_RESET:
            fpsensor_debug(INFO_LOG, "%s: chip reset command\n", __func__);
#ifdef CONFIG_TCT_PROJECT_MODEL_3
            fpsensor_hw_reset(4);
#else			
            fpsensor_hw_reset(1250);
#endif
            break;

        case FPSENSOR_IOC_ENABLE_IRQ:
            fpsensor_debug(INFO_LOG, "%s: chip ENable IRQ command\n", __func__);
            fpsensor_enable_irq(fpsensor_dev);
            break;

        case FPSENSOR_IOC_DISABLE_IRQ:
            fpsensor_debug(INFO_LOG, "%s: chip disable IRQ command\n", __func__);
            fpsensor_disable_irq(fpsensor_dev);
            break;
        case FPSENSOR_IOC_GET_INT_VAL:
            fpsensor_debug(INFO_LOG, "%s: fpsensor IOC_GET_INT_VAL begin======\n", __func__);
            val = gpio_get_value(fpsensor_dev->irq_gpio);
            if (copy_to_user((void __user *)arg, (void *)&val, sizeof(uint32_t))) {
                fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
                retval = -EFAULT;
                break;
            }
            retval = 0;
            break;
        case FPSENSOR_IOC_ENABLE_SPI_CLK:
            fpsensor_debug(INFO_LOG, "%s: ENABLE_SPI_CLK ======\n", __func__);
            fpsensor_spi_clk_enable(1);
            break;
        case FPSENSOR_IOC_DISABLE_SPI_CLK:
            fpsensor_debug(INFO_LOG, "%s: DISABLE_SPI_CLK ======\n", __func__);
            fpsensor_spi_clk_enable(0);
            break;
        case FPSENSOR_IOC_ENABLE_POWER:
            fpsensor_enable_power();
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_POWER ======\n", __func__);
            break;
        case FPSENSOR_IOC_DISABLE_POWER:
            fpsensor_disable_power();
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_DISABLE_POWER ======\n", __func__);
            break;
        case FPSENSOR_IOC_REMOVE:
            fpsensor_debug(INFO_LOG, "%s: fpsensor remove begin======\n", __func__);
	    /* Turn off the spi clk. */
            fpsensor_spi_clk_enable(0);
            fpsensor_debug(INFO_LOG, "%s: fpsensor remove(disable spi clk) finished======\n", __func__);

	    /* Turn off the fingerprint irq. */
            fpsensor_disable_irq(fpsensor_dev);
            fpsensor_debug(INFO_LOG, "%s: fpsensor remove(disable irq) finished======\n", __func__);

    	    if (fpsensor_dev->irq)
        	free_irq(fpsensor_dev->irq, fpsensor_dev);
            fpsensor_debug(INFO_LOG, "%s: fpsensor remove(free irq) finished======\n", __func__);
	    if (g_fpsensor_spidev){
    		spi_unregister_driver(&fpsensor_spi_driver);
		g_fpsensor_spidev = NULL;
                fpsensor_debug(INFO_LOG, "%s: fpsensor remove(unregister spi driver) finished======\n", __func__);
    	    }
#if FP_NOTIFY
            fb_unregister_client(&fpsensor_dev->notifier);
            fpsensor_debug(INFO_LOG, "%s: fpsensor remove(unregist callback client) finished======\n", __func__);
#endif
            fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
            break;
        case FPSENSOR_IOC_CANCEL_WAIT:
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR CANCEL WAIT\n", __func__);
            wake_up_interruptible(&fpsensor_dev->wq_irq_return);
            fpsensor_dev->cancel = 1;
            break;
#if FP_NOTIFY
        case FPSENSOR_IOC_GET_FP_STATUS :
            val = fpsensor_dev->fb_status;
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_GET_FP_STATUS  %d \n",__func__, fpsensor_dev->fb_status);
            if (copy_to_user((void __user *)arg, (void *)&val, sizeof(uint32_t))) {
                fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
                retval = -EFAULT;
                break;
            }
            retval = 0;
            break;
#endif
        case FPSENSOR_IOC_ENABLE_REPORT_BLANKON:
            if (copy_from_user(&val, (void __user *)arg, sizeof(uint32_t))) {
                retval = -EFAULT;
                break;
            }
            fpsensor_dev->enable_report_blankon = val;
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_REPORT_BLANKON: %d\n", __func__, val);
    #ifdef CONFIG_TCT_DEVICEINFO
	    sprintf(fp_module_name, chipone_info);
    #endif
            break;
        case FPSENSOR_IOC_UPDATE_DRIVER_SN:
            if (copy_from_user(&g_cmd_sn, (void __user *)arg, sizeof(uint32_t))) {
                fpsensor_debug(ERR_LOG, "Failed to copy g_cmd_sn from user to kernel\n");
                retval = -EFAULT;
                break;
            }
            break;
        case FPSENSOR_IOC_SET_HARDWARE_INFO:
            fpsensor_debug(ERR_LOG, "add fpvendor for hwinfo\n");
            break;
        default:
            fpsensor_debug(ERR_LOG, "fpsensor doesn't support this command(0x%x)\n", cmd);
            break;
    }
    return retval;
}

static long fpsensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fpsensor_ioctl(filp, cmd, (unsigned long)(arg));
}

static unsigned int fpsensor_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int ret = 0;

    ret |= POLLIN;
    poll_wait(filp, &g_fpsensor->wq_irq_return, wait);
    if (g_fpsensor->cancel == 1) {
        fpsensor_debug(ERR_LOG, " cancle\n");
        ret =  POLLERR;
        g_fpsensor->cancel = 0;
        return ret;
    }

    if ( g_fpsensor->RcvIRQ) {
        if (g_fpsensor->RcvIRQ == 2) {
            fpsensor_debug(ERR_LOG, " get fp on notify\n");
            ret |= POLLHUP;
        } else {
            fpsensor_debug(ERR_LOG, " get irq\n");
            ret |= POLLRDNORM;
        }
    } else {
        ret = 0;
    }
    return ret;
}

static int fpsensor_open(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;

    FUNC_ENTRY();
    fpsensor_dev = container_of(inode->i_cdev, fpsensor_data_t, cdev);
    fpsensor_dev->users++;
   // fpsensor_dev->device_available = 1;
    filp->private_data = fpsensor_dev;
    FUNC_EXIT();
    return 0;
}

static int fpsensor_release(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    int    status = 0;
    FUNC_ENTRY();
    if (fpsensor_dev)
    	fpsensor_dev_cleanup(fpsensor_dev);
    FUNC_EXIT();
    return status;
}

static ssize_t fpsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support read opertion in TEE version\n");
    return -EFAULT;
}

static ssize_t fpsensor_write(struct file *filp, const char __user *buf, size_t count,
        loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support write opertion in TEE version\n");
    return -EFAULT;
}

static const struct file_operations fpsensor_fops = {
    .owner          = THIS_MODULE,
    .write          = fpsensor_write,
    .read           = fpsensor_read,
    .unlocked_ioctl = fpsensor_ioctl,
    .compat_ioctl   = fpsensor_compat_ioctl,
    .open           = fpsensor_open,
    .release        = fpsensor_release,
    .poll           = fpsensor_poll,

};

// create and register a char device for fpsensor
static int fpsensor_dev_setup(fpsensor_data_t *fpsensor)
{
    int ret = 0;
    dev_t dev_no = 0;
    struct device *dev = NULL;
    int fpsensor_dev_major = FPSENSOR_DEV_MAJOR;
    int fpsensor_dev_minor = 0;
    fpsensor_debug(INFO_LOG, "%s:enter.\n",__func__);
    FUNC_ENTRY();

    if (fpsensor_dev_major) {
        dev_no = MKDEV(fpsensor_dev_major, fpsensor_dev_minor);
        ret = register_chrdev_region(dev_no, FPSENSOR_NR_DEVS, FPSENSOR_DEV_NAME);
    } else {
        ret = alloc_chrdev_region(&dev_no, fpsensor_dev_minor, FPSENSOR_NR_DEVS, FPSENSOR_DEV_NAME);
        fpsensor_dev_major = MAJOR(dev_no);
        fpsensor_dev_minor = MINOR(dev_no);
        fpsensor_debug(INFO_LOG, "fpsensor device major is %d, minor is %d\n",
                fpsensor_dev_major, fpsensor_dev_minor);
    }

    if (ret < 0) {
        fpsensor_debug(ERR_LOG, "can not get device major number %d\n", fpsensor_dev_major);
        goto out;
    }

    cdev_init(&fpsensor->cdev, &fpsensor_fops);
    fpsensor->cdev.owner = THIS_MODULE;
    fpsensor->cdev.ops   = &fpsensor_fops;
    fpsensor->devno      = dev_no;
    ret = cdev_add(&fpsensor->cdev, dev_no, FPSENSOR_NR_DEVS);
    if (ret) {
        fpsensor_debug(ERR_LOG, "add char dev for fpsensor failed\n");
        goto release_region;
    }

    fpsensor->class = class_create(THIS_MODULE, FPSENSOR_CLASS_NAME);
    if (IS_ERR(fpsensor->class)) {
        fpsensor_debug(ERR_LOG, "create fpsensor class failed\n");
        ret = PTR_ERR(fpsensor->class);
        goto release_cdev;
    }

    dev = device_create(fpsensor->class, &fpsensor->spi->dev, dev_no, fpsensor, FPSENSOR_DEV_NAME);
    if (IS_ERR(dev)) {
        fpsensor_debug(ERR_LOG, "create device for fpsensor failed\n");
        ret = PTR_ERR(dev);
        goto release_class;
    }
    fpsensor_debug(INFO_LOG, "%s:leave.\n",__func__);
    FUNC_EXIT();
    return ret;

release_class:
    class_destroy(fpsensor->class);
    fpsensor->class = NULL;
release_cdev:
    cdev_del(&fpsensor->cdev);
release_region:
    unregister_chrdev_region(dev_no, FPSENSOR_NR_DEVS);
out:
    FUNC_EXIT();
    return ret;
}

#if FP_NOTIFY
static int fpsensor_fb_notifier_callback(struct notifier_block* self, unsigned long event, void* data)
{
    int retval = 0;
    static char screen_status[64] = { '\0' };
    struct fb_event* evdata = data;
    unsigned int blank;
    fpsensor_data_t *fpsensor_dev = g_fpsensor;

    fpsensor_debug(INFO_LOG,"%s enter.  event : 0x%x\n", __func__, (unsigned)event);
    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    blank = *(int*)evdata->data;
    fpsensor_debug(INFO_LOG,"%s enter, blank=0x%x\n", __func__, blank);

    switch (blank) {
        case FB_BLANK_UNBLANK:
            fpsensor_debug(INFO_LOG,"%s: lcd on notify\n", __func__);
            sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
            fpsensor_dev->fb_status = 1;
            if( fpsensor_dev->enable_report_blankon) {
                fpsensor_dev->RcvIRQ = 2;
                wake_up_interruptible(&fpsensor_dev->wq_irq_return);
            }
            break;

        case FB_BLANK_POWERDOWN:
            fpsensor_debug(INFO_LOG,"%s: lcd off notify\n", __func__);
            sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
            fpsensor_dev->fb_status = 0;
            break;

        default:
            fpsensor_debug(INFO_LOG,"%s: other notifier, ignore\n", __func__);
            break;
    }

    fpsensor_debug(INFO_LOG,"%s %s leave.\n", screen_status, __func__);
    return retval;
}
#endif

//#define DEBUG_FP
#ifdef DEBUG_FP
static int tee_spi_transfer(const char *txbuf, char *rxbuf, int len)
{
    struct spi_transfer t;
    struct spi_message m;
    memset(&t, 0, sizeof(t));
    spi_message_init(&m);
    t.tx_buf = txbuf;
    t.rx_buf = rxbuf;
    t.bits_per_word = 8;
    t.len = len;
    t.speed_hz = 1*1000000;
    spi_message_add_tail(&t, &m);
    return spi_sync(g_fpsensor_spidev, &m);
}

static int chipone_read_sensor_id(void)
{
    int ret = -1;
    int trytimes = 3;
    char readbuf[16]  = {0};
    char writebuf[16] = {0};
	 do {
        //1.detect 7153 7222 7332 
        memset(readbuf,  0, sizeof(readbuf));
        memset(writebuf, 0, sizeof(writebuf));
        writebuf[0] = (uint8_t)(0x08);
        writebuf[1] = (uint8_t)(0x55);
		
		ret =  tee_spi_transfer(writebuf, readbuf, 6);
		 if (ret != 0) {
            printk("SPI transfer failed\n");
            continue;
        }
        memset(readbuf,  0, sizeof(readbuf));
        memset(writebuf, 0, sizeof(writebuf));
        writebuf[0] = (uint8_t)(0x00);
        writebuf[1] = (uint8_t)(0x00);
        writebuf[2] = (uint8_t)(0x00);
		
		ret =  tee_spi_transfer(writebuf, readbuf, 7);
		 
		if (ret != 0) {
            printk("SPI transfer failed\n");
            continue;
           }
       
        if (readbuf[1] == 0x71 || readbuf[1] == 0x72 || readbuf[1] == 0x73) {
			printk("jane fpsensor FINGERPRINT CHIPID is ICNT%x%x \n",readbuf[1],readbuf[2]);
            return 0;
        }
	}while (trytimes--);

    printk(" jane chipone probe read chip id is failed\n");
    return -1;
} 
#endif
static int fpsensor_probe(struct spi_device *spi)
{
    int status = 0;
    fpsensor_debug(INFO_LOG, "%s:enter.\n",__func__);

    g_fpsensor_spidev = spi;

#ifdef DEBUG_FP

#ifdef CONFIG_TCT_PROJECT_MODEL_3
    fpsensor_hw_reset(4);
#else
    fpsensor_hw_reset(1250);
#endif

    fpsensor_spi_clk_enable(1);
    chipone_read_sensor_id();
#endif
   
    return status;
}
static int fpsensor_init_driver(void)
{
    int status = 0;
    fpsensor_data_t *fpsensor_dev = NULL;
    fpsensor_debug(INFO_LOG, "%s:enter.\n",__func__);
    FUNC_ENTRY();

    /* Allocate driver data */
    fpsensor_dev = kzalloc(sizeof(*fpsensor_dev), GFP_KERNEL);
    if (!fpsensor_dev) {
        status = -ENOMEM;
        fpsensor_debug(ERR_LOG, "%s, Failed to alloc memory for fpsensor device.\n", __func__);
        goto out;
    }

    /* Initialize the driver data */
    g_fpsensor = fpsensor_dev;
    fpsensor_dev->device_available  = 0;
    fpsensor_dev->users             = 0;
    fpsensor_dev->irq               = 0;
    fpsensor_dev->irq_gpio          = 0;
    fpsensor_dev->irq_enabled       = 0;
    fpsensor_dev->free_flag         = 0;
    /* setup a char device for fpsensor */
    status = fpsensor_dev_setup(fpsensor_dev);
    if (status) {
        fpsensor_debug(ERR_LOG, "fpsensor setup char device failed, %d", status);
        goto release_drv_data;
    }
#ifdef CONFIG_TCT_PROJECT_MODEL_3
#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_init(&g_fpsensor->ttw_wl, "fpsensor_ttw_wl");
#else
    wake_lock_init(&g_fpsensor->ttw_wl, WAKE_LOCK_SUSPEND, "fpsensor_ttw_wl");
#endif
#endif
    fpsensor_debug(INFO_LOG, "%s finished, driver version: %s\n", __func__, FPSENSOR_SPI_VERSION);
    goto out;

release_drv_data:
    kfree(fpsensor_dev);
    fpsensor_dev = NULL;
out:
    FUNC_EXIT();
    fpsensor_debug(INFO_LOG, "%s:leave.\n",__func__);
    return status;
}

static int fpsensor_remove(struct spi_device *spi)
{
    FUNC_ENTRY();
/*
    if (g_fpsensor_spidev){
    	spi_unregister_driver(&fpsensor_spi_driver);
	g_fpsensor_spidev = NULL;
    }
*/
    FUNC_EXIT();
    return 0; 
}

static int __init fpsensor_init(void)
{
    int status;
    fpsensor_debug(INFO_LOG, "%s:enter.\n",__func__);
    status = fpsensor_init_driver();
    fpsensor_debug(INFO_LOG, "%s:leave.\n",__func__);
    return status;
}
module_init(fpsensor_init);

static void __exit fpsensor_exit(void)
{
	return;
}
module_exit(fpsensor_exit);

MODULE_AUTHOR("[TCT]-BSP-XYD/LY");
MODULE_DESCRIPTION(" Fingerprint chip TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fpsensor-drivers");

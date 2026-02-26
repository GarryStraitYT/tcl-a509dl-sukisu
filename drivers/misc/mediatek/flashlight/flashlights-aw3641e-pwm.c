
#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include <mt-plat/mtk_pwm.h> // new add


#include "flashlight-core.h"
#include "flashlight-dt.h"

#define TAG_NAME "[flashligh_aw3641e_pwm_drv]"
#define PK_DBG_NONE(fmt, arg...)  do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)  pr_info(TAG_NAME "%s: " fmt, __func__, ##arg)
#define PK_ERR(fmt, arg...)       pr_info(TAG_NAME "%s: " fmt, __func__, ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_INF(fmt, arg...)       pr_info(TAG_NAME "%s is called.\n", __func__)
#define PK_DBG                    PK_DBG_FUNC
#else
#define PK_INF(fmt, arg...)       do {} while (0)
#define PK_DBG(a, ...)
#endif


/* define device tree */
#ifndef aw3641e_pwm_DTNAME
#define aw3641e_pwm_DTNAME "mediatek,flashlights_aw3641e_pwm"
#endif

#define aw3641e_pwm_NAME "flashlights-aw3641e-pwm"

/* define registers */

/* define mutex and work queue */
static DEFINE_MUTEX(aw3641e_pwm_mutex);
static struct work_struct aw3641e_pwm_work;

/* define pinctrl */
#define aw3641e_pwm_PINCTRL_STATE_EN_HIGH "en_high"
#define aw3641e_pwm_PINCTRL_STATE_EN_LOW  "en_low"

static struct pinctrl *aw3641e_pwm_pinctrl;
static struct pinctrl_state *aw3641e_pwm_en_high;
static struct pinctrl_state *aw3641e_pwm_en_low;

/* define usage count */
static int use_count;
static int g_flash_duty = -1;
static int flashlevel = 15;
int myTHRESH;

/* platform data */
struct aw3641e_pwm_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

static struct pwm_spec_config aw3641e_pwm_conf={
	 .pwm_no = 0, //pwm num
	 .mode = PWM_MODE_OLD,
	 .clk_div = CLK_DIV2,
	 .clk_src = PWM_CLK_OLD_MODE_BLOCK,
	 .pmic_pad =0,
	 .PWM_MODE_OLD_REGS.IDLE_VALUE = IDLE_FALSE,
	 .PWM_MODE_OLD_REGS.GUARD_VALUE = GUARD_FALSE,
	 .PWM_MODE_OLD_REGS.GDURATION = 0,
	 .PWM_MODE_OLD_REGS.WAVE_NUM = 0,
	 .PWM_MODE_OLD_REGS.DATA_WIDTH = 1000,
	 .PWM_MODE_OLD_REGS.THRESH = 0,
};
static int aw3641e_pwm_config(int thresh)
{
	int ret;
	aw3641e_pwm_conf.PWM_MODE_OLD_REGS.THRESH =thresh;
	ret = pwm_set_spec_config(&aw3641e_pwm_conf);
	PK_DBG("THRESH =%d,(sucess :0) ret:%d\n",thresh,ret);
	return ret;
}
static int aw3641e_pwm_PINCTRL_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	aw3641e_pwm_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw3641e_pwm_pinctrl)) {
		PK_ERR("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3641e_pwm_pinctrl);
	}

	aw3641e_pwm_en_high = pinctrl_lookup_state(
			aw3641e_pwm_pinctrl, aw3641e_pwm_PINCTRL_STATE_EN_HIGH);
	if (IS_ERR(aw3641e_pwm_en_high)) {
		PK_ERR("Failed to init (%s)\n",
			aw3641e_pwm_PINCTRL_STATE_EN_HIGH);
		ret = PTR_ERR(aw3641e_pwm_en_high);
	}
	aw3641e_pwm_en_low = pinctrl_lookup_state(
			aw3641e_pwm_pinctrl, aw3641e_pwm_PINCTRL_STATE_EN_LOW);
	if (IS_ERR(aw3641e_pwm_en_low)) {
		PK_ERR("Failed to init (%s)\n", aw3641e_pwm_PINCTRL_STATE_EN_LOW);
		ret = PTR_ERR(aw3641e_pwm_en_low);
	}

	return ret;
}

/***************************************************************************/
//aw3641e Debug file add by liujunyi
/***************************************************************************/
static ssize_t
aw3641e_pwm_get_reg(struct device *cd, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len,"use_count    = %d\n", use_count);
	len += snprintf(buf+len, PAGE_SIZE-len,"g_flash_duty = %d\n", g_flash_duty);
	len += snprintf(buf+len, PAGE_SIZE-len,"flashlevel   = %d\n", flashlevel);
	len += snprintf(buf+len, PAGE_SIZE-len,"myTHRESH   = %d\n", myTHRESH);
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t
aw3641e_pwm_set_reg(struct device *cd,struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[1];
	if (sscanf(buf, "%d", &databuf[0]) == 1)
	{
		if(myTHRESH >=0 && myTHRESH <=100)
			myTHRESH = databuf[0];
		else
			myTHRESH = 15;
	}
	aw3641e_pwm_config(myTHRESH*10);
	return len;
}

static DEVICE_ATTR(aw3641ereg, 0660, aw3641e_pwm_get_reg, aw3641e_pwm_set_reg);

static int aw3641e_pwm_create_sysfs(struct device *dev)
{
	int err;
	err = device_create_file(dev, &dev_attr_aw3641ereg);
	return err;
}

static void aw3641e_pwm_ctrl_pulse(int duty_cycle)
{
	int ret=0;

	if(duty_cycle < 0)//error
		return;

	pinctrl_select_state(aw3641e_pwm_pinctrl, aw3641e_pwm_en_high);
	mdelay(2);
	ret=aw3641e_pwm_config(duty_cycle*100);
	PK_DBG("(sucess :0) ret:%d\n",ret);
}

/* flashlight enable function */
static int aw3641e_pwm_enable(void)
{
	PK_DBG("wxs aw3641e_pwm_enable g_flash_duty:%d\n",g_flash_duty);

	// switch(g_flash_duty){

	// 	case 1:  // torch
	// 		aw3641e_pwm_ctrl_pulse(10);
	// 		break;
	// 	case 2:  // capture
	// 		aw3641e_pwm_ctrl_pulse(90);
	// 		break;
	// 	case 3:  // video
	// 		aw3641e_pwm_ctrl_pulse(50);
	// 		break;
	// 	default:
	// 		aw3641e_pwm_ctrl_pulse(10);
	// 		break;
	// }
	if(g_flash_duty >=0 && g_flash_duty <=9){
		aw3641e_pwm_ctrl_pulse(g_flash_duty);
	}
	else{
		aw3641e_pwm_ctrl_pulse(1);
	}
	return 0;
}

/* flashlight disable function */
static int aw3641e_pwm_disable(void)
{
	PK_DBG("wxs aw3641e_pwm_disable\n");
	pinctrl_select_state(aw3641e_pwm_pinctrl, aw3641e_pwm_en_low);//lbc
	mt_pwm_disable(0, 0);
	mdelay(3);
	return 0;
}

/* set flashlight level */
static int aw3641e_pwm_set_level(int level)
{
	PK_DBG("wxs aw3641e_pwm_set_level g_flash_duty:%d\n",level);
	g_flash_duty = level;
	return 0;
}

/* flashlight init */
static int aw3641e_pwm_init(void)
{
	PK_DBG("wxs aw3641e_pwm_init\n");
	pinctrl_select_state(aw3641e_pwm_pinctrl, aw3641e_pwm_en_low);//lbc
	return 0;
}

/* flashlight uninit */
static int aw3641e_pwm_uninit(void)
{
	aw3641e_pwm_disable();
	return 0;
}

static struct hrtimer aw3641e_pwm_timer;
static unsigned int aw3641e_pwm_timeout_ms;

static void aw3641e_pwm_work_disable(struct work_struct *data)
{
	PK_DBG("work queue callback\n");
	//aw3641e_pwm_disable();
}

static enum hrtimer_restart aw3641e_pwm_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw3641e_pwm_work);
	return HRTIMER_NORESTART;
}


static int aw3641e_pwm_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;
	if(channel != 0)
		return 0;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3641e_pwm_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASH_IOC_SET_DUTY(channel:%d): duty:%d\n",
				channel, (int)fl_arg->arg);
		aw3641e_pwm_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASH_IOC_SET_ONOFF channel(%d): enable:%d ,aw3641e_pwm_timeout_ms:%d\n",
				channel, (int)fl_arg->arg, aw3641e_pwm_timeout_ms);
		if (fl_arg->arg == 1) {
			if (aw3641e_pwm_timeout_ms) {
				s = aw3641e_pwm_timeout_ms / 1000;
				ns = aw3641e_pwm_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&aw3641e_pwm_timer, ktime,
						HRTIMER_MODE_REL);
			}
			aw3641e_pwm_enable();
		} else {
			aw3641e_pwm_disable();
			hrtimer_cancel(&aw3641e_pwm_timer);
		}
		break;

	default:
		PK_INF("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw3641e_pwm_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3641e_pwm_release(void)
{
	/* Move to set driver for saving power */
	aw3641e_pwm_disable();
	return 0;
}

static int aw3641e_pwm_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&aw3641e_pwm_mutex);
	if (set) {
		if (!use_count)
			ret = aw3641e_pwm_init();
		use_count++;
		PK_DBG("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = aw3641e_pwm_uninit();
		if (use_count < 0)
			use_count = 0;
		PK_DBG("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&aw3641e_pwm_mutex);

	return ret;
}

static ssize_t aw3641e_pwm_strobe_store(struct flashlight_arg arg)
{
	aw3641e_pwm_set_driver(1);
	aw3641e_pwm_set_level(arg.level);
	aw3641e_pwm_timeout_ms = 0;
	aw3641e_pwm_enable();
	msleep(arg.dur);
	aw3641e_pwm_disable();
	aw3641e_pwm_set_driver(0);

	return 0;
}

static struct flashlight_operations aw3641e_pwm_ops = {
	aw3641e_pwm_open,
	aw3641e_pwm_release,
	aw3641e_pwm_ioctl,
	aw3641e_pwm_strobe_store,
	aw3641e_pwm_set_driver
};


static int aw3641e_pwm_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * aw3641e_pwm_init();
	 */

	return 0;
}

static int aw3641e_pwm_parse_dt(struct device *dev,
		struct aw3641e_pwm_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		PK_INF("Parse no dt, node.\n");
		return 0;
	}
	PK_INF("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		PK_INF("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				aw3641e_pwm_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int aw3641e_pwm_probe(struct platform_device *pdev)
{
	struct aw3641e_pwm_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	PK_DBG("Probe start.\n");

	/* init pinctrl */
	if (aw3641e_pwm_PINCTRL_init(pdev)) {
		PK_DBG("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = aw3641e_pwm_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&aw3641e_pwm_work, aw3641e_pwm_work_disable);

	/* init timer */
	hrtimer_init(&aw3641e_pwm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw3641e_pwm_timer.function = aw3641e_pwm_timer_func;
	aw3641e_pwm_timeout_ms = 100;

	/* init chip hw */
	aw3641e_pwm_chip_init();
	//add by junyilu for rapidtf
	aw3641e_pwm_create_sysfs(&pdev->dev);
	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&aw3641e_pwm_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(aw3641e_pwm_NAME, &aw3641e_pwm_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	PK_DBG("Probe done.\n");
	aw3641e_pwm_init();
	/* pwm config init */
	// aw3641e_pwm_config(0);
	return 0;
err:
	return err;
}

static int aw3641e_pwm_remove(struct platform_device *pdev)
{
	struct aw3641e_pwm_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	PK_DBG("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(aw3641e_pwm_NAME);

	/* flush work queue */
	flush_work(&aw3641e_pwm_work);

	PK_DBG("Remove done.\n");

	return 0;
}

static const struct of_device_id aw3641e_pwm_gpio_of_match[] = {
	{.compatible = aw3641e_pwm_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw3641e_pwm_gpio_of_match);

static struct platform_driver aw3641e_pwm_platform_driver = {
	.probe = aw3641e_pwm_probe,
	.remove = aw3641e_pwm_remove,
	.driver = {
		.name = aw3641e_pwm_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3641e_pwm_gpio_of_match,
#endif
	},
};

static int __init flashlight_aw3641e_pwm_init(void)
{
	int ret;

	PK_DBG("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3641e_pwm_gpio_platform_device);
	if (ret) {
		PK_ERR("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw3641e_pwm_platform_driver);
	if (ret) {
		PK_ERR("Failed to register platform driver\n");
		return ret;
	}

	PK_DBG("Init done.\n");

	return 0;
}

static void __exit flashlight_aw3641e_pwm_exit(void)
{
	PK_DBG("Exit start.\n");

	platform_driver_unregister(&aw3641e_pwm_platform_driver);

	PK_DBG("Exit done.\n");
}

module_init(flashlight_aw3641e_pwm_init);
module_exit(flashlight_aw3641e_pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dongchun Zhu <dongchun.zhu@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight aw3641e GPIO Driver");


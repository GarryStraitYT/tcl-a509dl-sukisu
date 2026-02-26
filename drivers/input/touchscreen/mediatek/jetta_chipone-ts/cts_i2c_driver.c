#define LOG_TAG         "I2CDrv"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_sysfs.h"

extern char dev_tp_fw[30];

bool cts_show_debug_log = 1;
module_param_named(debug_log, cts_show_debug_log, bool, 0660);
MODULE_PARM_DESC(debug_log, "Show debug log control");

struct chipone_ts_data *g_cts_data = NULL;
int cts_suspend(struct chipone_ts_data *cts_data)
{
    int ret;
    //cts_info("CTS:%s",dev_tp_fw);   
    cts_info("Suspend");

    cts_lock_device(&cts_data->cts_dev);
    ret = cts_suspend_device(&cts_data->cts_dev);
    cts_unlock_device(&cts_data->cts_dev);

    if (ret) {
        cts_err("Suspend device failed %d", ret);
        // TODO:
        //return ret;
    }

    ret = cts_stop_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Stop device failed %d", ret);
        return ret;
    }

#ifdef CONFIG_CTS_GESTURE
    /* Enable IRQ wake if gesture wakeup enabled */
    if (cts_is_gesture_wakeup_enabled(&cts_data->cts_dev)) {
        ret = cts_plat_enable_irq_wake(cts_data->pdata);
        if (ret) {
            cts_err("Enable IRQ wake failed %d", ret);
            return ret;
        }
        ret = cts_plat_enable_irq(cts_data->pdata);
        if (ret){
            cts_err("Enable IRQ failed %d",ret);
            return ret;
        }
    }
#endif /* CONFIG_CTS_GESTURE */

    /** - To avoid waking up while not sleeping,
            delay 20ms to ensure reliability */
    msleep(20);

    return 0;
}

int cts_resume(struct chipone_ts_data *cts_data)
{
    int ret;

    cts_info("Resume");

#ifdef CONFIG_CTS_GESTURE
    if (cts_is_gesture_wakeup_enabled(&cts_data->cts_dev)) {
        ret = cts_plat_disable_irq_wake(cts_data->pdata);
        if (ret) {
            cts_warn("Disable IRQ wake failed %d", ret);
            //return ret;
        }
        if ((ret = cts_plat_disable_irq(cts_data->pdata)) < 0) {
            cts_err("Disable IRQ failed %d", ret);
            //return ret;
        }
    }
#endif /* CONFIG_CTS_GESTURE */

    ret = cts_resume_device(&cts_data->cts_dev);
    if(ret) {
        cts_warn("Resume device failed %d", ret);
        return ret;
    }

    ret = cts_start_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Start device failed %d", ret);
        return ret;
    }

    return 0;
}

extern int is_4th_tp;
extern char ctp_module_name1[256];

static int cts_i2c_driver_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct chipone_ts_data *cts_data = NULL;
    int ret = 0;

    cts_info("Probe i2c client: name='%s' addr=0x%02x flags=0x%02x irq=%d",
        client->name, client->addr, client->flags, client->irq);

#if !defined(CONFIG_MTK_PLATFORM)
    if (client->addr != CTS_NORMAL_MODE_I2CADDR) {
        cts_err("Probe i2c addr 0x%02x != driver config addr 0x%02x",
            client->addr, CTS_NORMAL_MODE_I2CADDR);
        return -ENODEV;
    };
#endif

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        cts_err("Check functionality failed");
        return -ENODEV;
    }

    cts_data = (struct chipone_ts_data *)kzalloc(sizeof(*cts_data), GFP_KERNEL);
    if (cts_data == NULL) {
        cts_err("Allocate chipone_ts_data failed");
        return -ENOMEM;
    }

    cts_data->pdata = (struct cts_platform_data *)kzalloc(
            sizeof(struct cts_platform_data), GFP_KERNEL);
    if (cts_data->pdata == NULL) {
        cts_err("Allocate cts_platform_data failed");
        ret = -ENOMEM;
        goto err_free_cts_data;
    }

    i2c_set_clientdata(client, cts_data);
    cts_data->i2c_client = client;
	g_cts_data = cts_data;
    cts_init_platform_data(cts_data->pdata, client);

    cts_data->cts_dev.pdata = cts_data->pdata;
    cts_data->pdata->cts_dev = &cts_data->cts_dev;

    cts_data->workqueue = create_singlethread_workqueue(CFG_CTS_DEVICE_NAME "-workqueue");
    if (cts_data->workqueue == NULL) {
        cts_err("Create workqueue failed");
        ret = -ENOMEM;
        goto err_deinit_pdata;
    }

    ret = cts_plat_request_resource(cts_data->pdata);
    if (ret < 0) {
        cts_err("Request resource failed %d", ret);
        goto err_destroy_workqueue;
    }

    ret = cts_plat_reset_device(cts_data->pdata);
    if (ret < 0) {
        cts_err("Reset device failed %d", ret);
        goto err_free_resource;
    }

    ret = cts_probe_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Probe device failed %d", ret);
        goto err_free_resource;
    }

    ret = cts_plat_init_touch_device(cts_data->pdata);
    if (ret < 0) {
        cts_err("Init touch device failed %d", ret);
        goto err_free_resource;
    }

    ret = cts_plat_init_vkey_device(cts_data->pdata);
    if (ret < 0) {
        cts_err("Init vkey device failed %d", ret);
        goto err_deinit_touch_device;
    }
    
    ret = cts_plat_init_gesture(cts_data->pdata);
    if (ret < 0) {
        cts_err("Init gesture failed %d", ret);
        goto err_deinit_vkey_device;
    }

    cts_init_esd_protection(cts_data);

    ret = cts_tool_init(cts_data);
    if (ret < 0) {
        cts_warn("Init tool node failed %d", ret);
    }

    #ifdef CTS_TOUCH_FACTORY_TEST
    ret = cts_factory_init(cts_data);
    if (ret < 0) {
        cts_warn("Init factory test node failed %d", ret);
    }
    #endif
    #ifdef CONFIG_CTS_PROC_GLOVE
    ret = cts_proc_glove_init(cts_data);
    if (ret < 0) {
        cts_warn("Init glove node failed %d", ret);
    }
    #endif
	
    ret = cts_sysfs_add_device(&client->dev);
    if (ret < 0) {
        cts_warn("Add sysfs entry for device failed %d", ret);
    }

    ret = cts_plat_request_irq(cts_data->pdata);
    if (ret < 0) {
        cts_err("Request IRQ failed %d", ret);
        goto err_deinit_sysfs;
    }

    ret = cts_start_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Start device failed %d", ret);
        goto err_free_irq;
    }
    cts_info("probe is finshed");
    cts_info("CTS:%s",dev_tp_fw);
	if (is_4th_tp == 0)
		snprintf(ctp_module_name1, 256, "CHIPONE:3RD\n");
	else
		snprintf(ctp_module_name1, 256, "CHIPONE:4TH\n");

#ifdef CONFIG_MTK_PLATFORM
    tpd_load_status = 1;
#endif /* CONFIG_MTK_PLATFORM */

    return 0;

err_free_irq:
    cts_plat_free_irq(cts_data->pdata);
err_deinit_sysfs:
    cts_sysfs_remove_device(&client->dev);
#ifdef CONFIG_CTS_LEGACY_TOOL
    cts_tool_deinit(cts_data);
#endif /* CONFIG_CTS_LEGACY_TOOL */

#ifdef CTS_TOUCH_FACTORY_TEST
    cts_factory_deinit(cts_data);
#endif
#ifdef CONFIG_CTS_PROC_GLOVE
    cts_proc_glove_deinit(cts_data);
#endif

#ifdef CONFIG_CTS_ESD_PROTECTION
    cts_deinit_esd_protection(cts_data);
#endif /* CONFIG_CTS_ESD_PROTECTION */

#ifdef CONFIG_CTS_GESTURE
    cts_plat_deinit_gesture(cts_data->pdata);
#endif /* CONFIG_CTS_GESTURE */

err_deinit_vkey_device:
#ifdef CONFIG_CTS_VIRTUALKEY
    cts_plat_deinit_vkey_device(cts_data->pdata);
#endif /* CONFIG_CTS_VIRTUALKEY */

err_deinit_touch_device:
    cts_plat_deinit_touch_device(cts_data->pdata);

err_free_resource:
    cts_plat_free_resource(cts_data->pdata);

err_destroy_workqueue:
    destroy_workqueue(cts_data->workqueue);

err_deinit_pdata:
    cts_deinit_platform_data(cts_data->pdata);
    kfree(cts_data->pdata);

err_free_cts_data:
    kfree(cts_data);

    cts_err("Probe failed %d", ret);

    return ret;
}

static int cts_i2c_driver_remove(struct i2c_client *client)
{
    struct chipone_ts_data *cts_data;
    int ret = 0;

    cts_info("Remove");

    cts_data = (struct chipone_ts_data *)i2c_get_clientdata(client);
    if (cts_data) {
        ret = cts_stop_device(&cts_data->cts_dev);
        if (ret) {
            cts_warn("Stop device failed %d", ret);
        }

        cts_plat_free_irq(cts_data->pdata);

        cts_tool_deinit(cts_data);
#ifdef CTS_TOUCH_FACTORY_TEST
	cts_factory_deinit(cts_data);
#endif
#ifdef CONFIG_CTS_GESTURE
    	cts_plat_deinit_gesture(cts_data->pdata);
#endif /* CONFIG_CTS_GESTURE */
#ifdef CONFIG_CTS_PROC_GLOVE
	cts_proc_glove_deinit(cts_data);
#endif
        cts_sysfs_remove_device(&client->dev);

        cts_deinit_esd_protection(cts_data);

        if (cts_data->pdata) {
            cts_plat_deinit_touch_device(cts_data->pdata);

            cts_plat_deinit_vkey_device(cts_data->pdata);

            //cts_plat_deinit_gesture(cts_data->pdata);

            cts_plat_free_resource(cts_data->pdata);

            cts_deinit_platform_data(cts_data->pdata);

            kfree(cts_data->pdata);
        }

        if (cts_data->workqueue) {
            destroy_workqueue(cts_data->workqueue);
        }

        kfree(cts_data);
    }else {
        cts_warn("Chipone i2c driver remove while NULL chipone_ts_data");
        return -EINVAL;
    }

    return ret;
}

#ifdef CONFIG_CTS_GESTURE_SWITCH
static ssize_t cts_gesture_switch_read(struct file *file,
        char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len;
	char gesture_switch_enable_str[32];
	struct cts_platform_data *pdata = NULL;
	if(g_cts_data ==  NULL){
		cts_err("Read proc file gesture swithc with cts_data = NULL");
	 	return -EFAULT;
		}
	pdata = g_cts_data->pdata;
	if(buffer == NULL){
		cts_err("Read proc file gesture swithc with buff = NULL");
		return -EINVAL;
		}
	if (*ppos != 0) {
		return 0;
	}
	cts_info("Read '"CFG_CTS_GESTURE_STATE"'");

	len = scnprintf(gesture_switch_enable_str, sizeof(gesture_switch_enable_str),
	    "GESTURE_STATE = %d\n", pdata->cts_dev->rtdata.gesture_wakeup_enabled);
	*ppos += len;

    ret = copy_to_user(buffer, gesture_switch_enable_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d", ret);
		return 0;
    }
	
    return len;

}


static ssize_t cts_gesture_switch_write(struct file *file,
				const char __user * buffer, size_t count, loff_t * ppos)
{
	int ret = 0;
	char gesture_switch_enable[8];
	struct cts_platform_data *pdata = NULL;
	
	if(g_cts_data ==  NULL){
		cts_err("write proc file gesture swithc with cts_data = NULL");
	 	return -EFAULT;
		}
	pdata = g_cts_data->pdata;
	
	if(buffer == NULL){
		cts_err("write proc file gesture swithc with buff = NULL");
		return -EINVAL;
		}

	cts_info("write '"CFG_CTS_GESTURE_STATE"'count: %zu", count);

	ret = copy_from_user(gesture_switch_enable, buffer, count);
    if (ret) {
        cts_err("Write invalid gesture_switch %d", ret);
        return -EINVAL;
    }

	if('o' == gesture_switch_enable[0]){
        cts_enable_gesture_wakeup(pdata->cts_dev);
    }else if('c' == gesture_switch_enable[0]){
        cts_disable_gesture_wakeup(pdata->cts_dev);
    }
	
    cts_info("Set gesture_switch = %c", gesture_switch_enable[0]);

    return count;
}


static struct file_operations cts_gesture_switch_fops = {
    .owner = THIS_MODULE,
    .read  = cts_gesture_switch_read,
    .write = cts_gesture_switch_write,
};

int cts_gesture_switch_init(struct chipone_ts_data *cts_data)
{
    int ret = 0;

    cts_info("cts_gesture_switch Init");

    cts_data->proc_gesture_state = proc_create_data(CFG_CTS_GESTURE_STATE,
            0666, NULL, &cts_gesture_switch_fops, cts_data);
    if (IS_ERR(cts_data->proc_gesture_state)) {
        cts_err("Create proc gesture entry failed %ld",
            PTR_ERR(cts_data->proc_gesture_state));
        cts_data->proc_gesture_state = NULL;
    }

    return ret;
}

void cts_gesture_switch_deinit(struct chipone_ts_data *cts_data)
{
    cts_info("cts_gesture_switch Deinit");

    if (cts_data->proc_gesture_state) {
        remove_proc_entry(CFG_CTS_GESTURE_STATE, NULL);
    }
}

#endif
#ifdef CONFIG_CTS_PROC_GLOVE
static ssize_t cts_proc_glove_read(struct file *file,
        char __user *buffer, size_t count, loff_t *ppos)
{
    int ret = 0;
    int len;
    char glove_enable_str[32];
    struct cts_platform_data *pdata = NULL;
    if(g_cts_data ==  NULL){
	cts_err("Read proc file glove with cts_data = NULL");
        return -EFAULT;
        }
    pdata = g_cts_data->pdata;
    if(buffer == NULL){
	cts_err("Read proc file glove switch with buff = NULL");
	return -EINVAL;
	}
    if (*ppos != 0) {
	return 0;
    }
    cts_info("Read '"CFG_CTS_GLOVE_NODE"'");

    len = scnprintf(glove_enable_str, sizeof(glove_enable_str),
        "glove = %d\n", pdata->cts_dev->rtdata.glove_mode_enabled);
    *ppos += len;

    ret = copy_to_user(buffer, glove_enable_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d", ret);
		return 0;
    }
	
    return len;

}

static ssize_t cts_proc_glove_write(struct file *file,
				const char __user * buffer, size_t count, loff_t * ppos)
{
    int ret = 0;
    char glove_enable[8];
    struct cts_platform_data *pdata = NULL;
	
    if(g_cts_data ==  NULL){
	cts_err("write proc file glove with cts_data = NULL");
	return -EFAULT;
    }
    pdata = g_cts_data->pdata;
	
    if(buffer == NULL){
	cts_err("write proc file glove with buff = NULL");
	return -EINVAL;
    }

    cts_info("write '"CFG_CTS_GLOVE_NODE"'count: %zu", count);

    ret = copy_from_user(glove_enable, buffer, count);
    if (ret) {
        cts_err("Write invalid glove %d", ret);
        return -EINVAL;
    }

    if('o' == glove_enable[0]){
        //cts_resume(g_cts_data);
		cts_enter_glove_mode(pdata->cts_dev);
    }else if('c' == glove_enable[0]){
        //cts_suspend(g_cts_data);
        cts_exit_glove_mode(pdata->cts_dev);
    }
	
    cts_info("Set glove = %c", glove_enable[0]);

    return count;
}
static struct file_operations cts_glove_fops = {
    .owner = THIS_MODULE,
    .read  = cts_proc_glove_read,
    .write = cts_proc_glove_write,
};

int cts_proc_glove_init(struct chipone_ts_data *cts_data)
{
    int ret = 0;

    cts_info("cts_proc_glove Init");

    cts_data->proc_glove = proc_create_data(CFG_CTS_GLOVE_NODE,
            0666, NULL, &cts_glove_fops, cts_data);
    if (IS_ERR(cts_data->proc_glove)) {
        cts_err("Create proc_glove entry failed %ld",
            PTR_ERR(cts_data->proc_glove));
        cts_data->proc_glove = NULL;
    }

    return ret;
}

void cts_proc_glove_deinit(struct chipone_ts_data *cts_data)
{
    cts_info("cts_proc glove Deinit");

    if (cts_data->proc_glove) {
        remove_proc_entry(CFG_CTS_GLOVE_NODE, NULL);
    }
}
#endif

#ifdef CONFIG_CTS_SYSFS

#define DRIVER_ATTR(_name, _mode, _show, _store) \
  	struct driver_attribute driver_attr_##_name = \
  	__ATTR(_name, _mode, _show, _store)


static ssize_t reset_pin_show(struct device_driver *driver, char *buf)
{
	return sprintf(buf, "CFG_CTS_HAS_RESET_PIN: %c\n",
#ifdef CFG_CTS_HAS_RESET_PIN
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(reset_pin, S_IRUGO, reset_pin_show, NULL);

static ssize_t swap_xy_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CFG_CTS_SWAP_XY: %c\n",
#ifdef CFG_CTS_SWAP_XY
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(swap_xy, S_IRUGO, swap_xy_show, NULL);

static ssize_t wrap_x_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CFG_CTS_WRAP_X: %c\n",
#ifdef CFG_CTS_WRAP_X
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(wrap_x, S_IRUGO, wrap_x_show, NULL);

static ssize_t wrap_y_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CFG_CTS_WRAP_Y: %c\n",
#ifdef CFG_CTS_WRAP_Y
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(wrap_y, S_IRUGO, wrap_y_show, NULL);

static ssize_t force_update_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CFG_CTS_HAS_RESET_PIN: %c\n",
#ifdef CFG_CTS_FIRMWARE_FORCE_UPDATE
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(force_update, S_IRUGO, force_update_show, NULL);

static ssize_t max_touch_num_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CFG_CTS_MAX_TOUCH_NUM: %d\n",
        CFG_CTS_MAX_TOUCH_NUM);
}
static DRIVER_ATTR(max_touch_num, S_IRUGO, max_touch_num_show, NULL);

static ssize_t vkey_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CONFIG_CTS_VIRTUALKEY: %c\n",
#ifdef CONFIG_CTS_VIRTUALKEY
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(vkey, S_IRUGO, vkey_show, NULL);

static ssize_t gesture_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CONFIG_CTS_GESTURE: %c\n",
#ifdef CONFIG_CTS_GESTURE
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(gesture, S_IRUGO, gesture_show, NULL);

static ssize_t esd_protection_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CONFIG_CTS_ESD_PROTECTION: %c\n",
#ifdef CONFIG_CTS_ESD_PROTECTION
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(esd_protection, S_IRUGO, esd_protection_show, NULL);

static ssize_t slot_protocol_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CONFIG_CTS_SLOTPROTOCOL: %c\n",
#ifdef CONFIG_CTS_SLOTPROTOCOL
        'Y'
#else
        'N'
#endif
	    );
}
static DRIVER_ATTR(slot_protocol, S_IRUGO, slot_protocol_show, NULL);

static ssize_t i2c_xfer_size_show(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "CFG_CTS_MAX_I2C_XFER_SIZE: %d\n",
        CFG_CTS_MAX_I2C_XFER_SIZE);
}
static DRIVER_ATTR(i2c_xfer_size, S_IRUGO, i2c_xfer_size_show, NULL);

static struct attribute *cts_i2c_driver_config_attrs[] = {
    &driver_attr_reset_pin.attr,
    &driver_attr_swap_xy.attr,
    &driver_attr_wrap_x.attr,
    &driver_attr_wrap_y.attr,
    &driver_attr_force_update.attr,
	&driver_attr_max_touch_num.attr,
	&driver_attr_vkey.attr,
    &driver_attr_gesture.attr,
	&driver_attr_esd_protection.attr,
    &driver_attr_slot_protocol.attr,
    &driver_attr_i2c_xfer_size.attr,
	NULL
};

static const struct attribute_group cts_i2c_driver_config_group = {
    .name = "config",
	.attrs = cts_i2c_driver_config_attrs,
};

static const struct attribute_group *cts_i2c_driver_config_groups[] = {
    &cts_i2c_driver_config_group,
    NULL,
};
#endif /* CONFIG_CTS_SYSFS */

#ifdef CONFIG_CTS_OF
static const struct of_device_id cts_i2c_of_match_table[] = {
    {.compatible = CFG_CTS_OF_DEVICE_ID_NAME,},
    { },
};
MODULE_DEVICE_TABLE(of, cts_i2c_of_match_table);
#endif /* CONFIG_CTS_OF */

static const struct i2c_device_id cts_i2c_device_id_table[] = {
    {CFG_CTS_DEVICE_NAME, 0},
    {}
};

static struct i2c_driver cts_i2c_driver = {
    .probe = cts_i2c_driver_probe,
    .remove = cts_i2c_driver_remove,
    .driver = {
        .name = CFG_CTS_DRIVER_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_CTS_OF
        .of_match_table = of_match_ptr(cts_i2c_of_match_table),
#endif /* CONFIG_CTS_OF */
#ifdef CONFIG_CTS_SYSFS
        .groups = cts_i2c_driver_config_groups,
#endif /* CONFIG_CTS_SYSFS */
    },
    .id_table = cts_i2c_device_id_table,
};

int cts_i2c_driver_init(void)
{
    cts_info("Init");

    return i2c_add_driver(&cts_i2c_driver);
}

void cts_i2c_driver_exit(void)
{
    cts_info("Exit");

    i2c_del_driver(&cts_i2c_driver);
}


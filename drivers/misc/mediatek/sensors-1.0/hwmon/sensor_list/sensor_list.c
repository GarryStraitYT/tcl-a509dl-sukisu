// SPDX-License-Identifier: GPL-2.0

#define pr_fmt(fmt) "<SEN_LIST> " fmt

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "SCP_sensorHub.h"
#include "sensor_list.h"
#include "SCP_power_monitor.h"
#include "hwmsensor.h"

struct sensorlist_info_t {
	char name[16];
};

enum {
	accel,
	gyro,
	mag,
	als,
	ps,
	baro,
	sar,
	maxhandle,
};

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static struct work_struct sensorlist_work;
#endif
static atomic_t first_ready_after_boot;
static struct sensorlist_info_t sensorlist_info[maxhandle];
static DEFINE_SPINLOCK(sensorlist_info_lock);

inline int sensor_to_handle(int sensor)
{
	int handle = -1;

	switch (sensor) {
	case ID_ACCELEROMETER:
		handle = accel;
		break;
	case ID_GYROSCOPE:
		handle = gyro;
		break;
	case ID_MAGNETIC:
		handle = mag;
		break;
	case ID_LIGHT:
		handle = als;
		break;
	case ID_PROXIMITY:
		handle = ps;
		break;
	case ID_PRESSURE:
		handle = baro;
		break;
	case ID_SAR:
		handle = sar;
		break;
	}
	return handle;
}

static inline int handle_to_sensor(int handle)
{
	int sensor = -1;

	switch (handle) {
	case accel:
		sensor = ID_ACCELEROMETER;
		break;
	case gyro:
		sensor = ID_GYROSCOPE;
		break;
	case mag:
		sensor = ID_MAGNETIC;
		break;
	case als:
		sensor = ID_LIGHT;
		break;
	case ps:
		sensor = ID_PROXIMITY;
		break;
	case baro:
		sensor = ID_PRESSURE;
		break;
	case sar:
		sensor = ID_SAR;
		break;
	}
	return sensor;
}

static void init_sensorlist_info(void)
{
	int handle = -1;

	for (handle = accel; handle < maxhandle; ++handle)
		strlcpy(sensorlist_info[handle].name,
			"NULL",
			sizeof(sensorlist_info[handle].name));
}
//begin add by yan.gong for task 11476711 deviceinfo on 2020/09/09
#ifdef CONFIG_TCT_DEVICEINFO
extern char lsensor_module_name[256];
extern char psensor_module_name[256];
extern char gsensor_module_name[256];
extern char compass_module_name[256];
extern char barometer_module_name[256];
extern char gyroscope_module_name[256];
#endif
//end add by yan.gong for task 11476711 deviceinfo on 2021/09/09
//Begin added yan.gong for CIVICS on 2022-11-27
#ifdef CONFIG_SR_HARDWARE_INFOMATION
extern void sr_add_gsensor_ic_name(char *name);
extern void sr_add_alsps_ic_name(char *name);
#endif
//End added by yan.gong for CIVICS on 2022-11-27

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static void sensorlist_get_deviceinfo(struct work_struct *work)
{
	int err = 0, handle = -1, sensor = -1;
	struct sensorInfo_t devinfo;

	for (handle = accel; handle < maxhandle; ++handle) {
		sensor = handle_to_sensor(handle);
		if (sensor < 0)
			continue;
		memset(&devinfo, 0, sizeof(struct sensorInfo_t));
		err = sensor_set_cmd_to_hub(sensor,
			CUST_ACTION_GET_SENSOR_INFO, &devinfo);
//Begin modified by xiakang.chen for CIVICS-2414 on 2022-11-01
#if defined (CONFIG_TCT_PROJECT_CIVIC_S)
	          if(ID_PROXIMITY==sensor)
	         {
				strlcpy(devinfo.name,	"TP_PS",sizeof("TP_PS"));
				 err=0;
	          }
		  else
#endif
//End modified by xiakang.chen for CIVICS-2414 on 2022-11-01
		if (err < 0) {
			pr_err("sensor(%d) not register\n", sensor);
			continue;
		}
	//begin add by yan.gong for task 11476711 deviceinfo on 2020/09/09
        #ifdef CONFIG_TCT_DEVICEINFO
		// begin add by jianchuan.hu for task BORATF-44 sensor info on 2022/08/22
		// alsps device info
		if(!strcmp(devinfo.name,"stk3a5x_p"))
			sprintf(psensor_module_name, "STK33562:SENSORTEK:0x46:AMY0001878C1");
		if(!strcmp(devinfo.name,"stk3a5x_l"))
			sprintf(lsensor_module_name, "STK33562:SENSORTEK:0x46:AMY0001878C1");
		if(!strcmp(devinfo.name,"STK33562-175"))
			sprintf(psensor_module_name, "STK33562:SENSORTEK:0x46:AMY0001878C1");
		if(!strcmp(devinfo.name,"ucs148g1_p"))
			sprintf(psensor_module_name, "UCS148G1-T17:UCAP:0x38:AMY0002198C1");
		if(!strcmp(devinfo.name,"ucs148g1_l"))
			sprintf(lsensor_module_name, "UCS148G1-T17:UCAP:0x38:AMY0002198C1");
//Begin added by liangjiaqiang for MODEL3-1949 on 2022-09-26
		if(!strcmp(devinfo.name,"ltr569_p"))
			sprintf(psensor_module_name, "LTR569:GB:0x23:178226940");
		if(!strcmp(devinfo.name,"ltr569_l"))
			sprintf(lsensor_module_name, "LTR569:GB:0x23:178226940");
//End added by liangjiaqiang for MODEL3-1949 on 2022-09-26

//Begin added by liangjiaqiang for MODEL3-4928 on 2022-11-17
	#ifdef CONFIG_TCT_PROJECT_MODEL_3
		if(!strcmp(devinfo.name,"stk3a5x_p"))
			sprintf(psensor_module_name, "STK33562:SENSORTEK:0x46:178192743");
		if(!strcmp(devinfo.name,"stk3a5x_l"))
			sprintf(lsensor_module_name, "STK33562:SENSORTEK:0x46:178192743");
	#endif
//End added by liangjiaqiang for MODEL3-4928 on 2022-11-17

		// Barometer  device info
		if(!strcmp(devinfo.name,"lps22hh"))
			sprintf(barometer_module_name, "LPS22HHTR:ST:0x5D:AMY0001671C1");
		if(!strcmp(devinfo.name,"icp201xx_B"))
			sprintf(barometer_module_name, "ICP-20100:TDK:0x64:AMY0002167C1");
		if(!strcmp(devinfo.name,"spl07_001"))
			sprintf(barometer_module_name, "SPL07-001:GOER:0x77:AMY0002102C1");

		// accgyro  device info
		if(!strcmp(devinfo.name,"lsm6dso_acc"))
			sprintf(gsensor_module_name, "LSM6DSOETR3:ST:NA:AMY0001889C1");
		if(!strcmp(devinfo.name,"lsm6dso_gyro"))
			sprintf(gyroscope_module_name, "LSM6DSOETR3:ST:NA:AMY0001889C1");
		if(!strcmp(devinfo.name,"icm4n607_acc"))
			sprintf(gsensor_module_name, "ICM-42607-P:TDK:NA:AMY0001985C1");
		if(!strcmp(devinfo.name,"icm4n607_gyro"))
			sprintf(gyroscope_module_name, "ICM-42607-P:TDK:NA:AMY0001985C1");

		// acc  device info
		if(!strcmp(devinfo.name,"stk8ba58"))
			sprintf(gsensor_module_name, "STK8BA58:SENSORTEK:0x18:AMY0001796C1");
		if(!strcmp(devinfo.name,"MC3416"))
			sprintf(gsensor_module_name, "MMC3416:MCUBE:0x4C:AMY0001586C1");
//Begin added by liangjiaqiang for MODEL3-1949 on 2022-09-26
		if(!strcmp(devinfo.name,"sc7a20"))
			sprintf(gsensor_module_name, "SC7A20:SL:0x18:178238106");
//End added by liangjiaqiang for MODEL3-1949 on 2022-09-26
		// compass device info
		if(!strcmp(devinfo.name,"mmc5603"))
			sprintf(compass_module_name, "MMC5603NJ:MEMSIC:0x30:AMY0001890C1");
		if(!strcmp(devinfo.name,"qmc6308"))
			sprintf(compass_module_name, "QMC6308:QST:0x2c:AMY0002198C1");
		// end add by jianchuan.hu for task SNTTF-4287 deviceinfo on 2022/08/22
	#endif
	//end add by yan.gong for task 11476711 deviceinfo on 2021/09/09

//Begin added by yan.gong for CIVICS on 2022-11-27
#ifdef CONFIG_SR_HARDWARE_INFOMATION
		printk("sensorlist get sensor_name: %s\n", devinfo.name);
		if(!strcmp(devinfo.name, "TP_PS")) {
			sr_add_alsps_ic_name("TP_PS");
		}

		if(!strcmp(devinfo.name, "sc7a20")) {
			sr_add_gsensor_ic_name("SC7A20");
		}
		if(!strcmp(devinfo.name, "stk8ba58")) {
			sr_add_gsensor_ic_name("STK8BA58");
		}

#endif
//End added by yan.gong for CIVICS on 2022-11-27

		spin_lock(&sensorlist_info_lock);
		strlcpy(sensorlist_info[handle].name,
			devinfo.name,
			sizeof(sensorlist_info[handle].name));
		spin_unlock(&sensorlist_info_lock);
	}
}

static int scp_ready_event(uint8_t event, void *ptr)
{
	switch (event) {
	case SENSOR_POWER_UP:
		if (likely(atomic_xchg(&first_ready_after_boot, 1)))
			return 0;
		schedule_work(&sensorlist_work);
		break;
	case SENSOR_POWER_DOWN:
		break;
	}
	return 0;
}

static struct scp_power_monitor scp_ready_notifier = {
	.name = "sensorlist",
	.notifier_call = scp_ready_event,
};
#else
int sensorlist_register_deviceinfo(int sensor,
		struct sensorInfo_NonHub_t *devinfo)
{
	int handle = -1;

	handle = sensor_to_handle(sensor);
	if (handle < 0)
		return -1;
	spin_lock(&sensorlist_info_lock);
	strlcpy(sensorlist_info[handle].name,
		devinfo->name,
		sizeof(sensorlist_info[handle].name));
	spin_unlock(&sensorlist_info_lock);
	return 0;
}
#endif

static int sensorlist_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static ssize_t
sensorlist_read(struct file *file, char __user *buf,
	size_t count, loff_t *ptr)
{
	if (!atomic_read(&first_ready_after_boot))
		return -EINVAL;
	if (count == 0)
		return -EINVAL;
	if (count < sizeof(struct sensorlist_info_t))
		return -EINVAL;
	if (count > maxhandle * sizeof(struct sensorlist_info_t))
		count = maxhandle * sizeof(struct sensorlist_info_t);

	spin_lock(&sensorlist_info_lock);
	if (copy_to_user(buf, sensorlist_info, count)) {
		spin_unlock(&sensorlist_info_lock);
		return -EFAULT;
	}
	spin_unlock(&sensorlist_info_lock);
	return count;
}

static const struct file_operations sensorlist_fops = {
	.owner		= THIS_MODULE,
	.open		= sensorlist_open,
	.read		= sensorlist_read,
};

static struct miscdevice sensorlist_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sensorlist",
	.fops = &sensorlist_fops,
};

static int __init sensorlist_init(void)
{
	int ret = 0;

	init_sensorlist_info();
	ret = misc_register(&sensorlist_miscdev);
	if (ret < 0)
		return -1;
	atomic_set(&first_ready_after_boot, 0);
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&sensorlist_work, sensorlist_get_deviceinfo);
	scp_power_monitor_register(&scp_ready_notifier);
#endif
	return 0;
}

static void __exit sensorlist_exit(void)
{

}
module_init(sensorlist_init);
module_exit(sensorlist_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("dynamic sensorlist driver");
MODULE_AUTHOR("hongxu.zhao@mediatek.com");

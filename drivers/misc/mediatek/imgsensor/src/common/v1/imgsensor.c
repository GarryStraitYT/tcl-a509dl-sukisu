// SPDX-License-Identifier: GPL-2.0

#include "imgsensor_cfg_table.h"
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/types.h>

#undef CONFIG_MTK_SMI_EXT
#ifdef CONFIG_MTK_SMI_EXT
#include "mmdvfs_mgr.h"
#endif
#ifdef CONFIG_OF
/* device tree */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CCU
#include "ccu_inc.h"
#endif

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_imgsensor_errcode.h"


#include "imgsensor_sensor_list.h"
#include "imgsensor_hw.h"
#include "imgsensor_i2c.h"
#include "imgsensor_proc.h"
#include "imgsensor_clk.h"
#include "imgsensor.h"

#define PDAF_DATA_SIZE 4096

#ifdef CONFIG_MTK_SMI_EXT
static int current_mmsys_clk = MMSYS_CLK_MEDIUM;
#endif

/* Test Only!! Open this define for temperature meter UT */
/* Temperature workqueue */
/* #define CONFIG_CAM_TEMPERATURE_WORKQUEUE */
#ifdef CONFIG_CAM_TEMPERATURE_WORKQUEUE
static void cam_temperature_report_wq_routine(struct work_struct *);
	struct delayed_work cam_temperature_wq;
#endif

#define FEATURE_CONTROL_MAX_DATA_SIZE 128000

struct platform_device *gpimgsensor_hw_platform_device;
struct device *gimgsensor_device;
/* 81 is used for V4L driver */
static struct cdev *gpimgsensor_cdev;
static struct class *gpimgsensor_class;

static DEFINE_MUTEX(gimgsensor_mutex);

struct IMGSENSOR  gimgsensor;
struct IMGSENSOR *pgimgsensor = &gimgsensor;
MUINT32 last_id;

/*prevent imgsensor race condition in vulunerbility test*/
struct mutex imgsensor_mutex;


DEFINE_MUTEX(pinctrl_mutex);
DEFINE_MUTEX(oc_mutex);


#define IMGSENSOR_PROF 1
#if IMGSENSOR_PROF
void IMGSENSOR_PROFILE_INIT(struct timeval *ptv)
{
	do_gettimeofday(ptv);
}

void IMGSENSOR_PROFILE(struct timeval *ptv, char *tag)
{
	struct timeval tv;
	unsigned long  time_interval;

	do_gettimeofday(&tv);
	time_interval =
	    (tv.tv_sec - ptv->tv_sec) * 1000000 + (tv.tv_usec - ptv->tv_usec);

	pr_info("[%s]Profile = %lu us\n", tag, time_interval);
}

#else
void IMGSENSOR_PROFILE_INIT(struct timeval *ptv) {}
void IMGSENSOR_PROFILE(struct timeval *ptv, char *tag) {}
#endif

#define IMGSENSOR_FUNCTION_ENTRY()    /*PK_DBG("[%s]:E\n",__FUNCTION__)*/
#define IMGSENSOR_FUNCTION_EXIT()     /*PK_DBG("[%s]:X\n",__FUNCTION__)*/
struct IMGSENSOR_SENSOR *
imgsensor_sensor_get_inst(enum IMGSENSOR_SENSOR_IDX idx)
{
	if (idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM)
		return NULL;
	else
		return &pgimgsensor->sensor[idx];
}

static void
imgsensor_mutex_init(struct IMGSENSOR_SENSOR_INST *psensor_inst)
{
	mutex_init(&psensor_inst->sensor_mutex);
}

static void imgsensor_mutex_lock(struct IMGSENSOR_SENSOR_INST *psensor_inst)
{
#ifdef IMGSENSOR_LEGACY_COMPAT
	if (psensor_inst->status.arch) {
		mutex_lock(&psensor_inst->sensor_mutex);
	} else {
		mutex_lock(&gimgsensor_mutex);
		imgsensor_i2c_set_device(&psensor_inst->i2c_cfg);
	}
#else
	mutex_lock(&psensor_inst->sensor_mutex);
#endif
}

static void imgsensor_mutex_unlock(struct IMGSENSOR_SENSOR_INST *psensor_inst)
{
#ifdef IMGSENSOR_LEGACY_COMPAT
	if (psensor_inst->status.arch)
		mutex_unlock(&psensor_inst->sensor_mutex);
	else
		mutex_unlock(&gimgsensor_mutex);
#else
	mutex_lock(&psensor_inst->sensor_mutex);
#endif
}

//zhongzhu add start,initialize imgsensor_custom_config
#include "tct_define.h"
static char * power_name[] = {"rst", "pnd", "vcama", "vcama1", "vcamd", "vcamio", "vcamaf", "vcama", "vcama1", "vcamd", "vcamio", "vcamaf", "none",   "en_value", "sel_value"};
static int    power_id[]   = { RST,   PDN,   AVDD,    AVDD1,    DVDD,    DOVDD,    AFVDD,    AVDD,    AVDD1,    DVDD,    DOVDD,    AFVDD,    VDD_None, IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL};

static char * mipi_switch_name[] = {"en_value", "sel_value"};
static int    mipi_switch_id[]   = {IMGSENSOR_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL};

struct IMGSENSOR_HW_POWER_SEQ tct_power_sequence[IMGSENSOR_SENSOR_IDX_MAX_NUM];
struct IMGSENSOR_HW_POWER_SEQ tct_platform_power_sequence[IMGSENSOR_SENSOR_IDX_MAX_NUM];
char sensor_name[IMGSENSOR_SENSOR_IDX_MAX_NUM][64];

static void tctimgsensor_init(void)
{
	#define LENGTH 32
	#define CONFIG_LENGTH 256
	unsigned int i = 0, index = 0,  count = 0, index_mipi = 0, count_mipi = 0, mipi_num_idx = 0;
	int ret = 0;
	const char *hw_cfg;
	char hw_cfg_local[CONFIG_LENGTH];
	char * p_hw_cfg_local = NULL;
	char * token;
	const char s[2] = " ";
	int i2c_device[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {0};
	int j = 0;
	char hw_cfg_name[LENGTH];
	bool mipi_switch_flag = false;			//to sign a mipi_switch_name,if it's a mipi config
	bool mipi_enabled     = false;			//to sign a camera if has mipi switch control
	struct device_node *of_node = of_find_compatible_node(NULL, NULL, "mediatek,camera_hw");

	memset(tct_power_sequence, 0, sizeof(struct IMGSENSOR_HW_POWER_SEQ) * IMGSENSOR_SENSOR_IDX_MAX_NUM);
	memset(tct_platform_power_sequence, 0, sizeof(struct IMGSENSOR_HW_POWER_SEQ) * IMGSENSOR_SENSOR_IDX_MAX_NUM);
	for(i = 0;i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++)
	{
		memcpy(sensor_name[i], "tctcamera_none", 14);
		tct_power_sequence[i].name = sensor_name[i];
	}

	for(i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++)
	{
		memset(hw_cfg_name, 0, LENGTH);
		memset(hw_cfg_local, 0, CONFIG_LENGTH);

		snprintf(hw_cfg_name, LENGTH, "%s%d%s", "cam", i, "_hw_cfg");
		ret = of_property_read_string(of_node, hw_cfg_name, &hw_cfg);
		if (ret < 0) {
			pr_err("TCAM: Property %s not defined\n", hw_cfg_name);
		}else {
			pr_err("TCAM: %s: %s\n", hw_cfg_name, hw_cfg);

			memset(imgsensor_custom_config[i].pwr_info, 0, sizeof(struct IMGSENSOR_HW_CUSTOM_POWER_INFO) * IMGSENSOR_HW_POWER_INFO_MAX);
			memcpy(hw_cfg_local, hw_cfg, strlen(hw_cfg) + 1);
			p_hw_cfg_local = hw_cfg_local;

			count = 0;
			count_mipi = 0;
			mipi_enabled = false;
			token = strsep(&p_hw_cfg_local, s);
			if(!strcmp(token, "i2c"))
			{
				token = strsep(&p_hw_cfg_local, s);
				kstrtoul(token, 10, (unsigned long int*)&i2c_device[i]);

				if(i <= IMGSENSOR_SENSOR_IDX_MAIN2)
					imgsensor_custom_config[i].i2c_dev = i;				
				else{
					for(j = 0; j <= IMGSENSOR_SENSOR_IDX_MAIN2; j++){
						if(i2c_device[i] == i2c_device[j]){
							imgsensor_custom_config[i].i2c_dev = imgsensor_custom_config[j].i2c_dev;
							break;
						}
					}
				}

//				kstrtoul(token, 10, (unsigned long int*)&imgsensor_custom_config[i].i2c_dev);
			}

			token = strsep(&p_hw_cfg_local, s);
			if(!strcmp(token, "mclk"))
			{
				token = strsep(&p_hw_cfg_local, s);
				imgsensor_custom_config[i].pwr_info[count].pin = IMGSENSOR_HW_PIN_MCLK;
				imgsensor_custom_config[i].pwr_info[count].id  = IMGSENSOR_HW_ID_MCLK;
				count++;
			}

			token = strsep(&p_hw_cfg_local, s);
			while (token != NULL) {
				mipi_switch_flag = false;
				for(index = 0; index <sizeof(power_name)/sizeof(*power_name); index++)
				{
					if(!strcmp(token ,power_name[index]))
						break;
				}
				if(index >= sizeof(power_name)/sizeof(*power_name)){
					token = strsep(&p_hw_cfg_local, s);					//just need  a strsep to match the relation in pair
					break;
				}
				imgsensor_custom_config[i].pwr_info[count].pin = power_id[index];

				for(index_mipi = 0; index_mipi <sizeof(mipi_switch_name)/sizeof(*mipi_switch_name); index_mipi++)
				{
					if(!strcmp(token ,mipi_switch_name[index_mipi]))
					{
						mipi_switch_flag = true;
						mipi_enabled     = true;
						break;
					}
				}

				token = strsep(&p_hw_cfg_local, s);
				if(!strcmp(token ,"gpio"))
					imgsensor_custom_config[i].pwr_info[count].id = IMGSENSOR_HW_ID_GPIO;
				else if (!strcmp(token ,"pmic"))
					imgsensor_custom_config[i].pwr_info[count].id = IMGSENSOR_HW_ID_REGULATOR;
				else if (!strcmp(token ,"none"))
					imgsensor_custom_config[i].pwr_info[count].id = IMGSENSOR_HW_ID_NONE;
				else
					imgsensor_custom_config[i].pwr_info[count].id = IMGSENSOR_HW_ID_GPIO;

				if(mipi_switch_flag){
					unsigned long int value = 0;
					kstrtoul(token, 10, &value);
					tct_platform_power_sequence[mipi_num_idx]._idx = i;
					tct_platform_power_sequence[mipi_num_idx].name = PLATFORM_POWER_SEQ_NAME;
					tct_platform_power_sequence[mipi_num_idx].pwr_info[count_mipi].pin           = mipi_switch_id[index_mipi];
					tct_platform_power_sequence[mipi_num_idx].pwr_info[count_mipi].pin_state_on  = value?Vol_High:Vol_Low;
					tct_platform_power_sequence[mipi_num_idx].pwr_info[count_mipi].pin_on_delay  = 0;
					tct_platform_power_sequence[mipi_num_idx].pwr_info[count_mipi].pin_state_off = value?Vol_Low:Vol_High;
					tct_platform_power_sequence[mipi_num_idx].pwr_info[count_mipi].pin_off_delay = 0;
					count_mipi++;
				}

				count++;
				token = strsep(&p_hw_cfg_local, s);
			}
			if(mipi_enabled)
				mipi_num_idx++;
		}
	}
}

static enum IMGSENSOR_RETURN tct_imgsensor_hw_power(enum IMGSENSOR_SENSOR_IDX sensor_idx, struct IMGSENSOR_HW *phw, enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	int ret = 0;
	char *curr_sensor_name = sensor_name[sensor_idx];
	char str_index[LENGTH_FOR_SNPRINTF];

	pr_err("sensor_idx %d, power %d curr_sensor_name %s, enable list %s\n",
		sensor_idx,
		pwr_status,
		curr_sensor_name,
		phw->enable_sensor_by_index[(uint32_t)sensor_idx] == NULL
		? "NULL"
		: phw->enable_sensor_by_index[(uint32_t)sensor_idx]);

	if (phw->enable_sensor_by_index[(uint32_t)sensor_idx] &&
	!strstr(phw->enable_sensor_by_index[(uint32_t)sensor_idx], curr_sensor_name))
		return IMGSENSOR_RETURN_ERROR;

	ret = snprintf(str_index, sizeof(str_index), "%d", sensor_idx);
	if (ret < 0) {
		pr_err("Error! snprintf allocate 0");
		ret = IMGSENSOR_RETURN_ERROR;
		return ret;
	}

	imgsensor_hw_power_sequence(phw, sensor_idx, pwr_status, tct_platform_power_sequence, IMGSENSOR_HW_SENSOR_MAX_NUM, str_index);
	imgsensor_hw_power_sequence(phw, sensor_idx, pwr_status, tct_power_sequence, IMGSENSOR_HW_SENSOR_MAX_NUM, curr_sensor_name);

	return IMGSENSOR_RETURN_SUCCESS;
}

extern void kdSetI2CSpeed(u16 i2cSpeed);
extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iReadRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId, u16 timing);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId, u16 timing);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);
extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length, u16 timing);
extern int eeprom_iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 timing);

#ifdef CONFIG_TCT_DEVICEINFO
extern char CamNameB_module_name[256];
extern char CamNameF_module_name[256];
extern char CamNameB2_module_name[256];
extern char CamNameB3_module_name[256];
extern char CamNameB4_module_name[256];
extern char CamOTPF_module_name[256];
extern char CamOTPB_module_name[256];
extern char CamOTPB2_module_name[256];
extern char CamOTPB3_module_name[256];
extern char CamOTPB4_module_name[256];
#endif

MUINT8      pu_send_cmd[4096];

static inline int TCT_CAMERA_HW_Control(void *pBuf)
{
	int i4RetValue = 0;
	signed int ret = 0;
	MUINT32                            InvokeCamera;
	TCT_SENSOR_FEATURE_ENUM            FeatureId;
	TCT_SENSOR_CONTROL_STRUCT          *pSensorCtrl;
	struct IMGSENSOR                   *pimgsensor   = &gimgsensor;

	struct IMGSENSOR_SENSOR *psensor;
	struct IMGSENSOR_SENSOR_INST       *psensor_inst;
	char * pmtk_ccm_name = NULL;

	struct ccu_sensor_info ccu_info;
	struct i2c_client *pi2c_client = NULL;

	pSensorCtrl  = (TCT_SENSOR_CONTROL_STRUCT *) pBuf;
	FeatureId    = pSensorCtrl->FeatureId;

	memset(pu_send_cmd,0,sizeof(pu_send_cmd));

	pr_err("%s, FeatureId = %d", __func__, FeatureId);

	switch(FeatureId){
		case TCT_SENSOR_FEATURE_POWER_INIT_POWER_SEQ:
			InvokeCamera = pSensorCtrl->InvokeCamera;
			memset(&tct_power_sequence[InvokeCamera], 0, sizeof(struct IMGSENSOR_HW_POWER_SEQ));
			memset(sensor_name[InvokeCamera],         0, sizeof(MUINT8) * 64);
			if(copy_from_user((void*)(tct_power_sequence[InvokeCamera].pwr_info), (void __user *)(pSensorCtrl->pFeaturePara1), sizeof(struct IMGSENSOR_HW_POWER_INFO) * IMGSENSOR_HW_POWER_INFO_MAX))
			{
				pr_err("[tct_power_sequence] ioctl copy from user failed\n");
				return -EFAULT;
			}
			if(copy_from_user((void*)(sensor_name[InvokeCamera]), (void __user *)(pSensorCtrl->pFeaturePara2), sizeof(MUINT8) * 64))
			{
				pr_err("[sensor_name] ioctl copy from user failed\n");
				return -EFAULT;
			}
			tct_power_sequence[InvokeCamera].name = sensor_name[InvokeCamera];
			tct_power_sequence[InvokeCamera]._idx = InvokeCamera;
			break;

		case TCT_SENSOR_FEATURE_POWER_OFF:
			InvokeCamera = pSensorCtrl->InvokeCamera;
			i4RetValue = tct_imgsensor_hw_power(InvokeCamera, &pimgsensor->hw, IMGSENSOR_HW_POWER_STATUS_OFF);
			break;

		case TCT_SENSOR_FEATURE_POWER_ON:
			InvokeCamera = pSensorCtrl->InvokeCamera;
			i4RetValue = tct_imgsensor_hw_power(InvokeCamera, &pimgsensor->hw, IMGSENSOR_HW_POWER_STATUS_ON);
			break;

//		case TCT_SENSOR_FEATURE_HW_DUMP:
//			i4RetValue = imgsensor_hw_dump(&gimgsensor.hw);
//			break;

		case TCT_SENSOR_FEATURE_MCLK_SET_DRIVER_CURRENT:
			InvokeCamera = pSensorCtrl->InvokeCamera;
			if (gimgsensor.mclk_set_drive_current != NULL)
				i4RetValue = gimgsensor.mclk_set_drive_current(gimgsensor.hw.pdev[IMGSENSOR_HW_ID_MCLK]->pinstance, InvokeCamera, pSensorCtrl->len1);
			else
				pr_err("%s, set drive current by pinctrl was not supported\n", __func__);
			break;


		case TCT_SENSOR_FEATURE_OC_IRQ_ENABLE:
			InvokeCamera = pSensorCtrl->InvokeCamera;
			psensor = imgsensor_sensor_get_inst(InvokeCamera);
			psensor_inst = &psensor->inst;			
			if (gimgsensor.imgsensor_oc_irq_enable != NULL)
				i4RetValue = gimgsensor.imgsensor_oc_irq_enable(psensor_inst->sensor_idx, (bool)pSensorCtrl->para);
			else
				pr_err("%s, imgsensor_oc_irq_enable was not supported\n", __func__);
			break;
#ifdef CONFIG_MTK_CCU
		case TCT_SENSOR_FEATURE_CCU_SET_SENSOR_INFO:
			InvokeCamera = pSensorCtrl->InvokeCamera;
			psensor = imgsensor_sensor_get_inst(InvokeCamera);
			psensor_inst = &psensor->inst;

			ccu_info.slave_addr = (psensor_inst->i2c_cfg.pinst->msg->addr << 1);
			ccu_info.sensor_name_string = (char *)sensor_name[InvokeCamera];
			pi2c_client = psensor_inst->i2c_cfg.pinst->pi2c_client;
			if (pi2c_client)
				ccu_info.i2c_id = (((struct mt_i2c *) i2c_get_adapdata(pi2c_client->adapter))->id);
			else
				ccu_info.i2c_id = -1;
			ccu_set_sensor_info(InvokeCamera, &ccu_info);
			break;

		case TCT_SENSOR_FEATURE_CCU_SET_CURRENT_FPS:
			ccu_set_current_fps(pSensorCtrl->len1);
			break;
#endif
		case TCT_SENSOR_FEATURE_ADD_CAMERA_INFO:
			if(pSensorCtrl->InvokeCamera == 0)
				memset(mtk_ccm_name, 0, camera_info_size);
			pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
			if(copy_from_user((void*)(pmtk_ccm_name), (void __user *)(pSensorCtrl->pdata1), pSensorCtrl->len1))
			{
				pr_err("[pmtk_ccm_name] ioctl copy from user failed\n");
				return -EFAULT;
			}
			break;

#ifdef CONFIG_TCT_DEVICEINFO
		case TCT_SENSOR_FEATURE_SET_OTP_STATUS:
			switch(pSensorCtrl->InvokeCamera){
				case IMGSENSOR_SENSOR_IDX_MAIN:
					memset(CamOTPB_module_name, 0, sizeof(CamOTPB_module_name));
					if(copy_from_user((void*)CamOTPB_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamOTPB_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_SUB:
					memset(CamOTPF_module_name, 0, sizeof(CamOTPF_module_name));
					if(copy_from_user((void*)CamOTPF_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamOTPF_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN2:
					memset(CamOTPB2_module_name, 0, sizeof(CamOTPB2_module_name));
					if(copy_from_user((void*)CamOTPB2_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamOTPB2_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_SUB2:
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN3:
					memset(CamOTPB3_module_name, 0, sizeof(CamOTPB3_module_name));
					if(copy_from_user((void*)CamOTPB3_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamOTPB3_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_SUB3:
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN4:
					memset(CamOTPB4_module_name, 0, sizeof(CamOTPB4_module_name));
					if(copy_from_user((void*)CamOTPB4_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamOTPB4_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_SUB4:
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN5:
					break;
				case IMGSENSOR_SENSOR_IDX_SUB5:
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN6:
					break;
				case IMGSENSOR_SENSOR_IDX_SUB6:
					break;
				default:
					break;
			}
			break;

		case TCT_SENSOR_FEATURE_SET_OTP_NAME:
			switch(pSensorCtrl->InvokeCamera){
				case IMGSENSOR_SENSOR_IDX_MAIN:
					memset(CamNameB_module_name, 0, sizeof(CamNameB_module_name));
					if(copy_from_user((void*)CamNameB_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamNameB_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_SUB:
					memset(CamNameF_module_name, 0, sizeof(CamNameF_module_name));
					if(copy_from_user((void*)CamNameF_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamNameF_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN2:
					memset(CamNameB2_module_name, 0, sizeof(CamNameB2_module_name));
					if(copy_from_user((void*)CamNameB2_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamNameB2_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_SUB2:
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN3:
					memset(CamNameB3_module_name, 0, sizeof(CamNameB3_module_name));
					if(copy_from_user((void*)CamNameB3_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamNameB3_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_SUB3:
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN4:
					memset(CamNameB4_module_name, 0, sizeof(CamNameB4_module_name));
					if(copy_from_user((void*)CamNameB4_module_name, (void __user *)pSensorCtrl->pdata1, pSensorCtrl->len1))
					{
						pr_err("[CamNameB4_module_name] ioctl copy from user failed\n");
						return -EFAULT;
					}
					break;
				case IMGSENSOR_SENSOR_IDX_SUB4:
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN5:
					break;
				case IMGSENSOR_SENSOR_IDX_SUB5:
					break;
				case IMGSENSOR_SENSOR_IDX_MAIN6:
					break;
				case IMGSENSOR_SENSOR_IDX_SUB6:
					break;
				default:
					break;
			}
			break;
#endif
		case TCT_SENSOR_FEATURE_I2C_SET_SPEED:
			kdSetI2CSpeed(pSensorCtrl->para);
			break;

		case TCT_SENSOR_FEATURE_I2C_READ_REG:
			if(copy_from_user((void*)pu_send_cmd, (void __user *)(pSensorCtrl->pdata1), pSensorCtrl->len1))
			{
				pr_err("[TCT_SENSOR_FEATURE_I2C_READ_REG pu_send_cmd] ioctl copy from user failed\n");
				return -EFAULT;
			}
			i4RetValue = iReadRegI2C(pu_send_cmd, pSensorCtrl->len1, (u8 *)pu_send_cmd, pSensorCtrl->len2, pSensorCtrl->para);
			if(copy_to_user((void __user *)(pSensorCtrl->pdata2), (void*)pu_send_cmd, pSensorCtrl->len2))
			{
				pr_err("[TCT_SENSOR_FEATURE_I2C_READ_REG pu_send_cmd] ioctl copy to user failed\n");
				return -EFAULT;
			}
			break;

		case TCT_SENSOR_FEATURE_I2C_READ_REG_TIMING:
			if(copy_from_user((void*)pu_send_cmd, (void __user *)(pSensorCtrl->pdata1), pSensorCtrl->len1))
			{
				pr_err("[TCT_SENSOR_FEATURE_I2C_READ_REG_TIMING pu_send_cmd] ioctl copy from user failed\n");
				return -EFAULT;
			}
			i4RetValue = iReadRegI2CTiming(pu_send_cmd, pSensorCtrl->len1, (u8 *)pu_send_cmd, pSensorCtrl->len2, pSensorCtrl->para, pSensorCtrl->timing);
			if(copy_to_user((void __user *)(pSensorCtrl->pdata2), (void*)pu_send_cmd, pSensorCtrl->len2))
			{
				pr_err("[TCT_SENSOR_FEATURE_I2C_READ_REG_TIMING pu_send_cmd] ioctl copy to user failed\n");
				return -EFAULT;
			}
			break;

		case TCT_SENSOR_FEATURE_I2C_WRITE_REG:
			if(copy_from_user((void*)pu_send_cmd, (void __user *)(pSensorCtrl->pdata1), pSensorCtrl->len1))
			{
				pr_err("[TCT_SENSOR_FEATURE_I2C_WRITE_REG pu_send_cmd] ioctl copy from user failed\n");
				return -EFAULT;
			}
			i4RetValue = iWriteRegI2C(pu_send_cmd, pSensorCtrl->len1, pSensorCtrl->para);
			break;

		case TCT_SENSOR_FEATURE_I2C_WRITE_REG_TIMING:
			if(copy_from_user((void*)pu_send_cmd, (void __user *)(pSensorCtrl->pdata1), pSensorCtrl->len1))
			{
				pr_err("[TCT_SENSOR_FEATURE_I2C_WRITE_REG_TIMING pu_send_cmd] ioctl copy from user failed\n");
				return -EFAULT;
			}
			i4RetValue = iWriteRegI2CTiming(pu_send_cmd, pSensorCtrl->len1, pSensorCtrl->para, pSensorCtrl->timing);
			break;

		case TCT_SENSOR_FEATURE_I2C_BURST_WRITE_REG:
			if(copy_from_user((void*)pu_send_cmd, (void __user *)(pSensorCtrl->pdata1), pSensorCtrl->len1))
			{
				pr_err("[TCT_SENSOR_FEATURE_I2C_BURST_WRITE_REG pu_send_cmd] ioctl copy from user failed\n");
				return -EFAULT;
			}
			i4RetValue = iBurstWriteReg(pu_send_cmd, pSensorCtrl->len1, pSensorCtrl->para);
			break;

		case TCT_SENSOR_FEATURE_I2C_BURST_WRITE_REG_MULTI:
			if(copy_from_user((void*)pu_send_cmd, (void __user *)(pSensorCtrl->pdata1), pSensorCtrl->len1))
			{
				pr_err("[TCT_SENSOR_FEATURE_I2C_BURST_WRITE_REG_MULTI pu_send_cmd] ioctl copy from user failed\n");
				return -EFAULT;
			}
			i4RetValue = iBurstWriteReg_multi(pu_send_cmd, pSensorCtrl->len1, pSensorCtrl->para, pSensorCtrl->para_ext, pSensorCtrl->timing);
			break;

		case TCT_SENSOR_FEATURE_EEPROM_I2C_BURST_WRITE_REG_MULTI:
			if(copy_from_user((void*)pu_send_cmd, (void __user *)(pSensorCtrl->pdata1), pSensorCtrl->len1))
			{
				pr_err("[TCT_SENSOR_FEATURE_EEPROM_I2C_BURST_WRITE_REG_MULTI pu_send_cmd] ioctl copy from user failed\n");
				return -EFAULT;
			}
			i4RetValue = eeprom_iBurstWriteReg_multi(pu_send_cmd, pSensorCtrl->len1, pSensorCtrl->para, pSensorCtrl->timing);
			break;

		case TCT_SENSOR_FEATURE_I2C_INIT:
			InvokeCamera = pSensorCtrl->InvokeCamera;
			psensor = imgsensor_sensor_get_inst(InvokeCamera);
			psensor_inst = &psensor->inst;
			i4RetValue = imgsensor_i2c_init(&psensor_inst->i2c_cfg, imgsensor_custom_config[(unsigned int)InvokeCamera].i2c_dev);
			pr_err("tctcamera i2c init !!! sensor_idx:%d, i2c:%d", InvokeCamera, imgsensor_custom_config[(unsigned int)InvokeCamera].i2c_dev);
			break;

		case TCT_SENSOR_FEATURE_I2C_SET:
			if(pSensorCtrl->para == 0)
				imgsensor_i2c_set_device(NULL);
			else{
				InvokeCamera = pSensorCtrl->InvokeCamera;
				psensor = imgsensor_sensor_get_inst(InvokeCamera);
				psensor_inst = &psensor->inst;
				imgsensor_i2c_set_device(&psensor_inst->i2c_cfg);
			}
			break;

		case TCT_SENSOR_FEATURE_I2C_CONFIG:
			InvokeCamera = pSensorCtrl->InvokeCamera;
			psensor = imgsensor_sensor_get_inst(InvokeCamera);
			psensor_inst = &psensor->inst;
			imgsensor_i2c_filter_msg(&psensor_inst->i2c_cfg, pSensorCtrl->para == 1);
			break;

		case TCT_SENSOR_FEATURE_I2C_MODE_ENABLE:
			ret = imgsensor_i2c_buffer_mode(pSensorCtrl->para);
			pSensorCtrl->para = ret;
			break;

		default:
			break;
	}

	return i4RetValue;
}
//zhongzhu add end


MINT32 imgsensor_sensor_open(struct IMGSENSOR_SENSOR *psensor)
{
	MINT32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;
	struct SENSOR_FUNCTION_STRUCT *psensor_func =  psensor->pfunc;

#ifdef CONFIG_MTK_CCU
	struct ccu_sensor_info ccuSensorInfo;
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;
	struct i2c_client *pi2c_client = NULL;
#endif

	IMGSENSOR_FUNCTION_ENTRY();

	if (psensor_func &&
	    psensor_func->SensorOpen &&
	    psensor_inst) {

		/* turn on power */
		IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);
		if (pgimgsensor->imgsensor_oc_irq_enable != NULL)
			pgimgsensor->imgsensor_oc_irq_enable(
					psensor->inst.sensor_idx, false);
#ifndef TCT_CAM_DRIVER_SUPPORT
		ret = imgsensor_hw_power(&pgimgsensor->hw,
		    psensor,
		    psensor_inst->psensor_name,
		    IMGSENSOR_HW_POWER_STATUS_ON);
#else
                ret = imgsensor_hw_myPowers(&pgimgsensor->hw, psensor, IMGSENSOR_HW_POWER_STATUS_ON,psensor_func->camera_power_info);
#endif
		if (ret != IMGSENSOR_RETURN_SUCCESS) {
			PK_DBG("[%s]", __func__);
			return -EIO;
		}
		/* wait for power stable */
		mDELAY(5);

		IMGSENSOR_PROFILE(&psensor_inst->profile_time,
		    "kdCISModulePowerOn");

		imgsensor_mutex_lock(psensor_inst);

		psensor_func->psensor_inst = psensor_inst;
		ret = psensor_func->SensorOpen();
		if (ret != ERROR_NONE) {
#ifndef TCT_CAM_DRIVER_SUPPORT
			imgsensor_hw_power(&pgimgsensor->hw,
			    psensor,
			    psensor_inst->psensor_name,
			    IMGSENSOR_HW_POWER_STATUS_OFF);
#else
                ret = imgsensor_hw_myPowers(&pgimgsensor->hw, psensor, IMGSENSOR_HW_POWER_STATUS_OFF,psensor_func->camera_power_info);
#endif
			PK_DBG("SensorOpen fail");
		} else {
			psensor_inst->state = IMGSENSOR_STATE_OPEN;
#ifdef CONFIG_MTK_CCU
			ccuSensorInfo.slave_addr =
			    (psensor_inst->i2c_cfg.pinst->msg->addr << 1);

			ccuSensorInfo.sensor_name_string =
			    (char *)(psensor_inst->psensor_name);

			pi2c_client = psensor_inst->i2c_cfg.pinst->pi2c_client;
			if (pi2c_client)
				ccuSensorInfo.i2c_id =
					(((struct mt_i2c *) i2c_get_adapdata(
						pi2c_client->adapter))->id);
			else
				ccuSensorInfo.i2c_id = -1;

			ccu_set_sensor_info(sensor_idx, &ccuSensorInfo);
#endif
			if (pgimgsensor->imgsensor_oc_irq_enable != NULL)
				pgimgsensor->imgsensor_oc_irq_enable(
						psensor->inst.sensor_idx, true);

		}

		imgsensor_mutex_unlock(psensor_inst);

		IMGSENSOR_PROFILE(&psensor_inst->profile_time, "SensorOpen");
	}

	IMGSENSOR_FUNCTION_EXIT();

	return ret ? -EIO : ret;
}

MUINT32
imgsensor_sensor_get_info(
	struct IMGSENSOR_SENSOR *psensor,
	MUINT32 ScenarioId,
	MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	MUINT32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;
	struct SENSOR_FUNCTION_STRUCT *psensor_func =  psensor->pfunc;

	IMGSENSOR_FUNCTION_ENTRY();

	if (psensor_func &&
	    psensor_func->SensorGetInfo &&
	    psensor_inst &&
	    pSensorInfo &&
	    pSensorConfigData) {

		imgsensor_mutex_lock(psensor_inst);

		psensor_func->psensor_inst = psensor_inst;

		ret = psensor_func->SensorGetInfo(
		    (enum MSDK_SCENARIO_ID_ENUM)(ScenarioId),
		    pSensorInfo,
		    pSensorConfigData);

		if (ret != ERROR_NONE)
			PK_DBG(" [%s]error : %d\n", __func__, ret);

		imgsensor_mutex_unlock(psensor_inst);
	}

	IMGSENSOR_FUNCTION_EXIT();

	return ret;
}

MUINT32
imgsensor_sensor_get_resolution(
	struct IMGSENSOR_SENSOR *psensor,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	MUINT32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;
	struct SENSOR_FUNCTION_STRUCT *psensor_func =  psensor->pfunc;

	IMGSENSOR_FUNCTION_ENTRY();

	if (psensor_func &&
	    psensor_func->SensorGetResolution &&
	    psensor_inst) {

		imgsensor_mutex_lock(psensor_inst);

		psensor_func->psensor_inst = psensor_inst;

		ret = psensor_func->SensorGetResolution(pSensorResolution);
		if (ret != ERROR_NONE)
			PK_DBG(" [%s]error : %d\n", __func__, ret);

		imgsensor_mutex_unlock(psensor_inst);
	}

	IMGSENSOR_FUNCTION_EXIT();

	return ret;
}

MUINT32
imgsensor_sensor_feature_control(
	struct IMGSENSOR_SENSOR *psensor,
	MSDK_SENSOR_FEATURE_ENUM FeatureId,
	MUINT8 *pFeaturePara,
	MUINT32 *pFeatureParaLen)
{
	MUINT32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST  *psensor_inst = &psensor->inst;
	struct SENSOR_FUNCTION_STRUCT *psensor_func =  psensor->pfunc;

	IMGSENSOR_FUNCTION_ENTRY();

	if (psensor_func &&
	    psensor_func->SensorFeatureControl &&
	    psensor_inst) {

		imgsensor_mutex_lock(psensor_inst);

		psensor_func->psensor_inst = psensor_inst;

		ret = psensor_func->SensorFeatureControl(
		    FeatureId,
		    pFeaturePara,
		    pFeatureParaLen);

		if (ret != ERROR_NONE)
			PK_DBG("[%s] %d\n", __func__, ret);

		imgsensor_mutex_unlock(psensor_inst);
	}

	IMGSENSOR_FUNCTION_EXIT();

	return ret;
}

MUINT32
imgsensor_sensor_control(
	struct IMGSENSOR_SENSOR *psensor,
	enum MSDK_SCENARIO_ID_ENUM ScenarioId)
{
	MUINT32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;
	struct SENSOR_FUNCTION_STRUCT *psensor_func =  psensor->pfunc;

	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT image_window;
	MSDK_SENSOR_CONFIG_STRUCT sensor_config_data;

	IMGSENSOR_FUNCTION_ENTRY();

	if (psensor_func &&
	    psensor_func->SensorControl &&
	    psensor_inst) {

		IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

		imgsensor_mutex_lock(psensor_inst);

		psensor_func->psensor_inst = psensor_inst;
		psensor_func->ScenarioId = ScenarioId;

		ret = psensor_func->SensorControl(ScenarioId,
		    &image_window,
		    &sensor_config_data);

		if (ret != ERROR_NONE)
			PK_DBG(" [%s]error : %d\n", __func__, ret);

		imgsensor_mutex_unlock(psensor_inst);

		IMGSENSOR_PROFILE(
		    &psensor_inst->profile_time,
		    "SensorControl");
	}

	IMGSENSOR_FUNCTION_EXIT();

	return ret;
}

MINT32
imgsensor_sensor_close(struct IMGSENSOR_SENSOR *psensor)
{
	MINT32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST  *psensor_inst = &psensor->inst;
	struct SENSOR_FUNCTION_STRUCT *psensor_func =  psensor->pfunc;

	IMGSENSOR_FUNCTION_ENTRY();

	if (psensor_func &&
	    psensor_func->SensorClose &&
	    psensor_inst) {

		imgsensor_mutex_lock(psensor_inst);

		psensor_func->psensor_inst = psensor_inst;

		if (pgimgsensor->imgsensor_oc_irq_enable != NULL)
			pgimgsensor->imgsensor_oc_irq_enable(
					psensor->inst.sensor_idx, false);

		ret = psensor_func->SensorClose();
		if (ret != ERROR_NONE) {
			PK_DBG(" [%s]error : %d\n", __func__, ret);
		} else {
			psensor_inst->state = IMGSENSOR_STATE_CLOSE;
#ifndef TCT_CAM_DRIVER_SUPPORT
			imgsensor_hw_power(&pgimgsensor->hw,
			    psensor,
			    psensor_inst->psensor_name,
			    IMGSENSOR_HW_POWER_STATUS_OFF);
#else
                ret = imgsensor_hw_myPowers(&pgimgsensor->hw, psensor, IMGSENSOR_HW_POWER_STATUS_OFF,psensor_func->camera_power_info);
#endif
		}

		imgsensor_mutex_unlock(psensor_inst);
	}

	IMGSENSOR_FUNCTION_EXIT();

	return ret ? -EIO : ret;
}

/*Begin ersen.shang for [Task][ 11425156][cruze/cruze pro camera bring up] 202108*/
#ifdef CONFIG_TCT_DEVICEINFO
extern char CamNameB_module_name[256];
extern char CamNameF_module_name[256];
extern char CamNameB2_module_name[256];
extern char CamNameB3_module_name[256];
extern char CamNameB4_module_name[256];
extern char CamOTPF_module_name[256];
extern char CamOTPB_module_name[256];
extern char CamOTPB2_module_name[256];
extern char CamOTPB3_module_name[256];
extern char CamOTPB4_module_name[256];
#else
char CamNameB_module_name[256];
char CamNameF_module_name[256];
char CamNameB2_module_name[256];
char CamNameB3_module_name[256];
char CamNameB4_module_name[256];
char CamOTPF_module_name[256];
char CamOTPB_module_name[256];
char CamOTPB2_module_name[256];
char CamOTPB3_module_name[256];
char CamOTPB4_module_name[256];
#endif
#ifndef TCT_CAM_DRIVER_SUPPORT
static void set_camera_name(MUINT32 sensor_id)
{
	switch(sensor_id)
	{
		/*main*/
		case TSPPHCP1088_HI1336_SENSOR_ID:
			strcpy(CamNameB_module_name, "HI1336:TSP:13M:ASA1300092C1");
			break;
		case TSPPSNP1082_S5KJN1_SENSOR_ID:
			strcpy(CamNameB_module_name, "S5KJN1:TSP:50M:ASA5000000C1");
			break;
                case SHNBA815M_GC08A3_SENSOR_ID:
			strcpy(CamNameB_module_name, "GC08A3:SHINE:8M:ASA8001127C1");
			break;
		case SHNDS41092_OV13B10_SENSOR_ID:
			strcpy(CamNameB_module_name, "OV13B10:SHN:13M:ASA1300096C1");
			break;
		case SHNBAD17B_OV13B10_SENSOR_ID:
			strcpy(CamNameB_module_name, "ov13b10:SHINE:13M:ASA1300096C1");
			break;
                case TSPPH8A1327_HI846_SENSOR_ID:
			strcpy(CamNameB_module_name, "HI846:TSP:8M:ASA8001128C1");
			break;
        case TSPPH8A1408_HI846_SENSOR_ID:
			strcpy(CamNameB_module_name, "HI846:TSP:8M:ASA8001132C1");
			break;
		case TCL0003BA_GC08A3_SENSOR_ID:
			strcpy(CamNameB_module_name, "GC08A3:UMG:8M:AWC2718M12C1");
			break;
                case R_CXT_GC5035_SENSOR_ID:
			strcpy(CamNameB_module_name, "GC5035:CXT:5M:ASA5001253C1");
			break;
                case R_SHINETECH_GC5035_SENSOR_ID:
			strcpy(CamNameB_module_name, "GC5035:SHINETECH:5M:ASA5001226C1");
			break;
                case R_EWELLY_HI556_SENSOR_ID:
			strcpy(CamNameB_module_name, "HI556:EWELLY:5M:ASA5001194C1");
			break;
                case GC5025HMIPI_SENSOR_ID:
			strcpy(CamNameB_module_name, "GC5025:CXT:5M:ASA5001251C1");
			break;
		case TSPPSNP1269_S5KJN1_SENSOR_ID:
			strcpy(CamNameB_module_name, "S5KJN1:TSP:50M:ASA5000001C1");
			break;

		/*front*/
		case SHN9F511B_GC5035_SENSOR_ID:
			strcpy(CamNameF_module_name, "GC5035:SHN:5M:ASA5001197C1");
			break;
		case EWYPC5319_HI556_SENSOR_ID:
			strcpy(CamNameF_module_name, "HI556:EWY:5M:ASA5001223C1");
			break;
		case SHNBF821B_GC08A3_SENSOR_ID:
			strcpy(CamNameF_module_name, "GC08A3:SHINE:8M:ASA8001129C1");
			break;
		case SHNBF811B_GC08A3_SENSOR_ID:
			strcpy(CamNameF_module_name, "GC08A3:SHINE:8M:ASA8001129C1");
			break;
		case TSPS8F9074_S5K4H7_SENSOR_ID:
			strcpy(CamNameF_module_name, "S5K4H7:EWY:8M:ASA8001090C1");
			break;
                case F_SHINETECH_GC5035_SENSOR_ID:
			strcpy(CamNameF_module_name, "GC5035:SHINETECH:5M:ASA5001216C1");
                        break;
                case F_EWELLY_HI556_SENSOR_ID:
			strcpy(CamNameF_module_name, "HI556:EWELLY:5M:ASA5001222C1");
			break;
                case F_SHINETECH_OV5670_SENSOR_ID:
			strcpy(CamNameF_module_name, "GC5670:SHINETECH:5M:ASA5001247C1");
                        break;
		case TSPSCF0002_S5K3L6_SENSOR_ID:
			strcpy(CamNameF_module_name, "S5K3L6:TSP:13M:ASA1300083C1");
			break;
                /*Begin majinrui for [Task: 11669645 ] AustinTF bring up the second camera on 20211112*/
		case SHNBMD28B_OV13B10_SENSOR_ID:
			strcpy(CamNameF_module_name, "OV13B10:SHN:13M:ASA1300099C1");
			break;
                /*End majinrui for [Task: 11669645 ] AustinTF bring up the second camera on 20211112*/

		case TSPPHCF1318_HI1336_SENSOR_ID:
			strcpy(CamNameB_module_name, "HI1336:TSP:13M:ASA1300098C1");
			break;
		case TSPPS5F2052_S5K5E9_SENSOR_ID:
			strcpy(CamNameF_module_name, "S5K5E9:TSP:5M:ASA5001262C1");
			break;
		/*depth*/
		case CXTVC12511_GC2905_SENSOR_ID:
			strcpy(CamNameB2_module_name,  "GC2905:SHN:2M:ASA2001238C1");
			break;
                case CXTVC12499_GC02M1B_SENSOR_ID:
			strcpy(CamNameB2_module_name,  "GC02M1B:CXT:2M:ASA2001236C1");
			break;

		/*macro*/
		case SHNBF204C_GC02M1_SENSOR_ID:
			strcpy(CamNameB3_module_name, "GC02M1:TSP:2M:ASA2001237C1");
			break;

		/*wide*/
		case SHNAF538M_GC5035_SENSOR_ID:
			strcpy(CamNameB4_module_name, "GC5035:SHN:5M:ASA5001240C1");
			break;
		default:
			pr_err("[%s](%d) no find this sensor id 0x%x  \n", __func__, __LINE__, sensor_id);
	}
	return;
}
/*End   ersen.shang for [Task][ 11425156][cruze/cruze pro camera bring up] 202108*/
#else
/*Begin junyiliu for [Task][PASSAT-1876][passat MINI otp display] 20211122*/
static void set_otp_info(char *CamName_OTP_name,MUINT32 otpStatus)
{
#ifdef TARGET_BUILD_MMITEST
	if(strcmp(CamName_OTP_name,"NA:NA:NA:NA") == 0)
	{
		sprintf(CamName_OTP_name, "%d", otpStatus);
	}
#else
	sprintf(CamName_OTP_name, "%d", otpStatus);
#endif
}
/*End junyiliu for [Task][PASSAT-1876][passat MINI otp display] 20211122*/
#endif

static inline int imgsensor_check_is_alive(struct IMGSENSOR_SENSOR *psensor)
{
	struct IMGSENSOR_SENSOR_INST  *psensor_inst = &psensor->inst;
	UINT32 err = 0;
	MUINT32 sensorID = 0;
	MUINT32 retLen = sizeof(MUINT32);
#ifndef TCT_CAM_DRIVER_SUPPORT
	IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

	err = imgsensor_hw_power(&pgimgsensor->hw,
				psensor,
				psensor_inst->psensor_name,
				IMGSENSOR_HW_POWER_STATUS_ON);

	if (err == IMGSENSOR_RETURN_SUCCESS)
		imgsensor_sensor_feature_control(
			psensor,
			SENSOR_FEATURE_CHECK_SENSOR_ID,
			(MUINT8 *)&sensorID,
			&retLen);

	if (sensorID == 0 || sensorID == 0xFFFFFFFF) {
		pr_info("Fail to get sensor ID %x\n", sensorID);
		err = ERROR_SENSOR_CONNECT_FAIL;
	} else {
        pr_info(" Sensor found ID = 0x%x\n", sensorID);

		/*Begin ersen.shang for [Task][ 11425156][cruze/cruze pro camera bring up] 202108*/
		if(psensor->inst.sensor_idx==IMGSENSOR_SENSOR_IDX_MAIN){
			sprintf(CamOTPB_module_name,"%d\n",(sensorID & 0xff000000)>>24);
		}else if(psensor->inst.sensor_idx==IMGSENSOR_SENSOR_IDX_SUB){
			sprintf(CamOTPF_module_name,"%d\n",(sensorID & 0xff000000)>>24);
		}else if(psensor->inst.sensor_idx==IMGSENSOR_SENSOR_IDX_MAIN2){
			sprintf(CamOTPB2_module_name,"%d\n",1);
		}else if(psensor->inst.sensor_idx==IMGSENSOR_SENSOR_IDX_SUB2){
			sprintf(CamOTPB3_module_name,"%d\n",1);
		}else if(psensor->inst.sensor_idx==IMGSENSOR_SENSOR_IDX_MAIN3){
			sprintf(CamOTPB4_module_name,"%d\n",1);
		}
		set_camera_name(sensorID&0xFEFFFFFF);
		/*End   ersen.shang for [Task][ 11425156][cruze/cruze pro camera bring up] 202108*/

		err = ERROR_NONE;
	}

	if (err != ERROR_NONE)
		pr_info("ERROR: No imgsensor alive\n");

	imgsensor_hw_power(&pgimgsensor->hw,
	    psensor,
	    psensor_inst->psensor_name,
	    IMGSENSOR_HW_POWER_STATUS_OFF);

	IMGSENSOR_PROFILE(&psensor_inst->profile_time, "CheckIsAlive");
#else
    struct SENSOR_FUNCTION_STRUCT *psensor_func = psensor->pfunc;
    int otpStatus = 0;
   printk("[heaven]idx: %d, sensnor name: %s\n", psensor->inst.sensor_idx,
         psensor->inst.psensor_name);
  if (psensor->inst.sensor_idx != psensor_func->cameraIndex) {
    printk("[heaven]error : camera index not match !!! %d %d",
           psensor->inst.sensor_idx, psensor_func->cameraIndex);
    return ERROR_SENSOR_CONNECT_FAIL;
  }

  IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

  err = imgsensor_hw_myPowers(&pgimgsensor->hw, psensor,
                         IMGSENSOR_HW_POWER_STATUS_ON,psensor_func->camera_power_info);

  if (err == IMGSENSOR_RETURN_SUCCESS) {
    imgsensor_sensor_feature_control(psensor, SENSOR_FEATURE_CHECK_SENSOR_ID,
                                     (MUINT8 *)&sensorID, &retLen);
  }

  if (sensorID == 0 || sensorID == 0xFFFFFFFF) {
    printk("Sensor %s Fail to get sensor ID %x\n", psensor_inst->psensor_name,sensorID);
    err = ERROR_SENSOR_CONNECT_FAIL;
  } else {
    printk("heaven Sensor %s found ID = 0x%x\n", psensor_inst->psensor_name,sensorID);

    if (sensorID & 0x01000000){
      otpStatus = 1;
    }
    sensorID &= 0xFEFFFFFF;
    if(psensor_func->name==NULL){
     psensor_func->name=psensor_inst->psensor_name;
    }

    if(strlen(psensor_func->name) >256)
    {
       printk("ERROR: psensor_name out of bound\n");
       return ERROR_SENSOR_CONNECT_FAIL;
    }

    if (psensor->inst.sensor_idx == 0) {
      strcpy(CamNameB_module_name, psensor_func->name);
      set_otp_info(CamOTPB_module_name,  otpStatus);
    } else if (psensor->inst.sensor_idx == 1) {
      strcpy(CamNameF_module_name, psensor_func->name);
      set_otp_info(CamOTPF_module_name,  otpStatus);
    } else if (psensor->inst.sensor_idx == 2) {
      strcpy(CamNameB2_module_name, psensor_func->name);
      set_otp_info(CamOTPB2_module_name,  otpStatus);
    } else if (psensor->inst.sensor_idx == 3) {
      strcpy(CamNameB3_module_name, psensor_func->name);
      set_otp_info(CamOTPB3_module_name,  otpStatus);
    } else if (psensor->inst.sensor_idx == 4) {
      strcpy(CamNameB4_module_name, psensor_func->name);
      set_otp_info(CamOTPB4_module_name,  otpStatus);
    } else {
      printk("ERROR: no support other camera index 4\n");
    }
    err = ERROR_NONE;
  }

  if (err != ERROR_NONE)
    PK_DBG("ERROR: No imgsensor alive\n");

  imgsensor_hw_myPowers(&pgimgsensor->hw, psensor,
                         IMGSENSOR_HW_POWER_STATUS_OFF,psensor_func->camera_power_info);

  IMGSENSOR_PROFILE(&psensor_inst->profile_time, "CheckIsAlive");
#endif
	return err ? -EIO:err;
}

int imgsensor_set_driver(struct IMGSENSOR_SENSOR *psensor)
{
	u32 drv_idx = 0;
	int ret = -EIO;

	struct IMGSENSOR_SENSOR_INST    *psensor_inst = &psensor->inst;
	struct IMGSENSOR_INIT_FUNC_LIST *pSensorList  = kdSensorList;
#define TOSTRING(value)           #value
#define STRINGIZE(stringizedName) TOSTRING(stringizedName)

	char *psensor_list_config = NULL, *psensor_list = NULL;
	char *sensor_configs = STRINGIZE(CONFIG_CUSTOM_KERNEL_IMGSENSOR);

	static int orderedSearchList[MAX_NUM_OF_SUPPORT_SENSOR] = {-1};
	static bool get_search_list = true;
	unsigned int i = 0;
	unsigned int j = 0;
	char *driver_name = NULL;
#ifdef TCT_CAM_DRIVER_SUPPORT
get_search_list = false;
#endif
	imgsensor_mutex_init(psensor_inst);
	imgsensor_i2c_init(&psensor_inst->i2c_cfg,
	imgsensor_custom_config[
	(unsigned int)psensor_inst->sensor_idx].i2c_dev);
	imgsensor_i2c_filter_msg(&psensor_inst->i2c_cfg, true);

	if (get_search_list) {
		psensor_list = psensor_list_config =
		    kmalloc(strlen(sensor_configs)-1, GFP_KERNEL);

		if (psensor_list_config) {
			for (j = 0; j < MAX_NUM_OF_SUPPORT_SENSOR; j++)
				orderedSearchList[j] = -1;

			memcpy(psensor_list_config,
			    sensor_configs+1,
			    strlen(sensor_configs)-2);

			*(psensor_list_config+strlen(sensor_configs)-2) = '\0';

			pr_info("sensor_list %s\n", psensor_list_config);
			driver_name = strsep(&psensor_list_config, " \0");

			while (driver_name != NULL) {
				for (j = 0;
				    j < MAX_NUM_OF_SUPPORT_SENSOR;
				    j++) {
					if (pSensorList[j].init == NULL)
						break;
					else if (!strcmp(
						    driver_name,
						    pSensorList[j].name)) {
						orderedSearchList[i++] = j;
						break;
					}
				}
				driver_name =
				    strsep(&psensor_list_config, " \0");
			}
			get_search_list = false;
		}
		kfree(psensor_list);
	}





	for (i = 0; i < MAX_NUM_OF_SUPPORT_SENSOR; i++) {
 #ifndef TCT_CAM_DRIVER_SUPPORT

		if (orderedSearchList[i] == -1)
			continue;
		drv_idx = orderedSearchList[i];
#else
        drv_idx=i;
#endif
		if (pSensorList[drv_idx].init) {
			pSensorList[drv_idx].init(&psensor->pfunc);
			if (psensor->pfunc) {
				/* get sensor name */
				psensor_inst->psensor_name =
				    (char *)pSensorList[drv_idx].name;
#ifdef IMGSENSOR_LEGACY_COMPAT
				psensor_inst->status.arch =
				    psensor->pfunc->arch;
#endif
				if (!imgsensor_check_is_alive(psensor)) {
					pr_info(
					    "[%s]:[%d][%d][%s]\n",
					    __func__,
					    psensor->inst.sensor_idx,
					    drv_idx,
					    psensor_inst->psensor_name);

					ret = drv_idx;
					break;
				}
			} else {
				PK_DBG(
				    "ERROR:NULL g_pInvokeSensorFunc[%d][%d]\n",
				    psensor->inst.sensor_idx,
				    drv_idx);
			}
		} else {
			PK_DBG("ERROR:NULL sensor list[%d]\n", drv_idx);
		}

	}
	imgsensor_i2c_filter_msg(&psensor_inst->i2c_cfg, false);

	return ret;
}

static inline int adopt_CAMERA_HW_GetInfo(void *pBuf)
{
	struct IMGSENSOR_GET_CONFIG_INFO_STRUCT *pSensorGetInfo;
	struct IMGSENSOR_SENSOR *psensor;

	MSDK_SENSOR_INFO_STRUCT *pInfo;
	MSDK_SENSOR_CONFIG_STRUCT *pConfig;
	MUINT32 *pScenarioId;

	pSensorGetInfo = (struct IMGSENSOR_GET_CONFIG_INFO_STRUCT *)pBuf;
	if (pSensorGetInfo == NULL ||
	     pSensorGetInfo->pInfo == NULL ||
	     pSensorGetInfo->pConfig == NULL) {
		PK_DBG("[CAMERA_HW] NULL arg.\n");
		return -EFAULT;
	}

	psensor = imgsensor_sensor_get_inst(pSensorGetInfo->SensorId);
	if (psensor == NULL) {
		PK_DBG("[CAMERA_HW] NULL psensor.\n");
		return -EFAULT;
	}

	pInfo = NULL;
	pConfig =  NULL;
	pScenarioId =  &(pSensorGetInfo->ScenarioId);

	pInfo = kmalloc(sizeof(MSDK_SENSOR_INFO_STRUCT), GFP_KERNEL);
	pConfig = kmalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);

	if (pInfo == NULL || pConfig == NULL) {
		kfree(pInfo);
		kfree(pConfig);
		PK_DBG(" ioctl allocate mem failed\n");
		return -ENOMEM;
	}

	memset(pInfo, 0, sizeof(MSDK_SENSOR_INFO_STRUCT));
	memset(pConfig, 0, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

	imgsensor_sensor_get_info(psensor, *pScenarioId, pInfo, pConfig);

    /* SenorInfo */
	if (copy_to_user((void __user *)(pSensorGetInfo->pInfo),
	    (void *)pInfo,
	    sizeof(MSDK_SENSOR_INFO_STRUCT))) {

		PK_DBG("[CAMERA_HW][info] ioctl copy to user failed\n");
		if (pInfo != NULL)
			kfree(pInfo);
		if (pConfig != NULL)
			kfree(pConfig);
		return -EFAULT;
	}

    /* SensorConfig */
	if (copy_to_user((void __user *) (pSensorGetInfo->pConfig),
	    (void *)pConfig,
	    sizeof(MSDK_SENSOR_CONFIG_STRUCT))) {

		PK_DBG("[CAMERA_HW][config] ioctl copy to user failed\n");

		if (pInfo != NULL)
			kfree(pInfo);
		if (pConfig != NULL)
			kfree(pConfig);
		return -EFAULT;
	}

	kfree(pInfo);
	kfree(pConfig);
	return 0;
}   /* adopt_CAMERA_HW_GetInfo() */

MUINT32 Get_Camera_Temperature(
	enum CAMERA_DUAL_CAMERA_SENSOR_ENUM senDevId,
	MUINT8 *valid,
	MUINT32 *temp)
{
	MUINT32 ret = IMGSENSOR_RETURN_SUCCESS;
	MUINT32 FeatureParaLen = 0;
	struct IMGSENSOR_SENSOR      *psensor =
		imgsensor_sensor_get_inst(IMGSENSOR_SENSOR_IDX_MAP(senDevId));
	struct IMGSENSOR_SENSOR_INST *psensor_inst;

	if (valid == NULL || temp == NULL || psensor == NULL)
		return IMGSENSOR_RETURN_ERROR;

	*valid =
	    SENSOR_TEMPERATURE_NOT_SUPPORT_THERMAL |
	    SENSOR_TEMPERATURE_NOT_POWER_ON;
	*temp  = 0;

	psensor_inst = &psensor->inst;

	FeatureParaLen = sizeof(MUINT32);

	/* In close state the temperature is not valid */
	if (psensor_inst->state != IMGSENSOR_STATE_CLOSE) {
		ret = imgsensor_sensor_feature_control(psensor,
		    SENSOR_FEATURE_GET_TEMPERATURE_VALUE,
		    (MUINT8 *)temp,
		    (MUINT32 *)&FeatureParaLen);

		PK_DBG("senDevId(%d), temperature(%d)\n", senDevId, *temp);

		*valid &= ~SENSOR_TEMPERATURE_NOT_POWER_ON;

		if (*temp != 0) {
			*valid |=  SENSOR_TEMPERATURE_VALID;
			*valid &= ~SENSOR_TEMPERATURE_NOT_SUPPORT_THERMAL;
		}
	}

	return ret;
}
EXPORT_SYMBOL(Get_Camera_Temperature);

#ifdef CONFIG_CAM_TEMPERATURE_WORKQUEUE
static void cam_temperature_report_wq_routine(
	struct work_struct *data)
{
	MUINT8 valid[4] = {0, 0, 0, 0};
	MUINT32 temp[4] = {0, 0, 0, 0};
	MUINT32 ret = 0;

	PK_DBG("Temperature Meter Report.\n");

	/* Main cam */
	ret = Get_Camera_Temperature(
	    DUAL_CAMERA_MAIN_SENSOR,
	    &valid[0],
	    &temp[0]);

	pr_info("senDevId(%d), valid(%d), temperature(%d)\n",
				DUAL_CAMERA_MAIN_SENSOR, valid[0], temp[0]);

	if (ret != ERROR_NONE)
		PK_DBG("Get Main cam temperature error(%d)!\n", ret);

	/* Sub cam */
	ret = Get_Camera_Temperature(
	    DUAL_CAMERA_SUB_SENSOR,
	    &valid[1],
	    &temp[1]);

	pr_info("senDevId(%d), valid(%d), temperature(%d)\n",
				DUAL_CAMERA_SUB_SENSOR, valid[1], temp[1]);

	if (ret != ERROR_NONE)
		PK_DBG("Get Sub cam temperature error(%d)!\n", ret);

	/* Main2 cam */
	ret = Get_Camera_Temperature(
	    DUAL_CAMERA_MAIN_2_SENSOR,
	    &valid[2],
	    &temp[2]);

	pr_info("senDevId(%d), valid(%d), temperature(%d)\n",
				DUAL_CAMERA_MAIN_2_SENSOR, valid[2], temp[2]);

	if (ret != ERROR_NONE)
		PK_DBG("Get Main2 cam temperature error(%d)!\n", ret);

	ret = Get_Camera_Temperature(
	    DUAL_CAMERA_SUB_2_SENSOR,
	    &valid[3],
	    &temp[3]);

	pr_info("senDevId(%d), valid(%d), temperature(%d)\n",
				DUAL_CAMERA_SUB_2_SENSOR, valid[3], temp[3]);

	if (ret != ERROR_NONE)
		PK_DBG("Get Sub2 cam temperature error(%d)!\n", ret);
	schedule_delayed_work(&cam_temperature_wq, HZ);

}
#endif

static inline int adopt_CAMERA_HW_GetInfo2(void *pBuf)
{
	int ret = 0;
	struct IMAGESENSOR_GETINFO_STRUCT *pSensorGetInfo;
	struct IMGSENSOR_SENSOR    *psensor;

	ACDK_SENSOR_INFO2_STRUCT *pSensorInfo = NULL;
	MSDK_SENSOR_INFO_STRUCT *pInfo = NULL;
	MSDK_SENSOR_CONFIG_STRUCT  *pConfig = NULL;
	MSDK_SENSOR_INFO_STRUCT *pInfo1 = NULL;
	MSDK_SENSOR_CONFIG_STRUCT  *pConfig1 = NULL;
	MSDK_SENSOR_INFO_STRUCT *pInfo2 = NULL;
	MSDK_SENSOR_CONFIG_STRUCT  *pConfig2 = NULL;
	MSDK_SENSOR_INFO_STRUCT *pInfo3 = NULL;
	MSDK_SENSOR_CONFIG_STRUCT  *pConfig3 = NULL;
	MSDK_SENSOR_INFO_STRUCT *pInfo4 = NULL;
	MSDK_SENSOR_CONFIG_STRUCT  *pConfig4 = NULL;
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT  *psensorResolution = NULL;
	char *pmtk_ccm_name = NULL;

	pSensorGetInfo = (struct IMAGESENSOR_GETINFO_STRUCT *)pBuf;
	if (pSensorGetInfo == NULL ||
	    pSensorGetInfo->pInfo == NULL ||
	    pSensorGetInfo->pSensorResolution == NULL) {
		pr_info("[%s] NULL arg.\n", __func__);
		return -EFAULT;
	}

	psensor = imgsensor_sensor_get_inst(pSensorGetInfo->SensorId);
	if (psensor == NULL) {
		pr_info("[%s] NULL psensor.\n", __func__);
		return -EFAULT;
	}

	PK_DBG("[%s]Entry%d\n", __func__, pSensorGetInfo->SensorId);

	pInfo =    kzalloc(sizeof(MSDK_SENSOR_INFO_STRUCT), GFP_KERNEL);
	pConfig =  kzalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	pInfo1 =   kzalloc(sizeof(MSDK_SENSOR_INFO_STRUCT), GFP_KERNEL);
	pConfig1 = kzalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	pInfo2 =   kzalloc(sizeof(MSDK_SENSOR_INFO_STRUCT), GFP_KERNEL);
	pConfig2 = kzalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	pInfo3 =   kzalloc(sizeof(MSDK_SENSOR_INFO_STRUCT), GFP_KERNEL);
	pConfig3 = kzalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	pInfo4 =   kzalloc(sizeof(MSDK_SENSOR_INFO_STRUCT), GFP_KERNEL);
	pConfig4 = kzalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	psensorResolution =
	    kzalloc(sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT), GFP_KERNEL);

	pSensorInfo = kzalloc(sizeof(ACDK_SENSOR_INFO2_STRUCT), GFP_KERNEL);

	if (pConfig == NULL ||
		pConfig1 == NULL ||
		pConfig2 == NULL ||
		pConfig3 == NULL ||
		pConfig4 == NULL ||
		pSensorInfo == NULL ||
		psensorResolution == NULL) {
		PK_DBG(" ioctl allocate mem failed\n");
		ret = -EFAULT;
		goto IMGSENSOR_GET_INFO_RETURN;
	}

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	    pInfo,
	    pConfig);

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG,
	    pInfo1,
	    pConfig1);

	imgsensor_sensor_get_info(
	    psensor, MSDK_SCENARIO_ID_VIDEO_PREVIEW,
	    pInfo2,
	    pConfig2);

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO,
	    pInfo3,
	    pConfig3);

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_SLIM_VIDEO,
	    pInfo4,
	    pConfig4);

	/* Basic information */
	pSensorInfo->SensorPreviewResolutionX = pInfo->SensorPreviewResolutionX;
	pSensorInfo->SensorPreviewResolutionY = pInfo->SensorPreviewResolutionY;
	pSensorInfo->SensorFullResolutionX = pInfo->SensorFullResolutionX;
	pSensorInfo->SensorFullResolutionY = pInfo->SensorFullResolutionY;
	pSensorInfo->SensorClockFreq = pInfo->SensorClockFreq;
	pSensorInfo->SensorCameraPreviewFrameRate =
	    pInfo->SensorCameraPreviewFrameRate;

	pSensorInfo->SensorVideoFrameRate = pInfo->SensorVideoFrameRate;
	pSensorInfo->SensorStillCaptureFrameRate =
	    pInfo->SensorStillCaptureFrameRate;

	pSensorInfo->SensorWebCamCaptureFrameRate =
	    pInfo->SensorWebCamCaptureFrameRate;

	pSensorInfo->SensorClockPolarity = pInfo->SensorClockPolarity;
	pSensorInfo->SensorClockFallingPolarity =
	    pInfo->SensorClockFallingPolarity;

	pSensorInfo->SensorClockRisingCount = pInfo->SensorClockRisingCount;
	pSensorInfo->SensorClockFallingCount = pInfo->SensorClockFallingCount;
	pSensorInfo->SensorClockDividCount = pInfo->SensorClockDividCount;
	pSensorInfo->SensorPixelClockCount = pInfo->SensorPixelClockCount;
	pSensorInfo->SensorDataLatchCount = pInfo->SensorDataLatchCount;
	pSensorInfo->SensorHsyncPolarity = pInfo->SensorHsyncPolarity;
	pSensorInfo->SensorVsyncPolarity = pInfo->SensorVsyncPolarity;
	pSensorInfo->SensorInterruptDelayLines =
	    pInfo->SensorInterruptDelayLines;

	pSensorInfo->SensorResetActiveHigh = pInfo->SensorResetActiveHigh;
	pSensorInfo->SensorResetDelayCount = pInfo->SensorResetDelayCount;
	pSensorInfo->SensroInterfaceType = pInfo->SensroInterfaceType;
	pSensorInfo->SensorOutputDataFormat  = pInfo->SensorOutputDataFormat;
	pSensorInfo->SensorMIPILaneNumber = pInfo->SensorMIPILaneNumber;
	pSensorInfo->CaptureDelayFrame = pInfo->CaptureDelayFrame;
	pSensorInfo->PreviewDelayFrame = pInfo->PreviewDelayFrame;
	pSensorInfo->VideoDelayFrame = pInfo->VideoDelayFrame;
	pSensorInfo->HighSpeedVideoDelayFrame =
	    pInfo->HighSpeedVideoDelayFrame;

	pSensorInfo->SlimVideoDelayFrame = pInfo->SlimVideoDelayFrame;
	pSensorInfo->Custom1DelayFrame = pInfo->Custom1DelayFrame;
	pSensorInfo->Custom2DelayFrame = pInfo->Custom2DelayFrame;
	pSensorInfo->Custom3DelayFrame = pInfo->Custom3DelayFrame;
	pSensorInfo->Custom4DelayFrame = pInfo->Custom4DelayFrame;
	pSensorInfo->Custom5DelayFrame = pInfo->Custom5DelayFrame;
	pSensorInfo->YUVAwbDelayFrame = pInfo->YUVAwbDelayFrame;
	pSensorInfo->YUVEffectDelayFrame = pInfo->YUVEffectDelayFrame;
	pSensorInfo->SensorGrabStartX_PRV = pInfo->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_PRV = pInfo->SensorGrabStartY;
	pSensorInfo->SensorGrabStartX_CAP = pInfo1->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_CAP = pInfo1->SensorGrabStartY;
	pSensorInfo->SensorGrabStartX_VD  = pInfo2->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_VD  = pInfo2->SensorGrabStartY;
	pSensorInfo->SensorGrabStartX_VD1 = pInfo3->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_VD1 = pInfo3->SensorGrabStartY;
	pSensorInfo->SensorGrabStartX_VD2 = pInfo4->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_VD2 = pInfo4->SensorGrabStartY;
	pSensorInfo->SensorDrivingCurrent = pInfo->SensorDrivingCurrent;
	pSensorInfo->SensorMasterClockSwitch = pInfo->SensorMasterClockSwitch;
	pSensorInfo->AEShutDelayFrame = pInfo->AEShutDelayFrame;
	pSensorInfo->AESensorGainDelayFrame = pInfo->AESensorGainDelayFrame;
	pSensorInfo->AEISPGainDelayFrame = pInfo->AEISPGainDelayFrame;
	pSensorInfo->FrameTimeDelayFrame = pInfo->FrameTimeDelayFrame;

	pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount =
	   pInfo->MIPIDataLowPwr2HighSpeedTermDelayCount;

	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount =
	   pInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount;

	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM0 =
	    pInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount;

	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM1 =
	    pInfo1->MIPIDataLowPwr2HighSpeedSettleDelayCount;

	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM2 =
	    pInfo2->MIPIDataLowPwr2HighSpeedSettleDelayCount;
	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM3 =
	    pInfo3->MIPIDataLowPwr2HighSpeedSettleDelayCount;

	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM4 =
	    pInfo4->MIPIDataLowPwr2HighSpeedSettleDelayCount;

	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount =
	    pInfo->MIPICLKLowPwr2HighSpeedTermDelayCount;

	pSensorInfo->SensorWidthSampling = pInfo->SensorWidthSampling;
	pSensorInfo->SensorHightSampling = pInfo->SensorHightSampling;
	pSensorInfo->SensorPacketECCOrder = pInfo->SensorPacketECCOrder;
	pSensorInfo->MIPIsensorType = pInfo->MIPIsensorType;
	pSensorInfo->IHDR_LE_FirstLine = pInfo->IHDR_LE_FirstLine;
	pSensorInfo->IHDR_Support = pInfo->IHDR_Support;
	pSensorInfo->ZHDR_Mode = pInfo->ZHDR_Mode;
	pSensorInfo->TEMPERATURE_SUPPORT = pInfo->TEMPERATURE_SUPPORT;
	pSensorInfo->SensorModeNum = pInfo->SensorModeNum;
	pSensorInfo->SettleDelayMode = pInfo->SettleDelayMode;
	pSensorInfo->PDAF_Support = pInfo->PDAF_Support;
	pSensorInfo->HDR_Support = pInfo->HDR_Support;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_PRE = pInfo->DPCM_INFO;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_CAP = pInfo1->DPCM_INFO;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_VD  = pInfo2->DPCM_INFO;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_VD1 = pInfo3->DPCM_INFO;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_VD2 = pInfo4->DPCM_INFO;
	/*Per-Frame conrol support or not */
	pSensorInfo->PerFrameCTL_Support  = pInfo->PerFrameCTL_Support;
	/*SCAM number*/
	pSensorInfo->SCAM_DataNumber = pInfo->SCAM_DataNumber;
	pSensorInfo->SCAM_DDR_En = pInfo->SCAM_DDR_En;
	pSensorInfo->SCAM_CLK_INV = pInfo->SCAM_CLK_INV;
	pSensorInfo->SCAM_DEFAULT_DELAY = pInfo->SCAM_DEFAULT_DELAY;
	pSensorInfo->SCAM_CRC_En  = pInfo->SCAM_CRC_En;
	pSensorInfo->SCAM_SOF_src = pInfo->SCAM_SOF_src;
	pSensorInfo->SCAM_Timeout_Cali = pInfo->SCAM_Timeout_Cali;
	/*Deskew*/
	pSensorInfo->SensorMIPIDeskew = pInfo->SensorMIPIDeskew;
	pSensorInfo->SensorVerFOV = pInfo->SensorVerFOV;
	pSensorInfo->SensorHorFOV = pInfo->SensorHorFOV;
	pSensorInfo->SensorOrientation = pInfo->SensorOrientation;

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_CUSTOM1,
	    pInfo,
	    pConfig);

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_CUSTOM2,
	    pInfo1,
	    pConfig1);

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_CUSTOM3,
	    pInfo2,
	    pConfig2);

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_CUSTOM4,
	    pInfo3,
	    pConfig3);

	imgsensor_sensor_get_info(
	    psensor,
	    MSDK_SCENARIO_ID_CUSTOM5,
	    pInfo4,
	    pConfig4);

	/* To set sensor information */
	pSensorInfo->SensorGrabStartX_CST1 = pInfo->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_CST1 = pInfo->SensorGrabStartY;
	pSensorInfo->SensorGrabStartX_CST2 = pInfo1->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_CST2 = pInfo1->SensorGrabStartY;
	pSensorInfo->SensorGrabStartX_CST3 = pInfo2->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_CST3 = pInfo2->SensorGrabStartY;
	pSensorInfo->SensorGrabStartX_CST4 = pInfo3->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_CST4 = pInfo3->SensorGrabStartY;
	pSensorInfo->SensorGrabStartX_CST5 = pInfo4->SensorGrabStartX;
	pSensorInfo->SensorGrabStartY_CST5 = pInfo4->SensorGrabStartY;

	if (copy_to_user(
	    (void __user *)(pSensorGetInfo->pInfo),
	    (void *)(pSensorInfo),
	    sizeof(ACDK_SENSOR_INFO2_STRUCT))) {

		PK_DBG("[CAMERA_HW][info] ioctl copy to user failed\n");
		ret = -EFAULT;
		goto IMGSENSOR_GET_INFO_RETURN;
	}

	/* Step2 : Get Resolution */
	imgsensor_sensor_get_resolution(psensor, psensorResolution);

	PK_DBG(
		"[CAMERA_HW][Pre]w=0x%x, h = 0x%x\n, [Full]w=0x%x, h = 0x%x\n, [VD]w=0x%x, h = 0x%x\n",
				psensorResolution->SensorPreviewWidth,
				psensorResolution->SensorPreviewHeight,
				psensorResolution->SensorFullWidth,
				psensorResolution->SensorFullHeight,
				psensorResolution->SensorVideoWidth,
				psensorResolution->SensorVideoHeight);

	if (pSensorGetInfo->SensorId <= last_id) {
		memset(mtk_ccm_name, 0, camera_info_size);
		PK_DBG("memset ok");
	}
	last_id = pSensorGetInfo->SensorId;

	/* Add info to proc: camera_info */
	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
				"\n\nCAM_Info[%d]:%s;",
				pSensorGetInfo->SensorId,
				psensor->inst.psensor_name);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nPre: TgGrab_w,h,x_,y=%5d,%5d,%3d,%3d, delay_frm=%2d",
		psensorResolution->SensorPreviewWidth,
		psensorResolution->SensorPreviewHeight,
		pSensorInfo->SensorGrabStartX_PRV,
		pSensorInfo->SensorGrabStartY_PRV,
		pSensorInfo->PreviewDelayFrame);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nCap: TgGrab_w,h,x_,y=%5d,%5d,%3d,%3d, delay_frm=%2d",
		psensorResolution->SensorFullWidth,
		psensorResolution->SensorFullHeight,
		pSensorInfo->SensorGrabStartX_CAP,
		pSensorInfo->SensorGrabStartY_CAP,
		pSensorInfo->CaptureDelayFrame);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nVid: TgGrab_w,h,x_,y=%5d,%5d,%3d,%3d, delay_frm=%2d",
		psensorResolution->SensorVideoWidth,
		psensorResolution->SensorVideoHeight,
		pSensorInfo->SensorGrabStartX_VD,
		pSensorInfo->SensorGrabStartY_VD,
		pSensorInfo->VideoDelayFrame);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nHSV: TgGrab_w,h,x_,y=%5d,%5d,%3d,%3d, delay_frm=%2d",
		psensorResolution->SensorHighSpeedVideoWidth,
		psensorResolution->SensorHighSpeedVideoHeight,
		pSensorInfo->SensorGrabStartX_VD1,
		pSensorInfo->SensorGrabStartY_VD1,
		pSensorInfo->HighSpeedVideoDelayFrame);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nSLV: TgGrab_w,h,x_,y=%5d,%5d,%3d,%3d, delay_frm=%2d",
		psensorResolution->SensorSlimVideoWidth,
		psensorResolution->SensorSlimVideoHeight,
		pSensorInfo->SensorGrabStartX_VD2,
		pSensorInfo->SensorGrabStartY_VD2,
		pSensorInfo->SlimVideoDelayFrame);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nSeninf_Type(0:parallel,1:mipi,2:serial)=%d, output_format(0:B,1:Gb,2:Gr,3:R)=%2d",
		pSensorInfo->SensroInterfaceType,
		pSensorInfo->SensorOutputDataFormat);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nDriving_Current(0:2mA,1:4mA,2:6mA,3:8mA)=%d, mclk_freq=%2d, mipi_lane=%d",
		pSensorInfo->SensorDrivingCurrent,
		pSensorInfo->SensorClockFreq,
		pSensorInfo->SensorMIPILaneNumber + 1);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nPDAF_Support(0:No PD,1:PD RAW,2:VC(Full),3:VC(Bin),4:Dual Raw,5:Dual VC=%2d",
		pSensorInfo->PDAF_Support);

	pmtk_ccm_name = strchr(mtk_ccm_name, '\0');
	snprintf(pmtk_ccm_name,
		camera_info_size - (int)(pmtk_ccm_name - mtk_ccm_name),
		"\nHDR_Support(0:NO HDR,1: iHDR,2:mvHDR,3:zHDR)=%2d",
		pSensorInfo->HDR_Support);

	/* Resolution */
	if (copy_to_user(
	    (void __user *) (pSensorGetInfo->pSensorResolution),
	    (void *)psensorResolution,
	    sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT))) {

		PK_DBG("[CAMERA_HW][Resolution] ioctl copy to user failed\n");
		ret = -EFAULT;
		goto IMGSENSOR_GET_INFO_RETURN;
	}

IMGSENSOR_GET_INFO_RETURN:

	kfree(pInfo);
	kfree(pInfo1);
	kfree(pInfo2);
	kfree(pInfo3);
	kfree(pInfo4);
	kfree(pConfig);
	kfree(pConfig1);
	kfree(pConfig2);
	kfree(pConfig3);
	kfree(pConfig4);
	kfree(psensorResolution);
	kfree(pSensorInfo);

	return ret;
}   /* adopt_CAMERA_HW_GetInfo() */

static inline int adopt_CAMERA_HW_Control(void *pBuf)
{
	int ret = 0;
	struct ACDK_SENSOR_CONTROL_STRUCT *pSensorCtrl;
	struct IMGSENSOR_SENSOR *psensor;

	pSensorCtrl = (struct ACDK_SENSOR_CONTROL_STRUCT *)pBuf;
	if (pSensorCtrl == NULL) {
		PK_DBG("[%s] NULL arg.\n", __func__);
		return -EFAULT;
	}

	psensor = imgsensor_sensor_get_inst(pSensorCtrl->InvokeCamera);
	if (psensor == NULL) {
		PK_DBG("[%s] NULL psensor.\n", __func__);
		return -EFAULT;
	}

	ret = imgsensor_sensor_control(psensor, pSensorCtrl->ScenarioId);

	return ret;
} /* adopt_CAMERA_HW_Control */

static inline int check_length_of_para(
	enum ACDK_SENSOR_FEATURE_ENUM FeatureId, unsigned int length)
{
	int ret = 0;

	switch (FeatureId) {
	case SENSOR_FEATURE_OPEN:
	case SENSOR_FEATURE_CLOSE:
	case SENSOR_FEATURE_CHECK_IS_ALIVE:
		break;
	case SENSOR_FEATURE_SET_DRIVER:
#ifdef TCT_CAM_DRIVER_SUPPORT
{
   if (length != sizeof(DRIVER_INFO)){
       pr_info("[%s] heaven psensor.length=%d\n", __func__,length);
      ret = -EFAULT;
  }
}break;
#endif
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	case SENSOR_FEATURE_SET_FRAMERATE:
	case SENSOR_FEATURE_SET_HDR:
	case SENSOR_FEATURE_SET_PDAF:
	{
		if (length != 4)
			ret = -EFAULT;
	}
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	case SENSOR_FEATURE_SET_GAIN:
	case SENSOR_FEATURE_SET_I2C_BUF_MODE_EN:
	case SENSOR_FEATURE_SET_SHUTTER_BUF_MODE:
	case SENSOR_FEATURE_SET_GAIN_BUF_MODE:
	case SENSOR_FEATURE_SET_VIDEO_MODE:
	case SENSOR_FEATURE_SET_AF_WINDOW:
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
	case SENSOR_FEATURE_GET_EV_AWB_REF:
	case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
	case SENSOR_FEATURE_GET_EXIF_INFO:
	case SENSOR_FEATURE_GET_DELAY_INFO:
	case SENSOR_FEATURE_SET_TEST_PATTERN:
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	case SENSOR_FEATURE_SET_OB_LOCK:
	case SENSOR_FEATURE_SET_SENSOR_OTP_AWB_CMD:
	case SENSOR_FEATURE_SET_SENSOR_OTP_LSC_CMD:
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
	case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
	case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
	case SENSOR_FEATURE_SET_YUV_3A_CMD:
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
	case SENSOR_FEATURE_GET_PERIOD:
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	{
		if (length != 8)
			ret = -EFAULT;
	}
		break;
	case SENSOR_FEATURE_SET_DUAL_GAIN:
	case SENSOR_FEATURE_SET_YUV_CMD:
	case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	case SENSOR_FEATURE_GET_CROP_INFO:
	case SENSOR_FEATURE_GET_VC_INFO:
	case SENSOR_FEATURE_GET_PDAF_INFO:
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
	case SENSOR_FEATURE_SET_PDFOCUS_AREA:
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
	{
		if (length != 16)
			ret = -EFAULT;
	}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
	case SENSOR_FEATURE_GET_PDAF_DATA:
	case SENSOR_FEATURE_GET_4CELL_DATA:
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	case SENSOR_FEATURE_GET_PIXEL_RATE:
	{
		if (length != 24)
			ret = -EFAULT;
	}
		break;
	case SENSOR_FEATURE_SET_SENSOR_SYNC:
	case SENSOR_FEATURE_SET_ESHUTTER_GAIN:
	{
		if (length != 32)
			ret = -EFAULT;
	}
		break;
	case SENSOR_FEATURE_SET_CALIBRATION_DATA:
	{
		if (length !=
			sizeof(struct SET_SENSOR_CALIBRATION_DATA_STRUCT))
			ret = -EFAULT;
	}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
	{
		if (length !=
			sizeof(struct SET_SENSOR_AWB_GAIN))
			ret = -EFAULT;
	}
		break;
	case SENSOR_FEATURE_SET_REGISTER:
	case SENSOR_FEATURE_GET_REGISTER:
	{

		if (length !=
			sizeof(MSDK_SENSOR_REG_INFO_STRUCT))
			ret = -EFAULT;
	}
		break;
	/* begin of legacy feature control; Do nothing */
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	case SENSOR_FEATURE_SET_CCT_REGISTER:
	case SENSOR_FEATURE_SET_ENG_REGISTER:
	case SENSOR_FEATURE_SET_ITEM_INFO:
	case SENSOR_FEATURE_GET_ITEM_INFO:
	case SENSOR_FEATURE_GET_ENG_INFO:
	case SENSOR_FEATURE_MOVE_FOCUS_LENS:
	case SENSOR_FEATURE_SET_AE_WINDOW:
	case SENSOR_FEATURE_SET_MIN_MAX_FPS:
	case SENSOR_FEATURE_GET_RESOLUTION:
	case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
	case SENSOR_FEATURE_GET_CONFIG_PARA:
	case SENSOR_FEATURE_GET_GROUP_COUNT:
	case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
	case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
	case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
	case SENSOR_FEATURE_CANCEL_AF:
	case SENSOR_FEATURE_CONSTANT_AF:
	/* end of legacy feature control */
	default:
		break;
	}
	if (ret != 0)
		pr_err(
			"check length failed, feature ctrl id = %d, length = %d\n",
			FeatureId,
			length);
	return ret;
} /* adopt_CAMERA_HW_Control */

static inline int adopt_CAMERA_HW_FeatureControl(void *pBuf)
{
	struct ACDK_SENSOR_FEATURECONTROL_STRUCT *pFeatureCtrl;
	struct IMGSENSOR_SENSOR *psensor;
	unsigned int FeatureParaLen = 0;
	void *pFeaturePara = NULL;
	struct ACDK_KD_SENSOR_SYNC_STRUCT *pSensorSyncInfo = NULL;
	signed int ret = 0;
        MINT32 drv_idx;
	struct SET_SENSOR_PATTERN_SOLID_COLOR *pData = NULL;
	unsigned long long *pFeaturePara_64 = NULL;
	void *usr_ptr = NULL;
	kal_uint32 u4RegLen = 0;
	char *pPdaf_data = NULL;
	void *usr_ptr_Reg = NULL;
	kal_uint32 buf_size = 0;
	kal_uint32 *pReg = NULL;

	pFeatureCtrl = (struct ACDK_SENSOR_FEATURECONTROL_STRUCT *)pBuf;
	if (pFeatureCtrl  == NULL) {
		PK_DBG(" NULL arg.\n");
		return -EFAULT;
	}

	psensor = imgsensor_sensor_get_inst(pFeatureCtrl->InvokeCamera);
	if (psensor == NULL) {
		PK_DBG("[%s] NULL psensor.\n", __func__);
		return -EFAULT;
	}

	if (pFeatureCtrl->FeatureId == SENSOR_FEATURE_SINGLE_FOCUS_MODE ||
		pFeatureCtrl->FeatureId == SENSOR_FEATURE_CANCEL_AF ||
		pFeatureCtrl->FeatureId == SENSOR_FEATURE_CONSTANT_AF ||
		pFeatureCtrl->FeatureId == SENSOR_FEATURE_INFINITY_AF) {
		/* YUV AF_init and AF_constent and AF_single has no params */
	} else {
		if (pFeatureCtrl->pFeaturePara == NULL ||
		    pFeatureCtrl->pFeatureParaLen == NULL) {
			PK_DBG(" NULL arg.\n");
			return -EFAULT;
		}
		if (copy_from_user((void *)&FeatureParaLen,
				(void *) pFeatureCtrl->pFeatureParaLen,
				sizeof(unsigned int))) {

			PK_DBG(" ioctl copy from user failed\n");
			return -EFAULT;
		}
		if (!FeatureParaLen ||
			FeatureParaLen > FEATURE_CONTROL_MAX_DATA_SIZE)
			return -EINVAL;
		ret = check_length_of_para(pFeatureCtrl->FeatureId,
							FeatureParaLen);
		if (ret != 0){
			return ret;
			}

		pFeaturePara = kmalloc(FeatureParaLen, GFP_KERNEL);
		if (pFeaturePara == NULL)
			return -ENOMEM;

		memset(pFeaturePara, 0x0, FeatureParaLen);
	}

	/* copy from user */
	switch (pFeatureCtrl->FeatureId) {
	case SENSOR_FEATURE_OPEN:
		ret = imgsensor_sensor_open(psensor);
		break;
	case SENSOR_FEATURE_CLOSE:
		ret = imgsensor_sensor_close(psensor);
		/* reset the delay frame flag */
		break;

	case SENSOR_FEATURE_SET_DRIVER:
	{
#ifndef TCT_CAM_DRIVER_SUPPORT

		psensor->inst.sensor_idx = pFeatureCtrl->InvokeCamera;
		drv_idx = imgsensor_set_driver(psensor);
		memcpy(pFeaturePara, &drv_idx, FeatureParaLen);
#else
    DRIVER_INFO deriver_info = {};
    psensor->inst.sensor_idx = pFeatureCtrl->InvokeCamera;
    drv_idx = imgsensor_set_driver(psensor);
    // heaven
    if (drv_idx != -EIO) {
      deriver_info.SensorId = kdSensorList[drv_idx].id;
      memcpy(deriver_info.drvname, kdSensorList[drv_idx].name,
             sizeof(kdSensorList[drv_idx].name));
      memcpy(deriver_info.macroName, kdSensorList[drv_idx].macro_name,
             sizeof(kdSensorList[drv_idx].macro_name));
    }else{
	pr_info("heaven3  no sensor.\n");
	}
    memcpy(pFeaturePara, &deriver_info, FeatureParaLen);
#endif
		break;
	}
	case SENSOR_FEATURE_CHECK_IS_ALIVE:
		imgsensor_check_is_alive(psensor);
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	case SENSOR_FEATURE_SET_GAIN:
	case SENSOR_FEATURE_SET_DUAL_GAIN:
	case SENSOR_FEATURE_SET_MCLK_DRIVE_CURRENT:
	case SENSOR_FEATURE_SET_I2C_BUF_MODE_EN:
	case SENSOR_FEATURE_SET_SHUTTER_BUF_MODE:
	case SENSOR_FEATURE_SET_GAIN_BUF_MODE:
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	case SENSOR_FEATURE_SET_REGISTER:
	case SENSOR_FEATURE_GET_REGISTER:
	case SENSOR_FEATURE_SET_CCT_REGISTER:
	case SENSOR_FEATURE_SET_ENG_REGISTER:
	case SENSOR_FEATURE_SET_ITEM_INFO:
	case SENSOR_FEATURE_GET_ITEM_INFO:
	case SENSOR_FEATURE_GET_ENG_INFO:
	case SENSOR_FEATURE_SET_VIDEO_MODE:
	case SENSOR_FEATURE_SET_YUV_CMD:
	case SENSOR_FEATURE_MOVE_FOCUS_LENS:
	case SENSOR_FEATURE_SET_AF_WINDOW:
	case SENSOR_FEATURE_SET_CALIBRATION_DATA:
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
	case SENSOR_FEATURE_GET_EV_AWB_REF:
	case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
	case SENSOR_FEATURE_SET_AE_WINDOW:
	case SENSOR_FEATURE_GET_EXIF_INFO:
	case SENSOR_FEATURE_GET_DELAY_INFO:
	case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	case SENSOR_FEATURE_SET_TEST_PATTERN:
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	case SENSOR_FEATURE_SET_OB_LOCK:
	case SENSOR_FEATURE_SET_SENSOR_OTP_AWB_CMD:
	case SENSOR_FEATURE_SET_SENSOR_OTP_LSC_CMD:
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
	case SENSOR_FEATURE_SET_FRAMERATE:
	case SENSOR_FEATURE_SET_HDR:
	case SENSOR_FEATURE_GET_CROP_INFO:
	case SENSOR_FEATURE_GET_VC_INFO:
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
	case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:

	/* return TRUE:play flashlight */
	case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:

	/* para: ACDK_SENSOR_3A_LOCK_ENUM */
	case SENSOR_FEATURE_SET_YUV_3A_CMD:
	case SENSOR_FEATURE_SET_AWB_GAIN:
	case SENSOR_FEATURE_SET_MIN_MAX_FPS:
	case SENSOR_FEATURE_GET_PDAF_INFO:
	case SENSOR_FEATURE_GET_PDAF_DATA:
	case SENSOR_FEATURE_GET_4CELL_DATA:
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
	case SENSOR_FEATURE_GET_PIXEL_RATE:
	case SENSOR_FEATURE_SET_PDAF:
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
	case SENSOR_FEATURE_SET_PDFOCUS_AREA:
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
	case SENSOR_FEATURE_SET_SENSOR_SYNC_MODE:
		if (copy_from_user(
		    (void *)pFeaturePara,
		    (void *) pFeatureCtrl->pFeaturePara,
		    FeatureParaLen)) {
			kfree(pFeaturePara);
			PK_DBG(
			    "[CAMERA_HW][pFeaturePara] ioctl copy from user failed\n");
			return -EFAULT;
		}
		break;
	case SENSOR_FEATURE_SET_SENSOR_SYNC:
	case SENSOR_FEATURE_SET_ESHUTTER_GAIN:
		PK_DBG("[kd_sensorlist]enter kdSetExpGain\n");
		if (copy_from_user(
		    (void *)pFeaturePara,
		    (void *) pFeatureCtrl->pFeaturePara,
		    FeatureParaLen)) {
			kfree(pFeaturePara);
			PK_DBG(
			    "[CAMERA_HW][pFeaturePara] ioctl copy from user failed\n");
			return -EFAULT;
		}
		/* keep the information to wait Vsync synchronize */
		pSensorSyncInfo =
		    (struct ACDK_KD_SENSOR_SYNC_STRUCT *)pFeaturePara;

		FeatureParaLen = 2;

		imgsensor_sensor_feature_control(
		    psensor,
		    SENSOR_FEATURE_SET_ESHUTTER,
		    (unsigned char *)&pSensorSyncInfo->u2SensorNewExpTime,
		    (unsigned int *)&FeatureParaLen);

		imgsensor_sensor_feature_control(
		    psensor,
		    SENSOR_FEATURE_SET_GAIN,
		    (unsigned char *)&pSensorSyncInfo->u2SensorNewGain,
		    (unsigned int *) &FeatureParaLen);
		break;

	/* copy to user */
	case SENSOR_FEATURE_GET_RESOLUTION:
	case SENSOR_FEATURE_GET_PERIOD:
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
	case SENSOR_FEATURE_GET_CONFIG_PARA:
	case SENSOR_FEATURE_GET_GROUP_COUNT:
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	/* do nothing */
	case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
	case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
	case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
	case SENSOR_FEATURE_CANCEL_AF:
	case SENSOR_FEATURE_CONSTANT_AF:
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	case SENSOR_FEATURE_GET_SENSOR_SYNC_MODE_CAPACITY:
	case SENSOR_FEATURE_GET_SENSOR_SYNC_MODE:
	default:
		break;
	}

	/*in case that some structure are passed from user sapce by ptr */
	switch (pFeatureCtrl->FeatureId) {
	case SENSOR_FEATURE_OPEN:
	case SENSOR_FEATURE_CLOSE:
	case SENSOR_FEATURE_SET_DRIVER:
	case SENSOR_FEATURE_CHECK_IS_ALIVE:
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	case SENSOR_FEATURE_GET_PIXEL_RATE:
	{
		MUINT32 *pValue = NULL;
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		if (FeatureParaLen < 2 * sizeof(unsigned long long)) {
			PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
			kfree(pFeaturePara);
			return -EINVAL;
		}

		pValue = kmalloc(sizeof(MUINT32), GFP_KERNEL);
		if (pValue == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		memset(pValue, 0x0, sizeof(MUINT32));
		*(pFeaturePara_64 + 1) = (uintptr_t)pValue;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		*(pFeaturePara_64 + 1) = *pValue;
		kfree(pValue);
	}
	break;

	case SENSOR_FEATURE_GET_AE_STATUS:
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
	case SENSOR_FEATURE_GET_AF_STATUS:
	case SENSOR_FEATURE_GET_AWB_STATUS:
	case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
	case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
	case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
	case SENSOR_FEATURE_GET_SENSOR_N3D_STREAM_TO_VSYNC_TIME:
	case SENSOR_FEATURE_GET_PERIOD:
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	case SENSOR_FEATURE_GET_SENSOR_SYNC_MODE_CAPACITY:
	case SENSOR_FEATURE_GET_SENSOR_SYNC_MODE:
	{
		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);
	}
	break;

	case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
	case SENSOR_FEATURE_AUTOTEST_CMD:
	{
		MUINT32 *pValue0 = NULL;
		MUINT32 *pValue1 = NULL;
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		if (FeatureParaLen < 2 * sizeof(unsigned long long)) {
			PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
			kfree(pFeaturePara);
			return -EINVAL;
		}

		pValue0 = kmalloc(sizeof(MUINT32), GFP_KERNEL);
		pValue1 = kmalloc(sizeof(MUINT32), GFP_KERNEL);

		if (pValue0 == NULL || pValue1 == NULL) {
			PK_DBG(" ioctl allocate mem failed\n");
			kfree(pValue0);
			kfree(pValue1);
			kfree(pFeaturePara);
			return -ENOMEM;
		}
		memset(pValue1, 0x0, sizeof(MUINT32));
		memset(pValue0, 0x0, sizeof(MUINT32));
		*(pFeaturePara_64) = (uintptr_t)pValue0;
		*(pFeaturePara_64 + 1) = (uintptr_t)pValue1;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		*(pFeaturePara_64) = *pValue0;
		*(pFeaturePara_64 + 1) = *pValue1;
		kfree(pValue0);
		kfree(pValue1);
	}
	break;

	case SENSOR_FEATURE_GET_EV_AWB_REF:
	{
		struct SENSOR_AE_AWB_REF_STRUCT *pAeAwbRef = NULL;
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr = (void *)(uintptr_t)(*(pFeaturePara_64));

		if (FeatureParaLen < 1 * sizeof(unsigned long long)) {
			PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
			kfree(pFeaturePara);
			return -EINVAL;
		}

		pAeAwbRef = kmalloc(
		    sizeof(struct SENSOR_AE_AWB_REF_STRUCT),
		    GFP_KERNEL);

		if (pAeAwbRef == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(
		    pAeAwbRef,
		    0x0,
		    sizeof(struct SENSOR_AE_AWB_REF_STRUCT));

		*(pFeaturePara_64) = (uintptr_t)pAeAwbRef;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pAeAwbRef,
		    sizeof(struct SENSOR_AE_AWB_REF_STRUCT))) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pAeAwbRef);
		*(pFeaturePara_64) = (uintptr_t)usr_ptr;
	}
	break;

	case SENSOR_FEATURE_GET_CROP_INFO:
	{
		struct SENSOR_WINSIZE_INFO_STRUCT *pCrop = NULL;
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64 + 1));

		if (FeatureParaLen < 2 * sizeof(unsigned long long)) {
			PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
			kfree(pFeaturePara);
			return -EINVAL;
		}

		pCrop = kmalloc(
		    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT),
		    GFP_KERNEL);

		if (pCrop == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(pCrop, 0x0, sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		*(pFeaturePara_64 + 1) = (uintptr_t)pCrop;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pCrop,
		    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT))) {

			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pCrop);
		*(pFeaturePara_64 + 1) = (uintptr_t)usr_ptr;
	}
	break;

	case SENSOR_FEATURE_GET_VC_INFO:
	{
		struct SENSOR_VC_INFO_STRUCT *pVcInfo = NULL;
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64 + 1));

		if (FeatureParaLen < 2 * sizeof(unsigned long long)) {
			PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
			kfree(pFeaturePara);
			return -EINVAL;
		}

		pVcInfo = kmalloc(
		    sizeof(struct SENSOR_VC_INFO_STRUCT),
		    GFP_KERNEL);

		if (pVcInfo == NULL) {
			PK_DBG(" ioctl allocate mem failed\n");
			kfree(pFeaturePara);
			return -ENOMEM;
		}
		memset(pVcInfo, 0x0, sizeof(struct SENSOR_VC_INFO_STRUCT));
		*(pFeaturePara_64 + 1) = (uintptr_t)pVcInfo;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pVcInfo,
		    sizeof(struct SENSOR_VC_INFO_STRUCT))) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pVcInfo);
		*(pFeaturePara_64 + 1) = (uintptr_t)usr_ptr;
	}
	break;

	case SENSOR_FEATURE_GET_PDAF_INFO:
	{
		struct SET_PD_BLOCK_INFO_T *pPdInfo = NULL;
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64 + 1));

		if (FeatureParaLen < 2 * sizeof(unsigned long long)) {
			PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
			kfree(pFeaturePara);
			return -EINVAL;
		}

		pPdInfo = kmalloc(
		    sizeof(struct SET_PD_BLOCK_INFO_T),
		    GFP_KERNEL);

		if (pPdInfo == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(pPdInfo, 0x0, sizeof(struct SET_PD_BLOCK_INFO_T));
		*(pFeaturePara_64 + 1) = (uintptr_t)pPdInfo;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pPdInfo,
		    sizeof(struct SET_PD_BLOCK_INFO_T))) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pPdInfo);
		*(pFeaturePara_64 + 1) = (uintptr_t)usr_ptr;
	}
	break;

	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
	{
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		u4RegLen = (*pFeaturePara_64);
		usr_ptr_Reg =
		    (void *)(uintptr_t) (*(pFeaturePara_64 + 1));

		if (FeatureParaLen < 2 * sizeof(unsigned long long)) {
			PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
			kfree(pFeaturePara);
			return -EINVAL;
		}

		if (u4RegLen > PDAF_DATA_SIZE) {
			kfree(pFeaturePara);
			PK_DBG("check: u4RegLen > PDAF_DATA_SIZE\n");
			return -EINVAL;
		}
		pReg = kmalloc_array(u4RegLen, sizeof(kal_uint8), GFP_KERNEL);
		if (pReg == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		memset(pReg, 0x0, sizeof(kal_uint8)*u4RegLen);

		if (copy_from_user(
		    (void *)pReg,
		    (void *)usr_ptr_Reg,
		    sizeof(kal_uint8)*u4RegLen)) {
			PK_DBG("[CAMERA_HW]ERROR: copy from user fail\n");
		}

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pReg,
		    (unsigned int *)&u4RegLen);

		if (copy_to_user(
		    (void __user *)usr_ptr_Reg,
		    (void *)pReg,
		    sizeof(kal_uint8)*u4RegLen)) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pReg);
	}

	break;
	case SENSOR_FEATURE_SET_AF_WINDOW:
	case SENSOR_FEATURE_SET_AE_WINDOW:
	{
		MUINT32 *pApWindows = NULL;
		unsigned long long *pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64));

		pApWindows = kmalloc(sizeof(MUINT32) * 6, GFP_KERNEL);
		if (pApWindows == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(pApWindows, 0x0, sizeof(MUINT32) * 6);
		*(pFeaturePara_64) = (uintptr_t)pApWindows;

		if (copy_from_user(
		    (void *)pApWindows,
		    (void *)usr_ptr,
		    sizeof(MUINT32) * 6)) {
			PK_DBG("[CAMERA_HW]ERROR: copy from user fail\n");
		}

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		kfree(pApWindows);
		*(pFeaturePara_64) = (uintptr_t)usr_ptr;
	}
	break;

	case SENSOR_FEATURE_GET_EXIF_INFO:
	{
		struct SENSOR_EXIF_INFO_STRUCT *pExif = NULL;
		unsigned long long *pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr =  (void *)(uintptr_t) (*(pFeaturePara_64));

		if (FeatureParaLen < 1 * sizeof(unsigned long long)) {
			PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
			kfree(pFeaturePara);
			return -EINVAL;
		}

		pExif = kmalloc(
		    sizeof(struct SENSOR_EXIF_INFO_STRUCT),
		    GFP_KERNEL);

		if (pExif == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(pExif, 0x0, sizeof(struct SENSOR_EXIF_INFO_STRUCT));
		*(pFeaturePara_64) = (uintptr_t)pExif;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pExif,
		    sizeof(struct SENSOR_EXIF_INFO_STRUCT))) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pExif);
		*(pFeaturePara_64) = (uintptr_t)usr_ptr;
	}
	break;

	case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
	{

		struct SENSOR_AE_AWB_CUR_STRUCT *pCurAEAWB = NULL;
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64));

			if (FeatureParaLen < 1 * sizeof(unsigned long long)) {
				PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pFeaturePara);
				return -EINVAL;
			}

		pCurAEAWB = kmalloc(
		    sizeof(struct SENSOR_AE_AWB_CUR_STRUCT),
		    GFP_KERNEL);

		if (pCurAEAWB == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(
		    pCurAEAWB,
		    0x0,
		    sizeof(struct SENSOR_AE_AWB_CUR_STRUCT));

		*(pFeaturePara_64) = (uintptr_t)pCurAEAWB;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)
		    pFeaturePara,
		    (unsigned int *)
		    &FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pCurAEAWB,
		    sizeof(struct SENSOR_AE_AWB_CUR_STRUCT))) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pCurAEAWB);
		*(pFeaturePara_64) = (uintptr_t)usr_ptr;
	}
	break;

	case SENSOR_FEATURE_GET_DELAY_INFO:
	{
		struct SENSOR_DELAY_INFO_STRUCT *pDelayInfo = NULL;
		unsigned long long *pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64));

			if (FeatureParaLen < 1 * sizeof(unsigned long long)) {
				PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pFeaturePara);
				return -EINVAL;
			}

		pDelayInfo = kmalloc(
		    sizeof(struct SENSOR_DELAY_INFO_STRUCT),
		    GFP_KERNEL);

		if (pDelayInfo == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(
		    pDelayInfo,
		    0x0,
		    sizeof(struct SENSOR_DELAY_INFO_STRUCT));

		*(pFeaturePara_64) = (uintptr_t)pDelayInfo;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pDelayInfo,
		    sizeof(struct SENSOR_DELAY_INFO_STRUCT))) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pDelayInfo);
		*(pFeaturePara_64) = (uintptr_t)usr_ptr;

	}
	break;

	case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
	{
		struct SENSOR_FLASHLIGHT_AE_INFO_STRUCT *pFlashInfo = NULL;
		pFeaturePara_64 =
		    (unsigned long long *)pFeaturePara;

		usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64));

			if (FeatureParaLen < 1 * sizeof(unsigned long long)) {
				PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pFeaturePara);
				return -EINVAL;
			}

		pFlashInfo = kmalloc(
		    sizeof(struct SENSOR_FLASHLIGHT_AE_INFO_STRUCT),
		    GFP_KERNEL);

		if (pFlashInfo == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(
		    pFlashInfo,
		    0x0,
		    sizeof(struct SENSOR_FLASHLIGHT_AE_INFO_STRUCT));

		*(pFeaturePara_64) = (uintptr_t)pFlashInfo;

		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pFlashInfo,
		    sizeof(struct SENSOR_FLASHLIGHT_AE_INFO_STRUCT))) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pFlashInfo);
		*(pFeaturePara_64) = (uintptr_t)usr_ptr;

	}
	break;

	case SENSOR_FEATURE_GET_PDAF_DATA:
	case SENSOR_FEATURE_GET_4CELL_DATA:
	{
		pFeaturePara_64 =
		    (unsigned long long *) pFeaturePara;
		usr_ptr = (void *)(uintptr_t)(*(pFeaturePara_64 + 1));
		buf_size = (kal_uint32) (*(pFeaturePara_64 + 2));

			if (FeatureParaLen < 3 * sizeof(unsigned long long)) {
				PK_DBG("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pFeaturePara);
				return -EINVAL;
			}

		if (buf_size > PDAF_DATA_SIZE) {
			kfree(pFeaturePara);
			PK_DBG("check: buf_size > PDAF_DATA_SIZE\n");
			return -EINVAL;
		}
		pPdaf_data = kmalloc(
		    sizeof(char) * PDAF_DATA_SIZE,
		    GFP_KERNEL);

		if (pPdaf_data == NULL) {
			kfree(pFeaturePara);
			PK_DBG(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(pPdaf_data, 0xff, sizeof(char) * PDAF_DATA_SIZE);

		if (pFeaturePara_64 != NULL)
			*(pFeaturePara_64 + 1) = (uintptr_t)pPdaf_data;


		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

		if (copy_to_user(
		    (void __user *)usr_ptr,
		    (void *)pPdaf_data,
		    buf_size)) {
			PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail\n");
		}
		kfree(pPdaf_data);
		*(pFeaturePara_64 + 1) = (uintptr_t) usr_ptr;
	}
	break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		{
			pr_debug("SENSOR_FEATURE_SET_TEST_PATTERN");
			pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			usr_ptr =
				(void *)(uintptr_t)(*(pFeaturePara_64 + 1));
			if (usr_ptr) {
				if (FeatureParaLen < 2 * sizeof(unsigned long long)) {
					pr_debug("FeatureParaLen is too small %d\n",
					FeatureParaLen);
					kfree(pFeaturePara);
					return -EINVAL;
				}
				pData = kmalloc(sizeof(struct SET_SENSOR_PATTERN_SOLID_COLOR),
					GFP_KERNEL);

				if (pData == NULL) {
					kfree(pFeaturePara);
					pr_debug("No color data %d\n");
					return -ENOMEM;
				}
				memset(pData, 0x0,
					sizeof(struct SET_SENSOR_PATTERN_SOLID_COLOR));

				if (copy_from_user
					((void *)pData, (void __user *)usr_ptr,
					sizeof(struct SET_SENSOR_PATTERN_SOLID_COLOR))) {
					kfree(pData);
					pr_debug("[CAMERA_HW]ERROR: copy_from_user fail\n");
				}
				//pr_debug("%x %x %x %x",
				//pData->COLOR_R, pData->COLOR_Gr,
				//pData->COLOR_Gb, pData->COLOR_B);
				if (pFeaturePara_64 != NULL)
					*(pFeaturePara_64 + 1) = (uintptr_t)pData;
			}
			ret = imgsensor_sensor_feature_control(psensor,
					pFeatureCtrl->FeatureId,
					(unsigned char *)pFeaturePara,
					(unsigned int *)&FeatureParaLen);

			kfree(pData);
		}
		break;

	default:
		ret = imgsensor_sensor_feature_control(
		    psensor,
		    pFeatureCtrl->FeatureId,
		    (unsigned char *)pFeaturePara,
		    (unsigned int *)&FeatureParaLen);

#ifdef CONFIG_MTK_CCU
		if (pFeatureCtrl->FeatureId == SENSOR_FEATURE_SET_FRAMERATE)
			ccu_set_current_fps(*((int32_t *)pFeaturePara));
#endif
		break;
	}
	/* copy to user */
	switch (pFeatureCtrl->FeatureId) {
	case SENSOR_FEATURE_SET_MCLK_DRIVE_CURRENT:
	{
		MUINT32 __current = (*(MUINT32 *)pFeaturePara);

		if (gimgsensor.mclk_set_drive_current != NULL) {
			gimgsensor.mclk_set_drive_current(
			gimgsensor.hw.pdev[IMGSENSOR_HW_ID_MCLK]->pinstance,
				pFeatureCtrl->InvokeCamera,
				__current);
		} else {
			PK_DBG(
				"%s, set drive current by pinctrl was not supported\n",
				__func__);
		}
		break;
	}
	case SENSOR_FEATURE_SET_I2C_BUF_MODE_EN:
		imgsensor_i2c_buffer_mode(
		    (*(unsigned long long *)pFeaturePara));

		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	case SENSOR_FEATURE_SET_GAIN:
	case SENSOR_FEATURE_SET_DUAL_GAIN:
	case SENSOR_FEATURE_SET_SHUTTER_BUF_MODE:
	case SENSOR_FEATURE_SET_GAIN_BUF_MODE:
	case SENSOR_FEATURE_SET_GAIN_AND_ESHUTTER:
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	case SENSOR_FEATURE_SET_REGISTER:
	case SENSOR_FEATURE_SET_CCT_REGISTER:
	case SENSOR_FEATURE_SET_ENG_REGISTER:
	case SENSOR_FEATURE_SET_ITEM_INFO:
	/* do nothing */
	case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
	case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
	case SENSOR_FEATURE_GET_PDAF_DATA:
	case SENSOR_FEATURE_GET_4CELL_DATA:
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
	case SENSOR_FEATURE_SET_SENSOR_SYNC_MODE:
		break;
	/* copy to user */
	case SENSOR_FEATURE_SET_DRIVER:
	case SENSOR_FEATURE_GET_EV_AWB_REF:
	case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
	case SENSOR_FEATURE_GET_EXIF_INFO:
	case SENSOR_FEATURE_GET_DELAY_INFO:
	case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
	case SENSOR_FEATURE_GET_RESOLUTION:
	case SENSOR_FEATURE_GET_PERIOD:
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	case SENSOR_FEATURE_GET_REGISTER:
	case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
	case SENSOR_FEATURE_GET_CONFIG_PARA:
	case SENSOR_FEATURE_GET_GROUP_COUNT:
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	case SENSOR_FEATURE_GET_ITEM_INFO:
	case SENSOR_FEATURE_GET_ENG_INFO:
	case SENSOR_FEATURE_GET_AF_STATUS:
	case SENSOR_FEATURE_GET_AE_STATUS:
	case SENSOR_FEATURE_GET_AWB_STATUS:
	case SENSOR_FEATURE_GET_AF_INF:
	case SENSOR_FEATURE_GET_AF_MACRO:
	case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:

	/* return TRUE:play flashlight */
	case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:

	/* para: ACDK_SENSOR_3A_LOCK_ENUM */
	case SENSOR_FEATURE_SET_YUV_3A_CMD:
	case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
	case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	case SENSOR_FEATURE_SET_TEST_PATTERN:
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
	case SENSOR_FEATURE_SET_FRAMERATE:
	case SENSOR_FEATURE_SET_HDR:
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
	case SENSOR_FEATURE_GET_CROP_INFO:
	case SENSOR_FEATURE_GET_VC_INFO:
	case SENSOR_FEATURE_SET_MIN_MAX_FPS:
	case SENSOR_FEATURE_GET_PDAF_INFO:
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
	case SENSOR_FEATURE_GET_PIXEL_RATE:
	case SENSOR_FEATURE_SET_ISO:
	case SENSOR_FEATURE_SET_PDAF:
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
	case SENSOR_FEATURE_SET_PDFOCUS_AREA:
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	case SENSOR_FEATURE_GET_SENSOR_SYNC_MODE_CAPACITY:
	case SENSOR_FEATURE_GET_SENSOR_SYNC_MODE:
		if (copy_to_user(
		    (void __user *) pFeatureCtrl->pFeaturePara,
		    (void *)pFeaturePara,
		    FeatureParaLen)) {

			kfree(pFeaturePara);
			PK_DBG(
			    "[CAMERA_HW][pSensorRegData] ioctl copy to user failed\n");
			return -EFAULT;
		}
		break;

	default:
		break;
	}

	kfree(pFeaturePara);
	if (copy_to_user(
	    (void __user *) pFeatureCtrl->pFeatureParaLen,
	    (void *)&FeatureParaLen,
	    sizeof(unsigned int))) {

		PK_DBG(
		    "[CAMERA_HW][pFeatureParaLen] ioctl copy to user failed\n");

		return -EFAULT;
	}

	return ret;
}   /* adopt_CAMERA_HW_FeatureControl() */

#ifdef CONFIG_COMPAT
static int compat_get_acdk_sensor_getinfo_struct(
	struct COMPAT_IMGSENSOR_GET_CONFIG_INFO_STRUCT __user *data32,
	struct IMGSENSOR_GET_CONFIG_INFO_STRUCT __user *data)
{
	compat_uint_t i;
	compat_uptr_t p;
	int err;

	err = get_user(i, &data32->SensorId);
	err |= put_user(i, &data->SensorId);
	err = get_user(i, &data32->ScenarioId);
	err |= put_user(i, &data->ScenarioId);
	err = get_user(p, &data32->pInfo);
	err |= put_user(compat_ptr(p), &data->pInfo);
	err = get_user(p, &data32->pConfig);
	err |= put_user(compat_ptr(p), &data->pConfig);

	return err;
}

static int compat_put_acdk_sensor_getinfo_struct(
	struct COMPAT_IMGSENSOR_GET_CONFIG_INFO_STRUCT __user *data32,
	struct IMGSENSOR_GET_CONFIG_INFO_STRUCT __user *data)
{
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->SensorId);
	err |= put_user(i, &data->SensorId);
	err = get_user(i, &data->ScenarioId);
	err |= put_user(i, &data32->ScenarioId);
	err = get_user(i, &data->ScenarioId);
	err |= put_user(i, &data32->ScenarioId);
	return err;
}

static int compat_get_imagesensor_getinfo_struct(
	struct COMPAT_IMAGESENSOR_GETINFO_STRUCT __user *data32,
	struct IMAGESENSOR_GETINFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->SensorId);
	err |= put_user(i, &data->SensorId);
	err |= get_user(p, &data32->pInfo);
	err |= put_user(compat_ptr(p), &data->pInfo);
	err |= get_user(p, &data32->pSensorResolution);
	err |= put_user(compat_ptr(p), &data->pSensorResolution);
	return err;
}

static int compat_put_imagesensor_getinfo_struct(
	struct COMPAT_IMAGESENSOR_GETINFO_STRUCT __user *data32,
	struct IMAGESENSOR_GETINFO_STRUCT __user *data)
{
	/* compat_uptr_t p; */
	compat_uint_t i;
	int err;

	err = get_user(i, &data->SensorId);
	err |= put_user(i, &data32->SensorId);

	return err;
}

static int compat_get_acdk_sensor_featurecontrol_struct(
	struct COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT
	__user *data32,
	struct ACDK_SENSOR_FEATURECONTROL_STRUCT __user *
	data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->InvokeCamera);
	err |= put_user(i, &data->InvokeCamera);
	err |= get_user(i, &data32->FeatureId);
	err |= put_user(i, &data->FeatureId);
	err |= get_user(p, &data32->pFeaturePara);
	err |= put_user(compat_ptr(p), &data->pFeaturePara);
	err |= get_user(p, &data32->pFeatureParaLen);
	err |= put_user(compat_ptr(p), &data->pFeatureParaLen);
	return err;
}

static int compat_put_acdk_sensor_featurecontrol_struct(
	struct COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT
	__user *data32,
	struct ACDK_SENSOR_FEATURECONTROL_STRUCT __user *
	data)
{
	MUINT8 *p;
	MUINT32 *q;
	compat_uint_t i;
	int err;

	err = get_user(i, &data->InvokeCamera);
	err |= put_user(i, &data32->InvokeCamera);
	err |= get_user(i, &data->FeatureId);
	err |= put_user(i, &data32->FeatureId);
	/* Assume pointer is not change */

	err |= get_user(p, &data->pFeaturePara);
	err |= put_user(ptr_to_compat(p), &data32->pFeaturePara);
	err |= get_user(q, &data->pFeatureParaLen);
	err |= put_user(ptr_to_compat(q), &data32->pFeatureParaLen);

	return err;
}

static int compat_get_acdk_sensor_control_struct(
	struct COMPAT_ACDK_SENSOR_CONTROL_STRUCT __user *data32,
	struct ACDK_SENSOR_CONTROL_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->InvokeCamera);
	err |= put_user(i, &data->InvokeCamera);
	err |= get_user(i, &data32->ScenarioId);
	err |= put_user(i, &data->ScenarioId);
	err |= get_user(p, &data32->pImageWindow);
	err |= put_user(compat_ptr(p), &data->pImageWindow);
	err |= get_user(p, &data32->pSensorConfigData);
	err |= put_user(compat_ptr(p), &data->pSensorConfigData);
	return err;
}

static int compat_put_acdk_sensor_control_struct(
	struct COMPAT_ACDK_SENSOR_CONTROL_STRUCT __user *data32,
	struct ACDK_SENSOR_CONTROL_STRUCT __user *data)
{
	/* compat_uptr_t p; */
	compat_uint_t i;
	int err;

	err = get_user(i, &data->InvokeCamera);
	err |= put_user(i, &data32->InvokeCamera);
	err |= get_user(i, &data->ScenarioId);
	err |= put_user(i, &data32->ScenarioId);

	return err;
}

static long
imgsensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_KDIMGSENSORIOC_X_GETINFO:
	{
		struct COMPAT_IMGSENSOR_GET_CONFIG_INFO_STRUCT __user *data32;
		struct IMGSENSOR_GET_CONFIG_INFO_STRUCT __user *data;
		int err;
		/*PK_DBG("CAOMPAT_KDIMGSENSORIOC_X_GETINFO E\n"); */

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_acdk_sensor_getinfo_struct(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(
		    filp,
		    KDIMGSENSORIOC_X_GET_CONFIG_INFO,
		    (unsigned long)data);

		err = compat_put_acdk_sensor_getinfo_struct(data32, data);

		if (err != 0)
			PK_DBG(
			    "[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	case COMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL:
	{
		struct COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT __user *data32;
		struct ACDK_SENSOR_FEATURECONTROL_STRUCT __user *data;
		int err;

		/* PK_DBG("CAOMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL\n"); */

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_acdk_sensor_featurecontrol_struct(
		    data32,
		    data);

		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(
		    filp,
		    KDIMGSENSORIOC_X_FEATURECONCTROL,
		    (unsigned long)data);

		err = compat_put_acdk_sensor_featurecontrol_struct(
		    data32,
		    data);

		if (err != 0)
			PK_DBG(
			    "[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	case COMPAT_KDIMGSENSORIOC_X_CONTROL:
	{
		struct COMPAT_ACDK_SENSOR_CONTROL_STRUCT __user *data32;
		struct ACDK_SENSOR_CONTROL_STRUCT __user *data;
		int err;

		PK_DBG("[CAMERA SENSOR] CAOMPAT_KDIMGSENSORIOC_X_CONTROL\n");

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_acdk_sensor_control_struct(data32, data);
		if (err)
			return err;
		ret = filp->f_op->unlocked_ioctl(
		    filp,
		    KDIMGSENSORIOC_X_CONTROL,
		    (unsigned long)data);

		err = compat_put_acdk_sensor_control_struct(data32, data);

		if (err != 0)
			PK_DBG(
			    "[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	case COMPAT_KDIMGSENSORIOC_X_GETINFO2:
	{
		struct COMPAT_IMAGESENSOR_GETINFO_STRUCT __user *data32;
		struct IMAGESENSOR_GETINFO_STRUCT __user *data;
		int err;

		PK_DBG("[CAMERA SENSOR] CAOMPAT_KDIMGSENSORIOC_X_GETINFO2\n");

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_imagesensor_getinfo_struct(data32, data);
		if (err)
			return err;
		ret = filp->f_op->unlocked_ioctl(
		    filp,
		    KDIMGSENSORIOC_X_GETINFO2,
		    (unsigned long)data);

		err = compat_put_imagesensor_getinfo_struct(data32, data);

		if (err != 0)
			PK_DBG(
			    "[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
		return ret;
	}
	case COMPAT_KDIMGSENSORIOC_X_GETRESOLUTION2:
	{
		return 0;
	}

	default:
		return filp->f_op->unlocked_ioctl(filp, cmd, arg);
	}
}
#endif

static long imgsensor_ioctl(
	struct file *a_pstFile,
	unsigned int a_u4Command,
	unsigned long a_u4Param)
{
	int i4RetValue = 0;
	void *pBuff = NULL;

	if (_IOC_DIR(a_u4Command) != _IOC_NONE) {
		pBuff = kmalloc(_IOC_SIZE(a_u4Command), GFP_KERNEL);
		if (pBuff == NULL) {
			/*
			 * PK_DBG("[CAMERA SENSOR] ioctl allocate
			 * mem failed\n");
			 */
			i4RetValue = -ENOMEM;
			goto CAMERA_HW_Ioctl_EXIT;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user(
			    pBuff,
			    (void *)a_u4Param,
			    _IOC_SIZE(a_u4Command))) {
				PK_DBG(
				    "[CAMERA SENSOR] ioctl copy from user failed\n");
				i4RetValue =  -EFAULT;
				goto CAMERA_HW_Ioctl_EXIT;
			}
		} else
			memset(pBuff, 0, _IOC_SIZE(a_u4Command));
	} else {
		i4RetValue =  -EFAULT;
		goto CAMERA_HW_Ioctl_EXIT;
	}

	switch (a_u4Command) {
	case KDIMGSENSORIOC_X_GET_CONFIG_INFO:
		i4RetValue = adopt_CAMERA_HW_GetInfo(pBuff);
		break;
	case KDIMGSENSORIOC_X_GETINFO2:
		i4RetValue = adopt_CAMERA_HW_GetInfo2(pBuff);
		break;
	case KDIMGSENSORIOC_X_FEATURECONCTROL:
		i4RetValue = adopt_CAMERA_HW_FeatureControl(pBuff);
		break;
	case KDIMGSENSORIOC_X_CONTROL:
		i4RetValue = adopt_CAMERA_HW_Control(pBuff);
		break;
	case KDIMGSENSORIOC_X_SET_MCLK_PLL:
		i4RetValue = imgsensor_clk_set(
		    &pgimgsensor->clk,
		    (struct ACDK_SENSOR_MCLK_STRUCT *)pBuff);
		break;
	case KDIMGSENSORIOC_X_GET_ISP_CLK:
/*E1(High):490, (Medium):364, (low):273*/
#define ISP_CLK_LOW    273
#define ISP_CLK_MEDIUM 364
#define ISP_CLK_HIGH   490
#ifdef CONFIG_MTK_SMI_EXT
		PK_DBG(
		"KDIMGSENSORIOC_X_GET_ISP_CLK current_mmsys_clk=%d\n",
		current_mmsys_clk);

		if (mmdvfs_get_stable_isp_clk() == MMSYS_CLK_HIGH)
			*(unsigned int *)pBuff = ISP_CLK_HIGH;
		else if (mmdvfs_get_stable_isp_clk() == MMSYS_CLK_MEDIUM)
			*(unsigned int *)pBuff = ISP_CLK_MEDIUM;
		else
			*(unsigned int *)pBuff = ISP_CLK_LOW;
#else
		*(unsigned int *)pBuff = ISP_CLK_HIGH;
#endif
		break;
	case KDIMGSENSORIOC_X_GET_CSI_CLK:
		i4RetValue = imgsensor_clk_ioctrl_handler(pBuff);
		break;

	/*mmdvfs start*/
#ifdef IMGSENSOR_DFS_CTRL_ENABLE
	case KDIMGSENSORIOC_DFS_UPDATE:
		i4RetValue = imgsensor_dfs_ctrl(DFS_UPDATE, pBuff);
		break;
	case KDIMGSENSORIOC_GET_SUPPORTED_ISP_CLOCKS:
		i4RetValue = imgsensor_dfs_ctrl(
						DFS_SUPPORTED_ISP_CLOCKS,
						pBuff);
		break;
	case KDIMGSENSORIOC_GET_CUR_ISP_CLOCK:
		i4RetValue = imgsensor_dfs_ctrl(DFS_CUR_ISP_CLOCK, pBuff);
		break;
#endif
	/*mmdvfs end*/
	case KDIMGSENSORIOC_T_OPEN:
	case KDIMGSENSORIOC_T_CLOSE:
	case KDIMGSENSORIOC_T_CHECK_IS_ALIVE:
	case KDIMGSENSORIOC_X_SET_DRIVER:
	case KDIMGSENSORIOC_X_GETRESOLUTION2:
	case KDIMGSENSORIOC_X_GET_SOCKET_POS:
	case KDIMGSENSORIOC_X_SET_GPIO:
	case KDIMGSENSORIOC_X_SET_I2CBUS:
	case KDIMGSENSORIOC_X_RELEASE_I2C_TRIGGER_LOCK:
	case KDIMGSENSORIOC_X_SET_SHUTTER_GAIN_WAIT_DONE:
	case KDIMGSENSORIOC_X_SET_CURRENT_SENSOR:
		i4RetValue = 0;
		break;

//zhongzhu add start,initialize imgsensor_custom_config
	case TCT_IMGSENSORIOC_X_CONTROL:
		i4RetValue = TCT_CAMERA_HW_Control(pBuff);
		break;
//zhongzhu add end,initialize imgsensor_custom_config

	default:
		PK_DBG("No such command %d\n", a_u4Command);
		i4RetValue = -EPERM;
		goto CAMERA_HW_Ioctl_EXIT;
		break;
	}

	if ((_IOC_READ & _IOC_DIR(a_u4Command)) &&
		    copy_to_user((void __user *) a_u4Param,
						  pBuff,
						_IOC_SIZE(a_u4Command))) {
		PK_DBG("[CAMERA SENSOR] ioctl copy to user failed\n");
		i4RetValue =  -EFAULT;
		goto CAMERA_HW_Ioctl_EXIT;
	}

CAMERA_HW_Ioctl_EXIT:
	if (pBuff != NULL) {
		kfree(pBuff);
		pBuff = NULL;
	}
	return i4RetValue;
}

static int imgsensor_open(struct inode *a_pstInode, struct file *a_pstFile)
{
	mutex_lock(&imgsensor_mutex);

	if (atomic_read(&pgimgsensor->imgsensor_open_cnt) == 0)
		imgsensor_clk_enable_all(&pgimgsensor->clk);

	atomic_inc(&pgimgsensor->imgsensor_open_cnt);
	pr_info(
	    "%s %d\n",
	    __func__,
	    atomic_read(&pgimgsensor->imgsensor_open_cnt));

	mutex_unlock(&imgsensor_mutex);

	return 0;
}

static int imgsensor_release(struct inode *a_pstInode, struct file *a_pstFile)
{
	enum IMGSENSOR_SENSOR_IDX i = IMGSENSOR_SENSOR_IDX_MIN_NUM;

	mutex_lock(&imgsensor_mutex);

	atomic_dec(&pgimgsensor->imgsensor_open_cnt);
	if (atomic_read(&pgimgsensor->imgsensor_open_cnt) == 0) {
		imgsensor_clk_disable_all(&pgimgsensor->clk);

		if (pgimgsensor->imgsensor_oc_irq_enable != NULL) {
			for (; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++)
				pgimgsensor->imgsensor_oc_irq_enable(i, false);
		}

		imgsensor_hw_release_all(&pgimgsensor->hw);
#ifdef IMGSENSOR_DFS_CTRL_ENABLE
		imgsensor_dfs_ctrl(DFS_RELEASE, NULL);
#endif
	}
	pr_info(
	    "%s %d\n",
	    __func__,
	    atomic_read(&pgimgsensor->imgsensor_open_cnt));

	mutex_unlock(&imgsensor_mutex);

	return 0;
}

static const struct file_operations gimgsensor_file_operations = {
	.owner = THIS_MODULE,
	.open = imgsensor_open,
	.release = imgsensor_release,
	.unlocked_ioctl = imgsensor_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = imgsensor_compat_ioctl
#endif
};

static inline int imgsensor_driver_register(void)
{
	dev_t dev_no = MKDEV(IMGSENSOR_DEVICE_NNUMBER, 0);

	if (alloc_chrdev_region(&dev_no, 0, 1, IMGSENSOR_DEV_NAME)) {
		PK_DBG("[CAMERA SENSOR] Allocate device no failed\n");

		return -EAGAIN;
	}

	/* Allocate driver */
	gpimgsensor_cdev = cdev_alloc();
	if (gpimgsensor_cdev ==  NULL) {
		unregister_chrdev_region(dev_no, 1);
		PK_DBG("[CAMERA SENSOR] Allocate mem for kobject failed\n");
		return -ENOMEM;
	}

	/* Attatch file operation. */
	cdev_init(gpimgsensor_cdev, &gimgsensor_file_operations);

	gpimgsensor_cdev->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(gpimgsensor_cdev, dev_no, 1)) {
		PK_DBG("Attatch file operation failed\n");
		unregister_chrdev_region(dev_no, 1);
		return -EAGAIN;
	}

	gpimgsensor_class = class_create(THIS_MODULE, "sensordrv");
	if (IS_ERR(gpimgsensor_class)) {
		int ret = PTR_ERR(gpimgsensor_class);

		PK_DBG("Unable to create class, err = %d\n", ret);
		return ret;
	}

	gimgsensor_device =
	    device_create(
		    gpimgsensor_class,
		    NULL,
		    dev_no,
		    NULL,
		    IMGSENSOR_DEV_NAME);

	return 0;
}

static inline void imgsensor_driver_unregister(void)
{
	/* Release char driver */
	cdev_del(gpimgsensor_cdev);

	unregister_chrdev_region(MKDEV(IMGSENSOR_DEVICE_NNUMBER, 0), 1);

	device_destroy(gpimgsensor_class, MKDEV(IMGSENSOR_DEVICE_NNUMBER, 0));
	class_destroy(gpimgsensor_class);
}

#ifdef CONFIG_MTK_SMI_EXT
int mmsys_clk_change_cb(int ori_clk_mode, int new_clk_mode)
{
	/*PK_DBG("mmsys_clk_change_cb ori:%d, new:%d, current_mmsys_clk %d\n",
	 *		ori_clk_mode,
	 *		new_clk_mode,
	 *		current_mmsys_clk);
	 */
	current_mmsys_clk = new_clk_mode;
	return 1;
}
#endif

static int imgsensor_probe(struct platform_device *pdev)
{
	/* Register char driver */
	if (imgsensor_driver_register()) {
		PK_DBG("[CAMERA_HW] register char device failed!\n");
		return -1;
	}

	gpimgsensor_hw_platform_device = pdev;

#ifndef CONFIG_FPGA_EARLY_PORTING
	imgsensor_clk_init(&pgimgsensor->clk);
#endif
//zhongzhu add start,initialize imgsensor_custom_config
	tctimgsensor_init();
//zhongzhu add end,initialize imgsensor_custom_config
	imgsensor_hw_init(&pgimgsensor->hw);
	imgsensor_i2c_create();
	imgsensor_proc_init();

	atomic_set(&pgimgsensor->imgsensor_open_cnt, 0);
#ifdef CONFIG_MTK_SMI_EXT
	mmdvfs_register_mmclk_switch_cb(
	    mmsys_clk_change_cb,
	    MMDVFS_CLIENT_ID_ISP);
#endif

	return 0;
}

static int imgsensor_remove(struct platform_device *pdev)
{
	imgsensor_i2c_delete();
	imgsensor_driver_unregister();

	return 0;
}

static int imgsensor_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int imgsensor_resume(struct platform_device *pdev)
{
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id gimgsensor_of_device_id[] = {
	{ .compatible = "mediatek,camera_hw", },
	{}
};
#endif

static struct platform_driver gimgsensor_platform_driver = {
	.probe      = imgsensor_probe,
	.remove     = imgsensor_remove,
	.suspend    = imgsensor_suspend,
	.resume     = imgsensor_resume,
	.driver     = {
		.name   = "image_sensor",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gimgsensor_of_device_id,
#endif
	}
};

static int __init imgsensor_init(void)
{
	pr_info("[camerahw_probe] start\n");

	if (platform_driver_register(&gimgsensor_platform_driver)) {
		PK_DBG("failed to register CAMERA_HW driver\n");
		return -ENODEV;
	}

	/*prevent imgsensor race condition in vulunerbility test*/
	mutex_init(&imgsensor_mutex);

#ifdef CONFIG_CAM_TEMPERATURE_WORKQUEUE
	memset((void *)&cam_temperature_wq, 0, sizeof(cam_temperature_wq));
	INIT_DELAYED_WORK(
	    &cam_temperature_wq,
	    cam_temperature_report_wq_routine);

	schedule_delayed_work(&cam_temperature_wq, HZ);
#endif
#ifdef IMGSENSOR_DFS_CTRL_ENABLE
	imgsensor_dfs_ctrl(DFS_CTRL_ENABLE, NULL);
#endif

	return 0;
}

static void __exit imgsensor_exit(void)
{
#ifdef IMGSENSOR_DFS_CTRL_ENABLE
	imgsensor_dfs_ctrl(DFS_CTRL_DISABLE, NULL);
#endif
	platform_driver_unregister(&gimgsensor_platform_driver);
}

late_initcall(imgsensor_init);
module_exit(imgsensor_exit);

MODULE_DESCRIPTION("image sensor driver");
MODULE_AUTHOR("Mediatek");
MODULE_LICENSE("GPL v2");


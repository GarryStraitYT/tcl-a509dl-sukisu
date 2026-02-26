// SPDX-License-Identifier: GPL-2.0

#include "regulator.h"
//#include "upmu_common.h"


#include <mt-plat/aee.h>

#include <asm/siginfo.h>

#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/regulator/driver.h>
#include "../../../../../../../../drivers/regulator/internal.h"
#include "kd_imgsensor.h"
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/sched/signal.h>

//#include <mt-plat/upmu_common.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/mfd/mt6357/registers.h>

static struct REGULATOR *preg_own;
static bool Is_Notify_call[IMGSENSOR_SENSOR_IDX_MAX_NUM][REGULATOR_TYPE_MAX_NUM];

struct reg_oc_debug_t {
	const char *name;
	struct notifier_block nb;
	struct regulator *regulator;
	struct work_struct work;
	unsigned int times;
	unsigned int md_reg_idx;
	bool is_md_reg;
};

static struct reg_oc_debug_t
	reg_oc_debug[IMGSENSOR_SENSOR_IDX_MAX_NUM][REGULATOR_TYPE_MAX_NUM];

static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1050,
	REGULATOR_VOLTAGE_1100,
	REGULATOR_VOLTAGE_1200,
	REGULATOR_VOLTAGE_1210,
	REGULATOR_VOLTAGE_1220,
	REGULATOR_VOLTAGE_1500,
	REGULATOR_VOLTAGE_1800,
	REGULATOR_VOLTAGE_2500,
	REGULATOR_VOLTAGE_2800,
	REGULATOR_VOLTAGE_2900,
};

struct REGULATOR_CTRL regulator_control[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama"},
	{"vcama1"},
	{"vcamd"},
	{"vcamio"},
};
static int flag=0;
//begin 20200411 liujunting add for camera sensor regulator
static int regulator_status[IMGSENSOR_SENSOR_IDX_MAX_NUM][REGULATOR_TYPE_MAX_NUM] = {0};
static void check_for_regulator_get(struct REGULATOR *preg, struct device *pdevice, enum IMGSENSOR_SENSOR_IDX sensor_idx, int index);
static void check_for_regulator_put(struct REGULATOR *preg, enum IMGSENSOR_SENSOR_IDX sensor_idx, int index);
static struct device_node *of_node_record = NULL;
static DEFINE_MUTEX(g_regulator_state_mutex);
//end 20200411 liujunting add for camera sensor regulator

static struct REGULATOR reg_instance;
struct regmap *g_regmap;
static int regulator_oc_notify(
	struct notifier_block *nb, unsigned long event, void *data)
{
		struct reg_oc_debug_t *reg_oc_dbg =
			container_of(nb, struct reg_oc_debug_t, nb);

		if (event != REGULATOR_EVENT_OVER_CURRENT)
			return NOTIFY_OK;

		/* Do OC handling */
		pr_info("Imgsensor OC notify regulator: %s OC pid %ld\n",
			reg_oc_dbg->name, (long)reg_instance.pid);

		gimgsensor.status.oc = 1;
		aee_kernel_warning("Imgsensor OC", "Over current");
		if (reg_instance.pid != -1 &&
		pid_task(find_get_pid(reg_instance.pid), PIDTYPE_PID) != NULL) {
			force_sig(SIGKILL,
				pid_task(find_get_pid(reg_instance.pid),
				PIDTYPE_PID));
		}
		return NOTIFY_OK;
}

#define OC_MODULE "camera"
enum IMGSENSOR_RETURN imgsensor_oc_interrupt(
	enum IMGSENSOR_SENSOR_IDX sensor_idx, bool enable)
{
	int i = 0;
	int ret = 0;

	mutex_lock(&oc_mutex);
	if (enable) {
		mdelay(5);
		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			if (preg_own->pregulator[sensor_idx][i] &&
					regulator_is_enabled(preg_own->pregulator[sensor_idx][i]) &&
					!Is_Notify_call[sensor_idx][i]
				) {
				/* oc notifier callback function */
				reg_oc_debug[sensor_idx][i].name =
					regulator_control[i].pregulator_type;
				reg_oc_debug[sensor_idx][i].regulator =
					preg_own->pregulator[sensor_idx][i];
				reg_oc_debug[sensor_idx][i].nb.notifier_call =
					regulator_oc_notify;
				ret = devm_regulator_register_notifier(
					preg_own->pregulator[sensor_idx][i],
					&reg_oc_debug[sensor_idx][i].nb);
				Is_Notify_call[sensor_idx][i] = true;

				if (ret) {
					pr_info(
					"regulator notifier request error\n");
				}
				pr_debug(
					"[regulator] %s idx=%d %s enable=%d oc enabled\n",
					__func__,
					sensor_idx,
					regulator_control[i].pregulator_type,
					enable);
			}
		}
		rcu_read_lock();
		reg_instance.pid = current->tgid;
		rcu_read_unlock();
	} else {
		reg_instance.pid = -1;
		/* Disable interrupt before power off */

		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			if (preg_own->pregulator[sensor_idx][i] &&
				regulator_is_enabled(preg_own->pregulator[sensor_idx][i]) &&
				Is_Notify_call[sensor_idx][i]
				) {
				/* oc notifier callback function */
				devm_regulator_unregister_notifier(
					preg_own->pregulator[sensor_idx][i],
					&reg_oc_debug[sensor_idx][i].nb);
				Is_Notify_call[sensor_idx][i] = false;
				pr_info("Unregister OC notifier");
			}
		}

	}
	mutex_unlock(&oc_mutex);
	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_oc_init(void)
{
	/* Register your interrupt handler of OC interrupt at first */

	gimgsensor.status.oc  = 0;
	gimgsensor.imgsensor_oc_irq_enable = imgsensor_oc_interrupt;
	reg_instance.pid = -1;

	return IMGSENSOR_RETURN_SUCCESS;
}


static enum IMGSENSOR_RETURN regulator_init(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	struct device            *pdevice;
	struct device_node       *pof_node;
	struct device_node	 *pmic_node;
	struct platform_device	 *pmic_pdev;
	struct mt6397_chip	 *chip;
	int j, i;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	pdevice  = gimgsensor_device;
	pof_node = pdevice->of_node;
	pdevice->of_node =
		of_find_compatible_node(NULL, NULL, "mediatek,camera_hw");

	//begin 20200411 liujunting add for camera sensor regulator
	of_node_record = pdevice->of_node;
	//end 20200411 liujunting add for camera sensor regulator

	if (pdevice->of_node == NULL) {
		pr_err("regulator get cust camera node failed!\n");
		pdevice->of_node = pof_node;
		return IMGSENSOR_RETURN_ERROR;
	}

	/* Begin binchang.liang for camera sensor regulator 2021/11/10 */
	pmic_node = of_parse_phandle(pdevice->of_node, "pmic", 0);
	if (!pmic_node) {
		pr_info("regulator get pmic_node fail!\n");
		return IMGSENSOR_RETURN_ERROR;
	}
	pmic_pdev = of_find_device_by_node(pmic_node);
	if (!pmic_pdev) {
		pr_info("get pmic_pdev fail!\n");
		return IMGSENSOR_RETURN_ERROR;
	}
	chip = dev_get_drvdata(&(pmic_pdev->dev));
	if (!chip) {
		pr_info("get chip fail!\n");
		return IMGSENSOR_RETURN_ERROR;
	}
	g_regmap = chip->regmap;
	if (IS_ERR_VALUE(g_regmap)) {
		g_regmap = NULL;
		pr_info("get g_regmap fail\n");
	}
	/* End binchang.liang for camera sensor regulator 2021/11/10 */

	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {
		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			snprintf(str_regulator_name,
					sizeof(str_regulator_name),
					"cam%d_%s",
					j,
					regulator_control[i].pregulator_type);
			preg->pregulator[j][i] =
			    regulator_get_optional(
				pdevice, str_regulator_name);
			if (IS_ERR(preg->pregulator[j][i]))
				preg->pregulator[j][i] = NULL;
			if (preg->pregulator[j][i] == NULL)
				pr_err("regulator[%d][%d]  %s fail!\n",
					j, i, str_regulator_name);

			atomic_set(&preg->enable_cnt[j][i], 0);

			//begin 20200411 liujunting add for camera sensor regulator
			regulator_status[j][i] = 1;
			//end 20200411 liujunting add for camera sensor regulator

		}
	}
	pdevice->of_node = pof_node;
	imgsensor_oc_init();
	preg_own = (struct REGULATOR *)pinstance;
	return IMGSENSOR_RETURN_SUCCESS;
}
static enum IMGSENSOR_RETURN regulator_release(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	struct regulator *pregulator = NULL;
	atomic_t *enable_cnt = NULL;

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {

		for (type = 0; type < REGULATOR_TYPE_MAX_NUM; type++) {
			pregulator = preg->pregulator[idx][type];
			enable_cnt = &preg->enable_cnt[idx][type];
			if (pregulator != NULL) {
				for (; atomic_read(enable_cnt) > 0; ) {
					regulator_disable(pregulator);
					atomic_dec(enable_cnt);
				}
			}
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct regulator     *pregulator;
	struct REGULATOR     *preg = (struct REGULATOR *)pinstance;
	int reg_type_offset;
	int regval;
	int regtemp;
	atomic_t             *enable_cnt;

	if (pin > IMGSENSOR_HW_PIN_DOVDD   ||
		pin < IMGSENSOR_HW_PIN_AVDD    ||
		pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
		pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH ||
		sensor_idx < 0){

		return IMGSENSOR_RETURN_ERROR;

	}

	reg_type_offset = REGULATOR_TYPE_VCAMA;

	//begin 20200411 liujunting add for camera sensor regulator
	check_for_regulator_get(preg, gimgsensor_device, sensor_idx, (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
	//end 20200411 liujunting add for camera sensor regulator

	pregulator =
		preg->pregulator[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	enable_cnt =
		&preg->enable_cnt[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	if (pregulator) {

		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {

			/* Begin majinrui for camera sensor regulator 2021/11/17 */
			unsigned int voltage = regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0];
			pr_info("tcl_setv  now  power up   sensor_idx=%d \n", sensor_idx);
			pr_info("test DVDD current voltage=%d \n", voltage);

			if (voltage == REGULATOR_VOLTAGE_1050 ) {

				// --------------------------  add for +- 50mV dvdd  --------------------------
				pr_info(" tcl_setv sensor vcamd to %dmV\n", voltage/1000);
				regmap_write(g_regmap, MT6357_VCAMD_ANA_CON0, 0x405);// 1050 mV
				regmap_read(g_regmap, MT6357_VCAMD_ANA_CON0, &regval);

				if (regval == 0x405){
					flag =1;
					pr_info("tcl_setv set vcamd to %dmV success!\n", voltage/1000);
				}
				// --------------------------  add for +- 50mV dvdd   end --------------------------

			}else {

				//fix steps in the dvdd voltage when  porwerdown . Remove the extra 50 mV
				// --------------------------  Remove the extra 50 mV dvdd  --------------------------
				if(flag == 0){

					//pr_info("Remove the extra 50 mV  sensor_idx=%d \n", sensor_idx);
					regmap_read(g_regmap, MT6357_VCAMD_ANA_CON0,&regval);
					//pr_info("MT6357_VCAMD_ANA_CON0 = %x \n",regval);
					regtemp = regval & 0x0f;

					if (regtemp != 0){
						regmap_write(g_regmap, MT6357_VCAMD_ANA_CON0,regval&0xfff0);// last 4 bit to 0000
						regmap_read(g_regmap, MT6357_VCAMD_ANA_CON0,&regval);
						//pr_info("MT6357_VCAMD_ANA_CON0 = %x \n",regval);
						if ((regval & 0x0f) == 0){
							pr_info("Remove the extra 50 mV  success!\n");
						}
					}
				}
				// --------------------------  Remove the extra 50 mV dvdd   end --------------------------

				if (regulator_set_voltage(pregulator,
					regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
					regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

					pr_err("[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
						pin,regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				}
			}
			/* End majinrui for camera sensor regulator 2021/11/17 */

			if (regulator_enable(pregulator)) {

				pr_err("[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
					pin, regulator_voltage[pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);

				//begin 20200411 liujunting add for camera sensor regulator
				check_for_regulator_put(preg, sensor_idx, (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
				//end 20200411 liujunting add for camera sensor regulator

				return IMGSENSOR_RETURN_ERROR;
			}

			atomic_inc(enable_cnt);

		} else {

			flag=0;

			if (regulator_is_enabled(pregulator)) {
				/*pr_debug("[regulator]%d is enabled\n", pin);*/

				if (regulator_disable(pregulator)) {

					pr_err("[regulator]fail to regulator_disable, powertype: %d\n", pin);

					//begin 20200411 liujunting add for camera sensor regulator
					check_for_regulator_put(preg, sensor_idx, (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
					//end 20200411 liujunting add for camera sensor regulator

					return IMGSENSOR_RETURN_ERROR;
				}
			}

			//begin 20200411 liujunting add for camera sensor regulator
			check_for_regulator_put(preg, sensor_idx, (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
			//end 20200411 liujunting add for camera sensor regulator

			atomic_dec(enable_cnt);

		}
	} else {
		pr_err("regulator == NULL %d %d %d\n", reg_type_offset, pin, IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.pinstance = (void *)&reg_instance,
	.init      = regulator_init,
	.set       = regulator_set,
	.release   = regulator_release,
	.id        = IMGSENSOR_HW_ID_REGULATOR
};

enum IMGSENSOR_RETURN imgsensor_hw_regulator_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

//begin 20200411 liujunting add for camera sensor regulator
//#if 0
static void check_for_regulator_get(struct REGULATOR *preg, struct device *pdevice, enum IMGSENSOR_SENSOR_IDX sensor_idx, int index)
{
	struct device_node *pof_node;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	if (!preg || !pdevice) {
		pr_err("Fatal: Null ptr.preg:%pK,pdevice:%pK\n", preg, pdevice);
		return;
	}

	if (sensor_idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM || index >= REGULATOR_TYPE_MAX_NUM ) {
		pr_err("[%s]Invalid sensor_idx:%d regulator_idx: %d\n", __func__, sensor_idx, index);
		return;
	}

	mutex_lock(&g_regulator_state_mutex);
	if(regulator_status[sensor_idx][index]==0)
	{
		pof_node = pdevice->of_node;
		pdevice->of_node = of_node_record;

		snprintf(str_regulator_name, sizeof(str_regulator_name), "cam%d_%s", sensor_idx, 
                        regulator_control[index].pregulator_type);

		preg->pregulator[sensor_idx][index] = regulator_get(pdevice, str_regulator_name);

		pdevice->of_node = pof_node;

		regulator_status[sensor_idx][index] = 1;
		//pr_err("regulator_dbg regulator_get %s, of_node:%p\n", regulator_control[index].pregulator_type, of_node_record);
	}
	mutex_unlock(&g_regulator_state_mutex);
	return;
}

static void check_for_regulator_put(struct REGULATOR *preg, enum IMGSENSOR_SENSOR_IDX sensor_idx, int index)
{
	//struct device_node *pof_node;
	//char str_regulator_name[LENGTH_FOR_SNPRINTF];

	if (!preg) {
		pr_err("Fatal: Null ptr\n");
		return;
	}

	if (sensor_idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM || index >= REGULATOR_TYPE_MAX_NUM ) {
		pr_err("[%s]Invalid sensor_idx:%d regulator_idx: %d\n", __func__, sensor_idx, index);
		return;
	}

	mutex_lock(&g_regulator_state_mutex);
	if(regulator_status[sensor_idx][index]==1)
	{
		//pof_node = pdevice->of_node;
		//pdevice->of_node = of_node_record;

		//snprintf(str_regulator_name, sizeof(str_regulator_name), "cam%d_%s", sensor_idx, 
                        //regulator_control[index].pregulator_type);

		//preg->pregulator[sensor_idx][index] = regulator_get(pdevice, str_regulator_name);
		regulator_put(preg->pregulator[sensor_idx][index]);
                preg->pregulator[sensor_idx][index] = NULL;

		//pdevice->of_node = pof_node;
		regulator_status[sensor_idx][index]=0;
		//pr_err("regulator_dbg regulator_put %s\n", regulator_control[index].pregulator_type);
	}
	mutex_unlock(&g_regulator_state_mutex);
	return;
}
//#endif
//end 20200411 liujunting add for camera sensor regulator

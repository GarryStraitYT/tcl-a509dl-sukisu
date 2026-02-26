// SPDX-License-Identifier: GPL-2.0

#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
//Begin add by bing-zhang for SNTTF-3211 on 2022/07/20
#include <linux/delay.h>
//End add by bing-zhang for SNTTF-3211 on 2022/07/20

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;
static int regulator_inited;

int display_bias_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_info("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_info("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}
EXPORT_SYMBOL(display_bias_regulator_init);

int disp_late_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
		ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
		ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(disp_late_bias_enable);

//Begin add by yan.gong for CIVIC-3104 on 2022-07-25
#if defined (CONFIG_TCT_PROJECT_CIVIC_S) || defined (CONFIG_TCT_PROJECT_CIVIC_PLUS_S)
int display_bias_enable_VSP(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 6000000, 6000000);
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;
	return retval;
}
EXPORT_SYMBOL(display_bias_enable_VSP);

int display_bias_enable_VSN(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/

	ret = regulator_set_voltage(disp_bias_neg, 6000000, 6000000);
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable_VSN);
#endif
//End add by yan.gong for CIVIC-3104 on 2022-07-25

//Begin add by bing-zhang for SNTTF-3211 on 2022/07/20
#if defined(CONFIG_TCT_PROJECT_SONATA)
int display_bias_vsp_setting(void)
{
	int ret = 0;
	int retval = 0;

	pr_info("Set bias vsp\n");
	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5900000, 5900000);
	if (ret < 0)
		pr_info("!!!set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_vsp_setting);

int display_bias_vsn_setting(void)
{
	int ret = 0;
	int retval = 0;

	pr_info("Set bias vsn\n");
	ret = regulator_set_voltage(disp_bias_neg, 5900000, 5900000);
	if (ret < 0)
		pr_info("@@@set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_vsn_setting);
#endif
//End add by bing-zhang for SNTTF-3211 on 2022/07/20

int display_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
//Begin modified by liangjiaqiang for MODEL3-1880 on 2022-09-08
#ifdef CONFIG_TCT_PROJECT_MODEL_3
	ret = regulator_set_voltage(disp_bias_pos, 6000000, 6000000);
#else
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
#endif
//End modified by liangjiaqiang for MODEL3-1880 on 2022-09-08
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

//Begin modified by liangjiaqiang for MODEL3-1880 on 2022-09-08
#ifdef CONFIG_TCT_PROJECT_MODEL_3
	ret = regulator_set_voltage(disp_bias_neg, 6000000, 6000000);
#else
	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
#endif
//End modified by liangjiaqiang for MODEL3-1880 on 2022-09-08
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable);

//Begin added by liangjiaqiang for MODEL3-1880 on 2022-09-08
#ifdef CONFIG_TCT_PROJECT_MODEL_3
int display_bias_enable_VSP(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 6000000, 6000000);
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;
	return retval;
}
EXPORT_SYMBOL(display_bias_enable_VSP);

int display_bias_enable_VSN(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	/* set voltage with min & max*/

	ret = regulator_set_voltage(disp_bias_neg, 6000000, 6000000);
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_enable_VSN);
#endif
//End added by liangjiaqiang for MODEL3-1880 on 2022-09-08

int display_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	display_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_info("disable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	//Begin add by bing-zhang for SNTTF-3211 on 2022/07/20
	msleep(5);
	//End add by bing-zhang for SNTTF-3211 on 2022/07/20
	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_info("disable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}
EXPORT_SYMBOL(display_bias_disable);

//Begin add by dingting.meng for T-11334387 on 2021.07.27
int tct_lcm_pmu_bias_set(int min_uV, int max_uV){
        int ret = 0;
        int retval = 0;

    printk("#### %s | %d ####\n",__func__,__LINE__);

        display_bias_regulator_init();

        /* set voltage with min & max*/
        ret = regulator_set_voltage(disp_bias_pos, min_uV, max_uV);
        if (ret < 0)
                pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
        retval |= ret;

        ret = regulator_set_voltage(disp_bias_neg, min_uV, max_uV);
        if (ret < 0)
                pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
        retval |= ret;

        /* enable regulator */
        ret = regulator_enable(disp_bias_pos);
        if (ret < 0)
                pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
                        ret);
        retval |= ret;

        ret = regulator_enable(disp_bias_neg);
        if (ret < 0)
                pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
                        ret);
        retval |= ret;

        return retval;
}
EXPORT_SYMBOL(tct_lcm_pmu_bias_set);

int tct_lcm_pmu_bias_unset(void){
        int ret = 0;
        int retval = 0;

    printk("#### %s | %d ####\n",__func__,__LINE__);
        display_bias_regulator_init();

        ret = regulator_disable(disp_bias_neg);
        if (ret < 0)
                pr_info("disable regulator disp_bias_neg fail, ret = %d\n",
                        ret);
        retval |= ret;

        ret = regulator_disable(disp_bias_pos);
        if (ret < 0)
                pr_info("disable regulator disp_bias_pos fail, ret = %d\n",
                        ret);
        retval |= ret;

        return retval;
}
EXPORT_SYMBOL(tct_lcm_pmu_bias_unset);
//End add by dingting.meng for T-11334387 on 2021.07.27

#else
int display_bias_regulator_init(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_regulator_init);

int display_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_enable);

int disp_late_bias_enable(void)
{
	return 0;
}
EXPORT_SYMBOL(disp_late_bias_enable);

int display_bias_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(display_bias_disable);
#endif


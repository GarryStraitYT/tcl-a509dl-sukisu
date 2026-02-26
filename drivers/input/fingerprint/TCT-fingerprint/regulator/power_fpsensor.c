#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>
static struct regulator *irtx_ldo_pos;
static int regulator_inited;

int finger_power_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	irtx_ldo_pos = regulator_get(NULL, "irtx_ldo");
	if (IS_ERR(irtx_ldo_pos)) { /* handle return value */
		ret = PTR_ERR(irtx_ldo_pos);
		pr_info("get irtx_ldo fail, error: %d\n", ret);
		return ret;
	}
        pr_info("zk get irtx_ldo success\n");
	regulator_inited = 1;
	return ret; /* must be 0 */

}
EXPORT_SYMBOL(finger_power_regulator_init);

int finger_power_enable(void)
{
	int ret = 0;
	int retval = 0;

	finger_power_regulator_init();

	ret = regulator_set_voltage(irtx_ldo_pos, 3100000, 3300000);

	if (ret < 0)
		pr_info("set voltage irtx_ldo_pos fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(irtx_ldo_pos);
	if (ret < 0)
		pr_info("enable regulator irtx_ldo_pos fail, ret = %d\n",
			ret);
	retval |= ret;
        pr_info("zk enable irtx_ldo success\n");
	return retval;
}
EXPORT_SYMBOL(finger_power_enable);
int finger_power_disable(void)
{
	int ret = 0;
	int retval = 0;
	finger_power_regulator_init();

	ret = regulator_disable(irtx_ldo_pos);
	if (ret < 0)
		pr_info("disable regulator irtx_ldo_pos fail, ret = %d\n",
			ret);
	retval |= ret;
        pr_info("zk enable irtx_ldo disable\n");
	return retval;
}
EXPORT_SYMBOL(finger_power_disable);

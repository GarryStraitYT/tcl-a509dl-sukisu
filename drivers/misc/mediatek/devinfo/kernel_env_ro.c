#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

/*
0: disable fem, 2.4G IPA, 5G IPA
1: 2.4G EPA, 5G EPA
2: 2.4G EPA, 5G IPA
3: 2.4G IPA, 5G EPA
*/
#define WIFI_ALL_IPA 0
#define WIFI_ALL_EPA 1
#define WIFI_2G4E_5GI_PA 2
#define WIFI_2G4I_5GE_PA 3

int pcba_boardid = 0;
EXPORT_SYMBOL(pcba_boardid);

static int wifi_fem_state = WIFI_ALL_IPA;
int get_wifi_fem_state(void)
{
	return wifi_fem_state;
}
EXPORT_SYMBOL(get_wifi_fem_state); 

static int read_boardid(struct device *dev)
{
	struct iio_channel *channel;
	int voltage = 0;
	int ret = 0;

	channel = iio_channel_get(dev, "board_id-channel");
	if (IS_ERR(channel)) {
		ret = PTR_ERR(channel);
		pr_err("[%s] iio channel not found %d\n",
		__func__, ret);
		return ret;
	}

	if (channel)
		ret = iio_read_channel_processed(channel, &voltage);

	if (ret <= 0) {
		pr_err("[%s] iio_read_channel_processed failed\n", __func__);
		return ret;
	}

	pr_err("%s:voltage=%d\n", __func__, voltage);
	return voltage;
}

static int kernel_env_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "export wifi_fem %d\n", wifi_fem_state);
	return 0;
}

static int kernel_env_ro_probe(struct platform_device *pdev)
{
	
	int wifi_all_epa[2] = { -1 };
        int wifi_all_ipa[2] = { -1 };
        int wifi_2g4e_5gi_pa[2] = { -1 };
        int wifi_2g4i_5ge_pa[2] = { -1 };
	struct device_node *env_node = pdev->dev.of_node;
	
	pcba_boardid = read_boardid(&pdev->dev);
	if (pcba_boardid <= 0) {
		return 0;
	}

	of_property_read_u32_array(env_node, "wifi_all_epa",
			wifi_all_epa, ARRAY_SIZE(wifi_all_epa));
	of_property_read_u32_array(env_node, "wifi_all_ipa",
			wifi_all_ipa, ARRAY_SIZE(wifi_all_ipa));
	of_property_read_u32_array(env_node, "wifi_2g4e_5gi_pa",
			wifi_2g4e_5gi_pa, ARRAY_SIZE(wifi_2g4e_5gi_pa));
	of_property_read_u32_array(env_node, "wifi_2g4i_5ge_pa",
			wifi_2g4i_5ge_pa, ARRAY_SIZE(wifi_2g4i_5ge_pa));

	if (wifi_all_epa[0] != -1 && pcba_boardid >= wifi_all_epa[0] && pcba_boardid <= wifi_all_epa[1]) {
		wifi_fem_state = WIFI_ALL_EPA;
	} else if (wifi_all_ipa[0] != -1 && pcba_boardid >= wifi_all_ipa[0] && pcba_boardid <= wifi_all_ipa[1]) {
		wifi_fem_state = WIFI_ALL_IPA;
	} else if (wifi_2g4e_5gi_pa[0] != -1 && pcba_boardid >= wifi_2g4e_5gi_pa[0] && pcba_boardid <= wifi_2g4e_5gi_pa[1]) {
		wifi_fem_state = WIFI_2G4E_5GI_PA;
	} else if (wifi_2g4i_5ge_pa[0] != -1 && pcba_boardid >= wifi_2g4i_5ge_pa[0] && pcba_boardid <= wifi_2g4i_5ge_pa[1]) {
		wifi_fem_state = WIFI_2G4I_5GE_PA;
	} else {
		wifi_fem_state = WIFI_ALL_EPA;
	}

	pr_info("pcba_boardid: %d wifi_fem_state=%d\n", pcba_boardid,wifi_fem_state);
	proc_create_single("kernel_env_ro", 0, NULL, kernel_env_proc_show);

	return 0;
}

static const struct of_device_id kernel_env_ro_match[] = {
	{ .compatible = "tcl,kernel_env_ro" },
	{}
};

static struct platform_driver kernel_env_ro_drv = {
	.probe = kernel_env_ro_probe,
	.driver = {
		.name = "kernel_env",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
		.of_match_table	= kernel_env_ro_match,
	}
};

static int __init kernel_env_ro_init(void)
{
	int ret;

	ret = platform_driver_register(&kernel_env_ro_drv);
	if (ret) {
		pr_err("%s:%d: platform_driver_register failed\n", __func__, __LINE__);
		return -ENODEV;
	}
	return 0;
}

fs_initcall(kernel_env_ro_init);

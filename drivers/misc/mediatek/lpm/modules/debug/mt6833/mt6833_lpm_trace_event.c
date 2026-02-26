// SPDX-License-Identifier: GPL-2.0
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>

#include <mtk_lpm_trace_event/mtk_lpm_trace_event.h>
#include <mtk_spm_sysfs.h>
#include <mt6833_spm_reg.h>


#define plat_mmio_read(offset)	__raw_readl(spm_base + offset)

void __iomem *spm_base;
struct timer_list spm_resource_req_timer;
u32 spm_resource_req_timer_is_enabled;
u32 spm_resource_req_timer_ms;


ssize_t get_spm_resource_req_timer_enable(char *ToUserBuf
		, size_t sz, void *priv)
{
	int bLen = snprintf(ToUserBuf, sz
				, "spm resource request timer is enabled: %d\n",
				spm_resource_req_timer_is_enabled);
	return (bLen > sz) ? sz : bLen;
}

ssize_t set_spm_resource_req_timer_enable(char *ToUserBuf
		, size_t sz, void *priv)
{
	u32 is_enable;
	u32 timer_ms;

	if (!ToUserBuf)
		return -EINVAL;

	if (sscanf(ToUserBuf, "%d %d", &is_enable, &timer_ms) == 2) {
/* 		spm_resource_req_timer_en(is_enable, timer_ms); */
		return sz;
	}

	return -EINVAL;
}

static const struct mtk_lp_sysfs_op spm_resource_req_timer_enable_fops = {
	.fs_read = get_spm_resource_req_timer_enable,
	.fs_write = set_spm_resource_req_timer_enable,
};

bool spm_is_md1_sleep(void)
{
	return !(plat_mmio_read(SRC_REQ_STA_0) & 0x3F);
}
EXPORT_SYMBOL(spm_is_md1_sleep);

int __init mt6833_lpm_trace_init(void)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,sleep");

	if (node) {
		spm_base = of_iomap(node, 0);
		of_node_put(node);
	}

	mtk_spm_sysfs_root_entry_create();
	mtk_spm_sysfs_entry_node_add("spm_dump_res_req_enable", 0444
			, &spm_resource_req_timer_enable_fops, NULL);

	return 0;
}
late_initcall_sync(mt6833_lpm_trace_init);


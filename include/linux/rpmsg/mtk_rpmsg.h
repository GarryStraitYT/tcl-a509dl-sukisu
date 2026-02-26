/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __LINUX_RPMSG_MTK_RPMSG_H
#define __LINUX_RPMSG_MTK_RPMSG_H

#include <linux/platform_device.h>
#include <linux/remoteproc.h>

typedef void (*ipi_handler_t)(void *data, unsigned int len, void *priv);

struct mtk_rpmsg_info {
	int (*register_ipi)(struct platform_device *pdev, u32 id,
			    ipi_handler_t handler, void *priv);
	void (*unregister_ipi)(struct platform_device *pdev, u32 id);
	int (*send_ipi)(struct platform_device *pdev, u32 id,
			void *buf, unsigned int len, unsigned int wait);
	int ns_ipi_id;
};

struct rproc_subdev *
mtk_rpmsg_create_rproc_subdev(struct platform_device *pdev,
			      struct mtk_rpmsg_info *info);

void mtk_rpmsg_destroy_rproc_subdev(struct rproc_subdev *subdev);

struct mtk_rpmsg_channel_info {
	struct rpmsg_channel_info info;
	//bool registered;
	//struct list_head list;
	unsigned int send_slot; //send slot offset
	unsigned int recv_slot; //recv slot offset
	unsigned int send_slot_size; // send slot count
	unsigned int recv_slot_size; // recv slot count
	unsigned int send_pin_index; // pin irq index
	unsigned int recv_pin_index; // pin irq index
	unsigned int send_pin_offset;// pin array offset
	unsigned int recv_pin_offset;// pin array offset
	unsigned int mbox; //mbox
	spinlock_t channel_lock;
};

struct mtk_rpmsg_endpoint {
	struct rpmsg_endpoint ept;
	//struct mtk_rpmsg_rproc_subdev *mtk_subdev;
	struct mtk_rpmsg_device *mdev;
	struct mtk_rpmsg_channel_info *mchan;
};

struct mtk_rpmsg_operations {
	int (*mbox_send)(struct mtk_rpmsg_endpoint *mept,
		struct mtk_rpmsg_channel_info *mchan,
		void *buf, unsigned int len, unsigned int wait);
};

struct mtk_rpmsg_device {
	struct rpmsg_device rpdev;
	//struct mtk_rpmsg_rproc_subdev *mtk_subdev;
	struct platform_device *pdev;
	struct mtk_rpmsg_operations *ops;
	struct mtk_mbox_device *mbdev;

};

struct mtk_rpmsg_device *mtk_rpmsg_create_device(struct platform_device *pdev,
		struct mtk_mbox_device *mbdev, unsigned int ipc_chan_id);
struct mtk_rpmsg_channel_info *
mtk_rpmsg_create_channel(struct mtk_rpmsg_device *mdev, u32 chan_id,
		char *name);

#endif

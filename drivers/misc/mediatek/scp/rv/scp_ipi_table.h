/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _SCP_IPI_TABLE_H_
#define _SCP_IPI_TABLE_H_

#include "scp_mbox_layout.h"
#include "scp_ipi_pin.h"

struct mtk_mbox_info *scp_mbox_info;

struct mtk_mbox_pin_recv *scp_mbox_pin_recv;


struct mtk_mbox_pin_send *scp_mbox_pin_send;


struct mtk_mbox_device scp_mboxdev = {
	.name = "scp_mboxdev",
	.pin_recv_table = 0,
	.pin_send_table = 0,
	.info_table = 0,
	.count = 0,
	.recv_count = 0,
	.send_count = 0,
	.post_cb = (mbox_rx_cb_t)scp_clr_spm_reg,
};

struct mtk_ipi_device scp_ipidev = {
	.name = "scp_ipidev",
	.id = IPI_DEV_SCP,
	.mbdev = &scp_mboxdev,
	.pre_cb = (ipi_tx_cb_t)scp_awake_lock,
	.post_cb = (ipi_tx_cb_t)scp_awake_unlock,
	.prdata = 0,
};
EXPORT_SYMBOL(scp_ipidev);

#endif

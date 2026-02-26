/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __MT6781_H__
#define __MT6781_H__

#include <linux/delay.h>
#include <mtk_lpm_type.h>
#include <mt6781_common.h>

int mt6781_do_mcusys_prepare_pdn(unsigned int status,
					   unsigned int *resource_req);

int mt6781_do_mcusys_prepare_on_ex(unsigned int clr_status);

int mt6781_do_mcusys_prepare_on(void);

#endif

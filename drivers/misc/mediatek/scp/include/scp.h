/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __SCP_H
#define __SCP_H

#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_CM4_SUPPORT)
#include "scp_cm4.h"
#elif IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_RV_SUPPORT)
#include "scp_rv.h"
#endif

#endif


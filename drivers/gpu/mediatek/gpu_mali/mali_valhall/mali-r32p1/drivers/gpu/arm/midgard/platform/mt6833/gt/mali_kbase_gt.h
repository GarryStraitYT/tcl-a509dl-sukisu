// SPDX-License-Identifier: GPL-2.0
/********************************************************************************
** Copyright (C), 2014-2021, TCL TV Comm Corp., Ltd
** VENDOR_EDIT, All rights reserved.
**
** File: - mali_kbase_gt.h
** Description:
**     GPU Memory Compression
**
** ------------------------------- Revision History: ---------------------------
** <author>            <date>       <version>      <desc>
** -----------------------------------------------------------------------------
** shu5.zhang@tcl.com    2022-05-12       1.0       add init version.
*******************************************************************************/

#ifndef MALI_GT_H
#define MALI_GT_H

#include <mali_kbase.h>
#include <tcl/gputurbo.h>
#include <kernel.h>

unsigned int kbase_get_frametime(struct GpuRunParameter *data);
unsigned int kbase_gpu_utilisation(void);
unsigned int kbase_gpu_curfreq(void);
unsigned int kbase_get_devfreq(struct devfreq **);

#endif
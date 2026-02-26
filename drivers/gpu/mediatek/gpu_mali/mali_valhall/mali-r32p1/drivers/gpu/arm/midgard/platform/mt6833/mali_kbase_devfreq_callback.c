// SPDX-License-Identifier: GPL-2.0
/********************************************************************************
** Copyright (C), 2014-2021, TCL TV Comm Corp., Ltd
** VENDOR_EDIT, All rights reserved.
**
** File: - mali_kbase_devfreq_callback.c
** Description:
**     GPU dynamic frequency governor
**
** ------------------------------- Revision History: ---------------------------
** <author>            <date>       <version>      <desc>
** -----------------------------------------------------------------------------
** shu5.zhang@tcl.com    2021-06-25       1.0       add init version.
*******************************************************************************/

#include <mali_kbase.h>
#include "mali_kbase_devfreq_callback.h"

void gpu_cl_boost_init(void *dev)
{
	struct kbase_device *kbdev = NULL;

	kbdev = (struct kbase_device *)dev;
	KBASE_DEBUG_ASSERT(kbdev != NULL);

	atomic_set(&kbdev->pm.backend.metrics.time_compute_jobs, 0);
	atomic_set(&kbdev->pm.backend.metrics.time_vertex_jobs, 0);
	atomic_set(&kbdev->pm.backend.metrics.time_fragment_jobs, 0);
}

void gpu_cl_boost_update_utilization(void *dev, void *atom,
	u64 microseconds_spent)
{
	struct kbase_jd_atom *katom = NULL;
	struct kbase_device *kbdev = NULL;

	kbdev = (struct kbase_device *)dev;
	KBASE_DEBUG_ASSERT(kbdev != NULL);

	katom = (struct kbase_jd_atom *)atom;
	KBASE_DEBUG_ASSERT(katom != NULL);

	if (katom->core_req & BASE_JD_REQ_ONLY_COMPUTE)
		atomic_add((microseconds_spent >> KBASE_PM_TIME_SHIFT),
			&kbdev->pm.backend.metrics.time_compute_jobs);
	else if (katom->core_req & BASE_JD_REQ_FS)
		atomic_add((microseconds_spent >> KBASE_PM_TIME_SHIFT),
			&kbdev->pm.backend.metrics.time_fragment_jobs);
	else if (katom->core_req & BASE_JD_REQ_CS)
		atomic_add((microseconds_spent >> KBASE_PM_TIME_SHIFT),
			&kbdev->pm.backend.metrics.time_vertex_jobs);
}

struct mali_kbase_callbacks mali_gpu_callbacks = {
	.cl_boost_init = gpu_cl_boost_init,
	.cl_boost_update_utilization = gpu_cl_boost_update_utilization,
};

uintptr_t gpu_get_callbacks(void)
{
	return (uintptr_t)&mali_gpu_callbacks;
}

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * List of cgroup subsystems.
 *
 * DO NOT ADD ANY SUBSYSTEM WITHOUT EXPLICIT ACKS FROM CGROUP MAINTAINERS.
 */

/*
 * This file *must* be included with SUBSYS() defined.
 */

#if IS_ENABLED(CONFIG_CPUSETS)
SUBSYS(cpuset)
#endif

#if IS_ENABLED(CONFIG_CGROUP_SCHED)
SUBSYS(cpu)
#endif

#if IS_ENABLED(CONFIG_CGROUP_CPUACCT)
SUBSYS(cpuacct)
#endif

#if IS_ENABLED(CONFIG_SCHED_TUNE)
SUBSYS(schedtune)
#endif

#if IS_ENABLED(CONFIG_BLK_CGROUP)
SUBSYS(io)
#endif

#if IS_ENABLED(CONFIG_MEMCG)
SUBSYS(memory)
#endif

#if IS_ENABLED(CONFIG_CGROUP_DEVICE)
SUBSYS(devices)
#endif

#if IS_ENABLED(CONFIG_CGROUP_FREEZER)
SUBSYS(freezer)
#endif

#if IS_ENABLED(CONFIG_CGROUP_NET_CLASSID)
SUBSYS(net_cls)
#endif

#if IS_ENABLED(CONFIG_CGROUP_PERF)
SUBSYS(perf_event)
#endif

#if IS_ENABLED(CONFIG_CGROUP_NET_PRIO)
SUBSYS(net_prio)
#endif

#if IS_ENABLED(CONFIG_CGROUP_HUGETLB)
SUBSYS(hugetlb)
#endif

#if IS_ENABLED(CONFIG_CGROUP_PIDS)
SUBSYS(pids)
#endif

#if IS_ENABLED(CONFIG_CGROUP_RDMA)
SUBSYS(rdma)
#endif

/*
 * The following subsystems are not supported on the default hierarchy.
 */
#if IS_ENABLED(CONFIG_CGROUP_DEBUG)
SUBSYS(debug)
#endif

//[TCT-ROM]Begin added by xizheng.mo for 9572106 blkio type on 20200716
#if IS_ENABLED(CONFIG_TCT_IOLIMIT) || IS_ENABLED(CONFIG_CGROUP_IOLIMIT)
SUBSYS(iolimit)
#endif
//[TCT-ROM]End added by xizheng.mo for 9572106 blkio type on 20200716

// #ifdef VENDOR_EDIT
// xiwu1.peng@kernel 2021/09/03 add for iolimit
//#if IS_ENABLED(CONFIG_CGROUP_IOLIMIT)
//SUBSYS(iolimit)
//#endif
// #endif /* VENDOR_EDIT */

/*
 * DO NOT ADD ANY SUBSYSTEM WITHOUT EXPLICIT ACKS FROM CGROUP MAINTAINERS.
 */

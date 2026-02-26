/* SPDX-License-Identifier: GPL-2.0 */


#ifndef _KBASE_CPU_VEXPRESS_H_
#define _KBASE_CPU_VEXPRESS_H_

int kbase_get_vexpress_cpu_clock_speed(u32 *cpu_clock);

u32 kbase_get_platform_min_freq(void);

u32 kbase_get_platform_max_freq(void);

#endif              /* _KBASE_CPU_VEXPRESS_H_ */

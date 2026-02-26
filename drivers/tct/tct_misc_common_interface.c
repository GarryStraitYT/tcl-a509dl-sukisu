/*
 *  Generic DT helper functions for touchscreen devices
 *
 *  Copyright (c) 2014 Sebastian Reichel <sre@kernel.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

/*
 *  use for misc common modify in this file
*/


#include <linux/device.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <tct_debug.h>

//Begin add by bing-zhang for 11669573 on 2021/11/12
#if defined(CONFIG_TCT_BOARD_ID_COMPATIBLE)
unsigned int androidboot_dtbo_idx = 0;
EXPORT_SYMBOL_GPL(androidboot_dtbo_idx);
int _androidboot_dtbo_idx(char *str)
{
	if (!strcmp(str,"1")) {
		androidboot_dtbo_idx = 1;
	}else if (!strcmp(str,"2")) {
		androidboot_dtbo_idx = 2;
	}else if (!strcmp(str,"3")) {
		androidboot_dtbo_idx = 3;
	}else {
		androidboot_dtbo_idx = 0;
	}

	printk("%s[%d] tct.dtbo_idx = %d\n",__func__,__LINE__,androidboot_dtbo_idx);

	return androidboot_dtbo_idx;
}

__setup("tct.dtbo_idx=",_androidboot_dtbo_idx);
#endif
//End add by bing-zhang for 11669573 on 2021/11/12


/*****************************************************************************
*  Name: tct_bsp_init
*  Brief: 1. create bsp node as interface.
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int __init tct_bsp_init(void)
{

	TCT_FUNC_ENTER();

	TCT_FUNC_EXIT();
	return 0;
}

static void __exit tct_bsp_exit(void)
{
    TCT_FUNC_ENTER();

    TCT_FUNC_EXIT();
}

module_init(tct_bsp_init);
module_exit(tct_bsp_exit);

MODULE_AUTHOR("TCT HZ BSP Team");
MODULE_DESCRIPTION("TCT HZ BSP Driver");
MODULE_LICENSE("GPL v2");



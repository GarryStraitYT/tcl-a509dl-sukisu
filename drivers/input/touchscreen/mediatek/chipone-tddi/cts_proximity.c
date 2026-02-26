/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_proximity.c
*
*    Author: luoguojin
*
*   Created: 2016-09-19
*
*  Abstract: close proximity function
*
*   Version: v1.0
*
* Revision History:
*        v1.0:
*            First release based on xiaguobin's solution. By luougojin 2016-08-19
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_sfctrl.h"
#include "cts_spi_flash.h"
#include "cts_sysfs.h"
#include "cts_firmware.h"
#include "cts_charger_detect.h"
#include "cts_earjack_detect.h"
#include "cts_tcs.h"


#ifdef CONFIG_CTS_TP_PROXIMITY

#include "hwmsensor.h"
#include "sensors_io.h"
#include "hwmsen_helper.h"
#include <alsps.h>


extern int cts_is_suspend(void);
extern int cts_get_prox_status(void);


/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct cts_proximity_st {
    u8      mode                : 1;    /* 1- proximity enable 0- disable */
    u8      detect              : 1;    /* 0-->close ; 1--> far away */
    u8 	sleepen           :1;    /*TP in sleep modem ,buf en TP proximity*/
    u8      unused              : 4;
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct cts_proximity_st cts_proximity_data;

extern void cts_set_prox_en(unsigned int enable);
static int cts_enter_proximity_mode( int mode);


/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t cts_touch_proximity_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t cts_touch_proximity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
/*****************************************************************************
* functions body
*****************************************************************************/


/* read and write proximity mode
*   read example: cat  cts_touch_proximity_mode---read  proximity mode
*   write example:echo 01 > cts_touch_proximity_mode ---write proximity mode to 01
*/
static DEVICE_ATTR (cts_touch_proximity_mode, S_IRUGO | S_IWUSR, cts_touch_proximity_show, cts_touch_proximity_store);
static struct attribute *cts_touch_proximity_attrs[] = {
    &dev_attr_cts_touch_proximity_mode.attr,
    NULL,
};

static struct attribute_group cts_touch_proximity_group = {
    .attrs = cts_touch_proximity_attrs,
};


static ssize_t cts_touch_proximity_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    return snprintf(buf, PAGE_SIZE, "Proximity: %s  cts_get_prox_status()=%d \n", cts_proximity_data.mode ? "On" : "Off"  ,  cts_get_prox_status());
 
}

static ssize_t cts_touch_proximity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    int value1[] = {1,0};
    val = simple_strtoul(buf, 0, 10);
    if (val == 1) 
   {
        if (!cts_proximity_data.mode) 
  	{
            cts_proximity_data.mode = 1;

  	     cts_info("[ TP_PS ] cts_touch_proximity_store cts_set_prox_en 1" );
             cts_set_prox_en( 1);
        }
    }
  else  if (val == 2)  //test send near
   {
					
		cts_proximity_data.detect=0;
                value1[0]=0;
		ps_report_interrupt_data(value1);  //near
		cts_info("[ TP_PS ] report near" );
					
    }
  else  if (val == 3)  //test send far
   {
					
		cts_proximity_data.detect=1;
                value1[0]=1;
		ps_report_interrupt_data(value1);
		cts_info("[ TP_PS ]   report far" );
					
    }
	
 else
 {
        if (cts_proximity_data.mode) 
	{
            cts_proximity_data.mode = 0;
  	     cts_info("[ TP_PS ] cts_touch_proximity_store cts_set_prox_en 0 " );		
             cts_set_prox_en( 0);
         }
   }
  

    return count;
}




/************************************************************************
* Name: cts_enter_proximity_mode
* Brief:  change proximity mode
* Input:  proximity mode
* Output: no
* Return: success =0
***********************************************************************/
static int cts_enter_proximity_mode( int mode)
{
    int ret = 0;
    u8 buf_value = 0;

    cts_info("[ TP_PS ] cts_enter_proximity_mode= %d", mode);


    if (mode)
        buf_value = 0x01;
    else
        buf_value = 0x00;

       cts_set_prox_en(mode);
	


    cts_proximity_data.mode = buf_value ? 1 : 0;

    return ret ;
}







static int ps_open_report_data(int open)
{

	return 0;
}



static int ps_enable_nodata(int en)
{
    int value1[] = {1,0};
    if(( cts_is_suspend()==1))
   	{

		extern void  cts_set_prox_en2(int en);
		cts_set_prox_en2(en);
	   	cts_err("ps_enable_nodata error TP is in suspend \n" );
	   return 0;
   	}		
	

	ps_report_interrupt_data(value1);
	cts_enter_proximity_mode( en);

	return 0;
}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int *value, int *status)
{

        int prox_status;
	    prox_status=cts_get_prox_status();

	if(	prox_status>1)
		*value = 10;
	else
		*value = 0;
	
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;
	

	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}



/*****************************************************************************
*  Name: cts_proximity_init
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int cts_proximity_init(struct chipone_ts_data *cts_data)
{
		int err = 0;
		struct ps_control_path ps_ctl = { 0 };
		struct ps_data_path ps_data = { 0 };	
		
		cts_info("  cts_proximity_init() in \n");			
		memset((u8 *)&cts_proximity_data, 0, sizeof(struct cts_proximity_st));
		cts_proximity_data.detect = 1;  /* defalut far awway */		
				
		err = sysfs_create_group(&cts_data->device->kobj, &cts_touch_proximity_group);
		if (0 != err) {
		cts_err("[ TP_PS ] Create sysfs node failed,ret=%d", err);
		return err;
		}		
		
		ps_ctl.open_report_data = ps_open_report_data;
		ps_ctl.enable_nodata = ps_enable_nodata;
		ps_ctl.set_delay = ps_set_delay;
		ps_ctl.is_report_input_direct = true;
		ps_ctl.batch = ps_batch;
		ps_ctl.flush = ps_flush;
		ps_ctl.is_support_batch = false;	
		
		
		err = ps_register_control_path(&ps_ctl);
		if (err) {
				 cts_err("call ps_register_control_path() failed = %d\n", err);
		}
		
		ps_data.get_data = ps_get_data;
		ps_data.vender_div = 100;
		err = ps_register_data_path(&ps_data);
		if (err) {
		cts_err("call ps_register_data_path() failed = %d\n", err);
		}

    return 0;
}



static int cts_proximity_local_uninit(void)
{
	return 0;
}

/*----------------------------------------------------------------------------*/

static int cts_proximity_local_init(void)
{
	return 0;
}

static struct alsps_init_info tpd_proximity_info = {
	.name = "tp_ps",
	.init = cts_proximity_local_init,
	.uninit = cts_proximity_local_uninit,
};


void cts_add_virtual_proximity(void)
{

		cts_info("GZL  cts_proximity_init()\n");
		
		alsps_driver_add(&tpd_proximity_info);
}

#endif /* cts_PSENSOR_EN */


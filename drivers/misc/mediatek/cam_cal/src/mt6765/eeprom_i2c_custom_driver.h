/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __EEPROM_I2C_CUSTOM_DRIVER_H
#define __EEPROM_I2C_CUSTOM_DRIVER_H
#include <linux/i2c.h>

unsigned int Custom_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

//begin 20210816 ljt add for jetta
unsigned int gc08a3_read_region(struct i2c_client *client, 
				unsigned int addr,
				unsigned char *data, 
				unsigned int size);
//end 20210816 ljt add for jetta
unsigned int hi846_read_region(struct i2c_client *client, 
				unsigned int addr,
				unsigned char *data, 
				unsigned int size);
#if defined(CONFIG_MTK_LUNA84GVZW_CAMERA)
unsigned int gc05a2_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size);
/*Begin jiantaohuang for LUNA84GVZW-3892, otp porting for lunavzw hi846 on 20221215*/
unsigned int hi846_new_read_region(struct i2c_client *client, 
				unsigned int addr,
				unsigned char *data, 
				unsigned int size);
/*End jiantaohuang for LUNA84GVZW-3892, otp porting for lunavzw hi846 on 20221215*/
#endif
#if defined(TCT_CAMERA_PROJECT_CIVIC)
unsigned int gc05a2_front_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size);
//Begin modified by chengyixuan for CIVICPL-3060 on 2022-07-16
#endif
#endif				/* __CAM_CAL_LIST_H */

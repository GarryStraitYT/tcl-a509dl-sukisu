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
unsigned int hi846_read_region(struct i2c_client *client, 
				unsigned int addr,
				unsigned char *data, 
				unsigned int size);
//end 20210816 ljt add for jetta

#endif				/* __CAM_CAL_LIST_H */

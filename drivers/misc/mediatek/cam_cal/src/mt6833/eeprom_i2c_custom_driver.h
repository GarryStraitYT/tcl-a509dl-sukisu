/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __EEPROM_I2C_CUSTOM_DRIVER_H
#define __EEPROM_I2C_CUSTOM_DRIVER_H
#include <linux/i2c.h>

unsigned int Custom_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int BA815M_gc0a83_read_region(struct i2c_client *client, 
				unsigned int addr,
				unsigned char *data, 
				unsigned int size);

unsigned int shnbf821b_gc0a83_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif				/* __CAM_CAL_LIST_H */

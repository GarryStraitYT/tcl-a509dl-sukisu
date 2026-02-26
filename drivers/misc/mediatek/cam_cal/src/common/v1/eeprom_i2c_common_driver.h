/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __EEPROM_I2C_COMMON_DRIVER_H
#define __EEPROM_I2C_COMMON_DRIVER_H
#include <linux/i2c.h>

unsigned int Common_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

/*Begin ersen.shang for [Task][ 11425156][cruze/cruze pro camera bring up] 202108*/
unsigned int Common_write_region_p24c128e(struct i2c_client *client,
				unsigned int addr,
 				unsigned char *data,
				unsigned int size);
unsigned int Common_write_region_gt24p64b(struct i2c_client *client,
				unsigned int addr,
 				unsigned char *data,
				unsigned int size);
unsigned int Common_write_region_p24c64f(struct i2c_client *client,
				unsigned int addr,
 				unsigned char *data,
				unsigned int size);
/*End   ersen.shang for [Task][ 11425156][cruze/cruze pro camera bring up] 202108*/

#endif				/* __CAM_CAL_LIST_H */

/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __IMX766_EEPROM_H__
#define __IMX766_EEPROM_H__

#include "kd_camera_typedef.h"

unsigned int read_imx766_LRC(BYTE *data);

unsigned int read_imx766_DCC(BYTE *data);

#endif


/*
 * GalaxyCore touchscreen driver
 *
 * Copyright (C) 2021 GalaxyCore Incorporated
 *
 * Copyright (C) 2021 Neo Chen <neo_chen@gcoreinc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

enum fw_event_type {
	FW_UPDATE = 0,
	FW_READ_REG,
	FW_WRITE_REG,
	FW_READ_OPEN,
	FW_READ_SHORT,
	FW_EDGE_0,
	FW_EDGE_90,
	FW_CHARGER_PLUG,
	FW_CHARGER_UNPLUG,
	FW_HEADSET_PLUG,
	FW_HEADSET_UNPLUG,
	FW_READ_RAWDATA,
	FW_READ_DIFFDATA,
	FW_READ_NOISE,
	FW_GESTURE_ENABLE,
	FW_GESTURE_DISABLE,
	FW_GLOVE_ENABLE, 
	FW_GLOVE_DISABLE,
    FW_REPORT_RATE_120,
    FW_REPORT_RATE_180
};


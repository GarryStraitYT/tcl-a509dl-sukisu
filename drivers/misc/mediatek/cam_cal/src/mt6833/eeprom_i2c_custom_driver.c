/* SPDX-License-Identifier: GPL-2.0 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define EEPROM_I2C_MSG_SIZE_READ 2

 #define PAGE_SIZE_ 256

 //begin add for BA815 08a3 otp
 struct gc08a3_otp_t {
	u8  awb_flag;
	u8  awb_param[12];
        u8  awbChksum;
	u8  lsc_flag;
	u8  lsc_param[1868];
        u8  lscChksum;
	u8  af_flag;
	u8  af_param[8];
        u8  afChksum;
};
struct gc08a3_otp_t gc08a3_otp_info;
unsigned int BA815M_gc0a83_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{
    int i=0;

    pr_err("[ljt]addr =%x size %d\n", addr, size);
    if (addr == 0x0)
    {
            *data = 0x0f;
    }
    else if (addr == 0x1)
    {
        if ((gc08a3_otp_info.lsc_flag == 0x01) || (gc08a3_otp_info.lsc_flag == 0x04))
        {
            pr_err("[ljt]lsc_flag valid\n");
            for(i=0; i<size; i++){
                data[i] = gc08a3_otp_info.lsc_param[i];
            }
        }
    }
    else if (addr == 0x2)
    {
        if ((gc08a3_otp_info.awb_flag == 0x01) || (gc08a3_otp_info.awb_flag == 0x04))
        {
            *data = 1;
        }
    }
    else if (addr == 0x3)
    {
        for(i=0; i<size; i++){
            data[i] = gc08a3_otp_info.awb_param[i];
            //pr_err("[ljt]    awb data[%d] =%x \n", i, data[i]);
        }
    }
    else if (addr == 0x4)
    {
        if ((gc08a3_otp_info.af_flag == 0x01) ||(gc08a3_otp_info.af_flag == 0x04))
        {
            *data = 1;
        }
    }
    else if (addr == 0x5)
    {
        for(i=0; i<size; i++){
            data[i] = gc08a3_otp_info.af_param[i];
            //pr_err("[ljt]af data[%d] =%x \n", i, data[i]);
        }
    }
    else
    {
        pr_err("[ljt] error addr\n", addr, size);
    }

    return size;
}
//end for BA815 08a3 otp

// start binchang.liang for FF GC08A3 OTP
unsigned int shnbf821b_gc0a83_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{
    pr_err("addr =%x size %d\n", addr, size);
    switch(addr)
    {
        case 0x15a8: // module ID
            data[0] = 0x43;
        break;
        case 0x1650: // awb
            memcpy(data, &gc08a3_otp_info.awb_param, size);
        break;
        case 0x1728: // lsc
            memcpy(data, &gc08a3_otp_info.lsc_param, size);
        break;

        default:
            pr_err("addr 0x%x not define \n",addr);
        break;
    }
    return size;
}

// end binchang.liang for FF GC08A3 OTP

static int iReadRegI2C(struct i2c_client *client,
		u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int i4RetValue = 0;
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

	client->addr = (i2cId >> 1);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue = i2c_transfer(client->adapter, msg,
				EEPROM_I2C_MSG_SIZE_READ);

	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
		pr_debug("I2C read failed!!\n");
		return -1;
	}
	return 0;
}

static int custom_read_region(struct i2c_client *client,
			      u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	u8 *buff = data;
	u32 size_to_read = size;

	int ret = 0;

	while (size_to_read > 0) {
		u8 page = addr / PAGE_SIZE_;
		u8 offset = addr % PAGE_SIZE_;
		char *Buff = data;

		if (iReadRegI2C(client, &offset, 1, (u8 *)Buff, 1,
			i2c_id + (page << 1)) < 0) {
			pr_debug("fail addr=0x%x 0x%x, P=%d, offset=0x%x",
				addr, *Buff, page, offset);
			break;
		}
		addr++;
		buff++;
		size_to_read--;
		ret++;
	}
	pr_debug("addr =%x size %d data read = %d\n", addr, size, ret);
	return ret;
}



unsigned int Custom_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	if (custom_read_region(client, addr, data, client->addr, size) == 0)
		return size;
	else
		return 0;
}



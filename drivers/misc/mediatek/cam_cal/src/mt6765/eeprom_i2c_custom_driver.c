// SPDX-License-Identifier: GPL-2.0
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
#include "cam_cal_list.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define EEPROM_I2C_MSG_SIZE_READ 2

static DEFINE_SPINLOCK(g_spinLock);
static struct i2c_client *g_pstI2CclientG;

//begin 20210816 ljt add for jetta
#ifdef JETTATF_GC08A3_MIPI_RAW
extern struct gc08a3_otp_t gc08a3_otp_info;
#else
struct gc08a3_otp_t gc08a3_otp_info;
#endif
struct hi846_otp_t hi846_otp_info;
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
//end 20210816 ljt add for jetta
struct hi846_otp_t {
    u8  awb_flag;
    u8  awb_param[18];
    u8  awbChksum;
    u8  lsc_flag;
    u8  lsc_param[1868+1];
    u8  lscChksum;
    u8  af_flag;
    u8  af_param[7];
    u8  afChksum;
};
#define PAGE_SIZE_ 256
static int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
                       u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
    int  i4RetValue = 0;
    struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

    spin_lock(&g_spinLock);
    g_pstI2CclientG->addr = (i2cId >> 1);
    spin_unlock(&g_spinLock);

    msg[0].addr = g_pstI2CclientG->addr;
    msg[0].flags = g_pstI2CclientG->flags & I2C_M_TEN;
    msg[0].len = a_sizeSendData;
    msg[0].buf = a_pSendData;

    msg[1].addr = g_pstI2CclientG->addr;
    msg[1].flags = g_pstI2CclientG->flags & I2C_M_TEN;
    msg[1].flags |= I2C_M_RD;
    msg[1].len = a_sizeRecvData;
    msg[1].buf = a_pRecvData;

    i4RetValue = i2c_transfer(g_pstI2CclientG->adapter,
                              msg,
                              EEPROM_I2C_MSG_SIZE_READ);

    if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
        pr_debug("I2C read failed!!\n");
        return -1;
    }
    return 0;
}

static int custom_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
    u8 *buff = data;
    u32 size_to_read = size;

    int ret = 0;

    while (size_to_read > 0) {
        u8 page = addr / PAGE_SIZE_;
        u8 offset = addr % PAGE_SIZE_;
        char *Buff = data;

        if (iReadRegI2C(&offset, 1, (u8 *)Buff, 1,
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
    g_pstI2CclientG = client;
    if (custom_read_region(addr, data, g_pstI2CclientG->addr, size) == 0)
        return size;
    else
        return 0;
}

//begin 20210816 ljt add for jetta
unsigned int gc08a3_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{
    pr_err("[ljt]addr =%x size %d\n", addr, size);

    if (addr == 0x0) {
        *(u32 *)data = 0x020b00ff;
    }

    if (addr == 0x1) {
        if ((gc08a3_otp_info.lsc_flag == 0x01) || (gc08a3_otp_info.lsc_flag == 0x04)) {
            memcpy(data, &gc08a3_otp_info.lsc_param, size);
        }
    } else if (addr == 0x2) {
        if ((gc08a3_otp_info.awb_flag == 0x01) || (gc08a3_otp_info.awb_flag == 0x04)) {
            *data = 1;//awb flag
        }
    } else if (addr == 0x3) {
        memcpy(data, &gc08a3_otp_info.awb_param, size);
    } else if (addr == 0x4) {
        if ((gc08a3_otp_info.af_flag == 0x01) ||(gc08a3_otp_info.af_flag == 0x04)) {
            *data = 1;//af flag
        }
    } else if (addr == 0x5) {
        memcpy(data, &gc08a3_otp_info.af_param, size);
    } else {
        pr_err("[ljt]error addr\n", addr, size);
    }

    return size;
}
//end 20210816 ljt add for jetta
unsigned int hi846_read_region(struct i2c_client *client, unsigned int addr,
                               unsigned char *data, unsigned int size)
{
    int i=0;

    pr_err("[ljt]addr =%x size %d\n", addr, size);
    if (addr == 0x0) {
        *(u32 *)data = 0x010b00ff;
    } else if (addr == 0x1) {
        if ((hi846_otp_info.lsc_flag == 0x01) || (hi846_otp_info.lsc_flag == 0x04)) {
            pr_err("[ljt]lsc_flag valid\n");
            for(i=0; i<size; i++) {
                data[i] = hi846_otp_info.lsc_param[i];
            }
        }
    } else if (addr == 0x2) {
        if ((hi846_otp_info.awb_flag == 0x01) || (hi846_otp_info.awb_flag == 0x04)) {
            *data = 1;
        }
    } else if (addr == 0x3) {
        for(i=0; i<size; i++) {
            data[i] = hi846_otp_info.awb_param[i];
            //pr_err("[ljt]awb data[%d] =%x \n", i, data[i]);
        }
    } else if (addr == 0x4) {
        if ((hi846_otp_info.af_flag == 0x01) ||(hi846_otp_info.af_flag == 0x04)) {
            *data = 1;
        }
    } else if (addr == 0x5) {
        for(i=0; i<size; i++) {
            data[i] = hi846_otp_info.af_param[i];
            //pr_err("[ljt]af data[%d] =%x \n", i, data[i]);
        }
    } else {
        pr_err("[ljt]error addr\n", addr, size);
    }
    return size;
}

// begin modified by tct-hq/yzheng4 in 2022-12-09
#if defined(CONFIG_MTK_LUNA84GVZW_CAMERA)
/*End jiantaohuang for LUNA84GVZW-3892, otp porting for lunavzw hi846 on 20221215*/
unsigned int hi846_new_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{
    int i=0;

    pr_err("[ljt]b addr =%x size %d\n", addr, size);
    if (addr == 0x202)
    {
        *(u32 *)data = 0x48;
    }
    else if (addr == 0x1)
    {
        if ((hi846_otp_info.lsc_flag == 0x01) || (hi846_otp_info.lsc_flag == 0x04))
        {
            pr_err("[ljt]lsc_flag valid\n");
            for(i=0; i<size; i++){
                data[i] = hi846_otp_info.lsc_param[i];
            }
        }
    }
    else if (addr == 0x2)
    {
        if ((hi846_otp_info.awb_flag == 0x01) || (hi846_otp_info.awb_flag == 0x04))
        {
            *data = 1;
        }
    }
    else if (addr == 0x3)
    {
        for(i=0; i<size; i++){
            data[i] = hi846_otp_info.awb_param[i];
            pr_err("[JTl]awb data[%d] =%x \n", i, data[i]);
        }
    }
    else if (addr == 0x4)
    {
        if ((hi846_otp_info.af_flag == 0x01) ||(hi846_otp_info.af_flag == 0x04))
        {
            *data = 1;
        }
    }
    else if (addr == 0x5)
    {
        for(i=0; i<size; i++){
            data[i] = hi846_otp_info.af_param[i];
            //pr_err("[ljt]af data[%d] =%x \n", i, data[i]);
        }
    }
    else
    {
        pr_err("[ljt]error addr\n", addr, size);
    }

    return size;
}
/*End jiantaohuang for LUNA84GVZW-3892, otp porting for lunavzw hi846 on 20221215*/

struct GC05A2_otp_struct {
    uint8_t  moduleinfo_flag;
    uint8_t  moduleinfo_checksum;
    uint8_t  lens_id;
    uint8_t  module_id;
    uint8_t  vcm_id;
    uint8_t  driver_ic_id;

    uint8_t  awb_flag;
    uint8_t  awb_param[12];
    uint8_t  awb_checksum;

    uint8_t  lsc_flag;
    uint8_t  lsc_param[1868];
    uint8_t  lsc_checksum;
};

extern struct GC05A2_otp_struct GC05A2_otp_info;

unsigned int gc05a2_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{
    int i=0;

    pr_err("[ljt]addr =%x size %d\n", addr, size);
    if (addr == 0x0) {
        *(u32 *)data = 0x010b00ff;
    } else if (addr == 0x1) {
        if ((GC05A2_otp_info.lsc_flag == 0x01) || (GC05A2_otp_info.lsc_flag == 0x04)) {
            pr_err("[ljt]lsc_flag valid\n");
            for(i=0; i<size; i++) {
                data[i] = GC05A2_otp_info.lsc_param[i];
            }
        }
    } else if (addr == 0x2) {
        if ((GC05A2_otp_info.awb_flag == 0x01) || (GC05A2_otp_info.awb_flag == 0x04)) {
            *data = 1;
        }
    } else if (addr == 0x3) {
        for(i=0; i<size; i++) {
            data[i] = GC05A2_otp_info.awb_param[i];
            //pr_err("[ljt]awb data[%d] =%x \n", i, data[i]);
        }
    } else {
        pr_err("[ljt]error addr\n", addr, size);
    }

    return size;
}
#endif
#if defined(TCT_CAMERA_PROJECT_CIVIC)
struct GC05A2_otp_struct {
    uint8_t  moduleinfo_flag;
    uint8_t  moduleinfo_checksum;
    uint8_t  lens_id;
    uint8_t  module_id;
    uint8_t  vcm_id;
    uint8_t  driver_ic_id;

    uint8_t  awb_flag;
    uint8_t  awb_param[16];
    uint8_t  awb_checksum;

    uint8_t  lsc_flag;
    uint8_t  lsc_param[1868];
    uint8_t  lsc_checksum;
};

extern struct GC05A2_otp_struct GC05A2_front_otp_info;


unsigned int gc05a2_front_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{
    int i=0;

    pr_err("[ljt]addr =%x size %d\n", addr, size);
    if (addr == 0x0) {
        *(u32 *)data = 0x010b00ff;
    } else if (addr == 0x1) {
        if ((GC05A2_front_otp_info.lsc_flag == 0x01) || (GC05A2_front_otp_info.lsc_flag == 0x13) || (GC05A2_front_otp_info.lsc_flag == 0x37)) {
            pr_err("[ljt]lsc_flag valid\n");
            for(i=0; i<size; i++) {
                data[i] = GC05A2_front_otp_info.lsc_param[i];
            }
        }
    } else if (addr == 0x2) {
        if ((GC05A2_front_otp_info.awb_flag == 0x01) || (GC05A2_front_otp_info.awb_flag == 0x13) || (GC05A2_front_otp_info.awb_flag == 0x37)) {
            *data = 1;
        }
    } else if (addr == 0x3) {
        // R
        data[0] = GC05A2_front_otp_info.awb_param[0];
        data[1] = GC05A2_front_otp_info.awb_param[1];
        // B
        data[2] = GC05A2_front_otp_info.awb_param[2];
        data[3] = GC05A2_front_otp_info.awb_param[3];
        // Gr
        data[4] = GC05A2_front_otp_info.awb_param[4];
        data[5] = GC05A2_front_otp_info.awb_param[5];
        // Gb
        data[6] = GC05A2_front_otp_info.awb_param[6];
        data[7] = GC05A2_front_otp_info.awb_param[7];
        // Golden R
        data[8] = GC05A2_front_otp_info.awb_param[8];
        data[9] = GC05A2_front_otp_info.awb_param[9];
        // Golden B
        data[10] = GC05A2_front_otp_info.awb_param[10];
        data[11] = GC05A2_front_otp_info.awb_param[11];
        // Golden Gr
        data[12] = GC05A2_front_otp_info.awb_param[12];
        data[13] = GC05A2_front_otp_info.awb_param[13];
         // Golden Gb
        data[14] = GC05A2_front_otp_info.awb_param[14];
        data[15] = GC05A2_front_otp_info.awb_param[15];
    } else {
        pr_err("[ljt]error addr\n", addr, size);
    }

    return size;
}
#endif
//End modified by chengyixuan for CIVICPL-3060 on 2022-07-16
// end modified by tct-hq/yzheng4 in 2022-12-09

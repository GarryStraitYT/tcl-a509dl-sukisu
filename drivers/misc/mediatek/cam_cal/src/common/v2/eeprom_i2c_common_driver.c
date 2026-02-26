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
#include "eeprom_utils.h"

/* Include platform define if necessary */
#ifdef EEPROM_PLATFORM_DEFINE
#include "eeprom_platform_def.h"
#endif


/* add for linux-4.4 */
#ifndef I2C_WR_FLAG
#define I2C_WR_FLAG		(0x1000)
#define I2C_MASK_FLAG	(0x00ff)
#endif

#define EEPROM_I2C_MSG_SIZE_READ 2

#ifndef EEPROM_I2C_READ_MSG_LENGTH_MAX
#define EEPROM_I2C_READ_MSG_LENGTH_MAX 1024
#endif
#ifndef EEPROM_I2C_WRITE_MSG_LENGTH_MAX
#define EEPROM_I2C_WRITE_MSG_LENGTH_MAX 1
#endif
#ifndef EEPROM_WRITE_EN
#define EEPROM_WRITE_EN 1
#endif

static int Read_I2C_CAM_CAL(struct i2c_client *client,
			    u16 a_u2Addr,
			    u32 ui4_length,
			    u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[2] = { (char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

	if (ui4_length > EEPROM_I2C_READ_MSG_LENGTH_MAX) {
		pr_debug("exceed one transition %d bytes limitation\n",
			 EEPROM_I2C_READ_MSG_LENGTH_MAX);
		return -1;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = puReadCmd;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = ui4_length;
	msg[1].buf = a_puBuff;

	i4RetValue = i2c_transfer(client->adapter, msg,
				EEPROM_I2C_MSG_SIZE_READ);

	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
		pr_debug("I2C read data failed!!\n");
		return -1;
	}

	return 0;
}

static int iReadData_CAM_CAL(struct i2c_client *client,
			     unsigned int ui4_offset,
			     unsigned int ui4_length,
			     unsigned char *pinputdata)
{
	int i4ResidueSize;
	u32 u4CurrentOffset, u4Size;
	u8 *pBuff;

	i4ResidueSize = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		u4Size = (i4ResidueSize >= EEPROM_I2C_READ_MSG_LENGTH_MAX)
			? EEPROM_I2C_READ_MSG_LENGTH_MAX : i4ResidueSize;

		if (Read_I2C_CAM_CAL(client, (u16) u4CurrentOffset,
				     u4Size, pBuff) != 0) {
			pr_debug("I2C iReadData failed!!\n");
			return -1;
		}

		i4ResidueSize -= u4Size;
		u4CurrentOffset += u4Size;
		pBuff += u4Size;
	} while (i4ResidueSize > 0);

	return 0;
}

#if EEPROM_WRITE_EN
static int Write_I2C_CAM_CAL(struct i2c_client *client,
			     u16 a_u2Addr,
			     u32 ui4_length,
			     u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puCmd[2 + EEPROM_I2C_WRITE_MSG_LENGTH_MAX];
	struct i2c_msg msg;

	if (ui4_length > EEPROM_I2C_WRITE_MSG_LENGTH_MAX) {
		pr_debug("exceed one transition %d bytes limitation\n",
			 EEPROM_I2C_WRITE_MSG_LENGTH_MAX);
		return -1;
	}

	puCmd[0] = (char)(a_u2Addr >> 8);
	puCmd[1] = (char)(a_u2Addr & 0xFF);
	memcpy(puCmd + 2, a_puBuff, ui4_length);

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 2 + ui4_length;
	msg.buf = puCmd;

	i4RetValue = i2c_transfer(client->adapter, &msg, 1);

	if (i4RetValue != 1) {
		pr_debug("I2C write data failed!!\n");
		return -1;
	}

	/* Wait for write complete */
	mdelay(5);

	return 0;
}

//Begin ersen.shang for [T10939600][orlando camera bring up] 202103
#define EEPROM_WRITE_PROTECT_DISABLE_WRITE_ADDR     0x8000
#define EEPROM_WRITE_PROTECT_DISABLE_WRITE_VALUE    0x0006
#define EEPROM_WRITE_PROTECT_ENABLE_WRITE_ADDR      0x8000
#define EEPROM_WRITE_PROTECT_ENABLE_WRITE_VALUE     0x000e

static bool disable_write_protect(struct i2c_client *client)
{
	u8 delay = 5;
	u8 disable_value = EEPROM_WRITE_PROTECT_DISABLE_WRITE_VALUE;
	if (Write_I2C_CAM_CAL(client,EEPROM_WRITE_PROTECT_DISABLE_WRITE_ADDR, 1, &disable_value) < 0)
	{
		pr_err("disable eeprom write  protect error.\n");
		return false;
	}
	mdelay(delay);
	pr_debug("disable eeprom write  protect ok.\n");
	return true;
}
static bool enable_write_protect(struct i2c_client *client)
{
	u8 delay = 5;
	u8 enable_value = EEPROM_WRITE_PROTECT_ENABLE_WRITE_VALUE;
	if (Write_I2C_CAM_CAL(client, EEPROM_WRITE_PROTECT_ENABLE_WRITE_ADDR, 1, &enable_value) < 0)
	{
		pr_debug("enable eeprom write  protect error.\n");
		return false;
	}
	mdelay(delay);
	pr_debug("enable eeprom write  protect ok.\n");
	return true;
}

//#undef DEBUG_LOG_WA
#define DEBUG_LOG_WA
#ifdef DEBUG_LOG_WA
static void logData(u16 len, u8 *buf, const char *tagname)
{
	u16 i = 0;
	for (; i < len; i++)
	{
		if (i % 0x10 == 0)  pr_err("%s data[%d] = 0x%x\n", tagname, i, buf[i]);
	}
}
#endif
//End    ersen.shang for [T10939600][orlando camera bring up] 202103

static int iWriteData_CAM_CAL(struct i2c_client *client,
				unsigned int ui4_offset,
				unsigned int ui4_length,
				unsigned char *pinputdata)
{
	int i4RetValue = 0;
	int i4ResidueSize;
	u32 u4CurrentOffset, u4Size;
	u8 *pBuff;

	//Begin ersen.shang for [T10939600][orlando camera bring up] 202103
	#ifdef DEBUG_LOG_WA
	u8 pTmpBuf[EEPROM_I2C_WRITE_MSG_LENGTH_MAX];
	u8 retry = 0, delay = 0;
	#endif

	pr_err("[CAM_CAL] iWriteData\n");
	pr_err("ui4_offset:0x%x, ui4_length:%d", ui4_offset, ui4_length);

	if (ui4_offset + ui4_length >= 0x4000){
		pr_debug("[CAM_CAL] Write Error!! S-24CS64A not supprt address >= 0x4000!!\n");
		return -1;
	}

	logData(ui4_length, pinputdata, "write");
	disable_write_protect(client);
	//End   ersen.shang for [T10939600][orlando camera bring up] 202103

	i4ResidueSize = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		u4Size = (i4ResidueSize >= EEPROM_I2C_WRITE_MSG_LENGTH_MAX)
			? EEPROM_I2C_WRITE_MSG_LENGTH_MAX : i4ResidueSize;

		if (Write_I2C_CAM_CAL(client, (u16) u4CurrentOffset,
					u4Size, pBuff) != 0) {
			pr_debug("I2C iWriteData failed!!\n");
			return -1;
		}
		//Begin ersen.shang for [T10939600][orlando camera bring up] 202103
		#ifdef DEBUG_LOG_WA
		do{
			logData(u4Size, pBuff, "write");
			i4RetValue = Read_I2C_CAM_CAL(client, (u16)u4CurrentOffset, u4Size, pTmpBuf);
			if (i4RetValue != 0){
				pr_debug("[CAM_CAL] I2C iReadData failed!!\n");
				return -1;
			}
			logData(u4Size, pTmpBuf, "read");
			if (0 == strncmp(pBuff, pTmpBuf, u4Size)){
				//printk("write addr 0x%0x data 0x%0x",offset,(u32)*(data + i));
				break;
			}else{
				printk("try to write offset(%x) retry: %d\n", u4CurrentOffset, retry);
				i4RetValue = Write_I2C_CAM_CAL(client, (u16)u4CurrentOffset, u4Size, pBuff);
				delay += 5;
				mdelay(delay);
			}
		} while (retry != 0);
		#endif
		//End   ersen.shang for [T10939600][orlando camera bring up] 202103
		i4ResidueSize -= u4Size;
		u4CurrentOffset += u4Size;
		pBuff += u4Size;
	} while (i4ResidueSize > 0);
	enable_write_protect(client);
	pr_err("[CAM_CAL] iWriteData done\n");
	return 0;
}
#endif

unsigned int Common_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;

	EEPROM_PROFILE_INIT(&t);
    pr_debug("Common_read_region addr 0x%x size %d\n", addr, size);
	if (iReadData_CAM_CAL(client, addr, size, data) == 0)
		ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
//add begain by yixuan.cheng for gc0a83 otp buffalo tmo
static char LscBuf[1870] = {0};
static char MdBuf[11] = {0};
static char AWBuf[14] = {0};
static char OnceRead = 0;
extern char CamOTPF_module_name[256];
typedef struct {
	char md_cs;
	char awb_cs;
	char lsc_cs;
} gc0a83_inf;
gc0a83_inf gc0a83inf = {0};		
unsigned int gc0a83_read_module(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	char buff[5] = {0x00, 0x00, 0x00, 0x20, 0x12};
	int sum = 0;
	int i = 0;
	u8 *pBuf = data;
	buff[1] = (addr)>>8&0xff;
	buff[2] = (addr)&0xff;
	Write_I2C_CAM_CAL(client,0x0313, 1,buff);//0x00);
	Write_I2C_CAM_CAL(client,0x0a69, 1,buff+1);//((addr+i) >> 8) & 0xff);
	Write_I2C_CAM_CAL(client,0x0a6a, 1,buff+2);//(addr+i) & 0xff);
	Write_I2C_CAM_CAL(client,0x0313, 1,buff+3);//0x20);
	Write_I2C_CAM_CAL(client,0x0313, 1,buff+4);//buff+0x12);
	for (i = 0; i < size; i++) {
		
		if (Read_I2C_CAM_CAL(client, 0x0a6c, 1, pBuf+i) != 0) {
			pr_debug("I2C iReadData failed!!\n");
			return -1;
		}	
		sum += pBuf[i];
		pr_err("cyx3addr = 0x%x, data = 0x%x sum = 0x%x\n", addr + i * 8, pBuf[i], sum);
	}
	return sum;	

}

unsigned int gc0a83_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	char buff[12] = {0x44, 0x09, 0x80, 0x00, 0x0e, 0x17, 0xA1, 0x00, 0x00, 0x0c,0x00,0x00};
	struct timeval t;
	EEPROM_PROFILE_INIT(&t);
	if(OnceRead == 0){
		OnceRead = 1;
		Write_I2C_CAM_CAL(client, 0x0324, 1, buff);
	    Write_I2C_CAM_CAL(client, 0x0316, 1, buff+1);
	    Write_I2C_CAM_CAL(client, 0x0a67, 1, buff+2);
	    Write_I2C_CAM_CAL(client, 0x0313, 1, buff+3);
	    Write_I2C_CAM_CAL(client, 0x0a53, 1, buff+4);
	    Write_I2C_CAM_CAL(client, 0x0a65, 1, buff+5);
	    Write_I2C_CAM_CAL(client, 0x0a68, 1, buff+6);
	    Write_I2C_CAM_CAL(client, 0x0a47, 1, buff+7);
	    Write_I2C_CAM_CAL(client, 0x0a58, 1, buff+8);
	    Write_I2C_CAM_CAL(client, 0x0ace, 1, buff+9);
	    mdelay(10);
		gc0a83_read_module(client, 0x15a0, MdBuf, 1);
		if(MdBuf[0] == 0x01){
			if (((gc0a83_read_module(client, 0x15a8, MdBuf+1, 10) - MdBuf[10]) % 256) == MdBuf[10])
			{
					gc0a83inf.md_cs = 1;
					pr_err("cyx module grpup 1 modulee");
			}
			else
			{
					memset(MdBuf, 0, 11);
					return  0;
			}
		}
		else if(((MdBuf[0]>>2)&3)==0x01){
			if (((gc0a83_read_module(client, 0x15f8, MdBuf+1, 10) - MdBuf[10]) % 256) == MdBuf[10])
			{
					gc0a83inf.md_cs = 1;
					pr_err("cyx module grpup 2 modulee");
			}
			else
			{
					memset(MdBuf, 0, 11);
					return  0;
			}
		}
		else{
			return 0;
		}	
		gc0a83_read_module(client, 0x1648, AWBuf, 1);
		if(AWBuf[0] == 0x01){
			if (((gc0a83_read_module(client, 0x1650, AWBuf+1, 13) - AWBuf[13]) % 256) == AWBuf[13])
			{
					gc0a83inf.awb_cs = 1;
					pr_err("cyx awb grpup 1 modulee");
			}
			else
			{
					memset(AWBuf, 0, 11);
					return  0;
			}
		}
		else if(((AWBuf[0]>>2)&3)==0x01){
			if (((gc0a83_read_module(client, 0x16b8, AWBuf+1, 13) - AWBuf[13]) % 256) == AWBuf[13])
			{
					gc0a83inf.awb_cs = 1;
					pr_err("cyx awb grpup 2 modulee");
			}
			else
			{
					memset(AWBuf, 0, 11);
					return  0;
			}
		}
		else {
			return 0;
		}
		gc0a83_read_module(client, 0x1720, LscBuf, 1);
		if(LscBuf[0] == 0x01){
			if (((gc0a83_read_module(client, 0x1728, LscBuf+1, 1869) - LscBuf[1869]) % 256) == LscBuf[1869])
			{
					gc0a83inf.lsc_cs = 1;
					pr_err("cyx LSC grpup 1 modulee");
			}
			else
			{
					memset(LscBuf, 0, 11);
					return  0;
			}
		}
		else if(((LscBuf[0]>>2)&3)==0x01){
			if (((gc0a83_read_module(client, 0x5190, LscBuf+1, 1869) - LscBuf[1869]) % 256) == LscBuf[1869])
			{
					gc0a83inf.lsc_cs = 1;
					pr_err("cyx LSC grpup 2 modulee");
			}
			else
			{
					memset(LscBuf, 0, 11);
					return  0;
			}
		}
		else{
			return 0;
		}
		Write_I2C_CAM_CAL(client, 0x0316, 1, buff+10);
		Write_I2C_CAM_CAL(client, 0x0a67, 1, buff+11);		
	}
	if((gc0a83inf.md_cs == 1)&&(addr==0x15a8)&&(size==1)){
		
		data[0]=MdBuf[1];
		ret = 1;
	}
	else if((gc0a83inf.md_cs == 1)&&(addr==0x0001)&&(size==1)){
		data[0] = MdBuf[0];
		ret = 1;
	}
	else if((gc0a83inf.awb_cs == 1)&&(addr==0x1650)&&(size==12)){
		memcpy(data, AWBuf+1, 12);
		ret = 12;
	}
	else if((gc0a83inf.lsc_cs == 1)&&(addr==0x1728)&&(size==1868)){
		memcpy(data, LscBuf+1, 1868);
		ret = 1868;
	}
	else
	{
		ret = 0;
	}
	if((gc0a83inf.md_cs == 1)&&(gc0a83inf.awb_cs == 1)&&(gc0a83inf.lsc_cs == 1)){
		sprintf(CamOTPF_module_name,"1");
	}else{
		sprintf(CamOTPF_module_name,"0");
	}
	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
//end begain by yixuan.cheng for gc0a83 otp buffalo tmo
unsigned int Common_write_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
#if EEPROM_WRITE_EN
	struct timeval t;

	EEPROM_PROFILE_INIT(&t);
    pr_debug("Common_write_region addr 0x%x size %d\n", addr, size);
	if (iWriteData_CAM_CAL(client, addr, size, data) == 0)
		ret = size;

	EEPROM_PROFILE(&t, "common_write_time");
#else
	pr_debug("Write operation disabled\n");
#endif

	return ret;
}

unsigned int DW9763_write_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
#if EEPROM_WRITE_EN
	struct timeval t;

	int i4RetValue = 0;
	char puCmd[2];
	struct i2c_msg msg;

	EEPROM_PROFILE_INIT(&t);

	puCmd[0] = (char)(0x81);
	puCmd[1] = (char)(0xEE);

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 2;
	msg.buf = puCmd;

	i4RetValue = i2c_transfer(client->adapter, &msg, 1);

	if (i4RetValue != 1) {
		pr_debug("I2C erase data failed!!\n");
		return -1;
	}

	/* Wait for erase complete */
	mdelay(30);

	if (iWriteData_CAM_CAL(client, addr, size, data) == 0)
		ret = size;

	EEPROM_PROFILE(&t, "DW9763_write_time");
#else
	pr_debug("Write operation disabled\n");
#endif

	return ret;
}

unsigned int BL24SA64_write_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
#if EEPROM_WRITE_EN
	struct timeval t;

	unsigned char test_read = 0x00;
	unsigned char unlock_cmd = 0x40;
	unsigned char lock_cmd = 0x78;
	unsigned char unlock_val = 0x00 | ((client->addr) & 0x7) << 4; // 0x50 -> 0x00
	unsigned char lock_val = unlock_val | 0x0F; // 0x50 -> 0x0F
	unsigned int ori_addr = 0x00; // to store the current address during sending cmd.
	unsigned int exp_addr = 0x00; // to store the expected address after lock EEPROM.
	unsigned int i = 0;

	EEPROM_PROFILE_INIT(&t);

/************ test read EEPROM ************/

	exp_addr = client->addr;
	if (iReadData_CAM_CAL(client, 0x0008, 1, &test_read) < 0) {
		pr_debug("Read EEPROM ID failed\n");
		pr_debug("Start looping slave address 0x50 ~ 0x57\n");
		for (i = 0; i < 8; i++) {
			client->addr = 0x50+i;
			pr_debug("Change slave address to 0x%02x\n", client->addr);
			if (iReadData_CAM_CAL(client, 0x0008, 1, &test_read) == 0) {
				pr_debug("EEPROM ID = 0x%02x\n", test_read);
				break;
			}
		}
	} else
		pr_debug("EEPROM ID = 0x%02x\n", test_read);

	if (iReadData_CAM_CAL(client, 0x8000, 1, &test_read) < 0) {
		pr_debug("Read register failed\n");
		return -1;
	}
	pr_debug("Register ID = 0x%02x\n", test_read);

/************ unlock EEPROM ************/

	pr_debug("BL24SA64 write unlock 0x%02x\n", unlock_val);

	ori_addr = client->addr;
	client->addr = unlock_cmd;

	iWriteData_CAM_CAL(client, 0x8000, 1, &unlock_val);
	pr_debug("BL24SA64 unlock part1\n");

	client->addr = ori_addr;

	if (iWriteData_CAM_CAL(client, 0x8000, 1, &unlock_val) < 0) {
		pr_debug("Unlock protection failed!!\n");
		return -1;
	}
	pr_debug("BL24SA64 unlock done\n");

/************ test read EEPROM ************/

	if (iReadData_CAM_CAL(client, 0x8000, 1, &test_read) == 0)
		pr_debug("Register ID = 0x%02x\n", test_read);
	else {
		pr_debug("Read register failed!!\n");
		return -1;
	}

/************ write EEPROM ************/

	if (iWriteData_CAM_CAL(client, addr, size, data) < 0)
		pr_debug("Write EEPROM failed!!\n");
	else
		ret = size;
	pr_debug("Write EEPROM ret = %d\n", ret);

/************ lock EEPROM ************/

	pr_debug("BL24SA64 write lock 0x%02x\n", lock_val);

	ori_addr = client->addr;
	client->addr = lock_cmd;

	iWriteData_CAM_CAL(client, 0x8000, 1, &lock_val);
	pr_debug("BL24SA64 lock part1\n");

	client->addr = ori_addr;

	if (iWriteData_CAM_CAL(client, 0x8000, 1, &lock_val) < 0) {
		pr_debug("Lock protection failed!!\n");
		return -1;
	}
	pr_debug("BL24SA64 lock done\n");

/************ test read EEPROM ************/

	client->addr = exp_addr;
	if (iReadData_CAM_CAL(client, 0x8000, 1, &test_read) == 0)
		pr_debug("Register ID = 0x%02x\n", test_read);
	else {
		pr_debug("Failed to read register!!\n");
		return -1;
	}

/***************************************/

	EEPROM_PROFILE(&t, "BL24SA64_write_time");
#else
	pr_debug("Write operation disabled\n");
#endif

	return ret;
}

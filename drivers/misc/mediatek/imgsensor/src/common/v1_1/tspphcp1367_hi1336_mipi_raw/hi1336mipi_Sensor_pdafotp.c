#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/types.h>

//#include "hi1336_mipi_raw_Sensor.h"

#define PFX "HI1336_pdafotp"
#define LOG_INF(format, args...)                                               \
	pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
		       u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId, u8 number);

#define USHORT unsigned short
#define BYTE unsigned char
#define Sleep(ms) mdelay(ms)

#define HI1336_EEPROM_READ_ID 0xA2
#define HI1336_EEPROM_WRITE_ID 0xA3
#define HI1336_I2C_SPEED 100
#define HI1336_MAX_OFFSET 0xFFFF

#define DATA_SIZE 2048

#define READ_EEPROM_ID 0xB0
#define I2C_SPEED 100

#define EEPROM_MODULEINFO_FLAG (0x0001)
#define EEPROM_MODULEINFO_VALUE (0x0002)
#define EEPROM_MODULEINFO_CHKSUM (0x0004)
#define EEPROM_AWB_FLAG (0x0008)
#define EEPROM_AWB_CHKSUM (0x0010)
#define EEPROM_LSC_FLAG (0x0020)
#define EEPROM_LSC_CHKSUM (0x0040)
#define EEPROM_AF_FLAG (0x0080)
#define EEPROM_AF_CHKSUM (0x0100)
#define EEPROM_PDAF_FLAG (0x0200)
#define EEPROM_PDAF_CHKSUM (0x0400)
#define EEPROM_AWB_GOLDEN_FLAG (0x0800)
#define EEPROM_AWB_GOLDEN_CHKSUM (0x1000)

#define EEPROM_READ_ID 0xA0
#define EEPROM_WRITE_ID 0xA1

#define MODULE_INFO_START_ADDR 0x0000
#define AWB_INFO_START_ADDR 0x000B
#define LSC_INFO_START_ADDR 0x0B05
#define AF_INFO_START_ADDR 0x0708
#define PDAF_INFO_START_ADDR 0x1253

typedef struct moduleInformation_struct {
	BYTE moduleInoramtionFlag;
	BYTE moduleID;
	BYTE Year;
	BYTE Month;
	BYTE Day;
	BYTE LENSID;
	BYTE VCMID;
	BYTE DriverICID;
	BYTE reserved1;
	BYTE reserved2;
	BYTE chksum;
} moduleInformation_struct;

typedef struct awb_struct {
	BYTE awbFlag;
	BYTE awbInformation[17];
	BYTE chksum;
} awb_struct;

typedef struct lsc_struct {
	BYTE lscFlag;
	BYTE lscData[1868];
	BYTE chksum;
} lsc_struct;

typedef struct af_struct {
	BYTE afFlag;
	BYTE afData[23];
	BYTE chksum;
} af_struct;

typedef struct pdaf_struct {
	BYTE pdafFlag;
	BYTE pdafData1[496];
	BYTE pdafData2[1004];
	BYTE chksum1;
	BYTE chksum2;
} pdaf_struct;

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
		       u16 a_sizeRecvData, u16 i2cId);

static bool read_hi1336_eeprom(kal_uint16 addr, BYTE *data, int size)
{
	int i = 0;
	int offset = addr;
	int ret;
	u8 pu_send_cmd[2];

#define MAX_READ_WRITE_SIZE 255

	for (i = 0; i < size; i += MAX_READ_WRITE_SIZE) {
		pu_send_cmd[0] = (u8)(offset >> 8);
		pu_send_cmd[1] = (u8)(offset & 0xFF);

		if (i + MAX_READ_WRITE_SIZE > size) {
			ret = iReadRegI2C(pu_send_cmd, 2, (u8 *)(data + i),
					  (size - i), EEPROM_READ_ID);
		} else {
			ret = iReadRegI2C(pu_send_cmd, 2, (u8 *)(data + i),
					  MAX_READ_WRITE_SIZE, EEPROM_READ_ID);
		}
		if (ret < 0) {
			LOG_INF("read spc failed!\n");
			return false;
		}

		offset += MAX_READ_WRITE_SIZE;
	}

	LOG_INF("exit _read_eeprom size = %d\n", size);
	return true;
}

static void dump_data(void)
{
#if 0
	int i;
	BYTE *buf=NULL;

	buf = kmalloc(DATA_SIZE, GFP_KERNEL);
	read_ov16e10_eeprom(0x0, buf, DATA_SIZE);

	for(i = 0 ; i < DATA_SIZE/0x10 ; i++)
		LOG_INF("addr 0x%8x:  0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x", \
		i*0x10,\
		buf[i*0x10+0],buf[i*0x10+1],buf[i*0x10+ 2],buf[i*0x10+ 3],buf[i*0x10+ 4],buf[i*0x10+ 5],buf[i*0x10+ 6],buf[i*0x10+ 7],\
		buf[i*0x10+8],buf[i*0x10+9],buf[i*0x10+10],buf[i*0x10+11],buf[i*0x10+12],buf[i*0x10+13],buf[i*0x10+14],buf[i*0x10+15]);

	kfree(buf);
    return;
#else
	return;
#endif
}

static bool check_sum(BYTE *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 1; i <= size - 2; i++) {
		sum += buf[i];
		LOG_INF("buf[%d] = 0x%x %d", i, buf[i], buf[i]);
	}

	if ((sum % 256) != buf[size - 1]) {
		LOG_INF("chksum fail size = %d sum=%d sum-in-eeprom=%d", size,
			(sum % 256), buf[size - 1]);
		return false;
	}
	return true;
}

static bool check_sum_awb(awb_struct *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 0; i < sizeof(buf->awbInformation); i++) {
		sum += buf->awbInformation[i];
		LOG_INF("awbData[%d] = 0x%x %d", i, buf->awbInformation[i],
			buf->awbInformation[i]);
	}

	if ((sum % 256) != buf->chksum) {
		LOG_INF("chksum fail size = %d sum=%d sum-in-eeprom=%d", size,
			(sum % 256), buf->chksum);
		return false;
	}
	return true;
}

static bool check_sum_lsc(lsc_struct *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 0; i < sizeof(buf->lscData); i++) {
		sum += buf->lscData[i];
	}

	if ((sum % 256) != buf->chksum) {
		LOG_INF("chksum fail size = %d sum=%d sum-in-eeprom=%d", size,
			(sum % 256), buf->chksum);
		return false;
	}
	return true;
}

static bool check_sum_af(af_struct *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 0; i < sizeof(buf->afData); i++) {
		sum += buf->afData[i];
		LOG_INF("afData[%d] = 0x%x %d", i, buf->afData[i],
			buf->afData[i]);
	}

	if ((sum % 256) != buf->chksum) {
		LOG_INF("chksum fail size = %d sum=%d sum-in-eeprom=%d", size,
			(sum % 256), buf->chksum);
		return false;
	}
	return true;
}

static bool check_sum_pdaf(pdaf_struct *buf, unsigned int size)
{
	int i, sum1 = 0, sum2 = 0;

	for (i = 0; i < 496; i++) {
		sum1 += buf->pdafData1[i];
	}

	for (i = 0; i < 1004; i++) {
		sum2 += buf->pdafData2[i];
	}

	if ((sum1 % 256) != buf->chksum1 || (sum2 % 256) != buf->chksum2) {
		LOG_INF("chksum fail size = %d sum1=%d sum1-in-eeprom=%d sum2=%d sum2-in-eeprom=%d",
			size, (sum1 % 256), buf->chksum1, (sum2 % 256),
			buf->chksum2);
		return false;
	}
	return true;
}

bool check_tspphcp1367hi1336_otp(void)
{
	moduleInformation_struct *pModuleInfo;
	awb_struct *pAWBInfo;
	lsc_struct *pLSCInfo;
	af_struct *pAFInfo;
	pdaf_struct *pPDAFInfo;
	unsigned int otpStatus = 0;

	pModuleInfo = kmalloc(sizeof(moduleInformation_struct), GFP_KERNEL);
	read_hi1336_eeprom(MODULE_INFO_START_ADDR, (BYTE *)pModuleInfo,
			   sizeof(moduleInformation_struct));
	if (pModuleInfo->moduleInoramtionFlag == 0x01) {
		otpStatus |= EEPROM_MODULEINFO_FLAG;
		if (check_sum((BYTE *)pModuleInfo,
			      sizeof(moduleInformation_struct))) {
			otpStatus |= EEPROM_MODULEINFO_CHKSUM;
			if (pModuleInfo->moduleID == 0x48 &&
			    pModuleInfo->LENSID == 0x21 &&
			    pModuleInfo->VCMID == 0xe4 &&
			    pModuleInfo->DriverICID == 0x13) {
				otpStatus |= EEPROM_MODULEINFO_VALUE;
				LOG_INF("module info flag chksum value pass");
			} else {
				LOG_INF("moduleID =0x%x,LENSID=0x%x,VCMID=0x%x,DriverICID=0x%x",
					pModuleInfo->moduleID,
					pModuleInfo->LENSID, pModuleInfo->VCMID,
					pModuleInfo->DriverICID);
			}
		}
	} else {
		LOG_INF("moduleInoramtionFlag=%d",
			pModuleInfo->moduleInoramtionFlag);
	}
	kfree(pModuleInfo);

	pAWBInfo = kmalloc(sizeof(awb_struct), GFP_KERNEL);
	read_hi1336_eeprom(AWB_INFO_START_ADDR, (BYTE *)pAWBInfo,
			   sizeof(awb_struct));
	if (pAWBInfo->awbFlag == 0x01) {
		otpStatus |= EEPROM_AWB_FLAG;
		if (check_sum_awb(pAWBInfo, sizeof(awb_struct))) {
			otpStatus |= EEPROM_AWB_CHKSUM;
			LOG_INF("awb flag chksum pass");
		} else {
			int i;
			for (i = 0; i < sizeof(pAWBInfo->awbInformation); i++)
				LOG_INF("awb[%d]=0x%x  %d\n", i,
					pAWBInfo->awbInformation[i],
					pAWBInfo->awbInformation[i]);
		}
	} else {
		LOG_INF("awbFlag=%d", pAWBInfo->awbFlag);
	}
	kfree(pAWBInfo);

	pLSCInfo = kmalloc(sizeof(lsc_struct), GFP_KERNEL);
	read_hi1336_eeprom(LSC_INFO_START_ADDR, (BYTE *)pLSCInfo,
			   sizeof(lsc_struct));
	if (pLSCInfo->lscFlag == 0x01) {
		otpStatus |= EEPROM_LSC_FLAG;
		if (check_sum_lsc(pLSCInfo, sizeof(lsc_struct))) {
			otpStatus |= EEPROM_LSC_CHKSUM;
			LOG_INF("lsc flag chksum pass");
		}
	} else {
		LOG_INF("lscFlag=%d", pLSCInfo->lscFlag);
	}
	kfree(pLSCInfo);

	pAFInfo = kmalloc(sizeof(af_struct), GFP_KERNEL);
	read_hi1336_eeprom(AF_INFO_START_ADDR, (BYTE *)pAFInfo,
			   sizeof(af_struct));
	if (pAFInfo->afFlag == 0x01) {
		otpStatus |= EEPROM_AF_FLAG;
		if (check_sum_af(pAFInfo, sizeof(af_struct))) {
			otpStatus |= EEPROM_AF_CHKSUM;
			LOG_INF("af flag chksum pass");
		}
	} else {
		LOG_INF("afFlag=%d", pAFInfo->afFlag);
	}
	kfree(pAFInfo);

	pPDAFInfo = kmalloc(sizeof(pdaf_struct), GFP_KERNEL);
	read_hi1336_eeprom(PDAF_INFO_START_ADDR, (BYTE *)pPDAFInfo,
			   sizeof(pdaf_struct));
	if (pPDAFInfo->pdafFlag == 0x01) {
		otpStatus |= EEPROM_PDAF_FLAG;
		if (check_sum_pdaf(pPDAFInfo, sizeof(pdaf_struct))) {
			otpStatus |= EEPROM_PDAF_CHKSUM;
			LOG_INF("pdaf flag chksum pass");
		} else {
			int i;
			for (i = 0; i < 5; i++)
				LOG_INF("pdaf[%d]=0x%x", i,
					pPDAFInfo->pdafData1[i]);
		}
	} else {
		int i;
		LOG_INF("pdafFlag=%d", pPDAFInfo->pdafFlag);

		for (i = 0; i < 5; i++)
			LOG_INF("pdaf[%d]=0x%x", i, pPDAFInfo->pdafData1[i]);
	}
	kfree(pPDAFInfo);

	dump_data();

	if (otpStatus ==
	    (EEPROM_MODULEINFO_CHKSUM | EEPROM_MODULEINFO_FLAG |
	     EEPROM_MODULEINFO_VALUE | EEPROM_AWB_CHKSUM | EEPROM_AWB_FLAG |
	     EEPROM_AF_CHKSUM | EEPROM_AF_FLAG | EEPROM_LSC_CHKSUM |
	     EEPROM_LSC_FLAG | EEPROM_PDAF_CHKSUM | EEPROM_PDAF_FLAG)) {
		return true;
	} else {
		LOG_INF("otp check fail otpStatus=0x%x", otpStatus);
		return false;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

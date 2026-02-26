#undef PFX
#define PFX "ov13b10_eeprom"
#define pr_fmt(fmt) PFX "[%s](%d) " fmt, __func__, __LINE__

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#define USHORT unsigned short
#define BYTE unsigned char
#define Sleep(ms) mdelay(ms)

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
#define EEPROM_HWGGC_CHKSUM (0x2000)
#define EEPROM_HWGGC_FLAG (0x4000)

#define EEPROM_READ_ID  0xA2
#define EEPROM_WRITE_ID 0xA3

#define DATA_SIZE              0x4000
#define MAX_READ_WRITE_SIZE    255

#define MODULE_INFO_START_ADDR 0x0000
#define AWB_INFO_START_ADDR    0x0020
#define LSC_INFO_START_ADDR    0x0800


typedef struct moduleInformation_struct
{
	BYTE moduleInoramtionFlag;
    BYTE moduleID;
	BYTE Year;
	BYTE Month;
	BYTE Day;
	BYTE LENSID;
	BYTE VCMID;
	BYTE DriverICID;
    BYTE reseved[13];
	BYTE chksum;
} moduleInformation_struct;

typedef struct awb_struct
{
	BYTE awbFlag;
	BYTE awbInformation[12];
	BYTE chksum;
} awb_struct;

typedef struct lsc_struct
{
	BYTE lscFlag;
	BYTE lscData[1868];
	BYTE chksum;
} lsc_struct;



extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
					   u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

static bool read_shinetech_ov13b10_eeprom(kal_uint16 addr, BYTE *data, int size)
{
	int i = 0;
	int offset = addr;
	int ret;
	u8 pu_send_cmd[2];

	for (i = 0; i < size; i += MAX_READ_WRITE_SIZE)
	{
		pu_send_cmd[0] = (u8)(offset >> 8);
		pu_send_cmd[1] = (u8)(offset & 0xFF);

		if (i + MAX_READ_WRITE_SIZE > size)
		{
			ret = iReadRegI2C(pu_send_cmd, 2,
							  (u8 *)(data + i),
							  (size - i),
							  EEPROM_READ_ID);
		}
		else
		{
			ret = iReadRegI2C(pu_send_cmd, 2,
							  (u8 *)(data + i),
							  MAX_READ_WRITE_SIZE,
							  EEPROM_READ_ID);
		}
		if (ret < 0)
		{
			pr_err("read spc failed!\n");
			return false;
		}

		offset += MAX_READ_WRITE_SIZE;
	}

	pr_err("exit _read_eeprom size = %d\n", size);
	return true;
}

static bool check_sum(BYTE *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 1; i <= size - 2; i++)
	{
		sum += buf[i];
		pr_err("buf[%d] = 0x%x %d",i,buf[i],buf[i]);
	}

	if ((sum % 255 + 1) != buf[size - 1])
	{
		pr_err("check_sum fail size = %d sum=%d sum-in-eeprom=%d", size, (sum % 255 + 1), buf[size - 1]);
		return false;
	}
	return true;
}

static bool check_sum_awb(awb_struct *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 0; i < sizeof(buf->awbInformation); i++)
	{
		sum += buf->awbInformation[i];
		pr_err("awbData[%d] = 0x%x %d",i,buf->awbInformation[i],buf->awbInformation[i]);
	}

	if ((sum % 255 + 1) != buf->chksum)
	{
		pr_err("check_sum_awb fail size = %d sum=%d sum-in-eeprom=%d", size, (sum % 255 + 1), buf->chksum);
		return false;
	}
	return true;
}

static bool check_sum_lsc(lsc_struct *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 0; i < sizeof(buf->lscData); i++)
	{
		sum += buf->lscData[i];
		if(i < 5)
			pr_err("lscData[%d] = 0x%x %d",i,buf->lscData[i],buf->lscData[i]);
	}

	if ((sum % 255 + 1) != buf->chksum)
	{
		pr_err("check_sum_lsc fail size = %d sum=%d sum-in-eeprom=%d", size, (sum % 255 + 1), buf->chksum);
		return false;
	}
	return true;
}


bool check_ov13b10_otp(void)
{
	moduleInformation_struct *pModuleInfo;
	awb_struct *pAWBInfo;
	lsc_struct *pLSCInfo;
	unsigned int otpStatus = 0;
	int i = 0;

	pModuleInfo = kmalloc(sizeof(moduleInformation_struct), GFP_KERNEL);
    read_shinetech_ov13b10_eeprom(MODULE_INFO_START_ADDR, (BYTE *)pModuleInfo, sizeof(moduleInformation_struct));
	if (pModuleInfo->moduleInoramtionFlag == 0x01)
	{
		otpStatus |= EEPROM_MODULEINFO_FLAG;
		if (check_sum((BYTE *)pModuleInfo, sizeof(moduleInformation_struct)))
		{
			otpStatus |= EEPROM_MODULEINFO_CHKSUM;
			if (pModuleInfo->moduleID   == 0x43 &&
				pModuleInfo->LENSID     == 0xa1 &&
				pModuleInfo->VCMID      == 0x00 &&
				pModuleInfo->DriverICID == 0x00)
			{
				otpStatus |= EEPROM_MODULEINFO_VALUE;
				pr_err("module info  chksum value pass");
			}
			else
			{
				pr_err("moduleID =0x%x,LENSID=0x%x,VCMID=0x%x,DriverICID=0x%x",
					   pModuleInfo->moduleID,
					   pModuleInfo->LENSID,
					   pModuleInfo->VCMID,
					   pModuleInfo->DriverICID);
			}
		}
        else
        {
            pr_err("check_sum fail  pModuleInfo ");
        }
	}
	else
	{
		pr_err("error  moduleInoramtionFlag=%d", pModuleInfo->moduleInoramtionFlag);
	}
	kfree(pModuleInfo);

	pAWBInfo = kmalloc(sizeof(awb_struct), GFP_KERNEL);
	read_shinetech_ov13b10_eeprom(AWB_INFO_START_ADDR, (BYTE *)pAWBInfo, sizeof(awb_struct));
	if (pAWBInfo->awbFlag == 0x01)
	{
		otpStatus |= EEPROM_AWB_FLAG;
		if (check_sum_awb(pAWBInfo, sizeof(awb_struct)))
		{
			otpStatus |= EEPROM_AWB_CHKSUM;
			pr_err("awb  chksum pass");
		}
		else
		{
            pr_err("fail  check_sum_ awb ");
			for(i = 0 ; i < sizeof(pAWBInfo->awbInformation); i++)
				pr_err("awb[%d]=0x%x  %d\n",i,pAWBInfo->awbInformation[i],pAWBInfo->awbInformation[i]);
		}
	}
	else
	{
		pr_err("error awbFlag=%d", pAWBInfo->awbFlag);
	}
	kfree(pAWBInfo);

	pLSCInfo = kmalloc(sizeof(lsc_struct), GFP_KERNEL);
	read_shinetech_ov13b10_eeprom(LSC_INFO_START_ADDR, (BYTE *)pLSCInfo, sizeof(lsc_struct));
	if (pLSCInfo->lscFlag == 0x01)
	{
		otpStatus |= EEPROM_LSC_FLAG;
		if (check_sum_lsc(pLSCInfo, sizeof(lsc_struct)))
		{
			otpStatus |= EEPROM_LSC_CHKSUM;
			pr_err("lsc  chksum pass");
	}
	else
	{
            pr_err(" error  check_sum_ lsc ");
	}
		}
		else
		{
		pr_err(" error  lscFlag=%d", pLSCInfo->lscFlag);
		}
	kfree(pLSCInfo);


	if (otpStatus == (EEPROM_MODULEINFO_CHKSUM | EEPROM_MODULEINFO_FLAG | EEPROM_MODULEINFO_VALUE |
					  EEPROM_AWB_CHKSUM | EEPROM_AWB_FLAG |
					  EEPROM_LSC_CHKSUM | EEPROM_LSC_FLAG))
	{
        pr_err("otp check all  ok !!!!  otpStatus=0x%x", otpStatus);
		return true;
	}
	else
	{
		pr_err("otp check fail otpStatus=0x%x", otpStatus);
		return true;
	}
}

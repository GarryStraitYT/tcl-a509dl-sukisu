#undef PFX
#define PFX "s5kjn1_eeprom"
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

#define EEPROM_HWGGC_CHKSUM       (0x2000)
#define EEPROM_HWGGC_FLAG         (0x4000)

#define EEPROM_READ_ID  0xA0
#define EEPROM_WRITE_ID 0xA1

#define DATA_SIZE              0x4000
#define MAX_READ_WRITE_SIZE    255

#define MODULE_INFO_START_ADDR 0x0000
#define AWB_INFO_START_ADDR    0x000B
#define LSC_INFO_START_ADDR    0x0B05
#define AF_INFO_START_ADDR     0x0708
#define PDAF_INFO_START_ADDR   0x1253
#define HWGGC_START_ADDR       0x378E

typedef struct moduleInformation_struct
{
    BYTE moduleInoramtionFlag;
    BYTE moduleID;
    BYTE Year;
    BYTE Month;
    BYTE Day;
    BYTE LENSID;
    BYTE VCMID;
	BYTE EEPROMID;
    BYTE DriverICID;
    BYTE reseved;
    BYTE chksum;
} moduleInformation_struct;

typedef struct awb_struct
{
	BYTE awbFlag;
	BYTE awbInformation[12];
	BYTE reseved[5];
	BYTE chksum;
} awb_struct;

typedef struct lsc_struct
{
	BYTE lscFlag;
	BYTE lscData[1868];
	BYTE chksum;
} lsc_struct;

typedef struct af_struct
{
	BYTE afFlag;
	BYTE afData[4];
	BYTE reseved[19];
	BYTE chksum;
} af_struct;

typedef struct pdaf_struct
{
	BYTE pdafFlag;
	BYTE pdafData1[496];
	BYTE pdafData2[1004];
	BYTE chksum1;
	BYTE chksum2;
} pdaf_struct;

typedef struct hw_ggc
{
	BYTE hwGGCFlag;
	BYTE hwGGCData[346];
	BYTE chksum;
} hwggc_struct;

hwggc_struct hwggc;

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
					   u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

static bool read_tsp_s5kjn1_eeprom(kal_uint16 addr, BYTE *data, int size)
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

	pr_info("exit _read_eeprom size = %d\n", size);
	return true;
}

static bool check_sum(BYTE *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 1; i <= size - 2; i++)
	{
		sum += buf[i];
		pr_info("buf[%d] = 0x%x %d",i,buf[i],buf[i]);
	}

	if ((sum % 256) != buf[size - 1])
	{
		pr_err("chksum fail size = %d sum=%d sum-in-eeprom=%d", size, (sum % 256), buf[size - 1]);
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
		pr_info("awbData[%d] = 0x%x %d",i,buf->awbInformation[i],buf->awbInformation[i]);
	}

	if ((sum % 256) != buf->chksum)
	{
		pr_err("chksum fail size = %d sum=%d sum-in-eeprom=%d", size, (sum % 256), buf->chksum);
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
	}

	if ((sum % 256) != buf->chksum)
	{
		pr_err("chksum fail size = %d sum=%d sum-in-eeprom=%d", size, (sum % 256), buf->chksum);
		return false;
	}
	return true;
}

static bool check_sum_af(af_struct *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 0; i < sizeof(buf->afData); i++)
	{
		sum += buf->afData[i];
		pr_info("afData[%d] = 0x%x %d",i,buf->afData[i],buf->afData[i]);
	}

	if ((sum % 256) != buf->chksum)
	{
		pr_err("chksum fail size = %d sum=%d sum-in-eeprom=%d", size, (sum % 256), buf->chksum);
		return false;
	}
	return true;
}

static bool check_sum_pdaf(pdaf_struct *buf, unsigned int size)
{
	int i, sum1 = 0;
        int sum2 = 0;
	for (i = 0; i < sizeof(buf->pdafData1); i++)
	{
		sum1 += buf->pdafData1[i];
	}
	for (i = 0; i < sizeof(buf->pdafData2); i++)
	{
		sum2 += buf->pdafData2[i];
	}

	if ((sum1 % 256) != buf->chksum1)
	{
		pr_err("s3kjn1 pdaf chksum1 fail size = %d sum1=%d sum1-in-eeprom=%d", size, (sum1 % 256), buf->chksum1);
		return false;
	}
        if ((sum2 % 256) != buf->chksum2)
	{
		pr_err("s3kjn1 pdaf chksum2 fail size = %d sum2=%d sum2-in-eeprom=%d", size, (sum2 % 256), buf->chksum2);
		return false;
	}
	return true;
}

BYTE get_tsppsnp1082_s5kjn1_lensID(void)
{
	moduleInformation_struct *pModuleInfo;
	BYTE retVal = 0;

	pModuleInfo = kmalloc(sizeof(moduleInformation_struct), GFP_KERNEL);
	read_tsp_s5kjn1_eeprom(MODULE_INFO_START_ADDR, (BYTE *)pModuleInfo, sizeof(moduleInformation_struct));

	if (pModuleInfo->moduleInoramtionFlag == 0x01)
	{
		retVal = pModuleInfo->LENSID;
	}
	kfree(pModuleInfo);

	pr_err("get_tsppsnp1082_s5kjn1_lensID retVal = %d", retVal);
	return retVal;
}

bool check_tsp_s5kjn1_otp(void)
{
    moduleInformation_struct *pModuleInfo;
    awb_struct *pAWBInfo;
    lsc_struct *pLSCInfo;
    af_struct  *pAFInfo;
    pdaf_struct *pPDAFInfo;
    unsigned int otpStatus = 0;
    int i = 0;

    pModuleInfo = kmalloc(sizeof(moduleInformation_struct), GFP_KERNEL);
    read_tsp_s5kjn1_eeprom(MODULE_INFO_START_ADDR, (BYTE *)pModuleInfo, sizeof(moduleInformation_struct));
    if (pModuleInfo->moduleInoramtionFlag == 0x01)
    {
		otpStatus |= EEPROM_MODULEINFO_FLAG;
		if (check_sum((BYTE *)pModuleInfo, sizeof(moduleInformation_struct)))
		{
			otpStatus |= EEPROM_MODULEINFO_CHKSUM;
			if (pModuleInfo->moduleID   == 0x48 && 
				pModuleInfo->LENSID     == 0x01 && 
				pModuleInfo->VCMID      == 0xe1 &&
				pModuleInfo->EEPROMID   == 0x06 &&
				pModuleInfo->DriverICID == 0x0e)
			{
				otpStatus |= EEPROM_MODULEINFO_VALUE;
				pr_info("module info  chksum value pass");
			}
			else
			{
				pr_err("moduleID =0x%x,LENSID=0x%x,VCMID=0x%x,EEPROMID=0x%x,DriverICID=0x%x",
					   pModuleInfo->moduleID,
					   pModuleInfo->LENSID,
					   pModuleInfo->VCMID,
					   pModuleInfo->EEPROMID,
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
	read_tsp_s5kjn1_eeprom(AWB_INFO_START_ADDR, (BYTE *)pAWBInfo, sizeof(awb_struct));
	if (pAWBInfo->awbFlag == 0x01)
	{
		otpStatus |= EEPROM_AWB_FLAG;
		if (check_sum_awb(pAWBInfo, sizeof(awb_struct)))
		{
			otpStatus |= EEPROM_AWB_CHKSUM;
			pr_info("awb  chksum pass");
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
	read_tsp_s5kjn1_eeprom(LSC_INFO_START_ADDR, (BYTE *)pLSCInfo, sizeof(lsc_struct));
	if (pLSCInfo->lscFlag == 0x01)
	{
		otpStatus |= EEPROM_LSC_FLAG;
		if (check_sum_lsc(pLSCInfo, sizeof(lsc_struct)))
		{
			otpStatus |= EEPROM_LSC_CHKSUM;
			pr_info("lsc  chksum pass");
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

	pAFInfo = kmalloc(sizeof(af_struct), GFP_KERNEL);
	read_tsp_s5kjn1_eeprom(AF_INFO_START_ADDR, (BYTE *)pAFInfo, sizeof(af_struct));
	if (pAFInfo->afFlag == 0x01)
	{
		otpStatus |= EEPROM_AF_FLAG;
		if (check_sum_af(pAFInfo, sizeof(af_struct)))
		{
			otpStatus |= EEPROM_AF_CHKSUM;
			pr_info("af  chksum pass");
		}
        else
        {
            pr_err(" error  check_sum_ af ");
        }
	}
	else
	{
		pr_err("error afFlag=%d", pAFInfo->afFlag);
	}
	kfree(pAFInfo);

	pPDAFInfo = kmalloc(sizeof(pdaf_struct), GFP_KERNEL);
	read_tsp_s5kjn1_eeprom(PDAF_INFO_START_ADDR, (BYTE *)pPDAFInfo, sizeof(pdaf_struct));
	if (pPDAFInfo->pdafFlag == 0x01)
	{
		otpStatus |= EEPROM_PDAF_FLAG;
		if (check_sum_pdaf(pPDAFInfo, sizeof(pdaf_struct)))
		{
			otpStatus |= EEPROM_PDAF_CHKSUM;
			pr_info("pdaf  chksum pass");
		}
		else
		{
            pr_err(" error  check_sum_ pdaf ");
			for(i = 0 ; i < 5 ; i++)
				pr_err("pdaf[%d]=0x%x",i,pPDAFInfo->pdafData1[i]);
		}
			
	}
	else
	{
		pr_err("error pdafFlag=%d", pPDAFInfo->pdafFlag);
		for(i = 0 ; i < 5 ; i++)
			pr_err("pdaf[%d]=0x%x",i,pPDAFInfo->pdafData1[i]);
		
	}
	kfree(pPDAFInfo);

	read_tsp_s5kjn1_eeprom(HWGGC_START_ADDR, (BYTE *)&hwggc, sizeof(hwggc_struct));
	if (hwggc.hwGGCFlag == 0x01)
	{
		otpStatus |= EEPROM_HWGGC_FLAG;
		if (check_sum((BYTE *)&hwggc, sizeof(hwggc_struct)))
		{
			otpStatus |= EEPROM_HWGGC_CHKSUM;
			pr_info("hwggc  chksum pass");
		}
		else
		{
			pr_err(" error  check_sum_ hwggc ");
		}
	}
	else
	{
		pr_err("error pdafFlag=%d", hwggc.hwGGCFlag);
	}

	if (otpStatus == (EEPROM_MODULEINFO_CHKSUM | EEPROM_MODULEINFO_FLAG | EEPROM_MODULEINFO_VALUE |
					  EEPROM_AWB_CHKSUM | EEPROM_AWB_FLAG |
					  EEPROM_AF_CHKSUM | EEPROM_AF_FLAG |
					  EEPROM_LSC_CHKSUM | EEPROM_LSC_FLAG |
					  EEPROM_PDAF_CHKSUM | EEPROM_PDAF_FLAG|
					  EEPROM_HWGGC_CHKSUM| EEPROM_HWGGC_FLAG))
	{
        pr_info("otp check all  ok !!!!  otpStatus=0x%x", otpStatus);
		return true;
	}
	else
	{
		pr_err("otp check fail otpStatus=0x%x", otpStatus);
		return false;
	}
}
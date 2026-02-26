/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5K5E9mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6763
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k5e9mipiraw_Sensor.h"

/*===FEATURE SWITH===*/
 // #define FPTPDAFSUPPORT   //for pdaf switch
 // #define FANPENGTAO   //for debug log

 //#define NONCONTINUEMODE
/*===FEATURE SWITH===*/

/****************************Modify Following Strings for Debug****************************/
#define PFX "S5K5E9"
#define LOG_INF_NEW(format, args...)    pr_err(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF LOG_INF_NEW
#define LOG_1 LOG_INF("S5K5E9,MIPI 4LANE\n")
#define SENSORDB LOG_INF
/****************************   Modify end    *******************************************/

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = TSPPS5F2052_S5K5E9_SENSOR_ID,		//Sensor ID Value: 0x30C8//record sensor id defined in Kd_imgsensor.h

	.checksum_value = 0x52500dc0, //0x49c09f86,		//checksum value for Camera Auto Test

	.pre = {
		.pclk = 190000000,
		.linelength  = 3112,
		.framelength = 2030,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 190000000,
		.linelength  = 3112,
		.framelength = 2030,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 190000000,
		.linelength  = 3112,
		.framelength = 2030,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 190000000,
		.linelength  = 3112,
		.framelength = 544,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 1120,
	},
	.slim_video = {
		.pclk = 190000000,
		.linelength  = 3604,
		.framelength = 1757,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 150000000,
		.max_framerate = 300,
	},

	.margin = 8,			//sensor framelength & shutter margin
	.min_shutter = 5,		//min shutter
	.max_frame_length = 0xFFFF,//REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 10,	  //support sensor mode num ,don't support Slow motion

	.cap_delay_frame = 3,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
  .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
  .mipi_settle_delay_mode = 1, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//SENSOR_OUTPUT_FORMAT_RAW_Gb, SENSOR_OUTPUT_FORMAT_RAW_Gr //sensor output first pixel color
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x20, 0x5a, 0xff}, //record sensor support all write id addr, only supprt 4must end with 0xff
  .i2c_speed = 300, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x200,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x5a,//record current sensor's i2c write id
};


/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 2592, 1944,	 0,  	0, 2592, 1944, 1296,  972, 0,	0, 1296,  972, 0, 0, 1296,  972}, // Preview
 { 2592, 1944,	 0,  	0, 2592, 1944, 2592, 1944, 0,	0, 2592, 1944, 0, 0, 2592, 1944}, // capture
 { 2592, 1944,	 0,  	0, 2592, 1944, 2592, 1944, 0,	0, 2592, 1944, 0, 0, 2592, 1944}, // video
 { 2592, 1944,  16,  12, 2560, 1920,  640,  480, 0,	0,  640,  480, 0, 0,  640,  480}, //hight speed video
 { 2592, 1944, 336, 432, 1920, 1080, 1920, 1080, 0,	0, 1920, 1080, 0, 0, 1920, 1080}, // slim video
};


static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

///////////////////

struct s5k5e9_otp_t s5k5e9_otp_info;

static void s5k5e9_otp_read_init(u8 page)
{
    kal_uint16 get_byte=0;
    kal_uint8 retry_time =3;
	write_cmos_sensor_8(0x0a02, page); //page num
	write_cmos_sensor_8(0x3b41, 0x01);
	write_cmos_sensor_8(0x3b42, 0x03);
	write_cmos_sensor_8(0x3b40, 0x01);
	write_cmos_sensor_8(0x0a00, 0x01); //read cmd
	mdelay(1);//add

    //check read cmd result
    get_byte =read_cmos_sensor(0x0a01);
    do{
        if((get_byte & 0x01)==0x01)
            break;

        mdelay(1);//add
        //check read cmd result
        get_byte =read_cmos_sensor(0x0a01);
        LOG_INF("s5k5e9OTP: read cmd result 0x%x\n",get_byte);
        retry_time--;
    }while(retry_time >0);
}

static void s5k5e9_init(void){

	write_cmos_sensor_8(0x0a00, 0x04);
	write_cmos_sensor_8(0x0a00, 0x00);
}

#if 1
static void s5k5e9_lsc_on(void){

	write_cmos_sensor_8(0x3400, 0x00);
	write_cmos_sensor_8(0x0B00, 0x01);
}
#endif
// static void s5k5e9_lsc_off(void){

// 	write_cmos_sensor_8(0x3400, 0x01);
// 	write_cmos_sensor_8(0x0B00, 0x00);
// }

#if 1
static bool s5k5e9_awb_apply()
{

	kal_uint16 r_ratio, b_ratio;
    kal_uint16 goldenRG,goldenBG,currentRG,currentBG;
    kal_uint16 R_GAIN;
	kal_uint16 B_GAIN;
	kal_uint16 Gr_GAIN;
	kal_uint16 Gb_GAIN;
	kal_uint16 G_GAIN;

	int GAIN_DEFAULT   =    0x0100;

    currentRG =s5k5e9_otp_info.awb_param[0];
    currentRG =(currentRG <<8)|s5k5e9_otp_info.awb_param[1];

    currentBG =s5k5e9_otp_info.awb_param[2];
    currentBG =(currentBG <<8)|s5k5e9_otp_info.awb_param[3];

    goldenRG =s5k5e9_otp_info.awb_param[6];
    goldenRG =(goldenRG <<8)|s5k5e9_otp_info.awb_param[7];

    goldenBG =s5k5e9_otp_info.awb_param[8];
    goldenBG =(goldenBG <<8)|s5k5e9_otp_info.awb_param[9];

	r_ratio = 512 * (goldenRG) /(currentRG);
	b_ratio = 512 * (goldenBG) /(currentBG);

    LOG_INF("s5k5e9OTP: get otp currentRG=0x%x,currentRG=0x%x,goldenRG=0x%x,goldenBG=0x%x \n",currentRG,currentBG,goldenRG,goldenBG);
	if( !r_ratio || !b_ratio)
	{
		LOG_INF("s5k5e9OTP: please get awb data\n");
		return FALSE;
	}

	if(r_ratio >= 512 )
	{
		if(b_ratio>=512)
		{
			R_GAIN = (kal_uint16)(GAIN_DEFAULT * r_ratio / 512);
			G_GAIN = GAIN_DEFAULT;
			B_GAIN = (kal_uint16)(GAIN_DEFAULT * b_ratio / 512);
		}
		else
		{
			R_GAIN =  (kal_uint16)(GAIN_DEFAULT * r_ratio / b_ratio);
			G_GAIN = (kal_uint16)(GAIN_DEFAULT * 512 / b_ratio);
			B_GAIN = GAIN_DEFAULT;
		}
	}
	else
	{
		if(b_ratio >= 512)
		{
			R_GAIN = GAIN_DEFAULT;
			G_GAIN =(kal_uint16)(GAIN_DEFAULT * 512 / r_ratio);
			B_GAIN =(kal_uint16)(GAIN_DEFAULT *  b_ratio / r_ratio);
		}
		else
		{
			Gr_GAIN = (kal_uint16)(GAIN_DEFAULT * 512 / r_ratio );
			Gb_GAIN = (kal_uint16)(GAIN_DEFAULT * 512 / b_ratio );

			if(Gr_GAIN >= Gb_GAIN)
			{
				R_GAIN = GAIN_DEFAULT;
				G_GAIN = (kal_uint16)(GAIN_DEFAULT * 512 / r_ratio );
				B_GAIN = (kal_uint16)(GAIN_DEFAULT * b_ratio / r_ratio);
			}
			else
			{
				R_GAIN =  (kal_uint16)(GAIN_DEFAULT * r_ratio / b_ratio );
				G_GAIN = (kal_uint16)(GAIN_DEFAULT * 512 / b_ratio );
				B_GAIN = GAIN_DEFAULT;
			}
		}
	}

	write_cmos_sensor_8(0x3c0f, 0x00);

	write_cmos_sensor(0x020E, G_GAIN);
	write_cmos_sensor(0x0210, R_GAIN);
	write_cmos_sensor(0x0212, B_GAIN);
	write_cmos_sensor(0x0214, G_GAIN);

	return TRUE;
}
#endif
static u16 s5k5e9_otp_read_group(u16 addr, u8 *data, u16 length)
{
	u16 i = 0;

	for (i = 0; i < length; i++) {
		data[i] = read_cmos_sensor(addr+i);

	//  LOG_INF("s5k5e9OTP: addr = 0x%x, data = 0x%x\n", addr + i, data[i]);

	}

	return 0;
}

static int s5k5e9_iReadData(unsigned int ui4_offset, unsigned int ui4_length, unsigned char *pinputdata,unsigned char page_num)
{
	int i4RetValue = 0;
	int i4ResidueDataLength;
	u32 u4CurrentOffset;
	u8 *pBuff;


	LOG_INF("s5k5e9OTP:ui4_offset = 0x%x, ui4_length = %d ,page_num =%d\n", ui4_offset, ui4_length,page_num);

	i4ResidueDataLength = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
    s5k5e9_otp_read_init(page_num);
	i4RetValue =s5k5e9_otp_read_group((u16) u4CurrentOffset, pBuff, i4ResidueDataLength);
	if (i4RetValue != 0) {
		LOG_INF("s5k5e9OTP: I2C iReadData failed!!\n");
		return -1;
	}
    s5k5e9_init();
	return 0;
}

static bool check_sum(kal_uint8 *buf, unsigned int size)
{
	int i, sum = 0;

	for (i = 0; i < size - 1; i++)
	{
		sum += buf[i];
		LOG_INF("s5k5e9OTP:mdinfor[%d] = 0x%x %d", i, buf[i], buf[i]);
	}

	if ((sum % 256) != buf[size - 1])
	{
		LOG_INF("s5k5e9OTP:chksum fail size = %d sum=0x%x sum-in-eeprom=0x%x", size, (sum % 256), buf[size - 1]);
		return false;
	}
	return true;
}

static bool check_sum_awb(kal_uint8 *buf, unsigned int size, kal_uint8 chksum)
{
	int i, sum = 0;

	for (i = 0; i < size; i++)
	{
		sum += buf[i];
		LOG_INF("s5k5e9OTP:awbData[%d] = 0x%x %d", i, buf[i], buf[i]);
	}

	if ((sum % 256) != chksum)
	{
		LOG_INF("s5k5e9OTP:chksum awb fail size = %d sum=0x%x sum-in-eeprom=0x%x", size, (sum % 256), chksum);
		return false;
	}
	return true;
}

// static bool check_sum_lsc(kal_uint8 *buf, unsigned int size, kal_uint8 chksum)
// {
// 	int i, sum = 0;

// 	for (i = 0; i < size; i++)
// 	{
// 		sum += buf[i];
//         // LOG_INF("s5k5e9OTP:lscData[%d] = 0x%x %d", i, buf[i], buf[i]);
// 	}

// 	if ((sum % 256) != chksum)
// 	{
// 		LOG_INF("chksum lsc fail size = %d sum=0x%d sum-in-eeprom=0x%x", size, (sum % 256), chksum);
// 		return false;
// 	}
// 	return true;
// }


bool check_s5k5e9_otp(void)
{

	kal_uint8 moduleInoramtionFlag =0;
	kal_uint8 awbFlag =0;
	// kal_uint8 lscFlag =0;

	moduleInfo_struct_t *pModuleInfo;
	unsigned int otpStatus = 0;

    memset(&s5k5e9_otp_info, 0, sizeof(struct s5k5e9_otp_t));

    //module info
    pModuleInfo = kmalloc(sizeof(moduleInfo_struct_t), GFP_KERNEL);

	s5k5e9_iReadData(S5K5E9_MODULE_INFO_FLAG, 1, &moduleInoramtionFlag,S5K5E9_MDINFO_PAGE);
	if ((moduleInoramtionFlag & 0x03) == 0x01)
	{
		LOG_INF("s5k5e9OTP:module info group1 valid");
		otpStatus |= S5K5E9_MODULEINFO_FLAG;
                s5k5e9_iReadData(S5K5E9_MODULE_INFO_FLAG + 1, sizeof(moduleInfo_struct_t), (unsigned char *)pModuleInfo,S5K5E9_MDINFO_PAGE);

	}
	else if((moduleInoramtionFlag & 0x0C) == 0x04)
	{
		LOG_INF("s5k5e9OTP:module info group2 valid");
		otpStatus |= S5K5E9_MODULEINFO_FLAG;
                s5k5e9_iReadData(S5K5E9_MODULE_INFO_FLAG + (S5K5E9_MODULE_LENGTH), sizeof(moduleInfo_struct_t), (unsigned char *)pModuleInfo, S5K5E9_MDINFO_PAGE);
	}
	else
	{
		LOG_INF("s5k5e9OTP:moduleInoramtionFlag=%d", moduleInoramtionFlag);
	}

	if(otpStatus == S5K5E9_MODULEINFO_FLAG)
	{
		if (check_sum((kal_uint8 *)pModuleInfo, sizeof(moduleInfo_struct_t)))
		{
			otpStatus |= S5K5E9_MODULEINFO_CHKSUM;
			if (pModuleInfo->moduleID == 0x48 && pModuleInfo->LENSID == 0xE7 && pModuleInfo->VCMID == 0x0 && pModuleInfo->DriverICID == 0x0)
			{
				otpStatus |= S5K5E9_MODULEINFO_VALUE;
				LOG_INF("s5k5e9OTP:module info flag chksum value pass");
			}
			else
			{
				LOG_INF("s5k5e9OTP:moduleID =0x%x,LENSID=0x%x,VCMID=0x%x,DriverICID=0x%x",
					   pModuleInfo->moduleID, pModuleInfo->LENSID, pModuleInfo->VCMID, pModuleInfo->DriverICID);
			}
		}
	}
	kfree(pModuleInfo);

    //awb info
	s5k5e9_iReadData(S5K5E9_AWB_INFO_FLAG, 1, &awbFlag, S5K5E9_AWB_PAGE);
	if ((awbFlag  & 0x03) == 0x01)
	{
		LOG_INF("s5k5e9OTP:awb info group1 valid");

                s5k5e9_otp_info.awb_flag = 0x01;
                //awb data
                s5k5e9_iReadData(S5K5E9_AWB_INFO_FLAG + 1, S5K5E9_AWB_LENGTH, &s5k5e9_otp_info.awb_param[0], S5K5E9_AWB_PAGE);
                //awb checksum
                s5k5e9_iReadData(S5K5E9_AWB_INFO_FLAG + S5K5E9_AWB_LENGTH, 1, &s5k5e9_otp_info.awbChksum, S5K5E9_AWB_PAGE);
	}
	else if ((awbFlag  & 0x0C) == 0x04)
	{
		LOG_INF("s5k5e9OTP:awb info group2 valid");

                s5k5e9_otp_info.awb_flag = 0x04;
                s5k5e9_iReadData(S5K5E9_AWB_INFO_FLAG + (S5K5E9_AWB_LENGTH), S5K5E9_AWB_LENGTH, &s5k5e9_otp_info.awb_param[0], S5K5E9_AWB_PAGE);
                s5k5e9_iReadData(S5K5E9_AWB_INFO_FLAG + (S5K5E9_AWB_LENGTH) + S5K5E9_AWB_LENGTH, 1, &s5k5e9_otp_info.awbChksum, S5K5E9_AWB_PAGE);
	}
	else
	{
		LOG_INF("s5k5e9OTP awbFlag=%d", awbFlag);
	}

	if ((s5k5e9_otp_info.awb_flag == 0x01) || (s5k5e9_otp_info.awb_flag == 0x04))
	{
		otpStatus |= S5K5E9_AWB_FLAG;

		if (check_sum_awb(&s5k5e9_otp_info.awb_param[0], S5K5E9_AWB_LENGTH-1, s5k5e9_otp_info.awbChksum))
		{
			otpStatus |= S5K5E9_AWB_CHKSUM;
			LOG_INF("s5k5e9OTP:awb flag chksum pass");
		}
		else
		{
			int i;
			for (i = 0; i < S5K5E9_AWB_LENGTH-1; i++)
			     LOG_INF("s5k5e9OTP:awb[%d]=0x%x  %d\n", i, s5k5e9_otp_info.awb_param[i], s5k5e9_otp_info.awb_param[i]);
		}
	}

#if 0
    //LSC info
	s5k5e9_iReadData(S5K5E9_LSC_INFO_FLAG, 1, &lscFlag, S5K5E9_LSC_PAGE);
	if ((lscFlag  & 0x03) == 0x01)
	{
		LOG_INF("s5k5e9OTP:lsc info group1 valid");

                s5k5e9_otp_info.lsc_flag = 0x01;
                s5k5e9_iReadData(S5K5E9_LSC_INFO_START, S5K5E9_LSC_LENGTH, &s5k5e9_otp_info.lsc_param[0], S5K5E9_LSC_PAGE+1);
                s5k5e9_iReadData(S5K5E9_LSC_INFO_START + S5K5E9_LSC_LENGTH, 1, &s5k5e9_otp_info.lscChksum, S5K5E9_LSC_PAGE+17);
	}
	else if ((lscFlag  & 0x0C) == 0x04)
	{
		LOG_INF("s5k5e9OTP:lsc info group2 valid");

                s5k5e9_otp_info.lsc_flag = 0x04;
                s5k5e9_iReadData(S5K5E9_LSC_INFO_START + (S5K5E9_LSC_LENGTH), S5K5E9_LSC_LENGTH-1, &s5k5e9_otp_info.lsc_param[0], S5K5E9_LSC_PAGE+6);
                s5k5e9_iReadData(S5K5E9_LSC_INFO_START + (S5K5E9_LSC_LENGTH-1)+ S5K5E9_LSC_LENGTH, 1, &s5k5e9_otp_info.lscChksum, S5K5E9_LSC_PAGE+17);
	}
	else
	{
		LOG_INF("s5k5e9OTP:lscFlag=%d", lscFlag);
	}

	if ((s5k5e9_otp_info.lsc_flag == 0x01) || (s5k5e9_otp_info.lsc_flag == 0x04))
	{
		otpStatus |= S5K5E9_LSC_FLAG;

		if (check_sum_lsc(&s5k5e9_otp_info.lsc_param[0], S5K5E9_LSC_LENGTH-1, s5k5e9_otp_info.lscChksum))
		{
			otpStatus |= S5K5E9_LSC_CHKSUM;
			LOG_INF("s5k5e9OTP:lsc flag chksum pass");
		}
	}
#endif
    // s5k5e9_init();

	if (otpStatus == (S5K5E9_MODULEINFO_CHKSUM | S5K5E9_MODULEINFO_FLAG | S5K5E9_MODULEINFO_VALUE |
					  S5K5E9_AWB_CHKSUM | S5K5E9_AWB_FLAG
					  // |S5K5E9_LSC_CHKSUM | S5K5E9_LSC_FLAG
                      ))
	{
        LOG_INF("s5k5e9OTP:otp check all  ok !!!!  otpStatus=0x%x", otpStatus);
		return true;
	}
	else
	{
		LOG_INF("s5k5e9OTP:otp check fail otpStatus=0x%x", otpStatus);
		return false;
	}
}
/////////////////
static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable(%d) \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, shutter & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
    reg_gain = gain/2;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	LOG_INF("set_gain %d \n", gain);
  //gain = 64 = 1x real gain.

	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

    reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));
	return gain;
}	/*	set_gain  */


static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror;
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor_8(0x0101, 0x00); //GR
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor_8(0x0101, 0x01); //R
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor_8(0x0101, 0x02); //B
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor_8(0x0101, 0x03); //GB
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
static void sensor_init(void)
{
  LOG_INF("E\n");
  write_cmos_sensor_8(0x0100, 0x00);
  write_cmos_sensor_8(0x3B45, 0x01);
  write_cmos_sensor_8(0x0B05, 0x01);
  write_cmos_sensor_8(0x392F, 0x01);
  write_cmos_sensor_8(0x3930, 0x00);
  write_cmos_sensor_8(0x3924, 0x7F);
  write_cmos_sensor_8(0x3925, 0xFD);
  write_cmos_sensor_8(0x3C08, 0xFF);
  write_cmos_sensor_8(0x3C09, 0xFF);
  write_cmos_sensor_8(0x3C31, 0xFF);
  write_cmos_sensor_8(0x3C32, 0xFF);
  write_cmos_sensor_8(0x3290, 0x10);
  write_cmos_sensor_8(0x3200, 0x01);
  write_cmos_sensor_8(0x3074, 0x06);
  write_cmos_sensor_8(0x3075, 0x2F);
  write_cmos_sensor_8(0x308A, 0x20);
  write_cmos_sensor_8(0x308B, 0x08);
  write_cmos_sensor_8(0x308C, 0x0B);
  write_cmos_sensor_8(0x3081, 0x07);
  write_cmos_sensor_8(0x307B, 0x85);
  write_cmos_sensor_8(0x307A, 0x0A);
  write_cmos_sensor_8(0x3079, 0x0A);
  write_cmos_sensor_8(0x306E, 0x71);
  write_cmos_sensor_8(0x306F, 0x28);
  write_cmos_sensor_8(0x301F, 0x20);
  write_cmos_sensor_8(0x3012, 0x4E);
  write_cmos_sensor_8(0x306B, 0x9A);
  write_cmos_sensor_8(0x3091, 0x16);
  write_cmos_sensor_8(0x30C4, 0x06);
  write_cmos_sensor_8(0x306A, 0x79);
  write_cmos_sensor_8(0x30B0, 0xFF);
  write_cmos_sensor_8(0x306D, 0x08);
  write_cmos_sensor_8(0x3084, 0x16);
  write_cmos_sensor_8(0x3070, 0x0F);
  write_cmos_sensor_8(0x30C2, 0x05);
  write_cmos_sensor_8(0x3069, 0x87);
  write_cmos_sensor_8(0x3C0F, 0x00);
  write_cmos_sensor_8(0x0A02, 0x3F);
  write_cmos_sensor_8(0x3083, 0x14);
  write_cmos_sensor_8(0x3080, 0x08);
  write_cmos_sensor_8(0x3C34, 0xEA);
  write_cmos_sensor_8(0x3C35, 0x5C);
}	/*	sensor_init  */


static void preview_setting(void)
{
  LOG_INF("E\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor_8(0x0136, 0x18);
  write_cmos_sensor_8(0x0137, 0x00);
  write_cmos_sensor_8(0x0305, 0x04);
  write_cmos_sensor_8(0x0306, 0x00);
  write_cmos_sensor_8(0x0307, 0x5F);
  write_cmos_sensor_8(0x030D, 0x04);
  write_cmos_sensor_8(0x030E, 0x00);
  write_cmos_sensor_8(0x030F, 0x92);
  write_cmos_sensor_8(0x3C1F, 0x00);
  write_cmos_sensor_8(0x3C17, 0x00);
  write_cmos_sensor_8(0x0112, 0x0A);
  write_cmos_sensor_8(0x0113, 0x0A);
  write_cmos_sensor_8(0x0114, 0x01);
  write_cmos_sensor_8(0x0820, 0x03);
  write_cmos_sensor_8(0x0821, 0x6C);
  write_cmos_sensor_8(0x0822, 0x00);
  write_cmos_sensor_8(0x0823, 0x00);
  write_cmos_sensor_8(0x3929, 0x0F);
  write_cmos_sensor_8(0x0344, 0x00);
  write_cmos_sensor_8(0x0345, 0x08);
  write_cmos_sensor_8(0x0346, 0x00);
  write_cmos_sensor_8(0x0347, 0x08);
  write_cmos_sensor_8(0x0348, 0x0A);
  write_cmos_sensor_8(0x0349, 0x27);
  write_cmos_sensor_8(0x034A, 0x07);
  write_cmos_sensor_8(0x034B, 0x9F);
  write_cmos_sensor_8(0x034C, 0x05);
  write_cmos_sensor_8(0x034D, 0x10);
  write_cmos_sensor_8(0x034E, 0x03);
  write_cmos_sensor_8(0x034F, 0xCC);
  write_cmos_sensor_8(0x0900, 0x01);
  write_cmos_sensor_8(0x0901, 0x22);
  write_cmos_sensor_8(0x0381, 0x01);
  write_cmos_sensor_8(0x0383, 0x01);
  write_cmos_sensor_8(0x0385, 0x01);
  write_cmos_sensor_8(0x0387, 0x03);
  write_cmos_sensor_8(0x0101, 0x03);
  write_cmos_sensor_8(0x0340, 0x07);
  write_cmos_sensor_8(0x0341, 0xEE);
  write_cmos_sensor_8(0x0342, 0x0C);
  write_cmos_sensor_8(0x0343, 0x28);
  write_cmos_sensor_8(0x0200, 0x0B);
  write_cmos_sensor_8(0x0201, 0x9C);
  write_cmos_sensor_8(0x0202, 0x00);
  write_cmos_sensor_8(0x0203, 0x02);
  write_cmos_sensor_8(0x30B8, 0x2A);
  write_cmos_sensor_8(0x30BA, 0x2E);
  write_cmos_sensor_8(0x0100, 0x01);
}	/*	preview_setting  */


static void capture_setting(void)
{
	LOG_INF("E! currefps:%d\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor_8(0x0136, 0x18);
  write_cmos_sensor_8(0x0137, 0x00);
  write_cmos_sensor_8(0x0305, 0x04);
  write_cmos_sensor_8(0x0306, 0x00);
  write_cmos_sensor_8(0x0307, 0x5F);
  write_cmos_sensor_8(0x030D, 0x04);
  write_cmos_sensor_8(0x030E, 0x00);
  write_cmos_sensor_8(0x030F, 0x92);
  write_cmos_sensor_8(0x3C1F, 0x00);
  write_cmos_sensor_8(0x3C17, 0x00);
  write_cmos_sensor_8(0x0112, 0x0A);
  write_cmos_sensor_8(0x0113, 0x0A);
  write_cmos_sensor_8(0x0114, 0x01);
  write_cmos_sensor_8(0x0820, 0x03);
  write_cmos_sensor_8(0x0821, 0x6C);
  write_cmos_sensor_8(0x0822, 0x00);
  write_cmos_sensor_8(0x0823, 0x00);
  write_cmos_sensor_8(0x3929, 0x0F);
  write_cmos_sensor_8(0x0344, 0x00);
  write_cmos_sensor_8(0x0345, 0x08);
  write_cmos_sensor_8(0x0346, 0x00);
  write_cmos_sensor_8(0x0347, 0x08);
  write_cmos_sensor_8(0x0348, 0x0A);
  write_cmos_sensor_8(0x0349, 0x27);
  write_cmos_sensor_8(0x034A, 0x07);
  write_cmos_sensor_8(0x034B, 0x9f);
  write_cmos_sensor_8(0x034C, 0x0A);
  write_cmos_sensor_8(0x034D, 0x20);
  write_cmos_sensor_8(0x034E, 0x07);
  write_cmos_sensor_8(0x034F, 0x98);
  write_cmos_sensor_8(0x0900, 0x00);
  write_cmos_sensor_8(0x0901, 0x00);
  write_cmos_sensor_8(0x0381, 0x01);
  write_cmos_sensor_8(0x0383, 0x01);
  write_cmos_sensor_8(0x0385, 0x01);
  write_cmos_sensor_8(0x0387, 0x01);
  write_cmos_sensor_8(0x0101, 0x03);
  write_cmos_sensor_8(0x0340, 0x07);
  write_cmos_sensor_8(0x0341, 0xEE);
  write_cmos_sensor_8(0x0342, 0x0C);
  write_cmos_sensor_8(0x0343, 0x28);
  write_cmos_sensor_8(0x0200, 0x0B);
  write_cmos_sensor_8(0x0201, 0x9C);
  write_cmos_sensor_8(0x0202, 0x00);
  write_cmos_sensor_8(0x0203, 0x02);
  write_cmos_sensor_8(0x30B8, 0x2E);
  write_cmos_sensor_8(0x30BA, 0x36);
  write_cmos_sensor_8(0x0100, 0x01);
}


static void normal_video_setting(void)
{
	LOG_INF("E! currefps:%d\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor_8(0x0136, 0x18);
  write_cmos_sensor_8(0x0137, 0x00);
  write_cmos_sensor_8(0x0305, 0x04);
  write_cmos_sensor_8(0x0306, 0x00);
  write_cmos_sensor_8(0x0307, 0x5F);
  write_cmos_sensor_8(0x030D, 0x04);
  write_cmos_sensor_8(0x030E, 0x00);
  write_cmos_sensor_8(0x030F, 0x92);
  write_cmos_sensor_8(0x3C1F, 0x00);
  write_cmos_sensor_8(0x3C17, 0x00);
  write_cmos_sensor_8(0x0112, 0x0A);
  write_cmos_sensor_8(0x0113, 0x0A);
  write_cmos_sensor_8(0x0114, 0x01);
  write_cmos_sensor_8(0x0820, 0x03);
  write_cmos_sensor_8(0x0821, 0x6C);
  write_cmos_sensor_8(0x0822, 0x00);
  write_cmos_sensor_8(0x0823, 0x00);
  write_cmos_sensor_8(0x3929, 0x0F);
  write_cmos_sensor_8(0x0344, 0x00);
  write_cmos_sensor_8(0x0345, 0x08);
  write_cmos_sensor_8(0x0346, 0x00);
  write_cmos_sensor_8(0x0347, 0x08);
  write_cmos_sensor_8(0x0348, 0x0A);
  write_cmos_sensor_8(0x0349, 0x27);
  write_cmos_sensor_8(0x034A, 0x07);
  write_cmos_sensor_8(0x034B, 0x9f);
  write_cmos_sensor_8(0x034C, 0x0A);
  write_cmos_sensor_8(0x034D, 0x20);
  write_cmos_sensor_8(0x034E, 0x07);
  write_cmos_sensor_8(0x034F, 0x98);
  write_cmos_sensor_8(0x0900, 0x00);
  write_cmos_sensor_8(0x0901, 0x00);
  write_cmos_sensor_8(0x0381, 0x01);
  write_cmos_sensor_8(0x0383, 0x01);
  write_cmos_sensor_8(0x0385, 0x01);
  write_cmos_sensor_8(0x0387, 0x01);
  write_cmos_sensor_8(0x0101, 0x03);
  write_cmos_sensor_8(0x0340, 0x07);
  write_cmos_sensor_8(0x0341, 0xEE);
  write_cmos_sensor_8(0x0342, 0x0C);
  write_cmos_sensor_8(0x0343, 0x28);
  write_cmos_sensor_8(0x0200, 0x0B);
  write_cmos_sensor_8(0x0201, 0x9C);
  write_cmos_sensor_8(0x0202, 0x00);
  write_cmos_sensor_8(0x0203, 0x02);
  write_cmos_sensor_8(0x30B8, 0x2E);
  write_cmos_sensor_8(0x30BA, 0x36);
  write_cmos_sensor_8(0x0100, 0x01);
}


static void hs_video_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor_8(0x0136, 0x18);
  write_cmos_sensor_8(0x0137, 0x00);
  write_cmos_sensor_8(0x0305, 0x04);
  write_cmos_sensor_8(0x0306, 0x00);
  write_cmos_sensor_8(0x0307, 0x5F);
  write_cmos_sensor_8(0x030D, 0x04);
  write_cmos_sensor_8(0x030E, 0x00);
  write_cmos_sensor_8(0x030F, 0x92);
  write_cmos_sensor_8(0x3C1F, 0x00);
  write_cmos_sensor_8(0x3C17, 0x00);
  write_cmos_sensor_8(0x0112, 0x0A);
  write_cmos_sensor_8(0x0113, 0x0A);
  write_cmos_sensor_8(0x0114, 0x01);
  write_cmos_sensor_8(0x0820, 0x03);
  write_cmos_sensor_8(0x0821, 0x6C);
  write_cmos_sensor_8(0x0822, 0x00);
  write_cmos_sensor_8(0x0823, 0x00);
  write_cmos_sensor_8(0x3929, 0x0F);
  write_cmos_sensor_8(0x0344, 0x00);
  write_cmos_sensor_8(0x0345, 0x18);
  write_cmos_sensor_8(0x0346, 0x00);
  write_cmos_sensor_8(0x0347, 0x14);
  write_cmos_sensor_8(0x0348, 0x0A);
  write_cmos_sensor_8(0x0349, 0x17);
  write_cmos_sensor_8(0x034A, 0x07);
  write_cmos_sensor_8(0x034B, 0x93);
  write_cmos_sensor_8(0x034C, 0x02);
  write_cmos_sensor_8(0x034D, 0x80);
  write_cmos_sensor_8(0x034E, 0x01);
  write_cmos_sensor_8(0x034F, 0xE0);
  write_cmos_sensor_8(0x0900, 0x01);
  write_cmos_sensor_8(0x0901, 0x44);
  write_cmos_sensor_8(0x0381, 0x01);
  write_cmos_sensor_8(0x0383, 0x01);
  write_cmos_sensor_8(0x0385, 0x01);
  write_cmos_sensor_8(0x0387, 0x07);
  write_cmos_sensor_8(0x0101, 0x03);
  write_cmos_sensor_8(0x0340, 0x02);
  write_cmos_sensor_8(0x0341, 0x20);
  write_cmos_sensor_8(0x0342, 0x0C);
  write_cmos_sensor_8(0x0343, 0x28);
  write_cmos_sensor_8(0x0200, 0x0B);
  write_cmos_sensor_8(0x0201, 0x9C);
  write_cmos_sensor_8(0x0202, 0x00);
  write_cmos_sensor_8(0x0203, 0x02);
  write_cmos_sensor_8(0x30B8, 0x2E);
  write_cmos_sensor_8(0x30BA, 0x36);
  write_cmos_sensor_8(0x0100, 0x01);
}


static void slim_video_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor_8(0x0100, 0x00);
  mdelay(33);
  write_cmos_sensor_8(0x0136, 0x18);
  write_cmos_sensor_8(0x0137, 0x00);
  write_cmos_sensor_8(0x0305, 0x04);
  write_cmos_sensor_8(0x0306, 0x00);
  write_cmos_sensor_8(0x0307, 0x5F);
  write_cmos_sensor_8(0x030D, 0x04);
  write_cmos_sensor_8(0x030E, 0x00);
  write_cmos_sensor_8(0x030F, 0x7D);
  write_cmos_sensor_8(0x3C1F, 0x00);
  write_cmos_sensor_8(0x3C17, 0x00);
  write_cmos_sensor_8(0x0112, 0x0A);
  write_cmos_sensor_8(0x0113, 0x0A);
  write_cmos_sensor_8(0x0114, 0x01);
  write_cmos_sensor_8(0x0820, 0x02);
  write_cmos_sensor_8(0x0821, 0xEE);
  write_cmos_sensor_8(0x0822, 0x00);
  write_cmos_sensor_8(0x0823, 0x00);
  write_cmos_sensor_8(0x3929, 0x0F);
  write_cmos_sensor_8(0x0344, 0x01);
  write_cmos_sensor_8(0x0345, 0x58);
  write_cmos_sensor_8(0x0346, 0x01);
  write_cmos_sensor_8(0x0347, 0xB8);
  write_cmos_sensor_8(0x0348, 0x08);
  write_cmos_sensor_8(0x0349, 0xD7);
  write_cmos_sensor_8(0x034A, 0x05);
  write_cmos_sensor_8(0x034B, 0xEF);
  write_cmos_sensor_8(0x034C, 0x07);
  write_cmos_sensor_8(0x034D, 0x80);
  write_cmos_sensor_8(0x034E, 0x04);
  write_cmos_sensor_8(0x034F, 0x38);
  write_cmos_sensor_8(0x0900, 0x00);
  write_cmos_sensor_8(0x0901, 0x00);
  write_cmos_sensor_8(0x0381, 0x01);
  write_cmos_sensor_8(0x0383, 0x01);
  write_cmos_sensor_8(0x0385, 0x01);
  write_cmos_sensor_8(0x0387, 0x01);
  write_cmos_sensor_8(0x0101, 0x03);
  write_cmos_sensor_8(0x0340, 0x06);
  write_cmos_sensor_8(0x0341, 0xDD);
  write_cmos_sensor_8(0x0342, 0x0E);
  write_cmos_sensor_8(0x0343, 0x14);
  write_cmos_sensor_8(0x0200, 0x0D);
  write_cmos_sensor_8(0x0201, 0x90);
  write_cmos_sensor_8(0x0202, 0x00);
  write_cmos_sensor_8(0x0203, 0x02);
  write_cmos_sensor_8(0x30B8, 0x2E);
  write_cmos_sensor_8(0x30BA, 0x36);
  write_cmos_sensor_8(0x0100, 0x01);
}


static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor_byte(0x0000) << 8) | read_cmos_sensor_byte(0x0001));
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);
                if(check_s5k5e9_otp())
                    *sensor_id |= 0x01000000;
                return ERROR_NONE;
            }
			LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
	LOG_1;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();
    s5k5e9_lsc_on();
    s5k5e9_awb_apply();
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorCustom2Width = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorCustom3Width = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorCustom3Height = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorCustom4Width = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorCustom4Height = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorCustom5Width = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorCustom5Height = imgsensor_info.cap.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.cap_delay_frame;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;
	#ifdef FPTPDAFSUPPORT
	sensor_info->PDAF_Support = 1;
	#else
	sensor_info->PDAF_Support = 0;
	#endif

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_CUSTOM5:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;

		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_CUSTOM5:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_CUSTOM5:
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_CUSTOM5:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
    write_cmos_sensor(0x0600, 0x0002);
	} else {
    write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
//	struct SET_PD_BLOCK_INFO_T *PDAFinfo;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_CUSTOM1:
                case MSDK_SCENARIO_ID_CUSTOM2:
                case MSDK_SCENARIO_ID_CUSTOM3:
                case MSDK_SCENARIO_ID_CUSTOM4:
                case MSDK_SCENARIO_ID_CUSTOM5:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
			break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            //ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        /******************** PDAF START >>> *********/
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CUSTOM1:
                case MSDK_SCENARIO_ID_CUSTOM2:
                case MSDK_SCENARIO_ID_CUSTOM3:
                case MSDK_SCENARIO_ID_CUSTOM4:
                case MSDK_SCENARIO_ID_CUSTOM5:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
/*
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					break;
			}
			break;
*/
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//S5K5E9_read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;
        /******************** PDAF END   <<< *********/
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close,
};


UINT32 S5K5E9_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	S5K5E9_MIPI_RAW_SensorInit	*/




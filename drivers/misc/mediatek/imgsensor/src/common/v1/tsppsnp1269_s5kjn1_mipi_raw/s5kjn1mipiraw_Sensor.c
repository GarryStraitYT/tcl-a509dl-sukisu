/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5KJN1mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6873
 *
 * Description:
 * ------------
 *---------------------------------------------------------------------------
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
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5kjn1mipiraw_Sensor.h"

#define MULTI_WRITE 1
#if     MULTI_WRITE
#define I2C_BUFFER_LEN 1020 /* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 4
#endif


/****************************Modify Following Strings for Debug****************************/
#define PFX "S5KJN1"

#define LOG_INF(format, args...)                                               \
	pr_debug(PFX "[JW][%s] " format, __func__, ##args)

#define LOG_1 LOG_INF("S5KJN1,MIPI 4LANE\n")

/****************************   Modify end    *******************************************/

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = TSPPSNP1269_S5KJN1_SENSOR_ID,
	.checksum_value =  0x40631970,
	.pre = {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	.cap = {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	.normal_video =  {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	.hs_video =  {
		.pclk = 600000000,
		.linelength = 2096,
		.framelength = 2384,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 640000000,
	},
	.slim_video =  {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 480000000,
	},

	.margin = 5,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter

	.min_gain = 64, 		/*1x gain*/
	.max_gain = 1024, 		/*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 2,
	.exp_step = 2,
	.gain_type = 2,
	.max_frame_length = 0xFFFF,		//REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,		//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,	//isp gain delay frame for AE cycle
	.ihdr_support = 0,				//1, support; 0,not support
	.ihdr_le_firstline = 0,			//1,le first ; 0, se first
	.temperature_support = 0,		/* 1, support; 0,not support */
	.sensor_mode_num = 5,			//support sensor mode num ,don't support Slow motion

	.cap_delay_frame = 2,			//enter capture delay frame num
	.pre_delay_frame = 2,			//enter preview delay frame num
	.video_delay_frame = 2,			//enter video delay frame num
	.hs_video_delay_frame = 2,		//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,	//enter slim video delay frame num
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,
	.custom5_delay_frame = 2,
	.frame_time_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA, 				//mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	//sensor_interface_type
	.mipi_sensor_type = MIPI_OPHY_NCSI2, 					//0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 0,							//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,	//SENSOR_OUTPUT_FORMAT_RAW_Gb,//sensor output first pixel color
	.mclk = 24,												//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,					//mipi lane num
	.i2c_speed = 1000,
	.i2c_addr_table = {0x7a,0x20,0x5a,0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode =IMGSENSOR_MODE_INIT,	//IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x200,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps =0,					//full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en =KAL_FALSE,			//auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern =KAL_FALSE,			//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id =MSDK_SCENARIO_ID_CAMERA_PREVIEW, //current scenario id
	.ihdr_en = KAL_FALSE,				//sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,				//record current sensor's i2c write id

	//Long exposure
	.current_ae_effective_frame = 1,
};

/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 4080, 3072, 0, 0, 4080, 3072, 4080, 3072,   0,   0, 4080, 3072, 0, 0,4080, 3072 }, // preveiw
	{ 4080, 3072, 0, 0, 4080, 3072, 4080, 3072,   0,   0, 4080, 3072, 0, 0,4080, 3072 }, // capture
	{ 4080, 3072, 0, 0, 4080, 3072, 4080, 3072,   0,   0, 4080, 3072, 0, 0,4080, 3072 }, // video
	{ 4080, 3072, 0, 0, 4080, 3072, 2040, 1536, 380, 408, 1280,  720, 0, 0,1280,  720 }, // high speed
	{ 4080, 3072, 0, 0, 4080, 3072, 4080, 3072,   0, 388, 4080, 2296, 0, 0,4080, 2296 }, // slim video
};

#if MULTI_WRITE
static kal_uint16 sensor_init_setting_array[] = {
0x6028,	0x2400,
0x602A,	0x1354,
0x6F12,	0x0100,
0x6F12,	0x7017,
0x602A,	0x13B2,
0x6F12,	0x0000,
0x602A,	0x1236,
0x6F12,	0x0000,
0x602A,	0x1A0A,
0x6F12,	0x4C0A,
0x602A,	0x2210,
0x6F12,	0x3401,
0x602A,	0x2176,
0x6F12,	0x6400,
0x602A,	0x222E,
0x6F12,	0x0001,
0x602A,	0x06B6,
0x6F12,	0x0A00,
0x602A,	0x06BC,
0x6F12,	0x1001,
0x602A,	0x2140,
0x6F12,	0x0101,
0x602A,	0x1A0E,
0x6F12,	0x9600,
0x6028,	0x4000,
0xF44E,	0x0011,
0xF44C,	0x0B0B,
0xF44A,	0x0006,
0x0118,	0x0002,
0x011A,	0x0001,
};

//FDSUM+A2A2 2040 x 1536 30fps mipi dphy 1632Mbps LLP 2848 FLL 7008 Tail size x 508 y 764 offset x 4 y 4
static kal_uint16 preview_setting_array[] = {
0x6028	,0x2400,
0x602A	,0x1A28,
0x6F12	,0x4C00,
0x602A	,0x065A,
0x6F12	,0x0000,
0x602A	,0x139E,
0x6F12	,0x0100,
0x602A	,0x139C,
0x6F12	,0x0000,
0x602A	,0x13A0,
0x6F12	,0x0A00,
0x6F12	,0x0120,
0x602A	,0x2072,
0x6F12	,0x0000,
0x602A	,0x1A64,
0x6F12	,0x0301,
0x6F12	,0xFF00,
0x602A	,0x19E6,
0x6F12	,0x0200,
0x602A	,0x1A30,
0x6F12	,0x3401,
0x602A	,0x19FC,
0x6F12	,0x0B00,
0x602A	,0x19F4,
0x6F12	,0x0606,
0x602A	,0x19F8,
0x6F12	,0x1010,
0x602A	,0x1B26,
0x6F12	,0x6F80,
0x6F12	,0xA060,
0x602A	,0x1A3C,
0x6F12	,0x6207,
0x602A	,0x1A48,
0x6F12	,0x6207,
0x602A	,0x1444,
0x6F12	,0x2000,
0x6F12	,0x2000,
0x602A	,0x144C,
0x6F12	,0x3F00,
0x6F12	,0x3F00,
0x602A	,0x7F6C,
0x6F12	,0x0100,
0x6F12	,0x2F00,
0x6F12	,0xFA00,
0x6F12	,0x2400,
0x6F12	,0xE500,
0x602A	,0x0650,
0x6F12	,0x0600,
0x602A	,0x0654,
0x6F12	,0x0000,
0x602A	,0x1A46,
0x6F12	,0x8A00,
0x602A	,0x1A52,
0x6F12	,0xBF00,
0x602A	,0x0674,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x602A	,0x0668,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x602A	,0x0684,
0x6F12	,0x4001,
0x602A	,0x0688,
0x6F12	,0x4001,
0x602A	,0x147C,
0x6F12	,0x1000,
0x602A	,0x1480,
0x6F12	,0x1000,
0x602A	,0x19F6,
0x6F12	,0x0904,
0x602A	,0x0812,
0x6F12	,0x0000,
0x602A	,0x1A02,
0x6F12	,0x1800,
0x602A	,0x2148,
0x6F12	,0x0100,
0x602A	,0x2042,
0x6F12	,0x1A00,
0x602A	,0x0874,
0x6F12	,0x0100,
0x602A	,0x09C0,
0x6F12	,0x2008,
0x602A	,0x09C4,
0x6F12	,0x2000,
0x602A	,0x19FE,
0x6F12	,0x0E1C,
0x602A	,0x4D92,
0x6F12	,0x0100,
0x602A	,0x84C8,
0x6F12	,0x0100,
0x602A	,0x4D94,
0x6F12	,0x0005,
0x6F12	,0x000A,
0x6F12	,0x0010,
0x6F12	,0x0810,
0x6F12	,0x000A,
0x6F12	,0x0040,
0x6F12	,0x0810,
0x6F12	,0x0810,
0x6F12	,0x8002,
0x6F12	,0xFD03,
0x6F12	,0x0010,
0x6F12	,0x1510,
0x602A	,0x3570,
0x6F12	,0x0000,
0x602A	,0x3574,
0x6F12	,0x1201,
0x602A	,0x21E4,
0x6F12	,0x0400,
0x602A	,0x21EC,
0x6F12	,0x1F04,
0x602A	,0x2080,
0x6F12	,0x0100,
0x6F12	,0xFF00,
0x602A	,0x2086,
0x6F12	,0x0001,
0x602A	,0x208E,
0x6F12	,0x14F4,
0x602A	,0x208A,
0x6F12	,0xD244,
0x6F12	,0xD244,
0x602A	,0x120E,
0x6F12	,0x1000,
0x602A	,0x212E,
0x6F12	,0x0200,
0x602A	,0x13AE,
0x6F12	,0x0101,
0x602A	,0x0718,
0x6F12	,0x0001,
0x602A	,0x0710,
0x6F12	,0x0002,
0x6F12	,0x0804,
0x6F12	,0x0100,
0x602A	,0x1B5C,
0x6F12	,0x0000,
0x602A	,0x0786,
0x6F12	,0x7701,
0x602A	,0x2022,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x602A	,0x1360,
0x6F12	,0x0100,
0x602A	,0x1376,
0x6F12	,0x0100,
0x6F12	,0x6038,
0x6F12	,0x7038,
0x6F12	,0x8038,
0x602A	,0x1386,
0x6F12	,0x0B00,
0x602A	,0x06FA,
0x6F12	,0x1000,
0x602A	,0x4A94,
0x6F12	,0x0900,
0x6F12	,0x0000,
0x6F12	,0x0300,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0300,
0x6F12	,0x0000,
0x6F12	,0x0900,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x602A	,0x0A76,
0x6F12	,0x1000,
0x602A	,0x0AEE,
0x6F12	,0x1000,
0x602A	,0x0B66,
0x6F12	,0x1000,
0x602A	,0x0BDE,
0x6F12	,0x1000,
0x602A	,0x0C56,
0x6F12	,0x1000,
0x602A	,0x0CB6,
0x6F12	,0x0100,
0x602A	,0x0CF2,
0x6F12	,0x0001,
0x602A	,0x0CF0,
0x6F12	,0x0101,
0x602A	,0x11B8,
0x6F12	,0x0100,
0x602A	,0x11F6,
0x6F12	,0x0020,
0x602A	,0x4A74,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0xD8FF,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0xD8FF,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x602A	,0x218E,
0x6F12	,0x0001,
0x602A	,0x2268,
0x6F12	,0xF379,
0x602A	,0x5006,
0x6F12	,0x0000,
0x602A	,0x500E,
0x6F12	,0x0100,
0x602A	,0x4E70,
0x6F12	,0x2062,
0x6F12	,0x5501,
0x6028	,0x4000,
0xF46A	,0xAE80,
0x0344	,0x0000,
0x0346	,0x0000,
0x0348	,0x1FFF,
0x034A	,0x181F,
0x034C	,0x0FF0,
0x034E	,0x0C00,
0x0350	,0x0008,
0x0352	,0x0008,
0x0900	,0x0122,
0x0380	,0x0002,
0x0382	,0x0002,
0x0384	,0x0002,
0x0386	,0x0002,
0x0110	,0x1002,
0x0114	,0x0301,
0x0116	,0x3000,
0x0136	,0x1800,
0x013E	,0x0000,
0x0300	,0x0006,
0x0302	,0x0001,
0x0304	,0x0004,
0x0306	,0x008C,
0x0308	,0x0008,
0x030A	,0x0001,
0x030C	,0x0000,
0x030E	,0x0004,
0x0310	,0x0067,
0x0312	,0x0000,
0x080E	,0x0000,
0x0340	,0x0C54,
0x0342	,0x1716,
0x0702	,0x0000,
0x0202	,0x0100,
0x0200	,0x0100,
0x0D00	,0x0101,
0x0D02	,0x0101,/*ersen.shang 0x0101*/
0x0D04	,0x0102,
0x6226	,0x0000,
}; /*	preview_setting_array  */

//Bin FDSUM 4080 x 3072 30fps mipi dphy 1236Mbps LLP 5910 FLL 3156 Tail size x 508 y 3056 offset x 8 y 8
static kal_uint16 capture_setting_array[] = {
0x6028	,0x2400,
0x602A	,0x1A28,
0x6F12	,0x4C00,
0x602A	,0x065A,
0x6F12	,0x0000,
0x602A	,0x139E,
0x6F12	,0x0100,
0x602A	,0x139C,
0x6F12	,0x0000,
0x602A	,0x13A0,
0x6F12	,0x0A00,
0x6F12	,0x0120,
0x602A	,0x2072,
0x6F12	,0x0000,
0x602A	,0x1A64,
0x6F12	,0x0301,
0x6F12	,0xFF00,
0x602A	,0x19E6,
0x6F12	,0x0200,
0x602A	,0x1A30,
0x6F12	,0x3401,
0x602A	,0x19FC,
0x6F12	,0x0B00,
0x602A	,0x19F4,
0x6F12	,0x0606,
0x602A	,0x19F8,
0x6F12	,0x1010,
0x602A	,0x1B26,
0x6F12	,0x6F80,
0x6F12	,0xA060,
0x602A	,0x1A3C,
0x6F12	,0x6207,
0x602A	,0x1A48,
0x6F12	,0x6207,
0x602A	,0x1444,
0x6F12	,0x2000,
0x6F12	,0x2000,
0x602A	,0x144C,
0x6F12	,0x3F00,
0x6F12	,0x3F00,
0x602A	,0x7F6C,
0x6F12	,0x0100,
0x6F12	,0x2F00,
0x6F12	,0xFA00,
0x6F12	,0x2400,
0x6F12	,0xE500,
0x602A	,0x0650,
0x6F12	,0x0600,
0x602A	,0x0654,
0x6F12	,0x0000,
0x602A	,0x1A46,
0x6F12	,0x8A00,
0x602A	,0x1A52,
0x6F12	,0xBF00,
0x602A	,0x0674,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x602A	,0x0668,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x602A	,0x0684,
0x6F12	,0x4001,
0x602A	,0x0688,
0x6F12	,0x4001,
0x602A	,0x147C,
0x6F12	,0x1000,
0x602A	,0x1480,
0x6F12	,0x1000,
0x602A	,0x19F6,
0x6F12	,0x0904,
0x602A	,0x0812,
0x6F12	,0x0000,
0x602A	,0x1A02,
0x6F12	,0x1800,
0x602A	,0x2148,
0x6F12	,0x0100,
0x602A	,0x2042,
0x6F12	,0x1A00,
0x602A	,0x0874,
0x6F12	,0x0100,
0x602A	,0x09C0,
0x6F12	,0x2008,
0x602A	,0x09C4,
0x6F12	,0x2000,
0x602A	,0x19FE,
0x6F12	,0x0E1C,
0x602A	,0x4D92,
0x6F12	,0x0100,
0x602A	,0x84C8,
0x6F12	,0x0100,
0x602A	,0x4D94,
0x6F12	,0x0005,
0x6F12	,0x000A,
0x6F12	,0x0010,
0x6F12	,0x0810,
0x6F12	,0x000A,
0x6F12	,0x0040,
0x6F12	,0x0810,
0x6F12	,0x0810,
0x6F12	,0x8002,
0x6F12	,0xFD03,
0x6F12	,0x0010,
0x6F12	,0x1510,
0x602A	,0x3570,
0x6F12	,0x0000,
0x602A	,0x3574,
0x6F12	,0x1201,
0x602A	,0x21E4,
0x6F12	,0x0400,
0x602A	,0x21EC,
0x6F12	,0x1F04,
0x602A	,0x2080,
0x6F12	,0x0100,
0x6F12	,0xFF00,
0x602A	,0x2086,
0x6F12	,0x0001,
0x602A	,0x208E,
0x6F12	,0x14F4,
0x602A	,0x208A,
0x6F12	,0xD244,
0x6F12	,0xD244,
0x602A	,0x120E,
0x6F12	,0x1000,
0x602A	,0x212E,
0x6F12	,0x0200,
0x602A	,0x13AE,
0x6F12	,0x0101,
0x602A	,0x0718,
0x6F12	,0x0001,
0x602A	,0x0710,
0x6F12	,0x0002,
0x6F12	,0x0804,
0x6F12	,0x0100,
0x602A	,0x1B5C,
0x6F12	,0x0000,
0x602A	,0x0786,
0x6F12	,0x7701,
0x602A	,0x2022,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x602A	,0x1360,
0x6F12	,0x0100,
0x602A	,0x1376,
0x6F12	,0x0100,
0x6F12	,0x6038,
0x6F12	,0x7038,
0x6F12	,0x8038,
0x602A	,0x1386,
0x6F12	,0x0B00,
0x602A	,0x06FA,
0x6F12	,0x1000,
0x602A	,0x4A94,
0x6F12	,0x0900,
0x6F12	,0x0000,
0x6F12	,0x0300,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0300,
0x6F12	,0x0000,
0x6F12	,0x0900,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x602A	,0x0A76,
0x6F12	,0x1000,
0x602A	,0x0AEE,
0x6F12	,0x1000,
0x602A	,0x0B66,
0x6F12	,0x1000,
0x602A	,0x0BDE,
0x6F12	,0x1000,
0x602A	,0x0C56,
0x6F12	,0x1000,
0x602A	,0x0CB6,
0x6F12	,0x0100,
0x602A	,0x0CF2,
0x6F12	,0x0001,
0x602A	,0x0CF0,
0x6F12	,0x0101,
0x602A	,0x11B8,
0x6F12	,0x0100,
0x602A	,0x11F6,
0x6F12	,0x0020,
0x602A	,0x4A74,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0xD8FF,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0xD8FF,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x602A	,0x218E,
0x6F12	,0x0001,
0x602A	,0x2268,
0x6F12	,0xF379,
0x602A	,0x5006,
0x6F12	,0x0000,
0x602A	,0x500E,
0x6F12	,0x0100,
0x602A	,0x4E70,
0x6F12	,0x2062,
0x6F12	,0x5501,
0x6028	,0x4000,
0xF46A	,0xAE80,
0x0344	,0x0000,
0x0346	,0x0000,
0x0348	,0x1FFF,
0x034A	,0x181F,
0x034C	,0x0FF0,
0x034E	,0x0C00,
0x0350	,0x0008,
0x0352	,0x0008,
0x0900	,0x0122,
0x0380	,0x0002,
0x0382	,0x0002,
0x0384	,0x0002,
0x0386	,0x0002,
0x0110	,0x1002,
0x0114	,0x0301,
0x0116	,0x3000,
0x0136	,0x1800,
0x013E	,0x0000,
0x0300	,0x0006,
0x0302	,0x0001,
0x0304	,0x0004,
0x0306	,0x008C,
0x0308	,0x0008,
0x030A	,0x0001,
0x030C	,0x0000,
0x030E	,0x0004,
0x0310	,0x0067,
0x0312	,0x0000,
0x080E	,0x0000,
0x0340	,0x0C54,
0x0342	,0x1716,
0x0702	,0x0000,
0x0202	,0x0100,
0x0200	,0x0100,
0x0D00	,0x0101,
0x0D02	,0x0101,/*ersen.shang 0x0101*/
0x0D04	,0x0102,
0x6226	,0x0000,
};

//Bin FDSUM 4080 x 3072 30fps mipi dphy 1236Mbps LLP 5910 FLL 3156 Tail size x 508 y 3056 offset x 8 y 8
static kal_uint16 normal_video_setting_array[] = {
0x6028	,0x2400,
0x602A	,0x1A28,
0x6F12	,0x4C00,
0x602A	,0x065A,
0x6F12	,0x0000,
0x602A	,0x139E,
0x6F12	,0x0100,
0x602A	,0x139C,
0x6F12	,0x0000,
0x602A	,0x13A0,
0x6F12	,0x0A00,
0x6F12	,0x0120,
0x602A	,0x2072,
0x6F12	,0x0000,
0x602A	,0x1A64,
0x6F12	,0x0301,
0x6F12	,0xFF00,
0x602A	,0x19E6,
0x6F12	,0x0200,
0x602A	,0x1A30,
0x6F12	,0x3401,
0x602A	,0x19FC,
0x6F12	,0x0B00,
0x602A	,0x19F4,
0x6F12	,0x0606,
0x602A	,0x19F8,
0x6F12	,0x1010,
0x602A	,0x1B26,
0x6F12	,0x6F80,
0x6F12	,0xA060,
0x602A	,0x1A3C,
0x6F12	,0x6207,
0x602A	,0x1A48,
0x6F12	,0x6207,
0x602A	,0x1444,
0x6F12	,0x2000,
0x6F12	,0x2000,
0x602A	,0x144C,
0x6F12	,0x3F00,
0x6F12	,0x3F00,
0x602A	,0x7F6C,
0x6F12	,0x0100,
0x6F12	,0x2F00,
0x6F12	,0xFA00,
0x6F12	,0x2400,
0x6F12	,0xE500,
0x602A	,0x0650,
0x6F12	,0x0600,
0x602A	,0x0654,
0x6F12	,0x0000,
0x602A	,0x1A46,
0x6F12	,0x8A00,
0x602A	,0x1A52,
0x6F12	,0xBF00,
0x602A	,0x0674,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x602A	,0x0668,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x602A	,0x0684,
0x6F12	,0x4001,
0x602A	,0x0688,
0x6F12	,0x4001,
0x602A	,0x147C,
0x6F12	,0x1000,
0x602A	,0x1480,
0x6F12	,0x1000,
0x602A	,0x19F6,
0x6F12	,0x0904,
0x602A	,0x0812,
0x6F12	,0x0000,
0x602A	,0x1A02,
0x6F12	,0x1800,
0x602A	,0x2148,
0x6F12	,0x0100,
0x602A	,0x2042,
0x6F12	,0x1A00,
0x602A	,0x0874,
0x6F12	,0x0100,
0x602A	,0x09C0,
0x6F12	,0x2008,
0x602A	,0x09C4,
0x6F12	,0x2000,
0x602A	,0x19FE,
0x6F12	,0x0E1C,
0x602A	,0x4D92,
0x6F12	,0x0100,
0x602A	,0x84C8,
0x6F12	,0x0100,
0x602A	,0x4D94,
0x6F12	,0x0005,
0x6F12	,0x000A,
0x6F12	,0x0010,
0x6F12	,0x0810,
0x6F12	,0x000A,
0x6F12	,0x0040,
0x6F12	,0x0810,
0x6F12	,0x0810,
0x6F12	,0x8002,
0x6F12	,0xFD03,
0x6F12	,0x0010,
0x6F12	,0x1510,
0x602A	,0x3570,
0x6F12	,0x0000,
0x602A	,0x3574,
0x6F12	,0x1201,
0x602A	,0x21E4,
0x6F12	,0x0400,
0x602A	,0x21EC,
0x6F12	,0x1F04,
0x602A	,0x2080,
0x6F12	,0x0100,
0x6F12	,0xFF00,
0x602A	,0x2086,
0x6F12	,0x0001,
0x602A	,0x208E,
0x6F12	,0x14F4,
0x602A	,0x208A,
0x6F12	,0xD244,
0x6F12	,0xD244,
0x602A	,0x120E,
0x6F12	,0x1000,
0x602A	,0x212E,
0x6F12	,0x0200,
0x602A	,0x13AE,
0x6F12	,0x0101,
0x602A	,0x0718,
0x6F12	,0x0001,
0x602A	,0x0710,
0x6F12	,0x0002,
0x6F12	,0x0804,
0x6F12	,0x0100,
0x602A	,0x1B5C,
0x6F12	,0x0000,
0x602A	,0x0786,
0x6F12	,0x7701,
0x602A	,0x2022,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x602A	,0x1360,
0x6F12	,0x0100,
0x602A	,0x1376,
0x6F12	,0x0100,
0x6F12	,0x6038,
0x6F12	,0x7038,
0x6F12	,0x8038,
0x602A	,0x1386,
0x6F12	,0x0B00,
0x602A	,0x06FA,
0x6F12	,0x1000,
0x602A	,0x4A94,
0x6F12	,0x0900,
0x6F12	,0x0000,
0x6F12	,0x0300,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0300,
0x6F12	,0x0000,
0x6F12	,0x0900,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x602A	,0x0A76,
0x6F12	,0x1000,
0x602A	,0x0AEE,
0x6F12	,0x1000,
0x602A	,0x0B66,
0x6F12	,0x1000,
0x602A	,0x0BDE,
0x6F12	,0x1000,
0x602A	,0x0C56,
0x6F12	,0x1000,
0x602A	,0x0CB6,
0x6F12	,0x0100,
0x602A	,0x0CF2,
0x6F12	,0x0001,
0x602A	,0x0CF0,
0x6F12	,0x0101,
0x602A	,0x11B8,
0x6F12	,0x0100,
0x602A	,0x11F6,
0x6F12	,0x0020,
0x602A	,0x4A74,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0xD8FF,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0xD8FF,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x602A	,0x218E,
0x6F12	,0x0001,
0x602A	,0x2268,
0x6F12	,0xF379,
0x602A	,0x5006,
0x6F12	,0x0000,
0x602A	,0x500E,
0x6F12	,0x0100,
0x602A	,0x4E70,
0x6F12	,0x2062,
0x6F12	,0x5501,
0x6028	,0x4000,
0xF46A	,0xAE80,
0x0344	,0x0000,
0x0346	,0x0000,
0x0348	,0x1FFF,
0x034A	,0x181F,
0x034C	,0x0FF0,
0x034E	,0x0C00,
0x0350	,0x0008,
0x0352	,0x0008,
0x0900	,0x0122,
0x0380	,0x0002,
0x0382	,0x0002,
0x0384	,0x0002,
0x0386	,0x0002,
0x0110	,0x1002,
0x0114	,0x0301,
0x0116	,0x3000,
0x0136	,0x1800,
0x013E	,0x0000,
0x0300	,0x0006,
0x0302	,0x0001,
0x0304	,0x0004,
0x0306	,0x008C,
0x0308	,0x0008,
0x030A	,0x0001,
0x030C	,0x0000,
0x030E	,0x0004,
0x0310	,0x0067,
0x0312	,0x0000,
0x080E	,0x0000,
0x0340	,0x0C54,
0x0342	,0x1716,
0x0702	,0x0000,
0x0202	,0x0100,
0x0200	,0x0100,
0x0D00	,0x0101,
0x0D02	,0x0101,/*ersen.shang 0x0101*/
0x0D04	,0x0102,
0x6226	,0x0000,
};

//FDSUM+A2A2 1280 x 720 120fps mipi dphy 1368Mbps LLP 2096 FLL 2384 Tail size x 320 y 356 offset x 0 y 4
static kal_uint16 hs_video_setting_array[] = {
0x6028	,0x2400,
0x602A	,0x1A28,
0x6F12	,0x4C00,
0x602A	,0x065A,
0x6F12	,0x0000,
0x602A	,0x139E,
0x6F12	,0x0300,
0x602A	,0x139C,
0x6F12	,0x0000,
0x602A	,0x13A0,
0x6F12	,0x0A00,
0x6F12	,0x0020,
0x602A	,0x2072,
0x6F12	,0x0000,
0x602A	,0x1A64,
0x6F12	,0x0301,
0x6F12	,0x3F00,
0x602A	,0x19E6,
0x6F12	,0x0201,
0x602A	,0x1A30,
0x6F12	,0x3401,
0x602A	,0x19FC,
0x6F12	,0x0B00,
0x602A	,0x19F4,
0x6F12	,0x0606,
0x602A	,0x19F8,
0x6F12	,0x1010,
0x602A	,0x1B26,
0x6F12	,0x6F80,
0x6F12	,0xA020,
0x602A	,0x1A3C,
0x6F12	,0x5207,
0x602A	,0x1A48,
0x6F12	,0x5207,
0x602A	,0x1444,
0x6F12	,0x2100,
0x6F12	,0x2100,
0x602A	,0x144C,
0x6F12	,0x4200,
0x6F12	,0x4200,
0x602A	,0x7F6C,
0x6F12	,0x0100,
0x6F12	,0x3100,
0x6F12	,0xF700,
0x6F12	,0x2600,
0x6F12	,0xE100,
0x602A	,0x0650,
0x6F12	,0x0600,
0x602A	,0x0654,
0x6F12	,0x0000,
0x602A	,0x1A46,
0x6F12	,0x8600,
0x602A	,0x1A52,
0x6F12	,0xBF00,
0x602A	,0x0674,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x6F12	,0x0500,
0x602A	,0x0668,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x6F12	,0x0800,
0x602A	,0x0684,
0x6F12	,0x4001,
0x602A	,0x0688,
0x6F12	,0x4001,
0x602A	,0x147C,
0x6F12	,0x1000,
0x602A	,0x1480,
0x6F12	,0x1000,
0x602A	,0x19F6,
0x6F12	,0x0904,
0x602A	,0x0812,
0x6F12	,0x0000,
0x602A	,0x1A02,
0x6F12	,0x0800,
0x602A	,0x2148,
0x6F12	,0x0100,
0x602A	,0x2042,
0x6F12	,0x1A00,
0x602A	,0x0874,
0x6F12	,0x1100,
0x602A	,0x09C0,
0x6F12	,0x1803,
0x602A	,0x09C4,
0x6F12	,0x1803,
0x602A	,0x19FE,
0x6F12	,0x0E1C,
0x602A	,0x4D92,
0x6F12	,0x0100,
0x602A	,0x84C8,
0x6F12	,0x0100,
0x602A	,0x4D94,
0x6F12	,0x4001,
0x6F12	,0x0004,
0x6F12	,0x0010,
0x6F12	,0x0810,
0x6F12	,0x0004,
0x6F12	,0x0010,
0x6F12	,0x0810,
0x6F12	,0x0810,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0010,
0x6F12	,0x0010,
0x602A	,0x3570,
0x6F12	,0x0000,
0x602A	,0x3574,
0x6F12	,0x3801,
0x602A	,0x21E4,
0x6F12	,0x0400,
0x602A	,0x21EC,
0x6F12	,0x6801,
0x602A	,0x2080,
0x6F12	,0x0100,
0x6F12	,0x7F00,
0x602A	,0x2086,
0x6F12	,0x8000,
0x602A	,0x208E,
0x6F12	,0x14F4,
0x602A	,0x208A,
0x6F12	,0xC244,
0x6F12	,0xD244,
0x602A	,0x120E,
0x6F12	,0x1000,
0x602A	,0x212E,
0x6F12	,0x0A00,
0x602A	,0x13AE,
0x6F12	,0x0102,
0x602A	,0x0718,
0x6F12	,0x0005,
0x602A	,0x0710,
0x6F12	,0x0004,
0x6F12	,0x0401,
0x6F12	,0x0100,
0x602A	,0x1B5C,
0x6F12	,0x0300,
0x602A	,0x0786,
0x6F12	,0x7701,
0x602A	,0x2022,
0x6F12	,0x0101,
0x6F12	,0x0101,
0x602A	,0x1360,
0x6F12	,0x0000,
0x602A	,0x1376,
0x6F12	,0x0200,
0x6F12	,0x6038,
0x6F12	,0x7038,
0x6F12	,0x8038,
0x602A	,0x1386,
0x6F12	,0x0B00,
0x602A	,0x06FA,
0x6F12	,0x1000,
0x602A	,0x4A94,
0x6F12	,0x1600,
0x6F12	,0x0000,
0x6F12	,0x1000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x1000,
0x6F12	,0x0000,
0x6F12	,0x1600,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x602A	,0x0A76,
0x6F12	,0x1000,
0x602A	,0x0AEE,
0x6F12	,0x1000,
0x602A	,0x0B66,
0x6F12	,0x1000,
0x602A	,0x0BDE,
0x6F12	,0x1000,
0x602A	,0x0C56,
0x6F12	,0x1000,
0x602A	,0x0CB6,
0x6F12	,0x0000,
0x602A	,0x0CF2,
0x6F12	,0x0001,
0x602A	,0x0CF0,
0x6F12	,0x0101,
0x602A	,0x11B8,
0x6F12	,0x0000,
0x602A	,0x11F6,
0x6F12	,0x0010,
0x602A	,0x4A74,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0xD8FF,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0xD8FF,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x6F12	,0x0000,
0x602A	,0x218E,
0x6F12	,0x0000,
0x602A	,0x2268,
0x6F12	,0xF279,
0x602A	,0x5006,
0x6F12	,0x0000,
0x602A	,0x500E,
0x6F12	,0x0100,
0x602A	,0x4E70,
0x6F12	,0x2062,
0x6F12	,0x5501,
0x6028	,0x4000,
0xF46A	,0xAE80,
0x0344	,0x05F0,
0x0346	,0x0660,
0x0348	,0x1A0F,
0x034A	,0x11BF,
0x034C	,0x0500,
0x034E	,0x02D0,
0x0350	,0x0004,
0x0352	,0x0004,
0x0900	,0x0144,
0x0380	,0x0002,
0x0382	,0x0006,
0x0384	,0x0002,
0x0386	,0x0006,
0x0110	,0x1002,
0x0114	,0x0301,
0x0116	,0x3000,
0x0136	,0x1800,
0x013E	,0x0000,
0x0300	,0x0006,
0x0302	,0x0001,
0x0304	,0x0004,
0x0306	,0x0096,
0x0308	,0x0008,
0x030A	,0x0001,
0x030C	,0x0000,
0x030E	,0x0004,
0x0310	,0x0072,
0x0312	,0x0000,
0x080E	,0x0000,
0x0340	,0x0950,
0x0342	,0x0830,
0x0702	,0x0000,
0x0202	,0x0100,
0x0200	,0x0100,
0x0D00	,0x0101,
0x0D02	,0x0101,
0x0D04	,0x0102,
0x011E	,0x0100,
0x6226	,0x0000,
};

static kal_uint16 slim_video_setting_array[] = {

	0x6028, 0x2400, 0x602A, 0x1A28, 0x6F12, 0x4C00, 0x602A, 0x065A, 0x6F12,
	0x0000, 0x602A, 0x139E, 0x6F12, 0x0100, 0x602A, 0x139C, 0x6F12, 0x0000,
	0x602A, 0x13A0, 0x6F12, 0x0A00, 0x6F12, 0x0120, 0x602A, 0x2072, 0x6F12,
	0x0000, 0x602A, 0x1A64, 0x6F12, 0x0301, 0x6F12, 0xFF00, 0x602A, 0x19E6,
	0x6F12, 0x0200, 0x602A, 0x1A30, 0x6F12, 0x3401, 0x602A, 0x19FC, 0x6F12,
	0x0B00, 0x602A, 0x19F4, 0x6F12, 0x0606, 0x602A, 0x19F8, 0x6F12, 0x1010,
	0x602A, 0x1B26, 0x6F12, 0x6F80, 0x6F12, 0xA060, 0x602A, 0x1A3C, 0x6F12,
	0x6207, 0x602A, 0x1A48, 0x6F12, 0x6207, 0x602A, 0x1444, 0x6F12, 0x2000,
	0x6F12, 0x2000, 0x602A, 0x144C, 0x6F12, 0x3F00, 0x6F12, 0x3F00, 0x602A,
	0x7F6C, 0x6F12, 0x0100, 0x6F12, 0x2F00, 0x6F12, 0xFA00, 0x6F12, 0x2400,
	0x6F12, 0xE500, 0x602A, 0x0650, 0x6F12, 0x0600, 0x602A, 0x0654, 0x6F12,
	0x0000, 0x602A, 0x1A46, 0x6F12, 0xB000, 0x602A, 0x1A52, 0x6F12, 0xBF00,
	0x602A, 0x0674, 0x6F12, 0x0500, 0x6F12, 0x0500, 0x6F12, 0x0500, 0x6F12,
	0x0500, 0x602A, 0x0668, 0x6F12, 0x0800, 0x6F12, 0x0800, 0x6F12, 0x0800,
	0x6F12, 0x0800, 0x602A, 0x0684, 0x6F12, 0x4001, 0x602A, 0x0688, 0x6F12,
	0x4001, 0x602A, 0x147C, 0x6F12, 0x1000, 0x602A, 0x1480, 0x6F12, 0x1000,
	0x602A, 0x19F6, 0x6F12, 0x0904, 0x602A, 0x0812, 0x6F12, 0x0010, 0x602A,
	0x2148, 0x6F12, 0x0100, 0x602A, 0x2042, 0x6F12, 0x1A00, 0x602A, 0x0874,
	0x6F12, 0x0100, 0x602A, 0x09C0, 0x6F12, 0x2008, 0x602A, 0x09C4, 0x6F12,
	0x2000, 0x602A, 0x19FE, 0x6F12, 0x0E1C, 0x602A, 0x4D92, 0x6F12, 0x0100,
	0x602A, 0x8104, 0x6F12, 0x0100, 0x602A, 0x4D94, 0x6F12, 0x0005, 0x6F12,
	0x000A, 0x6F12, 0x0010, 0x6F12, 0x1510, 0x6F12, 0x000A, 0x6F12, 0x0040,
	0x6F12, 0x1510, 0x6F12, 0x1510, 0x602A, 0x3570, 0x6F12, 0x0000, 0x602A,
	0x3574, 0x6F12, 0x4700, 0x602A, 0x21E4, 0x6F12, 0x0400, 0x602A, 0x21EC,
	0x6F12, 0xC702, 0x602A, 0x2080, 0x6F12, 0x0100, 0x6F12, 0xFF00, 0x602A,
	0x2086, 0x6F12, 0x0001, 0x602A, 0x208E, 0x6F12, 0x14F4, 0x602A, 0x208A,
	0x6F12, 0xD244, 0x6F12, 0xD244, 0x602A, 0x120E, 0x6F12, 0x1000, 0x602A,
	0x212E, 0x6F12, 0x0200, 0x602A, 0x13AE, 0x6F12, 0x0101, 0x602A, 0x0718,
	0x6F12, 0x0001, 0x602A, 0x0710, 0x6F12, 0x0002, 0x6F12, 0x0804, 0x6F12,
	0x0100, 0x602A, 0x1B5C, 0x6F12, 0x0000, 0x602A, 0x0786, 0x6F12, 0x7701,
	0x602A, 0x2022, 0x6F12, 0x0500, 0x6F12, 0x0500, 0x602A, 0x1360, 0x6F12,
	0x0100, 0x602A, 0x1376, 0x6F12, 0x0100, 0x6F12, 0x6038, 0x6F12, 0x7038,
	0x6F12, 0x8038, 0x602A, 0x1386, 0x6F12, 0x0B00, 0x602A, 0x06FA, 0x6F12,
	0x0000, 0x602A, 0x4A94, 0x6F12, 0x0600, 0x6F12, 0x0600, 0x6F12, 0x0600,
	0x6F12, 0x0600, 0x6F12, 0x0600, 0x6F12, 0x0600, 0x6F12, 0x0600, 0x6F12,
	0x0600, 0x6F12, 0x0600, 0x6F12, 0x0600, 0x6F12, 0x0600, 0x6F12, 0x0600,
	0x6F12, 0x0600, 0x6F12, 0x0600, 0x6F12, 0x0600, 0x6F12, 0x0600, 0x602A,
	0x0A76, 0x6F12, 0x1000, 0x602A, 0x0AEE, 0x6F12, 0x1000, 0x602A, 0x0B66,
	0x6F12, 0x1000, 0x602A, 0x0BDE, 0x6F12, 0x1000, 0x602A, 0x0C56, 0x6F12,
	0x1000, 0x602A, 0x0CF2, 0x6F12, 0x0001, 0x602A, 0x0CF0, 0x6F12, 0x0101,
	0x602A, 0x11B8, 0x6F12, 0x0100, 0x602A, 0x11F6, 0x6F12, 0x0020, 0x602A,
	0x4A74, 0x6F12, 0x0000, 0x6F12, 0x0000, 0x6F12, 0xD8FF, 0x6F12, 0x0000,
	0x6F12, 0x0000, 0x6F12, 0x0000, 0x6F12, 0x0000, 0x6F12, 0x0000, 0x6F12,
	0xD8FF, 0x6F12, 0x0000, 0x6F12, 0x0000, 0x6F12, 0x0000, 0x6F12, 0x0000,
	0x6F12, 0x0000, 0x6F12, 0x0000, 0x6F12, 0x0000, 0x6028, 0x4000, 0xF46A,
	0xAE80, 0x0344, 0x0000, 0x0346, 0x0308, 0x0348, 0x1FFF, 0x034A, 0x1517,
	0x034C, 0x0FF0, 0x034E, 0x08F8, 0x0350, 0x0008, 0x0352, 0x0008, 0x0900,
	0x0122, 0x0380, 0x0002, 0x0382, 0x0002, 0x0384, 0x0002, 0x0386, 0x0002,
	0x0110, 0x1002, 0x0114, 0x0300, 0x0116, 0x3000, 0x0136, 0x1800, 0x013E,
	0x0000, 0x0300, 0x0006, 0x0302, 0x0001, 0x0304, 0x0004, 0x0306, 0x008C,
	0x0308, 0x0008, 0x030A, 0x0001, 0x030C, 0x0000, 0x030E, 0x0004, 0x0310,
	0x0064, 0x0312, 0x0000, 0x080E, 0x0000, 0x0340, 0x0C54, 0x0342, 0x1716,
	0x0702, 0x0000, 0x0202, 0x0100, 0x0200, 0x0100, 0x0D00, 0x0101, 0x0D02,
	0x0001, 0x0D04, 0x0102, 0x6226, 0x0000,
};
#endif

#define FPT_PDAF_SUPPORT 1

#if FPT_PDAF_SUPPORT
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum = 4,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	.i4BlockNumX = 508,
	.i4BlockNumY = 382,
	.iMirrorFlip = 0,
	.i4PosL =
	{
		{ 11,  8 },
		{  9, 11 },
		{ 13, 12 },
		{ 15, 15 }
	},
	.i4PosR =
	{
		{ 10,  8 },
		{  8, 11 },
		{ 12, 12 },
		{ 14, 15 }
	},
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting PD output size is 288(Pixel)*1728 */
	{
		0x03, 0x0A,   0x00,   0x08, 0x40, 0x00,
		0x00, 0x2B, 0x07F8, 0x0600, 0x00, 0x00, 0x0000, 0x0000,
		0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Capture mode setting,PD output size is 288(Pixel)*1728 */
	{
		0x03, 0x0A,   0x00,   0x08, 0x40, 0x00,
		0x00, 0x2B, 0x0FF0, 0x0CC0, 0x00, 0x00, 0x0000, 0x0000,
		0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Video mode setting 288(Pixel)*1728 */
	{
		0x03, 0x0A,   0x00,   0x08, 0x40, 0x00,
		0x00, 0x2B, 0x0FF0, 0x0CC0, 0x00, 0x00, 0x0000, 0x0000,
		0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000
	},
};
#endif
static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	//kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF),(char)(para & 0xFF) };

	//kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = { (char)(addr >> 8), (char)(addr & 0xFF),(char)(para >> 8), (char)(para & 0xFF) };

	//kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

#if MULTI_WRITE
static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data = 0;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];
		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;
			//LOG_INF("i2c_write_id start[0x%x] Addr[0x%x] Data[0x%x] addr_last[0x%x]\n", imgsensor.i2c_write_id, addr, data, addr_last);
		}

		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len ||addr != addr_last) {
			LOG_INF("i2c_write_id end[0x%x] Addr[0x%x] Data[0x%x] addr_last[0x%x], tosend[%d], IDX[%d], len[%d]\n",
				imgsensor.i2c_write_id, addr, data, addr_last,tosend, IDX, len);

			while (iBurstWriteReg_multi(puSendCmd, tosend,
					imgsensor.i2c_write_id, 4,imgsensor_info.i2c_speed) !=0) {
				LOG_INF("iBurstWriteReg_multi FAIL!, retry!");
			}
			tosend = 0;
		}
	}
	return 0;
}
#endif
/* Begin binchang.liang optimize ggc write reg operation 2021/11/18 */
#if 1
static kal_uint16 hwggc_table_write_cmos_sensor(kal_uint16 uw_addr,kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data = 0;

	tosend = 0;
	IDX = 0;
	addr = uw_addr;
	while (len > IDX) {
		//addr = uw_addr;
		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 1;
			//addr_last = addr;
			//uw_addr++;
			//LOG_INF("i2c_write_id start[0x%x] Addr[0x%x] Data[0x%x] addr_last[0x%x]\n", imgsensor.i2c_write_id, addr, data, addr_last);
		}

		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len /*||addr != addr_last*/) {
			LOG_INF("lbc i2c_write_id end[0x%x] Addr[0x%x] Data[0x%x] addr_last[0x%x], tosend[%d], IDX[%d], len[%d]\n",
				imgsensor.i2c_write_id, addr, data, addr_last,tosend, IDX, len);

			while (iBurstWriteReg_multi(puSendCmd, tosend,
					imgsensor.i2c_write_id, 4,imgsensor_info.i2c_speed) !=0) {
				LOG_INF("iBurstWriteReg_multi FAIL!, retry!");
			}
			tosend = 0;
		}
	}
	return 0;
}
#endif
/* End binchang.liang optimize ggc write reg operation 2021/11/18 */
static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line,imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
} /*set_dummy*/

static void set_max_framerate(UINT16 framerate,
							  kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable(%d)\n",framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
							frame_length :
							imgsensor.min_frame_length;
	imgsensor.dummy_line =imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
} /*set_max_framerate  */

static kal_uint32 streaming_control(kal_bool enable)
{
	int i = 0;
	int framecnt = 0;
	int isStreamOn = 0;

	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		for (i = 0; i < 1000; i++) {
			write_cmos_sensor_byte(0x0100, 0X01);
			isStreamOn = read_cmos_sensor_byte(0x0100); /* waiting for sensor to  stop output  then  set the  setting */
			LOG_INF("isStreamOn %d ", isStreamOn);

			if ((isStreamOn & 0x1) == 0x01) {
				return ERROR_NONE;
			} else {
				mdelay(1);
			}
		}
	} else {
		for (i = 0; i < 1000; i++) {
			write_cmos_sensor_byte(0x0100, 0x00);
			framecnt = read_cmos_sensor_byte(0x0005);
			if ((framecnt & 0xff) == 0xFF) {
				LOG_INF("StreamOff OK at framecnt=%d.\n",framecnt);
				break;
			} else {
				LOG_INF("StreamOFF is not on, %d, i=%d",framecnt, i);
				mdelay(1);
			}
		}
	}
	return ERROR_NONE;
}

static int bNeedSetNormalMode = 0;

static void write_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	/*if shutter bigger than frame_length, should extend frame length first*/
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
				imgsensor_info.min_shutter :
				shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			// Extend frame length
			write_cmos_sensor(0x0340,imgsensor.frame_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	/* long expsoure */
	if (shutter >= 122159) {
		LOG_INF("Enter long exposure mode");
		bNeedSetNormalMode = 1;
		write_cmos_sensor(0x6028, 0x4000);
		streaming_control(KAL_FALSE);
		
		if (shutter <= 122159){
			LOG_INF("1s");
			write_cmos_sensor(0x0340, 0x0780);
			write_cmos_sensor(0x0202, 0x0774);
			write_cmos_sensor(0x0702, 0x0600);
			write_cmos_sensor(0x0704, 0x0600);
		} else if (shutter <= 244319){
			LOG_INF("2s");
			write_cmos_sensor(0x0340, 0x0EF5);
			write_cmos_sensor(0x0202, 0x0EE9);
			write_cmos_sensor(0x0702, 0x0600);
			write_cmos_sensor(0x0704, 0x0600);
		} else if (shutter <= 488639){
			LOG_INF("4s");
			write_cmos_sensor(0x0340, 0x1DDF);
			write_cmos_sensor(0x0202, 0x1DD3);
			write_cmos_sensor(0x0702, 0x0600);
			write_cmos_sensor(0x0704, 0x0600);
		} else if (shutter <= 977278){
			LOG_INF("8s");
			write_cmos_sensor(0x0340, 0x3BB2);
			write_cmos_sensor(0x0202, 0x3BA6);
			write_cmos_sensor(0x0702, 0x0600);
			write_cmos_sensor(0x0704, 0x0600);
		} else{
			LOG_INF("16s");
			write_cmos_sensor(0x0340, 0x7759);
			write_cmos_sensor(0x0202, 0x774D);
			write_cmos_sensor(0x0702, 0x0600);
			write_cmos_sensor(0x0704, 0x0600);
		}
		streaming_control(KAL_TRUE);
		/* Frame exposure mode customization for LE*/
		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 1;
	} else {
		if (bNeedSetNormalMode == 1) {
			LOG_INF("Exit long exposure mode");
			streaming_control(KAL_FALSE);
			write_cmos_sensor(0x0340,imgsensor.frame_length & 0xFFFF);
			write_cmos_sensor(0x0702, 0x0000);
			write_cmos_sensor(0x0704, 0x0000);
			write_cmos_sensor(0x0202, shutter & 0xFFFF);
			streaming_control(KAL_TRUE);
			bNeedSetNormalMode = 0;
		} else {
			write_cmos_sensor(0x0202, shutter & 0xFFFF);
			imgsensor.current_ae_effective_frame = 1;
		}
	}
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
}

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

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
} /* set_shutter */

/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
//lihao for dul cam frame sync 20210914
static void set_shutter_frame_length(kal_uint16 shutter,
					kal_uint16 frame_length)
{
	unsigned long flags;
        kal_int32 dummy_line = 0;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;

	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
//	else
//		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ?
				imgsensor_info.min_shutter :
				shutter;
	shutter = (shutter >(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
				(imgsensor_info.max_frame_length -imgsensor_info.margin) :
				shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			// Extend frame length
			write_cmos_sensor(0x0340,imgsensor.frame_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0X0202, shutter & 0xFFFF);
	LOG_INF("Exit! 0914 shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

} /* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	//gain = 64 = 1x real gain.
	reg_gain = gain / 2;
	//reg_gain = reg_gain & 0xFFFF;
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
	//gain = 64 = 1x real gain.
	kal_uint16 reg_gain;

	if (gain < BASEGAIN || gain > 64 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 64 * BASEGAIN)
			gain = 64 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain & 0xFFFF));
	return gain;
} /*set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se,
								    kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;

		// Extend frame length first
		set_gain(gain);
	}
}

static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	spin_lock(&imgsensor_drv_lock);
	//image_mirror &= 3;
	imgsensor.mirror = image_mirror;
	spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
	case IMAGE_NORMAL:
		//write_cmos_sensor_byte(0x0101, 0X03); //GR
		write_cmos_sensor_byte(0x0101, 0x00); //GR
		break;
	case IMAGE_H_MIRROR:
		//write_cmos_sensor_byte(0x0101, 0X02); //R
		write_cmos_sensor_byte(0x0101, 0x01); //R
		break;
	case IMAGE_V_MIRROR:
		//write_cmos_sensor_byte(0x0101, 0X01); //B
		write_cmos_sensor_byte(0x0101, 0x02); //B
		break;
	case IMAGE_HV_MIRROR:
		//write_cmos_sensor_byte(0x0101, 0X00); //GB
		write_cmos_sensor_byte(0x0101, 0x03); //GB
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
} /*night_mode*/

typedef struct hw_ggc
{
	BYTE hwGGCFlag;
	BYTE hwGGCData[346];
	BYTE chksum;
} hwggc_struct;

extern hwggc_struct tsnp1269_hwggc;

static void sensor_init(void)
{
	LOG_INF("[%s] +", __func__);
	LOG_INF("[%s] start", __func__);

	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0000, 0x0002);
	write_cmos_sensor(0x0000, 0x38E1);
	write_cmos_sensor(0x001E, 0x0007);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6010, 0x0001);
	mdelay(5);
	write_cmos_sensor(0x6226, 0x0001);
	mdelay(10);

	table_write_cmos_sensor(sensor_init_setting_array,sizeof(sensor_init_setting_array) /sizeof(kal_uint16));

	/*write hw ggc follow samsung's requirement*/
	if(tsnp1269_hwggc.hwGGCFlag){
		//int i;
		kal_uint16 *phwGGCData = (kal_uint16 *)&tsnp1269_hwggc.hwGGCData[0];

		write_cmos_sensor(0x6028, 0x2400);
		write_cmos_sensor(0x602A, 0x0CFC);
	/* Begin binchang.liang optimize ggc write reg operation 2021/11/18 */
		//for(i = 0; i < sizeof(tsnp1269_hwggc.hwGGCData)/2 ; i++)
			//write_cmos_sensor(0x6F12, phwGGCData[i]);
		hwggc_table_write_cmos_sensor(0x6F12,phwGGCData,sizeof(tsnp1269_hwggc.hwGGCData)/2);
	/* End binchang.liang optimize ggc write reg operation 2021/11/18 */
	}

	LOG_INF("[%s] -", __func__);
	LOG_INF("[%s] end", __func__);
}

static void preview_setting(void)
{
	LOG_INF("[%s] +", __func__);
	LOG_INF("[%s] start", __func__);

	table_write_cmos_sensor(preview_setting_array,sizeof(preview_setting_array) /sizeof(kal_uint16));

	LOG_INF("[%s] -", __func__);
	LOG_INF("[%s] end", __func__);

} /* preview_setting */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("[%s] +", __func__);
	LOG_INF("[%s] start", __func__);
	table_write_cmos_sensor(capture_setting_array,sizeof(capture_setting_array)/sizeof(kal_uint16));
	LOG_INF("[%s] -", __func__);
	LOG_INF("[%s] end", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("[%s] +", __func__);
	LOG_INF("[%s] start", __func__);
	table_write_cmos_sensor(normal_video_setting_array,sizeof(normal_video_setting_array) /sizeof(kal_uint16));
	LOG_INF("[%s] -", __func__);
	LOG_INF("[%s] end", __func__);
}

static void hs_video_setting(void)
{
	LOG_INF("[%s] start", __func__);
	LOG_INF("[%s] +", __func__);
	table_write_cmos_sensor(hs_video_setting_array,sizeof(hs_video_setting_array) /sizeof(kal_uint16));
	LOG_INF("[%s] -", __func__);
	LOG_INF("[%s] end", __func__);
}

static void slim_video_setting(void)
{
	LOG_INF("[%s] start", __func__);
	LOG_INF("[%s] +", __func__);
	table_write_cmos_sensor(slim_video_setting_array,sizeof(slim_video_setting_array) /sizeof(kal_uint16));
	LOG_INF("[%s] -", __func__);
	LOG_INF("[%s] end", __func__);
}

static kal_uint32 return_sensor_id(void)
{
	kal_uint32 physicalID = 0;
	physicalID = ((read_cmos_sensor_byte(0x0000) << 8) |read_cmos_sensor_byte(0x0001));

	if(physicalID == 0x38e1 && (get_tsppsnp1269_s5kjn1_lensID() == 0xA1))
	{
		return physicalID+1;
	}
	return 0;
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

	LOG_INF("[%s] +", __func__);

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n",
					imgsensor.i2c_write_id, *sensor_id,
					imgsensor_info.sensor_id);

				if(check_tsppsnp1269_s5kjn1_otp())
					*sensor_id |= 0x01000000;

				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n",
				imgsensor.i2c_write_id, *sensor_id,
				imgsensor_info.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}

	if (*sensor_id != imgsensor_info.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		LOG_INF("[%s] -error-", __func__);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	LOG_INF("[%s] -", __func__);
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
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	LOG_INF("[%s] +", __func__);
	LOG_1;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	/* initail sequence write in  */
	LOG_INF("[jw] imgsensor init start");
	sensor_init();
	LOG_INF("[jw] imgsensor init fin");
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
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

	LOG_INF("[%s] -", __func__);
	return ERROR_NONE;
} /*  open  */

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
	LOG_INF("[%s] +", __func__);

	/*No Need to implement this function*/

	LOG_INF("[%s] -", __func__);
	return ERROR_NONE;
} /*	close  */

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
	mdelay(2);

	return ERROR_NONE;
} /*	preview   */

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

	capture_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_HV_MIRROR);
	mdelay(2);

	return ERROR_NONE;
} /* capture() */

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

	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_HV_MIRROR);

	return ERROR_NONE;
} /*normal_video   */

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
} /*hs_video   */

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
	sensor_resolution->SensorFullWidth =imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
} /*get_resolution*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity =SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity =SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =imgsensor_info.slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame =imgsensor_info.ae_shut_delay_frame; /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame =imgsensor_info.ae_sensor_gain_delay_frame; /* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame =imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0; // 0 is default 1x
	sensor_info->SensorHightSampling = 0; // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;
#if FPT_PDAF_SUPPORT
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;//PDAF_SUPPORT_RAW;
#endif

	sensor_info->FrameTimeDelayFrame =imgsensor_info.frame_time_delay_frame;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX =imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->PDAF_Support = 0;
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
} /*	get_info  */

static char *scenarios[6] = {
	"MSDK_SCENARIO_ID_CAMERA_PREVIEW",
	"MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG",
	"MSDK_SCENARIO_ID_VIDEO_PREVIEW",
	"MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO",
	"MSDK_SCENARIO_ID_SLIM_VIDEO",
	"MSDK_SCENARIO_ID_MAX"
};

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d[%s]\n", scenario_id, scenarios[scenario_id]);
	LOG_INF("[%s] +", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
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
	LOG_INF("[%s] -", __func__);
	return ERROR_NONE;
} /* control() */

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

	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id,MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d[%s], framerate = %d\n", scenario_id,
		scenarios[scenario_id], framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 /imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =(frame_length > imgsensor_info.pre.framelength) ?
			(frame_length -imgsensor_info.pre.framelength) :
			0;
		imgsensor.frame_length =imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate *10 / imgsensor_info.normal_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =(frame_length >imgsensor_info.normal_video.framelength) ?
			(frame_length -imgsensor_info.normal_video.framelength) :
			0;
		imgsensor.frame_length =imgsensor_info.normal_video.framelength +imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frame_length = imgsensor_info.cap.pclk / framerate * 10 /imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =(frame_length > imgsensor_info.cap.framelength) ?
			(frame_length -imgsensor_info.cap.framelength) :
			0;
		imgsensor.frame_length =imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 /imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =(frame_length > imgsensor_info.hs_video.framelength) ?
				(frame_length -imgsensor_info.hs_video.framelength) :
				0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength +imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 /
			       imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =(frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length -imgsensor_info.slim_video.framelength) :
			0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength +imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default: //coding with  preview scenario by default
		frame_length = imgsensor_info.pre.pclk / framerate * 10 /imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =(frame_length > imgsensor_info.pre.framelength) ?
			(frame_length -imgsensor_info.pre.framelength) :
			0;
		imgsensor.frame_length =imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id,MUINT32 *framerate)
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
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x3202, 0x0080);
		write_cmos_sensor(0x3204, 0x0080);
		write_cmos_sensor(0x3206, 0x0080);
		write_cmos_sensor(0x3208, 0x0080);
		write_cmos_sensor(0x3232, 0x0000);
		write_cmos_sensor(0x3234, 0x0000);
		write_cmos_sensor(0x32a0, 0x0100);
		write_cmos_sensor(0x3300, 0x0001);
		write_cmos_sensor(0x3400, 0x0001);
		write_cmos_sensor(0x3402, 0x4e00);
		write_cmos_sensor(0x3268, 0x0000);
		write_cmos_sensor(0x0600, 0x0002);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x3202, 0x0000);
		write_cmos_sensor(0x3204, 0x0000);
		write_cmos_sensor(0x3206, 0x0000);
		write_cmos_sensor(0x3208, 0x0000);
		write_cmos_sensor(0x3232, 0x0000);
		write_cmos_sensor(0x3234, 0x0000);
		write_cmos_sensor(0x32a0, 0x0000);
		write_cmos_sensor(0x3300, 0x0000);
		write_cmos_sensor(0x3400, 0x0000);
		write_cmos_sensor(0x3402, 0x0000);
		write_cmos_sensor(0x3268, 0x0000);
		write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

#if 0
static char *features[300] = {

	"SENSOR_FEATURE_BEGIN,                              ",
	"SENSOR_FEATURE_GET_RESOLUTION,						",
	"SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE,    ",
	"SENSOR_FEATURE_GET_PERIOD,                         ",
	"SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ,               ",
	"SENSOR_FEATURE_SET_ESHUTTER,                       ",
	"SENSOR_FEATURE_SET_NIGHTMODE,                      ",
	"SENSOR_FEATURE_SET_GAIN,                           ",
	"SENSOR_FEATURE_SET_DUAL_GAIN,                      ",
	"SENSOR_FEATURE_SET_GAIN_AND_ESHUTTER,              ",
	"SENSOR_FEATURE_SET_FLASHLIGHT,                     ",
	"SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ,          ",
	"SENSOR_FEATURE_SET_REGISTER,                       ",
	"SENSOR_FEATURE_GET_REGISTER,                       ",
	"SENSOR_FEATURE_SET_CCT_REGISTER,                   ",
	"SENSOR_FEATURE_GET_CCT_REGISTER,                   ",
	"SENSOR_FEATURE_SET_ENG_REGISTER,                   ",
	"SENSOR_FEATURE_GET_ENG_REGISTER,                   ",
	"SENSOR_FEATURE_GET_REGISTER_DEFAULT,               ",
	"SENSOR_FEATURE_GET_CONFIG_PARA,                    ",
	"SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR,              ",
	"SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA,              ",
	"SENSOR_FEATURE_GET_GROUP_COUNT,                    ",
	"SENSOR_FEATURE_GET_GROUP_INFO,                     ",
	"SENSOR_FEATURE_GET_ITEM_INFO,                      ",
	"SENSOR_FEATURE_SET_ITEM_INFO,                      ",
	"SENSOR_FEATURE_GET_ENG_INFO,                       ",
	"SENSOR_FEATURE_GET_LENS_DRIVER_ID,                 ",
	"SENSOR_FEATURE_SET_YUV_CMD,                        ",
	"SENSOR_FEATURE_SET_VIDEO_MODE,                     ",
	"SENSOR_FEATURE_SET_TARGET_FRAME_RATE,              ",
	"SENSOR_FEATURE_SET_CALIBRATION_DATA,               ",
	"SENSOR_FEATURE_SET_SENSOR_SYNC,                    ",
	"SENSOR_FEATURE_INITIALIZE_AF,                      ",
	"SENSOR_FEATURE_CONSTANT_AF,                        ",
	"SENSOR_FEATURE_INFINITY_AF,                        ",
	"SENSOR_FEATURE_MOVE_FOCUS_LENS,                    ",
	"SENSOR_FEATURE_GET_AF_STATUS,                      ",
	"SENSOR_FEATURE_GET_AE_STATUS,                      ",
	"SENSOR_FEATURE_GET_AWB_STATUS,                     ",
	"SENSOR_FEATURE_GET_AF_INF,                         ",
	"SENSOR_FEATURE_GET_AF_MACRO,                       ",
	"SENSOR_FEATURE_CHECK_SENSOR_ID,                    ",
	"SENSOR_FEATURE_SET_AUTO_FLICKER_MODE,              ",
	"SENSOR_FEATURE_SET_TEST_PATTERN,                   ",
	"SENSOR_FEATURE_SET_SOFTWARE_PWDN,                  ",
	"SENSOR_FEATURE_SINGLE_FOCUS_MODE,                  ",
	"SENSOR_FEATURE_CANCEL_AF,                          ",
	"SENSOR_FEATURE_SET_AF_WINDOW,                      ",
	"SENSOR_FEATURE_GET_EV_AWB_REF,                     ",
	"SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN,          ",
	"SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS,         ",
	"SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS,      ",
	"SENSOR_FEATURE_SET_AE_WINDOW,                      ",
	"SENSOR_FEATURE_GET_EXIF_INFO,                      ",
	"SENSOR_FEATURE_GET_DELAY_INFO,                     ",
	"SENSOR_FEATURE_SET_SLAVE_I2C_ID,                   ",
	"SENSOR_FEATURE_SUSPEND,                            ",
	"SENSOR_FEATURE_RESUME,                             ",
	"SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO,     ",
	"SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO, ",
	"SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO,               ",
	"SENSOR_FEATURE_AUTOTEST_CMD,                       ",
	"SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE,    ",
	"SENSOR_FEATURE_GET_TEMPERATURE_VALUE,              ",
	"SENSOR_FEATURE_GET_SENSOR_CURRENT_TEMPERATURE,     ",
	"SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO,             ",
	"SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO,        ",
	"SENSOR_FEATURE_SET_YUV_3A_CMD,                     ",
	"SENSOR_FEATURE_SET_N3D_I2C_STREAM_REGDATA,         ",
	"SENSOR_FEATURE_SET_N3D_STOP_STREAMING,             ",
	"SENSOR_FEATURE_SET_N3D_START_STREAMING,            ",
	"SENSOR_FEATURE_GET_SENSOR_N3D_STREAM_TO_VSYNC_TIME,",
	"SENSOR_FEATURE_SET_ESHUTTER_GAIN,                  ",
	"SENSOR_FEATURE_SET_OB_LOCK,                        ",
	"SENSOR_FEATURE_SET_SENSOR_OTP_AWB_CMD,             ",
	"SENSOR_FEATURE_SET_SENSOR_OTP_LSC_CMD,             ",
	"SENSOR_FEATURE_GET_YUV_CAPTURE_OUTPUT_JPEG,        ",
	"SENSOR_FEATURE_SET_YUV_JPEG_PARA,                  ",
	"SENSOR_FEATURE_GET_YUV_JPEG_INFO,                  ",
	"SENSOR_FEATURE_SET_FRAMERATE,                      ",
	"SENSOR_FEATURE_SET_HDR,                            ",
	"SENSOR_FEATURE_GET_CROP_INFO,                      ",
	"SENSOR_FEATURE_GET_VC_INFO,                        ",
	"SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN,              ",
	"SENSOR_FEATURE_SET_AWB_GAIN,                       ",
	"SENSOR_FEATURE_SET_MIN_MAX_FPS,                    ",
	"SENSOR_FEATURE_GET_PDAF_INFO,                      ",
	"SENSOR_FEATURE_GET_PDAF_DATA,                      ",
	"SENSOR_FEATURE_SET_PDFOCUS_AREA,                   ",
	"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY,           ",
	"SENSOR_FEATURE_DEBUG_IMGSENSOR,                    ",
	"SENSOR_FEATURE_SET_HDR_SHUTTER,                    ",
	"SENSOR_FEATURE_SET_ISO,                            ",
	"SENSOR_FEATURE_SET_PDAF,                           ",
	"SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME,             ",
	"SENSOR_FEATURE_SET_SHUTTER_BUF_MODE,               ",
	"SENSOR_FEATURE_SET_GAIN_BUF_MODE,                  ",
	"SENSOR_FEATURE_SET_I2C_BUF_MODE_EN,                ",
	"SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY,            ",
	"SENSOR_FEATURE_GET_PDAF_TYPE,                      ",
	"SENSOR_FEATURE_SET_PDAF_TYPE,                      ",
	"SENSOR_FEATURE_GET_PDAF_REG_SETTING,               ",
	"SENSOR_FEATURE_SET_PDAF_REG_SETTING,               ",
	"SENSOR_FEATURE_SET_STREAMING_SUSPEND,              ",
	"SENSOR_FEATURE_SET_STREAMING_RESUME,               ",
	"SENSOR_FEATURE_OPEN,                               ",
	"SENSOR_FEATURE_CLOSE,                              ",
	"SENSOR_FEATURE_SET_DRIVER,                         ",
	"SENSOR_FEATURE_CHECK_IS_ALIVE,                     ",
	"SENSOR_FEATURE_GET_4CELL_DATA,                     ",
	"SENSOR_FEATURE_SET_WAKE_LOCK,                      ",
	"SENSOR_FEATURE_GET_MIPI_PIXEL_RATE,                ",
	"SENSOR_FEATURE_SET_HDR_ATR,                        ",
	"SENSOR_FEATURE_SET_HDR_TRI_GAIN,                   ",
	"SENSOR_FEATURE_SET_HDR_TRI_SHUTTER,                ",
	"SENSOR_FEATURE_SET_LSC_TBL,                        ",
	"SENSOR_FEATURE_GET_SENSOR_SYNC_MODE_CAPACITY,      ",
	"SENSOR_FEATURE_GET_SENSOR_SYNC_MODE,               ",
	"SENSOR_FEATURE_SET_SENSOR_SYNC_MODE,               ",
	"SENSOR_FEATURE_GET_PIXEL_RATE,                     ",
	"SENSOR_FEATURE_MAX									"
};
#endif
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =(MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

#if FPT_PDAF_SUPPORT
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
#endif

	//LOG_INF("feature_id = %d[%s]\n", feature_id, features[feature_id-SENSOR_FEATURE_START]);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_byte(sensor_reg_data->RegAddr,sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =read_cmos_sensor_byte(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
		// if EEPROM does not exist in camera module.
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)(*feature_data_16),*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM) *feature_data,*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM) * (feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)(*feature_data));
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (UINT8) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			 (UINT32) *feature_data);

		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16) *feature_data, 
                        (UINT16) * (feature_data + 1),
			(UINT16) * (feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data,
					(UINT16) * (feature_data + 1),
					(UINT16) * (feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			 (UINT16) *feature_data,
			 (UINT16) *(feature_data + 1));
		/* ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1)); */
		break;


#if FPT_PDAF_SUPPORT
		//////////////////////////////////////////////////
		/*					PDAF START					*/
		//////////////////////////////////////////////////
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioID:%lld\n ",
			*feature_data);
		PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32)
		{
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
			memcpy((void *)pvcinfo, (void *) &SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n",*feature_data);

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1; // video & capture use same setting
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:	/*get cal data from eeprom*/
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		//imgsensor.pdaf_mode = *feature_Data_16;
		break;

		/******************** PDAF END   <<< *********/
#endif

	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:/*lzl*/
		LOG_INF("SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME\n");
		set_shutter_frame_length((UINT16)*feature_data,
					 (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;

	case SENSOR_FEATURE_GET_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.cap.pclk /
				(imgsensor_info.cap.linelength - 80)) *
				 imgsensor_info.cap.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.normal_video.pclk /
				(imgsensor_info.normal_video.linelength -80)) *
				 imgsensor_info.normal_video.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.hs_video.pclk /
				(imgsensor_info.hs_video.linelength - 80)) *
				 imgsensor_info.hs_video.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.slim_video.pclk /
				(imgsensor_info.slim_video.linelength - 80)) *
				 imgsensor_info.slim_video.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.pre.pclk /
				(imgsensor_info.pre.linelength - 80)) *
				 imgsensor_info.pre.grabwindow_width;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;

	default:
		break;
	}

	return ERROR_NONE;
} /*feature_control()*/

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close,
	#ifdef TCT_CAM_DRIVER_SUPPORT
	"S5KJN1:TSP:50M:ASA5000001C1",
	0,//main:0 front:1 sub:2 
	{
		{RST, Vol_Low,  0},
		{DOVDD, Vol_1800, 0},
		{DVDD, Vol_1050, 0},
		{AVDD, Vol_2800, 0},
		{RST, Vol_High, 1},
		{SensorMCLK, Vol_High, 1},
		{AFVDD, Vol_2800, 2},
	}
	#endif
};

UINT32 TSPPSNP1269_S5KJN1_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	LOG_INF("S5KJN1_MIPI_RAW_SensorInit in\n");
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /*	S5KJN1_MIPI_RAW_SensorInit	*/


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <linux/slab.h>

//#ifndef VENDOR_EDIT
//#include "kd_camera_hw.h"
/*Caohua.Lin@Camera.Drv, 20180126 remove to adapt with mt6771*/
//#endif
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"

#include "s5k4h7mipiraw_Sensor.h"
#include "s5k4h7otp.h"


/******************Modify Following Strings for Debug*******************/
#define PFX "S5K4H7OTP"
#define LOG_1 SENSORDB("S5K4H7,MIPI CAM\n")
#define LOG_INF(format, args...) \
	pr_err(PFX "[%s] " format, __func__, ##args)
/*********************   Modify end    *********************************/

#define USHORT        unsigned short
#define BYTE          unsigned char
#define I2C_ID          0x20

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, I2C_ID);
	return get_byte;
}



static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {
		(char)(addr >> 8),
		(char)(addr & 0xFF),
		(char)(para & 0xFF) };

	iWriteRegI2C(pusendcmd, 3, I2C_ID);
}

bool wb_gain_set(kal_uint32 r_ratio, kal_uint32 b_ratio)
{
	kal_uint32 R_GAIN = 0;
	kal_uint32 B_GAIN = 0;
	kal_uint32 Gr_GAIN = 0;
	kal_uint32 Gb_GAIN = 0;
	kal_uint32 G_GAIN = 0;
	kal_uint32 GAIN_DEFAULT = 0x0100;

	if (!r_ratio || !b_ratio) {
		LOG_INF(" OTP WB ratio Data Err!");
		return 0;
	}
	if (r_ratio >= 512) {
		if (b_ratio >= 512) {
			R_GAIN = (USHORT) (GAIN_DEFAULT * r_ratio / 512);
			G_GAIN = GAIN_DEFAULT;
			B_GAIN = (USHORT) (GAIN_DEFAULT * b_ratio / 512);
		} else {
			R_GAIN = (USHORT) (GAIN_DEFAULT * r_ratio / b_ratio);
			G_GAIN = (USHORT) (GAIN_DEFAULT * 512 / b_ratio);
			B_GAIN = GAIN_DEFAULT;
		}
	} else {
		if (b_ratio >= 512) {
			R_GAIN = GAIN_DEFAULT;
			G_GAIN = (USHORT) (GAIN_DEFAULT * 512 / r_ratio);
			B_GAIN = (USHORT) (GAIN_DEFAULT * b_ratio / r_ratio);
		} else {
			Gr_GAIN = (USHORT) (GAIN_DEFAULT * 512 / r_ratio);
			Gb_GAIN = (USHORT) (GAIN_DEFAULT * 512 / b_ratio);
			if (Gr_GAIN >= Gb_GAIN) {
				R_GAIN = GAIN_DEFAULT;
				G_GAIN = (USHORT)
					(GAIN_DEFAULT * 512 / r_ratio);
				B_GAIN = (USHORT)
					(GAIN_DEFAULT * b_ratio / r_ratio);
			} else {
				R_GAIN = (USHORT)
					(GAIN_DEFAULT * r_ratio / b_ratio);
				G_GAIN = (USHORT)
					(GAIN_DEFAULT * 512 / b_ratio);
				B_GAIN = GAIN_DEFAULT;
			}
		}
	}

	write_cmos_sensor_8(0x3C0F, 0x01);
	if (R_GAIN > GAIN_DEFAULT) {
		write_cmos_sensor_8(0x0210, (R_GAIN >> 8) & 0x0F);
		write_cmos_sensor_8(0x0211, R_GAIN & 0xFF);
	}
	if (B_GAIN > GAIN_DEFAULT) {
		write_cmos_sensor_8(0x0212, (B_GAIN >> 8) & 0x0F);
		write_cmos_sensor_8(0x0213, B_GAIN & 0xFF);
	}
	if (G_GAIN > GAIN_DEFAULT) {
		write_cmos_sensor_8(0x020E, (G_GAIN >> 8) & 0x0F);
		write_cmos_sensor_8(0x020F, G_GAIN & 0xFF);
		write_cmos_sensor_8(0x0214, (G_GAIN >> 8) & 0x0F);
		write_cmos_sensor_8(0x0215, G_GAIN & 0xFF);
	}
	return 1;
}

 typedef struct s5k4h7_otp_info {
	kal_uint16  unit_rg;
	kal_uint16	unit_bg;	
	kal_uint16  golden_rg;
	kal_uint16	golden_bg;	
	kal_uint16	module_id;
	kal_uint16	lens_id;	
} s5k4h7_otp_info;

#define group1wb_start_addr 0x0a14
#define group2wb_start_addr 0x0a23
#define group1MID_start_addr 0x0a05
#define group2MID_start_addr 0x0a0C

static s5k4h7_otp_info s5k4h7wb_otp_info1 = {0};
static char wb_valid_flag = 0;
static char module_flag = 0;
static char wb_flag = 0;

static char buff[14] = {0};
static char buff1[14] = {0};
static char buff2[7] = {0};
static char buff3[7] = {0};


static void s5k4h7_read_group(unsigned int addr, char* buf, unsigned int size)
{
	int i ;
	for( i=0 ; i < size; i++){
		buf[i] = read_cmos_sensor_8(addr + i);
		//LOG_INF("s5k4h7 wb buf[i] = %d\n",i,buf[i]);
	}

}
		

bool S5K4H7_update_awb(unsigned char page)
{
	//char flag = 0;
	kal_uint32 r_ratio = 0;
	kal_uint32 b_ratio = 0;

	kal_uint32 golden_rg = 0;
	kal_uint32 golden_bg = 0;

	kal_uint32 unit_rg = 0;
	kal_uint32 unit_bg = 0;

	write_cmos_sensor_8(0x0A02, page);
	write_cmos_sensor_8(0x0A00, 0x01);

	
	//LOG_INF("s5k4h7_wb_flag = %d\n",flag);

	if (((wb_flag>>6)&0x3) == 0x1)
	{
		if(((buff[0]+buff[1]+buff[2]+buff[3]+buff[4]+buff[5])%255+1) == buff[12])
		{
			LOG_INF("s5k4h7_wb_flag grop 1\n");
			//unit_rg1 = read_cmos_sensor_8(0x0A15) | read_cmos_sensor_8(0x0A14) << 8;
			//unit_bg1 = read_cmos_sensor_8(0x0A17) | read_cmos_sensor_8(0x0A16) << 8;
			unit_rg = buff[1] | buff[0] << 8;
			unit_bg = buff[3] | buff[2] << 8;
		}
	}
	else if(((wb_flag>>4)&0x3) == 0x1)
	{
		if(((buff1[0]+buff1[1]+buff1[2]+buff1[3]+buff1[4]+buff1[5])%255+1) == buff1[12])
		{
			LOG_INF("s5k4h7_wb_flag grop 2\n");
			//unit_rg1 = read_cmos_sensor_8(0x0A24) | read_cmos_sensor_8(0x0A23) << 8;
			//unit_bg1 = read_cmos_sensor_8(0x0A26) | read_cmos_sensor_8(0x0A25) << 8;
			unit_rg = buff1[1] | buff1[0] << 8;
			unit_bg = buff1[3] | buff1[2] << 8;
		}
	}
	if (((wb_flag>>2)&0x3) == 0x1)
	{
		if(((buff[6]+buff[7]+buff[8]+buff[9]+buff[10]+buff[11])%255+1) == buff[13])
		{
			LOG_INF("s5k4h7_golden_wb_flag grop 1\n");
			//golden_rg1 = read_cmos_sensor_8(0x0A1B) | read_cmos_sensor_8(0x0A1A) << 8;
			//golden_bg1 = read_cmos_sensor_8(0x0A1D) | read_cmos_sensor_8(0x0A1C) << 8;
			golden_rg = buff[7] | buff[6] << 8;
			golden_bg = buff[9] | buff[8] << 8;
		}
	}
	else if(((wb_flag)&0x3) == 0x1)
	{	
		if(((buff1[6]+buff1[7]+buff1[8]+buff1[9]+buff1[10]+buff1[11])%255+1) == buff1[13]){
			LOG_INF("s5k4h7_golden_wb_flag grop 2\n");
			//golden_rg1 = read_cmos_sensor_8(0x0A2A) | read_cmos_sensor_8(0x0A29) << 8;
			//golden_bg1 = read_cmos_sensor_8(0x0A2c) | read_cmos_sensor_8(0x0A2B) << 8;
			golden_rg = buff1[7] | buff1[6] << 8;
			golden_bg = buff1[9] | buff1[8] << 8;
		}
	}		
	LOG_INF(
		"updata wb golden_rg=0x%x golden_bg=0x%x unit_rg=0x%x unit_bg =0x%x\n",
		 golden_rg, golden_bg, unit_rg, unit_bg);
	

	if (!golden_rg || !golden_bg || !unit_rg || !unit_bg) {
		LOG_INF("updata wb err");
		return 0;
	}
	
	s5k4h7wb_otp_info1.unit_rg = unit_rg;
	s5k4h7wb_otp_info1.unit_bg = unit_bg;
	
	s5k4h7wb_otp_info1.golden_rg = golden_rg;
	s5k4h7wb_otp_info1.golden_bg = golden_bg;
	
	r_ratio = 512 * (golden_rg) / (unit_rg);
	b_ratio = 512 * (golden_bg) / (unit_bg);
	wb_gain_set(r_ratio, b_ratio);
	return 1;
}




bool S5K4H7_otp_update(void)
{
	bool flag = 0;
	unsigned char page = 0;

	write_cmos_sensor_8(0x0136, 0x18);	// 24MHz
	write_cmos_sensor_8(0x0137, 0x00);
	write_cmos_sensor_8(0x0305, 0x06);	// PLL pre div
	write_cmos_sensor_8(0x0306, 0x00);	//PLL multiplier
	write_cmos_sensor_8(0x0307, 0x8C);

	write_cmos_sensor_8(0x030D, 0x06);	// second_pre_pll_clk_div

	write_cmos_sensor_8(0x030E, 0x00);	// second_pll_multiplier
	write_cmos_sensor_8(0x030F, 0xAF);	// second_pll_multiplier
	write_cmos_sensor_8(0x0301, 0x04);	// vt_pix_clk_div

	//Streaming ON
	write_cmos_sensor_8(0x0100, 0x01);	// Streaming ON
	mDELAY(10);

	write_cmos_sensor_8(0x0A02, 0x15);	// page 21
	write_cmos_sensor_8(0x0A00, 0x01);
	mDELAY(10);
	flag = read_cmos_sensor_8(0x0A13);
	
	if (flag) {
		page = 0x15;
		LOG_INF("flag = %d\n",flag);
	} else {
		LOG_INF("otp read page 21 failed\n");
		write_cmos_sensor_8(0x0A02, 0x17);	// page 21
		write_cmos_sensor_8(0x0A00, 0x01);
		flag = read_cmos_sensor_8(0x0A10);
		if (flag)
			page = 0x17;
		else
			LOG_INF("otp read page 23 failed\n");
	}
	if (!flag) {
		page = 0x0;
		LOG_INF("otp read failed\n");
		return 0;
	}
	
	s5k4h7_read_group(group1wb_start_addr, buff, 14);
	s5k4h7_read_group(group2wb_start_addr, buff1, 14);
	
	s5k4h7_read_group(group1MID_start_addr, buff2, 7);
	s5k4h7_read_group(group2MID_start_addr, buff3, 7);
	
	wb_flag = read_cmos_sensor_8(0x0A13);
	module_flag = read_cmos_sensor_8(0x0A04);

	LOG_INF(" line 336S5K4H7 wb_flag = 0x%x module_flag = 0x%x \n", wb_flag, module_flag);
	
	s5k4h7_get_module_id();
	wb_valid_flag = S5K4H7_update_awb(page);
	
	if((s5k4h7wb_otp_info1.module_id == 0x48)&&(s5k4h7wb_otp_info1.lens_id == 0xb4) && wb_flag)
	{
		LOG_INF("S5K4H7 module_id = 0x%x,lend_id = 0x%x, unit_rg = 0x%x, unit_bg = 0x%x, golden_rg = 0x%x, golden_bg = 0x%x\n",s5k4h7wb_otp_info1.module_id,s5k4h7wb_otp_info1.lens_id, s5k4h7wb_otp_info1.unit_rg, s5k4h7wb_otp_info1.unit_bg, s5k4h7wb_otp_info1.golden_rg, s5k4h7wb_otp_info1.golden_bg);	
		return 1;	  
	}
		
	return 0;
}

bool lsc_otp_checksum(void)
{
	unsigned int i;
	unsigned int j;
	unsigned int sum = 0;
	unsigned int lsc_flag = 0;
	unsigned int check_sum = 0;
	
	write_cmos_sensor_8(0x0136, 0x18);	// 24MHz
	write_cmos_sensor_8(0x0137, 0x00);
	write_cmos_sensor_8(0x0305, 0x06);	// PLL pre div
	write_cmos_sensor_8(0x0306, 0x00);	//PLL multiplier
	write_cmos_sensor_8(0x0307, 0x8C);

	write_cmos_sensor_8(0x030D, 0x06);	// second_pre_pll_clk_div

	write_cmos_sensor_8(0x030E, 0x00);	// second_pll_multiplier
	write_cmos_sensor_8(0x030F, 0xAF);	// second_pll_multiplier
	write_cmos_sensor_8(0x0301, 0x04);	// vt_pix_clk_div

	//Streaming ON
	write_cmos_sensor_8(0x0100, 0x01);	// Streaming ON
	mDELAY(10);
	
	write_cmos_sensor_8(0x0A02, 0x00);	// page 0
	write_cmos_sensor_8(0x0A00, 0x01);
	mDELAY(10);
	lsc_flag = read_cmos_sensor_8(0x0A3D);

	if(lsc_flag == 1)
	{
		write_cmos_sensor_8(0x0A02, 0x15);	// page 15
		write_cmos_sensor_8(0x0A00, 0x01);
		mDELAY(10);
		check_sum = read_cmos_sensor_8(0x0A22);

		LOG_INF("s5k4h7_lsc grop 1\n");
		// page 1~5 for group 1
		for(i = 1; i <= 5; i ++)
		{
			write_cmos_sensor_8(0x0A02, 0x00+i);	// page 1
			write_cmos_sensor_8(0x0A00, 0x01);
			mDELAY(10);
	
			for(j = 0;j <= 0x3f; j++)
			{
				sum += read_cmos_sensor_8(0xA04+j);
				LOG_INF("s5k4h7_lsc sum = 0x%x, addr = 0x%x\n", sum, (0xA04+j));
			}
		}
		// page 1~5 for group 1
		
		// page 6 for group 1
		write_cmos_sensor_8(0x0A02, 0x06);	// page 6
		write_cmos_sensor_8(0x0A00, 0x01);
		mDELAY(10);
		for(j = 0;j <= 0x27; j++)
		{
			sum += read_cmos_sensor_8(0xA04+j);
			LOG_INF("s5k4h7_s5k4h7_lsc sum = 0x%x, addr = 0x%x\n", sum, (0xA04+j));
		}
		// page 6 for group 1

		if((sum % 255)+1 == check_sum)
			return 1;
		else
			return 0;	
	}
	else if(lsc_flag == 3)
	{
		write_cmos_sensor_8(0x0A02, 0x15);	// page 15
		write_cmos_sensor_8(0x0A00, 0x01);
		mDELAY(10);
		check_sum = read_cmos_sensor_8(0x0A31);	
		LOG_INF("s5k4h7_lsc grop 2 check_sum = 0x%x\n",check_sum);
		
		//page 6 for gropup2
		write_cmos_sensor_8(0x0A02, 0x06);	// page 6
		write_cmos_sensor_8(0x0A00, 0x01);
		mDELAY(10);
		for(j = 0;j <= 0x17; j++)
		{
			sum += read_cmos_sensor_8(0xA2C+j);
			LOG_INF("s5k4h7_lsc sum = 0x%x, addr = 0x%x\n", sum, (0xA2C+j));
		}
		//page 6 for gropup2
		
		//page 7~11 for gropup2
		for(i = 1; i <= 5; i ++)
		{
			write_cmos_sensor_8(0x0A02, 0x06+i);	// page 7
			write_cmos_sensor_8(0x0A00, 0x01);
			mDELAY(10);
			LOG_INF("s5k4h7_lsc page = 0x%x\n",(0x06+i));
			for(j = 0;j <= 0x3f; j++)
			{
				sum += read_cmos_sensor_8(0xA04+j);
				LOG_INF("s5k4h7_lsc sum = 0x%x, addr = 0x%x\n", sum, (0xA04+j));
			}
		}
		//page 7~11 for gropup2

		//page 12 for gropup2
		write_cmos_sensor_8(0x0A02, 0x0c);	// page 12
		write_cmos_sensor_8(0x0A00, 0x01);
		mDELAY(10);
		for(j = 0;j <= 0xf; j++)
		{
			sum += read_cmos_sensor_8(0xA04+j);
			LOG_INF("s5k4h7_lsc  sum = 0x%x, addr = 0x%x\n", sum, (0xA04+j));
		}
		//page 12 for gropup2

		if((sum % 255)+1 == check_sum)
			return 1;
		else
			return 0;
	}
	else
	{
		return 0;
	}
}

unsigned char s5k4h7_get_module_id(void)
{
	//unsigned char module_id = 0;
	/*
	write_cmos_sensor_8(0x0136, 0x18);	// 24MHz
	write_cmos_sensor_8(0x0137, 0x00);
	write_cmos_sensor_8(0x0305, 0x06);	// PLL pre div
	write_cmos_sensor_8(0x0306, 0x00);	//PLL multiplier
	write_cmos_sensor_8(0x0307, 0x8C);

	write_cmos_sensor_8(0x030D, 0x06);	// second_pre_pll_clk_div

	write_cmos_sensor_8(0x030E, 0x00);	// second_pll_multiplier
	write_cmos_sensor_8(0x030F, 0xAF);	// second_pll_multiplier
	write_cmos_sensor_8(0x0301, 0x04);	// vt_pix_clk_div

	//Streaming ON
	write_cmos_sensor_8(0x0100, 0x01);	// Streaming ON
	mDELAY(10);

	write_cmos_sensor_8(0x0A02, 0x15);	// page 21
	write_cmos_sensor_8(0x0A00, 0x01);
	mDELAY(10);

	module_id = module_flag;
*/
	if (((module_flag>>6)&0x3) == 0x1){
	
		if((buff2[0]+buff2[1]+buff2[2]+buff2[3]+buff2[4]+buff2[5])%255+1 == buff2[6])
		{
			
			s5k4h7wb_otp_info1.module_id = buff2[0];
			s5k4h7wb_otp_info1.lens_id = buff2[1];

			LOG_INF("S5K4H7 ropu 1 module id = 0x%x lens id = 0x%x \n", s5k4h7wb_otp_info1.module_id, s5k4h7wb_otp_info1.lens_id);
		}
	}
	else if(((module_flag>>4)&0x3) == 0x1){
		if((buff3[0]+buff3[1]+buff3[2]+buff3[3]+buff3[4]+buff3[5])%255+1 == buff3[6])
		{
			s5k4h7wb_otp_info1.module_id = buff3[0];
			s5k4h7wb_otp_info1.lens_id = buff3[1];

			LOG_INF("S5K4H7 gropu 2 module id = 0x%x lens id = 0x%x \n", s5k4h7wb_otp_info1.module_id, s5k4h7wb_otp_info1.lens_id);
		}
	}
	
	return s5k4h7wb_otp_info1.module_id;
}

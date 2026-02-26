/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _MT6833_AFE_GPIO_H_
#define _MT6833_AFE_GPIO_H_

enum mt6833_afe_gpio {
	MT6833_AFE_GPIO_DAT_MISO0_OFF,
	MT6833_AFE_GPIO_DAT_MISO0_ON,
	MT6833_AFE_GPIO_DAT_MISO1_OFF,
	MT6833_AFE_GPIO_DAT_MISO1_ON,
	MT6833_AFE_GPIO_DAT_MISO2_OFF,
	MT6833_AFE_GPIO_DAT_MISO2_ON,
	MT6833_AFE_GPIO_DAT_MOSI_OFF,
	MT6833_AFE_GPIO_DAT_MOSI_ON,
	MT6833_AFE_GPIO_I2S0_OFF,
	MT6833_AFE_GPIO_I2S0_ON,
	MT6833_AFE_GPIO_I2S1_OFF,
	MT6833_AFE_GPIO_I2S1_ON,
	MT6833_AFE_GPIO_I2S2_OFF,
	MT6833_AFE_GPIO_I2S2_ON,
	MT6833_AFE_GPIO_I2S3_OFF,
	MT6833_AFE_GPIO_I2S3_ON,
	MT6833_AFE_GPIO_I2S5_OFF,
	MT6833_AFE_GPIO_I2S5_ON,
	MT6833_AFE_GPIO_VOW_DAT_OFF,
	MT6833_AFE_GPIO_VOW_DAT_ON,
	MT6833_AFE_GPIO_VOW_CLK_OFF,
	MT6833_AFE_GPIO_VOW_CLK_ON,
//Begin added by lanying.he for XR11082897 on 2021/05/06
	GPIO_EXTAMP_HIGH,
	GPIO_EXTAMP_LOW,
	GPIO_EXTAMP2_HIGH,
	GPIO_EXTAMP2_LOW,
	GPIO_RCVSPK_HIGH,
	GPIO_RCVSPK_LOW,
	GPIO_HAC_HIGH,
	GPIO_HAC_LOW,
//End added by lanying.he for XR11082897 on 2021/05/06	
	MT6833_AFE_GPIO_GPIO_NUM
};

struct mtk_base_afe;

int mt6833_afe_gpio_init(struct mtk_base_afe *afe);

int mt6833_afe_gpio_request(struct mtk_base_afe *afe, bool enable,
			    int dai, int uplink);

bool mt6833_afe_gpio_is_prepare(enum mt6833_afe_gpio type);

//Begin added by lanying.he for XR11082897 on 2021/05/06
int AudDrv_GPIO_EXTAMP_Select(int bEnable, int mode);
int AudDrv_GPIO_EXTAMP2_Select(int bEnable, int mode);
int AudDrv_GPIO_RCVSPK_Select(int bEnable, int mode);
int AudDrv_GPIO_HAC_Select(int bEnable, int mode);
//End added by lanying.he for XR XR11082897 2021/05/06


#endif

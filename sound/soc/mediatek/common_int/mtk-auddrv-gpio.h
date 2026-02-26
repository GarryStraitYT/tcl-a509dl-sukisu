/* SPDX-License-Identifier: GPL-2.0 */


#ifndef _AUDDRV_GPIO_H_
#define _AUDDRV_GPIO_H_


#include "mtk-auddrv-def.h"
#include "mtk-soc-digital-type.h"



#include <linux/gpio.h>

void AudDrv_GPIO_probe(void *dev);

int AudDrv_GPIO_Request(bool _enable, enum soc_aud_digital_block _usage);

int AudDrv_GPIO_SMARTPA_Select(int mode);
int AudDrv_GPIO_TDM_Select(int mode);

int AudDrv_GPIO_I2S_Select(int bEnable);

//Begin added by liangjiaqiang for CIVIC-3069 on 2022-07-13
#ifdef CONFIG_EXTPA_AW8737
void down_speaker_gpio_extamp_select(int bEnable);
#endif

#ifdef CONFIG_EXTPA_OCA72317
void up_speaker_gpio_extamp_select(int bEnable);
void up_voice_gpio_extamp_select(int bEnable);
#endif

//End added by liangjiaqiang for CIVIC-3069 on 2022-07-13

int AudDrv_GPIO_EXTAMP_Select(int bEnable, int mode);
int AudDrv_GPIO_EXTAMP2_Select(int bEnable, int mode);
int AudDrv_GPIO_RCVSPK_Select(int bEnable);
int AudDrv_GPIO_HPDEPOP_Select(int bEnable);

int audio_drv_gpio_aud_clk_pull(bool high);
/* Begin meng.zhang HAC control for task 10014578 on 2020/09/28 */
int AudDrv_GPIO_HAC_PA_Select(int bEnable);
/* End meng.zhang HAC control for task 10014578 on 2020/09/28 */
#endif

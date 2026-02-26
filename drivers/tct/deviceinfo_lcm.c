/*
 *  FOR tct lcm device info
 *  History 1.0: Add by wenhaodeng@tcl.com, for task LOGANS-2237 on 20220811	 
 * 
 */
#ifdef CONFIG_MTK_LCM
#include <linux/init.h>
#include <linux/module.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include <linux/uaccess.h>

extern char LCM_module_name[256];
extern char *mtkfb_find_lcm_driver(void);

int tct_use_lcm_devinfo_list = 0;
EXPORT_SYMBOL(tct_use_lcm_devinfo_list);

static u8 *tct_lcd_info_list[][2] = {
//bora start
#ifdef CONFIG_TCT_PROJECT_BORATF
	{"bora_ft8006s_coe_hd_vdo",	"FT8006S:COE:720x1440:AUC0600141C1"},
	{"bora_gc7202h_ykl_hd_vdo",	"GC7202H:YKL:720x1440:AUC0600142C1"},
	{"bora_nl9911c_tdt_hd_vdo",	"NL9911C:TDT:720x1440:NA"},
#endif
//bora end
//sonata start
#ifdef CONFIG_TCT_PROJECT_SONATA
	{"sonata_icnl9916_hdp_dsi_vdo",	"NL9916:Truly:720*1600:AUC0670116C1"},
	{"sonata_ft8057_hdp_dsi_vdo",	"FT8057:LCE:720*1600:AUC0670117C1"},
#endif
//sonata end

//encore start
#ifdef CONFIG_TCT_PROJECT_ENCORE
	{"panel-encore-nl9916-truly-truly-hdplus1612-dsi-vdo",	"ICNL9916:TRULY:1st:AUC0660117C1"},
#endif
//encore end

//Begin added by liangjiaqiang for MODEL3-1949 on 2022-09-22
#ifdef CONFIG_TCT_PROJECT_MODEL_3
	{"icnl9916_hdplus1600_dsi_vdo_hx_xe671",	"ICNL9916:INX:720*1600:178263620"},
	{"ft8057_hdplus1600_dsi_vdo_gx_xe671",	"FT8057:GXGD:720*1600:178264213"},
#endif
//End added by liangjiaqiang for MODEL3-1949 on 2022-09-22

//Logan start	
#ifdef CONFIG_TCT_PROJECT_PASSAT
	{"passat_ili7835_hehui_edo_fhdplus_amoled_cmd",	"ILI7835:EDO:1080*2400:AUC0670114C1"},
#else
	{"logan_ili9881c_kd_auo_hd_dsi_vdo",	"ILI9881C:KINGDISPLAY-AUO:800*1280:135006005001"},
#endif  

//Luna84gvzw start
#ifdef CONFIG_TCT_PROJECT_LUNA84GVZW
       {"luna_er88577b_xy_hd_vdo",  "ER88577B:XY:800*1280:AUA0800131C1"},//pbw
       {"luna_er88577_zs_hd_vdo",  "ER88577B:XY:800*1280:AUA0800131C1"},//pbw
#endif	

	{NULL,NULL},//Don't add value after this line
};


static int tct_set_lcm_module_name_init(void)
{
	int tct_lcd_index=0;
	
	if(tct_use_lcm_devinfo_list == 1){
		while(tct_lcd_info_list[tct_lcd_index][0] != NULL) {
			if(strcmp(mtkfb_find_lcm_driver(), tct_lcd_info_list[tct_lcd_index][0])==0){
				sprintf(LCM_module_name, tct_lcd_info_list[tct_lcd_index][1]);
				printk("%s, set devinfo lcm  = %s\n",__func__,mtkfb_find_lcm_driver());
                          	break;
			}
			tct_lcd_index++;
		}
		
		if(tct_lcd_info_list[tct_lcd_index][0] == NULL) {
			sprintf(LCM_module_name, "NA:NA:NA:720*1600");
		}
	}
    return 0;
}

static void set_lcm_module_name_exit(void)
{
      return;
}

late_initcall(tct_set_lcm_module_name_init);
module_exit(set_lcm_module_name_exit);
MODULE_LICENSE("GPL");
#endif

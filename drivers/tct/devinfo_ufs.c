/*
   Add a new file.c by dingting.meng on 2019/9/30,for task-8325724,to add dev_info_emmc
   Update by dingting.meng for T-9869946 on 3/9/2020
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#define TCT_BOOT_DEV_MAX    256
#define TCT_BOOT_DEV_ERR    (-1)
#define TCT_BOOT_DEV_EMMC   1
#define TCT_BOOT_DEV_UFS    2
#define QCOM_BOOT_EMMC      "sdhci"
#define QCOM_BOOT_UFS       "ufshc"

extern void get_dev_info_emmc(char *pdest);
#ifndef CONFIG_TCT_DEVICEINFO_EMMC
extern void get_dev_info_ufs(char *pdest);
#endif
extern char eMMC_module_name[256];

static char tct_boot_dev[TCT_BOOT_DEV_MAX] = {'\0'};

static u8 *emmc_spt_tab[][5] =
{
        {"150100525036344D42","KMRP60014M-B614","Samsung","64G+4G","AMD0000635C1"},
        {"8F0100555936435330","UNPVN6G5CACA4CS","UNIC","64G+4G","AMD0000663C1"},
        {"13014E47314A395238","MT29VZZZAD8GQFSL_046W_9R8","Micron","64G+4G","AMD0000700C1"},
        {"150100444836444142", "KMDH6001DA_B422", "Samsung", "64G+4G", "AMD0000607C1"},
        {NULL,NULL,NULL,NULL,NULL},
};

static u8 *ufs_spt_tab[][5] =
{
        {"KM5L9001DM-B424","KM5L9001DM-B424","Samsung","128G+4G","AMD0000773C1"}, //challenger TMO1st
        {"KM5V8001DM-B622","KM5V8001DM-B622","Samsung","128G+4G","AMD0000717C1"}, //challenger TMO 3ND
        {"MT128GAXAU2U228C","MT29VZZZCD91SFSM_046W_18C","Mircon","128G+4G","AMD0000812C1"}, //challenger TMO 2ND new
        {"MT128GASAO4U21","MT29VZZZAD9FQFSM_046W_G9K","Micron","128G+4G","AMD0000779C1"}, //challenger TMO 2ND old
        {"KM5P9001DM-B424","KM5P9001DM-B424","Samsung","64G+4G","AMD0000811C1"},//buffaloboost 3th
        {"MT064GASAO2U21","MT064GASAO2U21","MICRON","64G+4G","AMD0000748C1"},//MT29VZZZAD8FQFSL-046W.G8K buffaloboost 2nd
        {"KM5P8001DM-B424","KM5P8001DM-B424","Samsung","64G+4G","AMD0000754C1"},//buffaloboost 1st
        {NULL,NULL,NULL,NULL,NULL},
};

static int __init tct_get_boot_dev(char *str)
{
    strlcpy(tct_boot_dev, str, TCT_BOOT_DEV_MAX);
    return 1;
}

__setup("androidboot.boot_devices=",tct_get_boot_dev);

static int check_boot_dev(const char *str)
{
    printk("#### %s | %d %s ####\n",__func__,__LINE__,str);

    if(strnstr(str,QCOM_BOOT_EMMC,strlen(str)))
        return TCT_BOOT_DEV_EMMC;

    if(strnstr(str,QCOM_BOOT_UFS,strlen(str)))
        return TCT_BOOT_DEV_UFS;

    return TCT_BOOT_DEV_ERR;
}

static void remap_emmc_dev_info(char *src,char *dst)
{
    int index = 0;

    while(emmc_spt_tab[index][0] != NULL)
    {
        if(!strncmp(emmc_spt_tab[index][0],src,18))
        {
            sprintf(dst, "%s:%s:%s:%s\n",emmc_spt_tab[index][1],emmc_spt_tab[index][2],emmc_spt_tab[index][3],emmc_spt_tab[index][4]);
            
            return;
        }

        index++;
    }

    sprintf(dst, "%s:Unknown:Unknown:Unknown:Unknown\n",src);

    return;
}

static void remap_ufs_dev_info(char *src,char *dst)
{
    int index = 0;

    while(ufs_spt_tab[index][0] != NULL)
    {
        if(!strncmp(ufs_spt_tab[index][0],src,10))
        {
            printk("tct fdsafd\n");
            sprintf(dst, "%s:%s:%s:%s",ufs_spt_tab[index][1],ufs_spt_tab[index][2],ufs_spt_tab[index][3],ufs_spt_tab[index][4]);
            return;
        }

        index++;
    }

    sprintf(dst, "%s:Unknown:Unknown:Unknown:Unknown\n",src);

    return;
}

static int get_boot_dev_init(void)
{
    int ret = -1;

    char tmp_buf[64] = {'\0'};

    ret = check_boot_dev(tct_boot_dev); 

    if(ret == TCT_BOOT_DEV_ERR)
    {
        printk("#### The Boot Dev Is Neither EMMC Nor UFS %s | %d ####\n",__func__,__LINE__);
        
        return -ENODEV;
    }

    if(ret == TCT_BOOT_DEV_EMMC)
    {
        printk("#### The Boot Dev Is EMMC: %s | %d ####\n",__func__,__LINE__);

        get_dev_info_emmc(tmp_buf);
        printk("#### %s | %d %s ####\n",__func__,__LINE__,tmp_buf);
    
        remap_emmc_dev_info(tmp_buf,eMMC_module_name);

        return 0;
    }

    if(ret == TCT_BOOT_DEV_UFS)
    {
        printk("#### The Boot Dev Is UFS: %s | %d ####\n",__func__,__LINE__);

#ifndef CONFIG_TCT_DEVICEINFO_EMMC
        get_dev_info_ufs(tmp_buf);
        printk("#### %s | %d %s ####\n",__func__,__LINE__,tmp_buf);
        remap_ufs_dev_info(tmp_buf,eMMC_module_name);
#endif
        return 0;
    }

    return 0;
}

static void get_boot_dev_exit(void)
{
    ;
}

late_initcall(get_boot_dev_init);
module_exit(get_boot_dev_exit);
MODULE_LICENSE("GPL");

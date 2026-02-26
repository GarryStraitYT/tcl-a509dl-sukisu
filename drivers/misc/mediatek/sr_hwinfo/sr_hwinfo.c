#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>

#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <asm/io.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/gpio.h> 
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <mt-plat/mtk_boot_common.h>

struct SR_memory_table 
{
	char emmc_name[32];
	char emmc_id_str[32];
};

static struct SR_memory_table sr_memory_table[] =
{
	{"BWCTARV11X64G",   "f40122415256313158"},
	{"EMMC32G-TJ30",    "700100544a33303332"},
	{"UNPVN6G5CACA4CS", "8f0100555936435330"},
	{"FEMDNN064G-A3A56","d60103413341353631"},
	{"KMDX60018M_B425", "150100445836384d42"},
	{"SDADA4DR_64G",    "450100444134303634"},
	{"BWCTARJ11X32G",   "f4012241524a313158"},
	{"KMDV6001DM_B620 ","150100445636444d42"},
	{"BWCTARV11X64G-BA","f40122425256313158"},
	{"BWCTARJ11X32G-BA","f4012242524a313158"},
};


static char sr_emmc_name[32] ;	
static char sr_emmc_id_str[32] ;
void sr_add_emmc_id(unsigned int cid0,unsigned int cid1,unsigned int cid2,unsigned int cid3)
{
	unsigned int emmc_raw_cid[4] ;
	char buf[12];	
	int i;
	emmc_raw_cid[0] = cid0;
	emmc_raw_cid[1] = cid1;
	emmc_raw_cid[2] = cid2;
	emmc_raw_cid[3] = cid3;	
	buf[0] = (cid0 >> 24) & 0xFF; /* Manufacturer ID */
	buf[1] = (cid0 >> 16) & 0xFF; /* Reserved(6)+Card/BGA(2) */
	buf[2] = (cid0 >> 8 ) & 0xFF; /* OEM/Application ID */
	buf[3] = (cid0 >> 0 ) & 0xFF; /* Product name [0] */
	buf[4] = (cid1 >> 24) & 0xFF; /* Product name [1] */
	buf[5] = (cid1 >> 16) & 0xFF; /* Product name [2] */
	buf[6] = (cid1 >> 8 ) & 0xFF; /* Product name [3] */
	buf[7] = (cid1 >> 0 ) & 0xFF; /* Product name [4] */
	buf[8] = (cid2 >> 24) & 0xFF; /* Product name [5] */
	buf[9] = 0;
	sprintf(sr_emmc_id_str, "%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", 
				buf[0], buf[1],buf[2],buf[3],buf[4], buf[5],buf[6],buf[7],buf[8]);

	printk("emmc cid=%08x%08x%08x%08x\n", cid0, cid1,cid2,cid3);
	printk("sr_emmc_id_str =%s\n", sr_emmc_id_str); 

	memcpy(sr_emmc_name,sr_emmc_id_str, 31);
	for(i = 0; i< sizeof(sr_memory_table) / sizeof(struct SR_memory_table); i++)
	{
		//printk("i=%d %s, %s\n",i,sr_memory_table[i].emmc_id_str,sr_emmc_id_str);
		if(0 == strncmp(sr_memory_table[i].emmc_id_str, sr_emmc_id_str, 18))
		{
			memcpy(sr_emmc_name, sr_memory_table[i].emmc_name, 31);
			break;
		}
	}
	printk("sr_emmc_name =%s\n", sr_emmc_name); 
  
}

//LCM
static char lcd_ic_name[32]=  {"NULL"};		
void sr_add_lcd_ic_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(lcd_ic_name, name);
	} 
}

static char lcd_vendor_name[32]=  {"NULL"};		
void sr_add_lcd_vendor_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(lcd_vendor_name, name);
	} 
}

static char lcd_size_name[32]=  {"NULL"};		
void sr_add_lcd_size_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(lcd_size_name, name);
	} 
}

//Touch
//int BC2_TP_used = 0;
unsigned int g_touch_firmware;
static char touch_ic_name[32]=  {"NULL"};		
void sr_add_touch_ic_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(touch_ic_name, name);
	} 
}

static char touch_vendor_name[32]=  {"NULL"};		
void sr_add_touch_vendor_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(touch_vendor_name, name);
	} 
}


//gsensor
static char gsensor_ic_name[32]=  {"NULL"};		
void sr_add_gsensor_ic_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(gsensor_ic_name, name);
	} 
}


//alsps
static char alsps_ic_name[32]=  {"NULL"};		
void sr_add_alsps_ic_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(alsps_ic_name, name);
	} 
}


//finger
static char g_finger_ic_name[32]=  {"NULL"};
void sr_add_finger_ic_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(g_finger_ic_name, name);
	} 
}

static char finger_vendor_name[32]=  {"NULL"};		
void sr_add_finger_vendor_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(finger_vendor_name, name);
	} 
}


//nfc
static char g_nfc_ic_name[32]=  {"NULL"};
void sr_add_nfc_ic_name(char *name)
{
	if(strlen(name) < 31)
	{
		strcpy(g_nfc_ic_name, name);
	} 
}


//camera_main
static char camera_main_ic_name[64]=  {"NULL"};		
void sr_add_camera_main_ic_name(char *name)
{
	if(strlen(name) < 64)
	{
		strcpy(camera_main_ic_name, name);
	} 
}

//camera_main2
static char camera_depth_ic_name[64]=  {"NULL"};		
void sr_add_camera_depth_ic_name(char *name)
{
	if(strlen(name) < 64)
	{
		strcpy(camera_depth_ic_name, name);
	} 
}
	

//camera_sub
static char camera_sub_ic_name[64]=  {"NULL"};		
void sr_add_camera_sub_ic_name(char *name)
{
	if(strlen(name) < 64)
	{
		strcpy(camera_sub_ic_name, name);
	} 
}


static ssize_t hw_info_write(struct file *file_p, const char __user *buf, size_t size, loff_t *loft)
{
	return size;
}

static unsigned int with_finger_support;
static unsigned int with_nfc_support;
static int hw_info_show(struct seq_file *seq, void *v)
{
	seq_printf(seq,	"Camera_Main: %s\n", camera_main_ic_name);

	seq_printf(seq,	"Camera_Depth: %s\n", camera_depth_ic_name);

	seq_printf(seq,	"Camera_Sub: %s\n", camera_sub_ic_name);

	seq_printf(seq, "Gsensor_IC: %s\n", gsensor_ic_name);
	
	seq_printf(seq, "ALSPS_IC: %s\n", alsps_ic_name);

	if (1 == with_nfc_support) {
		seq_printf(seq, "NFC_IC: %s\n", g_nfc_ic_name);
	}

	if (1 == with_finger_support) {
		seq_printf(seq, "Finger_IC: %s\n", g_finger_ic_name);
		seq_printf(seq, "Finger_Vendor: %s\n", finger_vendor_name);
	}

        seq_printf(seq, "LCM_IC: %s\n", lcd_ic_name);

        seq_printf(seq, "LCM_Vendor: %s\n", lcd_vendor_name);

        seq_printf(seq, "LCM_Inch: %s\n", lcd_size_name);

        seq_printf(seq, "Touch_IC: %s\n", touch_ic_name);

	seq_printf(seq, "Touch_Vendor: %s\n", touch_vendor_name);
	
	seq_printf(seq, "Touch_Firmware(0x): %04x\n", g_touch_firmware); 
	  
	seq_printf(seq, "Current_EMMC: %s\n", sr_emmc_name); 
 
	return 0;

}

static int hw_info_read(struct inode* i_node, struct file* file_p)
{
	single_open(file_p, &hw_info_show, NULL);
	return 0;
}

static const struct file_operations hw_info_ops = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.write = hw_info_write,
	.open = hw_info_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static ssize_t tp_info_write(struct file *file_p, const char __user *buf, size_t size, loff_t *loft)
{
	return size;
}

static int tp_info_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "0x%04x\n", g_touch_firmware); 
	return 0;

}

static int tp_info_read(struct inode* i_node, struct file* file_p)
{
	single_open(file_p, &tp_info_show, NULL);
	return 0;
}

static const struct file_operations tp_info_ops = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.write = tp_info_write,
	.open = tp_info_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static ssize_t finger_info_write(struct file *file_p, const char __user *buf, size_t size, loff_t *loft)
{
	return size;
}

static int finger_info_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "%s\n", g_finger_ic_name);
	return 0;
}

static int finger_info_read(struct inode* i_node, struct file* file_p)
{
	single_open(file_p, &finger_info_show, NULL);
	return 0;
}

static const struct file_operations finger_info_ops = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.write = finger_info_write,
	.open = finger_info_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init hw_info_init(void)
{
	char *ptr;
	struct proc_dir_entry *hw_info_dir = proc_mkdir("hw_info",NULL);
	proc_create("hw_info",0644,hw_info_dir,&hw_info_ops);
	proc_create("tp_ver",0644,hw_info_dir,&tp_info_ops);
	proc_create("finger",0644,hw_info_dir,&finger_info_ops);
	printk(KERN_ERR "liuyu--1");
	ptr = strstr(saved_command_line, "androidboot.with_nfc=");
	if (ptr) {
		ptr += strlen("androidboot.with_nfc=");
		with_nfc_support = simple_strtol(ptr, NULL, strlen("androidboot.with_nfc="));
		printk("hwinfo with_nfc_support=%d\n", with_nfc_support);
	} else {
		with_nfc_support = 0;
		printk("hwinfo cannot find \"androidboot.with_nfc=\"\n");
	}

	ptr = strstr(saved_command_line, "androidboot.with_finger=");
	if (ptr) {
		ptr += strlen("androidboot.with_finger=");
		with_finger_support = simple_strtol(ptr, NULL, strlen("androidboot.with_finger="));
		printk("hwinfo with_finger_support=%d\n", with_finger_support);
	} else {
		with_finger_support = 0;
		printk("hwinfo cannot find \"androidboot.with_finger=\"\n");
	}
	
	return 0;
}

static void __exit hw_info_exit(void)
{
	return;
}

module_init(hw_info_init);
module_exit(hw_info_exit);

MODULE_LICENSE("GPL"); 

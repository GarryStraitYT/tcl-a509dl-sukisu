#ifndef __HALL_H
#define __HALL_H


struct hall_data {
	int irq;
	int irq_gpio;

	struct workqueue_struct *hall_wq;
	struct work_struct hall_work;
	struct platform_device	*pdev;
	int power_enabled;
};

#endif



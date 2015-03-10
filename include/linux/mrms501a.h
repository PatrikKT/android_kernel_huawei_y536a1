#ifndef __MRMS501A_H_
#define __MRMS501A_H_


#include <linux/wakelock.h>
struct mrms501a_data_struct {
    //int    (*gpio_config) (int mode);
	struct input_dev *hw_input_hall;
    int    irq_gpio;

    struct wake_lock mrus_wake_lock;
    struct workqueue_struct  *mrms501a_wq;
    struct work_struct work;
    struct timer_list  timer_detect;//timer to schdule the task to send uevent;
    struct device    *dev;
};

#endif

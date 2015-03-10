/*
 *
 * Copyright (C) 2013 HUAWEI, Inc.
 *File Name: kernel/drivers/misc/ak8789.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/wakelock.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <asm/atomic.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

#include <misc/app_info.h>

/*prevent shake time is AKM8789_TIMER_DEBOUNCE*/
/*AKM8789_WAKEUP_TIME is for wake_lock_timeout()*/
#define AKM8789_TIMER_DEBOUNCE  20
#define AKM8789_WAKEUP_TIME  100

/*the gpio defined in dtsi and you can check the gpio configulation in huawei_mate2_va/huawei_sensors.dtsi"*/
#define GPIO_CONFIG_RIGHT_NORTH "huawei,hall_gpio_config_rn"
#define GPIO_CONFIG_RIGHT_SOUTH "huawei,hall_gpio_config_rs"
#define GPIO_CONFIG_LEFT_NORTH "huawei,hall_gpio_config_ln"
#define GPIO_CONFIG_LEFT_SOUTH "huawei,hall_gpio_config_ls"

/*gpio name*/
#define HALL_RN_INTERRUPT "hall_gpio_config_rn"
#define HALL_RS_INTERRUPT "hall_gpio_config_rs"
#define HALL_LN_INTERRUPT "hall_gpio_config_ln"
#define HALL_LS_INTERRUPT "hall_gpio_config_ls"

/*wake up flag*/
#define WAKE_FLAG_RIGHT_NORTH IRQF_NO_SUSPEND
#define WAKE_FLAG_RIGHT_SOUTH IRQF_TRIGGER_NONE
#define WAKE_FLAG_LEFT_NORTH IRQF_NO_SUSPEND
#define WAKE_FLAG_LEFT_SOUTH IRQF_TRIGGER_NONE

/*hall value*/
#define HALL_VALUE_LEFT_NORTH	(1<<0)
#define HALL_VALUE_LEFT_SOUTH	(1<<2)
#define HALL_VALUE_RIGHT_NORTH	(1<<1)
#define HALL_VALUE_RIGHT_SOUTH	(1<<3)

/*the level to print log of ak8789, default level is just to print info log*/
#define AK8789_LOG_FLOW 4
#define AK8789_LOG_INFO 3
#define AK8789_LOG_ERR 2
int akm8789_debug_mask = AK8789_LOG_INFO;
module_param_named(akm8789_debug_mask, akm8789_debug_mask, int, 0664);

#define AK8789_FLOWMSG(format, args...)\
do{\
	if( akm8789_debug_mask >= AK8789_LOG_FLOW )\
	{\
		printk(KERN_ERR "[%s] (line: %u) " format "\n",__FUNCTION__, __LINE__, ##args);\
	}\
} while(0)

#define AK8789_INFOMSG(format, args...)\
do{\
	if( akm8789_debug_mask >= AK8789_LOG_INFO )\
	{\
		printk(KERN_ERR "[%s] (line: %u) " format "\n",__FUNCTION__, __LINE__, ##args);\
	}\
} while(0)

#define AK8789_ERRMSG(format, args...)\
do{\
	if( akm8789_debug_mask >= AK8789_LOG_ERR)\
	{\
		printk(KERN_ERR "[%s] (line: %u) " format "\n",__FUNCTION__, __LINE__, ##args);\
	}\
} while(0)

typedef struct gpio_struct{
	int gpio;
	/*the flag of wake up present that the pole can be or not be waked up*/
	/*can: IRQF_NO_SUSPEND, can not: IRQF_TRIGGER_NONE */
	unsigned long wake_up;
	char *name;
	int hall_value;/*hall value*/
}gpio_data_t;

/*support four type 2 4,the number presents how many poles the mobile has*/
/*mate2 has two hall device ,four poles*/
typedef enum hall_used_type{
	TWO_POLE = 2,
	FOUR_POLE = 4,
} hall_used_type_t;

struct hall_dev {
	struct input_dev *hw_input_hall;
	struct platform_driver hall_drv_pf;
	struct workqueue_struct *hall_wq;
	struct work_struct hall_work;
	struct timer_list hall_timer;
	hall_used_type_t used_type;
	gpio_data_t* gpio_data;
};

static struct wake_lock hall_wk;
static int hall_enable_status = 0;
/*irq_no_at is a counter of hall interrupts and the initial value is 0*/
static atomic_t irq_no_at = ATOMIC_INIT(0);

void hall_work_func(struct work_struct *work);
static int hall_pf_probe(struct platform_device *pdev);
static irqreturn_t hall_event_isr(int irq, void *dev);

static struct of_device_id ak8789_match_table[] = {
	{	.compatible = "huawei,hall-ak8789",
	},
};

static struct hall_dev hw_hall_dev = {
	.hw_input_hall = NULL,
	.hall_drv_pf = {
		.probe = hall_pf_probe,
		.driver = {
			.name = "hall_platform",
			.owner = THIS_MODULE,
			.of_match_table = ak8789_match_table,
		},
	},
	.gpio_data = NULL,
};

/*when gpio low, the interrupt trigged, set bit as 1*/
#define GROUP_VALUE(GPIO_NUM, GPIO_VALUE)\
	do{\
		ret = gpio_get_value(GPIO_NUM);\
		if (!ret)\
			value |= (GPIO_VALUE);\
		else\
			value &= (~GPIO_VALUE);\
	}while(0)

/***************************************************************
Function: query_hall_event
Description: request the state of hall gpios,if four gpios state are low-low-high-high,than the value will be 1100
Parameters:void
Return:value of state of hall gpios
***************************************************************/
int query_hall_event(void)
{
	int value = 0;
	int ret = 0;
	int i = 0;

	gpio_data_t *gpio_ptr = hw_hall_dev.gpio_data;

	for ( i = 0; i < hw_hall_dev.used_type; i++){
		GROUP_VALUE(gpio_ptr->gpio, gpio_ptr->hall_value);
		AK8789_FLOWMSG("gpio_ptr->gpio=%d,gpio_ptr->hall_value=0x%x,value=0x%x",gpio_ptr->gpio,gpio_ptr->hall_value,value);
		gpio_ptr++;
	}

	return value;
}

/***************************************************************
Function: ak8987_store_enable_hall_sensor
Description: set enable flags to enable or diable ak8987,you can change the value at
                  /sys/devices/huawei_hall_sensor.4/enable_hall_sensor
Parameters:void
Return:value of state of hall gpios
***************************************************************/
static ssize_t ak8987_store_enable_hall_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int value = 0;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	AK8789_FLOWMSG("enable value %lu \n", val);

	if (val == 1){
		hall_enable_status = 1;
		value = query_hall_event();
		input_event(hw_hall_dev.hw_input_hall, EV_MSC, MSC_SCAN, value);
		input_sync(hw_hall_dev.hw_input_hall);
	}else if (val == 0){
		hall_enable_status = 0;
	}else{
		AK8789_ERRMSG("enable value error\n");
		return count;
	}

	return count;
}

static ssize_t ak8987_show_enable_hall_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", hall_enable_status);
}
/*change the permissions of sys devices of hall*/
static DEVICE_ATTR(enable_hall_sensor, S_IWUSR|S_IRUSR|S_IRUGO, ak8987_show_enable_hall_sensor, ak8987_store_enable_hall_sensor);

static ssize_t ak8987_show_irq_count(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&irq_no_at));
}
/*you can get the value at /sys/devices/huawei_hall_sensor.4/irq_count*/
static DEVICE_ATTR(irq_count, S_IWUSR|S_IRUSR|S_IRUGO, ak8987_show_irq_count, NULL);

static ssize_t ak8987_show_get_hall_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int value = 0;
	value = query_hall_event();
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}
/*/sys/devices/huawei_hall_sensor.4/get_hall_status,it shows the state of gpios,see the query_hall_event function*/
static DEVICE_ATTR(get_hall_status, S_IWUSR|S_IRUSR|S_IRUGO, ak8987_show_get_hall_status, NULL);
static struct attribute *ak8987_attributes[] = {
	&dev_attr_enable_hall_sensor.attr,
	&dev_attr_get_hall_status.attr, /*debug, purpose*/
	&dev_attr_irq_count.attr,/*debug purpose*/
	NULL
};

static const struct attribute_group ak8987_attr_group = {
	.attrs = ak8987_attributes,
};

static void hall_timer_handler(unsigned long data)
{
	struct hall_dev *hall_timer_temp= (struct hall_dev *)data;
	queue_work(hall_timer_temp->hall_wq, &hall_timer_temp->hall_work);
}

void hall_work_func(struct work_struct *work)
{
	int value = 0;

	/*report events of hall*/
	value = query_hall_event();
	input_event(hw_hall_dev.hw_input_hall, EV_MSC, MSC_SCAN, value);
	input_sync(hw_hall_dev.hw_input_hall);
	atomic_dec(&irq_no_at);

	AK8789_INFOMSG("input hall event:ox%x\n",value);
}

int gpio_setup(int gpio_num, const char* gpio_name)
{
	int ret = 0;

	ret = gpio_request(gpio_num, gpio_name);
	if(ret){
		AK8789_ERRMSG("requset gpio %d err %d\n", gpio_num, ret);
		return ret;
	}
	/*we have set the state of gpio107 108 109 119 in board-8226-gpiomux*/
	//gpio_tlmm_config(GPIO_CFG(gpio_num, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	ret = gpio_direction_input(gpio_num);
	if(ret){
		AK8789_ERRMSG("gpio %d direction input err %d\n", gpio_num, ret);
		return ret;
	}

	return ret;
}

/*interrupts handle function*/
irqreturn_t hall_event_isr(int irq, void *dev)
{
	struct hall_dev *data = dev;
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned int trigger = 0;
	int ret = 0;

	AK8789_FLOWMSG("called hall_event_isr,irq=%d\n",irq);

	if ((!data)||(!desc)){
		AK8789_ERRMSG("dev null, or irq_desc null\n");
		return IRQ_NONE;
	}
	/*delay 100 ms, wait for timer schdule the work, then light up the lcd.*/
	wake_lock_timeout(&hall_wk, AKM8789_WAKEUP_TIME);

	trigger = desc->irq_data.state_use_accessors & IRQD_TRIGGER_MASK;

	/*set the irq type of hall irq*/
	if (trigger & IRQF_TRIGGER_LOW){
		ret = irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
		if (ret){
			AK8789_ERRMSG(" hall irq_set_irq_type error %s\n", desc->name);
		}
	}else if (trigger & IRQF_TRIGGER_HIGH){
		ret = irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
		if (ret){
			AK8789_ERRMSG(" hall irq_set_irq_type error %s\n", desc->name);
		}
	}else{
		wake_unlock(&hall_wk);
		AK8789_ERRMSG(" hall trigger not level type, error\n");
		return IRQ_NONE;
	}

	/*prevent the shake*/
	if (AKM8789_TIMER_DEBOUNCE){
		mod_timer(&(data->hall_timer) , jiffies + msecs_to_jiffies(AKM8789_TIMER_DEBOUNCE));
	}
	else{
		queue_work(data->hall_wq, &data->hall_work);
	}

	/*interrupts counter increases 1*/
	atomic_inc(&irq_no_at);

	return IRQ_HANDLED;
}

int hall_request_irq(int current_value, int hall_value, int irq, const char *name, unsigned long wake_flags)
{
	int ret = 0;
	AK8789_FLOWMSG("name=%s current_value=0x%x hall_value=0x%x irq %d flags %lu\n", name, current_value, hall_value, irq, wake_flags);

	/*if current gpio is high, set low as irq, otherwise vs*/
	if (!(current_value & hall_value)){
		ret = request_irq(irq, hall_event_isr,
			 IRQF_TRIGGER_LOW | wake_flags, name, &hw_hall_dev);
		if (ret){
			AK8789_ERRMSG("gpio %s request_irq fail %d\n", name, ret);
			return ret;
		}
		/*if the gpio can wake up, then set up the irq wake type*/
		if(IRQF_NO_SUSPEND == wake_flags){
			irq_set_irq_wake(irq , 1);
		}
	}else{
		ret = request_irq(irq, hall_event_isr,
			 IRQF_TRIGGER_HIGH | wake_flags, name, &hw_hall_dev);
		if (ret){
			AK8789_ERRMSG("gpio %s request_irq fail %d\n",name, ret);
			return ret;
		}
		/*if the gpio can wake up, then set up the irq wake type*/
		if(IRQF_NO_SUSPEND == wake_flags){
			irq_set_irq_wake(irq , 1);
		}
	}

	return ret;
}

static int hall_gpio_irq_setup(void)
{
	int ret = 0;
	int value = 0;
	int i = 0;
	gpio_data_t *gpio_ptr = hw_hall_dev.gpio_data;

	for (i = 0; i < hw_hall_dev.used_type; i++){
		ret = gpio_setup(gpio_ptr->gpio, gpio_ptr->name);
		if (ret){
			AK8789_ERRMSG("gpio_setup failed %s\n", gpio_ptr->name);
			return ret;
		}
		AK8789_INFOMSG("gpio_setup success gpio=%d\n",gpio_ptr->gpio);
		gpio_ptr++;
	}

	value = query_hall_event();

	gpio_ptr = hw_hall_dev.gpio_data;

	/*just N need wakeup*/
	for (i = 0; i < hw_hall_dev.used_type; i++){
		ret = hall_request_irq(value, gpio_ptr->hall_value, gpio_to_irq(gpio_ptr->gpio), gpio_ptr->name, gpio_ptr->wake_up);
		if (ret){
			AK8789_ERRMSG("hall _request_irq error%d\n", ret);
			return ret;
		}
		gpio_ptr++;
	}

	return ret;
}

int hall_pf_probe(struct platform_device *pdev)
{
	int err = 0;
	int ret = 0;
	int used_type = 0;
	int temp_val = 0;
	int gpio = 0;
	gpio_data_t *gpio_ptr;

	AK8789_INFOMSG("hall_pf_probe called\n");

	err = of_property_read_u32(pdev->dev.of_node, "hall_poles", &used_type);
	if (err) {
		AK8789_ERRMSG("Unable to read hall_poles\n");
		return err;
	}
	hw_hall_dev.used_type = (hall_used_type_t)(used_type);

	switch(used_type){
	case FOUR_POLE:
	{
		hw_hall_dev.gpio_data = kzalloc(sizeof(*hw_hall_dev.gpio_data)*used_type, GFP_KERNEL);
		if (IS_ERR(hw_hall_dev.gpio_data)){
			AK8789_ERRMSG("kzalloc err\n");
			return PTR_ERR(hw_hall_dev.gpio_data);
		}
		gpio_ptr = hw_hall_dev.gpio_data;

		/*RightNorth*/
		temp_val = of_get_named_gpio(pdev->dev.of_node,GPIO_CONFIG_RIGHT_NORTH,0);
		if (!gpio_is_valid(temp_val)) {
		AK8789_ERRMSG("Unable to read ak8789 irq gpio\n");
		err = temp_val;
		goto gpio_rn_err;
		} else {
		gpio = temp_val;
		AK8789_FLOWMSG("ak8789 irq gpio=%d \n",gpio);
		}

		gpio_ptr->gpio = gpio;
		gpio_ptr->wake_up = WAKE_FLAG_RIGHT_NORTH;
		gpio_ptr->name = kzalloc(sizeof(HALL_RN_INTERRUPT), GFP_KERNEL);
		if (IS_ERR(gpio_ptr->name)){
			AK8789_ERRMSG("kzalloc  hall_gpio_config_rn err\n");
			err =  PTR_ERR(hw_hall_dev.gpio_data);
			goto rn_mem;
		}
		memcpy(gpio_ptr->name, HALL_RN_INTERRUPT, sizeof(HALL_RN_INTERRUPT));
		gpio_ptr->hall_value = HALL_VALUE_RIGHT_NORTH;

		/*RightSouth*/
		gpio_ptr++;
		temp_val = of_get_named_gpio(pdev->dev.of_node,GPIO_CONFIG_RIGHT_SOUTH, 0);
		if (!gpio_is_valid(temp_val)) {
			AK8789_ERRMSG("Unable to read ak8789 irq gpio\n");
			err = temp_val;
			goto gpio_rs_err;
		} else {
			gpio = temp_val;
			AK8789_FLOWMSG("ak8789 irq gpio=%d \n",gpio);
		}

		gpio_ptr->gpio = gpio;
		gpio_ptr->wake_up = WAKE_FLAG_RIGHT_SOUTH;
		gpio_ptr->name = kzalloc(sizeof(HALL_RS_INTERRUPT), GFP_KERNEL);
		if (IS_ERR(gpio_ptr->name)){
			AK8789_ERRMSG("kzalloc  hall_gpio_config_rn err\n");
			err =  PTR_ERR(hw_hall_dev.gpio_data);
			goto rs_mem;
		}
		memcpy(gpio_ptr->name, HALL_RS_INTERRUPT, sizeof(HALL_RS_INTERRUPT));
		gpio_ptr->hall_value = HALL_VALUE_RIGHT_SOUTH;

		/*LeftNorth*/
		gpio_ptr++;
		temp_val = of_get_named_gpio(pdev->dev.of_node,GPIO_CONFIG_LEFT_NORTH, 0);
		if (!gpio_is_valid(temp_val)) {
			AK8789_ERRMSG("Unable to read ak8789 irq gpio\n");
			err = temp_val;
			goto gpio_ln_err;
		} else {
			gpio = temp_val;
			AK8789_ERRMSG("ak8789 irq gpio=%d \n",gpio);
		}

		gpio_ptr->gpio = gpio;
		gpio_ptr->wake_up = WAKE_FLAG_LEFT_NORTH;
		gpio_ptr->name = kzalloc(sizeof(HALL_LN_INTERRUPT), GFP_KERNEL);
		if (IS_ERR(gpio_ptr->name)){
			AK8789_ERRMSG("kzalloc  hall_gpio_config_rn err\n");
			err =  PTR_ERR(hw_hall_dev.gpio_data);
			goto ln_mem;
		}
		memcpy(gpio_ptr->name, HALL_LN_INTERRUPT, sizeof(HALL_LN_INTERRUPT));
		gpio_ptr->hall_value = HALL_VALUE_LEFT_NORTH;

		/*LeftSouth*/
		gpio_ptr++;
		temp_val = of_get_named_gpio(pdev->dev.of_node,GPIO_CONFIG_LEFT_SOUTH, 0);
		AK8789_FLOWMSG("ak8789 irq ret=%d \n",ret);
		if (!gpio_is_valid(temp_val)) {
			AK8789_ERRMSG("Unable to read ak8789 irq gpio\n");
			err = temp_val;
			goto gpio_ls_err;
		} else {
			gpio = temp_val;
			AK8789_FLOWMSG("ak8789 irq gpio=%d \n",gpio);
		}

		gpio_ptr->gpio = gpio;
		gpio_ptr->wake_up = WAKE_FLAG_LEFT_SOUTH;
		gpio_ptr->name = kzalloc(sizeof(HALL_LS_INTERRUPT), GFP_KERNEL);
		if (IS_ERR(gpio_ptr->name)){
			AK8789_ERRMSG("kzalloc  hall_gpio_config_rn err\n");
			err =  PTR_ERR(hw_hall_dev.gpio_data);
			goto ls_mem;
		}
		memcpy(gpio_ptr->name, HALL_LS_INTERRUPT, sizeof(HALL_LS_INTERRUPT));
		gpio_ptr->hall_value = HALL_VALUE_LEFT_SOUTH;
		break;
	}
	case TWO_POLE:
	{
		AK8789_ERRMSG("2 poles function is not supported%d\n",err);
		return -ENXIO;
	}
	default:
	{
		AK8789_ERRMSG("err in the hall poles value%d\n",err);
		return -ENXIO;
	}
	}/*end switch*/

	err =  sysfs_create_group(&pdev->dev.kobj, &ak8987_attr_group);
	if (err){
		AK8789_ERRMSG("sysfs create error %d\n", err);
		goto sysfs_create_fail;
	}

	hw_hall_dev.hw_input_hall = input_allocate_device();
	if (IS_ERR(hw_hall_dev.hw_input_hall)){
		AK8789_ERRMSG("hw_input_hall alloc error %ld\n", PTR_ERR(hw_hall_dev.hw_input_hall));
		goto input_err;
	}
	hw_hall_dev.hw_input_hall->name = "hall-sensor-ak8789";
	set_bit(EV_MSC, hw_hall_dev.hw_input_hall->evbit);
	set_bit(MSC_SCAN, hw_hall_dev.hw_input_hall->mscbit);
	err = input_register_device(hw_hall_dev.hw_input_hall);
	if (err){
		AK8789_ERRMSG("hw_input_hall regiset error %d\n", err);
		goto input_register_fail;
	}

	wake_lock_init(&hall_wk, WAKE_LOCK_SUSPEND, "hall_wakelock");

	hw_hall_dev.hall_wq = create_singlethread_workqueue("hall_wq");
	if (IS_ERR(hw_hall_dev.hall_wq)){
		AK8789_ERRMSG("wq create error %ld\n", PTR_ERR(hw_hall_dev.hall_wq));
		goto wq_fail;
	}

	INIT_WORK(&hw_hall_dev.hall_work, hall_work_func);

	init_timer(&(hw_hall_dev.hall_timer));
	hw_hall_dev.hall_timer.data= (unsigned long)(&hw_hall_dev);  //pointer the current platfrom data
	hw_hall_dev.hall_timer.function = &hall_timer_handler;

	ret = hall_gpio_irq_setup();

	device_init_wakeup(&pdev->dev, true);

	/* set app info */
	ret = app_info_set("Hall", "akm8789");
	if (ret < 0)
	{
		AK8789_ERRMSG("%s(line %d): app_info_set error,ret=%d\n",__func__,__LINE__,ret);
	}
	return err;

wq_fail:
input_register_fail:
	input_free_device(hw_hall_dev.hw_input_hall);
input_err:
	kfree(gpio_ptr->name);
sysfs_create_fail:
ls_mem:
gpio_ls_err:
	gpio_ptr--;
	kfree(gpio_ptr->name);
ln_mem:
gpio_ln_err:
	gpio_ptr--;
	kfree(gpio_ptr->name);
rs_mem:
gpio_rs_err:
	gpio_ptr--;
	kfree(gpio_ptr->name);
rn_mem:
gpio_rn_err:
	kfree(hw_hall_dev.gpio_data);
	return err;
}

static int ak8987_init(void)
{
	int err = 0;

	err = platform_driver_register(&hw_hall_dev.hall_drv_pf);
	if (err){
		AK8789_ERRMSG("hall_pf_drv_fall regiset error %d\n", err);
		goto hall_pf_drv_fail;
	}

	return err;

hall_pf_drv_fail:
	platform_driver_unregister(&hw_hall_dev.hall_drv_pf);
	return err;
}

static void __exit ak8987_exit(void)
{
	input_unregister_device(hw_hall_dev.hw_input_hall);
	platform_driver_unregister(&hw_hall_dev.hall_drv_pf);
}

MODULE_AUTHOR("huawei");
MODULE_DESCRIPTION("ak8987 hall");
MODULE_LICENSE("GPL");

module_init(ak8987_init);
module_exit(ak8987_exit);

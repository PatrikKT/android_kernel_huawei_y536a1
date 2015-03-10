/**************************************************
 Copyright (C), 2010-2012, Huawei Tech. Co., Ltd.
 File Name: kernel/drivers/misc/mrms501a.c
**************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mrms501a.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/input.h>

#include <misc/app_info.h>

#define MR_OPEN_EVENT_VALUE  (~(0x00000002))
#define MR_CLOSE_EVENT_VALUE (0x00000002)
#define mrms501a_TIMER_INTERVAL  HZ //jiffies 1S

/*keep the current status:
1: open
0: close
*/
#define MR_OPEN  (1)
#define MR_CLOSE (0)

int giMRsensorValue = 1;

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
struct early_suspend mrms501a_earlysuspend;
static void mrms501a_early_suspend(struct early_suspend *h);
static void mrms501a_late_resume(struct early_suspend *h);
#endif

//#define DEBUG_mrms501a

/*Macro switch*/
#ifdef DEBUG_mrms501a
#define MRUS_DMSG(format, args...) printk(KERN_INFO "[%s] (line: %u) " format "\n", \
__FUNCTION__, __LINE__, ##args)
#else
#define MRUS_DMSG(format, args...)
#endif

#define MRUS_ERRMSG(format, args...) printk(KERN_ERR "[%s] (line: %u) " format "\n", \
__FUNCTION__, __LINE__, ##args)


static ssize_t mrms501a_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", giMRsensorValue);
}
static struct device_attribute mrms501a_attr =
    __ATTR(mrms501a_info, 0444, mrms501a_show, NULL);


static irqreturn_t mrms501a_interrupt(int irq, void *handle)
{
    struct mrms501a_data_struct *mr = handle;

    static unsigned long lTimerPre = 0;

    unsigned long lTimerCur = jiffies;

    wake_lock_timeout(&mr->mrus_wake_lock, 5*HZ); //delay 5 s, waite for timer schdule the work, then light up the lcd.
    if(time_before(lTimerCur, lTimerPre + mrms501a_TIMER_INTERVAL))
    {
        mod_timer(&(mr->timer_detect), jiffies+mrms501a_TIMER_INTERVAL/5);//after 0.2s, read a value, then report the value
    }
    else
    {
        mod_timer(&(mr->timer_detect), jiffies+1);//read a value, then report the value immediately.
    }

    lTimerPre = jiffies;
    return IRQ_HANDLED;
}
static void mrms501a_work(struct work_struct *work)
{
    struct mrms501a_data_struct *mrus = container_of(work, struct mrms501a_data_struct, work);

    int gpio_val = 0;

    gpio_val = gpio_get_value(mrus->irq_gpio);

    MRUS_ERRMSG("old status: giMRsensorValue: %d, new status gpio: %d", giMRsensorValue, gpio_val);

    giMRsensorValue = gpio_val;

    if(gpio_val == MR_OPEN)
    {
        input_event(mrus->hw_input_hall, EV_MSC, MSC_SCAN, MR_OPEN_EVENT_VALUE);
    }
    else
    {
        input_event(mrus->hw_input_hall, EV_MSC, MSC_SCAN, MR_CLOSE_EVENT_VALUE);
    }
    input_sync(mrus->hw_input_hall);

}
static void timer_detect_func(unsigned long arg)
{
    struct mrms501a_data_struct *mrus = (struct mrms501a_data_struct *)arg;

    queue_work(mrus->mrms501a_wq, &mrus->work);
}

static int __devinit mrms501a_probe(struct platform_device *pdev)
{
    	int ret = 0;
	u32 temp_val;

	struct mrms501a_data_struct *mrus = kzalloc(sizeof(struct mrms501a_data_struct), GFP_KERNEL);
	int irq_gpio = 0;

	temp_val = of_get_named_gpio(pdev->dev.of_node,
			"qcom,hall-int-gpio", 0);
	MRUS_DMSG("mrms501a irq ret=%d \n",ret);
	if (!gpio_is_valid(temp_val)) {
		MRUS_ERRMSG("Unable to read mrms501a irq gpio\n");
		return ret;
	} else {
		irq_gpio = temp_val;
		MRUS_DMSG("mrms501a irq gpio=%d \n",irq_gpio);
	}

	mrus->dev = &(pdev->dev);
	mrus->irq_gpio = irq_gpio;
#if 0
	mrus->gpio_config = msplatdev->gpio_config;

	if (msplatdev->gpio_config) {
	    ret = msplatdev->gpio_config(NORMAL);
	    if (ret < 0) {
	        MRUS_ERRMSG("mrms501a: set gpio error %d", ret);
	        goto gpio_config_failed;
	    }
	}
#else
	ret = gpio_request(irq_gpio, "mrms501a");
	if (ret < 0) {
	    MRUS_ERRMSG("mrms501a: request gpio error %d", ret);
	    goto gpio_request_failed;
	}

	ret = gpio_tlmm_config(GPIO_CFG(irq_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(0 != ret)
	{
	    printk("%s: Failed to config gpio %d. code: %d.\n",__func__, 5, ret);
	    gpio_free(5);
		goto gpio_config_failed;
	}
#endif

	ret = gpio_direction_input(irq_gpio);
	if (ret < 0) {
	       MRUS_ERRMSG("gpio direction input failed %d", ret);
	    goto gpio_direction_input_failed;
	}
	irq_set_irq_wake(gpio_to_irq(irq_gpio),1);

	ret = device_create_file(&pdev->dev, &mrms501a_attr);
	if (ret < 0) {
	    MRUS_ERRMSG("creat sys file failed %d", ret);
	    goto device_create_file_failed;
	}
	giMRsensorValue = gpio_get_value(mrus->irq_gpio);
	/* allocate input device */
	mrus->hw_input_hall = input_allocate_device();
	if (NULL == mrus->hw_input_hall)
	{
	    printk("input_allocate_device=NULL!! \n");
	}
	if (IS_ERR(mrus->hw_input_hall)){
		    dev_err(&mrus->hw_input_hall->dev, "hw_input_hall alloc error %ld\n", PTR_ERR(mrus->hw_input_hall));
	    return PTR_ERR(mrus->hw_input_hall);
	}

	mrus->hw_input_hall->name = "mrms501a";
	set_bit(EV_MSC, mrus->hw_input_hall->evbit);
	set_bit(MSC_SCAN, mrus->hw_input_hall->mscbit);
	/* register input device */
	ret = input_register_device(mrus->hw_input_hall);
	if (ret)
	{
	    printk("input_register_device!=0!! \n");
	    dev_err(&mrus->hw_input_hall->dev, "hw_input_hall regiset error %d\n", ret);
	    goto input_register_fail;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	mrms501a_earlysuspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	mrms501a_earlysuspend.suspend = mrms501a_early_suspend;
	mrms501a_earlysuspend.resume = mrms501a_late_resume;
	register_early_suspend(&mrms501a_earlysuspend);
#endif

	wake_lock_init(&mrus->mrus_wake_lock, WAKE_LOCK_SUSPEND, "mrms501a sleep");

	mrus->mrms501a_wq = create_singlethread_workqueue("mrms501a_wq");
	if (!mrus->mrms501a_wq)
	{
	    MRUS_ERRMSG("create workque failed \n");
	    goto create_singlethread_workqueue_failed;
	}

	INIT_WORK(&mrus->work, mrms501a_work);
	init_timer(&mrus->timer_detect);
	mrus->timer_detect.data = (unsigned long)mrus;  //pointer the current platfrom data
	mrus->timer_detect.function = &timer_detect_func;

	 ret = request_irq(gpio_to_irq(irq_gpio), (irq_handler_t)mrms501a_interrupt,
					 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
					 "mrms501a", mrus);
	if (ret < 0) {
	    MRUS_ERRMSG("mrms501a: request irq failed %d", ret);
	    goto request_threaded_irq_failed;
	}

	/* set app info */
	ret = app_info_set("Hall", "Mrms501a");
	if (ret < 0)
	{
		MRUS_ERRMSG("%s(line %d): app_info_set error,ret=%d\n",__func__,__LINE__,ret);
	}

	device_init_wakeup(&pdev->dev, true);
	printk(KERN_INFO "[%s] (line: %u) mrms501a_probe succuss \n",__FUNCTION__, __LINE__);
	return ret;
request_threaded_irq_failed:
    del_timer_sync(&mrus->timer_detect);
create_singlethread_workqueue_failed:
    device_remove_file(&pdev->dev, &mrms501a_attr);
input_register_fail:
    input_free_device(mrus->hw_input_hall);
device_create_file_failed:
gpio_direction_input_failed:
    gpio_free(irq_gpio);
gpio_request_failed:
    //msplatdev->gpio_config(LOWPOWER);
gpio_config_failed:
    return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mrms501a_early_suspend(struct early_suspend *h)
{
    return;
}

static void mrms501a_late_resume(struct early_suspend *h)
{
    return;
}
#endif

static struct of_device_id mrms501a_match_table[] = {
	{	.compatible = "huawei,hall-mrms501a",
	},
};

static struct platform_driver mrms501a_driver = {
    .probe = mrms501a_probe,
    .driver = {
        .name  = "mrms501a",
        .owner = THIS_MODULE,
		.of_match_table = mrms501a_match_table,
    },
};

static int __init mrms501a_init(void)
{
    return platform_driver_register(&mrms501a_driver);
}

static void __exit mrms501a_exit (void)
{
    platform_driver_unregister(&mrms501a_driver);
}

late_initcall(mrms501a_init);
module_exit(mrms501a_exit);

MODULE_DESCRIPTION("mrms501a MR sensor");
MODULE_LICENSE("GPL");


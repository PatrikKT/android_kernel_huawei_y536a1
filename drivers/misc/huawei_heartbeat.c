
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/printk.h>

#define KOBJ_ATTR_RW(_name)  static struct kobj_attribute _name##_attr = \
	__ATTR(_name, 0644, _name##_show, _name##_store)

struct heartbeat_data{
	struct kobject  *obj;
	int running;
	int enable;
	struct delayed_work work;
	struct workqueue_struct *wq;
};

struct heartbeat_data *heartbeat = NULL;

static void keep_alive_response(void *info)
{
	int cpu = smp_processor_id();
        pr_debug("cpu %d is alive!\n",cpu);
	smp_mb();
}

static void call_other_cpus( struct work_struct *work )
{
        int cpu;
	smp_mb();
	for_each_cpu(cpu, cpu_online_mask)
	{
		pr_debug("heartbeat call cpu %d\n", cpu);
		smp_call_function_single(cpu, keep_alive_response, NULL, 1);
	}
	
	
	if( heartbeat->enable )
	{
	     queue_delayed_work_on(0, heartbeat->wq, &heartbeat->work, msecs_to_jiffies(200) );  
	     pr_debug("heartbeat restarting\n");         
	}
	else
	{
	     pr_debug("heartbeat stopped\n");       
	}
}

static ssize_t run_show(struct kobject *kobj, 
			    struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 8, "%d", heartbeat->enable );
}

static ssize_t run_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	int val;
	sscanf(buf,"%d",&val);
	pr_debug( "heartbeat val %d, run:%d\n", val,heartbeat->running );       
	if( val )
	{
	        if( !heartbeat->enable )
	        {
	                pr_debug( "heartbeat starting\n"); 
	              	queue_delayed_work_on(0, heartbeat->wq, &heartbeat->work, msecs_to_jiffies(200) );
	                heartbeat->enable = 1;
	        }
	}
	else
	{
	        cancel_delayed_work_sync(&heartbeat->work);
	        heartbeat->enable = 0;
	        pr_debug( "heartbeat stopping\n"); 
	}
	
	return count;
}

KOBJ_ATTR_RW(run);


static struct attribute *heartbeat_attrs[]={
	&run_attr.attr,
	NULL
};

static struct attribute_group heartbeat_group = {
	.attrs = heartbeat_attrs,	
};


static int __init heartbeat_init(void)
{
	int ret=0;

	heartbeat = kzalloc(sizeof(*heartbeat), GFP_KERNEL);
	if (!heartbeat) 
	{
		ret = -ENOMEM;
		goto err_data;
	}
        heartbeat->enable = 0;
        heartbeat->running = 0;
        
	//create kobject
	heartbeat->obj = kobject_create_and_add("heartbeat", NULL);
	if (!heartbeat->obj)
	{
		ret = -ENOMEM;
		goto err_kobj;
	}

	ret = sysfs_create_group(heartbeat->obj, &heartbeat_group);
	if (ret) 
	{
		goto err_sysfs;
	}
	
	heartbeat->wq = alloc_workqueue("heartbeat", WQ_HIGHPRI, 0);
	
	INIT_DELAYED_WORK(&heartbeat->work, call_other_cpus);
	
	
	return 0;
	
err_sysfs:
	kobject_put(heartbeat->obj);
err_kobj:
	kfree(heartbeat); 	
err_data:
	return ret;
}

static void __exit heartbeat_exit(void)
{
	destroy_workqueue( heartbeat->wq );
	sysfs_remove_group(heartbeat->obj, &heartbeat_group);
	kobject_put( heartbeat->obj );
	kfree(heartbeat);
}

module_init(heartbeat_init);
module_exit(heartbeat_exit);

MODULE_DESCRIPTION("huawei_heartbeat");
MODULE_LICENSE("GPL");

 

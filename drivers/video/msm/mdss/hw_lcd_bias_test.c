/*
 * lcd_bias_test.c
 * Test driver for TI TPS6132_B LCD BIAS IC.
 * Supported parts include:
 *
 * Copyright (C) 2013 Huawei Technologies Pvt Ltd
 * Author: Anurup M <anurup.m@huawei.com>
 *         Anisha Mariam Varghese <anisha.varghese@huawei.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/limits.h>
#include <linux/delay.h>

#define CMD_CODE_MAX 				(0x14)

enum lcd_biasic_reg_id {
	BIASIC_VPOS_REG,
	BIASIC_VNEG_REG,
	COMMON,
};

#define BIASIC_CONTROL_REG          (0xFF)
#define LCD_BIAS_ERR  	1
#define LCD_BIAS_DBG 	2

int lcd_bias_debug_mask = LCD_BIAS_ERR;

#define lcd_bias_log_err(x...)				\
do{										\
	if( lcd_bias_debug_mask >= LCD_BIAS_ERR )	\
	{									\
		printk(KERN_ERR "[LCD_BIAS_ERR] " x);	\
	}									\
										\
}while(0)

#define lcd_bias_log_dbg(x...)				\
do{										\
	if( lcd_bias_debug_mask >= LCD_BIAS_DBG )	\
	{									\
		printk(KERN_INFO"[LCD_BIAS_DBG] " x);	\
	}									\
										\
}while(0)

module_param_named(lcd_bias_debug_mask, lcd_bias_debug_mask, int, 0664);


struct lcd_bias {
	struct i2c_driver i2c;
};

static int lcd_bias_i2c_write(struct i2c_client *client, u8 *writebuf,
		int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		lcd_bias_log_err( "%s: i2c write error,ret=%d!\n", __func__,ret);

	return ret;
}

static int lcd_bias_i2c_read(struct i2c_client *client, u8 *readbuf,
		int readlen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = readbuf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = readlen,
			.buf = readbuf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		lcd_bias_log_err( "%s:i2c read error,ret=%d!\n", __func__,ret);

	return ret;
}

/* Parse input from sysfs, validate and get the mapping cmd code for VPOS
 * and VNEG */
static int parse_input(char *pvalue, u8 reg)
{
	int cmdcode = 0;
	int retval;
	char *ptr = pvalue;
	int unitval = 0;
	int tenval = 0;
	char ch = *ptr;

	/* Call toupper */
	while (ch) {
		*ptr = toupper(ch);
		ptr++;
		ch = *ptr;
	}

	/* If first character is a +/- then validate it */
	switch (pvalue[0]) {
	case '0' ... '9':
		break;
	case '+':
		if (BIASIC_VNEG_REG == reg) {
			lcd_bias_log_err ("Invalid voltage!\n");
			return -EINVAL;
		}
		pvalue++;
		break;
	case '-':
		if (BIASIC_VPOS_REG == reg) {
			lcd_bias_log_err ("Invalid voltage!\n");
			return -EINVAL;
		}
		pvalue++;
		break;
	default:
		lcd_bias_log_err ("Invalid voltage!\n");
		return -EINVAL;
	}

	retval = sscanf(pvalue, "%d.%dV", &unitval, &tenval);
	if (retval < 0) {
		lcd_bias_log_err (" Invalid voltage:%s!\n", pvalue);
		return -EINVAL;
	}

	switch (unitval) {
	/* 4V */
	case 4:
		cmdcode = tenval;
		break;
		/* 5V */
	case 5:
		cmdcode = 0xA + tenval;
		break;
		/* 6V */
	case 6:
		/* Maximum value supported by IC is 6.0 V */
		if (0 != tenval) {
			lcd_bias_log_err ("Invalid voltage:%s!\n", pvalue);
			return -EINVAL;
		}
		cmdcode = CMD_CODE_MAX;
		break;
	default:
		lcd_bias_log_err (" Invalid voltage%s!\n", pvalue);
		return -EINVAL;
	}

	lcd_bias_log_dbg("command code is 0x%x\n", cmdcode);
	return cmdcode;
}

/* Generate the command code to write ti IC using I2C */
static int generate_cmd_code(const char *buf, size_t size, u8 reg,
		int *cmdcode)
{
	char *ipbuf = NULL;

	ipbuf = kzalloc(size + 1, GFP_KERNEL);
	if (!ipbuf) {
		lcd_bias_log_err("%s:sysfs store alloc error!\n", __func__);
		return -ENOMEM;
	}

	strncpy(ipbuf, buf, size);
	/* Parse the input and covert to Hex value */
	*cmdcode = parse_input(ipbuf, reg);
	if (*cmdcode < 0) {
		kfree(ipbuf);
		return -EINVAL;
	}
	kfree(ipbuf);
	return *cmdcode;
}

/* Get the voltage mapping from command code */
static int get_volt_from_cmdcode(int cmdcode, char *buf, u8 reg)
{
	int len = 0;
	char ch;
	char *volt = NULL;

	if (BIASIC_VPOS_REG == reg) {
		ch = '+';
		volt  = "VPOS";
	} else {
		ch = '-';
		volt = "VNEG";
	}

	if (cmdcode >= 0x0 && cmdcode < 0xA) {
		len = snprintf(buf, 14, "%s : %c4.%dV\n", volt,ch, cmdcode);
	} else if(cmdcode >= 0xA && cmdcode < 0x14) {
		len = snprintf(buf, 14, "%s : %c5.%dV\n", volt,ch, cmdcode - 0xA);
	} else if (cmdcode == CMD_CODE_MAX){
		len = snprintf(buf, 14, "%s : %c6.0V\n", volt,ch);
	} else {
		lcd_bias_log_err("Invalid cmd code:0x%d!\n", cmdcode);
	}

	return len;
}

static int lcd_bias_read_current_value(struct device *dev, 
									u8 reg, u8 cmdcode, 
									u8 *current_volt)
{
	int retval = 0;
	u8 readbuf;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	/* Call i2c Read to see the current voltage */
	readbuf = reg;
	retval = lcd_bias_i2c_read(client, &readbuf, 1);
	if (retval < 0) {
		lcd_bias_log_err("%s:i2c read error!\n", __func__);
		return retval;
	}

	if (NULL != current_volt)
		*current_volt = readbuf;

	if (readbuf == cmdcode) {
		lcd_bias_log_dbg("Voltage read from device is same as Input. Writing again!\n");
	}

	return retval;
}


/* i2c volt store called by the sysyfs store */
static int lcd_bias_volt_store(struct device *dev,
								size_t size, u8 reg, u8 cmdcode)
{
	int retval = 0;
	u8 writebuf[2];

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	/* Call i2c Write to set if required */
	writebuf[0] = reg;
	writebuf[1] = cmdcode;
	retval = lcd_bias_i2c_write(client, writebuf, 2);
	if (retval < 0) {
		lcd_bias_log_err("%s:i2c write error!\n", __func__);
		return retval;
	}

	/* data register backup to EEPROM */
	writebuf[0] = BIASIC_CONTROL_REG;
	writebuf[1] = 0x80;
	retval = lcd_bias_i2c_write(client, writebuf, 2);
	if (retval < 0) {
		lcd_bias_log_err("%s:i2c write error!\n", __func__);
		return retval;
	}

	/* IC needs 50 ms delay to flush the data */
	mdelay(50);

	return size;
}

/* i2c volt show called by the sysyfs show */
static int lcd_bias_volt_show(struct device *dev,
									char *buf, u8 reg)
{
	int retval = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	u8 readbuf;

	/* Call i2c Read to see the current voltage */
	readbuf = reg;
	retval = lcd_bias_i2c_read(client, &readbuf, 1);
	if (retval < 0) {
		lcd_bias_log_err("%s:i2c read error!\n", __func__);
		return retval;
	}

	retval = get_volt_from_cmdcode(readbuf, buf, reg);
	lcd_bias_log_dbg("Buf read is = %s!\n", buf);

	return retval;
}

/* sysfs show for vpos */
static ssize_t lcd_bias_volt_show_vpos(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	int retval;

	retval = lcd_bias_volt_show(dev, buf, BIASIC_VPOS_REG);
	if(retval < 0 )
		lcd_bias_log_dbg("Operation to read VPOS failed!\n");

	return (ssize_t)retval;
}

/* sysfs show for vneg */
static ssize_t lcd_bias_volt_show_vneg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;

	retval = lcd_bias_volt_show(dev, buf, BIASIC_VNEG_REG);
	if(retval < 0 )
		lcd_bias_log_dbg("Operation to read VNEG failed!\n");

	return (ssize_t)retval;
}

/* sysfs store for vpos */
static ssize_t lcd_bias_volt_store_vpos(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t size)
{
	int retval;
	int cmdcode;

	retval = generate_cmd_code(buf, size, BIASIC_VPOS_REG, &cmdcode);
	if(retval < 0 )
		return retval;

	/* Read the current value */
	retval = lcd_bias_read_current_value(dev, BIASIC_VPOS_REG, cmdcode, NULL);
	if (retval < 0)
		return retval;

	/* Write the new value */
	retval = lcd_bias_volt_store(dev, size, BIASIC_VPOS_REG, cmdcode);
	if(retval < 0 )
		lcd_bias_log_dbg("Operation to write VPOS failed!\n");

	return (ssize_t)retval;
}

/* sysfs store for vneg */
static ssize_t lcd_bias_volt_store_vneg(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t size)
{
	int retval, cmdcode;

	retval = generate_cmd_code(buf, size, BIASIC_VNEG_REG, &cmdcode);
	if(retval < 0 )
		return retval;

	/* Read the current value */
	retval = lcd_bias_read_current_value(dev, BIASIC_VNEG_REG, cmdcode, NULL);
	if (retval < 0)
		return retval;

	/* Write the new value */
	retval = lcd_bias_volt_store(dev, size, BIASIC_VNEG_REG, cmdcode);
	if(retval < 0 )
		lcd_bias_log_dbg("Operation to write VNEG failed!\n");

	return (ssize_t)retval;
}

/* sysfs show for vpos and vneg */
static ssize_t lcd_bias_volt_show_common(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval_pos, retval_neg;

	retval_pos = lcd_bias_volt_show(dev, buf, BIASIC_VPOS_REG);
	if(retval_pos < 0 )
		return retval_pos;

	retval_neg = lcd_bias_volt_show(dev, &buf[retval_pos], BIASIC_VNEG_REG);
	if(retval_neg < 0 )
		return retval_neg;

	return (ssize_t)(retval_pos+retval_neg);
}

/* sysfs store for vpos and vneg */
static ssize_t lcd_bias_volt_store_common(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t size)
{
	int retval = 0;
	int retval_pos = 0;
	int retval_neg = 0;
	int cmdcode = 0;
	u8 current_volt = 0;

	retval = generate_cmd_code(buf, size, COMMON, &cmdcode);
	if (retval < 0)
		return retval;

	/* Read the current value */
	retval = lcd_bias_read_current_value(dev, BIASIC_VPOS_REG, cmdcode,
												&current_volt);
	if (retval < 0)
		return retval;

	/* Write the new value */
	retval_pos = lcd_bias_volt_store(dev, size, BIASIC_VPOS_REG, cmdcode);
	if (retval_pos < 0)
		return retval_pos;

	/* Read the current value */
	retval = lcd_bias_read_current_value(dev, BIASIC_VNEG_REG, cmdcode, NULL);
	if (retval < 0)
		return retval;

	/* Write the new value */
	retval_neg = lcd_bias_volt_store(dev, size, BIASIC_VNEG_REG, cmdcode);
	if (retval_neg < 0){
		/* Reset VPOS to old one */
		if (0 != current_volt)
			lcd_bias_volt_store(dev, size, BIASIC_VPOS_REG, current_volt);
		return retval_neg;
	}

	return (ssize_t)(retval_pos+retval_neg);
}

/* sysfs attributes */
static DEVICE_ATTR(lcd_bias_volt_vpos, S_IRUGO | S_IWUSR,
				   lcd_bias_volt_show_vpos, lcd_bias_volt_store_vpos);

static DEVICE_ATTR(lcd_bias_volt_vneg, S_IRUGO | S_IWUSR,
				   lcd_bias_volt_show_vneg, lcd_bias_volt_store_vneg);

static DEVICE_ATTR(lcd_bias_volt_common, S_IRUGO | S_IWUSR,
				   lcd_bias_volt_show_common, lcd_bias_volt_store_common);

static struct attribute *lcd_bias_input_attributes[] = {
	&dev_attr_lcd_bias_volt_vpos.attr,
	&dev_attr_lcd_bias_volt_vneg.attr,
	&dev_attr_lcd_bias_volt_common.attr,
	NULL
};

static const struct attribute_group lcd_bias_attr = {
	.attrs = lcd_bias_input_attributes,
};

static struct of_device_id lcd_bias_i2c_of_match[] = {
	{ .compatible = "ti,lcd_bias",},
	{ },
};

MODULE_DEVICE_TABLE(of, lcd_bias_i2c_of_match);

static const struct i2c_device_id lcd_bias_id[] = {
	{ "lcd_bias", 0 },
	{ }
};

static int __devinit lcd_bias_probe(struct i2c_client *client,
		const struct i2c_device_id *i2c_id)
{
	int err = 0;
	const struct of_device_id *match =  NULL;
	struct device *dev = NULL;
	u8 readbuf = BIASIC_VPOS_REG;
	if (NULL == client) {
		lcd_bias_log_err("client is NULL!\n");
		return -ENODEV;
	}

	dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		lcd_bias_log_err("i2c check_functionality failed!\n");
		err = -ENODEV;
		return err;
	}

	/* Check device connection */
	/* Call i2c Read to see the device is connected or not */
	err = lcd_bias_i2c_read(client, &readbuf, 1);
	if (err < 0) {
		lcd_bias_log_err("%s:error communicating to device !\n", __func__);
		return err;
	}
	
	match = of_match_device(of_match_ptr(lcd_bias_i2c_of_match), dev);
	if (!match) {
		lcd_bias_log_err("match device failed!\n");
		return -ENODEV;
	}

	err = sysfs_create_group(&dev->kobj, &lcd_bias_attr);
	if (err < 0) {
		lcd_bias_log_err("sysfs_create_group failed!\n");
		return -ENODEV;
	}

	lcd_bias_log_dbg("lcd_bias:Probe successful!\n");
	return 0;
}

static int __devexit lcd_bias_release(struct i2c_client *client)
{
	struct device *dev = NULL;

	if (NULL == client) {
		lcd_bias_log_err("client is NULL!\n");
		return -ENODEV;
	}

	dev = &client->dev;
	sysfs_remove_group(&dev->kobj, &lcd_bias_attr);

	return 0;
}

static struct lcd_bias lcd_bias_driver = {
	.i2c = {
		.driver = {
			.name = "lcd_bias",
			.owner = THIS_MODULE,
			.of_match_table = lcd_bias_i2c_of_match,
		},
		.probe = lcd_bias_probe,
		.remove = lcd_bias_release,
		.id_table = lcd_bias_id,
	},
};

static int __init lcd_bias_init(void)
{
	int ret = i2c_add_driver(&lcd_bias_driver.i2c);

	lcd_bias_log_dbg("%s, lcd_bias module init , ret = %d\n",__func__, ret);
	return ret;
}

static void __exit lcd_bias_exit(void)
{
	i2c_del_driver(&lcd_bias_driver.i2c);
	lcd_bias_log_dbg("%s: lcd_bias module exit\n",__func__);
}

module_init(lcd_bias_init);
module_exit(lcd_bias_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Huawei - TI - TPS6132_B LCD BIAS IC debug driver");
MODULE_AUTHOR("Huawei Technologies India Pvt Ltd");

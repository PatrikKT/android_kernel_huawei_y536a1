/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"

#define FLASH_NAME "qcom,hw_lm3642"
#define FLASH_FLAG_REGISTER 0x0B

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver hw_lm3642_i2c_driver;

static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	{0x0A, 0x00},
};
static struct msm_camera_i2c_reg_array hw_lm3642_off_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array hw_lm3642_release_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array hw_lm3642_low_array[] = {
	{0x09, 0x40},
	{0x0A, 0x02},
};

static struct msm_camera_i2c_reg_array hw_lm3642_high_array[] = {
	{0x09, 0x0A},
	{0x08, 0x05},
	{0x0A, 0x23},
};

static struct msm_camera_i2c_reg_array lm3642_torch_array[] = {
	{0x09, 0x20},//140mA
	{0x0A, 0x02},
};


static void __exit msm_flash_hw_lm3642_i2c_remove(void)
{
	i2c_del_driver(&hw_lm3642_i2c_driver);
	return;
}

static const struct of_device_id hw_lm3642_i2c_trigger_dt_match[] = {
	{.compatible = "qcom,hw_lm3642"},
	{}
};

MODULE_DEVICE_TABLE(of, hw_lm3642_i2c_trigger_dt_match);

static const struct i2c_device_id hw_lm3642_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_hw_lm3642_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!id) {
		pr_err("msm_flash_adp1660_i2c_probe: id is NULL");
		id = hw_lm3642_i2c_id;
	}
      pr_err("msm_flash_hw_lm3642_i2c_probe:  enter lishiliang.");
	return msm_flash_i2c_probe(client, id);
}


static void hw_lm3624_shutdown(struct i2c_client * client)
{
    pr_err("[%s],[%d]\n", __func__, __LINE__);
    msm_flash_led_off(&fctrl);
    msm_flash_led_release(&fctrl);
}

static struct i2c_driver hw_lm3642_i2c_driver = {
	.id_table = hw_lm3642_i2c_id,
	.probe  = msm_flash_hw_lm3642_i2c_probe,
	.remove = __exit_p(msm_flash_hw_lm3642_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hw_lm3642_i2c_trigger_dt_match,
	},
	.shutdown = hw_lm3624_shutdown,
};

static int __init msm_flash_hw_lm3642_i2c_add_driver(void)
{
	CDBG("%s called\n", __func__);
	return i2c_add_driver(&hw_lm3642_i2c_driver);
}
static struct msm_camera_i2c_client hw_lm3642_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};
static struct msm_camera_i2c_reg_setting hw_lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting hw_lm3642_off_setting = {
	.reg_setting = hw_lm3642_off_array,
	.size = ARRAY_SIZE(hw_lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting hw_lm3642_release_setting = {
	.reg_setting = hw_lm3642_release_array,
	.size = ARRAY_SIZE(hw_lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting hw_lm3642_low_setting = {
	.reg_setting = hw_lm3642_low_array,
	.size = ARRAY_SIZE(hw_lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting hw_lm3642_high_setting = {
	.reg_setting = hw_lm3642_high_array,
	.size = ARRAY_SIZE(hw_lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_torch_setting = {
	.reg_setting = lm3642_torch_array,
	.size = ARRAY_SIZE(lm3642_torch_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t hw_lm3642_regs = {
	.init_setting = &hw_lm3642_init_setting,
	.off_setting = &hw_lm3642_off_setting,
	.low_setting = &hw_lm3642_low_setting,
	.high_setting = &hw_lm3642_high_setting,
	.release_setting = &hw_lm3642_release_setting,
	.torch_setting = &lm3642_torch_setting,

};

static struct msm_flash_fn_t hw_lm3642_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
	.torch_led_on = msm_torch_led_on,

};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &hw_lm3642_i2c_client,
	.reg_setting = &hw_lm3642_regs,
	.func_tbl = &hw_lm3642_func_tbl,
};

module_init(msm_flash_hw_lm3642_i2c_add_driver);
module_exit(msm_flash_hw_lm3642_i2c_remove);
MODULE_DESCRIPTION("hw_lm3642 FLASH");
MODULE_LICENSE("GPL v2");

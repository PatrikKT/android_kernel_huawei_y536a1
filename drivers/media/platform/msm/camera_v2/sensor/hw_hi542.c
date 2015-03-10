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
#include "msm_sensor.h"

#define HW_HI542_SENSOR_NAME "hw_hi542"
DEFINE_MSM_MUTEX(hw_hi542_mut);

static struct msm_sensor_ctrl_t hw_hi542_s_ctrl;
static int8_t hw_hi542_module_id = 0;

static struct msm_sensor_power_setting hw_hi542_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info hw_hi542_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id hw_hi542_i2c_id[] = {
	{HW_HI542_SENSOR_NAME, (kernel_ulong_t)&hw_hi542_s_ctrl},
	{ }
};

static int32_t msm_hw_hi542_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hw_hi542_s_ctrl);
}


static struct i2c_driver hw_hi542_i2c_driver = {
	.id_table = hw_hi542_i2c_id,
	.probe  = msm_hw_hi542_i2c_probe,
	.driver = {
		.name = HW_HI542_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hw_hi542_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

int32_t hw_hi542_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	pr_err("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

/****************************************************************************
* FunctionName: hw_hi542_match_module;
* Description : Check which manufacture this module based sensor hi542 belong to;
***************************************************************************/
static int hw_hi542_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
	hw_hi542_module_id = 1;
	strncpy((char *)s_ctrl->sensordata->sensor_name, "hw_hi542_sunny", sizeof("hw_hi542_sunny"));
	pr_info("check module id from camera id PIN:OK");
	return 0;
}

static struct msm_sensor_fn_t hw_hi542_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = hw_hi542_sensor_match_id,
	.sensor_match_module = hw_hi542_match_module,
};

static struct msm_sensor_ctrl_t hw_hi542_s_ctrl = {
	.sensor_i2c_client = &hw_hi542_sensor_i2c_client,
	.power_setting_array.power_setting = hw_hi542_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hw_hi542_power_setting),
	.msm_sensor_mutex = &hw_hi542_mut,
	.sensor_v4l2_subdev_info = hw_hi542_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hw_hi542_subdev_info),
	.func_tbl = &hw_hi542_sensor_func_tbl,
};

static const struct of_device_id hw_hi542_dt_match[] = {
	{.compatible = "qcom,hw_hi542", .data = &hw_hi542_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hw_hi542_dt_match);

static struct platform_driver hw_hi542_platform_driver = {
	.driver = {
		.name = "qcom,hw_hi542",
		.owner = THIS_MODULE,
		.of_match_table = hw_hi542_dt_match,
	},
};

static int32_t hw_hi542_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(hw_hi542_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hw_hi542_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&hw_hi542_platform_driver,
		hw_hi542_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&hw_hi542_i2c_driver);
}

static void __exit hw_hi542_exit_module(void)
{
	if (hw_hi542_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hw_hi542_s_ctrl);
		platform_driver_unregister(&hw_hi542_platform_driver);
	} else
		i2c_del_driver(&hw_hi542_i2c_driver);
	return;
}

module_init(hw_hi542_init_module);
module_exit(hw_hi542_exit_module);
MODULE_DESCRIPTION("hw_hi542");
MODULE_LICENSE("GPL v2");

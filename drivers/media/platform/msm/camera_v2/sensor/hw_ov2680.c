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

#define HW_OV2680_SENSOR_NAME "hw_ov2680"
DEFINE_MSM_MUTEX(hw_ov2680_mut);

static struct msm_sensor_ctrl_t hw_ov2680_s_ctrl;

static struct msm_sensor_power_setting hw_ov2680_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info hw_ov2680_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id hw_ov2680_i2c_id[] = {
	{HW_OV2680_SENSOR_NAME, (kernel_ulong_t)&hw_ov2680_s_ctrl},
	{ }
};

static int32_t msm_hw_ov2680_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hw_ov2680_s_ctrl);
}

static struct i2c_driver hw_ov2680_i2c_driver = {
	.id_table = hw_ov2680_i2c_id,
	.probe  = msm_hw_ov2680_i2c_probe,
	.driver = {
		.name = HW_OV2680_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hw_ov2680_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int hw_ov2680_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
    if(s_ctrl && s_ctrl->sensordata && s_ctrl->sensordata->sensor_info && s_ctrl->sensordata->sensor_info->sensor_project_name)
    {
    	s_ctrl->sensordata->sensor_name = "hw_ov2680_ofilm";
        memcpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060157FF-OV-O", MAX_SENSOR_NAME-1);
    }
    return 0;
}

static struct msm_sensor_fn_t hw_ov2680_func_tbl = {
    .sensor_config = msm_sensor_config,
    .sensor_power_up = msm_sensor_power_up,
    .sensor_power_down = msm_sensor_power_down,
    .sensor_match_id = msm_sensor_match_id,
    .sensor_match_module = hw_ov2680_match_module,
};

static struct msm_sensor_ctrl_t hw_ov2680_s_ctrl = {
	.sensor_i2c_client = &hw_ov2680_sensor_i2c_client,
	.power_setting_array.power_setting = hw_ov2680_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hw_ov2680_power_setting),
	.msm_sensor_mutex = &hw_ov2680_mut,
	.sensor_v4l2_subdev_info = hw_ov2680_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hw_ov2680_subdev_info),
	.func_tbl = &hw_ov2680_func_tbl,
};

static const struct of_device_id hw_ov2680_dt_match[] = {
	{.compatible = "qcom,hw_ov2680", .data = &hw_ov2680_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hw_ov2680_dt_match);

static struct platform_driver hw_ov2680_platform_driver = {
	.driver = {
		.name = "qcom,hw_ov2680",
		.owner = THIS_MODULE,
		.of_match_table = hw_ov2680_dt_match,
	},
};

static int32_t hw_ov2680_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(hw_ov2680_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hw_ov2680_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&hw_ov2680_platform_driver, hw_ov2680_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&hw_ov2680_i2c_driver);
}

static void __exit hw_ov2680_exit_module(void)
{
	if (hw_ov2680_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hw_ov2680_s_ctrl);
		platform_driver_unregister(&hw_ov2680_platform_driver);
	} else
		i2c_del_driver(&hw_ov2680_i2c_driver);
	return;
}

module_init(hw_ov2680_init_module);
module_exit(hw_ov2680_exit_module);
MODULE_DESCRIPTION("hw_ov2680");
MODULE_LICENSE("GPL v2");



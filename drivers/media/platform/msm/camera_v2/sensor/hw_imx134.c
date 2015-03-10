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
#include <misc/app_info.h>
#include "./msm.h"
#include "./actuator/msm_actuator.h"
#define HW_IMX134_SENSOR_NAME "hw_imx134"
DEFINE_MSM_MUTEX(hw_imx134_mut);

static struct msm_sensor_ctrl_t hw_imx134_s_ctrl;

static struct msm_sensor_power_setting hw_imx134_power_setting[] = {
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
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = MSM_SENSOR_MCLK_24HZ,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info hw_imx134_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id hw_imx134_i2c_id[] = {
	{HW_IMX134_SENSOR_NAME, (kernel_ulong_t)&hw_imx134_s_ctrl},
	{ }
};

static int32_t msm_hw_imx134_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hw_imx134_s_ctrl);
}

static struct i2c_driver hw_imx134_i2c_driver = {
	.id_table = hw_imx134_i2c_id,
	.probe  = msm_hw_imx134_i2c_probe,
	.driver = {
		.name = HW_IMX134_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hw_imx134_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hw_imx134_dt_match[] = {
	{.compatible = "qcom,hw_imx134", .data = &hw_imx134_s_ctrl},
	{}
};

/****************************************************************************
* FunctionName: hw_imx134_match_module;
* Description : Check which manufacture this module based Sony image sensor imx134 belong to;
***************************************************************************/
static int hw_imx134_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct v4l2_subdev *subdev_act[MAX_ACTUATOR_NUMBER] = {NULL};
	struct msm_actuator_ctrl_t *a_ctrl = NULL;
	int i=0;

	/*add project name for the project menu*/
	s_ctrl->sensordata->sensor_name = "hw_imx134";
	strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060131FA-IMX-L", strlen("23060131FA-IMX-L")+1);

	pr_info("%s %d : hw_imx134_match_module sensor_name=%s, sensor_project_name=%s \n",  __func__, __LINE__,
            s_ctrl->sensordata->sensor_name, s_ctrl->sensordata->sensor_info->sensor_project_name);
	pr_info("check module id from camera id PIN:OK \n");

	app_info_set("camera_main", s_ctrl->sensordata->sensor_info->sensor_project_name);

	if(!strncmp(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060131FA-IMX-L", MAX_SENSOR_NAME))
	{
		/*if it's liteon module, we need to get the actuator ctrl to change af parameter to index 4*/
		msm_sd_get_actdev(subdev_act);
		for(i=0; i<MAX_ACTUATOR_NUMBER; i++)
		{
			if(NULL != subdev_act[i])
				a_ctrl =  subdev_act[i]->dev_priv;
			if(NULL != a_ctrl)
				a_ctrl->cam_name = MSM_ACTUATOR_MAIN_CAM_4;
		}
	}

	return 0;
}

static struct msm_sensor_fn_t hw_imx134_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
	.sensor_match_module = hw_imx134_match_module,
#endif
};

static struct platform_driver hw_imx134_platform_driver = {
	.driver = {
		.name = "qcom,hw_imx134",
		.owner = THIS_MODULE,
		.of_match_table = hw_imx134_dt_match,
	},
};

static int32_t hw_imx134_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	
	pr_err("%s , %d \n", __func__, __LINE__);
	match = of_match_device(hw_imx134_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);

	return rc;
}

static int __init hw_imx134_init_module(void)
{
	int32_t rc = 0;
	
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&hw_imx134_platform_driver,
		hw_imx134_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	
	return i2c_add_driver(&hw_imx134_i2c_driver);
}

static void __exit hw_imx134_exit_module(void)
{
	if (hw_imx134_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hw_imx134_s_ctrl);
		platform_driver_unregister(&hw_imx134_platform_driver);
	} else
		i2c_del_driver(&hw_imx134_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hw_imx134_s_ctrl = {
	.sensor_i2c_client = &hw_imx134_sensor_i2c_client,
	.power_setting_array.power_setting = hw_imx134_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hw_imx134_power_setting),
	.msm_sensor_mutex = &hw_imx134_mut,
	.sensor_v4l2_subdev_info = hw_imx134_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hw_imx134_subdev_info),
	.func_tbl = &hw_imx134_func_tbl,
};

module_init(hw_imx134_init_module);
module_exit(hw_imx134_exit_module);
MODULE_DEVICE_TABLE(of, hw_imx134_dt_match);
MODULE_DESCRIPTION("Sony 8M Bayer Sensor -Imx134");
MODULE_LICENSE("GPL v2");

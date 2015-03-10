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
#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>


#undef CDBG
//#define IMX219_LITEON_DEBUG
#ifdef IMX219_LITEON_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define IMX219_LITEON_SENSOR_NAME "hw_imx219"
DEFINE_MSM_MUTEX(imx219_liteon_mut);

static struct msm_sensor_ctrl_t imx219_liteon_s_ctrl;
static int8_t imx219_liteon_module_id = 0;

static struct msm_sensor_power_setting imx219_liteon_power_setting[] = {
     {
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 15,
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

static struct v4l2_subdev_info imx219_liteon_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id imx219_liteon_i2c_id[] = {
	{IMX219_LITEON_SENSOR_NAME, (kernel_ulong_t)&imx219_liteon_s_ctrl},
	{ }
};
static int32_t msm_imx219_liteon_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx219_liteon_s_ctrl);
}

static struct i2c_driver imx219_liteon_i2c_driver = {
	.id_table = imx219_liteon_i2c_id,
	.probe  = msm_imx219_liteon_i2c_probe,
	.driver = {
		.name = IMX219_LITEON_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx219_liteon_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx219_liteon_dt_match[] = {
	{.compatible = "qcom,hw_imx219", .data = &imx219_liteon_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx219_liteon_dt_match);

static struct platform_driver imx219_liteon_platform_driver = {
	.driver = {
		.name = "qcom,hw_imx219",
		.owner = THIS_MODULE,
		.of_match_table = imx219_liteon_dt_match,
	},
};

static int32_t imx219_liteon_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	
	pr_err("%s: %d\n", __func__, __LINE__);
	match = of_match_device(imx219_liteon_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	
	return rc;
}

static int __init imx219_liteon_init_module(void)
{
	int32_t rc = 0;

	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&imx219_liteon_platform_driver,
		imx219_liteon_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	
	return i2c_add_driver(&imx219_liteon_i2c_driver);
}

static void __exit imx219_liteon_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx219_liteon_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx219_liteon_s_ctrl);
		platform_driver_unregister(&imx219_liteon_platform_driver);
	} else
		i2c_del_driver(&imx219_liteon_i2c_driver);
	return;
}

/****************************************************************************
* FunctionName: imx219_liteon_match_module;
* Description : add the project name ;
***************************************************************************/
static int imx219_liteon_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
	imx219_liteon_module_id = 1;
	
	/*add project name for the project menu*/
	s_ctrl->sensordata->sensor_name = "hw_imx219_liteon";
	strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060152FA-IMX-L", strlen("23060152FA-IMX-L")+1);
	pr_info("%s %d : imx219_liteon_match_module sensor_name=%s, sensor_project_name=%s \n",  __func__, __LINE__,
            s_ctrl->sensordata->sensor_name, s_ctrl->sensordata->sensor_info->sensor_project_name);
	pr_info("check module id from camera id PIN:OK \n");
	
	return 0;
}

static struct msm_sensor_fn_t imx219_liteon_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA	
	.sensor_match_module = imx219_liteon_match_module,
#endif
};

static struct msm_sensor_ctrl_t imx219_liteon_s_ctrl = {
	.sensor_i2c_client = &imx219_liteon_sensor_i2c_client,
	.power_setting_array.power_setting = imx219_liteon_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx219_liteon_power_setting),
	.msm_sensor_mutex = &imx219_liteon_mut,
	.sensor_v4l2_subdev_info = imx219_liteon_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx219_liteon_subdev_info),
	.func_tbl = &imx219_liteon_sensor_func_tbl,

};

module_init(imx219_liteon_init_module);
module_exit(imx219_liteon_exit_module);
MODULE_DESCRIPTION("OVimison 1M Bayer sensor");
MODULE_LICENSE("GPL v2");


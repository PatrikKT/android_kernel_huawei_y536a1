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

#define HW_OV5648_SENSOR_NAME "hw_ov5648"
DEFINE_MSM_MUTEX(hw_ov5648_mut);

static struct msm_sensor_ctrl_t hw_ov5648_s_ctrl;

static int8_t hw_ov5648_module_id = 0;

static struct msm_sensor_power_setting hw_ov5648_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 20,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = MSM_SENSOR_MCLK_24HZ,
		.delay = 20,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
static struct v4l2_subdev_info hw_ov5648_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id hw_ov5648_i2c_id[] = {
	{HW_OV5648_SENSOR_NAME, (kernel_ulong_t)&hw_ov5648_s_ctrl},
	{ }
};

static int32_t msm_hw_ov5648_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hw_ov5648_s_ctrl);
}
static struct i2c_driver hw_ov5648_i2c_driver = {
	.id_table = hw_ov5648_i2c_id,
	.probe  = msm_hw_ov5648_i2c_probe,
	.driver = {
		.name = HW_OV5648_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hw_ov5648_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hw_ov5648_dt_match[] = {
	{.compatible = "qcom,hw_ov5648", .data = &hw_ov5648_s_ctrl},
	{}
};

int RG_Ratio_Typical = 0x26c;
int BG_Ratio_Typical = 0x2ed;

struct otp_struct
{
	uint16_t module_id;
	int rg_ratio;
	int bg_ratio;	
	int r_gain;
	int g_gain;
	int b_gain;
};

static bool otp_read = false; 
static bool otp_valid = true;
static struct otp_struct otp_struct;

static int32_t ov5648_write_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr,uint16_t data)
{
	int32_t rc = 0;
	if(NULL != s_ctrl)
	{
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			addr,
			data, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: write data failed\n", __func__);
			return rc;
		}
	}
	return rc;
}

static int32_t ov5648_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr,uint16_t *data)
{
	int32_t rc = 0;
	if(NULL != s_ctrl)
	{
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			data, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: read data failed\n", __func__);
			return rc;
		}
	}
	return rc;
}

/****************************************************************************
* index: index of otp group
* Return value : 0 : group 0
*                      1 : group 1			
*                      -1 : empty or invalid
***************************************************************************/
static int check_otp(struct msm_sensor_ctrl_t *s_ctrl,struct otp_struct * otp_ptr)
{
	uint16_t flag = 0;
	int rc = 1;
	int  i =0;
	uint16_t rg_H = 0,rg_L = 0,bg_H = 0,bg_L = 0;
	
	// clear otp buffer
	for (i=0;i<16;i++) 
	{
		ov5648_write_i2c(s_ctrl,0x3d00 + i, 0x00);
	}
	
	/*read bank 1 first*/
	ov5648_write_i2c(s_ctrl,0x3d84, 0xc0); /*read mode*/
	ov5648_write_i2c(s_ctrl,0x3d85, 0x10); /*OTP startt address, bank 1 */
	ov5648_write_i2c(s_ctrl,0x3d86, 0x1f); /*OTP end address */
	ov5648_write_i2c(s_ctrl,0x3d81, 0x01); /*enable otp read*/
	
	msleep(10);
	ov5648_read_i2c(s_ctrl,0x3d00,&flag);
	ov5648_read_i2c(s_ctrl,0x3d04,&(otp_ptr->module_id));	
	ov5648_read_i2c(s_ctrl,0x3d05,&rg_H);
	ov5648_read_i2c(s_ctrl,0x3d06,&rg_L);
	otp_ptr->rg_ratio = (rg_H << 8) + rg_L;
	ov5648_read_i2c(s_ctrl,0x3d07,&bg_H);
	ov5648_read_i2c(s_ctrl,0x3d08,&bg_L);	
	otp_ptr->bg_ratio = (bg_H << 8) + bg_L;	
	ov5648_write_i2c(s_ctrl,0x3d81, 0x00); /*disable otp read*/
	printk("%s:flag=0x%x,module_id=0x%x,rg=0x%x,bg=0x%x\n",__func__,
			(flag & 0xC0),(otp_ptr->module_id)>>4,otp_ptr->rg_ratio,otp_ptr->bg_ratio);
	
	if(0x40 != (flag & 0xC0))
	{
		rc = 0;
		printk("data of bank 1 is empty or invalid\n");
		// clear otp buffer
		for (i=0;i<16;i++) 
		{
			ov5648_write_i2c(s_ctrl,0x3d00 + i, 0x00);
		}
		
		/*read bank 0 */
		ov5648_write_i2c(s_ctrl,0x3d84, 0xc0); /*read mode*/
		ov5648_write_i2c(s_ctrl,0x3d85, 0x00); /*OTP start address, bank 0 */
		ov5648_write_i2c(s_ctrl,0x3d86, 0x0f); /*OTP end address */
		ov5648_write_i2c(s_ctrl,0x3d81, 0x01); /*enable otp read*/
		
		msleep(10);
		ov5648_read_i2c(s_ctrl,0x3d05,&flag);	
		ov5648_read_i2c(s_ctrl,0x3d09,&(otp_ptr->module_id));	
		ov5648_read_i2c(s_ctrl,0x3d0a,&rg_H);
		ov5648_read_i2c(s_ctrl,0x3d0b,&rg_L);
		otp_ptr->rg_ratio = (rg_H << 8) + rg_L;
		ov5648_read_i2c(s_ctrl,0x3d0c,&bg_H);
		ov5648_read_i2c(s_ctrl,0x3d0d,&bg_L);	
		otp_ptr->bg_ratio = (bg_H << 8) + bg_L;
		ov5648_write_i2c(s_ctrl,0x3d81, 0x00); /*disable otp read*/
		printk("%s:flag=0x%x,module_id=0x%x,rg=0x%x,bg=0x%x\n",__func__,
			(flag & 0xC0),(otp_ptr->module_id)>>4,otp_ptr->rg_ratio,otp_ptr->bg_ratio);
		if(0x40 != (flag & 0xC0))
		{
			printk("data of bank 0 is empty or invalid\n");
			rc = -1;		
		}
		
	}
	
	return rc;
			
}

static int update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl,int R_gain, int G_gain, int B_gain)
{
	if (R_gain>0x400) 
	{
		ov5648_write_i2c(s_ctrl,0x5186, R_gain>>8);
		ov5648_write_i2c(s_ctrl,0x5187, R_gain & 0x00ff);
	}
	
	if (G_gain>0x400) 
	{
		ov5648_write_i2c(s_ctrl,0x5188, G_gain>>8);
		ov5648_write_i2c(s_ctrl,0x5189, G_gain & 0x00ff);
	}
	
	if (B_gain>0x400) 
	{
		ov5648_write_i2c(s_ctrl,0x518a, B_gain>>8);
		ov5648_write_i2c(s_ctrl,0x518b, B_gain & 0x00ff);
	}
	
	return 0;
}

static int update_otp(struct msm_sensor_ctrl_t *s_ctrl,struct otp_struct * otp_ptr)
{
	int R_gain = 0, G_gain = 0, B_gain = 0, G_gain_R = 0, G_gain_B = 0;
	int rg = 0,bg = 0;
	
	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	if(-1 == check_otp(s_ctrl,otp_ptr))
	{
		otp_valid = false;
		return -1;	
	}
	otp_read = true;
	rg = otp_ptr->rg_ratio;
	bg = otp_ptr->bg_ratio;
	
	//calculate G gain
	//0x400 = 1x gain
	if(bg < BG_Ratio_Typical) 
	{
		if (rg< RG_Ratio_Typical) 
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&  
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / bg;
			R_gain = 0x400 * RG_Ratio_Typical / rg; 
		}
		else 
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&  
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else 
	{
		if (rg < RG_Ratio_Typical) 
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&  
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else 
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&  
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg / BG_Ratio_Typical;
			G_gain_R = 0x400 * rg / RG_Ratio_Typical;
			
			if(G_gain_B > G_gain_R ) 
			{
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			}
			else 
			{
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}    
	}

	otp_ptr->r_gain = R_gain;
	otp_ptr->g_gain = G_gain;
	otp_ptr->b_gain = B_gain;

	return 0;

}

int  ov5648_write_otp (struct msm_sensor_ctrl_t *s_ctrl)
{
	if(true == otp_valid)
	{
		update_awb_gain(s_ctrl,otp_struct.r_gain,otp_struct.g_gain,otp_struct.b_gain);
	}
	return 0;
}

/****************************************************************************
* FunctionName: hw_ov5648_match_module;
* Description : Check which manufacture this module based sensor ov5648 belong to;
***************************************************************************/
static int hw_ov5648_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
	/*read otp data only once*/
	if(false == otp_read)
	{
		memset(&otp_struct,0,sizeof(struct otp_struct));
		ov5648_write_i2c(s_ctrl,0x0100, 0x01);
		update_otp(s_ctrl,&otp_struct);
		ov5648_write_i2c(s_ctrl,0x0100, 0x00);
	}

	if(false == otp_valid)
	{
		printk("%s:read otp error!\n",__func__);
	}
	else
	{
		hw_ov5648_module_id = otp_struct.module_id >> 4;
	}

       /* formular to : driver use sensor name without module name */
	if(0x02 == hw_ov5648_module_id)
	{
		s_ctrl->sensordata->sensor_name = "hw_ov5648_foxconn";

		strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060132FF-OV-F", strlen("23060132FF-OV-F")+1);
	}
	else{
		s_ctrl->sensordata->sensor_name = "hw_ov5648_foxconn";
    		strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060132FF-OV", strlen("23060132FF-OV")+1);
	}
	pr_info("%s %d : hw_imx135_match_module sensor_name=%s, sensor_project_name=%s \n",  __func__, __LINE__,
    		s_ctrl->sensordata->sensor_name, s_ctrl->sensordata->sensor_info->sensor_project_name);
	pr_info("check module id from camera id PIN:OK \n");
	return 0;
}

static struct msm_sensor_fn_t hw_ov5648_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_match_module = hw_ov5648_match_module,
	.sensor_write_otp = ov5648_write_otp,
};

MODULE_DEVICE_TABLE(of, hw_ov5648_dt_match);

static struct platform_driver hw_ov5648_platform_driver = {
	.driver = {
		.name = "qcom,hw_ov5648",
		.owner = THIS_MODULE,
		.of_match_table = hw_ov5648_dt_match,
	},
};

static int32_t hw_ov5648_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	pr_info("%s:%d\n", __func__, __LINE__);
	match = of_match_device(hw_ov5648_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hw_ov5648_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&hw_ov5648_platform_driver,
							hw_ov5648_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&hw_ov5648_i2c_driver);
}

static void __exit hw_ov5648_exit_module(void)
{
	if (hw_ov5648_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hw_ov5648_s_ctrl);
		platform_driver_unregister(&hw_ov5648_platform_driver);
	} else
		i2c_del_driver(&hw_ov5648_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hw_ov5648_s_ctrl = {
	.sensor_i2c_client = &hw_ov5648_sensor_i2c_client,
	.power_setting_array.power_setting = hw_ov5648_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hw_ov5648_power_setting),
	.msm_sensor_mutex = &hw_ov5648_mut,
	.sensor_v4l2_subdev_info = hw_ov5648_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hw_ov5648_subdev_info),
	.func_tbl = &hw_ov5648_func_tbl,
};

module_init(hw_ov5648_init_module);
module_exit(hw_ov5648_exit_module);
MODULE_DESCRIPTION("hw_ov5648");
MODULE_LICENSE("GPL v2");

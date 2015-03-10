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

#define HW_S5K4E1_SENSOR_NAME "hw_s5k4e1"
DEFINE_MSM_MUTEX(hw_s5k4e1_mut);

static struct msm_sensor_ctrl_t hw_s5k4e1_s_ctrl;

static int8_t hw_s5k4e1_module_id = 0;
static struct msm_sensor_power_setting hw_s5k4e1_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 20,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
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
              /* formular to macro */
		.config_val = MSM_SENSOR_MCLK_24HZ,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
static struct v4l2_subdev_info hw_s5k4e1_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id hw_s5k4e1_i2c_id[] = {
	{HW_S5K4E1_SENSOR_NAME, (kernel_ulong_t)&hw_s5k4e1_s_ctrl},
	{ }
};

static int RG_Ratio_Typical = 0x2f1;
static int BG_Ratio_Typical = 0x2bb;
#define S5K4E1GA_OTP_MSB  1
#define S5K4E1GA_OTP_LSB  2

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

static int32_t s5k4e1_write_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr,uint16_t data)
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

static int32_t s5k4e1_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr,uint16_t *data)
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
static uint16_t s5k4e1_read_layer(struct msm_sensor_ctrl_t *s_ctrl,int16_t layer, uint16_t addr, uint16_t MSBorLSB)
{
	uint16_t otpData = 0;

	/* select the layer */
	s5k4e1_write_i2c(s_ctrl, 0x310C, layer);
	
	/*read the data form the layer and addr by byte*/
	s5k4e1_read_i2c(s_ctrl, addr, &otpData);
	
	if(S5K4E1GA_OTP_LSB == MSBorLSB)
	{
		otpData = otpData & 0x000F;
	}
	else
	{
		otpData = (otpData & 0x00FF) >> 4;
	}

	return otpData;
}

/*
 check opt data by the year, the year  on layer18 is always non 0
 if chek ok will retrun the year and also back the OTP place
 if check failed, it will return 0
*/
static int s5k4e1_otp_check_awb(struct msm_sensor_ctrl_t *s_ctrl, uint16_t *pAddr, uint16_t *pMSBorLSB)
{
	uint16_t otpData = 0;
	uint16_t otpAddr = 0;
	uint8_t  otpDataLsb= 0;
	uint8_t  otpDataMsb= 0;

	/* select the layer 18 to read the year */
	s5k4e1_write_i2c(s_ctrl, 0x310C, 18);

	/* read 0x310D LSB */
	otpAddr = 0x310D;
	s5k4e1_read_i2c(s_ctrl, otpAddr, &otpData);
	otpDataLsb = otpData & 0x000F;
	if(0 != otpDataLsb)
	{
		goto END_LSB;
	}

	/* read 0x310E MSB and LSB */
	otpAddr = 0x310E;
	s5k4e1_read_i2c(s_ctrl, otpAddr, &otpData);
	otpDataMsb = (otpData & 0x00FF) >> 4;
	otpDataLsb = otpData & 0x000F;
	if(0 != otpDataMsb)
	{
		goto END_MSB;
	}
	if(0 != otpDataLsb)
	{
		goto END_LSB;
	}

	/* read 0x310F MSB and LSB */
	otpAddr = 0x310F;
	s5k4e1_read_i2c(s_ctrl, otpAddr, &otpData);
	otpDataMsb = (otpData & 0x00FF) >> 4;
	otpDataLsb = otpData & 0x000F;
	if(0 != otpDataMsb)
	{
		goto END_MSB;
	}
	if(0 != otpDataLsb)
	{
		goto END_LSB;
	}

	goto END_FAILED;

END_LSB:
	*pAddr = otpAddr;
	*pMSBorLSB = S5K4E1GA_OTP_LSB;
	return otpDataLsb;

END_MSB:
	*pAddr = otpAddr;
	*pMSBorLSB = S5K4E1GA_OTP_MSB;
	return otpDataMsb;

END_FAILED:
	pr_info("%s: check otp failed\n",__func__);
	return 0;
}
static int check_otp(struct msm_sensor_ctrl_t *s_ctrl,struct otp_struct * otp_ptr)
{
	int16_t rc = -1;
	int16_t i=0;
	int16_t start_data=28;
	int16_t moduleid_layer = 26;
	uint16_t otpAddr = 0;
	uint16_t MSBorLSB = 0;

	/*ready for reading different layer*/
	s5k4e1_write_i2c(s_ctrl,0x30F9, 0x0E);
	s5k4e1_write_i2c(s_ctrl,0x30FA, 0x0A);
	s5k4e1_write_i2c(s_ctrl,0x30FB, 0x71);
	s5k4e1_write_i2c(s_ctrl,0x30FB, 0x70);
	msleep(5);

	/*check awb */
	if(0 == s5k4e1_otp_check_awb(s_ctrl, &otpAddr, &MSBorLSB))
	{
		return -1;
	}

	/*read module id*/
	otp_ptr->module_id = s5k4e1_read_layer(s_ctrl,moduleid_layer, otpAddr, MSBorLSB);

	/*read r/g*/
	for(i=0;i<4;i++)
	{
		otp_ptr->rg_ratio += s5k4e1_read_layer(s_ctrl,start_data, otpAddr, MSBorLSB)*(1<<(4*(3-i)));
		start_data++;
	}

	/*read b/g*/
	for(i=0;i<4;i++)
	{
		otp_ptr->bg_ratio += s5k4e1_read_layer(s_ctrl,start_data, otpAddr, MSBorLSB)*(1<<(4*(3-i)));
		start_data++;
	}

	pr_info("%s:module_id=0x%x,rg=0x%x,bg=0x%x\n",__func__,otp_ptr->module_id,otp_ptr->rg_ratio,otp_ptr->bg_ratio);
	rc = (0 == otp_ptr->rg_ratio || 0 == otp_ptr->module_id || 0 == otp_ptr->bg_ratio)?-1:0;
	return rc;


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
	//0x100 = 1x gain
	if(bg < BG_Ratio_Typical) 
	{
		if (rg< RG_Ratio_Typical) 
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&  
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x100;
			B_gain = 0x100 * BG_Ratio_Typical / bg;
			R_gain = 0x100 * RG_Ratio_Typical / rg; 
		}
		else 
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&  
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x100;
			G_gain = 0x100 * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else 
	{
		if (rg < RG_Ratio_Typical) 
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&  
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = 0x100;
			G_gain = 0x100 * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else 
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&  
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x100 * bg / BG_Ratio_Typical;
			G_gain_R = 0x100 * rg / RG_Ratio_Typical;
			
			if(G_gain_B > G_gain_R ) 
			{
				B_gain = 0x100;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			}
			else 
			{
				R_gain = 0x100;
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

static int update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl,int R_gain, int G_gain, int B_gain)
{
	if (R_gain>0x100) 
	{
	        s5k4e1_write_i2c(s_ctrl, 0x0210, R_gain>>8);
	        s5k4e1_write_i2c(s_ctrl, 0x0211, R_gain & 0x00ff);
	}
	
	if (G_gain>0x100) 
	{
	        //digital_gain_greenR
	        s5k4e1_write_i2c(s_ctrl, 0x020E, G_gain>>8);
	        s5k4e1_write_i2c(s_ctrl, 0x020F, G_gain & 0x00ff);

	        //digital_gain_greenB
	        s5k4e1_write_i2c(s_ctrl, 0x0214, G_gain>>8);
	        s5k4e1_write_i2c(s_ctrl, 0x0215, G_gain & 0x00ff);

	}
	
	if (B_gain>0x100) 
	{
	        s5k4e1_write_i2c(s_ctrl, 0x0212, B_gain>>8);
	        s5k4e1_write_i2c(s_ctrl, 0x0213, B_gain & 0x00ff);
	}
	
	return 0;
}

int  s5k4e1_write_otp (struct msm_sensor_ctrl_t *s_ctrl)
{
	if(true == otp_valid)
	{
		update_awb_gain(s_ctrl,otp_struct.r_gain,otp_struct.g_gain,otp_struct.b_gain);
	}
	return 0;
}
static int32_t msm_hw_s5k4e1_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hw_s5k4e1_s_ctrl);
}

static struct i2c_driver hw_s5k4e1_i2c_driver = {
	.id_table = hw_s5k4e1_i2c_id,
	.probe  = msm_hw_s5k4e1_i2c_probe,
	.driver = {
		.name = HW_S5K4E1_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hw_s5k4e1_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hw_s5k4e1_dt_match[] = {
	{.compatible = "qcom,hw_s5k4e1", .data = &hw_s5k4e1_s_ctrl},
	{}
};

/****************************************************************************
* FunctionName: hw_s5k4e1_match_module;
* Description : Check which manufacture this module based sensor s5k4e1 belong to;
***************************************************************************/
static int hw_s5k4e1_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
        /*add project name for the project menu*/
        /* formular to : driver use sensor name without module name */

	/*add project name for the project menu*/
	int rc = 0;
	hw_s5k4e1_module_id = -1;
	if(false == otp_read)
	{
		memset(&otp_struct,0,sizeof(struct otp_struct));
		update_otp(s_ctrl,&otp_struct);
	}

	if(false == otp_valid)
	{
		rc = -1;
		pr_err("%s:read otp error!\n",__func__);
	}
	else
	{
		hw_s5k4e1_module_id = otp_struct.module_id;
	}
	
	if(0x02 == hw_s5k4e1_module_id)
	{
		s_ctrl->sensordata->sensor_name = "hw_s5k4e1_sunny";
		strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060132FF-SAM-S", strlen("23060132FF-SAM-S")+1);
	
	}

	pr_info("%s %d : hw_s5k4e1_match_module sensor_name=%s, sensor_project_name=%s \n",  __func__, __LINE__,
	        s_ctrl->sensordata->sensor_name, s_ctrl->sensordata->sensor_info->sensor_project_name);
	return rc;
}

static struct msm_sensor_fn_t hw_s5k4e1_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_match_module = hw_s5k4e1_match_module,
	.sensor_write_otp = s5k4e1_write_otp,
};

MODULE_DEVICE_TABLE(of, hw_s5k4e1_dt_match);

static struct platform_driver hw_s5k4e1_platform_driver = {
	.driver = {
		.name = "qcom,hw_s5k4e1",
		.owner = THIS_MODULE,
		.of_match_table = hw_s5k4e1_dt_match,
	},
};

static int32_t hw_s5k4e1_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(hw_s5k4e1_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hw_s5k4e1_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&hw_s5k4e1_platform_driver,
		hw_s5k4e1_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&hw_s5k4e1_i2c_driver);
}

static void __exit hw_s5k4e1_exit_module(void)
{
	if (hw_s5k4e1_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hw_s5k4e1_s_ctrl);
		platform_driver_unregister(&hw_s5k4e1_platform_driver);
	} else
		i2c_del_driver(&hw_s5k4e1_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hw_s5k4e1_s_ctrl = {
	.sensor_i2c_client = &hw_s5k4e1_sensor_i2c_client,
	.power_setting_array.power_setting = hw_s5k4e1_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hw_s5k4e1_power_setting),
	.msm_sensor_mutex = &hw_s5k4e1_mut,
	.sensor_v4l2_subdev_info = hw_s5k4e1_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hw_s5k4e1_subdev_info),
	.func_tbl = &hw_s5k4e1_func_tbl,
};

module_init(hw_s5k4e1_init_module);
module_exit(hw_s5k4e1_exit_module);
MODULE_DESCRIPTION("hw_s5k4e1");
MODULE_LICENSE("GPL v2");

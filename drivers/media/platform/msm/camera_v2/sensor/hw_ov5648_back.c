/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
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

#define HW_OV5648_BACK_SENSOR_NAME "hw_ov5648_back"
DEFINE_MSM_MUTEX(hw_ov5648_back_mut);


#undef CDBG
#define HW_OV5648_BACK_DEBUG
#ifdef HW_OV5648_BACK_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

/* if support OTP or open OTP function, please define it */
//#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
#define OV5648_OTP_FEATURE
//#endif

#ifdef OV5648_OTP_FEATURE

//OV5648 Sunny 0x400 = 1x gain
#define OV5648_OTP_ONE_GAIN 0x400

//OV5648 has two bank
typedef enum ov5648_bank_count{
	BANK_0 = 0,
	BANK_1,
	BANK_MAX
}enum_ov5648_bank;

//Write OTP RGB sequence
enum gain_cfg_status {
	GAIN_CFG_R = 0,
	GAIN_CFG_G,
	GAIN_CFG_B,
	GAIN_CFG_MAX
};

//Write OTP RGB High addr, Low addr and name
typedef struct RGB_GAIN_ADDR{
	int32_t High;
	int32_t Low;
	char* Name;
}st_rgb_gain_addr;

//Bank Addr for bank select
typedef struct BANK_ADDR{
	int32_t High;
	int32_t Low;
}st_bank_addr;

//OTP write RGB addr and name
static const st_rgb_gain_addr RGB_Gain_Addr[GAIN_CFG_MAX] = {{0x5186, 0x5187, "R Gain"}, 
															{0x5188, 0x5189, "G Gain"},
															{0x518a, 0x518b, "B Gain"}};

//Bank Addr from 0~1
static const st_bank_addr OTP_Bank_Select[BANK_MAX] = {{0x00, 0x0f}, {0x10, 0x1f}};

//Golden sensor typical ratio
static int RG_Ratio_Typical = 0x2FC;
static int BG_Ratio_Typical = 0x2B5;

//OTP info struct
typedef struct ov5648_otp_info {
	uint16_t iProduct_Year;
	uint16_t iProduct_Month;
	uint16_t iProduct_Date;
	uint16_t iCamera_Id;
	uint16_t iVersion_Id;
	uint16_t iWB_RG_H;
	uint16_t iWB_RG_L;
	uint16_t iWB_BG_H;
	uint16_t iWB_BG_L;
	uint16_t iWB_GbGr_H;
	uint16_t iWB_GbGr_L;
	uint16_t iVCM_Start;
	uint16_t iVCM_End;
}st_ov5648_otp_info;

//OTP info
static st_ov5648_otp_info g_ov5648_otp = {0};
#endif
static struct msm_sensor_ctrl_t hw_ov5648_back_s_ctrl;

static int8_t hw_ov5648_module_id = 0;

static struct msm_sensor_power_setting hw_ov5648_back_power_setting[] = {
	//exchange the sequence of IOVDD and AVDD
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 25,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 15,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info hw_ov5648_back_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id hw_ov5648_back_i2c_id[] = {
	{HW_OV5648_BACK_SENSOR_NAME, (kernel_ulong_t)&hw_ov5648_back_s_ctrl},
	{ }
};

static int32_t msm_hw_ov5648_back_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hw_ov5648_back_s_ctrl);
}

static struct i2c_driver hw_ov5648_back_i2c_driver = {
	.id_table = hw_ov5648_back_i2c_id,
	.probe  = msm_hw_ov5648_back_i2c_probe,
	.driver = {
		.name = HW_OV5648_BACK_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hw_ov5648_back_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hw_ov5648_back_dt_match[] = {
	{.compatible = "qcom,hw_ov5648_back", .data = &hw_ov5648_back_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hw_ov5648_back_dt_match);

static struct platform_driver hw_ov5648_back_platform_driver = {
	.driver = {
		.name = "qcom,hw_ov5648_back",
		.owner = THIS_MODULE,
		.of_match_table = hw_ov5648_back_dt_match,
	},
};
static int hw_ov5648_back_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
	hw_ov5648_module_id = 1;
	
	/*add project name for the project menu*/
	s_ctrl->sensordata->sensor_name = "hw_ov5648_back";
	strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060129FA-OV-S", strlen("23060129FA-OV-S")+1);

	pr_info("%s %d : hw_ov5648_back_match_module sensor_name=%s, sensor_project_name=%s \n",  __func__, __LINE__,
            s_ctrl->sensordata->sensor_name, s_ctrl->sensordata->sensor_info->sensor_project_name);
	pr_info("check module id from camera id PIN:OK \n");
	
	return 0;
}

static int32_t hw_ov5648_back_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(hw_ov5648_back_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hw_ov5648_back_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&hw_ov5648_back_platform_driver,
		hw_ov5648_back_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hw_ov5648_back_i2c_driver);
}

static void __exit hw_ov5648_back_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hw_ov5648_back_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hw_ov5648_back_s_ctrl);
		platform_driver_unregister(&hw_ov5648_back_platform_driver);
	} else
		i2c_del_driver(&hw_ov5648_back_i2c_driver);
	return;
}

#ifdef OV5648_OTP_FEATURE

/****************************************************************************
* FunctionName: ov5648_otp_write_i2c;
* Description : write otp info via i2c;
***************************************************************************/
int32_t ov5648_otp_write_i2c(struct msm_sensor_ctrl_t *s_ctrl, int32_t addr, uint16_t data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
															addr,
															data,
															MSM_CAMERA_I2C_BYTE_DATA);

	if ( rc < 0 )
	{
		CDBG("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_read_i2c;
* Description : read otp info via i2c;
***************************************************************************/
int32_t ov5648_otp_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, uint16_t *data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
															addr,
															data,
															MSM_CAMERA_I2C_BYTE_DATA);
	if ( rc < 0 )
	{
		CDBG("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, *data);
	}
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_clear;
* Description : clear otp buffer 0x3D00~0x3D0F;
***************************************************************************/
int32_t ov5648_otp_clear(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i = 0;
	int32_t rc = 0;
	
	for ( i = 0; i < 16; i++ ) 
	{
		rc = ov5648_otp_write_i2c(s_ctrl, 0x3d00 + i, 0x00);
		
		if ( rc < 0 )
		{
			CDBG("%s:%d clear error: rc is %d, i2c addr is 0x3d00 + 0x%x \n", __func__, __LINE__, rc, i);
			break;
		}
	}
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_set_read_mode;
* Description : set sensor otp mode:read;
***************************************************************************/
int32_t ov5648_otp_set_read_mode(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	//set otp read mode before reading
	rc = ov5648_otp_write_i2c(s_ctrl, 0x3d84, 0xc0);
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_enable_read;
* Description : set sensor could be readed;
***************************************************************************/
int32_t ov5648_otp_enable_read(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	//enable otp reading
	rc = ov5648_otp_write_i2c(s_ctrl, 0x3d81, 0x01);
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_disable_read;
* Description : set sensor could not be readed;
***************************************************************************/
int32_t ov5648_otp_disable_read(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	//disable otp reading
	rc = ov5648_otp_write_i2c(s_ctrl, 0x3d81, 0x00);
	msleep(10);
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_select_bank;
* Description : select otp bank0 or bank1;
***************************************************************************/
int32_t ov5648_otp_select_bank(struct msm_sensor_ctrl_t *s_ctrl, enum_ov5648_bank bank)
{
	int32_t rc = 0;

	if ( ( bank >= BANK_0 ) && ( bank < BANK_MAX ) )
	{
		rc = ov5648_otp_write_i2c(s_ctrl, 0x3d85, OTP_Bank_Select[bank].High);
		rc = ov5648_otp_write_i2c(s_ctrl, 0x3d86, OTP_Bank_Select[bank].Low);
	}
	else
	{
		CDBG("%s error bank = %d\n", __func__, bank);
	}
	
	msleep(10);
	
	return rc;
}


/****************************************************************************
* FunctionName: ov5648_check_awb;
* Description : if WB data read from otp bank x is valid, return 0;
***************************************************************************/
int32_t ov5648_check_awb(const st_ov5648_otp_info *otp_ptr)
{
	int32_t ret = 0;

	if ( NULL != otp_ptr )
	{
		if ( (0 == otp_ptr->iWB_RG_H) && (0 == otp_ptr->iWB_RG_L) )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d otp_ptr is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_check_module;
* Description : if module info read from otp bank x is valid, return 0;
***************************************************************************/
int32_t ov5648_check_module(const st_ov5648_otp_info *otp_ptr)
{
	int32_t ret = 0;

	if ( NULL != otp_ptr )
	{
		if ( 0 == otp_ptr->iCamera_Id )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d otp_ptr is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_check_start_code;
* Description : if VCM start code read from otp bank x is valid, return 0;
***************************************************************************/
int32_t ov5648_check_start_code(const st_ov5648_otp_info *otp_ptr)
{
	int32_t ret = 0;

	if ( NULL != otp_ptr )
	{	
		if ( 0 == otp_ptr->iVCM_Start )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d otp_ptr is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_check_end_code;
* Description : if VCM end code read from otp bank x is valid, return 0;
***************************************************************************/
int32_t ov5648_check_end_code(const st_ov5648_otp_info *otp_ptr)
{
	int32_t ret = 0;

	if ( NULL != otp_ptr )
	{
		if ( 0 == otp_ptr->iVCM_End )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d otp_ptr is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_read_otp_bank1;
* Description : Read WB,module info and VCM info from bank1;
* return value:
* 0 means no need to read bank0
*-1 means need to read bank0
***************************************************************************/
int32_t ov5648_read_otp_bank1(struct msm_sensor_ctrl_t *s_ctrl, st_ov5648_otp_info *p_ov5648_otp)
{
	int32_t ret = 0;
	
	if ( NULL != p_ov5648_otp )
	{
		/*read AWB */
		ov5648_otp_read_i2c(s_ctrl, 0x3D07, &p_ov5648_otp->iWB_RG_H);
		ov5648_otp_read_i2c(s_ctrl, 0x3D08, &p_ov5648_otp->iWB_RG_L);
		ov5648_otp_read_i2c(s_ctrl, 0x3D09, &p_ov5648_otp->iWB_BG_H);
		ov5648_otp_read_i2c(s_ctrl, 0x3D0A, &p_ov5648_otp->iWB_BG_L);
		ov5648_otp_read_i2c(s_ctrl, 0x3D0B, &p_ov5648_otp->iWB_GbGr_H);
		ov5648_otp_read_i2c(s_ctrl, 0x3D0C, &p_ov5648_otp->iWB_GbGr_L);

		/*read module*/
		ov5648_otp_read_i2c(s_ctrl, 0x3D02, &p_ov5648_otp->iProduct_Year);
		ov5648_otp_read_i2c(s_ctrl, 0x3D03, &p_ov5648_otp->iProduct_Month);
		ov5648_otp_read_i2c(s_ctrl, 0x3D04, &p_ov5648_otp->iProduct_Date);
		ov5648_otp_read_i2c(s_ctrl, 0x3D05, &p_ov5648_otp->iCamera_Id);
		ov5648_otp_read_i2c(s_ctrl, 0x3D06, &p_ov5648_otp->iVersion_Id);

		/*read VCM start code*/
		ov5648_otp_read_i2c(s_ctrl, 0x3D0D, &p_ov5648_otp->iVCM_Start);
		
		if ( 0 == p_ov5648_otp->iVCM_Start )
		{
			ov5648_otp_read_i2c(s_ctrl, 0x3D00, &p_ov5648_otp->iVCM_Start);
			
			if( 0 == p_ov5648_otp->iVCM_Start )
			{
				CDBG("%s:%d ov5648_otp->iVCM_Start is No Value\n", __func__, __LINE__);
			}
		}

		p_ov5648_otp->iVCM_Start = p_ov5648_otp->iVCM_Start << 2;

		/*read VCM end code*/
		ov5648_otp_read_i2c(s_ctrl, 0x3D0E, &p_ov5648_otp->iVCM_End);
		
		if ( 0 == p_ov5648_otp->iVCM_End )
		{
			ov5648_otp_read_i2c(s_ctrl, 0x3D01, &p_ov5648_otp->iVCM_End);
		
			if ( 0 == p_ov5648_otp->iVCM_End )
			{
				CDBG("%s:%d ov5648_otp->iVCM_End is No Value\n", __func__, __LINE__);
			}
		}

		p_ov5648_otp->iVCM_End = p_ov5648_otp->iVCM_End << 2;
		
		if ( (0 != ov5648_check_awb(p_ov5648_otp)) || (0 != ov5648_check_module(p_ov5648_otp)) )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d p_ov5648_otp is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_read_otp_bank0;
* Description : Read WB and module info from bank0;
* return value:
* 0 means read bank0 OK
* -1 means read bank0 and bank1 all failed
***************************************************************************/
int32_t ov5648_read_otp_bank0(struct msm_sensor_ctrl_t *s_ctrl, st_ov5648_otp_info *p_ov5648_otp)
{
	int32_t ret = 0;

	if ( NULL != p_ov5648_otp )
	{
		if (0 != ov5648_check_awb(p_ov5648_otp))
		{
			/*read AWB */
			ov5648_otp_read_i2c(s_ctrl, 0x3D0A, &p_ov5648_otp->iWB_RG_H);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0B, &p_ov5648_otp->iWB_RG_L);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0C, &p_ov5648_otp->iWB_BG_H);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0D, &p_ov5648_otp->iWB_BG_L);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0E, &p_ov5648_otp->iWB_GbGr_H);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0F, &p_ov5648_otp->iWB_GbGr_L);
		}

		if (0 != ov5648_check_module(p_ov5648_otp))
		{
			/*read module*/
			ov5648_otp_read_i2c(s_ctrl, 0x3D05, &p_ov5648_otp->iProduct_Year);
			ov5648_otp_read_i2c(s_ctrl, 0x3D06, &p_ov5648_otp->iProduct_Month);
			ov5648_otp_read_i2c(s_ctrl, 0x3D07, &p_ov5648_otp->iProduct_Date);
			ov5648_otp_read_i2c(s_ctrl, 0x3D08, &p_ov5648_otp->iCamera_Id);
			ov5648_otp_read_i2c(s_ctrl, 0x3D09, &p_ov5648_otp->iVersion_Id);
		}
		
		if ( (0 != ov5648_check_awb(p_ov5648_otp)) || (0 != ov5648_check_module(p_ov5648_otp)) )
		{
			CDBG("%s:%d Read Bank0 failed or there is no valuable data!\n", __func__, __LINE__);
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d p_ov5648_otp is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_read_otp;
* Description : read otp info;
* return value:
* 0 means read otp info succeed.
* -1 means read otp info failed, should not write otp.
***************************************************************************/
int32_t ov5648_read_otp(struct msm_sensor_ctrl_t *s_ctrl)	
{
	int32_t ret = 0;
	st_ov5648_otp_info ov5648_otp;
	
	memset(&ov5648_otp, 0, sizeof(ov5648_otp));

	/* first read from bank 1 */
	
	ov5648_otp_set_read_mode(s_ctrl);

	ov5648_otp_clear(s_ctrl);

	ov5648_otp_select_bank(s_ctrl, BANK_1);

	ov5648_otp_enable_read(s_ctrl);

	ret = ov5648_read_otp_bank1(s_ctrl, &ov5648_otp);

	ov5648_otp_disable_read(s_ctrl);

	ov5648_otp_clear(s_ctrl); 

	/*if awb or module is 0, then read bank 0*/
	if ( 0 != ret )
	{
		printk("%s read from bank0\n", __func__);
		
		ov5648_otp_set_read_mode(s_ctrl);

		ov5648_otp_select_bank(s_ctrl, BANK_0);

		ov5648_otp_enable_read(s_ctrl);
		
		ret = ov5648_read_otp_bank0(s_ctrl, &ov5648_otp);
		
		ov5648_otp_disable_read(s_ctrl);

		ov5648_otp_clear(s_ctrl);
	}

	if ( (0 != ret)
	|| (0 != ov5648_check_start_code(&ov5648_otp))
	|| (0 != ov5648_check_end_code(&ov5648_otp)) )
	{
		CDBG("%s:%d ERROR:there is no valuable otp data!\n", __func__, __LINE__);
		ret = -1;
	}
	else
	{
		memcpy(&g_ov5648_otp, &ov5648_otp, sizeof(st_ov5648_otp_info));

		printk("%s:iProduct_Year = %d, \n \
		iProduct_Month = %d, \n \
		iProduct_Date = %d,\n  \
		iCamera_Id = %d, \n \
		iVersion_Id = %d, \n \
		iWB_RG_H = 0x0%X, \n \
		iWB_RG_L = 0x0%X, \n \
		iWB_BG_H = 0x0%X, \n \
		iWB_BG_L = 0x0%X,\n  \
		iWB_GbGr_H = 0x0%X, \n \
		iWB_GbGr_L = 0x0%X, \n \
		iVCM_Start = %d, \n \
		iVCM_End = %d\n ", 
		__func__, g_ov5648_otp.iProduct_Year, g_ov5648_otp.iProduct_Month, 
		g_ov5648_otp.iProduct_Date, g_ov5648_otp.iCamera_Id, g_ov5648_otp.iVersion_Id,	
		g_ov5648_otp.iWB_RG_H, g_ov5648_otp.iWB_RG_L, g_ov5648_otp.iWB_BG_H, 
		g_ov5648_otp.iWB_BG_L, g_ov5648_otp.iWB_GbGr_H, g_ov5648_otp.iWB_GbGr_L, 
		g_ov5648_otp.iVCM_Start, g_ov5648_otp.iVCM_End);
	}

	return ret;
}

/****************************************************************************
* FunctionName: ov5648_update_awb_gain;
* Description : write R_gain,G_gain,B_gain to otp;
* 0x400 =1x Gain
* 0 means write WB info succeed.
* -1 means write WB info failed.
***************************************************************************/
int32_t ov5648_update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl, const int *pGainCfg)
{
	int32_t rc = 0;
	int iGainindex = 0;
	
	if ( NULL != pGainCfg )
	{
		for ( iGainindex = GAIN_CFG_R; iGainindex < GAIN_CFG_MAX; iGainindex++ )
		{
			if ( pGainCfg[iGainindex] > OV5648_OTP_ONE_GAIN ) 
			{
				rc = ov5648_otp_write_i2c(s_ctrl, RGB_Gain_Addr[iGainindex].High, pGainCfg[iGainindex] >> 8);
				
				if ( rc < 0 )
				{
					CDBG("%s:%d write otp %s High failed, rc is %d\n", __func__, __LINE__, RGB_Gain_Addr[iGainindex].Name, rc);
					break;
				}
				
				rc = ov5648_otp_write_i2c(s_ctrl, RGB_Gain_Addr[iGainindex].Low, pGainCfg[iGainindex] & 0x00ff);
				
				if ( rc < 0 )
				{
					CDBG("%s:%d write otp %s Low failed, rc is %d\n", __func__, __LINE__, RGB_Gain_Addr[iGainindex].Name, rc);
					break;
				}

				printk("OV5648 sunny OTP AWB info:%s is 0x%X\n",  RGB_Gain_Addr[iGainindex].Name, pGainCfg[iGainindex]);
			}
			
		}

		if ( rc < 0 )
		{
			CDBG("Set ov5648 sunny OTP AWB Failed!\n");
		}
		else
		{
			CDBG("Set ov5648 sunny OTP AWB Succeed!\n");
		}
	}
	else
	{
		CDBG("%s:%d  ERROR: pGainCfg is NULL!\n", __func__, __LINE__);
	}

	return rc;
}

/****************************************************************************
* FunctionName: ov5648_sunny_get_otp_info;
* Description : get otp info from sensor;
***************************************************************************/
int32_t ov5648_sunny_get_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	CDBG("Get ov5648 sunny OTP info Enter\n");

	//set sensor mode:Active
	ov5648_otp_write_i2c(s_ctrl, 0x100, 0x1);
	
	rc = ov5648_read_otp(s_ctrl);
	
	usleep_range(5000, 6000);

	//set sensor mode:Standby
	ov5648_otp_write_i2c(s_ctrl, 0x100, 0x0);

	CDBG("Get ov5648 sunny OTP info Exit\n");

	return rc;
}

/****************************************************************************
* FunctionName: ov5648_sunny_set_otp_info;
* Description : set otp data to sensor;
* call this function after OV5648 initialization 
***************************************************************************/
static int32_t ov5648_sunny_set_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int GainCfg[GAIN_CFG_MAX] = {0};
	int rg, bg;
	int32_t rc = 0;

	static int i_read_otp = 0;
	
	//Get otp info on the first time
	if ( 0 == i_read_otp )
	{		
		rc = ov5648_sunny_get_otp_info(s_ctrl);
		
		if ( rc < 0 )
		{
			CDBG("%s:%d otp read failed.\n", __func__, __LINE__);
			return rc;
		}
		else
		{
			i_read_otp = 1;
		}
	}

	CDBG("Set ov5648 sunny OTP info Enter\n");
	
	rg = (g_ov5648_otp.iWB_RG_H << 8) + g_ov5648_otp.iWB_RG_L;
	bg = (g_ov5648_otp.iWB_BG_H << 8) + g_ov5648_otp.iWB_BG_L;

	//calculate G gain
	//0x400 = 1x gain
	if ( bg < BG_Ratio_Typical ) 
	{
		if ( rg< RG_Ratio_Typical ) 
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = OV5648_OTP_ONE_GAIN;
			B_gain = OV5648_OTP_ONE_GAIN * BG_Ratio_Typical / bg;
			R_gain = OV5648_OTP_ONE_GAIN * RG_Ratio_Typical / rg;
		}
		else 
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = OV5648_OTP_ONE_GAIN;
			G_gain = OV5648_OTP_ONE_GAIN * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else 
	{
		if ( rg < RG_Ratio_Typical ) 
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = OV5648_OTP_ONE_GAIN;
			G_gain = OV5648_OTP_ONE_GAIN * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else 
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = OV5648_OTP_ONE_GAIN * bg / BG_Ratio_Typical;
			G_gain_R = OV5648_OTP_ONE_GAIN * rg / RG_Ratio_Typical;

			if ( G_gain_B > G_gain_R ) 
			{
				B_gain = OV5648_OTP_ONE_GAIN;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			}
			else 
			{
				R_gain = OV5648_OTP_ONE_GAIN;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}
	}

	GainCfg[GAIN_CFG_R] = R_gain;
	GainCfg[GAIN_CFG_G] = G_gain;
	GainCfg[GAIN_CFG_B] = B_gain;

	//set sensor mode:Active
	ov5648_otp_write_i2c(s_ctrl, 0x100, 0x1);
	
	//Set the otp info to sensor
	rc = ov5648_update_awb_gain(s_ctrl, GainCfg);
	
	usleep_range(5000, 6000);

	//set sensor mode:Standby
	ov5648_otp_write_i2c(s_ctrl, 0x100, 0x0);

	CDBG("Set ov5648 sunny OTP info Exit\n");
	
	return rc;
}

#endif

static struct msm_sensor_fn_t hw_ov5648_back_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA	
	.sensor_match_module = hw_ov5648_back_match_module,
#endif
#ifdef OV5648_OTP_FEATURE
	//add otp function
	.sensor_write_otp = ov5648_sunny_set_otp_info,
#endif
};

static struct msm_sensor_ctrl_t hw_ov5648_back_s_ctrl = {
	.sensor_i2c_client = &hw_ov5648_back_sensor_i2c_client,
	.power_setting_array.power_setting = hw_ov5648_back_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hw_ov5648_back_power_setting),
	.msm_sensor_mutex = &hw_ov5648_back_mut,
	.sensor_v4l2_subdev_info = hw_ov5648_back_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hw_ov5648_back_subdev_info),
	.func_tbl = &hw_ov5648_back_sensor_func_tbl,
};

module_init(hw_ov5648_back_init_module);
module_exit(hw_ov5648_back_exit_module);
MODULE_DESCRIPTION("hw_ov5648_back");
MODULE_LICENSE("GPL v2");

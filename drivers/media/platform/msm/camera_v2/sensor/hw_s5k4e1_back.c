/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#include "msm_sensor.h"

#define HW_S5K4E1_BACK_SENSOR_NAME "hw_s5k4e1_back"
DEFINE_MSM_MUTEX(hw_s5k4e1_back_mut);

#define HW_S5K4E1_BACK_DEBUG
#undef CDBG
#ifdef HW_S5K4E1_BACK_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


#define S5K4E1GA_SUPPORT_OTP

#ifdef S5K4E1GA_SUPPORT_OTP
static int RG_Ratio_Typical =0x02B9;//0x02AF;
static int BG_Ratio_Typical =0x026C;//0x0265;
#define MAX_OTP_NUM 4
#define S5K4E1_OTP_MSB  1
#define S5K4E1_OTP_LSB  2
static uint16_t  g_s5k4e1ga_otp_awb[MAX_OTP_NUM] = {0}; //awb otp 
static uint16_t g_modeID = 0;;
#endif

static struct msm_sensor_ctrl_t hw_s5k4e1_back_s_ctrl;

static int8_t hw_s5k4e1_back_module_id = 0;

static struct msm_sensor_power_setting hw_s5k4e1_back_power_setting[] = {
	/* the power down sequence of AVDD prior to DOVDD*/
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
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
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

static struct v4l2_subdev_info hw_s5k4e1_back_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id hw_s5k4e1_back_i2c_id[] = {
	{HW_S5K4E1_BACK_SENSOR_NAME, (kernel_ulong_t)&hw_s5k4e1_back_s_ctrl},
	{ }
};

static int32_t msm_hw_s5k4e1_back_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hw_s5k4e1_back_s_ctrl);
}

static struct i2c_driver hw_s5k4e1_back_i2c_driver = {
	.id_table = hw_s5k4e1_back_i2c_id,
	.probe  = msm_hw_s5k4e1_back_i2c_probe,
	.driver = {
		.name = HW_S5K4E1_BACK_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hw_s5k4e1_back_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hw_s5k4e1_back_dt_match[] = {
	{.compatible = "qcom,hw_s5k4e1_back", .data = &hw_s5k4e1_back_s_ctrl},
	{}
};

static int hw_s5k4e1_back_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
	hw_s5k4e1_back_module_id = 1;

	s_ctrl->sensordata->sensor_name = "hw_s5k4e1_back";

    strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060129FA-SAM-L", strlen("23060129FA-SAM-L")+1);

    pr_info("%s %d : hw_s5k4e1_back_match_module sensor_name=%s, sensor_project_name=%s \n",  __func__, __LINE__,
            s_ctrl->sensordata->sensor_name, s_ctrl->sensordata->sensor_info->sensor_project_name);

    pr_info("check module id from camera id PIN:OK\n");
	return 0;
}

#ifdef S5K4E1GA_SUPPORT_OTP
/****************************************************************************
* FunctionName: s5k4e1ga_cci_i2c_write;
* Description : i2c write interface;
***************************************************************************/
static int32_t s5k4e1ga_cci_i2c_write(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);

    if(rc < 0)
    {
        CDBG("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
    }

    return rc;
}

/****************************************************************************
* FunctionName: s5k4e1ga_cci_i2c_read;
* Description : i2c read interface;
***************************************************************************/
static int32_t s5k4e1ga_cci_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
{
    int32_t rc = -EFAULT;
    uint16_t temp_data = 0;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
    		s_ctrl->sensor_i2c_client,
    		addr,
    		&temp_data, MSM_CAMERA_I2C_BYTE_DATA);    
    if(rc < 0)
    {
        CDBG("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
    }
    *data = temp_data;
    return rc;
}

/****************************************************************************
* FunctionName: s5k4e1ga_read_layer_data;
* Description : read layer interface;
***************************************************************************/
int s5k4e1ga_read_layer_data(struct msm_sensor_ctrl_t *s_ctrl, int layer, uint16_t addr, uint16_t MSBorLSB)
{
	uint16_t otpData = 0;

	/* select the layer */
	s5k4e1ga_cci_i2c_write(s_ctrl, 0x310C, layer);

	/*read the data form the layer and addr by byte*/
	s5k4e1ga_cci_i2c_read(s_ctrl, addr, &otpData);
	 
	if(S5K4E1_OTP_LSB == MSBorLSB)
	{
		otpData = otpData & 0x000F;
	}
	else
	{
		otpData = (otpData & 0x00FF)>> 4;
	}

	return otpData;
}

/****************************************************************************
* FunctionName: s5k4e1ga_read_rg;
* Description : read r/g data ;
***************************************************************************/
static uint16_t s5k4e1ga_read_rg(struct msm_sensor_ctrl_t *s_ctrl, uint16_t otpAddr, uint16_t MSBorLSB)
{
	uint16_t otpData = 0;
	uint16_t rg = 0;
		
    /*read RG */
	otpData = s5k4e1ga_read_layer_data(s_ctrl, 28, otpAddr, MSBorLSB);
	rg = otpData << 12;

	otpData = s5k4e1ga_read_layer_data(s_ctrl, 29, otpAddr, MSBorLSB);
	rg |= otpData << 8;

	otpData = s5k4e1ga_read_layer_data(s_ctrl, 30, otpAddr, MSBorLSB);
	rg |= otpData << 4;

	otpData = s5k4e1ga_read_layer_data(s_ctrl, 31, otpAddr, MSBorLSB);
	rg |= otpData ;

	return rg;
}

/****************************************************************************
* FunctionName: s5k4e1ga_read_bg;
* Description : read b/g data ;
***************************************************************************/
static uint16_t s5k4e1ga_read_bg(struct msm_sensor_ctrl_t *s_ctrl, uint16_t otpAddr, uint16_t MSBorLSB)
{
	uint8_t otpData = 0;
	uint16_t bg = 0;
	
    /*read BG */
	otpData = s5k4e1ga_read_layer_data(s_ctrl, 32, otpAddr, MSBorLSB);
	bg = otpData << 12;

	otpData = s5k4e1ga_read_layer_data(s_ctrl, 33, otpAddr, MSBorLSB);
	bg |= otpData << 8;

	otpData = s5k4e1ga_read_layer_data(s_ctrl, 34, otpAddr, MSBorLSB);
	bg |= otpData << 4;

	otpData = s5k4e1ga_read_layer_data(s_ctrl, 35, otpAddr, MSBorLSB);
	bg |= otpData ;

	return bg;
}


/*
 check opt data by the year, the year  on layer18 is always non 0
 if chek ok will retrun the year and also back the OTP place 
 if check failed, it will return 0
*/
static int s5k4e1_otp_check_data(struct msm_sensor_ctrl_t *s_ctrl, uint16_t *pAddr, uint16_t *pMSBorLSB)
{

    uint16_t otpData = 0;
	uint16_t otpAddr = 0;
	uint8_t  otpDataLsb= 0;
	uint8_t  otpDataMsb= 0;
	
	/* select the layer 18 to read the year */
	s5k4e1ga_cci_i2c_write(s_ctrl, 0x310C, 18);

	/* read 0x310D LSB */
	 otpAddr = 0x310D;
	 s5k4e1ga_cci_i2c_read(s_ctrl, otpAddr, &otpData);
	 otpDataLsb = otpData & 0x000F;
	 if(0 != otpDataLsb)
	 {
	 	goto END_LSB;;
	 }
	 
     /* read 0x310E MSB and LSB */
	 otpAddr = 0x310E;
	 s5k4e1ga_cci_i2c_read(s_ctrl, otpAddr, &otpData);
	 otpDataMsb = (otpData & 0x00FF)>> 4;
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
	 s5k4e1ga_cci_i2c_read(s_ctrl, otpAddr, &otpData);
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
	*pMSBorLSB = S5K4E1_OTP_LSB;
	return otpDataLsb;
	
END_MSB:
	*pAddr = otpAddr;
	*pMSBorLSB = S5K4E1_OTP_MSB;
	return otpDataMsb;

END_FAILED:
	printk("%s: check otp failed\n",__func__);
	return 0;
}

/****************************************************************************
* FunctionName: s5k4e1ga_read_start_current;
* Description : read start_current data ;
***************************************************************************/
static uint16_t s5k4e1ga_read_start_current(struct msm_sensor_ctrl_t *s_ctrl, uint16_t otpAddr, uint16_t MSBorLSB)
{
	uint16_t otpData = 0;
	uint16_t startCurrent = 0;

	/*read StartCurrent */
	otpData = s5k4e1ga_read_layer_data(s_ctrl,11, otpAddr, MSBorLSB);
	startCurrent|= otpData << 8;

	otpData = s5k4e1ga_read_layer_data(s_ctrl,12, otpAddr, MSBorLSB);
	startCurrent |= otpData << 4;

	otpData = s5k4e1ga_read_layer_data(s_ctrl,13, otpAddr, MSBorLSB);
	startCurrent |= otpData ;

	CDBG("%s: startCurrentt=%d \n", __func__, startCurrent);

	return startCurrent;
}

/****************************************************************************
* FunctionName: s5k4e1ga_read_macro_current;
* Description : read max_current data ;
***************************************************************************/
static uint16_t s5k4e1ga_read_macro_current(struct msm_sensor_ctrl_t *s_ctrl, uint16_t otpAddr, uint16_t MSBorLSB)
{
	uint16_t otpData = 0;
	uint16_t macroCurrent = 0;

	/*read MacroCurrent */
	otpData = s5k4e1ga_read_layer_data(s_ctrl,15, otpAddr, MSBorLSB);
	macroCurrent |= otpData << 8;

	otpData = s5k4e1ga_read_layer_data(s_ctrl,16, otpAddr, MSBorLSB);
	macroCurrent |= otpData << 4;

	otpData = s5k4e1ga_read_layer_data(s_ctrl,17, otpAddr, MSBorLSB);
	macroCurrent |= otpData ;

	CDBG("%s:  macroCurrent=%d \n", __func__, macroCurrent);

	return macroCurrent;
}

/****************************************************************************
* FunctionName: s5k4e1ga_otp_data_read;
* Description : read otp data interface;
***************************************************************************/
static int s5k4e1ga_otp_data_read(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t rg = 0;
	uint16_t bg = 0;
	uint16_t otpAddr = 0;
	uint16_t MSBorLSB = 0;

	uint16_t startCurrent = 0;
	uint16_t macroCurrent = 0;
    //if already read
	if(0 != g_s5k4e1ga_otp_awb[0])
	{
		return 0;
	}

	s5k4e1ga_cci_i2c_write(s_ctrl, 0x30F9, 0x0E);

	s5k4e1ga_cci_i2c_write(s_ctrl, 0x30FA, 0x0A);

	s5k4e1ga_cci_i2c_write(s_ctrl, 0x30FB, 0x71);

	s5k4e1ga_cci_i2c_write(s_ctrl, 0x30FB, 0x70);

	msleep(5);
	
	/*check awb */
	if(0== s5k4e1_otp_check_data(s_ctrl, &otpAddr, &MSBorLSB))
	{
		return -1;
	}

	printk("%s:otpAddr=0x%x, MSBorLSB=%d\n",__func__, otpAddr, MSBorLSB);

	rg = s5k4e1ga_read_rg(s_ctrl, otpAddr, MSBorLSB);//read r/g
	bg = s5k4e1ga_read_bg(s_ctrl, otpAddr, MSBorLSB);//read b/g
	startCurrent = s5k4e1ga_read_start_current(s_ctrl, otpAddr, MSBorLSB); //read start current
	macroCurrent = s5k4e1ga_read_macro_current(s_ctrl, otpAddr, MSBorLSB); //read macro current

	/* no otp data */
	if(0 == rg || 0 == bg)
	{
		printk("%s:otp data error: rg=0x%x bg=0x%x\n",__func__, rg, bg);
		return -1;
	}

	g_s5k4e1ga_otp_awb[0] = rg;
	g_s5k4e1ga_otp_awb[1] = bg;
	g_s5k4e1ga_otp_awb[2] = startCurrent;
	g_s5k4e1ga_otp_awb[3] = macroCurrent;

	
	g_modeID = s5k4e1ga_read_layer_data(s_ctrl,26, otpAddr, MSBorLSB);

	
	CDBG("%s rg = 0x%04x, bg = 0x%04x ,modeID = %d, DataTime:%d%d%d%d%d%d \n",
		__func__, rg, bg, g_modeID,
		s5k4e1ga_read_layer_data(s_ctrl,18, otpAddr, MSBorLSB), s5k4e1ga_read_layer_data(s_ctrl,19, otpAddr, MSBorLSB),
		s5k4e1ga_read_layer_data(s_ctrl,20, otpAddr, MSBorLSB), s5k4e1ga_read_layer_data(s_ctrl,21, otpAddr, MSBorLSB),
		s5k4e1ga_read_layer_data(s_ctrl,22, otpAddr, MSBorLSB), s5k4e1ga_read_layer_data(s_ctrl,23, otpAddr, MSBorLSB));

	return 0;
}

/****************************************************************************
* FunctionName: s5k4e1_liteon_affli_set_otp_info;
* Description : set otp data to sensor;
***************************************************************************/
static int s5k4e1_liteon_affli_set_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
    int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
    int rg,bg;
    int32_t rc = -EFAULT;
    rc = s5k4e1ga_otp_data_read(s_ctrl);
    if(rc < 0)//read fail
    {
        CDBG("%s read otp failed \n", __func__);
    }

    if(0 == g_s5k4e1ga_otp_awb[0])//no data
    {
        CDBG("%s: no otp data\n",__func__);
        return -1;
    }
    
    rg = g_s5k4e1ga_otp_awb[0];
    bg = g_s5k4e1ga_otp_awb[1];

    //calculate G gain
    //0x100 = 1x gain
    if(bg < BG_Ratio_Typical) {
        if (rg< RG_Ratio_Typical) {
            // current_otp.bg_ratio < BG_Ratio_typical &&  
            // current_otp.rg_ratio < RG_Ratio_typical
            G_gain = 0x100;
            B_gain = 0x100 * BG_Ratio_Typical / bg;
            R_gain = 0x100 * RG_Ratio_Typical / rg; 
        }
        else {
            // current_otp.bg_ratio < BG_Ratio_typical &&  
            // current_otp.rg_ratio >= RG_Ratio_typical
            R_gain = 0x100;
            G_gain = 0x100 * rg / RG_Ratio_Typical;
            B_gain = G_gain * BG_Ratio_Typical /bg;
        }
    }
    else {
        if (rg < RG_Ratio_Typical) {
            // current_otp.bg_ratio >= BG_Ratio_typical &&  
            // current_otp.rg_ratio < RG_Ratio_typical
            B_gain = 0x100;
            G_gain = 0x100 * bg / BG_Ratio_Typical;
            R_gain = G_gain * RG_Ratio_Typical / rg;
        }
        else {
            // current_otp.bg_ratio >= BG_Ratio_typical &&  
            // current_otp.rg_ratio >= RG_Ratio_typical
            G_gain_B = 0x100 * bg / BG_Ratio_Typical;
            G_gain_R = 0x100 * rg / RG_Ratio_Typical;

            if(G_gain_B > G_gain_R ) {
                B_gain = 0x100;
                G_gain = G_gain_B;
                R_gain = G_gain * RG_Ratio_Typical /rg;
            }
            else {
                R_gain = 0x100;
                G_gain = G_gain_R;
                B_gain = G_gain * BG_Ratio_Typical / bg;
            }
        }    
    }

    CDBG("%s: R_gain =0x%04x, G_gain =0x%04x, B_gain =0x%04x \n", __func__,R_gain,G_gain,B_gain);
    CDBG("%s: otp r_g = 0x%04x, b_g = 0x%04x\n",__func__,rg,bg);
    if (R_gain>0x100) {
       //digital_gain_R
        s5k4e1ga_cci_i2c_write(s_ctrl, 0x0210, R_gain>>8);
        s5k4e1ga_cci_i2c_write(s_ctrl, 0x0211, R_gain & 0x00ff);
    }

    if (G_gain>0x100) {
        //digital_gain_greenR
        s5k4e1ga_cci_i2c_write(s_ctrl, 0x020E, G_gain>>8);
        s5k4e1ga_cci_i2c_write(s_ctrl, 0x020F, G_gain & 0x00ff);

        //digital_gain_greenB
        s5k4e1ga_cci_i2c_write(s_ctrl, 0x0214, G_gain>>8);
        s5k4e1ga_cci_i2c_write(s_ctrl, 0x0215, G_gain & 0x00ff);
    }

    if (B_gain>0x100) {
        //digital_gain_B
        s5k4e1ga_cci_i2c_write(s_ctrl, 0x0212, B_gain>>8);
        s5k4e1ga_cci_i2c_write(s_ctrl, 0x0213, B_gain & 0x00ff);
    }

    return rc;
}
#endif

static struct msm_sensor_fn_t hw_s5k4e1_back_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA	
	.sensor_match_module = hw_s5k4e1_back_match_module,
	.sensor_write_otp = s5k4e1_liteon_affli_set_otp_info,
#endif
};

MODULE_DEVICE_TABLE(of, hw_s5k4e1_back_dt_match);

static struct platform_driver hw_s5k4e1_back_platform_driver = {
	.driver = {
		.name = "qcom,hw_s5k4e1_back",
		.owner = THIS_MODULE,
		.of_match_table = hw_s5k4e1_back_dt_match,
	},
};

static int32_t hw_s5k4e1_back_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(hw_s5k4e1_back_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hw_s5k4e1_back_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&hw_s5k4e1_back_platform_driver,
		hw_s5k4e1_back_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&hw_s5k4e1_back_i2c_driver);
}

static void __exit hw_s5k4e1_back_exit_module(void)
{
	if (hw_s5k4e1_back_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hw_s5k4e1_back_s_ctrl);
		platform_driver_unregister(&hw_s5k4e1_back_platform_driver);
	} else
		i2c_del_driver(&hw_s5k4e1_back_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hw_s5k4e1_back_s_ctrl = {
	.sensor_i2c_client = &hw_s5k4e1_back_sensor_i2c_client,
	.power_setting_array.power_setting = hw_s5k4e1_back_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hw_s5k4e1_back_power_setting),
	.msm_sensor_mutex = &hw_s5k4e1_back_mut,
	.sensor_v4l2_subdev_info = hw_s5k4e1_back_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hw_s5k4e1_back_subdev_info),
	.func_tbl = &hw_s5k4e1_back_func_tbl,
};

module_init(hw_s5k4e1_back_init_module);
module_exit(hw_s5k4e1_back_exit_module);
MODULE_DESCRIPTION("hw_s5k4e1_back");
MODULE_LICENSE("GPL v2");

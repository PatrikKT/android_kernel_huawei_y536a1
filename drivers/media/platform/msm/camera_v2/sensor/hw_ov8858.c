
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
#include <misc/app_info.h>

#define OV8858_FOXCONN_SENSOR_NAME "hw_ov8858"
DEFINE_MSM_MUTEX(hw_ov8858_mut);

#undef CDBG
#define OV8858_FOXCONN_DEBUG
#ifdef OV8858_FOXCONN_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
/* if support OTP or open OTP function, please define it */
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
#define OV8858_OTP_FEATURE
#endif

#ifdef OV8858_OTP_FEATURE

#define GROUP_OTP_EMPTY 0  //group is empty
#define GROUP_OTP_INVALID 1  //group is invalid
#define GROUP_OTP_VALID 2  //group is vaild

//OV8858 Foxconn 0x400 = 1x gain
#define OV8858_OTP_ONE_GAIN 0x400
/*module info & awb & lens shading & vcm flag reg address from otp criterion doc.*/
#define OV8858_OTP_MODULE_INFO_FLAG_ADDR 0x7010
#define OV8858_OTP_AWB_FLAG_ADDR 0x7020
#define OV8858_OTP_LENS_FLAG_ADDR 0x7033
#define OV8858_OTP_VCM_FLAG_ADDR 0x717E
/*lens shading num is 110*/
#define OV8858_MAX_OTP_LENS_NUM 110

//OV8858 has three groups: [1,2,3]
typedef enum ov8858_groups_count{
	GROUP_1 = 1,
	GROUP_2,
	GROUP_3,
	GROUP_MAX
}enum_ov8858_gruops;
/*OV8858 foxconn have four types otp*/
typedef enum ov8858_otp_type_e{
	MODULE_INFO_OTP,
	AWB_OTP,
	LENS_OTP,
	VCM_OTP
}enum_ov8858_otp_type;

typedef struct ov8858_otp_reg_addr {
    uint16_t start_address;
    uint16_t end_address;
}ST_ED_REG_ADDR;

//OTP info struct
typedef struct ov8858_otp_info {
    uint16_t module_integrator_id;
    uint16_t lens_id;
    uint16_t production_year;
    uint16_t production_month;
    uint16_t production_day;
    uint16_t rg_ratio;
    uint16_t bg_ratio;
    uint16_t gb_gr_ratio;
    //uint16_t light_rg;
    //uint16_t light_bg;
    uint16_t VCM_start;
    uint16_t VCM_end;
    uint8_t lenc[OV8858_MAX_OTP_LENS_NUM];
}st_ov8858_otp_info;

//Golden sensor typical ratio
static int RG_Ratio_Typical = 0x234;
static int BG_Ratio_Typical = 0x29A;

static ST_ED_REG_ADDR ov8858_module_info_otp_read_addr[GROUP_3] = {
    {0x7011,0x7015},
    {0x7016,0x701A},
    {0x701B,0x701F},
};
static ST_ED_REG_ADDR ov8858_awb_otp_read_addr[GROUP_3] = {
    {0x7021,0x7026},
    {0x7027,0x702C},
    {0x702D,0x7032},
};
static ST_ED_REG_ADDR ov8858_lens_otp_read_addr[GROUP_3] = {
    {0x7034,0x70A1},
    {0x70A2,0x71AF},
    {0x7110,0x717D},
};
static ST_ED_REG_ADDR ov8858_vcm_otp_read_addr[GROUP_3] = {
    {0x717F,0x7182},
    {0x7183,0x7186},
    {0x7187,0x718A},
};

//OTP info
static st_ov8858_otp_info g_ov8858_otp = {0};
#endif

static struct msm_sensor_ctrl_t ov8858_foxconn_s_ctrl;

static int hw_ov8858_module_id = 0;

static struct msm_sensor_power_setting ov8858_foxconn_power_setting[] = {
	//exchange the sequence of IOVDD and AVDD
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 22,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 20,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};



static struct v4l2_subdev_info ov8858_foxconn_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov8858_foxconn_i2c_id[] = {
	{OV8858_FOXCONN_SENSOR_NAME, (kernel_ulong_t)&ov8858_foxconn_s_ctrl},
	{ }
};

static int32_t msm_ov8858_foxconn_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov8858_foxconn_s_ctrl);
}

static struct i2c_driver ov8858_foxconn_i2c_driver = {
	.id_table = ov8858_foxconn_i2c_id,
	.probe  = msm_ov8858_foxconn_i2c_probe,
	.driver = {
		.name = OV8858_FOXCONN_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8858_foxconn_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov8858_foxconn_dt_match[] = {
	{.compatible = "qcom,hw_ov8858", .data = &ov8858_foxconn_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov8858_foxconn_dt_match);

static struct platform_driver ov8858_foxconn_platform_driver = {
	.driver = {
		.name = "qcom,hw_ov8858",
		.owner = THIS_MODULE,
		.of_match_table = ov8858_foxconn_dt_match,
	},
};

static int32_t ov8858_foxconn_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov8858_foxconn_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov8858_foxconn_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov8858_foxconn_platform_driver,
		ov8858_foxconn_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov8858_foxconn_i2c_driver);
}

static void __exit ov8858_foxconn_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov8858_foxconn_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov8858_foxconn_s_ctrl);
		platform_driver_unregister(&ov8858_foxconn_platform_driver);
	} else
		i2c_del_driver(&ov8858_foxconn_i2c_driver);
	return;
}

/****************************************************************************
* FunctionName: hw_ov8858_match_module;
* Description : add the project name ;
***************************************************************************/
static int hw_ov8858_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
	hw_ov8858_module_id = 1;
	
	/*add project name for the project menu*/
	s_ctrl->sensordata->sensor_name = "hw_ov8858_foxconn";
	strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060152FA-OV-F", strlen("23060152FA-OV-F")+1);

	pr_info("%s %d : hw_ov8858_match_module sensor_name=%s, sensor_project_name=%s \n",  __func__, __LINE__,
            s_ctrl->sensordata->sensor_name, s_ctrl->sensordata->sensor_info->sensor_project_name);
	pr_info("check module id from camera id PIN:OK \n");
	
	return 0;
}

#ifdef OV8858_OTP_FEATURE

/****************************************************************************
* FunctionName: ov8858_otp_write_i2c;
* Description : write otp info via i2c;
***************************************************************************/
int32_t ov8858_otp_write_i2c(struct msm_sensor_ctrl_t *s_ctrl, int32_t addr, uint16_t data)
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
* FunctionName: ov8858_otp_read_i2c;
* Description : read otp info via i2c;
***************************************************************************/
int32_t ov8858_otp_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, uint16_t *data)
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
// index: index of otp group will be checked: (1, 2, 3)
// check_address: otp flag address 
// return: 
// 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
***************************************************************************/
int32_t check_otp_group_index(struct msm_sensor_ctrl_t *s_ctrl, int index, uint16_t check_address)
{
    int32_t rc = 0;
    uint16_t data = 0;

    if(!s_ctrl || (index < GROUP_1 || index >= GROUP_MAX))
    {
        pr_err("%s: index = %d error! \n",__func__,index);
        return -1;
    }

    if(check_address != OV8858_OTP_MODULE_INFO_FLAG_ADDR && check_address != OV8858_OTP_AWB_FLAG_ADDR 
        && check_address != OV8858_OTP_LENS_FLAG_ADDR && check_address != OV8858_OTP_VCM_FLAG_ADDR)
    {
        pr_err("%s: check_address = 0x%x  error! \n",__func__,check_address);
        return -1;
    }

    // clear otp data buffer
    ov8858_otp_write_i2c(s_ctrl,check_address, 0x00);
    
    // enter otp read mode
    ov8858_otp_write_i2c(s_ctrl,0x3d84, 0xc0); // program disable, manual mode
    
    //partial mode OTP write start address
    ov8858_otp_write_i2c(s_ctrl,0x3d88, (check_address>>8));
    ov8858_otp_write_i2c(s_ctrl,0x3d89, (check_address & 0xff));
    
    // partial mode OTP write end address
    ov8858_otp_write_i2c(s_ctrl,0x3d8A, (check_address>>8));
    ov8858_otp_write_i2c(s_ctrl,0x3d8B, (check_address & 0xff));

    // open otp read function
    ov8858_otp_write_i2c(s_ctrl,0x3d81, 0x01); // read otp

    //delay 20ms
    msleep(20);

    //read otp data
    rc = ov8858_otp_read_i2c(s_ctrl,check_address, &data);
    if(rc < 0)
    {
        data = 0;
        pr_err("%s:%d read otp data fail\n",__func__,__LINE__);
    }
    else
    {
        //select group
        switch(index)
        {
            case GROUP_1:
                data = (data>>6) & 0x03;
            break;

            case GROUP_2:
                data = (data>>4) & 0x03;
            break;

            case GROUP_3:
                data = (data>>2) & 0x03;
            break;

            default:
                data = 0; 
                pr_err("%s:%d read otp data fail\n",__func__,__LINE__);
            break;
        }
    }

    // close otp read function
    ov8858_otp_write_i2c(s_ctrl,0x3d81, 0x00);
    
    // clear otp data buffer
    ov8858_otp_write_i2c(s_ctrl,check_address, 0x00);

    if (data == 0x00)   
    {
        rc = GROUP_OTP_EMPTY;
    }
    else if (data & 0x02)
    {
        rc = GROUP_OTP_INVALID;
    }
    else
    {
        /*find otp data success*/
        rc = GROUP_OTP_VALID;
    }

    return rc; 
}

/****************************************************************************
* FunctionName: ov8858_read_group_address_index;
* Description : find group index which real store otp data and convert the group index to address array index.
***************************************************************************/
int32_t ov8858_read_group_address_index(struct msm_sensor_ctrl_t *s_ctrl, uint16_t check_addr)
{
    int32_t ret = 0;
    int32_t index = 0;

    //get vaild gruop index
    for(index = 1; index < GROUP_MAX; index++)
    {
        ret = check_otp_group_index(s_ctrl,index,check_addr);
        if(ret == GROUP_OTP_VALID)
        {
            break;
        }
    }

    if(index >= GROUP_MAX)
    {
        pr_err("%s: check module info gruop index fail\n",__func__);   
        return -1;
    } 

    return (index-1);
}

/****************************************************************************
* FunctionName: ov8858_read_group_data;
* Description : read the specific otp data from group1 or group2 or group3. and fill the otp data to g_ov8858_otp
***************************************************************************/
int32_t ov8858_read_group_data(struct msm_sensor_ctrl_t *s_ctrl, enum_ov8858_otp_type otp_type)
{
    int32_t i = 0;
    int32_t rc = 0;
    int32_t index = 0;
    uint16_t msb_data = 0;
    uint16_t lsb_data = 0;
    uint16_t lens_temp = 0;
    uint16_t check_addr = 0;
    uint16_t start_address = 0;
    uint16_t end_address = 0;
    ST_ED_REG_ADDR *pcur_addr_array = NULL;

    switch(otp_type)
    {
        case MODULE_INFO_OTP:
            check_addr = OV8858_OTP_MODULE_INFO_FLAG_ADDR;
            pcur_addr_array = ov8858_module_info_otp_read_addr;
        break;
        
        case AWB_OTP:
            check_addr = OV8858_OTP_AWB_FLAG_ADDR;
            pcur_addr_array = ov8858_awb_otp_read_addr;
        break;
        
        case LENS_OTP:
            check_addr = OV8858_OTP_LENS_FLAG_ADDR;
            pcur_addr_array = ov8858_lens_otp_read_addr;
        break;
        
        case VCM_OTP:
            check_addr = OV8858_OTP_VCM_FLAG_ADDR;
            pcur_addr_array = ov8858_vcm_otp_read_addr;
        break;

        default:
            pr_err("%s: otp type error \n",__func__);
        return -1;
    }

    index = ov8858_read_group_address_index(s_ctrl,check_addr);
    if(index < 0 || index >= GROUP_3)
    {
       pr_err("%s: otp_type=%d fail index = %d\n",__func__,otp_type,index);
       return -1;
    }

    pr_info("%s: otp_type=%d index:%d \n", __func__,otp_type,index);

    start_address = pcur_addr_array[index].start_address;
    end_address = pcur_addr_array[index].end_address;

    //clear otp data buffer
    for(i = start_address; i <= end_address; i++)
    {
        ov8858_otp_write_i2c(s_ctrl,i, 0x00);
    }

    // enter otp read mode
    ov8858_otp_write_i2c(s_ctrl,0x3d84, 0xc0); // program disable, manual mode

    //partial mode OTP write start address
    ov8858_otp_write_i2c(s_ctrl,0x3d88, (start_address>>8));
    ov8858_otp_write_i2c(s_ctrl,0x3d89, (start_address & 0xff));

    // partial mode OTP write end address
    ov8858_otp_write_i2c(s_ctrl,0x3d8A, (end_address>>8));
    ov8858_otp_write_i2c(s_ctrl,0x3d8B, (end_address & 0xff));

    // enable otp read function
    ov8858_otp_write_i2c(s_ctrl,0x3d81, 0x01); // read otp

    //delay 20ms
    msleep(20);

    if(otp_type == MODULE_INFO_OTP)
    {   
        ov8858_otp_read_i2c(s_ctrl,start_address, &g_ov8858_otp.production_year);
        ov8858_otp_read_i2c(s_ctrl,start_address+1, &g_ov8858_otp.production_month);
        ov8858_otp_read_i2c(s_ctrl,start_address+2, &g_ov8858_otp.production_day);
        ov8858_otp_read_i2c(s_ctrl,start_address+3, &g_ov8858_otp.lens_id);
        ov8858_otp_read_i2c(s_ctrl,start_address+4, &g_ov8858_otp.module_integrator_id);
    }
    else if(otp_type == AWB_OTP)
    {
        ov8858_otp_read_i2c(s_ctrl,start_address, &msb_data); //rg msb
        ov8858_otp_read_i2c(s_ctrl,start_address + 1, &lsb_data); //rg lsb
        g_ov8858_otp.rg_ratio = ((msb_data & 0xFF) << 8) | (lsb_data & 0xFF);
        
        ov8858_otp_read_i2c(s_ctrl,start_address + 2, &msb_data); //bg msb
        ov8858_otp_read_i2c(s_ctrl,start_address + 3, &lsb_data); //bg lsb
        g_ov8858_otp.bg_ratio = ((msb_data & 0xFF) << 8) | (lsb_data & 0xFF);
        
        ov8858_otp_read_i2c(s_ctrl,start_address + 4, &msb_data); //gb_gr msb
        ov8858_otp_read_i2c(s_ctrl,start_address + 5, &lsb_data); //gb_gr lsb
        g_ov8858_otp.gb_gr_ratio= ((msb_data & 0xFF) << 8) | (lsb_data & 0xFF);
    }
    else if(otp_type == LENS_OTP)
    {
        for(i=0;i<OV8858_MAX_OTP_LENS_NUM;i++) {
            ov8858_otp_read_i2c(s_ctrl,start_address+i, &lens_temp);
            g_ov8858_otp.lenc[i] = (uint8_t)(lens_temp & 0xFF);
        }
    }
    else if(otp_type == VCM_OTP)
    {
        ov8858_otp_read_i2c(s_ctrl,start_address, &msb_data); //vcm start code msb
        ov8858_otp_read_i2c(s_ctrl,start_address + 1, &lsb_data); //vcm start code lsb
        g_ov8858_otp.VCM_start = ((msb_data & 0xFF) << 2) | ((lsb_data>>6) & 0x03);

        ov8858_otp_read_i2c(s_ctrl,start_address + 2, &msb_data); //vcm max current msb
        ov8858_otp_read_i2c(s_ctrl,start_address + 3, &lsb_data); //vcm max curren lsb
        g_ov8858_otp.VCM_end = ((msb_data & 0xFF) << 2) | ((lsb_data>>6) & 0x03); 
    }
    else
    {
        pr_err("%s: otp type error! \n",__func__);
        rc = -1;
    }

    // close otp read function
    ov8858_otp_write_i2c(s_ctrl,0x3d81, 0x00);

    //clear otp data buffer
    for(i = start_address; i <= end_address; i++)
    {
        ov8858_otp_write_i2c(s_ctrl,i, 0x00);
    }

    return rc;
}

/****************************************************************************
* FunctionName: ov8858_read_otp_data;
* Description : read all otp data info;
* return value:
* 0 means read otp info succeed.
* -1 means read otp info failed, should not write otp.
***************************************************************************/
int32_t ov8858_read_otp_data(struct msm_sensor_ctrl_t *s_ctrl)
{
    int index = 0;
    int ret = 0;
    
    if(!s_ctrl)
    {
        pr_err("%s:%d fail\n",__func__,__LINE__);
        return -1;
    }

    //read module info otp data
    ret = ov8858_read_group_data(s_ctrl,MODULE_INFO_OTP);
    if(ret < 0)
    {
        pr_err("%s: read module info otp data fail\n",__func__);   
        return ret;  
    }

    //read awb otp data
    ret = ov8858_read_group_data(s_ctrl,AWB_OTP);
    if(ret < 0)
    {
        pr_err("%s: read module info otp data fail\n",__func__);   
        return ret;  
    }
    
    //read lens otp data
    ret = ov8858_read_group_data(s_ctrl,LENS_OTP);
    if(ret < 0)
    {
        pr_err("%s: read module info otp data fail\n",__func__);   
        return ret;  
    }

    //read vcm otp data
    ret = ov8858_read_group_data(s_ctrl,VCM_OTP);
    if(ret < 0)
    {
        pr_err("%s: read module info otp data fail\n",__func__);   
        return ret;  
    }
   
    pr_info("%s: year=%d month=%d day=%d product=%d moduleid=%d rg=%d bg=%d gb_gr=%d vcm_start=%d vcm_end=%d \n", __func__,
    g_ov8858_otp.production_year,g_ov8858_otp.production_month,g_ov8858_otp.production_day,g_ov8858_otp.lens_id,g_ov8858_otp.module_integrator_id,
    g_ov8858_otp.rg_ratio,g_ov8858_otp.bg_ratio,g_ov8858_otp.gb_gr_ratio,g_ov8858_otp.VCM_start,g_ov8858_otp.VCM_end);

    //debug info test
    pr_info("%s: ---------lens shading otp data start----------\n",__func__);
    for(index = 0; index <OV8858_MAX_OTP_LENS_NUM; index++)
    {
        pr_info("[%d]=0x%02x \n",index,g_ov8858_otp.lenc[index]);
    }
    pr_info("%s: ---------lens shading otp data start----------\n",__func__);

    return ret;
}

/****************************************************************************
* FunctionName: ov8858_update_awb_gain;
* Description : write R_gain,G_gain,B_gain to otp;
* 0x400 =1x Gain
* 0 means write WB info succeed.
* -1 means write WB info failed.
***************************************************************************/
int32_t ov8858_update_awb_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint16_t R_gain=0, G_gain=0, B_gain=0, G_gain_R=0, G_gain_B=0;
    
    if(g_ov8858_otp.rg_ratio == 0 || g_ov8858_otp.bg_ratio == 0)
    {
        pr_err("%s: rg_ratio=%d bg_ratio=%d fail\n",__func__,g_ov8858_otp.rg_ratio,g_ov8858_otp.bg_ratio);
        return -1;
    }

    //calculate G gain
    //0x400 = 1x gain
    if(g_ov8858_otp.bg_ratio < BG_Ratio_Typical) {
        if (g_ov8858_otp.rg_ratio< RG_Ratio_Typical) {
            // current_otp.bg_ratio < BG_Ratio_typical &&
            // current_otp.rg_ratio < RG_Ratio_typical
            G_gain = 0x400;
            B_gain = 0x400 * BG_Ratio_Typical / g_ov8858_otp.bg_ratio;
            R_gain = 0x400 * RG_Ratio_Typical / g_ov8858_otp.rg_ratio;
        }
        else {
            // current_otp.bg_ratio < BG_Ratio_typical &&
            // current_otp.rg_ratio >= RG_Ratio_typical
            R_gain = 0x400;
            G_gain = 0x400 * g_ov8858_otp.rg_ratio / RG_Ratio_Typical;
            B_gain = G_gain * BG_Ratio_Typical /g_ov8858_otp.bg_ratio;
        }
    }
    else {
        if (g_ov8858_otp.rg_ratio < RG_Ratio_Typical) {
            // current_otp.bg_ratio >= BG_Ratio_typical &&
            // current_otp.rg_ratio < RG_Ratio_typical
            B_gain = 0x400;
            G_gain = 0x400 * g_ov8858_otp.bg_ratio / BG_Ratio_Typical;
            R_gain = G_gain * RG_Ratio_Typical / g_ov8858_otp.rg_ratio;
        }
        else {
            // current_otp.bg_ratio >= BG_Ratio_typical &&
            // current_otp.rg_ratio >= RG_Ratio_typical
            G_gain_B = 0x400 * g_ov8858_otp.bg_ratio / BG_Ratio_Typical;
            G_gain_R = 0x400 * g_ov8858_otp.rg_ratio / RG_Ratio_Typical;
            if(G_gain_B > G_gain_R ) {
                B_gain = 0x400;
                G_gain = G_gain_B;
                R_gain = G_gain * RG_Ratio_Typical /g_ov8858_otp.rg_ratio;
            }
            else {
                R_gain = 0x400;
                G_gain = G_gain_R;
                B_gain = G_gain * BG_Ratio_Typical / g_ov8858_otp.bg_ratio;
            }
        }
    }

    if (R_gain>0x400) {
        ov8858_otp_write_i2c(s_ctrl,0x5018, R_gain>>8);
        ov8858_otp_write_i2c(s_ctrl,0x5018, R_gain & 0x00ff);
    }
    if (G_gain>0x400) {
        ov8858_otp_write_i2c(s_ctrl,0x501A, G_gain>>8);
        ov8858_otp_write_i2c(s_ctrl,0x501B, G_gain & 0x00ff);
    }
    if (B_gain>0x400) {
        ov8858_otp_write_i2c(s_ctrl,0x501C, B_gain>>8);
        ov8858_otp_write_i2c(s_ctrl,0x501D, B_gain & 0x00ff);
    }

    pr_info("%s: R_gain=%d, G_gain=%d, B_gain=%d \n",__func__,R_gain,G_gain,B_gain);

    return 0;
}

/****************************************************************************
* FunctionName: ov8858_update_lenc_otp;
* Description : write 110 lens shading otp data to sensor;
***************************************************************************/
int32_t ov8858_update_lenc_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t i = 0;
    uint16_t temp = 0;
    
    ov8858_otp_read_i2c(s_ctrl,0x5000, &temp);
    temp = 0x80 | temp;
    ov8858_otp_write_i2c(s_ctrl, 0x5000, temp);
    
    for(i=0;i<OV8858_MAX_OTP_LENS_NUM;i++) {
        ov8858_otp_write_i2c(s_ctrl, 0x5800 + i, g_ov8858_otp.lenc[i]);
    }
    
    return 0;
}

/****************************************************************************
* FunctionName: ov8858_foxconn_get_otp_info;
* Description : get otp info from sensor;
***************************************************************************/
int32_t ov8858_foxconn_get_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	CDBG("Get ov8858 foxconn OTP info Enter\n");

	//set sensor mode:Active
	ov8858_otp_write_i2c(s_ctrl, 0x100, 0x1);
	
	rc = ov8858_read_otp_data(s_ctrl);
	
	usleep_range(5000, 6000);

	//set sensor mode:Standby
	ov8858_otp_write_i2c(s_ctrl, 0x100, 0x0);

	CDBG("Get ov8858 foxconn OTP info Exit\n");

	return rc;
}

/****************************************************************************
* FunctionName: ov8858_foxconn_set_otp_info;
* Description : set otp data to sensor;
* call this function after OV8858 initialization 
***************************************************************************/
static int32_t ov8858_foxconn_set_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	static int i_read_otp = 0;
	
	//Get otp info on the first time
	if ( 0 == i_read_otp )
	{		
		rc = ov8858_foxconn_get_otp_info(s_ctrl);
		
		if ( rc < 0 )
		{
			pr_err("%s:%d otp read failed.\n", __func__, __LINE__);
			return rc;
		}
		else
		{
			i_read_otp = 1;
		}
	}

	pr_info("Set ov8858 foxconn OTP info Enter\n");

    ov8858_otp_write_i2c(s_ctrl, 0x100, 0x1);
    
    ov8858_update_awb_otp(s_ctrl);

    ov8858_update_lenc_otp(s_ctrl);
	
	usleep_range(5000, 6000);

    ov8858_otp_write_i2c(s_ctrl, 0x100, 0x0);

	pr_info("Set ov8858 foxconn OTP info Exit\n");
	
	return rc;
}

#endif

static struct msm_sensor_fn_t ov8858_foxconn_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
    .sensor_match_module = hw_ov8858_match_module,
#ifdef OV8858_OTP_FEATURE
	//add otp function
	.sensor_write_otp = ov8858_foxconn_set_otp_info,
#endif
#endif
};

static struct msm_sensor_ctrl_t ov8858_foxconn_s_ctrl = {
	.sensor_i2c_client = &ov8858_foxconn_sensor_i2c_client,
	.power_setting_array.power_setting = ov8858_foxconn_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov8858_foxconn_power_setting),
	.msm_sensor_mutex = &hw_ov8858_mut,
	.sensor_v4l2_subdev_info = ov8858_foxconn_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8858_foxconn_subdev_info),
	.func_tbl = &ov8858_foxconn_sensor_func_tbl,
};

module_init(ov8858_foxconn_init_module);
module_exit(ov8858_foxconn_exit_module);
MODULE_DESCRIPTION("hw_ov8858");
MODULE_LICENSE("GPL v2");


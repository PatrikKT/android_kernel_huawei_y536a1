
#include "msm_sensor.h"
#define HW_IMX135_SENSOR_NAME "hw_imx135"
DEFINE_MSM_MUTEX(hw_imx135_mut);

/*#define HUAWEI_KERNEL_CAMERA_DEBUG*/
#undef CDBG
#ifdef HUAWEI_KERNEL_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_sensor_ctrl_t hw_imx135_s_ctrl;
static int8_t hw_imx135_module_id = -1;
#define HW_IMX135_LITEON 1
static struct msm_sensor_power_setting hw_imx135_power_setting[] = {
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
		.delay = 5,
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
static struct v4l2_subdev_info hw_imx135_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id hw_imx135_i2c_id[] = {
	{HW_IMX135_SENSOR_NAME, (kernel_ulong_t)&hw_imx135_s_ctrl},
	{ }
};

static int32_t msm_hw_imx135_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hw_imx135_s_ctrl);
}

static struct i2c_driver hw_imx135_i2c_driver = {
	.id_table = hw_imx135_i2c_id,
	.probe  = msm_hw_imx135_i2c_probe,
	.driver = {
		.name = HW_IMX135_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hw_imx135_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hw_imx135_dt_match[] = {
	{.compatible = "qcom,hw_imx135", .data = &hw_imx135_s_ctrl},
	{}
};

static uint8_t hw_imx135_cci_i2c_read(uint32_t addr)
{
    int32_t rc = -EFAULT;
    uint16_t data;
    rc = hw_imx135_s_ctrl.sensor_i2c_client->i2c_func_tbl->i2c_read(
        hw_imx135_s_ctrl.sensor_i2c_client, addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0)
    {
        pr_err("%s:Fail!rc = %d, addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
    }
    return (uint8_t)data;
}

static int32_t hw_imx135_cci_i2c_write(uint32_t addr, uint8_t data)
{
    int32_t rc = -EFAULT;

    rc = hw_imx135_s_ctrl.sensor_i2c_client->i2c_func_tbl->i2c_write(
        hw_imx135_s_ctrl.sensor_i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0)
    {
        pr_err("%s:Fail!rc = %d, addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
    }
    return rc;
}

/********************************OTP INFO START************************************/
typedef enum {
    //Lot No. data
    OTP_PAGE_0 = 0,
    //LSC
    OTP_PAGE_1,
    OTP_PAGE_2,
    OTP_PAGE_3,
    OTP_PAGE_4,
    OTP_PAGE_5,
    OTP_PAGE_6,
    OTP_PAGE_7,
    OTP_PAGE_8,
    //WB and ID
    OTP_PAGE_9,
    OTP_PAGE_10,
    OTP_PAGE_11,
    OTP_PAGE_12,
    //VCM
    OTP_PAGE_13,
    OTP_PAGE_14,
    OTP_PAGE_15,
    OTP_PAGE_16,
    OTP_PAGE_MAX,
} otp_page;

typedef enum {
    OTP_TYPE_ID_WB = 0,
    OTP_TYPE_VCM,
    OTP_TYPE_MAX,
} otp_type;

typedef struct opt_id_wb {
    /* Module information */
    uint8_t year;    /*0x3B04*/
    uint8_t month;    /*0x3B05*/
    uint8_t day;    /*0x3B06*/
    uint8_t cam_code;/*0x3B07*/
    uint8_t vend_ver;/*0x3B08*/

    /* WB parameters */
    uint8_t rg_h;    /*0x3B09*/
    uint8_t rg_l;    /*0x3B0A*/
    uint8_t bg_h;    /*0x3B0B*/
    uint8_t bg_l;    /*0x3B0C*/
    uint8_t gbgr_h;    /*0x3B0D*/
    uint8_t gbgr_l;    /*0x3B0E*/
} otp_id_wb_t;

/* VCM parameters */
typedef struct otp_vcm {
    uint8_t start_curr_h;    /*0x3B04*/
    uint8_t start_curr_l;    /*0x3B05*/
    uint8_t infinity_curr_h;/*0x3B06*/
    uint8_t infinity_curr_l;/*0x3B07*/
    uint8_t macro_curr_h;    /*0x3B08*/
    uint8_t macro_curr_l;    /*0x3B09*/
    uint8_t meter_curr_h;    /*0x3B0A*/
    uint8_t meter_curr_l;    /*0x3B0B*/
} otp_vcm_t;

#define IMX135_SENSOR_REG_LSC_ON    0x0700
#define IMX135_SENSOR_REG_LSC_ENABLE    0x4500
#define IMX135_SENSOR_REG_LSC_ADDR    0x4800
#define IMX135_SENSOR_REG_RAM_SEL_TOGGLE    0x3A63

#define IMX135_OTP_REG_MODE_ENABLE    0x3B00
#define IMX135_OTP_REG_STATUS_CHECK    0x3B01
#define IMX135_OTP_REG_PAGE_SELECT    0x3B02
#define IMX135_OTP_REG_CFA_FMT        0x3B2C
#define IMX135_OTP_REG_LOT_ID_START    0x3B24

#define IMX135_OTP_ID_WB_READ    (1 << 0)
#define IMX135_OTP_VCM_READ    (1 << 1)
#define IMX135_OTP_LSC_READ    (1 << 2)
#define IMX135_OTP_LSC_WRITED    (1 << 3)
#define IMX135_OTP_LSC_FILE_ERR    (1 << 4)

/* OTP lens shading parameters are 63*4[R-Gr-Gb-B] in toal, each parameter takes 2 bytes[9:0]. */
#define IMX135_OTP_LSC_SIZE    504
#define IMX135_OTP_LOTNO_SIZE    9
#define IMX135_OTP_LSC_LOTNO_SIZE    (IMX135_OTP_LSC_SIZE + IMX135_OTP_LOTNO_SIZE)

#define IMX135_OTP_LSC_FILE    "/data/lsc_param"

/* OTP parameters. */
static uint8_t hw_imx135_otp_flag = 0;
static uint8_t hw_imx135_otp_lsc_id = 0;
static otp_vcm_t hw_imx135_otp_vcm;
static otp_id_wb_t hw_imx135_otp_id_wb;
static uint8_t hw_imx135_otp_lsc_param[IMX135_OTP_LSC_LOTNO_SIZE] = {};
static uint8_t hw_imx135_otp_lotno[IMX135_OTP_LOTNO_SIZE] = {};

bool hw_imx135_otp_lotno_flag = false;

struct work_struct record_otp_lsc_work;

struct msm_sensor_dig_gain hw_imx135_otp_dig_gain;

/****************************************************************************
* FunctionName: hw_imx135_otp_open_page;
* Description : Open selected OTP page.;
***************************************************************************/
static bool hw_imx135_otp_open_page(uint8_t page)
{
    uint8_t loop = 0;
    uint16_t status = 0;

    CDBG("%s, %d:page = %d\n", __func__, __LINE__, page);
    if (page >= OTP_PAGE_MAX)
    {
        pr_err("%s:page%d over setting range\n", __func__, page);
        return false;
    }

    hw_imx135_cci_i2c_write(IMX135_OTP_REG_PAGE_SELECT, page); /* Select OTP page. */
    hw_imx135_cci_i2c_write(IMX135_OTP_REG_MODE_ENABLE, 0x01); /* Turn on OTP mode: 01 read mode;03 write mode*/
    udelay(2);

    /* Check OTP read ready status. */
    for (loop=0; loop<5; loop++)
    {
        status = hw_imx135_cci_i2c_read(IMX135_OTP_REG_STATUS_CHECK);/*bit0:1 read ready*/
        status &= 0x01;
        udelay(1);
        if (status)
            break;
    }
    if (!status)
    {
        pr_err("%s: Wait OTP page%d read ready status timeout\n", __func__, page);
        return false;
    }

    return true;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_check_page;
* Description : Check which page this parameters belong to.;
***************************************************************************/
static bool hw_imx135_otp_check_page(otp_type type)
{
    uint8_t start = 0;
    uint8_t end = 0;
    uint8_t page = 0;
    uint8_t val1 = 0;
    uint8_t val2 = 0;
    uint16_t temp = 0;

    switch (type)
    {
        case OTP_TYPE_ID_WB:
            /* Module ID and WB parameters take up Page9~Page12, and can be burned 4 times.*/
            start = OTP_PAGE_12;
            end = OTP_PAGE_9;

	    /* Find out valid OTP page. */
	    for (page = start; page >= end; page--)
	    {
	        if (hw_imx135_otp_open_page(page))
	        {
	            /* Check whether current page is valid. */
	            val1 = hw_imx135_cci_i2c_read(0x3B0F);
	            if (0xFF == val1)
	            {
	                pr_info("%s:Module, WB and LSC ID will be checked on page%d\n", __func__, page);
	                break;
	            }
	        }
	    }

            break;
        case OTP_TYPE_VCM:
            /* AF VCM parameters take up Page13~Page16, and can be burned 4 times. */
            start = OTP_PAGE_16;
            end = OTP_PAGE_13;

        /* Find out valid OTP page. */
        for (page = start; page >= end; page--)
        {
            if (hw_imx135_otp_open_page(page))
            {
                temp = 0;
                /* Check whether current page is valid. */
                val1 = hw_imx135_cci_i2c_read(0x3B04);
                val2 = hw_imx135_cci_i2c_read(0x3B05);
                temp = (val1 <<8) | val2;
                if (temp)
                {
                    pr_info("%s:VCM data will be checked on page%d\n", __func__, page);
                    break;
                }
            }
        }

            break;
        default:
            pr_err("%s: Unsupported OTP type %d\n", __func__, type);
            return false;
    }

    if (page < end)
    {
        pr_err("%s: None OTP data written!\n", __func__);
        return false;
    }

    return true;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_read_id_wb;
* Description : Get module ID and WB parameters form OTP.;
* Input       : otp_id_wb_t;
* Output      : NA;
* ReturnValue : bool;
* Other       : NA;
***************************************************************************/
static bool hw_imx135_otp_read_id_wb(void)
{
    if (!hw_imx135_otp_check_page(OTP_TYPE_ID_WB))
    {
        pr_err("%s: Check OTP_TYPE_ID_WB page fail\n", __func__);
        return false;
    }

    /* Read module information */
    hw_imx135_otp_id_wb.year = hw_imx135_cci_i2c_read(0x3B04);
    hw_imx135_otp_id_wb.month=hw_imx135_cci_i2c_read(0x3B05);
    hw_imx135_otp_id_wb.day=hw_imx135_cci_i2c_read(0x3B06);
    hw_imx135_otp_id_wb.cam_code=hw_imx135_cci_i2c_read(0x3B07);
    hw_imx135_otp_id_wb.vend_ver=hw_imx135_cci_i2c_read(0x3B08);

    CDBG("%s: Module yield date: %x-%x-%x\n", __func__,
        hw_imx135_otp_id_wb.year, hw_imx135_otp_id_wb.month, hw_imx135_otp_id_wb.day);
    CDBG("%s: Module code is: %#x, vendor and version info is %#x\n", __func__,
        hw_imx135_otp_id_wb.cam_code, hw_imx135_otp_id_wb.vend_ver);

        /* Read WB parameters */
    hw_imx135_otp_id_wb.rg_h=hw_imx135_cci_i2c_read(0x3B09);
    hw_imx135_otp_id_wb.rg_l=hw_imx135_cci_i2c_read(0x3B0A);
    hw_imx135_otp_id_wb.bg_h=hw_imx135_cci_i2c_read(0x3B0B);
    hw_imx135_otp_id_wb.bg_l=hw_imx135_cci_i2c_read(0x3B0C);
    hw_imx135_otp_id_wb.gbgr_h=hw_imx135_cci_i2c_read(0x3B0D);
    hw_imx135_otp_id_wb.gbgr_l=hw_imx135_cci_i2c_read(0x3B0E);

    /* Read lens shading ID */
    hw_imx135_otp_lsc_id=hw_imx135_cci_i2c_read(0x3B0F);
    CDBG("%s:hw_imx135_otp_lsc_id=%d\n", __func__, hw_imx135_otp_lsc_id);

    /* OTP module info and WB param is read  */
    hw_imx135_otp_flag |= IMX135_OTP_ID_WB_READ;
    CDBG("%s:OTP read ID_WB done\n", __func__);
    return true;
}
/***************************************************************************
 * FunctionName: hw_imx135_update_wb_gain_to_sensor;
 * Description : NA;
 * Input       : dig_gain_R: red gain of sensor MWB, 0x100 = 1;
 *               dig_gain_G: green gain of sensor MWB, 0x100 = 1;
 *               dig_gain_B: blue gain of sensor MWB, 0x100 = 1;
 ***************************************************************************/
void hw_imx135_set_digital_gain(uint16_t dig_gain_R, uint16_t dig_gain_G, uint16_t dig_gain_B)
{
    hw_imx135_cci_i2c_write(0x020E, (dig_gain_G>>8)&0x0F);    //DIG_GAIN_GR[11:8]
    hw_imx135_cci_i2c_write(0x020F, dig_gain_G&0xFF);    //DIG_GAIN_GR[7:0]

    hw_imx135_cci_i2c_write(0x0210, (dig_gain_R>>8)&0x0F);    //DIG_GAIN_R[11:8]
    hw_imx135_cci_i2c_write(0x0211, dig_gain_R&0xFF);    //DIG_GAIN_R[7:0]

    hw_imx135_cci_i2c_write(0x0212, (dig_gain_B>>8)&0x0F);    //DIG_GAIN_B[11:8]
    hw_imx135_cci_i2c_write(0x0213, dig_gain_B&0xFF);    //DIG_GAIN_B[7:0]

    hw_imx135_cci_i2c_write(0x0214, (dig_gain_G>>8)&0x0F);    //DIG_GAIN_GB[11:8]
    hw_imx135_cci_i2c_write(0x0215, dig_gain_G&0xFF);    //DIG_GAIN_GB[7:0]
}

/****************************************************************************
* FunctionName: sonyimx135_otp_set_wb;
* Description : Set WB parameters to ISP.;
***************************************************************************/
static void hw_imx135_otp_set_wb_gain(void)
{
    uint16_t rg_ratio, bg_ratio;
    uint16_t dig_gain_R, dig_gain_G, dig_gain_B, G_gain_R, G_gain_B;
    uint16_t RG_Ratio_typical;
    uint16_t BG_Ratio_typical;

    if (HW_IMX135_LITEON == hw_imx135_module_id)
    {
        /*the typical value should be got from vendor mass production and generally accord with GOLDEN SAMPLE*/
        RG_Ratio_typical = 0X220;//sample NO.151#;
        BG_Ratio_typical = 0X22D;//sample NO.151#;
    }
    else
    {
        RG_Ratio_typical = 0X220;
        BG_Ratio_typical = 0X22D;
    }

    /* R/G and B/G of current camera module is read out from sensor OTP */
    rg_ratio = ((hw_imx135_otp_id_wb.rg_h << 8) & 0xff00) | (hw_imx135_otp_id_wb.rg_l & 0xff);/* OTP_RG' */
    bg_ratio = ((hw_imx135_otp_id_wb.bg_h << 8) & 0xff00) | (hw_imx135_otp_id_wb.bg_l & 0xff);/* OTP_BG' */
    CDBG("%s:Read from otp rg_ratio=0x%x, bg_ratio=0x%x\n", __func__, rg_ratio, bg_ratio);

    //calculate digital gain, 0x100 == 1x gain
    if (bg_ratio < BG_Ratio_typical)
    {
        if (rg_ratio < RG_Ratio_typical)
        {
            // bg_ratio < BG_Ratio_typical && rg_ratio < RG_Ratio_typical
            dig_gain_G = 0x100;
            dig_gain_B = 0x100 * BG_Ratio_typical / bg_ratio;
            dig_gain_R = 0x100 * RG_Ratio_typical / rg_ratio;
        }
        else
        {
            // bg_ratio < BG_Ratio_typical && rg_ratio >= RG_Ratio_typical
            dig_gain_R = 0x100;
            dig_gain_G = 0x100 * rg_ratio / RG_Ratio_typical;
            dig_gain_B = dig_gain_G * BG_Ratio_typical / bg_ratio;
        }
    }
    else//bg_ratio > BG_Ratio_typical
    {
        if (rg_ratio < RG_Ratio_typical)
        {
            // bg_ratio >= BG_Ratio_typical && rg_ratio < RG_Ratio_typical
            dig_gain_B = 0x100;
            dig_gain_G = 0x100 * bg_ratio / BG_Ratio_typical;
            dig_gain_R = dig_gain_G * RG_Ratio_typical / rg_ratio;
        }
        else
        {
            // bg_ratio >= BG_Ratio_typical && rg_ratio >= RG_Ratio_typical
            G_gain_B = 0x100 * bg_ratio / BG_Ratio_typical;
            G_gain_R = 0x100 * rg_ratio / RG_Ratio_typical;

            if(G_gain_B > G_gain_R)
            {
                dig_gain_B = 0x100;
                dig_gain_G = G_gain_B;
                dig_gain_R = dig_gain_G * RG_Ratio_typical / rg_ratio;
            }
            else
            {
                dig_gain_R = 0x100;
                dig_gain_G = G_gain_R;
                dig_gain_B = dig_gain_G * BG_Ratio_typical / bg_ratio;
            }
        }
    }

    if (dig_gain_R<0x100)
        dig_gain_R = 0x100;
    if (dig_gain_G<0x100)
        dig_gain_G = 0x100;
    if (dig_gain_B<0x100)
        dig_gain_B = 0x100;

    hw_imx135_otp_dig_gain.gr_gain = dig_gain_G;
    hw_imx135_otp_dig_gain.r_gain = dig_gain_R;
    hw_imx135_otp_dig_gain.b_gain = dig_gain_B;
    hw_imx135_otp_dig_gain.gb_gain = dig_gain_G;

    CDBG("%s:dig_gain_R=0x%x, dig_gain_G=0x%x, dig_gain_B=0x%x\n", __func__, dig_gain_R, dig_gain_G, dig_gain_B);
    // write otp wb digital gain to sensor registers
    hw_imx135_set_digital_gain(dig_gain_R, dig_gain_G, dig_gain_B);
    CDBG("%s:OTP set WB gain OK\n", __func__);
    return;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_update_wb;
* Description : read WB info from otp and write to sensor registers;
***************************************************************************/
static bool hw_imx135_otp_update_wb(void)
{
    if ((hw_imx135_otp_flag & IMX135_OTP_ID_WB_READ)
        || hw_imx135_otp_read_id_wb())
    {
        hw_imx135_otp_set_wb_gain();
    }
    else
    {
        pr_err("%s: Read OTP Module info or WB parameters failed\n", __func__);
        return false;
    }

    CDBG("%s:OTP update wb done\n", __func__);
    return true;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_read_vcm;
* Description : Get AF motor parameters from OTP.;
***************************************************************************/
static bool hw_imx135_otp_read_vcm(void)
{
    if (!hw_imx135_otp_check_page(OTP_TYPE_VCM))
    {
        pr_err("%s: Check OTP_TYPE_VCM page fail\n", __func__);
        return false;
    }

    hw_imx135_otp_vcm.start_curr_h=hw_imx135_cci_i2c_read(0x3B04);
    hw_imx135_otp_vcm.start_curr_l=hw_imx135_cci_i2c_read(0x3B05);
    hw_imx135_otp_vcm.infinity_curr_h=hw_imx135_cci_i2c_read(0x3B06);
    hw_imx135_otp_vcm.infinity_curr_l=hw_imx135_cci_i2c_read(0x3B07);
    hw_imx135_otp_vcm.macro_curr_h=hw_imx135_cci_i2c_read(0x3B08);
    hw_imx135_otp_vcm.macro_curr_l=hw_imx135_cci_i2c_read(0x3B09);
    hw_imx135_otp_vcm.meter_curr_h=hw_imx135_cci_i2c_read(0x3B0A);
    hw_imx135_otp_vcm.meter_curr_l=hw_imx135_cci_i2c_read(0x3B0B);

    /* VCM param is read  */
    hw_imx135_otp_flag |= IMX135_OTP_VCM_READ;

    CDBG("%s:OTP read VCM done\n", __func__);
    return true;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_set_vcm;
* Description : Set AF motor parameters to ISP.;
***************************************************************************/
static void hw_imx135_otp_set_vcm(void)
{
    uint16_t start_curr = 0;
    uint16_t infinity_curr = 0;
    uint16_t macro_curr = 0;
    uint8_t temp_h = 0;
    uint8_t temp_l = 0;

    temp_h = hw_imx135_otp_vcm.start_curr_h;
    temp_l = hw_imx135_otp_vcm.start_curr_l;
    start_curr = (temp_h << 8) | temp_l;

    temp_h = hw_imx135_otp_vcm.infinity_curr_h;
    temp_l = hw_imx135_otp_vcm.infinity_curr_l;
    infinity_curr = (temp_h << 8) | temp_l;

    temp_h = hw_imx135_otp_vcm.macro_curr_h;
    temp_l = hw_imx135_otp_vcm.macro_curr_l;
    macro_curr = (temp_h << 8) | temp_l;

    CDBG("%s: start = 0x%x, infinity = 0x%x, macro = 0x%x\n", __func__,
        start_curr, infinity_curr, macro_curr);

    /*should write VCM info into af_main_camera_xxxx.h*/
    /*such as intial_code, but how???*/
    CDBG("%s:OTP set VCM OK[still not used]\n", __func__);
}

/****************************************************************************
* FunctionName: hw_imx135_otp_update_af;
* Description : read VCM info from otp and write to sensor registers;
***************************************************************************/
static bool hw_imx135_otp_update_af(void)
{
    if ((hw_imx135_otp_flag & IMX135_OTP_VCM_READ)
        || hw_imx135_otp_read_vcm())
    {
        hw_imx135_otp_set_vcm();
    }
    else
    {
        pr_err("%s: Read OTP AF VCM parameters failed\n", __func__);
        return false;
    }

    CDBG("%s:OTP update af done\n", __func__);
    return true;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_enable_lsc;
* Description : Enable LSC correct.;
***************************************************************************/
static void hw_imx135_otp_enable_lsc(bool enable)
{
    uint8_t lsc_on = 0x00;
    uint8_t lsc_enable = 0x00;
    uint8_t ram_sel_toggle = 0x00;

    CDBG("%s:enable = %d\n", __func__, enable);

    /* Open OTP LSC mode */
    if (enable)
    {
        lsc_on = 0x01;
        lsc_enable = 0x1F;
        ram_sel_toggle = 0x01;
    }

    hw_imx135_cci_i2c_write(IMX135_SENSOR_REG_LSC_ENABLE, lsc_enable);
    hw_imx135_cci_i2c_write(IMX135_SENSOR_REG_LSC_ON, lsc_on);
    hw_imx135_cci_i2c_write(IMX135_SENSOR_REG_RAM_SEL_TOGGLE, ram_sel_toggle);

}

/****************************************************************************
* FunctionName: hw_imx135_otp_lsc_file_exist;
* Description : Check if LSC parameter file exist;
***************************************************************************/
static bool hw_imx135_otp_lsc_file_exist(void)
{
    struct file *filp = NULL;

    filp = filp_open(IMX135_OTP_LSC_FILE, O_RDONLY, 0664);
    if (!IS_ERR_OR_NULL(filp))
    {
        filp_close(filp, NULL);
        CDBG("%s:OTP file exist\n", __func__);
        return true;
    }

    CDBG("%s:OTP file does not exist\n", __func__);
    return false;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_write_file;
* Description : Write OTP LSC parameters to file;
***************************************************************************/
static bool hw_imx135_otp_write_file(void)
{
    bool ret = true;
    mm_segment_t fs;
    struct file *filp = NULL;

    filp = filp_open(IMX135_OTP_LSC_FILE, O_CREAT|O_WRONLY, 0664);
    if (IS_ERR_OR_NULL(filp))
    {
        ret = false;
        pr_err("%s: OTP failed to open lsc_param file!\n", __func__);
        goto ERROR;
    }

    fs = get_fs();
    set_fs(KERNEL_DS);
    if (IMX135_OTP_LSC_LOTNO_SIZE != vfs_write(filp, hw_imx135_otp_lsc_param,
        IMX135_OTP_LSC_LOTNO_SIZE, &filp->f_pos))
    {
        set_fs(fs);
        ret = false;
        pr_err("%s: OTP write lsc_param file error!\n", __func__);
        goto ERROR;
    }
    set_fs(fs);
    CDBG("%s: OTP write lsc_param file OK\n", __func__);

ERROR:
    if (NULL != filp)
    {
        filp_close(filp, NULL);
    }
    if (ret)
    {
        hw_imx135_otp_flag |= IMX135_OTP_LSC_WRITED;
        hw_imx135_otp_flag &= ~IMX135_OTP_LSC_FILE_ERR;
    }
    return ret;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_read_file;
* Description : Read OTP LSC parameters to file;
***************************************************************************/
static bool hw_imx135_otp_read_file(void)
{
    bool ret = true;
    mm_segment_t fs;
    struct file *filp = NULL;
    int i;

    filp = filp_open(IMX135_OTP_LSC_FILE, O_RDONLY, 0664);
    if (IS_ERR_OR_NULL(filp))
    {
        pr_err("%s: failed to open lsc_param file\n", __func__);
        ret = false;
        goto ERROR;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    if (IMX135_OTP_LSC_LOTNO_SIZE != vfs_read(filp, hw_imx135_otp_lsc_param, IMX135_OTP_LSC_LOTNO_SIZE, &filp->f_pos))
    {
        set_fs(fs);
        ret = false;
        pr_err("%s: OTP read lsc_param file error\n", __func__);
        goto ERROR;
    }
    set_fs(fs);
    CDBG("%s: OTP read lsc_param file OK.\n", __func__);

    /*compare current module's lotno with the reserved moudle*/
    for(i=0;i<IMX135_OTP_LOTNO_SIZE;i++)
    {
        if(hw_imx135_otp_lotno[i] != hw_imx135_otp_lsc_param[IMX135_OTP_LSC_SIZE + i])
        {
            pr_info("%s:module changed, read lsc from OTP again!!\n", __func__);
            ret =  false;
            break;
        }
    }

ERROR:
    if (NULL != filp)
    {
        filp_close(filp, NULL);
    }

    if (ret)
    {
        hw_imx135_otp_flag |= IMX135_OTP_LSC_READ;
        hw_imx135_otp_flag |= IMX135_OTP_LSC_WRITED;
    }
    else
    {
        hw_imx135_otp_flag &= ~IMX135_OTP_LSC_READ;
        hw_imx135_otp_flag &= ~IMX135_OTP_LSC_WRITED;
        hw_imx135_otp_flag |= IMX135_OTP_LSC_FILE_ERR;
    }

    return ret;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_read_lsc;
* Description : Read lens shading parameters form OTP.;
***************************************************************************/
static bool hw_imx135_otp_read_lsc(void)
{
    uint8_t *lsc_param = NULL;
    uint8_t page = 0;
    uint16_t addr_start = 0;
    uint16_t addr_end = 0;
    int i;

    /* If lens shading ID is not read, then read it first. */
    if (0xFF != hw_imx135_otp_lsc_id)
    {
        if (hw_imx135_otp_check_page(OTP_TYPE_ID_WB))
        {
            hw_imx135_otp_lsc_id=hw_imx135_cci_i2c_read(0x3B0F);
        }
        else
        {
            pr_err("%s: Check OTP_TYPE_ID_WB page fail\n", __func__);
            return false;
        }
    }

    /* Lens shading burned OK. */
    if (0xFF == hw_imx135_otp_lsc_id) {
        if (hw_imx135_otp_lsc_file_exist() && hw_imx135_otp_read_file())
        {
            /* LSC param is already read  */
            CDBG("%s: Read LSC param form file OK\n", __func__);
            return true;
        }

        lsc_param = &hw_imx135_otp_lsc_param[0];
        memset(hw_imx135_otp_lsc_param, 0, sizeof(hw_imx135_otp_lsc_param));

        /* Lens shading parameters take up Page1~Page8 of OTP, and can be burned only once. */
        for (page = OTP_PAGE_1; page <= OTP_PAGE_8; page++) {
            if(hw_imx135_otp_open_page(page))
            {
                /* Get lens shading parameters from each page. */
                addr_start = 0x3B04;
                if (OTP_PAGE_8 == page)
                    addr_end = 0x3B3B; /* In page8, the valid data range is 0x3B04~0x3B3B. */
                else
                    addr_end = 0x3B43; /* From page1 to page7, the valid data range is 0x3B04~0x3B43. */

                do{
                    *lsc_param=hw_imx135_cci_i2c_read(addr_start);
                    addr_start++;
                    lsc_param++;
                } while (addr_start <= addr_end);
            }
            else
            {
                pr_err("%s:LSC OTP open page%d fail\n", __func__, page);
                return false;
            }
        }
        pr_info("%s:Read LSC from sensor OK\n", __func__);
    }
    else
    {
        pr_err("%s:hw_imx135_otp_lsc_id wrong!not equal to 0xFF!\n", __func__);
        return false;
    }

    /*append the module lot No. to LSC param*/
    for(i=0;i<IMX135_OTP_LOTNO_SIZE;i++)
    {
        *(lsc_param + i) = hw_imx135_otp_lotno[i];
    }

    /* LSC param is read  */
    hw_imx135_otp_flag |= IMX135_OTP_LSC_READ;

    return true;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_set_lsc;
* Description : Set lens shading parameters to sensor registers.;
***************************************************************************/
static bool hw_imx135_otp_set_lsc(void)
{
    uint8_t *lsc_param = NULL;
    int i = 0;

    /* Lens shading parameters are burned OK. */
    if (0xFF == hw_imx135_otp_lsc_id)
    {
        /* Get parameters from static array, which read from OTP. */
        lsc_param = hw_imx135_otp_lsc_param;
    }
    else
    {
        pr_err("%s: Unsupported lens shading ID, %#x\n", __func__, hw_imx135_otp_lsc_id);
        return false;
    }

    /* Write lens shading parameters to sensor registers. */
    for (i=0; i<IMX135_OTP_LSC_SIZE; i++)
    {
        hw_imx135_cci_i2c_write(IMX135_SENSOR_REG_LSC_ADDR+i, *(lsc_param+i));
    }

    CDBG("%s: OTP set LSC OK.\n", __func__);

    return true;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_record_lsc_func;
* Description : Recored OTP LSC parameter function, schedule by workqueue.;
***************************************************************************/
static void hw_imx135_otp_record_lsc_func(struct work_struct *work)
{
    if ((hw_imx135_otp_flag & IMX135_OTP_LSC_FILE_ERR)
        || (!hw_imx135_otp_lsc_file_exist()))
    {
        if(!hw_imx135_otp_write_file())
        {
            pr_err("%s:hw_imx135_otp_write_file fail\n", __func__);
        }
        CDBG("%s: OTP flag is: %#x\n", __func__, hw_imx135_otp_flag);
    }
}

/****************************************************************************
* FunctionName: hw_imx135_otp_update_lsc;
* Description : read lsc info from otp and write to sensor registers;
***************************************************************************/
static bool hw_imx135_otp_update_lsc(void)
{
    int i;

    /*read and reserve the module lot No. for LSC param comparation*/
    if(!hw_imx135_otp_lotno_flag)
    {
        if(hw_imx135_otp_open_page(OTP_PAGE_0))
        {
            memset(hw_imx135_otp_lotno, 0, sizeof(hw_imx135_otp_lotno));
            for(i=0;i<IMX135_OTP_LOTNO_SIZE;i++)
            {
                hw_imx135_otp_lotno[i] = hw_imx135_cci_i2c_read(IMX135_OTP_REG_LOT_ID_START+i);
            }
            hw_imx135_otp_lotno_flag = true;
        }
    }

    /*Disable LSC*/
    hw_imx135_otp_enable_lsc(false);

    /*Set LSC if read lsc parameter succesfully*/
    if ((hw_imx135_otp_flag & IMX135_OTP_LSC_READ)
        || hw_imx135_otp_read_lsc())
    {
        if(!hw_imx135_otp_set_lsc())
        {
            pr_err("hw_imx135_otp_set_lsc fail\n");
            return false;
        }
        /*Enable LSC*/
        hw_imx135_otp_enable_lsc(true);
    }
    else
    {
        pr_err("%s: Read LSC fail!\n", __func__);
        return false;
    }

    CDBG("%s: OTP flag = %#x\n", __func__, hw_imx135_otp_flag);
    if (!(hw_imx135_otp_flag & IMX135_OTP_LSC_WRITED))
    {
        schedule_work(&record_otp_lsc_work);
    }

    pr_info("%s: OTP update LSC done\n", __func__);
    return true;
}

/****************************************************************************
* FunctionName: hw_imx135_set_otp_info;
* Description : read info from otp and write to sensor registers;
***************************************************************************/
static int hw_imx135_set_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
    INIT_WORK(&record_otp_lsc_work, hw_imx135_otp_record_lsc_func);

    if(!hw_imx135_otp_update_wb())
    {
        pr_err("%s:hw_imx135_otp_update_wb fail\n", __func__);
        return -1;
    }
    if(!hw_imx135_otp_update_af())
    {
        pr_err("%s:hw_imx135_otp_update_af fail\n", __func__);
        return -1;
    }
    if(!hw_imx135_otp_update_lsc())
    {
        pr_err("%s:hw_imx135_otp_update_lsc fail\n", __func__);
        return -1;
    }
    s_ctrl->sensordata->sensor_info->sensor_otp_dig_gain = hw_imx135_otp_dig_gain;
    return 0;
}

/****************************************************************************
* FunctionName: hw_imx135_otp_match_module;
* Description : Check sensor module info from otp info;
***************************************************************************/
static bool hw_imx135_otp_match_module(void)
{
    uint8_t value = 0;

    /* Find out valid OTP page. */
    if (!hw_imx135_otp_check_page(OTP_TYPE_ID_WB))
    {
        pr_err("%s: Check OTP_TYPE_ID_WB page fail\n", __func__);
        return false;
    }

    value=hw_imx135_cci_i2c_read(0x3B07);//module code:26060094
    if(value != 0x94)
    {
        pr_err("%s: Check module code error: 0x%2x != 0x94\n", __func__, value);
        return false;
    }
    value=hw_imx135_cci_i2c_read(0x3B08);//module vendor code, bit[7:4]'01 == liteon
    if(value)
    {
        hw_imx135_module_id = (value&0xF0)>>4;
        pr_info("%s,%d:hw_imx135_module_id = %d\n",
            __func__, __LINE__, hw_imx135_module_id);
    }
    return true;
}

/********************************OTP INFO END************************************/

/****************************************************************************
* FunctionName: hw_imx135_match_module;
* Description : Check which manufacture this module based Sony image sensor imx135 belong to;
***************************************************************************/
static int hw_imx135_match_module(struct msm_sensor_ctrl_t *s_ctrl)
{
    if(!hw_imx135_otp_match_module())
    {
        pr_err("%s: Check OTP module info failed\n", __func__);
        goto DEFAULT_CONFIG;
    }

    /*add project name for the project menu*/
    if(HW_IMX135_LITEON == hw_imx135_module_id)
    {
        /* formular to : driver use sensor name without module name */
        s_ctrl->sensordata->sensor_name = "hw_imx135_liteon";
        strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060094FA-IMX-L", strlen("23060094FA-IMX-L")+1);
    }
    else
    {
        pr_err("%s:read a wrong vendor code from OTP!!\n", __func__);
        goto DEFAULT_CONFIG;
    }

    pr_info("%s: sensor_name=%s, sensor_project_name=%s\n",  __func__,
            s_ctrl->sensordata->sensor_name, s_ctrl->sensordata->sensor_info->sensor_project_name);

    return 0;

DEFAULT_CONFIG:
    pr_err("%s:use LITEON as default vendor\n", __func__);
    s_ctrl->sensordata->sensor_name = "hw_imx135_liteon";
    strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060094FA-IMX-L", strlen("23060094FA-IMX-L")+1);

    return 0;
}

static struct msm_sensor_fn_t hw_imx135_func_tbl = {
    .sensor_config = msm_sensor_config,
    .sensor_power_up = msm_sensor_power_up,
    .sensor_power_down = msm_sensor_power_down,
    .sensor_match_id = msm_sensor_match_id,
    .sensor_match_module = hw_imx135_match_module,
    .sensor_write_otp = hw_imx135_set_otp_info,
};

MODULE_DEVICE_TABLE(of, hw_imx135_dt_match);

static struct platform_driver hw_imx135_platform_driver = {
	.driver = {
		.name = "qcom,hw_imx135",
		.owner = THIS_MODULE,
		.of_match_table = hw_imx135_dt_match,
	},
};

static int32_t hw_imx135_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d\n", __func__, __LINE__);
	
	match = of_match_device(hw_imx135_dt_match, &pdev->dev);
    if(match && match->data)
    {
        rc = msm_sensor_platform_probe(pdev, match->data);
    }
	return rc;
}

static int __init hw_imx135_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&hw_imx135_platform_driver,
		hw_imx135_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hw_imx135_i2c_driver);
}

static void __exit hw_imx135_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hw_imx135_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hw_imx135_s_ctrl);
		platform_driver_unregister(&hw_imx135_platform_driver);
	} else
		i2c_del_driver(&hw_imx135_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hw_imx135_s_ctrl = {
	.sensor_i2c_client = &hw_imx135_sensor_i2c_client,
	.power_setting_array.power_setting = hw_imx135_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hw_imx135_power_setting),
	.msm_sensor_mutex = &hw_imx135_mut,
	.sensor_v4l2_subdev_info = hw_imx135_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hw_imx135_subdev_info),
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
	.func_tbl = &hw_imx135_func_tbl,
#endif
};

module_init(hw_imx135_init_module);
module_exit(hw_imx135_exit_module);
MODULE_DESCRIPTION("hw_imx135");
MODULE_LICENSE("GPL v2");

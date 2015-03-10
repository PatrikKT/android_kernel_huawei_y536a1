/* add cmi lcd driver */
/* Copyright (c) 2009, Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include<linux/init.h>
#include<linux/module.h>

#include <linux/hw_lcd_common.h>
#include <mdss_dsi.h>
#include <linux/of.h>

#define LCD_ID_0_GPIO    15
#define LCD_ID_1_GPIO    31

#define GET_GPIO_FAIL  -1
#define GET_LCD_ID_FAIL  -1 
#define LCD_ID_PULL_UP  1
#define LCD_ID_PULL_DOWN  0

struct of_device_id huawei_mdss_dsi_panel_match[] = {
	{.compatible = "huawei,lcd_panel_idA"},
	{}
};
/* add dynamic log */
u32 hw_get_board_id(void)
{
	struct device_node *root = NULL;
	u32 board_id[2] = {0};

	root = of_find_node_by_path("/");
	of_property_read_u32_array(root, "qcom,msm-id", board_id, 2);

	LCD_LOG_INFO("%s: board id = %d\n",__func__,board_id[1]);

	return board_id[1];

}


void setup_lcd_power(void)
{
	int rc;
	struct regulator *reg_l6;

	reg_l6 = regulator_get(NULL,
			"8226_l6");
	if (IS_ERR(reg_l6)) {
		LCD_LOG_ERR("could not get 8038_l6, rc = %ld\n",
			PTR_ERR(reg_l6));
		return;
	}
	rc = regulator_set_voltage(reg_l6, 1800000, 1800000);
	if (rc) {
		LCD_LOG_ERR("set_voltage l6 failed, rc=%d\n", rc);
		return;
	}
	rc = regulator_enable(reg_l6);
	if (rc) {
		LCD_LOG_ERR("enable l6 failed, rc=%d\n", rc);
		return;
	}
	LCD_LOG_INFO("%s: set reg_l6 finish!\n",__func__);

	return ;

}

/****************************************************************
function: get lcd id by gpio

*data structure*
*	   ID1	  ID0   *
 *	----------------- *
 *	|   |   |   |   | *
 *	|   |   |   |   | *
 *	----------------- *
 For each Gpio :
		00 means low  ,
		01 means high ,
		10 means float,
		11 is not defined,

 lcd id(hex):
 0	:ID0 low,	ID1 low
 1	:ID0 high,	ID1 low
 2	:ID0 float,	ID1 low

 4	:ID0 low,	ID1 high
 5	:ID0 high,	ID1 high
 6	:ID0 float,	ID1 high

 8	:ID0 low,	ID1 float
 9	:ID0 high,	ID1 float
 A	:ID0 float,	ID1 float, used for emulator
 ***************************************************************/
int hw_get_lcd_id(void)
{
	int ret = 0;
	int id0,id1;
	int gpio_id0,gpio_id1;
	int pullup_read,pulldown_read;
	static int lcd_id = GET_LCD_ID_FAIL;

	id0=0;
	id1=0;
	pullup_read = 0;
	pulldown_read = 0;
	gpio_id0 = LCD_ID_0_GPIO;
	gpio_id1 = LCD_ID_1_GPIO;

	if( (lcd_id >= 0x0) && (lcd_id <= 0xA) )//if lcd_id had read successfully,just return lcd_id.
		return lcd_id;

	if(gpio_id0 <= GET_GPIO_FAIL ||gpio_id1 <= GET_GPIO_FAIL)
		return GET_LCD_ID_FAIL;

    LCD_LOG_INFO("gpio_lcd_id0:%d gpio_lcd_id1:%d\n",gpio_id0,gpio_id1);

    ret = gpio_request(gpio_id0, "lcd_id0");
      if (ret) {
         LCD_LOG_ERR("lcd_id0 gpio[%d] request failed\n", gpio_id0);
         goto lcd_id0_req_fail;
          }

    ret = gpio_request(gpio_id1, "lcd_id1");
	if (ret) {
	     LCD_LOG_ERR("lcd_id1 gpio[%d] request failed\n", gpio_id1);
         goto lcd_id1_req_fail;
	}

	/*config id0 to pull down and read*/
	ret = gpio_tlmm_config(GPIO_CFG(gpio_id0,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	  if (ret) {
	     LCD_LOG_ERR("config id0 to pull down failed\n");
	     goto get_lcd_id_fail;
	    }
	udelay(10);
	pulldown_read = gpio_get_value(gpio_id0);

	/*config id0 to pull up and read*/
	ret = gpio_tlmm_config(GPIO_CFG(gpio_id0,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_UP,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if (ret) {
		LCD_LOG_ERR("config id0 to pull up failed\n");
		goto get_lcd_id_fail;
		}
	udelay(10);
	pullup_read = gpio_get_value(gpio_id0);
	if(pulldown_read != pullup_read)//float
	{
		id0 = BIT(1);
		gpio_tlmm_config(GPIO_CFG(gpio_id0,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	}
	else//connect 
	{
		id0 = pullup_read;//pullup_read==pulldown_read
		switch(id0)
		{
			case LCD_ID_PULL_DOWN:
			case LCD_ID_PULL_UP:
			default:
				gpio_tlmm_config(GPIO_CFG(gpio_id0,0,GPIO_CFG_INPUT,GPIO_CFG_NO_PULL,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
				break;
		}

	}

	/*config id1 to pull down and read*/
	ret = gpio_tlmm_config(GPIO_CFG(gpio_id1,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if (ret) {
		LCD_LOG_ERR("config id1 to pull down failed\n");
		goto get_lcd_id_fail;
		}
	udelay(10);
	pulldown_read = gpio_get_value(gpio_id1);

	/*config id1 to pull up and read*/
	ret = gpio_tlmm_config(GPIO_CFG(gpio_id1,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_UP,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if (ret) {
		LCD_LOG_ERR("config id1 to pull up failed\n");
		goto get_lcd_id_fail;
		}
	udelay(10);
	pullup_read = gpio_get_value(gpio_id1);
	if(pulldown_read != pullup_read)//float
	{
		id1 = BIT(1);
		gpio_tlmm_config(GPIO_CFG(gpio_id1,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	}
	else//connect
	{
		id1 = pullup_read;//pullup_read==pulldown_read
		switch(id1)
		{
			case LCD_ID_PULL_DOWN:
			case LCD_ID_PULL_UP:
			default:
				gpio_tlmm_config(GPIO_CFG(gpio_id1,0,GPIO_CFG_INPUT,GPIO_CFG_NO_PULL,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
				break;
		}
	}

	gpio_free(gpio_id0);
	gpio_free(gpio_id1);

	lcd_id = (id1<<2) | id0;
	return lcd_id;
get_lcd_id_fail:
	gpio_free(gpio_id1);
lcd_id1_req_fail:
	gpio_free(gpio_id0);
lcd_id0_req_fail:
	return GET_LCD_ID_FAIL;
}

void hw_get_lcd_panel(void)
{
	char *psKey = NULL;
	int id = 0;
	int ret = 0;
	psKey = kmalloc(30, GFP_KERNEL);
    if (NULL == psKey)  
    {
		ret = false;
		return ;
    }
    memset(psKey, 0, 30);

	id = hw_get_lcd_id();
	sprintf(psKey, "huawei,lcd_panel_id%X", id);
	strcpy(huawei_mdss_dsi_panel_match->compatible,psKey);

   kfree(psKey);
	return;
	
};

#ifdef CONFIG_DEBUG_FS
/* whether print log or not */
#define LCD_MIPI_DEBUG_ON    (1)

static struct mdss_dsi_ctrl_pdata *g_lcd_dbg_dsi_ctrl_pdata = NULL;    // global mdss_dsi_ctrl_pdata

/* set global ctrl_pdata pointer */
void lcd_dbg_set_dsi_ctrl_pdata(struct mdss_dsi_ctrl_pdata *ctrl)
{
	static char already_set = 0;  

	/* judge if already set or not*/
	if (already_set)
	{
		LCD_LOG_ERR("%s: already set\n", __func__);
	}
	else
	{
		g_lcd_dbg_dsi_ctrl_pdata = ctrl;
		already_set = 1;   
	}

	return;
}

/* get global ctrl_pdata pointer */
struct mdss_dsi_ctrl_pdata *lcd_dbg_get_dsi_ctrl_pdata(void)
{
	return g_lcd_dbg_dsi_ctrl_pdata;
}

/* check whether mipi input is legal or not */
/* return: 0 - success, negative - fail */
static int is_mipi_input_legal(int op_type,int ic_reg, int cmd_type, int param_num,char *buf)
{
	int ret = 0;
	if( (op_type != OPER_READ) && (op_type != OPER_WRITE))
	{
		ret = -1;
	}	
	/* ic_reg must in [0x00, 0xff] */
	if (!((unsigned int)ic_reg >= 0 && (unsigned int)ic_reg <= 0xff))
	{
		ret = -1;
	}

	/* cmd_type must be 0x01 or 0x04 */
	if ((cmd_type != MIPI_DCS_COMMAND) &&(cmd_type != MIPI_GEN_COMMAND))
	{
		ret = -1;
	}
	/* param_num must larger or equal 0 */
	if (param_num < 0)
	{
		ret = -1;
	}

	if(NULL == buf)
	{
		ret = -1;
	}
	return ret;
}


/**********************************************************************************
*function:process read and write commands  for lcd reg debug
*op_type:	read or write
*reg:		lcd register
*cmd_type:	DCS or GEN
*param_num:	the count of prameters to tranfer
*param_buf:	prameters
*read_value:  value from lcd panel
*delay_ms:	delay time
*return: 0 - success, negative - fail
**********************************************************************************/
int lcd_dbg_mipi_prcess_ic_reg(int op_type,int reg, int cmd_type, int param_num, char *param_buf,int *read_value, int delay_ms)
{
	static struct dcs_cmd_req cmdreq;
	static struct dsi_cmd_desc dsi_cmd;                  // dsi cmd struct
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
#if LCD_MIPI_DEBUG_ON
	int i = 0;
#endif
	
	/* check if input legal */
	if (is_mipi_input_legal(op_type,reg, cmd_type, param_num,param_buf))
	{
		LCD_LOG_ERR("%s, input illegal.\n", __func__);
		return -1;
	}

	ctrl = lcd_dbg_get_dsi_ctrl_pdata();
	/* translate cmd_type from huawei to qcom's format */
	switch (param_num)
	{
		case 0:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_READ;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_WRITE;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_WRITE1;
				}
			}
			break;
		}

		case 1:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					LCD_LOG_ERR("%s ,not support this kind of dcs read! \n",__func__);
					return -1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ1;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_WRITE1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_WRITE2;
				}
			}
			break;
		}

		default:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					LCD_LOG_ERR("%s ,not support this kind of dcs read! \n",__func__);
					return -1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ2;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_LWRITE;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_LWRITE;
				}
			}
			break;
		}
	}

	/* insert reg into param_buf's beginning */
	memmove(param_buf + 1, param_buf, param_num);
	param_buf[0] = reg;
	param_num++;

	/* debug begin */
#if LCD_MIPI_DEBUG_ON
	LCD_LOG_INFO("%s,op_type=%d, reg=0x%02x, cmd_type=0x%02x, param_num=0x%02x, delay_ms=0x%02x\n", __func__, op_type,reg, cmd_type, param_num, delay_ms);
	LCD_LOG_INFO("%s, print param_buf begin\n", __func__);
	for (i = 0; i < param_num; i++)
	{
		LCD_LOG_INFO("0x%02x ", param_buf[i]);
	}
	LCD_LOG_INFO("%s, print param_buf end\n", __func__);
#endif
	/* debug end */

	dsi_cmd.dchdr.dtype = cmd_type;
	dsi_cmd.dchdr.last =1;
	dsi_cmd.dchdr.vc = 0;
	dsi_cmd.dchdr.dlen = param_num;
	dsi_cmd.payload = param_buf;
	memset(&cmdreq, 0, sizeof(cmdreq));
	switch(op_type)
	{
		case OPER_READ:	
			dsi_cmd.dchdr.ack = 1;
			dsi_cmd.dchdr.wait = 5;//5 ms
			cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
			cmdreq.rbuf = (char*)read_value;
			break;
		case OPER_WRITE:
			dsi_cmd.dchdr.ack = 0;
			dsi_cmd.dchdr.wait = delay_ms;//5 ms
			cmdreq.flags = CMD_REQ_COMMIT;
			cmdreq.rbuf = NULL;
			break;
	}
	cmdreq.cmds = &dsi_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if( op_type ==OPER_READ )
	{
		LCD_LOG_INFO("%s, read value is 0x%02x\n", __func__,cmdreq.rbuf[0]);
	}
	return 0;
}
#endif


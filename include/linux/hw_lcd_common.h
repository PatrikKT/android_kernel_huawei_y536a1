/* Copyright (c), Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef HW_LCD_COMMON_H
#define HW_LCD_COMMON_H

#include "mdss_dsi.h"

/* Add dynamic_log interface */
#define LCD_ERR  1
#define LCD_INFO 2
#define LCD_DBG  3

#define OPER_READ  (1)
#define OPER_WRITE (2)
#define MIPI_DCS_COMMAND (1<<0)
#define MIPI_GEN_COMMAND 4
#define MIPI_PATH_OPEN		1
#define MIPI_PATH_CLOSE	0
#ifdef CONFIG_DEBUG_FS
extern atomic_t mipi_path_status;
#endif


extern int lcd_debug_mask ;

#ifndef LCD_LOG_ERR
#define LCD_LOG_ERR( x...)					\
do{											\
	if( lcd_debug_mask >= LCD_ERR )			\
	{										\
		printk(KERN_ERR "[LCD_ERR] " x);	\
	}										\
											\
}while(0)
#endif

#ifndef LCD_LOG_INFO
#define LCD_LOG_INFO( x...)					\
do{											\
	if( lcd_debug_mask >= LCD_INFO )		\
	{										\
		printk(KERN_ERR "[LCD_INFO] " x);	\
	}										\
											\
}while(0)
#endif

#ifndef LCD_LOG_DBG
#define LCD_LOG_DBG( x...)					\
do{											\
	if( lcd_debug_mask >= LCD_DBG )			\
	{										\
		printk(KERN_ERR "[LCD_DBG] " x);	\
	}										\
											\
}while(0)
#endif

/* LCD_MDELAY will select mdelay or msleep according value */
#define LCD_MDELAY(time_ms)   	\
	do							\
	{ 							\
		if (time_ms>10)			\
			msleep(time_ms);	\
		else					\
			mdelay(time_ms);	\
	}while(0)	

enum {
	HUAWEI_LCD_ID0,
	HUAWEI_LCD_ID1,
	HUAWEI_LCD_ID2,
	HUAWEI_LCD_ID4,
	HUAWEI_LCD_ID5,
	HUAWEI_LCD_ID6,
	HUAWEI_LCD_ID8,
	HUAWEI_LCD_ID9,
	HUAWEI_LCD_IDA,
};

extern struct of_device_id huawei_mdss_dsi_panel_match[2];

void hw_get_lcd_panel(void);
void setup_lcd_power(void);


#ifdef CONFIG_DEBUG_FS
void lcd_dbg_set_dsi_ctrl_pdata(struct mdss_dsi_ctrl_pdata *ctrl);
struct mdss_dsi_ctrl_pdata *lcd_dbg_get_dsi_ctrl_pdata(void);
int lcd_dbg_mipi_prcess_ic_reg(int op_type,int reg, int cmd_type, int param_num, char *param_buf,int *read_value, int delay_ms);
int lcd_debugfs_init(void);
#endif
#endif

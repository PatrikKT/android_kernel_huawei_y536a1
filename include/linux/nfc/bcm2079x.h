/*
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _BCM2079X_H
#define _BCM2079X_H

#define BCMNFC_MAGIC	0xFA

/*
 * BCMNFC power control via ioctl
 * BCMNFC_POWER_CTL(0): power off
 * BCMNFC_POWER_CTL(1): power on
 * BCMNFC_WAKE_CTL(0): wake off
 * BCMNFC_WAKE_CTL(1): wake on
 */
#define BCMNFC_POWER_CTL		_IO(BCMNFC_MAGIC, 0x01)
#define BCMNFC_CHANGE_ADDR		_IO(BCMNFC_MAGIC, 0x02)
#define BCMNFC_READ_FULL_PACKET		_IO(BCMNFC_MAGIC, 0x03)
#define BCMNFC_SET_WAKE_ACTIVE_STATE	_IO(BCMNFC_MAGIC, 0x04)
#define BCMNFC_WAKE_CTL			_IO(BCMNFC_MAGIC, 0x05)
#define BCMNFC_READ_MULTI_PACKETS	_IO(BCMNFC_MAGIC, 0x06)


/*
1. NFC_WAKE         MSM8226(GPIO_20)
2. NFC_INT            MSM8226(GPIO_21)
3. NFC_VEN           PM8226(GPIO_2)
4. CLK_REQ_NFC    PM8226(GPIO_3)
5. TCXO_OUT_NFC  PM8226(BB_CLK2) 
6. I2C5_DATA        MSM8226(GPIO_10)
7. I2C5_CLK          MSM8226(GPIO_11)
*/
#define BCM_NFC_IRQ     21
#define BCM_NFC_VEN     2
#define BCM_NFC_WAKE    20
#define BCM_NFC_CLK_REQ    3
#define BCM_NFC_I2C_ADD    0x77


struct bcm2079x_platform_data {
	unsigned int irq_gpio;
	unsigned int en_gpio;
	unsigned int wake_gpio;
	unsigned int clock_req_gpio;
};

#endif


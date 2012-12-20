/* arch/arm/mach-msm/include/mach/board-swift.h
 * Copyright (C) 2009 LGE, Inc.
 * Author: SungEun Kim <cleaneye@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ARCH_MSM_BOARD_SWIFT_H
#define __ARCH_MSM_BOARD_SWIFT_H

#include <asm/setup.h>

#include <linux/types.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include "pm.h"

/* LGE_S [ynj.kim@lge.com] 2010-05-21 : atcmd - virtual device */
#define KEY_SPEAKERMODE 241 // KEY_VIDEO_NEXT is not used in GED
#define KEY_CAMERAFOCUS 242 // KEY_VIDEO_PREV is not used in GED
#define KEY_FOLDER_HOME 243
#define KEY_FOLDER_MENU 244

#define ATCMD_VIRTUAL_KEYPAD_ROW	8
#define ATCMD_VIRTUAL_KEYPAD_COL	8
/* LGE_E [ynj.kim@lge.com] 2010-05-21 : atcmd - virtual device */

/* sdcard related macros */
#define GPIO_SD_DETECT_N	49
#define VREG_SD_LEVEL		2850

/* camera */
#define CAM_I2C_SLAVE_ADDR		0x1A
#define GPIO_CAM_RESET			0		/* GPIO_0 */
#define GPIO_CAM_PWDN		 	1		/* GPIO_1 */
#define GPIO_CAM_MCLK			15		/* GPIO_15 */
#define CAM_DEFAULT_CLOCK_RATE		24000000

#define CAM_POWER_OFF		0
#define CAM_POWER_ON		1

/* accelerometer */
#define ACCEL_GPIO_INT	 	33
#define ACCEL_GPIO_I2C_SCL  	40
#define ACCEL_GPIO_I2C_SDA  	41
#define ACCEL_I2C_ADDRESS	0x38

/*Ecompass*/
#define ECOM_GPIO_INT		18
#define ECOM_GPIO_I2C_SCL	30
#define ECOM_GPIO_I2C_SDA	31
#define ECOM_I2C_ADDRESS	0x1C

/* lcd & backlight */
#define GPIO_LCD_BL_EN		84
#define GPIO_LCD_RESET_N	101

/* bluetooth gpio pin */
enum {
	BT_WAKE         = 42,
	BT_RFR          = 43,
	BT_CTS          = 44,
	BT_RX           = 45,
	BT_TX           = 46,
	BT_PCM_DOUT     = 68,
	BT_PCM_DIN      = 69,
	BT_PCM_SYNC     = 70,
	BT_PCM_CLK      = 71,
	BT_HOST_WAKE    = 83,
	BT_RESET_N	= 96,
};

/* ear sense driver macros */
#define GPIO_EAR_SENSE		29
#define GPIO_HS_MIC_BIAS_EN	26

/* interface variable */
extern struct platform_device msm_device_snd;
extern struct platform_device msm_device_adspdec;
extern struct i2c_board_info i2c_devices[1];

/* interface functions */
int config_camera_on_gpios(void);
void config_camera_off_gpios(void);

#endif

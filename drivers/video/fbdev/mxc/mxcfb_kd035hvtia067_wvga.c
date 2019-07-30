/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/bitops.h>
#include <linux/gcd.h>
#include <linux/mipi_dsi_northwest.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/pm_runtime.h>
#include <linux/busfreq-imx.h>
#include <linux/backlight.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <video/mipi_display.h>
#include <video/mxc_edid.h>
#include <linux/mfd/syscon.h>

#include "mipi_dsi.h"

#define MIPI_DSI_MAX_RET_PACK_SIZE				(0x4)
#define MIPI_FIFO_TIMEOUT		msecs_to_jiffies(250)

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static struct fb_videomode truly_lcd_modedb[] = {
	{
	 "KD035HVTIA067", 	/* name (optional) */
	 60, 			/* refresh (optional) */
	 320, 			/*xres;*/
	 480, 			/*yres;*/
	 10500000, 		/*pixclock;*/
	 20, 			/*left_margin;*/
	 5, 			/*right_margin;*/
	 6, 			/*upper_margin;*/
	 2, 			/*lower_margin;*/
	 2, 			/*hsync_len;*/
	 2, 			/*vsync_len;*/
	 FB_SYNC_OE_LOW_ACT, /*sync;*/
	 FB_VMODE_NONINTERLACED, /*vmode;*/
	 0, 			/*flag;*/
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num  = 1,
	.max_phy_clk    = (850),
	.dpi_fmt		= MIPI_RGB666_PACKED,
};

void mipid_kd035hvtia067_get_lcd_videomode(struct fb_videomode **mode, int *size, struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}

int mipid_kd035hvtia067_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	int err;

	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 LCD setup.\n");


	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, buf, 0);
	CHECK_RETCODE(err);


	err = mipi_dsi->mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI COM43H Sleep out...OK.\n");
	msleep(200);

	err = mipi_dsi->mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI Display off...OK.\n");
	msleep(120);

#if 1
	buf[0] = 0xD3;
	err =  mipi_dsi->mipi_dsi_pkt_read(mipi_dsi, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM, buf, 4);
	if (!err )
		dev_info(&mipi_dsi->pdev->dev, "MIPI DSI LCD ID:0x%x.\n", buf[0]);
	else
		dev_info(&mipi_dsi->pdev->dev, "MIPI DSI LCD ID error:0x%x buf:0x%x.\n", err, buf[0]);
#endif

#if 1
	/* Positive Gamma Control */
	buf[0] = 0x0E0400E0;
	buf[1] = 0x400A1708;
	buf[2] = 0x0E074D79;
	buf[3] = 0x0F1D1A0A;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 16);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Positive Gamma Control...OK.\n");

	/* Negative Gamma Control */
	buf[0] = 0x1F1B00E1;
	buf[1] = 0x32051002;
	buf[2] = 0x0A024334;
	buf[3] = 0x0F373309;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 16);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Negative Gamma Control...OK.\n");

	/* Power Control 1 */
	buf[0] = 0x1618C0;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 3);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Power Control 1...OK.\n");

	/* Power Control 2 */
	buf[0] = 0x41c1;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 POWER CONTROL MODE 2...OK.\n");
	msleep(5);

	/* VCOM Control */
	buf[0] = 0x801E00C5;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 4);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 VCOM Control...OK.\n");

	/* Memory Access control */
	buf[0] = 0x4836;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Memory Access control...OK.\n");
	msleep(5);
#endif

	/* Interface Mode Control 24BIT */
	buf[0] = 0x553A; //0x773a;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Interface pixel format...OK.\n");
	msleep(5);

//	/* Frame Rate Control Frame rate 70HZ */
	buf[0] = 0x11A0B1;		//->60
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 3);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Frame Rate Control...OK.\n");
	msleep(5);

#if 1
	/* Display Inversion Control */
	buf[0] = 0x02B4;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Display Inversion Control...OK.\n");
	msleep(5);

	/* Set Image Function */
	buf[0] = 0x00E9;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Set Image Function...OK.\n");
	msleep(5);

	/* Adjust Control 3 */
	buf[0] = 0x2C51A9F7;
	buf[1] = 0x82;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Adjust Control 3...OK.\n");
#endif

#if 1
	/**********set rgb interface mode******************/
//	write_command(0xB6);
//	write_data(0x02); //30 set rgb
//	write_data(0x02); //GS,SS 02£¬42£¬62
//	write_data(0x3B);
	buf[0] = 0x3B0202B6;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 4);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Display Function Control...OK.\n");
	msleep(5);

	/* Column Address Set */
	buf[0] = 0x0100002A;
	buf[1] = 0x3F;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Column Address Set...OK.\n");

	/* Page Address Set */
	buf[0] = 0x0100002B;
	buf[1] = 0xDF;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Page Address Set...OK.\n");
#endif

	buf[1] = 0;
	buf[0] = MIPI_DCS_WRITE_MEMORY_START;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI Display WRITE_MEMORY_START..OK.\n");
	msleep(5);

	msleep(20);
	buf[1] = 0;
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 COM43H Sleep out...OK.\n");
	msleep(10);

	buf[1] = 0;
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI Display on...OK.\n");
	msleep(120);

	return err;
}


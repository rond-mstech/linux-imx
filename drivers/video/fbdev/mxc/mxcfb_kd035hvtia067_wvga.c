/*
 * Copyright (C) 2015-2019 MS Tech. All Rights Reserved.
 *
 * Author: Ron Donio. ron.d@ms-technologies.com
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
#include <linux/kernel.h>           //used for do_exit()
#include <linux/threads.h>          //used for allow_signal
#include <linux/kthread.h>          //used for kthread_create
#include <linux/delay.h>            //used for ssleep()

#include <linux/mipi_dsi.h>
#include <video/mipi_display.h>

#include "mipi_dsi.h"

#define MIPI_DSI_MAX_RET_PACK_SIZE				(0x4)
//#define WDT
#define DEBUG 1
#undef dev_dbg
#define dev_dbg(dev, format, arg...)		\
	dev_printk(KERN_DEBUG, dev, format, ##arg)



int mipi_dsi_lcd_init(struct mipi_dsi_info *mipi_dsi, struct mxc_dispdrv_setting *setting);

extern struct mxc_dispdrv_setting lcd_local_setting;
extern int mipid_esdp; 									 /* Activate ESD Protection */


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
	 KHZ2PICOS(12500), 		/*pixclock;*/
	 3, 			/*left_margin;*/
	 3, 			/*right_margin;*/
	 2, 			/*upper_margin;*/
	 2, 			/*lower_margin;*/
	 3, 			/*hsync_len;*/
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
	.dpi_fmt		= MIPI_RGB565_PACKED,
};
int mipid_kd035hvtia067_lcd_setup(struct mipi_dsi_info *mipi_dsi);
int mipid_kd035hvtia067_lcd_get_model(struct mipi_dsi_info *mipi_dsi);

void mipid_kd035hvtia067_get_lcd_videomode(struct fb_videomode **mode, int *size, struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}
#ifdef WDT

static inline void mipi_dsi_read_register(struct mipi_dsi_info *mipi_dsi,
				u32 reg, u32 *val)
{
	*val = ioread32(mipi_dsi->mmio_base + reg);
	dev_dbg(&mipi_dsi->pdev->dev, "read_reg:0x%02x, val:0x%08x.\n",
			reg, *val);
}

static inline void mipi_dsi_write_register(struct mipi_dsi_info *mipi_dsi,
				u32 reg, u32 val)
{
	iowrite32(val, mipi_dsi->mmio_base + reg);
	dev_dbg(&mipi_dsi->pdev->dev, "\t\twrite_reg:0x%02x, val:0x%08x.\n",
			reg, val);
}

static inline void mipi_dsi_set_mode(struct mipi_dsi_info *mipi_dsi, bool cmd_mode)
{
	u32	val;

	if (cmd_mode) {
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP, DSI_PWRUP_RESET);
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, &val);
		val |= MIPI_DSI_CMD_MODE_CFG_EN_CMD_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, val);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VID_MODE_CFG, 0);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP, DSI_PWRUP_POWERUP);
	} else {
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP, DSI_PWRUP_RESET);
		 /* Disable Command mode when tranfering video data */
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, &val);
		val &= ~MIPI_DSI_CMD_MODE_CFG_EN_CMD_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, val);
		val = DSI_VID_MODE_CFG_EN | DSI_VID_MODE_CFG_EN_BURSTMODE | DSI_VID_MODE_CFG_EN_LP_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VID_MODE_CFG, val);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP, DSI_PWRUP_POWERUP);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_IF_CTRL, DSI_PHY_IF_CTRL_TX_REQ_CLK_HS);
	}
}

static struct task_struct *watchdog_task = NULL;

void set_reset_required(void);

void reset_reset_required(void);

int get_reset_required(void);


static int watchdog_task_handler(void *arguments)
{
struct mipi_dsi_info *mipi_dsi = (struct mipi_dsi_info *)arguments;

	allow_signal(SIGKILL);
	printk(KERN_ERR "***MIPI DSI watchdog_task_handler started\n");

	while(1)
	{
		if (mipid_esdp ==0)
		{
			mdelay(5000);
			continue;
		}

		if (0 == get_reset_required())
		{
			mipi_dsi_set_mode(mipi_dsi, true);
			mipid_kd035hvtia067_lcd_get_model(mipi_dsi);
			mipi_dsi_set_mode(mipi_dsi, false);

			mdelay(5000);
		}
		else
		{
			mipi_dsi_set_mode(mipi_dsi, true);
			mipi_dsi_lcd_init(mipi_dsi, &lcd_local_setting);
			mipi_dsi_set_mode(mipi_dsi, false);

			reset_reset_required();
			mdelay(1000);
		}

		if (signal_pending(watchdog_task)) break;
	}


	printk(KERN_ERR "***MIPI DSI watchdog_task_handler ended\n");

	do_exit(0);

	return 0;
}
#endif

int mipid_kd035hvtia067_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;
	u8 id, model;

	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 LCD setup.\n");

	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, buf, 0);
	CHECK_RETCODE(err);


	err = mipi_dsi->mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI COM43H Sleep out...OK.\n");
	msleep(200);

	err = mipi_dsi->mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI Display off...OK.\n");
	msleep(120);

	buf[0] = 0xD3;
	err =  mipi_dsi->mipi_dsi_pkt_read(mipi_dsi, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM, buf, 4);
	CHECK_RETCODE(err);
	model = (buf[0] & 0xFF00) >> 8;
	id    = (buf[0] & 0xFF0000) >> 16;
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI LCD ID:0x%x MODEL:0x%x.\n", id, model);


	/* Positive Gamma Control */
	buf[0] = 0x0E0400E0;
	buf[1] = 0x400A1708;
	buf[2] = 0x0E074D79;
	buf[3] = 0x0F1D1A0A;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 16);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Positive Gamma Control...OK.\n");

	/* Negative Gamma Control */
	buf[0] = 0x1F1B00E1;
	buf[1] = 0x32051002;
	buf[2] = 0x0A024334;
	buf[3] = 0x0F373309;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 16);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Negative Gamma Control...OK.\n");

	/* Power Control 1 */
	buf[0] = 0x1618C0;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 3);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Power Control 1...OK.\n");

	/* Power Control 2 */
	buf[0] = 0x41c1;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 POWER CONTROL MODE 2...OK.\n");
	msleep(5);

	/* VCOM Control */
	buf[0] = 0x801E00C5;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 VCOM Control...OK.\n");

	/* Memory Access control */
	buf[0] = 0x4836;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Memory Access control...OK.\n");
	msleep(5);

	/* Interface Mode Control 18BIT RGB666 */
	buf[0] = 0x553A; //0x773a;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Interface pixel format...OK.\n");
	msleep(5);


//	/* Frame Rate Control Frame rate 60HZ */
	buf[0] = 0x11A0B1;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 3);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Frame Rate Control...OK.\n");
	msleep(5);


	/* Display Inversion Control */
	buf[0] = 0x02B4;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Display Inversion Control...OK.\n");
	msleep(5);

	/* Set Image Function */
	buf[0] = 0x00E9;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Set Image Function...OK.\n");
	msleep(5);

	/* Adjust Control 3 */
	buf[0] = 0x2C51A9F7;
	buf[1] = 0x82;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Adjust Control 3...OK.\n");

	/**********set rgb interface mode******************/
//	write_command(0xB6);
//	write_data(0x02); //30 set rgb
//	write_data(0x02); //GS,SS 02£¬42£¬62
//	write_data(0x3B);
	buf[0] = 0x3B0202B6;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Display Function Control...OK.\n");
	msleep(5);

	/* Column Address Set */
	buf[0] = 0x0100002A;
	buf[1] = 0x3F;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Column Address Set...OK.\n");

	/* Page Address Set */
	buf[0] = 0x0100002B;
	buf[1] = 0xDF;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Page Address Set...OK.\n");

	buf[1] = 0;
	buf[0] = MIPI_DCS_WRITE_MEMORY_START;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI Display WRITE_MEMORY_START..OK.\n");
	msleep(5);

	msleep(20);
	buf[1] = 0;
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 COM43H Sleep out...OK.\n");
	msleep(10);

	buf[1] = 0;
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI Display init ...DONE.\n");
	msleep(120);

#ifdef WDT

	if (mipid_esdp)
		dev_info(&mipi_dsi->pdev->dev, "MIPI ESD protection ON\n");
	else
		dev_info(&mipi_dsi->pdev->dev, "MIPI ESD protection OFF\n");

	if (watchdog_task == NULL)
	{
		watchdog_task = kthread_create(watchdog_task_handler, (void*)mipi_dsi, "mipi_watchdog");
		wake_up_process(watchdog_task);
	}
#endif

	return err;
}

int mipid_kd035hvtia067_lcd_get_model(struct mipi_dsi_info *mipi_dsi)
{
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;

	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 LCD setup.\n");

	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, buf, 0);
	CHECK_RETCODE(err);


	err = mipi_dsi->mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI COM43H Sleep out...OK.\n");
	msleep(200);

	err = mipi_dsi->mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI Display off...OK.\n");
	msleep(120);

//	buf[0] = 0xD3;
//	err =  mipi_dsi->mipi_dsi_pkt_read(mipi_dsi, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM, buf, 4);
//	CHECK_RETCODE(err);
//	model = (buf[0] & 0xFF00) >> 8;
//	id    = (buf[0] & 0xFF0000) >> 16;
//	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI LCD ID:0x%x MODEL:0x%x.\n", id, model);

	buf[0] = 0x09;	//Read Disaply Status
	buf[1] = buf[2] = buf[3] = buf[4] = 0;
	err =  mipi_dsi->mipi_dsi_pkt_read(mipi_dsi, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM, buf, 4);
//	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI LCD Display Status 0x%02hhx 0x%02hhx 0x%02hhx 0x%02hhx 0x%02hhx .\n", buf[0], buf[1], buf[2], buf[3], buf[4]);

	/* Positive Gamma Control */
	buf[0] = 0x0E0400E0;
	buf[1] = 0x400A1708;
	buf[2] = 0x0E074D79;
	buf[3] = 0x0F1D1A0A;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 16);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Positive Gamma Control...OK.\n");

	/* Negative Gamma Control */
	buf[0] = 0x1F1B00E1;
	buf[1] = 0x32051002;
	buf[2] = 0x0A024334;
	buf[3] = 0x0F373309;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 16);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Negative Gamma Control...OK.\n");

	/* Power Control 1 */
	buf[0] = 0x1618C0;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 3);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Power Control 1...OK.\n");

	/* Power Control 2 */
	buf[0] = 0x41c1;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 POWER CONTROL MODE 2...OK.\n");
	msleep(5);

	/* VCOM Control */
	buf[0] = 0x801E00C5;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 VCOM Control...OK.\n");

	/* Memory Access control */
	buf[0] = 0x4836;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Memory Access control...OK.\n");
	msleep(5);

	/* Interface Mode Control 18BIT RGB666 */
	buf[0] = 0x553A; //0x773a;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Interface pixel format...OK.\n");
	msleep(5);


//	/* Frame Rate Control Frame rate 60HZ */
	buf[0] = 0x11A0B1;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 3);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Frame Rate Control...OK.\n");
	msleep(5);


	/* Display Inversion Control */
	buf[0] = 0x02B4;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Display Inversion Control...OK.\n");
	msleep(5);

	/* Set Image Function */
	buf[0] = 0x00E9;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Set Image Function...OK.\n");
	msleep(5);

	/* Adjust Control 3 */
	buf[0] = 0x2C51A9F7;
	buf[1] = 0x82;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Adjust Control 3...OK.\n");

	/**********set rgb interface mode******************/
//	write_command(0xB6);
//	write_data(0x02); //30 set rgb
//	write_data(0x02); //GS,SS 02£¬42£¬62
//	write_data(0x3B);
	buf[0] = 0x3B0202B6;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Display Function Control...OK.\n");
	msleep(5);

	/* Column Address Set */
	buf[0] = 0x0100002A;
	buf[1] = 0x3F;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Column Address Set...OK.\n");

	/* Page Address Set */
	buf[0] = 0x0100002B;
	buf[1] = 0xDF;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 Page Address Set...OK.\n");

	buf[1] = 0;
	buf[0] = MIPI_DCS_WRITE_MEMORY_START;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI Display WRITE_MEMORY_START..OK.\n");
	msleep(5);

	msleep(20);
	buf[1] = 0;
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI KD035HVTIA067 COM43H Sleep out...OK.\n");
	msleep(10);

	buf[1] = 0;
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
	CHECK_RETCODE(err);
	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI Display init ...DONE.\n");
	msleep(120);

	return err;

}

EXPORT_SYMBOL_GPL(mipid_kd035hvtia067_lcd_get_model);

EXPORT_SYMBOL_GPL(mipid_kd035hvtia067_lcd_setup);


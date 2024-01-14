// SPDX-License-Identifier: GPL-2.0
/*
 * Raydium kd035 MIPI-DSI panel driver
 *
 * Copyright 2019 NXP
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <config/gpiolib.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77

/* Write Manufacture Command Set Control */
#define WRMAUCCTR 0xFE

/* */
#define MIPI_DSI_MAX_RET_PACK_SIZE				(0x4)
#ifndef DSI_CMD_BUF_MAXSIZE
#define	DSI_CMD_BUF_MAXSIZE         (128)
#endif

/* Manufacturer Command Set pages (CMD2) */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};

/* */
#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

/*
 * There is no description in the Reference Manual about these commands.
 * We received them from vendor, so just use them as is.
 */
static const struct cmd_set_entry mcs_rm67191[] = {
	{0xFE, 0x0B}, {0x28, 0x40}, {0x29, 0x4F}, {0xFE, 0x0E},
	{0x4B, 0x00}, {0x4C, 0x0F}, {0x4D, 0x20}, {0x4E, 0x40},
	{0x4F, 0x60}, {0x50, 0xA0}, {0x51, 0xC0}, {0x52, 0xE0},
	{0x53, 0xFF}, {0xFE, 0x0D}, {0x18, 0x08}, {0x42, 0x00},
	{0x08, 0x41}, {0x46, 0x02}, {0x72, 0x09}, {0xFE, 0x0A},
	{0x24, 0x17}, {0x04, 0x07}, {0x1A, 0x0C}, {0x0F, 0x44},
	{0xFE, 0x04}, {0x00, 0x0C}, {0x05, 0x08}, {0x06, 0x08},
	{0x08, 0x08}, {0x09, 0x08}, {0x0A, 0xE6}, {0x0B, 0x8C},
	{0x1A, 0x12}, {0x1E, 0xE0}, {0x29, 0x93}, {0x2A, 0x93},
	{0x2F, 0x02}, {0x31, 0x02}, {0x33, 0x05}, {0x37, 0x2D},
	{0x38, 0x2D}, {0x3A, 0x1E}, {0x3B, 0x1E}, {0x3D, 0x27},
	{0x3F, 0x80}, {0x40, 0x40}, {0x41, 0xE0}, {0x4F, 0x2F},
	{0x50, 0x1E}, {0xFE, 0x06}, {0x00, 0xCC}, {0x05, 0x05},
	{0x07, 0xA2}, {0x08, 0xCC}, {0x0D, 0x03}, {0x0F, 0xA2},
	{0x32, 0xCC}, {0x37, 0x05}, {0x39, 0x83}, {0x3A, 0xCC},
	{0x41, 0x04}, {0x43, 0x83}, {0x44, 0xCC}, {0x49, 0x05},
	{0x4B, 0xA2}, {0x4C, 0xCC}, {0x51, 0x03}, {0x53, 0xA2},
	{0x75, 0xCC}, {0x7A, 0x03}, {0x7C, 0x83}, {0x7D, 0xCC},
	{0x82, 0x02}, {0x84, 0x83}, {0x85, 0xEC}, {0x86, 0x0F},
	{0x87, 0xFF}, {0x88, 0x00}, {0x8A, 0x02}, {0x8C, 0xA2},
	{0x8D, 0xEA}, {0x8E, 0x01}, {0x8F, 0xE8}, {0xFE, 0x06},
	{0x90, 0x0A}, {0x92, 0x06}, {0x93, 0xA0}, {0x94, 0xA8},
	{0x95, 0xEC}, {0x96, 0x0F}, {0x97, 0xFF}, {0x98, 0x00},
	{0x9A, 0x02}, {0x9C, 0xA2}, {0xAC, 0x04}, {0xFE, 0x06},
	{0xB1, 0x12}, {0xB2, 0x17}, {0xB3, 0x17}, {0xB4, 0x17},
	{0xB5, 0x17}, {0xB6, 0x11}, {0xB7, 0x08}, {0xB8, 0x09},
	{0xB9, 0x06}, {0xBA, 0x07}, {0xBB, 0x17}, {0xBC, 0x17},
	{0xBD, 0x17}, {0xBE, 0x17}, {0xBF, 0x17}, {0xC0, 0x17},
	{0xC1, 0x17}, {0xC2, 0x17}, {0xC3, 0x17}, {0xC4, 0x0F},
	{0xC5, 0x0E}, {0xC6, 0x00}, {0xC7, 0x01}, {0xC8, 0x10},
	{0xFE, 0x06}, {0x95, 0xEC}, {0x8D, 0xEE}, {0x44, 0xEC},
	{0x4C, 0xEC}, {0x32, 0xEC}, {0x3A, 0xEC}, {0x7D, 0xEC},
	{0x75, 0xEC}, {0x00, 0xEC}, {0x08, 0xEC}, {0x85, 0xEC},
	{0xA6, 0x21}, {0xA7, 0x05}, {0xA9, 0x06}, {0x82, 0x06},
	{0x41, 0x06}, {0x7A, 0x07}, {0x37, 0x07}, {0x05, 0x06},
	{0x49, 0x06}, {0x0D, 0x04}, {0x51, 0x04},
};

static const u32 truly_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 truly_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE;

struct truly_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct gpio_desc *enable;
	struct backlight_device *backlight;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;

	const struct truly_platform_data *pdata;
};

struct truly_platform_data {
	int (*enable)(struct truly_panel *panel);
};

//static const struct drm_display_mode default_mode = {
//	.clock = 121000,
//	.hdisplay = 1080,
//	.hsync_start = 1080 + 20,
//	.hsync_end = 1080 + 20 + 2,
//	.htotal = 1080 + 20 + 2 + 34,
//	.vdisplay = 1920,
//	.vsync_start = 1920 + 10,
//	.vsync_end = 1920 + 10 + 2,
//	.vtotal = 1920 + 10 + 2 + 4,
//	.width_mm = 68,
//	.height_mm = 121,
//	.flags = DRM_MODE_FLAG_NHSYNC |
//		 DRM_MODE_FLAG_NVSYNC,
//};
//static struct fb_videomode truly_lcd_modedb[] = {
//	{
//	 "KD035HVTIA067", 	/* name (optional) */
//	 60, 			/* refresh (optional) */
//	 320, 			/*xres;*/
//	 480, 			/*yres;*/
//	 KHZ2PICOS(12500), 		/*pixclock;*/
//	 3, 			/*left_margin;*/
//	 3, 			/*right_margin;*/
//	 2, 			/*upper_margin;*/
//	 2, 			/*lower_margin;*/
//	 3, 			/*hsync_len;*/
//	 2, 			/*vsync_len;*/
//	 FB_SYNC_OE_LOW_ACT, /*sync;*/
//	 FB_VMODE_NONINTERLACED, /*vmode;*/
//	 0, 			/*flag;*/
//	},
//};

static const struct drm_display_mode default_mode = {
	.name = "KD035HVTIA067", 	/* name (optional) */
	.clock = PICOS2KHZ(12500),
	.hdisplay = 320,
	.hsync_start = 320 + 3,
	.hsync_end = 320 + 3 + 3,
	.htotal = 320 + 3 + 3 + 3,
	.vdisplay = 480,
	.vsync_start = 480 + 2,
	.vsync_end = 480 + 2 + 2,
	.vtotal = 480 + 2 + 2 + 2,
	.width_mm = 68,
	.height_mm = 121,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

/*
 * Ported for compatibilty
 */
static int mipi_dsi_dcs_cmd(struct truly_panel *panel,
				u8 cmd, const u32 *param, int num)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	int err = 0;
	u32 buf[DSI_CMD_BUF_MAXSIZE];

	switch (cmd) {
	case MIPI_DCS_EXIT_SLEEP_MODE:
	case MIPI_DCS_ENTER_SLEEP_MODE:
	case MIPI_DCS_SET_DISPLAY_ON:
	case MIPI_DCS_SET_DISPLAY_OFF:
		buf[0] = cmd;
//		err = mipi_dsi_pkt_write(mipi_dsi,
//				MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
		err = mipi_dsi_generic_write(dsi, buf, 0);
		CHECK_RETCODE(err);
		dev_dbg(dev, "MIPI DSI DCS Command KD035HVTIA067 ...OK.\n");
		break;

	default:
	dev_err(dev,
			"MIPI DSI DCS Command:0x%x Not supported!\n", cmd);
		break;
	}

	return err;
}

static int rad_panel_push_cmd_list(struct mipi_dsi_device *dsi,
				   struct cmd_set_entry const *cmd_set,
				   size_t count)
{
	size_t i;
	int ret = 0;

	for (i = 0; i < count; i++) {
		const struct cmd_set_entry *entry = cmd_set++;
		u8 buffer[2] = { entry->cmd, entry->param };

		ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer));
		if (ret < 0)
			return ret;
	}

	return ret;
};

/*
 *
 */
static int mipid_kd035hvtia067_lcd_setup(struct truly_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	u32 read_buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;
	u8 id, model;

	dev_info(dev, "MIPI DSI KD035HVTIA067 LCD setup.\n");

//	err = mipi_dsi_dcs_soft_reset(dsi);
//	CHECK_RETCODE(err);

	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	dev_dbg(dev, "MIPI_DSI_MAX_RET_PACK_SIZE...\n");
	CHECK_RETCODE(err);


	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	err = mipi_dsi_dcs_cmd(panel, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	dev_dbg(dev, "MIPI DSI COM43H Sleep out...\n");
	CHECK_RETCODE(err);
	msleep(200);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	err = mipi_dsi_dcs_cmd(panel, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	dev_dbg(dev, "MIPI DSI Display off...\n");
	CHECK_RETCODE(err);
	msleep(120);

//	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
//	buf[0] = 0xD3;
//	err =  mipi_dsi_generic_read(dsi, buf, 2,  read_buf, 4);
//	CHECK_RETCODE(err);
//	model = (read_buf[0] & 0xFF00) >> 8;
//	id    = (read_buf[0] & 0xFF0000) >> 16;
//	dev_info(dev, "MIPI DSI LCD ID:0x%x MODEL:0x%x.\n", id, model);


	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Positive Gamma Control */
	buf[0] = 0x0E0400E0;
	buf[1] = 0x400A1708;
	buf[2] = 0x0E074D79;
	buf[3] = 0x0F1D1A0A;
	err = mipi_dsi_generic_write(dsi, buf, 16);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Positive Gamma Control...\n");
	CHECK_RETCODE(err);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Negative Gamma Control */
	buf[0] = 0x1F1B00E1;
	buf[1] = 0x32051002;
	buf[2] = 0x0A024334;
	buf[3] = 0x0F373309;
	err = mipi_dsi_generic_write(dsi, buf, 16);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Negative Gamma Control...\n");
	CHECK_RETCODE(err);

	/* Power Control 1 */
	buf[0] = 0x1618C0;
	err = mipi_dsi_generic_write(dsi, buf, 3);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Power Control 1...\n");

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Power Control 2 */
	buf[0] = 0x41c1;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 POWER CONTROL MODE 2...\n");
	msleep(5);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* VCOM Control */
	buf[0] = 0x801E00C5;
	err = mipi_dsi_generic_write(dsi, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 VCOM Control...OK.\n");

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Memory Access control */
	buf[0] = 0x4836;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Memory Access control...\n");
	msleep(5);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Interface Mode Control 18BIT RGB666 */
	buf[0] = 0x553A; //0x773a;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Interface pixel format...\n");
	msleep(5);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
//	/* Frame Rate Control Frame rate 60HZ */
	buf[0] = 0x11A0B1;
//	err = mipi_dsi_generic_write(dsi, buf, 3);
	err = mipi_dsi_generic_write(dsi, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Frame Rate Control...\n");
	msleep(5);


	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Display Inversion Control */
	buf[0] = 0x02B4;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Display Inversion Control...\n");
	msleep(5);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Set Image Function */
	buf[0] = 0x00E9;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Set Image Function...\n");
	msleep(5);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Adjust Control 3 */
	buf[0] = 0x2C51A9F7;
	buf[1] = 0x82;
//	err = mipi_dsi_generic_write(dsi, buf, 5);
	err = mipi_dsi_generic_write(dsi, buf, 8);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Adjust Control 3...\n");

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/**********set rgb interface mode******************/
//	write_command(0xB6);
//	write_data(0x02); //30 set rgb
//	write_data(0x02); //GS,SS 02£¬42£¬62
//	write_data(0x3B);
	buf[0] = 0x3B0202B6;
	err = mipi_dsi_generic_write(dsi, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Display Function Control...OK.\n");
	msleep(5);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Column Address Set */
	buf[0] = 0x0100002A;
	buf[1] = 0x3F;
//	err = mipi_dsi_generic_write(dsi, buf, 5);
	err = mipi_dsi_generic_write(dsi, buf, 8);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Column Address Set...OK.\n");

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	/* Page Address Set */
	buf[0] = 0x0100002B;
	buf[1] = 0xDF;
//	err = mipi_dsi_generic_write(dsi, buf, 5);
	err = mipi_dsi_generic_write(dsi, buf, 8);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Page Address Set...OK.\n");

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	buf[1] = 0;
	buf[0] = MIPI_DCS_WRITE_MEMORY_START;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI Display WRITE_MEMORY_START..OK.\n");
	msleep(5);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	msleep(20);
	buf[1] = 0;
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 COM43H Sleep out...OK.\n");
	msleep(10);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	buf[1] = 0;
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	CHECK_RETCODE(err);
	dev_info(dev, "MIPI DSI Display init ...DONE.\n");
	msleep(120);

#ifdef WDT

	if (mipid_esdp)
		dev_info(dev, "MIPI ESD protection ON\n");
	else
		dev_info(dev, "MIPI ESD protection OFF\n");

	if (watchdog_task == NULL)
	{
		watchdog_task = kthread_create(watchdog_task_handler, (void*)mipi_dsi, "mipi_watchdog");
		wake_up_process(watchdog_task);
	}
#endif

	dev_info(dev, "MIPI DSI KD035HVTIA067 LCD setup ended.\n");

	return err;
}

static int mipid_kd035hvtia067_lcd_get_model(struct truly_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	u32 read_buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;

	dev_info(dev, "MIPI DSI KD035HVTIA067 LCD setup.\n");

	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	CHECK_RETCODE(err);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);

	err = mipi_dsi_dcs_cmd(panel, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI COM43H Sleep out...OK.\n");
	msleep(200);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	err = mipi_dsi_dcs_cmd(panel, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI Display off...OK.\n");
	msleep(120);

	dev_info(dev, "%s : %d\n", __FUNCTION__, __LINE__);
	//	ssize_t mipi_dsi_generic_read(struct mipi_dsi_device *dsi, const void *params,
	//				      size_t num_params, void *data, size_t size)
	buf[0] = 0x09;	//Read Disaply Status
	buf[1] = buf[2] = buf[3] = buf[4] = 0;
	err =  mipi_dsi_generic_read(dsi, buf, 2, read_buf, 4);
//	CHECK_RETCODE(err);
	dev_info(dev, "MIPI DSI LCD Display Status 0x%02hhx 0x%02hhx 0x%02hhx 0x%02hhx 0x%02hhx .\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4]);

	/* Positive Gamma Control */
	buf[0] = 0x0E0400E0;
	buf[1] = 0x400A1708;
	buf[2] = 0x0E074D79;
	buf[3] = 0x0F1D1A0A;
	err = mipi_dsi_generic_write(dsi, buf, 16);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Positive Gamma Control...OK.\n");

	/* Negative Gamma Control */
	buf[0] = 0x1F1B00E1;
	buf[1] = 0x32051002;
	buf[2] = 0x0A024334;
	buf[3] = 0x0F373309;
	err = mipi_dsi_generic_write(dsi, buf, 16);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Negative Gamma Control...OK.\n");

	/* Power Control 1 */
	buf[0] = 0x1618C0;
	err = mipi_dsi_generic_write(dsi, buf, 3);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Power Control 1...OK.\n");

	/* Power Control 2 */
	buf[0] = 0x41c1;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 POWER CONTROL MODE 2...OK.\n");
	msleep(5);

	/* VCOM Control */
	buf[0] = 0x801E00C5;
	err = mipi_dsi_generic_write(dsi, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 VCOM Control...OK.\n");

	/* Memory Access control */
	buf[0] = 0x4836;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Memory Access control...OK.\n");
	msleep(5);

	/* Interface Mode Control 18BIT RGB666 */
	buf[0] = 0x553A; //0x773a;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Interface pixel format...OK.\n");
	msleep(5);


//	/* Frame Rate Control Frame rate 60HZ */
	buf[0] = 0x11A0B1;
	err = mipi_dsi_generic_write(dsi, buf, 3);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Frame Rate Control...OK.\n");
	msleep(5);


	/* Display Inversion Control */
	buf[0] = 0x02B4;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Display Inversion Control...OK.\n");
	msleep(5);

	/* Set Image Function */
	buf[0] = 0x00E9;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Set Image Function...OK.\n");
	msleep(5);

	/* Adjust Control 3 */
	buf[0] = 0x2C51A9F7;
	buf[1] = 0x82;
	err = mipi_dsi_generic_write(dsi, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Adjust Control 3...OK.\n");

	/**********set rgb interface mode******************/
//	write_command(0xB6);
//	write_data(0x02); //30 set rgb
//	write_data(0x02); //GS,SS 02£¬42£¬62
//	write_data(0x3B);
	buf[0] = 0x3B0202B6;
	err = mipi_dsi_generic_write(dsi, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Display Function Control...OK.\n");
	msleep(5);

	/* Column Address Set */
	buf[0] = 0x0100002A;
	buf[1] = 0x3F;
	err = mipi_dsi_generic_write(dsi, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Column Address Set...OK.\n");

	/* Page Address Set */
	buf[0] = 0x0100002B;
	buf[1] = 0xDF;
	err = mipi_dsi_generic_write(dsi, buf, 5);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Page Address Set...OK.\n");

	buf[1] = 0;
	buf[0] = MIPI_DCS_WRITE_MEMORY_START;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI Display WRITE_MEMORY_START..OK.\n");
	msleep(5);

	msleep(20);
	buf[1] = 0;
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 COM43H Sleep out...OK.\n");
	msleep(10);

	buf[1] = 0;
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	CHECK_RETCODE(err);
	dev_info(dev, "MIPI DSI Display init ...DONE.\n");
	msleep(120);

	return err;

}



static inline struct truly_panel *to_truly_panel(struct drm_panel *panel)
{
	return container_of(panel, struct truly_panel, panel);
}

static int truly_panel_push_cmd_list(struct mipi_dsi_device *dsi,
				   struct cmd_set_entry const *cmd_set,
				   size_t count)
{
	size_t i;
	int ret = 0;

	for (i = 0; i < count; i++) {
		const struct cmd_set_entry *entry = cmd_set++;
		u8 buffer[2] = { entry->cmd, entry->param };

		ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer));
		if (ret < 0)
			return ret;
	}

	return ret;
};

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return COL_FMT_16BPP;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return COL_FMT_18BPP;
	case MIPI_DSI_FMT_RGB888:
		return COL_FMT_24BPP;
	default:
		return COL_FMT_24BPP; /* for backward compatibility */
	}
};

static int truly_panel_prepare(struct drm_panel *panel)
{
	struct truly_panel *rad = to_truly_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (rad->prepared)
		return 0;

	ret = regulator_bulk_enable(rad->num_supplies, rad->supplies);
	if (ret)
		return ret;

	/* At lest 10ms needed between power-on and reset-out as RM specifies */
	//usleep_range(10000, 12000);
	msleep(20);

	if (rad->reset) {
		gpiod_set_value(rad->reset, 0);
		msleep(20);
		gpiod_set_value(rad->reset, 1);
		msleep(200);
	}

	rad->prepared = true;

	return 0;
}

static int truly_panel_unprepare(struct drm_panel *panel)
{
	struct truly_panel *rad = to_truly_panel(panel);
	int ret;

	if (!rad->prepared)
		return 0;

	/*
	 * Right after asserting the reset, we need to release it, so that the
	 * touch driver can have an active connection with the touch controller
	 * even after the display is turned off.
	 */
	ret = regulator_bulk_disable(rad->num_supplies, rad->supplies);
	if (ret)
		return ret;

	rad->prepared = false;

	return 0;
}

static int kd035_enable(struct truly_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);
	int ret;

	dev_err(dev, "%s enterd\n", __FUNCTION__);

	if (panel->enabled){
		dev_err(dev, "%s quick ended\n", __FUNCTION__);
		return 0;
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

//	ret = rad_panel_push_cmd_list(dsi,
//				      &mcs_rm67191[0],
//				      ARRAY_SIZE(mcs_rm67191));
//	if (ret < 0) {
//		dev_err(dev, "Failed to send MCS (%d)\n", ret);
//		goto fail;
//	}

	/* Select User Command Set table (CMD1) */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ WRMAUCCTR, 0x00 }, 2);
	if (ret < 0)
		goto fail;

	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to do Software Reset (%d)\n", ret);
		goto fail;
	}

	usleep_range(15000, 17000);

	ret = mipid_kd035hvtia067_lcd_setup(panel);
	if (ret < 0)
		goto fail;

#if 0
	/* Set DSI mode */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0xC2, 0x0B }, 2);
	if (ret < 0) {
		dev_err(dev, "Failed to set DSI mode (%d)\n", ret);
		goto fail;
	}
	dev_err(dev, "%s %d\n", __FUNCTION__, __LINE__);
	/* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear ON (%d)\n", ret);
		goto fail;
	}
	/* Set tear scanline */
	if (ret < 0) {
		dev_err(dev, "Failed to set tear scanline (%d)\n", ret);
		goto fail;
	}

	/* Set pixel format */
	ret = mipi_dsi_dcs_set_pixel_format(dsi, color_format);
	dev_dbg(dev, "Interface color format set to 0x%x\n", color_format);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format (%d)\n", ret);
		goto fail;
	}

#endif
	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	usleep_range(5000, 7000);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}



	backlight_enable(panel->backlight);

	dev_err(dev, "%s ended\n", __FUNCTION__);

	panel->enabled = true;
	return 0;

fail:
//	gpiod_set_value_cansleep(panel->reset, 1);

	return ret;
}

static int truly_panel_enable(struct drm_panel *panel)
{
	struct truly_panel *rad = to_truly_panel(panel);

	return rad->pdata->enable(rad);
}

static int truly_panel_disable(struct drm_panel *panel)
{
	struct truly_panel *rad = to_truly_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (!rad->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	backlight_disable(rad->backlight);

	usleep_range(10000, 12000);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	rad->enabled = false;

	return 0;
}

static int truly_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = truly_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 truly_bus_formats,
					 ARRAY_SIZE(truly_bus_formats));
	return 1;
}

static int truly_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct truly_panel *rad = mipi_dsi_get_drvdata(dsi);
	u16 brightness;
	int ret;

	if (!rad->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int truly_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct truly_panel *rad = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!rad->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops truly_bl_ops = {
	.update_status = truly_bl_update_status,
	.get_brightness = truly_bl_get_brightness,
};

static const struct drm_panel_funcs truly_panel_funcs = {
	.prepare = truly_panel_prepare,
	.unprepare = truly_panel_unprepare,
	.enable = truly_panel_enable,
	.disable = truly_panel_disable,
	.get_modes = truly_panel_get_modes,
};

static const char * const truly_supply_names[] = {
	"v3p3",
	"v1p8",
};

static int truly_init_regulators(struct truly_panel *rad)
{
	struct device *dev = &rad->dsi->dev;
	int i;

	rad->num_supplies = ARRAY_SIZE(truly_supply_names);
	rad->supplies = devm_kcalloc(dev, rad->num_supplies,
				     sizeof(*rad->supplies), GFP_KERNEL);
	if (!rad->supplies)
		return -ENOMEM;

	for (i = 0; i < rad->num_supplies; i++)
		rad->supplies[i].supply = truly_supply_names[i];

	return devm_regulator_bulk_get(dev, rad->num_supplies, rad->supplies);
};

static const struct truly_platform_data truly_kd035 = {
	.enable = &kd035_enable,
};

static const struct of_device_id truly_of_match[] = {
	{ .compatible = "truly,kd035", .data = &truly_kd035 },
	{ .compatible = "truly,kd035-1", .data = &truly_kd035 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, truly_of_match);

static int truly_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct of_device_id *of_id = of_match_device(truly_of_match, dev);
	struct device_node *np = dev->of_node;
	struct truly_panel *panel;
	struct backlight_properties bl_props;
	int ret;
	u32 video_mode;

	if (!of_id || !of_id->data)
		return -ENODEV;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;
	panel->pdata = of_id->data;

	dsi->format = MIPI_DSI_FMT_RGB565; //MIPI_DSI_FMT_RGB666_PACKED;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE |
			  MIPI_DSI_MODE_VIDEO |
			  MIPI_DSI_MODE_EOT_PACKET;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;
		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	panel->reset = devm_gpiod_get_optional(dev, "mipi_reset",
					       GPIOD_OUT_HIGH); // |
//					       GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(panel->reset)) {
		ret = PTR_ERR(panel->reset);
		dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
		return ret;
	}

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_PLATFORM;	//BACKLIGHT_RAW;
//	bl_props.brightness = 7;
//	bl_props.max_brightness = 7;

	panel->backlight = devm_backlight_device_register(dev, dev_name(dev),
							  dev, dsi, &truly_bl_ops,
							  &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	ret = truly_init_regulators(panel);
	if (ret)
		return ret;

	drm_panel_init(&panel->panel, dev, &truly_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&panel->panel);

	printk("Truly KD035 panel probed %d Mode:%d Lanes:%d\n", ret, video_mode, dsi->lanes);
	return ret;
}

static int truly_panel_remove(struct mipi_dsi_device *dsi)
{
	struct truly_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(dev, "Failed to detach from host (%d)\n", ret);

	drm_panel_remove(&rad->panel);

	return 0;
}

static void truly_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct truly_panel *rad = mipi_dsi_get_drvdata(dsi);

	truly_panel_disable(&rad->panel);
	truly_panel_unprepare(&rad->panel);
}

static struct mipi_dsi_driver truly_panel_driver = {
	.driver = {
		.name = "panel-truly-kd035",
		.of_match_table = truly_of_match,
	},
	.probe = truly_panel_probe,
	.remove = truly_panel_remove,
//	.shutdown = truly_panel_shutdown,
};
module_mipi_dsi_driver(truly_panel_driver);

MODULE_AUTHOR("Ron Donio <ron.d@ms-technologies.com>");
MODULE_DESCRIPTION("DRM Driver for Truly kd035hvtia067 MIPI DSI panel");
MODULE_LICENSE("GPL v2");

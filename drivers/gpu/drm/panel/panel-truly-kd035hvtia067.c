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

static const struct drm_display_mode default_mode = {
	.name = "KD035HVTIA067", 	/* name (optional) */
	.clock = 12500,
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
static void kd035_set_maximum_return_packet_size(struct truly_panel *panel, u16 size)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_set_maximum_return_packet_size(dsi, size);
	if (ret < 0) {
		dev_err(dev, "error %d setting maximum return packet size to %d\n", ret, size);
	}
}

static int kd035_dcs_read(struct truly_panel *panel, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;

	int ret;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(dev, "error %d reading dcs seq(%#x)\n", ret, cmd);
	}

	return ret;
}
static void kd035_read_mtp_id(struct truly_panel *panel)
{
	u8 id[3];
	int ret;
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;

	kd035_set_maximum_return_packet_size(panel, 3);

	ret = kd035_dcs_read(panel, 0x04, id, ARRAY_SIZE(id));
	if (ret < 0 || ret < ARRAY_SIZE(id) || id[0] == 0x00) {
		dev_err(dev, "read id failed\n");
		return;
	}

	dev_info(dev, "ID: 0x%2x, 0x%2x, 0x%2x 0x%2x\n", id[0], id[1], id[2], id[3]);
}


static int kd035_read_mtp_brightness(struct truly_panel *panel)
{
	u8 brightness = 0;
	int ret;
	u8 buf[DSI_CMD_BUF_MAXSIZE];
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;

	kd035_set_maximum_return_packet_size(panel, 1);

//	ret = kd035_dcs_read(panel, MIPI_DCS_GET_DISPLAY_BRIGHTNESS, &brightness, sizeof(brightness));
	buf[0] = MIPI_DCS_GET_DISPLAY_BRIGHTNESS;
	ret =  mipi_dsi_generic_read(dsi, &buf[0], 1,  &brightness, 1);

	if (ret < 0 || brightness == 0x00) {
		dev_err(dev, "read brightness failed %d\n", ret);
		return ret;
	}

	dev_info(dev, "brightness: 0x%x\n", brightness);

	return brightness;
}

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

	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi_generic_write(dsi, buf, 0);
	dev_dbg(dev, "MIPI_DSI_MAX_RET_PACK_SIZE...\n");
	CHECK_RETCODE(err);


	err = mipi_dsi_dcs_cmd(panel, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	dev_dbg(dev, "MIPI DSI COM43H Sleep out...\n");
	CHECK_RETCODE(err);
	msleep(200);

	err = mipi_dsi_dcs_cmd(panel, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	dev_dbg(dev, "MIPI DSI Display off...\n");
	CHECK_RETCODE(err);
	msleep(120);

	kd035_set_maximum_return_packet_size(panel, 4);
	buf[0] = 0xD3;
	err =  mipi_dsi_generic_read(dsi, &buf[0], 2,  read_buf, 4);
	if (err >=0 ){
		model = (read_buf[0] & 0xFF00) >> 8;
		id    = (read_buf[0] & 0xFF0000) >> 16;
		dev_info(dev, "MIPI DSI LCD ID:0x%x MODEL:0x%x.\n", id, model);
	}

	/* Positive Gamma Control */
	buf[0] = 0x0E0400E0;
	buf[1] = 0x400A1708;
	buf[2] = 0x0E074D79;
	buf[3] = 0x0F1D1A0A;
	err = mipi_dsi_generic_write(dsi, buf, 16);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Positive Gamma Control...\n");
	CHECK_RETCODE(err);

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

	/* Power Control 2 */
	buf[0] = 0x41c1;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 POWER CONTROL MODE 2...\n");
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
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Memory Access control...\n");
	msleep(5);

	/* Interface Mode Control 18BIT RGB666 */
	buf[0] = 0x553A; //0x773a;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Interface pixel format...\n");
	msleep(5);

//	/* Frame Rate Control Frame rate 60HZ */
	buf[0] = 0x11A0B1;
//	err = mipi_dsi_generic_write(dsi, buf, 3);
	err = mipi_dsi_generic_write(dsi, buf, 4);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Frame Rate Control...\n");
	msleep(5);


	/* Display Inversion Control */
	buf[0] = 0x02B4;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Display Inversion Control...\n");
	msleep(5);

	/* Set Image Function */
	buf[0] = 0x00E9;
	err = mipi_dsi_generic_write(dsi, buf, 2);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Set Image Function...\n");
	msleep(5);

	/* Adjust Control 3 */
	buf[0] = 0x2C51A9F7;
	buf[1] = 0x82;
//	err = mipi_dsi_generic_write(dsi, buf, 5);
	err = mipi_dsi_generic_write(dsi, buf, 8);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Adjust Control 3...\n");

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
//	err = mipi_dsi_generic_write(dsi, buf, 5);
	err = mipi_dsi_generic_write(dsi, buf, 8);
	CHECK_RETCODE(err);
	dev_dbg(dev, "MIPI DSI KD035HVTIA067 Column Address Set...OK.\n");

	/* Page Address Set */
	buf[0] = 0x0100002B;
	buf[1] = 0xDF;
//	err = mipi_dsi_generic_write(dsi, buf, 5);
	err = mipi_dsi_generic_write(dsi, buf, 8);
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
#if 0
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
#endif


static inline struct truly_panel *to_truly_panel(struct drm_panel *panel)
{
	return container_of(panel, struct truly_panel, panel);
}

#if 0
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
#endif

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

	if (panel->enabled){
		return 0;
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

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

	/* Set DSI mode */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0xC2, 0x0B }, 2);
	if (ret < 0) {
		dev_err(dev, "Failed to set DSI mode (%d)\n", ret);
		goto fail;
	}

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

	kd035_read_mtp_id(panel);

	backlight_enable(panel->backlight);

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
	struct device *dev = &dsi->dev;
	u16 brightness;
	int ret;

	if (!rad->prepared)
		return 0;
	kd035_read_mtp_brightness(rad);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	kd035_set_maximum_return_packet_size(rad, 2);

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0){
		dev_err(dev, "Failed to get brightness%d\n", ret);
		return ret;
	}
	printk("%s:%d brightness=0x%x", __FUNCTION__, __LINE__, brightness);

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int truly_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct truly_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret = 0;

	if (!rad->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	printk("%s:%d brightness=0x%x", __FUNCTION__, __LINE__, bl->props.brightness);
//	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	ret = mipi_dsi_generic_write(dsi, (u8[]){ MIPI_DCS_SET_DISPLAY_BRIGHTNESS, (u8)bl->props.brightness }, 2);

	if (ret < 0){
		dev_err(dev, "Failed to update brightness 0x%x %d\n", bl->props.brightness, ret);
		return ret;
	}
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

#if 0
//In MS-TECH design we don't use regulators
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
#endif

static const struct truly_platform_data truly_kd035 = {
	.enable = &kd035_enable,
};

static const struct of_device_id truly_of_match[] = {
	{ .compatible = "truly,kd035", .data = &truly_kd035 },
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

	dsi->format = MIPI_DSI_FMT_RGB666_PACKED; //MIPI_DSI_FMT_RGB565;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE |
			  MIPI_DSI_MODE_VIDEO |
			  MIPI_DSI_MODE_LPM |
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
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 200;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(dev, dev_name(dev),
							  dev, dsi, &truly_bl_ops,
							  &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

#if 0
	ret = truly_init_regulators(panel);
	if (ret)
		return ret;
#endif
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
#if 0
static void truly_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct truly_panel *rad = mipi_dsi_get_drvdata(dsi);

	truly_panel_disable(&rad->panel);
	truly_panel_unprepare(&rad->panel);
}
#endif
static struct mipi_dsi_driver truly_panel_driver = {
	.driver = {
		.name = "panel-truly-kd035",
		.of_match_table = truly_of_match,
	},
	.probe = truly_panel_probe,
	.remove = truly_panel_remove,
#if 0
	.shutdown = truly_panel_shutdown,
#endif
};
module_mipi_dsi_driver(truly_panel_driver);

MODULE_AUTHOR("Ron Donio <ron.d@ms-technologies.com>");
MODULE_DESCRIPTION("DRM Driver for Truly kd035hvtia067 MIPI DSI panel");
MODULE_LICENSE("GPL v2");

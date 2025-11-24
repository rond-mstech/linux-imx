// SPDX-License-Identifier: GPL-2.0
/*
 * TFT-H050A11FWIST5N20 MIPI-DSI panel driver
 *
 * Copyright 2025 MSTECH
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
static const u32 st7701_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 st7701_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE;

struct st7701_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct gpio_desc *enable;
	struct backlight_device *backlight;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;

	const struct st7701_platform_data *pdata;
};

struct st7701_platform_data {
	int (*enable)(struct st7701_panel *panel);
};

#define HSPW 10
#define HBPD 30
#define HFPD 32
#define HDP 480
#define VSPW 4
#define VBPD 20
#define VFPD 20
#define VDP 854

// #define HSPW 10
// #define HBPD 30
// #define HFPD 32
// #define HDP 480
// #define VSPW 4
// #define VBPD 20
// #define VFPD 20
// #define VDP 854
// External system porch setting:
// Line Time：19 uS
// Frame Rate：60 Hz

static const struct drm_display_mode default_mode = {
	.name = "h050HVTIA067", 	/* name (optional) */
	.clock = 29050,            // kHz
        .hdisplay = 480,
        .hsync_start = 480 + 30,   // HDP + HBPD = 510
        .hsync_end   = 510 + 10,   // + HSPW = 520
        .htotal      = 480 + 30 + 10 + 32, // 552
        .vdisplay = 854,
        .vsync_start = 854 + 20,   // VDP + VBPD = 874
        .vsync_end   = 874 + 4,    // + VSPW = 878
        .vtotal      = 854 + 20 + 4 + 20, // 898
        
	.width_mm = 62,   /* as you already set */
	.height_mm = 110,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

/*
 * Ported for compatibilty
 */
static int mipi_dsi_dcs_cmd(struct st7701_panel *panel,
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
		dev_dbg(dev, "MIPI DSI DCS Command h050HVTIA067 ...OK.\n");
		break;

	default:
	dev_err(dev,
			"MIPI DSI DCS Command:0x%x Not supported!\n", cmd);
		break;
	}

	return err;
}


static void h050_clear_black(struct mipi_dsi_device *dsi)
{
    struct device *dev = &dsi->dev;
    int ret;

    const u16 width  = 480;
    const u16 height = 854;

    /* Black pixel in RGB888 */
    u8 pixel[3] = {0x00, 0x00, 0x00};

    /* 1. Set the column and page addresses */
    ret = mipi_dsi_dcs_set_column_address(dsi, 0, width - 1);
    if (ret < 0) {
        dev_err(dev, "ipi_dsi_dcs_set_column_address(): error: %d\n", ret);
        return;
    }

    ret = mipi_dsi_dcs_set_page_address(dsi, 0, height - 1);
    if (ret < 0) {
        dev_err(dev, "mipi_dsi_dcs_set_page_address(): error: %d\n", ret);
        return;
    }
    
    /* 2. Send WRITE MEMORY START (0x2C) */
    ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_MEMORY_START, NULL, 0);
    if (ret < 0) {
        dev_err(dev, "MIPI_DCS_WRITE_MEMORY_START: error: %d\n", ret);
        return;
    }
    
    /* 3. Fill full frame with black pixels */
    u16 y, x;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            /* Use CONTINUE (0x3C) to send pixels */
            ret = mipi_dsi_dcs_write(dsi,
                                     MIPI_DCS_WRITE_MEMORY_CONTINUE,
                                     pixel,
                                     sizeof(pixel));
        }
    }
}

// doesn't work on st7701 driver 
static int h050a11_get_rddid(struct mipi_dsi_device *dsi)
{
    struct device *dev = &dsi->dev;
    
    u8 cmd = 0x04;
    u8 id[4];
    int ret;
    ret = mipi_dsi_generic_read(dsi, &cmd, 1, id, sizeof(id));
    if (ret < 0) {
        dev_err(dev, "RDDID read failed: err: %d\n", ret);
    } else {
        // dev_info(dev, "RDDID: %*ph, ret: %d\n", (int)sizeof(id), id, ret);
        if (id[1] >= 0xD0 && id[1] <= 0xD4 && id[2] == 0x11) {
            dev_info(dev, "PANEL-NAME-H050HVTIA067\n");
        }else{
            dev_info(dev, "RDDID: %*ph\n", (int)sizeof(id), id);
        }
    }
    return ret;
}

static void h050_set_maximum_return_packet_size(struct st7701_panel *panel, u16 size)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_set_maximum_return_packet_size(dsi, size);
	if (ret < 0) {
		dev_err(dev, "error %d setting maximum return packet size to %d\n", ret, size);
	}
}

static int h050_dcs_read(struct st7701_panel *panel, u8 cmd, void *data, size_t len)
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
static void h050_read_mtp_id(struct st7701_panel *panel)
{
	u8 id[3];
	int ret;
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;

	h050_set_maximum_return_packet_size(panel, 3);

	ret = h050_dcs_read(panel, 0x04, id, ARRAY_SIZE(id));
	if (ret < 0 || ret < ARRAY_SIZE(id) || id[0] == 0x00) {
		dev_err(dev, "read id failed\n");
		return;
	}

	dev_info(dev, "ID: 0x%2x, 0x%2x, 0x%2x 0x%2x\n", id[0], id[1], id[2], id[3]);
}


static int h050_read_mtp_brightness(struct st7701_panel *panel)
{
	u8 brightness = 0;
	int ret;
	u8 buf[DSI_CMD_BUF_MAXSIZE];
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;

	h050_set_maximum_return_packet_size(panel, 1);

//	ret = h050_dcs_read(panel, MIPI_DCS_GET_DISPLAY_BRIGHTNESS, &brightness, sizeof(brightness));
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
static int mipid_h050hvtia067_lcd_setup(struct st7701_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	u32 read_buf[DSI_CMD_BUF_MAXSIZE];
	int err = 0;
	// u8 id, model;

	dev_info(dev, "MIPI DSI H050HVTIA067 LCD setup.\n");

/*
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

	h050_set_maximum_return_packet_size(panel, 4);
*/
        
        /* ---------------------- PAGE 0 ---------------------- */

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xFF, 0x77, 0x01, 0x00, 0x00, 0x10}, 6);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xC0, 0xE9, 0x03}, 3);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xC1, 0x11, 0x02}, 3);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xC2, 0x31, 0x08}, 3);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xCC, 0x10}, 2);
	CHECK_RETCODE(err);

	/* Positive Gamma */
	err = mipi_dsi_generic_write(dsi,
		(u8[]){
			0xB0,
			0x00, 0x0D, 0x14, 0x0D, 0x10, 0x05, 0x02, 0x08,
			0x08, 0x1E, 0x05, 0x13, 0x11, 0xA3, 0x29, 0x18
		}, 17);
	CHECK_RETCODE(err);

	/* Negative Gamma */
	err = mipi_dsi_generic_write(dsi,
		(u8[]){
			0xB1,
			0x00, 0x0C, 0x14, 0x0C, 0x10, 0x05, 0x03, 0x08,
			0x07, 0x20, 0x05, 0x13, 0x11, 0xA4, 0x29, 0x18
		}, 17);
	CHECK_RETCODE(err);


	/* ---------------------- PAGE 1 ---------------------- */

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xFF, 0x77, 0x01, 0x00, 0x00, 0x11}, 6);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xB0, 0x6C}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xB1, 0x43}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xB2, 0x07}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xB3, 0x80}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xB5, 0x47}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xB7, 0x85}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xB8, 0x20}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xB9, 0x10}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xC1, 0x78}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xC2, 0x78}, 2);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xD0, 0x88}, 2);
	CHECK_RETCODE(err);

	msleep(100);


	/* ---------------------- E0 – ED BLOCKS ---------------------- */

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xE0, 0x00, 0x00, 0x02}, 4);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){
			0xE1,
			0x08, 0x00, 0x0A, 0x00, 0x07, 0x00,
			0x09, 0x00, 0x00, 0x33, 0x33
		}, 12);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){
			0xE2,
			0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00
		}, 14);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xE3, 0x00, 0x00, 0x33, 0x33}, 5);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xE4, 0x44, 0x44}, 3);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){
			0xE5,
			0x0E,0x60,0xA0,0xA0,
			0x10,0x60,0xA0,0xA0,
			0x0A,0x60,0xA0,0xA0,
			0x0C,0x60,0xA0,0xA0
		}, 17);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xE6, 0x00, 0x00, 0x33, 0x33}, 5);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xE7, 0x44, 0x44}, 3);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){
			0xE8,
			0x0D,0x60,0xA0,0xA0,
			0x0F,0x60,0xA0,0xA0,
			0x09,0x60,0xA0,0xA0,
			0x0B,0x60,0xA0,0xA0
		}, 17);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){
			0xEB,
			0x02,0x01,0xE4,0xE4,0x44,0x00,0x40
		}, 8);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xEC, 0x02, 0x01}, 3);
	CHECK_RETCODE(err);

	err = mipi_dsi_generic_write(dsi,
		(u8[]){
			0xED,
			0xAB,0x89,0x76,0x54,0x01,
			0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
			0x10,0x45,0x67,0x98,0xBA
		}, 17);
	CHECK_RETCODE(err);


	/* ---------------------- FINAL PAGE SELECT ---------------------- */

	err = mipi_dsi_generic_write(dsi,
		(u8[]){0xFF, 0x77, 0x01, 0x00, 0x00, 0x00}, 6);
	CHECK_RETCODE(err);

	msleep(120);

        // err = h050a11_get_rddid(dsi);
        // CHECK_RETCODE(err);
        
	/* ---------------------- EXIT SLEEP & DISPLAY ON ---------------------- */

	err = mipi_dsi_dcs_write(dsi, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	CHECK_RETCODE(err);
	msleep(120);

        // err = h050a11_get_rddid(dsi);
        // CHECK_RETCODE(err);
        
	err = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	CHECK_RETCODE(err);
	msleep(120);
	
	// h050_clear_black(dsi);
	
	dev_info(dev, "MIPI DSI H050HVTIA067 Display init ...DONE.\n");

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

	dev_info(dev, "MIPI DSI H050HVTIA067 LCD setup ended.\n");

	return err;
}

static inline struct st7701_panel *to_st7701_panel(struct drm_panel *panel)
{
	return container_of(panel, struct st7701_panel, panel);
}

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

static int st7701_panel_prepare(struct drm_panel *panel)
{
	struct st7701_panel *rad = to_st7701_panel(panel);
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

static int st7701_panel_unprepare(struct drm_panel *panel)
{
	struct st7701_panel *rad = to_st7701_panel(panel);
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

static int h050_enable(struct st7701_panel *panel)
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

	ret = mipid_h050hvtia067_lcd_setup(panel);
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

	h050_read_mtp_id(panel);

	backlight_enable(panel->backlight);

	panel->enabled = true;
	return 0;

fail:
//	gpiod_set_value_cansleep(panel->reset, 1);

	return ret;
}

static int st7701_panel_enable(struct drm_panel *panel)
{
	struct st7701_panel *rad = to_st7701_panel(panel);

	return rad->pdata->enable(rad);
}

static int st7701_panel_disable(struct drm_panel *panel)
{
	struct st7701_panel *rad = to_st7701_panel(panel);
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

static int st7701_panel_get_modes(struct drm_panel *panel,
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
	connector->display_info.bus_flags = st7701_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 st7701_bus_formats,
					 ARRAY_SIZE(st7701_bus_formats));
	return 1;
}

static int st7701_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct st7701_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	u16 brightness;
	int ret;

	if (!rad->prepared)
		return 0;
	h050_read_mtp_brightness(rad);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	h050_set_maximum_return_packet_size(rad, 2);

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0){
		dev_err(dev, "Failed to get brightness%d\n", ret);
		return ret;
	}
	printk("%s:%d brightness=0x%x", __FUNCTION__, __LINE__, brightness);

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int st7701_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct st7701_panel *rad = mipi_dsi_get_drvdata(dsi);
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

static const struct backlight_ops st7701_bl_ops = {
	.update_status = st7701_bl_update_status,
	.get_brightness = st7701_bl_get_brightness,
};

static const struct drm_panel_funcs st7701_panel_funcs = {
	.prepare = st7701_panel_prepare,
	.unprepare = st7701_panel_unprepare,
	.enable = st7701_panel_enable,
	.disable = st7701_panel_disable,
	.get_modes = st7701_panel_get_modes,
};

static const struct st7701_platform_data st7701_h050 = {
	.enable = &h050_enable,
};

static const struct of_device_id st7701_of_match[] = {
	{ .compatible = "mst,h050a11fwist5c20", .data = &st7701_h050 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, st7701_of_match);

static int st7701_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct of_device_id *of_id = of_match_device(st7701_of_match, dev);
	struct device_node *np = dev->of_node;
	struct st7701_panel *panel;
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

	//dsi->format = MIPI_DSI_FMT_RGB666_PACKED; //MIPI_DSI_FMT_RGB565;
	dsi->format = MIPI_DSI_FMT_RGB888;
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
							  dev, dsi, &st7701_bl_ops,
							  &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

#if 0
	ret = st7701_init_regulators(panel);
	if (ret)
		return ret;
#endif
	drm_panel_init(&panel->panel, dev, &st7701_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&panel->panel);

	printk("MST H050a11fwist5c20 panel probed %d Mode:%d Lanes:%d\n", ret, video_mode, dsi->lanes);
	return ret;
}

static int st7701_panel_remove(struct mipi_dsi_device *dsi)
{
	struct st7701_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(dev, "Failed to detach from host (%d)\n", ret);

	drm_panel_remove(&rad->panel);

	return 0;
}
#if 0
static void st7701_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct st7701_panel *rad = mipi_dsi_get_drvdata(dsi);

	st7701_panel_disable(&rad->panel);
	st7701_panel_unprepare(&rad->panel);
}
#endif
static struct mipi_dsi_driver st7701_panel_driver = {
	.driver = {
		.name = "panel-h050a11fwist5c20",
		.of_match_table = st7701_of_match,
	},
	.probe = st7701_panel_probe,
	.remove = st7701_panel_remove,
#if 0
	.shutdown = st7701_panel_shutdown,
#endif
};
module_mipi_dsi_driver(st7701_panel_driver);

MODULE_AUTHOR("Andrey Turchenko <andrey.t@ms-technologies.com>");
MODULE_DESCRIPTION("MST TFT-H050A11FWIST5C20 MIPI DSI panel driver");
MODULE_LICENSE("GPL v2");

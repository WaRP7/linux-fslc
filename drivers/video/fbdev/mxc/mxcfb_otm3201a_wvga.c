/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>
#include <linux/gpio.h>

#include "mipi_dsi.h"

#define OTM3201A_DATA_LANE					(0x1)
#define OTM3201A_MAX_DPHY_CLK					(800)

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static void parse_variadic(int n, u8 *buf, ...)
{
	int i = 0;
	va_list args;

	if (unlikely(!n)) return;

	va_start(args, buf);

	for (i = 0; i < n; i++){
		buf[i + 1] = (u8)va_arg(args, int);
	}
	va_end(args);
}

#define TC358763_DCS_write_1A_nP(n, addr, ...) {		\
	int err;						\
								\
	buf[0] = addr;						\
	parse_variadic(n, buf, ##__VA_ARGS__);			\
								\
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,		\
		MIPI_DSI_GENERIC_LONG_WRITE, (u32*)buf, n + 1);	\
	CHECK_RETCODE(err);					\
}

#define TC358763_DCS_write_1A_0P(addr)		\
	TC358763_DCS_write_1A_nP(0, addr)

#define TC358763_DCS_write_1A_1P(addr, ...)	\
	TC358763_DCS_write_1A_nP(1, addr, __VA_ARGS__)

#define TC358763_DCS_write_1A_2P(addr, ...)	\
	TC358763_DCS_write_1A_nP(2, addr, __VA_ARGS__)

#define TC358763_DCS_write_1A_3P(addr, ...)	\
	TC358763_DCS_write_1A_nP(3, addr, __VA_ARGS__)

#define TC358763_DCS_write_1A_4P(addr, ...)	\
	TC358763_DCS_write_1A_nP(4, addr, __VA_ARGS__)

#define TC358763_DCS_write_1A_5P(addr, ...)	\
	TC358763_DCS_write_1A_nP(5, addr, __VA_ARGS__)

#define TC358763_DCS_write_1A_6P(addr, ...)	\
	TC358763_DCS_write_1A_nP(6, addr, __VA_ARGS__)

#define TC358763_DCS_write_1A_12P(addr, ...)	\
	TC358763_DCS_write_1A_nP(12, addr, __VA_ARGS__)

#define TC358763_DCS_write_1A_14P(addr, ...)	\
	TC358763_DCS_write_1A_nP(14, addr, __VA_ARGS__)

#define TC358763_DCS_write_1A_17P(addr, ...)	\
	TC358763_DCS_write_1A_nP(17, addr, __VA_ARGS__)

// setting from truly
#define REFRESH      53
#define XRES         320
#define YRES         320
#define LEFT_MARGIN  42
#define RIGHT_MARGIN 26
#define UPPER_MARGIN 10
#define LOWER_MARGIN 2
#define HSYNC_LEN    10
#define VSYNC_LEN    2
#define PIXCLOCK (1e12/((XRES+LEFT_MARGIN+RIGHT_MARGIN+HSYNC_LEN)*(YRES+UPPER_MARGIN+LOWER_MARGIN+VSYNC_LEN)*REFRESH))


static int otm3201abl_brightness;
static struct fb_videomode truly_lcd_modedb[] = {
    {
        "TRULY-WVGA",                 /* name */
        REFRESH,                    /* refresh /frame rate */
        XRES, YRES,                 /* resolution */
        PIXCLOCK,                   /* pixel clock*/
        LEFT_MARGIN, RIGHT_MARGIN,  /* l/r margin */
        UPPER_MARGIN, LOWER_MARGIN, /* u/l margin */
        HSYNC_LEN, VSYNC_LEN,       /* hsync/vsync length */
        FB_SYNC_OE_LOW_ACT,         /* sync */
        FB_VMODE_NONINTERLACED,     /* vmode */
        0,                          /* flag */
    },
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num  = OTM3201A_DATA_LANE,
	.max_phy_clk    = OTM3201A_MAX_DPHY_CLK,
	.dpi_fmt	= MIPI_RGB565_PACKED, /*MIPI_RGB888,*/
};

void mipid_otm3201a_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}


int mipid_otm3201a_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u8 buf[DSI_CMD_BUF_MAXSIZE];

	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD setup.\n");

	TC358763_DCS_write_1A_2P(0xF0,0x54,0x47);
	TC358763_DCS_write_1A_1P(0xA0,0x00);
	TC358763_DCS_write_1A_1P(0xB1,0x22);


	TC358763_DCS_write_1A_5P(0xB3,0x02,0x0A,0x1A,0x2A,0x2A);
	TC358763_DCS_write_1A_3P(0xBD,0x00,0x11,0x31);

	TC358763_DCS_write_1A_4P(0xBA,0x05,0x15,0x2B,0x01);
	TC358763_DCS_write_1A_1P(0xE9,0x46);

	TC358763_DCS_write_1A_1P(0xE2,0xF5);
	TC358763_DCS_write_1A_4P(0xB5,0x45,0x73,0x7a,0xFa);


	TC358763_DCS_write_1A_17P(0xC0,0x00,0x01,0x09,0x12,0x17,0x27,0x0C,0x0A,0x0E,0x0D,0x0B,0x2D,0x0D,0x11,0x25,0X2A,0X3F);

	TC358763_DCS_write_1A_17P(0xC1,0x00,0x00,0x08,0x12,0x16,0x27,0x0C,0x0A,0x04,0x0D,0x0B,0x2D,0x0D,0x11,0x25,0x2A,0x3F);

	TC358763_DCS_write_1A_17P(0xC2,0x00,0x01,0x09,0x12,0x17,0x27,0x0C,0x0A,0x0E,0x0D,0x0B,0x2D,0x0D,0x11,0x25,0x2A,0x3F);

	TC358763_DCS_write_1A_17P(0xC3,0x00,0x00,0x08,0x12,0x16,0x27,0x0C,0x0A,0x04,0x0D,0x0B,0x2D,0x0D,0x11,0x25,0x2A,0x3F);

	TC358763_DCS_write_1A_17P(0xC4,0x00,0x01,0x09,0x12,0x17,0x27,0x0C,0x0A,0x0E,0x0D,0x0B,0x2D,0x0D,0x11,0x25,0x2A,0x3F);

	TC358763_DCS_write_1A_17P(0xC5,0x00,0x00,0x08,0x12,0x16,0x27,0x0C,0x0A,0x04,0x0D,0x0B,0x2D,0x0D,0x11,0x25,0X2A,0X3F);


	TC358763_DCS_write_1A_1P(0x36,0x00);

	TC358763_DCS_write_1A_0P(0x11);
	msleep(100);

	TC358763_DCS_write_1A_0P(0x29);
	msleep(100);

	return 0;
}


static int mipid_bl_update_status(struct backlight_device *bl)
{
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return otm3201abl_brightness;
}

static int mipi_bl_check_fb(struct backlight_device *bl, struct fb_info *fbi)
{
	return 0;
}

static const struct backlight_ops mipid_lcd_bl_ops = {
	.update_status = mipid_bl_update_status,
	.get_brightness = mipid_bl_get_brightness,
	.check_fb = mipi_bl_check_fb,
};

/*
 * ST7796 Display Controller driver
 *
 * Copyright (c) 2016 Embest Tech, Inc.
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <video/mipi_display.h>
#include <linux/gpio.h>
#include "mipi_dsi.h"


#define ROTATE_DISPLAY		1
// TODO: read interface do not work
#define DUMP_REGISTER		0

#define CMD_DFC_A3			0xB6
#define ARG_DFC_1_GS		0x40
#define ARG_DFC_1_SS		0x20
#define ARG_DFC_1_SM		0x10

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

#if DUMP_REGISTER
static int st7796h_dcs_read(struct mipi_dsi_info *mipi_dsi, const char* name, int addr, int bytes) {
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	int err;

#define MIPI_DSI_MAX_RET_PACK_SIZE				(0x4)
	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
				buf, 0);
	CHECK_RETCODE(err);

	buf[0] = addr;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_DCS_LONG_WRITE,
				buf, 1);
	CHECK_RETCODE(err);

	buf[0] = 0xFFFF & bytes;
	err = mipi_dsi->mipi_dsi_pkt_read(mipi_dsi,
		MIPI_DSI_DCS_READ,
		buf, bytes);
	CHECK_RETCODE(err);

	if (err >= 0) {
		dev_notice(&mipi_dsi->pdev->dev, "REGS %s[ 0x%02X ] = 0x%08X\n", name, addr, buf[0]);
	}
	return err;
}
#endif

static int st7796h_parse_vararg(u8 *buf, ...)
{
	va_list args;
	int i, t;

	va_start(args, buf);

	for (i = 1; i < DSI_CMD_BUF_MAXSIZE; i++) {
		if ((t = va_arg(args, int)) < 0) {
			break;
		}
		buf[i] = (u8)t;
	}

	va_end(args);

	return i;
}

#define st7796h_dcs_write(addr, ...) {				\
	int n, err;						\
								\
	buf[0] = addr;						\
	n = st7796h_parse_vararg(buf, ##__VA_ARGS__, -1);	\
								\
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,		\
		/*MIPI_DSI_GENERIC_LONG_WRITE*/			\
		MIPI_DSI_DCS_LONG_WRITE, (u32*)buf, n);		\
	CHECK_RETCODE(err);					\
}


#define REFRESH		60
#define XRES		320
#define YRES		320
// VBP
#define UPPER_MARGIN	4
// VFP
#define LOWER_MARGIN	8
// VSA
#define VSYNC_LEN	4
// HBP
#define LEFT_MARGIN	4
// HFP
#define RIGHT_MARGIN	8
// HSA
#define HSYNC_LEN	4
#define PIXCLOCK	(1e12/((XRES+LEFT_MARGIN+RIGHT_MARGIN+HSYNC_LEN)*(YRES+UPPER_MARGIN+LOWER_MARGIN+VSYNC_LEN)*REFRESH))


static struct fb_videomode truly_lcd_modedb[] = {
	{
		"TRULY-WVGA",               /* name */
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
	.data_lane_num  = 1,
	.max_phy_clk    = 300,	// MHz
	.dpi_fmt	= MIPI_RGB888,
};

void mipid_st7796h_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}

#if DUMP_REGISTER
struct rd_regs {
	const char* name;
	int addr;
	int bytes;
} rd_regs[] = {
	"RDDID",  0x04, 4,
	"RD#ERR", 0x05, 2,
	"RDDPM",  0x0A, 2,
	"RDDSM",  0x0E, 2,
	"BPC",    0xB5, 4,
};
#endif

int mipid_st7796h_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u8 buf[DSI_CMD_BUF_MAXSIZE];
	int a1;

	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD setup.\n");

	// Enable Command 2 Part I
	st7796h_dcs_write(0xF0,0xC3);
	// Enable Command 2 Part II
	st7796h_dcs_write(0xF0,0x96);

	// Interface Pixel Format '0111 0111'=24bit/24bit
	st7796h_dcs_write(0x3A,0x77);

	// Frame Rate Control
	//st7796h_dcs_write(0xB1,0x80,0x10);

	/*
	 * Most these setting cause st7796h is a 320x480 display controller
	 * while the liquid is 320x320.
	 */

	// Display Function Control
	// st7796h_dcs_write(0xB6,0x80,0x62,0x27);
	a1 = 0x02;
	#if ROTATE_DISPLAY
	a1 |= ARG_DFC_1_GS;
	#else
	a1 |= ARG_DFC_1_SS;
	#endif
	st7796h_dcs_write(CMD_DFC_A3,0x00,a1,0x27);

	// Column Address Set
	st7796h_dcs_write(0x2A,0x00,0x00,0x01,0x3F);
	// Row Address Set
	st7796h_dcs_write(0x2B,0x00,0x00,0x01,0x3F);

	// Don't using partial area
	#if 0
	// Partial Area, 0 - 319
	st7796h_dcs_write(0x30,0x00,0x00,0x01,0x3F);

	// Partial Display Mode On
	st7796h_dcs_write(0x12);
	#endif

	#if 1
	// Vertical Scrolling Control
	// TFA = 0, VSA = 320, BFA = 0
	st7796h_dcs_write(0x33,0x00,0x00,0x01,0x40,0x00,0x00);

	// Vertical Scrolling Start Address of RAM
	st7796h_dcs_write(0x37,0x00,0x00);
	#endif

	// Memory Data Access Control
	// D7..D0 = MY,MX,MV,ML,RGB,MH,-,-
	// MY=0,MX=0,RGB=1
	st7796h_dcs_write(0x36,0x08);

	// Tearing Effect Line On
	// st7796h_dcs_write(0x35,0x00);

	// Display Inversion Control
	// st7796h_dcs_write(0xB4,0x01);

	// Vcom Control, 1.65V
	st7796h_dcs_write(0xC5,0x36);

	// Power Control 1
	st7796h_dcs_write(0xC0,0x80,0x35);
	// Power Control 2
	st7796h_dcs_write(0xC1,0x1C);

	// Mode Selection
	// st7796h_dcs_write(0xB9,0x02,0x00);

	// Display Output Control Adjust
	st7796h_dcs_write(0xE8,0x40,0x8A,0x00,0x00,0x29,0x19,0xA5,0x33);
	// Postive Gamma Control
	st7796h_dcs_write(0xE0,0xF0,0x00,0x02,0x04,0x03,0x13,0x2A,0x65,0x46,0x2B,0x17,0x15,0x1D,0x24);
	// Negtive Gamma Control
	st7796h_dcs_write(0xE1,0xF0,0x00,0x01,0x04,0x03,0x22,0x2C,0x33,0x44,0x2C,0x17,0x16,0x1C,0x1E);
	st7796h_dcs_write(0xEC,0x00,0x00,0x01);

	// Disable Command 2 Part I
	st7796h_dcs_write(0xF0,0x3C);
	// Disable Command 2 Part II
	st7796h_dcs_write(0xF0,0x69);

	// Sleep Out
	st7796h_dcs_write(0x11);
	msleep(150);

	// Display On
	st7796h_dcs_write(0x29);
	msleep(200);

	#if DUMP_REGISTER
	{
		int i;

		for (i = 0; i < ARRAY_SIZE(rd_regs); i++) {
			st7796h_dcs_read(mipi_dsi,
				rd_regs[i].name, 
				rd_regs[i].addr, 
				rd_regs[i].bytes
			);
		}
	}

	msleep(10);

	#endif

	return 0;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return 0;
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


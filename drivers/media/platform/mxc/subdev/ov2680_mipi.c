/*
 * Copyright (C) 2011-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define OV2680_VOLTAGE_ANALOG               2800000
#define OV2680_VOLTAGE_DIGITAL_CORE         1500000
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define OV2680_VOLTAGE_DIGITAL_IO           1800000

#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV2680_XCLK_MIN 6000000
#define OV2680_XCLK_MAX 24000000

#define OV2680_CHIP_ID_HIGH_BYTE	0x300A
#define OV2680_CHIP_ID_LOW_BYTE		0x300B

enum ov2680_mode {
	ov2680_mode_MIN = 0,
	ov2680_mode_VGA_640_480 = 0,
	ov2680_mode_QUXGA_800_600 = 1,
	ov2680_mode_720P_1280_720 = 2,
	ov2680_mode_SXGA_1280_960 = 3,
	ov2680_mode_HD1600_1600_900 = 4,
	ov2680_mode_UXGA_1600_1200 = 5,
	ov2680_mode_MAX = 6,
	ov2680_mode_INIT = 0xff, /*only for sensor init*/
};

enum ov2680_frame_rate {
	ov2680_15_fps,
	ov2680_30_fps,
	ov2680_60_fps
};

static int ov2680_framerates[] = {
	[ov2680_15_fps] = 15,
	[ov2680_30_fps] = 30,
	[ov2680_60_fps] = 60,
};

struct ov2680_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

enum ov2680_downsize_mode {
/*
	cropping and binning

	cropping reduces the image size less than 0.5 of the horizoantal and 0.5 verital pixels.

	2 x 2 binning reduces the image by 0.5 horizontal and 0.5 vertical resulting in 1/4 of the image size */
	SUBSAMPLING,
	SCALING,

};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov2680_mode_info {
	enum ov2680_mode mode;
	enum ov2680_downsize_mode dn_mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct ov2680 {
	struct v4l2_subdev		subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct ov2680_datafmt	*fmt;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	void (*io_init)(void);
};
/*!
 * Maintains the information on the current state of the sesor.
 */
static struct ov2680 ov2680_data;
static int pwn_gpio, rst_gpio;

static struct reg_value ov2680_init_setting_30fps_QUXGA[] = {
/* 800 x 600 one quarter of the full resolution */
  {0x3002, 0x00, 0x00, 0x00}, //gpio0 input, vsync input, fsin input
  {0x3016, 0x1c, 0x00, 0x00}, //drive strength = 0x01, bypass latch of hs_enable
  {0x3018, 0x44, 0x00, 0x00}, //MIPI 10-bit mode
  {0x3020, 0x00, 0x00, 0x00}, //output raw
  {0x3080, 0x02, 0x00, 0x00}, //PLL
  {0x3082, 0x37, 0x00, 0x00}, //PLL
  {0x3084, 0x09, 0x00, 0x00}, //PLL
  {0x3085, 0x04, 0x00, 0x00}, //PLL
  {0x3086, 0x01, 0x00, 0x00}, //PLL
  {0x3501, 0x26, 0x00, 0x00}, //exposure M
  {0x3502, 0x40, 0x00, 0x00}, //exposure L
  {0x3503, 0x03, 0x00, 0x00}, //vts auto, gain manual, exposure manual
  {0x350b, 0x36, 0x00, 0x00}, //gain L
  {0x3600, 0xb4, 0x00, 0x00}, //analog control
  {0x3603, 0x39, 0x00, 0x00}, //
  {0x3604, 0x24, 0x00, 0x00}, //
  {0x3605, 0x00, 0x00, 0x00}, //
  {0x3620, 0x26, 0x00, 0x00}, //
  {0x3621, 0x37, 0x00, 0x00}, //
  {0x3622, 0x04, 0x00, 0x00}, //
  {0x3628, 0x00, 0x00, 0x00}, //analog control
  {0x3705, 0x3c, 0x00, 0x00}, //sennsor control
  {0x370c, 0x50, 0x00, 0x00}, //
  {0x370d, 0xc0, 0x00, 0x00}, //
  {0x3718, 0x88, 0x00, 0x00}, //
  {0x3720, 0x00, 0x00, 0x00}, //
  {0x3721, 0x00, 0x00, 0x00}, //
  {0x3722, 0x00, 0x00, 0x00}, //
  {0x3723, 0x00, 0x00, 0x00}, //
  {0x3738, 0x00, 0x00, 0x00}, //
  {0x370a, 0x23, 0x00, 0x00}, //
  {0x3717, 0x58, 0x00, 0x00}, //sensor control
  {0x3781, 0x80, 0x00, 0x00}, //PSRAM
  {0x3789, 0x60, 0x00, 0x00}, //PSRAM
  {0x3800, 0x00, 0x00, 0x00}, //x start H
  {0x3801, 0x00, 0x00, 0x00}, //x start L
  {0x3802, 0x00, 0x00, 0x00}, //y start H
  {0x3803, 0x00, 0x00, 0x00}, //y start L
  {0x3804, 0x06, 0x00, 0x00}, //x end H
  {0x3805, 0x4f, 0x00, 0x00}, //x end L
  {0x3806, 0x04, 0x00, 0x00}, //y end H
  {0x3807, 0xbf, 0x00, 0x00}, //y end L
  {0x3808, 0x03, 0x00, 0x00}, //x output size H
  {0x3809, 0x20, 0x00, 0x00}, //x output size L
  {0x380a, 0x02, 0x00, 0x00}, //y output size H
  {0x380b, 0x58, 0x00, 0x00}, //y output size L
  {0x380c, 0x06, 0x00, 0x00}, //HTS H
  {0x380d, 0xac, 0x00, 0x00}, //HTS L
  {0x380e, 0x02, 0x00, 0x00}, //VTS H
  {0x380f, 0x84, 0x00, 0x00}, //VTS L
  {0x3810, 0x00, 0x00, 0x00}, //ISP x win H
  {0x3811, 0x04, 0x00, 0x00}, //ISP x win L
  {0x3812, 0x00, 0x00, 0x00}, //ISP y win H
  {0x3813, 0x04, 0x00, 0x00}, //ISP y win L
  {0x3814, 0x31, 0x00, 0x00}, //x inc
  {0x3815, 0x31, 0x00, 0x00}, //y inc
  {0x3819, 0x04, 0x00, 0x00}, //vsync end row
  {0x3820, 0xc2, 0x00, 0x00}, //vsun48_blc, vflip_blc, vbinf
  {0x3821, 0x01, 0x00, 0x00}, //hbin enabled
  {0x4000, 0x81, 0x00, 0x00}, //avg_weight = 0x08, mf_en
  {0x4001, 0x40, 0x00, 0x00}, //format_trig_beh
  {0x4008, 0x00, 0x00, 0x00}, //blc_start
  {0x4009, 0x03, 0x00, 0x00}, //blc_end
  {0x4602, 0x02, 0x00, 0x00}, //frame reset enable
  {0x481f, 0x36, 0x00, 0x00}, //CLK PREPARE MIN
  {0x4825, 0x36, 0x00, 0x00}, //LPX P MIN
  {0x4837, 0x30, 0x00, 0x00}, //MIPI global timing
  {0x5002, 0x30, 0x00, 0x00}, //
  {0x5080, 0x00, 0x00, 0x00}, // pattern
  {0x5081, 0x41, 0x00, 0x00}, //window cut enable, random seed = 0x01
};



static struct reg_value ov2680_setting_30fps_VGA_640_480[] = {
};

static struct reg_value ov2680_setting_30fps_QUXGA_800_600[] = {
  {0x3086, 0x01, 0x00, 0x00}, //PLL
  {0x3501, 0x26, 0x00, 0x00}, //exposure M
  {0x3502, 0x40, 0x00, 0x00}, //exposure L
  {0x3620, 0x26, 0x00, 0x00}, //analog control
  {0x3621, 0x37, 0x00, 0x00}, //
  {0x3622, 0x04, 0x00, 0x00}, //analog control
  {0x370a, 0x23, 0x00, 0x00}, //sennsor control
  {0x370d, 0xc0, 0x00, 0x00}, //
  {0x3718, 0x88, 0x00, 0x00}, //
  {0x3721, 0x00, 0x00, 0x00}, //
  {0x3722, 0x00, 0x00, 0x00}, //
  {0x3723, 0x00, 0x00, 0x00}, //
  {0x3738, 0x00, 0x00, 0x00}, //sennsor control
  {0x3803, 0x00, 0x00, 0x00}, //y start L
  {0x3807, 0xbf, 0x00, 0x00}, //y end L
  {0x3808, 0x03, 0x00, 0x00}, //x output size H
  {0x3809, 0x20, 0x00, 0x00}, //x output size L
  {0x380a, 0x02, 0x00, 0x00}, //y output size H
  {0x380b, 0x58, 0x00, 0x00}, //y output size L
  {0x380c, 0x06, 0x00, 0x00}, //HTS H
  {0x380d, 0xac, 0x00, 0x00}, //HTS L
  {0x380e, 0x02, 0x00, 0x00}, //VTS H
  {0x380f, 0x84, 0x00, 0x00}, //VTS L
  {0x3811, 0x04, 0x00, 0x00}, //ISP x win L
  {0x3813, 0x04, 0x00, 0x00}, //ISP y win L
  {0x3814, 0x31, 0x00, 0x00}, //x inc
  {0x3815, 0x31, 0x00, 0x00}, //y inc
  {0x3820, 0xc2, 0x00, 0x00}, //vsun48_blc, vflip_blc, vbinf
  {0x3821, 0x01, 0x00, 0x00}, //hbin enabled
  {0x4008, 0x00, 0x00, 0x00}, //blc_start
  {0x4009, 0x03, 0x00, 0x00}, //blc_end
/*  {0x4837, 0x30, 0x00, 0x00},*/ //MIPI global timing
   {0x4837, 0x1e, 0x00, 0x00}, //MIPI global timing
};

static struct reg_value ov2680_setting_30fps_720P_1280_720[] = {
};

static struct reg_value ov2680_setting_30fps_SXGA_1280_960[] = {
};

static struct reg_value ov2680_setting_30fps_HD1600_1600_900[] = {
};

static struct reg_value ov2680_setting_30fps_UXGA_1600_1200[] = {
  {0x3086, 0x00, 0x00, 0x00}, //PLL
  {0x3501, 0x4e, 0x00, 0x00}, //exposure M
  {0x3502, 0xe0, 0x00, 0x00}, //exposure L
  {0x3620, 0x26, 0x00, 0x00}, //analog control
  {0x3621, 0x37, 0x00, 0x00}, //
  {0x3622, 0x04, 0x00, 0x00}, //analog control
  {0x370a, 0x21, 0x00, 0x00}, //sennsor control
  {0x370d, 0xc0, 0x00, 0x00}, //
  {0x3718, 0x88, 0x00, 0x00}, //
  {0x3721, 0x00, 0x00, 0x00}, //
  {0x3722, 0x00, 0x00, 0x00}, //
  {0x3723, 0x00, 0x00, 0x00}, //
  {0x3738, 0x00, 0x00, 0x00}, //sennsor control
  {0x3803, 0x00, 0x00, 0x00}, //y start L
  {0x3807, 0xbf, 0x00, 0x00}, //y end L
  {0x3808, 0x06, 0x00, 0x00}, //x output size H
  {0x3809, 0x40, 0x00, 0x00}, //x output size L
  {0x380a, 0x04, 0x00, 0x00}, //y output size H
  {0x380b, 0xb0, 0x00, 0x00}, //y output size L
  {0x380c, 0x06, 0x00, 0x00}, //HTS H
  {0x380d, 0xa4, 0x00, 0x00}, //HTS L
  {0x380e, 0x05, 0x00, 0x00}, //VTS H
  {0x380f, 0x0e, 0x00, 0x00}, //VTS L
  {0x3811, 0x08, 0x00, 0x00}, //ISP x win L
  {0x3813, 0x08, 0x00, 0x00}, //ISP y win L
  {0x3814, 0x11, 0x00, 0x00}, //x inc
  {0x3815, 0x11, 0x00, 0x00}, //y inc
  {0x3820, 0xc0, 0x00, 0x00}, //vsun48_blc, vflip_blc, vbin off
  {0x3821, 0x00, 0x00, 0x00}, //hbin off
  {0x4008, 0x02, 0x00, 0x00}, //blc_start
  {0x4009, 0x09, 0x00, 0x00}, //blc_end
  {0x4837, 0x18, 0x00, 0x00}, //MIPI global timing
};

static struct ov2680_mode_info ov2680_mode_info_data[3][ov2680_mode_MAX + 1] = {
	/* 15 frames per second mode info */
	{
		{ov2680_mode_VGA_640_480, -1, 0, 0, NULL, 0},
		{ov2680_mode_QUXGA_800_600, -1, 0, 0, NULL, 0},
		{ov2680_mode_720P_1280_720, -1, 0, 0, NULL, 0},
		{ov2680_mode_SXGA_1280_960, -1, 0, 0, NULL, 0},
		{ov2680_mode_HD1600_1600_900, -1, 0, 0, NULL, 0},
		{ov2680_mode_UXGA_1600_1200, -1, 0, 0, NULL, 0},
		{ov2680_mode_UXGA_1600_1200, -1, 0, 0, NULL, 0},
	},
	/* 30 fps mode info */
	{
		{ov2680_mode_VGA_640_480, -1, 0, 0, NULL, 0},

		{ov2680_mode_QUXGA_800_600, SUBSAMPLING, 800, 600, \
		ov2680_setting_30fps_QUXGA_800_600, \
		ARRAY_SIZE(ov2680_setting_30fps_QUXGA_800_600)},

		{ov2680_mode_720P_1280_720, -1, 0, 0, NULL, 0},

		{ov2680_mode_SXGA_1280_960, -1, 0, 0, NULL, 0},

		{ov2680_mode_HD1600_1600_900, -1, 0, 0, NULL, 0},

		{ov2680_mode_UXGA_1600_1200, SCALING, 1600, 1200,
		ov2680_setting_30fps_UXGA_1600_1200,
		ARRAY_SIZE(ov2680_setting_30fps_UXGA_1600_1200)},
		{ov2680_mode_UXGA_1600_1200, -1, 0, 0, NULL, 0},
	},
	/* 60 fps mode info */
	{
		{ov2680_mode_VGA_640_480, -1, 0, 0, NULL, 0},
		{ov2680_mode_QUXGA_800_600, -1, 0, 0, NULL, 0},
		{ov2680_mode_720P_1280_720, -1, 0, 0, NULL, 0},
		{ov2680_mode_SXGA_1280_960, -1, 0, 0, NULL, 0},
		{ov2680_mode_HD1600_1600_900, -1, 0, 0, NULL, 0},
		{ov2680_mode_UXGA_1600_1200, -1, 0, 0, NULL, 0},
		{ov2680_mode_UXGA_1600_1200, -1, 0, 0, NULL, 0},
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;

static int ov2680_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov2680_remove(struct i2c_client *client);

static s32 ov2680_read_reg(u16 reg, u8 *val);
static s32 ov2680_write_reg(u16 reg, u8 val);

static const struct i2c_device_id ov2680_id[] = {
	{"ov2680_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov2680_id);

static struct i2c_driver ov2680_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov2680_mipi",
		  },
	.probe  = ov2680_probe,
	.remove = ov2680_remove,
	.id_table = ov2680_id,
};

static const struct ov2680_datafmt ov2680_colour_fmts[] = {
	{MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_COLORSPACE_JPEG},
};

static struct ov2680 *to_ov2680(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov2680, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct ov2680_datafmt
			*ov2680_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov2680_colour_fmts); i++)
		if (ov2680_colour_fmts[i].code == code)
			return ov2680_colour_fmts + i;

	return NULL;
}

static inline void ov2680_power_down(int assert)
{
	if (pwn_gpio < 0)
		return;

	if (!assert) /* assert power down? */
		gpio_set_value_cansleep(pwn_gpio, 1); /* power up the camera */
	else
		gpio_set_value_cansleep(pwn_gpio, 0); /* power down the camera */

	msleep(2);
}

static void ov2680_reset(void)
{
#if 0
	if (rst_gpio < 0 || pwn_gpio < 0)
		return;

	/* camera reset */
	gpio_set_value(rst_gpio, 1);

	/* camera power dowmn */
	gpio_set_value(pwn_gpio, 1);
	msleep(5);

	gpio_set_value(pwn_gpio, 0);
	msleep(5);

	gpio_set_value(rst_gpio, 0);
	msleep(1);

	gpio_set_value(rst_gpio, 1);
	msleep(5);

	gpio_set_value(pwn_gpio, 1);
#else
	if (pwn_gpio < 0)
		return;
	/* camera power dowmn */
	gpio_set_value(pwn_gpio, 0);
	msleep(5);
	gpio_set_value(pwn_gpio, 1);
	msleep(5);

	gpio_set_value(pwn_gpio, 0);
#endif
}

static int ov2680_regulator_enable(struct device *dev)
{
	int ret = 0;
#if 0

	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator,
				      OV2680_VOLTAGE_DIGITAL_IO,
				      OV2680_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(io_regulator);
		if (ret) {
			pr_err("%s:io set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:io set voltage ok\n", __func__);
		}
	} else {
		pr_err("%s: cannot get io voltage error\n", __func__);
		io_regulator = NULL;
	}

	core_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(core_regulator)) {
		regulator_set_voltage(core_regulator,
				      OV2680_VOLTAGE_DIGITAL_CORE,
				      OV2680_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(core_regulator);
		if (ret) {
			pr_err("%s:core set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:core set voltage ok\n", __func__);
		}
	} else {
		core_regulator = NULL;
		pr_err("%s: cannot get core voltage error\n", __func__);
	}

	analog_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(analog_regulator)) {
		regulator_set_voltage(analog_regulator,
				      OV2680_VOLTAGE_ANALOG,
				      OV2680_VOLTAGE_ANALOG);
		ret = regulator_enable(analog_regulator);
		if (ret) {
			pr_err("%s:analog set voltage error\n",
				__func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:analog set voltage ok\n", __func__);
		}
	} else {
		analog_regulator = NULL;
		pr_err("%s: cannot get analog voltage error\n", __func__);
	}
#endif
	return ret;
}

static s32 ov2680_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(ov2680_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 ov2680_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ov2680_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov2680_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

static int prev_sysclk, prev_HTS;
static int AE_low, AE_high, AE_Target = 52;

static inline void ov2680_dump_reg(u16 reg, char *str)
{
	u8 temp = 0;

	ov2680_read_reg(reg, &temp);

}

static void OV2680_stream_on(void)
{
	printk("*** manot - stream on!\n");
	//ov2680_write_reg(0x4202, 0x00);
	ov2680_write_reg(0x0100, 0x01);
//	ov2680_write_reg(0x3083, 0x04);

	ov2680_dump_reg(0x3080,"PLL PREDIV");
	ov2680_dump_reg(0x3081,"PLL MULTIPLIER");
	ov2680_dump_reg(0x3082,"PLL MULTIPLIER");
	ov2680_dump_reg(0x3083,"PLL MIPI DIV");
	ov2680_dump_reg(0x3084,"PLL SYS CLK DIV");
	ov2680_dump_reg(0x3085,"PLL DAC DIV");
	ov2680_dump_reg(0x3086,"PLL SP DIV");
	ov2680_dump_reg(0x3087,"PLL LANE DIV");
	ov2680_dump_reg(0x3088,"PLL CTRL");


}

static void OV2680_stream_off(void)
{
	printk("*** manot - stream off!\n");
	//ov2680_write_reg(0x4202, 0x0f);
	//ov2680_write_reg(0x3008, 0x42);
	ov2680_write_reg(0x0100, 0x00);
}

static int OV2680_get_sysclk(void)
{
	 /* calculate sysclk */
	int xvclk = ov2680_data.mclk / 10000;
	int temp1, temp2;
	int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv;
	int Bit_div2x = 1, sclk_rdiv, sysclk;
	u8 temp;

	int sclk_rdiv_map[] = {1, 2, 4, 8};

	temp1 = ov2680_read_reg(0x3034, &temp);
	temp2 = temp1 & 0x0f;
	if (temp2 == 8 || temp2 == 10)
		Bit_div2x = temp2 / 2;

	temp1 = ov2680_read_reg(0x3035, &temp);
	SysDiv = temp1>>4;
	if (SysDiv == 0)
		SysDiv = 16;

	temp1 = ov2680_read_reg(0x3036, &temp);
	Multiplier = temp1;

	temp1 = ov2680_read_reg(0x3037, &temp);
	PreDiv = temp1 & 0x0f;
	Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

	temp1 = ov2680_read_reg(0x3108, &temp);
	temp2 = temp1 & 0x03;
	sclk_rdiv = sclk_rdiv_map[temp2];

	VCO = xvclk * Multiplier / PreDiv;

	sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	return sysclk;
}

#if 0
static void OV2680_set_night_mode(void)
{
	 /* read HTS from register settings */
	u8 mode;

	ov2680_read_reg(0x3a00, &mode);
	mode &= 0xfb;
	ov2680_write_reg(0x3a00, mode);
}
#endif

static int OV2680_get_HTS(void)
{
	 /* read HTS from register settings */
	int HTS;
	u8 temp;

	HTS = ov2680_read_reg(0x380c, &temp);
	HTS = (HTS<<8) + ov2680_read_reg(0x380d, &temp);

	return HTS;
}

static int OV2680_get_VTS(void)
{
	 /* read VTS from register settings */
	int VTS;
	u8 temp;

	/* total vertical size[15:8] high byte */
	VTS = ov2680_read_reg(0x380e, &temp);

	VTS = (VTS<<8) + ov2680_read_reg(0x380f, &temp);

	return VTS;
}

static int OV2680_set_VTS(int VTS)
{
	 /* write VTS to registers */
	 int temp;

	 temp = VTS & 0xff;
	 ov2680_write_reg(0x380f, temp);

	 temp = VTS>>8;
	 ov2680_write_reg(0x380e, temp);

	 return 0;
}

static int OV2680_get_shutter(void)
{
	 /* read shutter, in number of line period */
	int shutter;
	u8 temp;

	shutter = (ov2680_read_reg(0x03500, &temp) & 0x0f);
	shutter = (shutter<<8) + ov2680_read_reg(0x3501, &temp);
	shutter = (shutter<<4) + (ov2680_read_reg(0x3502, &temp)>>4);

	 return shutter;
}

static int OV2680_set_shutter(int shutter)
{
	 /* write shutter, in number of line period */
	 int temp;

	 shutter = shutter & 0xffff;

	 temp = shutter & 0x0f;
	 temp = temp<<4;
	 ov2680_write_reg(0x3502, temp);

	 temp = shutter & 0xfff;
	 temp = temp>>4;
	 ov2680_write_reg(0x3501, temp);

	 temp = shutter>>12;
	 ov2680_write_reg(0x3500, temp);

	 return 0;
}

static int OV2680_get_gain16(void)
{
	 /* read gain, 16 = 1x */
	int gain16;
	u8 temp;

	gain16 = ov2680_read_reg(0x350a, &temp) & 0x03;
	gain16 = (gain16<<8) + ov2680_read_reg(0x350b, &temp);

	return gain16;
}

static int OV2680_set_gain16(int gain16)
{
	/* write gain, 16 = 1x */
	u8 temp;
	gain16 = gain16 & 0x3ff;

	temp = gain16 & 0xff;
	ov2680_write_reg(0x350b, temp);

	temp = gain16>>8;
	ov2680_write_reg(0x350a, temp);

	return 0;
}

static int OV2680_get_light_freq(void)
{
	/* get banding filter value */
	int temp, temp1, light_freq = 0;
	u8 tmp;

	temp = ov2680_read_reg(0x3c01, &tmp);

	if (temp & 0x80) {
		/* manual */
		temp1 = ov2680_read_reg(0x3c00, &tmp);
		if (temp1 & 0x04) {
			/* 50Hz */
			light_freq = 50;
		} else {
			/* 60Hz */
			light_freq = 60;
		}
	} else {
		/* auto */
		temp1 = ov2680_read_reg(0x3c0c, &tmp);
		if (temp1 & 0x01) {
			/* 50Hz */
			light_freq = 50;
		} else {
			/* 60Hz */
		}
	}
	return light_freq;
}

static void OV2680_set_bandingfilter(void)
{
	int prev_VTS;
	int band_step60, max_band60, band_step50, max_band50;

	/* read preview PCLK */
	prev_sysclk = OV2680_get_sysclk();
	/* read preview HTS */
	prev_HTS = OV2680_get_HTS();

	/* read preview VTS */
	prev_VTS = OV2680_get_VTS();

	/* calculate banding filter */
	/* 60Hz */
	band_step60 = prev_sysclk * 100/prev_HTS * 100/120;
	ov2680_write_reg(0x3a0a, (band_step60 >> 8));
	ov2680_write_reg(0x3a0b, (band_step60 & 0xff));

	max_band60 = (int)((prev_VTS-4)/band_step60);
	ov2680_write_reg(0x3a0d, max_band60);

	/* 50Hz */
	band_step50 = prev_sysclk * 100/prev_HTS;
	ov2680_write_reg(0x3a08, (band_step50 >> 8));
	ov2680_write_reg(0x3a09, (band_step50 & 0xff));

	max_band50 = (int)((prev_VTS-4)/band_step50);
	ov2680_write_reg(0x3a0e, max_band50);
}

static int OV2680_set_AE_target(int target)
{
	/* stable in high */
	int fast_high, fast_low;
	AE_low = target * 23 / 25;	/* 0.92 */
	AE_high = target * 27 / 25;	/* 1.08 */

	fast_high = AE_high<<1;
	if (fast_high > 255)
		fast_high = 255;

	fast_low = AE_low >> 1;

	ov2680_write_reg(0x3a0f, AE_high);
	ov2680_write_reg(0x3a10, AE_low);
	ov2680_write_reg(0x3a1b, AE_high);
	ov2680_write_reg(0x3a1e, AE_low);
	ov2680_write_reg(0x3a11, fast_high);
	ov2680_write_reg(0x3a1f, fast_low);

	return 0;
}

static void OV2680_turn_on_AE_AG(int enable)
{
	u8 ae_ag_ctrl;

	ov2680_read_reg(0x3503, &ae_ag_ctrl);
	if (enable) {
		/* turn on auto AE/AG */
		ae_ag_ctrl = ae_ag_ctrl & ~(0x03);
	} else {
		/* turn off AE/AG */
		ae_ag_ctrl = ae_ag_ctrl | 0x03;
	}
	ov2680_write_reg(0x3503, ae_ag_ctrl);
}

/* manot - 0x3821 [0] is hbin enable bit */
static bool binning_on(void)
{
	u8 temp;
	ov2680_read_reg(0x3821, &temp);
/* manot	temp &= 0xfe; */
	temp &= 0x1;
	if (temp)
		return true;
	else
		return false;
}

static void ov2680_set_virtual_channel(int channel)
{
	u8 channel_id;

	ov2680_read_reg(0x4814, &channel_id);
	channel_id &= ~(3 << 6);
	ov2680_write_reg(0x4814, channel_id | (channel << 6));
}

/* download ov2680 settings to sensor through i2c */
static int ov2680_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov2680_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov2680_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}

/* sensor changes between scaling and subsampling
 * go through exposure calcualtion
 */
static int ov2680_change_mode_exposure_calc(enum ov2680_frame_rate frame_rate,
				enum ov2680_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	u8 average;
	int prev_shutter, prev_gain16;
	int cap_shutter, cap_gain16;
	int cap_sysclk, cap_HTS, cap_VTS;
	int light_freq, cap_bandfilt, cap_maxband;
	long cap_gain16_shutter;
	int retval = 0;

	/* check if the input mode and frame rate is valid */
	pModeSetting =
		ov2680_mode_info_data[frame_rate][mode].init_data_ptr;
	ArySize =
		ov2680_mode_info_data[frame_rate][mode].init_data_size;

	ov2680_data.pix.width =
		ov2680_mode_info_data[frame_rate][mode].width;
	ov2680_data.pix.height =
		ov2680_mode_info_data[frame_rate][mode].height;

	if (ov2680_data.pix.width == 0 || ov2680_data.pix.height == 0 ||
		pModeSetting == NULL || ArySize == 0)
		return -EINVAL;

	/* auto focus */
	/* OV2680_auto_focus();//if no af function, just skip it */

	/* turn off AE/AG */
	OV2680_turn_on_AE_AG(0);

	/* read preview shutter */
	prev_shutter = OV2680_get_shutter();
/*	if ((binning_on()) && (mode != ov2680_mode_720P_1280_720)
			&& (mode != ov2680_mode_1080P_1920_1080))
		prev_shutter *= 2;
Manot - to be fixed */
	/* read preview gain */
	prev_gain16 = OV2680_get_gain16();

	/* get average */
	ov2680_read_reg(0x56a1, &average);

#if 0
	/* turn off night mode for capture */
	OV2680_set_night_mode();
#endif

#if 0
	/* turn off overlay */
	/* ov2680_write_reg(0x3022, 0x06);//if no af function, just skip it */
#endif
	OV2680_stream_off();

	/* Write capture setting */
	retval = ov2680_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	/* read capture VTS */
	cap_VTS = OV2680_get_VTS();
	cap_HTS = OV2680_get_HTS();
	cap_sysclk = OV2680_get_sysclk();

	/* calculate capture banding filter */
	light_freq = OV2680_get_light_freq();
	if (light_freq == 60) {
		/* 60Hz */
		cap_bandfilt = cap_sysclk * 100 / cap_HTS * 100 / 120;
	} else {
		/* 50Hz */
		cap_bandfilt = cap_sysclk * 100 / cap_HTS;
	}
	cap_maxband = (int)((cap_VTS - 4)/cap_bandfilt);

	/* calculate capture shutter/gain16 */
	if (average > AE_low && average < AE_high) {
		/* in stable range */
		cap_gain16_shutter =
		  prev_gain16 * prev_shutter * cap_sysclk/prev_sysclk
		  * prev_HTS/cap_HTS * AE_Target / average;
	} else {
		cap_gain16_shutter =
		  prev_gain16 * prev_shutter * cap_sysclk/prev_sysclk
		  * prev_HTS/cap_HTS;
	}

	/* gain to shutter */
	if (cap_gain16_shutter < (cap_bandfilt * 16)) {
		/* shutter < 1/100 */
		cap_shutter = cap_gain16_shutter/16;
		if (cap_shutter < 1)
			cap_shutter = 1;

		cap_gain16 = cap_gain16_shutter/cap_shutter;
		if (cap_gain16 < 16)
			cap_gain16 = 16;
	} else {
		if (cap_gain16_shutter >
				(cap_bandfilt * cap_maxband * 16)) {
			/* exposure reach max */
			cap_shutter = cap_bandfilt * cap_maxband;
			cap_gain16 = cap_gain16_shutter / cap_shutter;
		} else {
			/* 1/100 < (cap_shutter = n/100) =< max */
			cap_shutter =
			  ((int) (cap_gain16_shutter/16 / cap_bandfilt))
			  *cap_bandfilt;
			cap_gain16 = cap_gain16_shutter / cap_shutter;
		}
	}

	/* write capture gain */
	OV2680_set_gain16(cap_gain16);

	/* write capture shutter */
	if (cap_shutter > (cap_VTS - 4)) {
		cap_VTS = cap_shutter + 4;
		OV2680_set_VTS(cap_VTS);
	}
	OV2680_set_shutter(cap_shutter);

err:
	return retval;
}

/* if sensor changes inside scaling or subsampling
 * change mode directly
 * */
static int ov2680_change_mode_direct(enum ov2680_frame_rate frame_rate,
				enum ov2680_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;

	/* check if the input mode and frame rate is valid */
	pModeSetting =
		ov2680_mode_info_data[frame_rate][mode].init_data_ptr;
	ArySize =
		ov2680_mode_info_data[frame_rate][mode].init_data_size;

	ov2680_data.pix.width =
		ov2680_mode_info_data[frame_rate][mode].width;
	ov2680_data.pix.height =
		ov2680_mode_info_data[frame_rate][mode].height;

	if (ov2680_data.pix.width == 0 || ov2680_data.pix.height == 0 ||
		pModeSetting == NULL || ArySize == 0)
		return -EINVAL;

	/* turn off AE/AG */
	OV2680_turn_on_AE_AG(0);

	OV2680_stream_off();

	/* Write capture setting */
	retval = ov2680_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	OV2680_turn_on_AE_AG(1);

err:
	return retval;
}

static int ov2680_init_mode(enum ov2680_frame_rate frame_rate,
			    enum ov2680_mode mode, enum ov2680_mode orig_mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;
	u32 msec_wait4stable = 0;
	enum ov2680_downsize_mode dn_mode, orig_dn_mode;

	if ((mode > ov2680_mode_MAX || mode < ov2680_mode_MIN)
		&& (mode != ov2680_mode_INIT)) {
		pr_err("Wrong ov2680 mode detected!\n");
		return -1;
	}

	printk("*** manot - init mode: %d, %d \n",mode,orig_mode);
	printk("*** manot - dn_mode: %d \n", \
	dn_mode = ov2680_mode_info_data[frame_rate][mode==ov2680_mode_INIT?1:mode].dn_mode);
	printk("*** manot - orig_dn_mode: %d \n", \
	orig_dn_mode = ov2680_mode_info_data[frame_rate][orig_mode==ov2680_mode_INIT?1:orig_mode].dn_mode);
	if (mode == ov2680_mode_INIT) {
		pModeSetting = ov2680_init_setting_30fps_QUXGA;
		ArySize = ARRAY_SIZE(ov2680_init_setting_30fps_QUXGA);

		ov2680_data.pix.width = 800;
		ov2680_data.pix.height = 600;
		retval = ov2680_download_firmware(pModeSetting, ArySize);
		if (retval < 0)
			goto err;

		pModeSetting = ov2680_setting_30fps_QUXGA_800_600;
		ArySize = ARRAY_SIZE(ov2680_setting_30fps_QUXGA_800_600);
		retval = ov2680_download_firmware(pModeSetting, ArySize);
		printk("*** manot - downloaded init mode! \n");
	} else if ((dn_mode == SUBSAMPLING && orig_dn_mode == SCALING) ||
			(dn_mode == SCALING && orig_dn_mode == SUBSAMPLING)) {
		/* change between subsampling and scaling
		 * go through exposure calucation */
		printk("*** manot - change exposure! \n");
		retval = ov2680_change_mode_exposure_calc(frame_rate, mode);
	} else {
		/* change inside subsampling or scaling
		 * download firmware directly */
		printk("*** manot - changed mode direct! \n");
		retval = ov2680_change_mode_direct(frame_rate, mode);
	}

	if (retval < 0)
		goto err;

	printk("*** manot - setting AE target! \n");
	OV2680_set_AE_target(AE_Target);
	printk("*** manot - getting light frequency! \n");
	//OV2680_get_light_freq();
	printk("*** manot - setting band filter! \n");
	//OV2680_set_bandingfilter();
	printk("*** manot - setting virtual channel! \n");
	ov2680_set_virtual_channel(ov2680_data.csi);

	/* add delay to wait for sensor stable */
	if (mode == ov2680_mode_UXGA_1600_1200) {
		/* dump the first two frames: 1/7.5*2
		 * the frame rate of QSXGA is 7.5fps */
		msec_wait4stable = 267;
	} else {
		/* dump the first eighteen frames: 1/30*18 */
		msec_wait4stable = 600;
	}
	msleep(msec_wait4stable);

err:
	return retval;
}

/*!
 * ov2680_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ov2680_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2680 *sensor = to_ov2680(client);

	if (on && !sensor->on) {
		if (io_regulator)
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (core_regulator)
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (gpo_regulator)
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (analog_regulator)
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;
	} else if (!on && sensor->on) {
		if (analog_regulator)
			regulator_disable(analog_regulator);
		if (core_regulator)
			regulator_disable(core_regulator);
		if (io_regulator)
			regulator_disable(io_regulator);
		if (gpo_regulator)
			regulator_disable(gpo_regulator);
	}

	sensor->on = on;

	return 0;
}

/*!
 * ov2680_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ov2680_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2680 *sensor = to_ov2680(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ov5460_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ov2680_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2680 *sensor = to_ov2680(client);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov2680_frame_rate frame_rate;
	enum ov2680_mode orig_mode;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			frame_rate = ov2680_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ov2680_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		orig_mode = sensor->streamcap.capturemode;
		ret = ov2680_init_mode(frame_rate,
				(u32)a->parm.capture.capturemode, orig_mode);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ov2680_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	const struct ov2680_datafmt *fmt = ov2680_find_datafmt(mf->code);

	if (!fmt) {
		mf->code	= ov2680_colour_fmts[0].code;
		mf->colorspace	= ov2680_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov2680_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2680 *sensor = to_ov2680(client);

	/* MIPI CSI could have changed the format, double-check */
	if (!ov2680_find_datafmt(mf->code))
		return -EINVAL;

	ov2680_try_fmt(sd, mf);
	sensor->fmt = ov2680_find_datafmt(mf->code);

	return 0;
}

static int ov2680_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2680 *sensor = to_ov2680(client);

	const struct ov2680_datafmt *fmt = sensor->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov2680_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   u32 *code)
{
	if (index >= ARRAY_SIZE(ov2680_colour_fmts))
		return -EINVAL;

	*code = ov2680_colour_fmts[index].code;
	return 0;
}

/*!
 * ov2680_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ov2680_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > ov2680_mode_MAX)
		return -EINVAL;

	fse->max_width =
			max(ov2680_mode_info_data[0][fse->index].width,
			    ov2680_mode_info_data[1][fse->index].width);
	fse->min_width = fse->max_width;
	fse->max_height =
			max(ov2680_mode_info_data[0][fse->index].height,
			    ov2680_mode_info_data[1][fse->index].height);
	fse->min_height = fse->max_height;
	return 0;
}

/*!
 * ov2680_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ov2680_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int i, j, count = 0;

	if (fie->index < 0 || fie->index > ov2680_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warning("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(ov2680_mode_info_data); i++) {
		for (j = 0; j < (ov2680_mode_MAX + 1); j++) {
			if (fie->width == ov2680_mode_info_data[i][j].width
			 && fie->height == ov2680_mode_info_data[i][j].height
			 && ov2680_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fie->index == (count - 1)) {
				fie->interval.denominator =
						ov2680_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

/*!
 * dev_init - V4L2 sensor init
 * @s: pointer to standard V4L2 device structure
 *
 */
static int init_device(void)
{
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov2680_frame_rate frame_rate;
	int ret;

	ov2680_data.on = true;

	/* mclk */
	tgt_xclk = ov2680_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV2680_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV2680_XCLK_MIN);
	ov2680_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);

	printk("*** manot- tgt_xclk : %d \n",tgt_xclk / 1000000);
	if (0==ov2680_data.streamcap.timeperframe.numerator)
		printk("*** manot- time per frame numerator is zero! \n");
	/* Default camera frame rate is set in probe */
	tgt_fps = ov2680_data.streamcap.timeperframe.denominator /
		  ov2680_data.streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ov2680_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov2680_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	printk("*** manot- entering ov2680_init_mode \n");
	ret = ov2680_init_mode(frame_rate, ov2680_mode_INIT, ov2680_mode_INIT);

	return ret;
}

static int ov2680_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable)
		OV2680_stream_on();
	else
		OV2680_stream_off();
	return 0;
}

static struct v4l2_subdev_video_ops ov2680_subdev_video_ops = {
	.g_parm = ov2680_g_parm,
	.s_parm = ov2680_s_parm,
	.s_stream = ov2680_s_stream,

	.s_mbus_fmt	= ov2680_s_fmt,
	.g_mbus_fmt	= ov2680_g_fmt,
	.try_mbus_fmt	= ov2680_try_fmt,
	.enum_mbus_fmt	= ov2680_enum_fmt,
};

static const struct v4l2_subdev_pad_ops ov2680_subdev_pad_ops = {
	.enum_frame_size       = ov2680_enum_framesizes,
	.enum_frame_interval   = ov2680_enum_frameintervals,
};

static struct v4l2_subdev_core_ops ov2680_subdev_core_ops = {
	.s_power	= ov2680_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov2680_get_register,
	.s_register	= ov2680_set_register,
#endif
};

static struct v4l2_subdev_ops ov2680_subdev_ops = {
	.core	= &ov2680_subdev_core_ops,
	.video	= &ov2680_subdev_video_ops,
	.pad	= &ov2680_subdev_pad_ops,
};


/*!
 * ov2680 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov2680_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;

	/* ov2680 pinctrl */
	printk("*** manot - ov2680 probe ***\n");
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "no pin available\n");

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio))
		dev_warn(dev, "no sensor pwdn pin available");
	else {
		retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
						"ov2680_mipi_pwdn");
		if (retval < 0) {
			dev_warn(dev, "Failed to set power pin\n");
			dev_warn(dev, "retval=%d\n", retval);
			return retval;
		}
	}
#if 0
	/* request reset pin */
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(rst_gpio))
		dev_warn(dev, "no sensor reset pin available");
	else {
		retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
						"ov2680_mipi_reset");
		if (retval < 0) {
			dev_warn(dev, "Failed to set reset pin\n");
			return retval;
		}
	}
#endif

	/* Set initial values for the sensor struct. */
	memset(&ov2680_data, 0, sizeof(ov2680_data));
	ov2680_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ov2680_data.sensor_clk)) {
		/* assuming clock enabled by default */
		ov2680_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ov2680_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(ov2680_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ov2680_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ov2680_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	clk_prepare_enable(ov2680_data.sensor_clk);

	printk("*** manot - init the camera data structures \n");
	ov2680_data.io_init = ov2680_reset;
	ov2680_data.i2c_client = client;
	ov2680_data.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
	ov2680_data.pix.width = 800;
	ov2680_data.pix.height = 600;
	ov2680_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov2680_data.streamcap.capturemode = 1; /* 800 x 600 */
	ov2680_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov2680_data.streamcap.timeperframe.numerator = 1;

	ov2680_regulator_enable(&client->dev);

	ov2680_reset();

	printk("*** manot- setting power control pin \n");
	ov2680_power_down(0); /* power down = false */

	printk("*** manot - id the camera \n");
	retval = ov2680_read_reg(OV2680_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (retval < 0 || chip_id_high != 0x26) {
		pr_warning("camera ov2680_mipi is not found\n");
		clk_disable_unprepare(ov2680_data.sensor_clk);
		return -ENODEV;
	}
	retval = ov2680_read_reg(OV2680_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (retval < 0 || chip_id_low != 0x80) {
		pr_warning("camera ov2680_mipi is not found\n");
		clk_disable_unprepare(ov2680_data.sensor_clk);
		return -ENODEV;
	}

	printk("*** manot- run init_device() \n");
	retval = init_device();
	if (retval < 0) {
		clk_disable_unprepare(ov2680_data.sensor_clk);
		pr_warning("camera ov2680 init failed\n");
		ov2680_power_down(1);
		return retval;
	}

	printk("*** manot - run v4l2 ic subdev init \n");
	v4l2_i2c_subdev_init(&ov2680_data.subdev, client, &ov2680_subdev_ops);

	ov2680_data.subdev.grp_id = 678;
	retval = v4l2_async_register_subdev(&ov2680_data.subdev);
	if (retval < 0)
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);

	OV2680_stream_off();
	pr_info("camera ov2680_mipi is found\n");


	ov2680_dump_reg(0x3080,"PLL PREDIV");
	ov2680_dump_reg(0x3081,"PLL MULTIPLIER");
	ov2680_dump_reg(0x3082,"PLL MULTIPLIER");
	ov2680_dump_reg(0x3083,"PLL MIPI DIV");
	ov2680_dump_reg(0x3084,"PLL SYS CLK DIV");
	ov2680_dump_reg(0x3085,"PLL DAC DIV");
	ov2680_dump_reg(0x3086,"PLL SP DIV");
	ov2680_dump_reg(0x3087,"PLL LANE DIV");
	ov2680_dump_reg(0x3088,"PLL CTRL");


	return retval;
}

/*!
 * ov2680 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov2680_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);

	clk_disable_unprepare(ov2680_data.sensor_clk);

	ov2680_power_down(1); /* power down the camera */

	if (gpo_regulator)
		regulator_disable(gpo_regulator);

	if (analog_regulator)
		regulator_disable(analog_regulator);

	if (core_regulator)
		regulator_disable(core_regulator);

	if (io_regulator)
		regulator_disable(io_regulator);

	return 0;
}

module_i2c_driver(ov2680_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV2680 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
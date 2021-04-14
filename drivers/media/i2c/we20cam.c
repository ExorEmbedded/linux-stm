/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/i2c/we20cam.h>

enum we20cam_mode_id {
	WE20CAM_MODE_800_480,
	WE20CAM_MODE_640_480,
	WE20CAM_MODE_480_480,
	WE20CAM_MODE_320_480,
	WE20CAM_NUM_MODES,
};

enum we20cam_frame_rate {
	WE20CAM_FIRST_FPS = 0,
	WE20CAM_5_FPS = 0,
	WE20CAM_10_FPS,
	WE20CAM_15_FPS,
	WE20CAM_20_FPS,
	WE20CAM_25_FPS,
	WE20CAM_30_FPS,
	WE20CAM_NUM_FRAMERATES,
	WE20CAM_LAST_FPS = WE20CAM_NUM_FRAMERATES-1,
};

struct we20cam_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct we20cam_pixfmt we20cam_formats[] = {
	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_SRGB, },
	{ MEDIA_BUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB, },
};

static const int we20cam_framerates[] = {
	[WE20CAM_5_FPS] = 5,
	[WE20CAM_10_FPS] = 10,
	[WE20CAM_15_FPS] = 15,
	[WE20CAM_20_FPS] = 20,
	[WE20CAM_25_FPS] = 25,
	[WE20CAM_30_FPS] = 30,
};

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct we20cam_mode_info {
	enum we20cam_mode_id id;
	u32 hact;
	u32 vact;
};

struct we20cam_dev {
	struct i2c_client *i2c_client;
	
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct v4l2_ctrl_handler ctrl_hdl;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;

	struct v4l2_mbus_framefmt fmt;
	bool pending_fmt_change;

	const struct we20cam_mode_info *current_mode;
	const struct we20cam_mode_info *last_mode;
	enum we20cam_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	u32 prev_sysclk, prev_hts;

	bool pending_mode_change;
	bool streaming;
	bool controls_initialized;
	struct device* dev_folder;

	u32 i_fpga_control_register;
	u32 i_fpga_width;
	u32 i_fpga_height;
	u32 i_fpga_rotation;
	u32 i_fpga_mirror_horizontal;
	u32 i_fpga_mirror_vertical;
};

#define WE20CAM_VIDEO_SELECT_REG	0x00
#define WE20CAM_GPIO_IN_REG	0x00
#define WE20CAM_GPIO_OUT_REG	0x01
#define FPGA_ROTATION_CONTROL_REG	0x80
#define FPGA_SCALER_CONTROL_REG	0x200
#define FPGA_SCALER_WIDTH_REG	0x20C
#define FPGA_SCALER_HEIGHT_REG	0x210
#define FPGA_CLOCKED_VIDEO_OUTPUT_CONTROL_REG	0x400
#define FPGA_CLIPPER_CONTROL_REG	0x8000
#define FPGA_CLIPPER_WIDTH_REG	0x8004
#define FPGA_CLIPPER_HEIGHT_REG	0x8008
#define FPGA_CLIPPER_BLANKING_WIDTH_REG	0x800C
#define FPGA_CLIPPER_BLANKING_HEIGHT_REG	0x8010

#define DEFAULT_WIDTH	800
#define DEFAULT_HEIGHT	480

#define FPGA_ROTATION_ENABLE	0x1
#define FPGA_ROTATION_ANGLE_MASK	( (1<<2) | (1<<1) )
#define FPGA_SCALER_DISABLE	0x0
#define FPGA_SCALER_ENABLE	0x1
#define FPGA_MIRROR_HORIZONTAL_ENABLE	(1 << 3)
#define FPGA_MIRROR_VERTICAL_ENABLE	(1 << 4)
#define FPGA_CLIPPER_DISABLE	0x0
#define FPGA_CLIPPER_ENABLE	0x1
#define FPGA_CLIPPER_PASS_THROUGH	0x2
#define FPGA_CVO_DISABLE	0x0
#define FPGA_CVO_ENABLE	0x1

#define to_we20cam_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct we20cam_dev,	\
					    ctrl_hdl)->sd)

static inline struct we20cam_dev *to_we20cam_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct we20cam_dev, sd);
}

static const struct we20cam_mode_info
we20cam_mode_data[WE20CAM_NUM_MODES] = {
	{WE20CAM_MODE_800_480, 800, 480},
	{WE20CAM_MODE_640_480, 640, 480},
	{WE20CAM_MODE_640_480, 480, 480},
	{WE20CAM_MODE_320_480, 320, 480}
};

static int we20cam_write_reg_device(
	struct we20cam_dev *sensor,
	u8 dev_addr,
	u8 reg,
	u8 val)
{
	if (sensor->controls_initialized)
	{
		struct i2c_client *client = sensor->i2c_client;
		struct i2c_msg msg;
		u8 buf[2];
		int ret;

		buf[0] = reg;
		buf[1] = val;

		msg.addr = dev_addr;
		msg.flags = client->flags;
		msg.buf = buf;
		msg.len = sizeof(buf);

		/* must use i2c_transfer() here,
		 * i2c_smbus_write_byte_data() doesn't work
		 * */
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
				__func__, reg, val);
			return ret;
		}

		return 0;
	}
	
	return -ENODEV;
}

static int we20cam_read_reg_device(
	struct we20cam_dev *sensor,
	u8 dev_addr,
	u8 reg,
	u8 *val)
{
	if (sensor->controls_initialized)
	{
		struct i2c_client *client = sensor->i2c_client;

		/* must use i2c_smbus_read_byte_data() here,
		 * i2c_transfer() doesn't work
		 * */
		*val = i2c_smbus_read_byte_data(client, reg & 0xff);

		return 0;
	}
	
	return -ENODEV;
}

static int we20cam_write_reg32_device(
	struct we20cam_dev *sensor,
	u8 dev_addr,
	u16 reg,
	u32 val)
{
	if (sensor->controls_initialized)
	{
		struct i2c_client *client = sensor->i2c_client;
		struct i2c_msg msg;
		u8 regbuf[2];
		u8 buf[6];
		int ret;

		memcpy(&regbuf[0], &reg, 2);
		buf[0] = regbuf[1];
		buf[1] = regbuf[0];
	
		memcpy(&buf[2], &val, 4);

		msg.addr = dev_addr;
		msg.flags = client->flags;
		msg.buf = buf;
		msg.len = sizeof(buf);

		/* must use i2c_transfer() here,
		 * i2c_smbus_write_byte_data() doesn't work
		 * */
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
				__func__, reg, val);
			return ret;
		}

		return 0;
	}
	
	return -ENODEV;
}

static int we20cam_read_reg32_device(
	struct we20cam_dev *sensor,
	u8 dev_addr,
	u16 reg,
	u32 *val)
{
	if (sensor->controls_initialized)
	{
		struct i2c_client *client = sensor->i2c_client;
		struct i2c_msg msg[2];
		u8 bufreg[2];
		u8 buf[5];
		u8 buf2[5];
		int ret;

		memcpy(bufreg, &reg, 2);
		buf[0] = bufreg[1];
		buf[1] = bufreg[0];
		//buf[1] = val;

		msg[0].addr = dev_addr;
		msg[0].flags = client->flags;
		msg[0].buf = buf;
		msg[0].len = 2;
		msg[1].addr = dev_addr;
		msg[1].flags = client->flags | I2C_M_RD;
		msg[1].buf = buf2;
		msg[1].len = 4;

		/* must use i2c_transfer() here,
		 * i2c_smbus_write_byte_data() doesn't work
		 * */
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0) {
			dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
				__func__, reg, *val);
			return ret;
		}
		memcpy(val, &buf2[0], 4);

		return 0;
	}
	
	return -ENODEV;
}

static const struct we20cam_mode_info *
we20cam_find_mode(struct we20cam_dev *sensor, enum we20cam_frame_rate fr,
		 int width, int height, bool nearest)
{
	const struct we20cam_mode_info *mode;

	mode = v4l2_find_nearest_size(we20cam_mode_data,
				      ARRAY_SIZE(we20cam_mode_data),
				      hact, vact,
				      width, height);

	if (!mode ||
	    (!nearest && (mode->hact != width || mode->vact != height)))
		return NULL;

	return mode;
}

/* --------------- Subdev Operations --------------- */

static int we20cam_s_power(struct v4l2_subdev *sd, int on)
{
	struct we20cam_dev *sensor = to_we20cam_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);

	mutex_unlock(&sensor->lock);

	return ret;
}

static int we20cam_try_frame_interval(struct we20cam_dev *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
{
	const struct we20cam_mode_info *mode;
	u32 minfps, maxfps, fps;
	int ret;

	minfps = we20cam_framerates[WE20CAM_FIRST_FPS];
	maxfps = we20cam_framerates[WE20CAM_LAST_FPS];

	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		return WE20CAM_LAST_FPS;
	}

	fps = DIV_ROUND_CLOSEST(fi->denominator, fi->numerator);

	fi->numerator = 1;
	if (fps > maxfps)
	{
		fi->denominator = maxfps;
		ret = WE20CAM_LAST_FPS;
	}
	else if (fps < minfps)
	{
		fi->denominator = minfps;
		ret = WE20CAM_FIRST_FPS;
	}
	else
	{
		for (ret = 0; ret < WE20CAM_NUM_FRAMERATES; ++ret)
		{
			if (we20cam_framerates[ret] == fps)
			{
				break;
			}
		}
	}

	// not found?
	if (ret >= WE20CAM_NUM_FRAMERATES)
	{	// assume max is reasonable default
		ret = WE20CAM_LAST_FPS;
	}

	mode = we20cam_find_mode(sensor, ret, width, height, false);
	return mode ? ret : -EINVAL;
}

static int we20cam_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct we20cam_dev *sensor = to_we20cam_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						 format->pad);
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int we20cam_try_fmt_internal(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt,
				   enum we20cam_frame_rate fr,
				   const struct we20cam_mode_info **new_mode)
{
	struct we20cam_dev *sensor = to_we20cam_dev(sd);
	const struct we20cam_mode_info *mode;
	int i;

	mode = we20cam_find_mode(sensor, fr, fmt->width, fmt->height, true);
	if (!mode)
		return -EINVAL;
	fmt->width = mode->hact;
	fmt->height = mode->vact;

	if (new_mode)
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(we20cam_formats); i++)
		if (we20cam_formats[i].code == fmt->code)
			break;
	if (i >= ARRAY_SIZE(we20cam_formats))
		i = 0;

	fmt->code = we20cam_formats[i].code;
	fmt->colorspace = we20cam_formats[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}

static int we20cam_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct we20cam_dev *sensor = to_we20cam_dev(sd);
	const struct we20cam_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = we20cam_try_fmt_internal(sd, mbus_fmt,
				      sensor->current_fr, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
	else
		fmt = &sensor->fmt;

	*fmt = *mbus_fmt;

	if (new_mode != sensor->current_mode) {
		sensor->current_mode = new_mode;
		sensor->pending_mode_change = true;
	}
	if (mbus_fmt->code != sensor->fmt.code)
		sensor->pending_fmt_change = true;

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

/*
 * Controls
 * 
 * Handling them here allows us to do some video processing in FPGA,
 * as well as some in the ADV7280.
 * 
 * Currently following are done by FPGA:
 * - none
 * 
 * Following are done by 7280:
 * - brightness
 * - contrast
 * - hue
 * - saturation
 * - fast switching
 * - set input
 * - set pattern
 */

extern int adv7180_we20_command(int command, int param1, int param2);

static int we20cam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_we20cam_sd(ctrl);
	struct we20cam_dev *sensor = to_we20cam_dev(sd);
	if (sensor->controls_initialized)
	{
		return adv7180_we20_command(WE20_CMD_SET_CONTROL, ctrl->id, ctrl->val);
	}

	return 0;
}

static const struct v4l2_ctrl_ops we20cam_ctrl_ops = {
	.s_ctrl = we20cam_s_ctrl,
};

static const struct v4l2_ctrl_config we20cam_ctrl_fast_switch = {
	.ops = &we20cam_ctrl_ops,
	.id = V4L2_CID_ADV_FAST_SWITCH,
	.name = "fast_switching",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

static const struct v4l2_ctrl_config we20cam_ctrl_set_input = {
	.ops = &we20cam_ctrl_ops,
	.id = V4L2_CID_ADV_SET_INPUT,
	.name = "set_input",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 15,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

static const struct v4l2_ctrl_config we20cam_ctrl_set_pattern = {
	.ops = &we20cam_ctrl_ops,
	.id = V4L2_CID_ADV_SET_PATTERN,
	.name = "set_pattern",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 15,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

/* Contrast */
#define ADV7180_CON_MIN		0
#define ADV7180_CON_DEF		128
#define ADV7180_CON_MAX		255
/* Brightness*/
#define ADV7180_BRI_MIN		-128
#define ADV7180_BRI_DEF		0
#define ADV7180_BRI_MAX		127
/* Hue */
#define ADV7180_HUE_MIN		-127
#define ADV7180_HUE_DEF		0
#define ADV7180_HUE_MAX		128
/* Saturation */
#define ADV7180_SAT_MIN		0
#define ADV7180_SAT_DEF		128
#define ADV7180_SAT_MAX		255

static int we20cam_init_controls(struct we20cam_dev *sensor)
{
	v4l2_ctrl_handler_init(&sensor->ctrl_hdl, 4);

	v4l2_ctrl_new_std(&sensor->ctrl_hdl, &we20cam_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, ADV7180_BRI_MIN,
			  ADV7180_BRI_MAX, 1, ADV7180_BRI_DEF);
	v4l2_ctrl_new_std(&sensor->ctrl_hdl, &we20cam_ctrl_ops,
			  V4L2_CID_CONTRAST, ADV7180_CON_MIN,
			  ADV7180_CON_MAX, 1, ADV7180_CON_DEF);
	v4l2_ctrl_new_std(&sensor->ctrl_hdl, &we20cam_ctrl_ops,
			  V4L2_CID_SATURATION, ADV7180_SAT_MIN,
			  ADV7180_SAT_MAX, 1, ADV7180_SAT_DEF);
	v4l2_ctrl_new_std(&sensor->ctrl_hdl, &we20cam_ctrl_ops,
			  V4L2_CID_HUE, ADV7180_HUE_MIN,
			  ADV7180_HUE_MAX, 1, ADV7180_HUE_DEF);
	v4l2_ctrl_new_custom(&sensor->ctrl_hdl, &we20cam_ctrl_fast_switch, NULL);
	v4l2_ctrl_new_custom(&sensor->ctrl_hdl, &we20cam_ctrl_set_input, NULL);
	v4l2_ctrl_new_custom(&sensor->ctrl_hdl, &we20cam_ctrl_set_pattern, NULL);

	sensor->sd.ctrl_handler = &sensor->ctrl_hdl;
	if (sensor->ctrl_hdl.error) {
		int err = sensor->ctrl_hdl.error;

		v4l2_ctrl_handler_free(&sensor->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&sensor->ctrl_hdl);

	return 0;
}

static void we20cam_exit_controls(struct we20cam_dev *sensor)
{
	v4l2_ctrl_handler_free(&sensor->ctrl_hdl);
}

static int we20cam_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= WE20CAM_NUM_MODES)
		return -EINVAL;

	fse->min_width =
		we20cam_mode_data[fse->index].hact;
	fse->max_width = fse->min_width;
	fse->min_height =
		we20cam_mode_data[fse->index].vact;
	fse->max_height = fse->min_height;

	return 0;
}

static int we20cam_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct we20cam_dev *sensor = to_we20cam_dev(sd);
	struct v4l2_fract tpf;
	int ret;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= WE20CAM_NUM_FRAMERATES)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = we20cam_framerates[fie->index];

	ret = we20cam_try_frame_interval(sensor, &tpf,
					fie->width, fie->height);
	if (ret < 0)
		return -EINVAL;

	fie->interval = tpf;
	return 0;
}

static int we20cam_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct we20cam_dev *sensor = to_we20cam_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int we20cam_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct we20cam_dev *sensor = to_we20cam_dev(sd);
	const struct we20cam_mode_info *mode;
	int frame_rate, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;

	frame_rate = we20cam_try_frame_interval(sensor, &fi->interval,
					       mode->hact, mode->vact);
	if (frame_rate < 0)
		frame_rate = WE20CAM_15_FPS;

	mode = we20cam_find_mode(sensor, frame_rate, mode->hact,
				mode->vact, true);
	if (!mode) {
		ret = -EINVAL;
		goto out;
	}

	if (mode != sensor->current_mode ||
	   (frame_rate != sensor->current_fr)) {
		sensor->current_fr = frame_rate;
		sensor->frame_interval = fi->interval;
		sensor->current_mode = mode;
		sensor->pending_mode_change = true;
	}
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int we20cam_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(we20cam_formats))
		return -EINVAL;

	code->code = we20cam_formats[code->index].code;
	return 0;
}

static int we20cam_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;

	return ret;
}

static const struct v4l2_subdev_core_ops we20cam_core_ops = {
	.s_power = we20cam_s_power,
};

static const struct v4l2_subdev_video_ops we20cam_video_ops = {
	.g_frame_interval = we20cam_g_frame_interval,
	.s_frame_interval = we20cam_s_frame_interval,
	.s_stream = we20cam_s_stream,
};

static const struct v4l2_subdev_pad_ops we20cam_pad_ops = {
	.enum_mbus_code = we20cam_enum_mbus_code,
	.get_fmt = we20cam_get_fmt,
	.set_fmt = we20cam_set_fmt,
	.enum_frame_size = we20cam_enum_frame_size,
	.enum_frame_interval = we20cam_enum_frame_interval,
};

static const struct v4l2_subdev_ops we20cam_subdev_ops = {
	.core = &we20cam_core_ops,
	.video = &we20cam_video_ops,
	.pad = &we20cam_pad_ops,
};

/* sys fs stuff *******************************************************/

/* We provide 2 'files' in /sys/devices/we20cam/ folder:
 * - gpio_in = 4 inputs
 * - gpio_out = 4 outputs
 * 
 * gpio_in:
 * - bits 3..0
 * - read-only
 * 
 * gpio_out:
 * - bits 7..4 = mask, which lower/value bits to use (allows control
 * of single outputs, instead of always all at once)
 * - bits 3..0 = values
 * - read and write
 * E.g. 0xFF activates all 4 outputs, 0x11 activates only one.
 * 
 * Decimal format supported for reading;
 * decimal, hex and octal for writing.
 * 
 * Example:
 * cat /sys/devices/we20cam/gpio_in
 * ... will return:
 * 15 (assuming all inputs are active)
 * 
 * If permissions are not appropriate, change them with chmod; here
 * where the 'files' are created, Linux allows only a limited subset of
 * permissions to be assigned.
 * */

/* 4 inputs */

static ssize_t exor_camera_gpio_in_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	struct i2c_client *client = sensor->i2c_client;
	int len = 0;
	u8 value = 0;
	int ret = we20cam_read_reg_device(
		sensor,
		client->addr,
		WE20CAM_GPIO_IN_REG,
		&value);

	if (ret)
	{
		dev_err(dev, "Error %d reading fpga i2c\n", ret);
		return -EIO;
	}
	
	len = sprintf(buf, "%u\n", (unsigned int)value);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}
static DEVICE_ATTR(gpio_in, S_IRUGO, exor_camera_gpio_in_show, NULL);

/* 4 outputs **********************************************************/

static ssize_t exor_camera_gpio_out_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	struct i2c_client *client = sensor->i2c_client;
	int len = 0;
	u8 value = 0;
	int ret = we20cam_read_reg_device(
		sensor,
		client->addr,
		WE20CAM_GPIO_OUT_REG,
		&value);

	if (ret)
	{
		dev_err(dev, "Error %d reading fpga i2c\n", ret);
		return -EIO;
	}
	
	len = sprintf(buf, "%u\n", (unsigned int)value);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t exor_camera_gpio_out_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	struct i2c_client *client = sensor->i2c_client;
	unsigned int value = 0;
	int ret = 0;
	
	ret = kstrtouint(buf, 0, &value);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	
	ret = we20cam_write_reg_device(
		sensor,
		client->addr,
		WE20CAM_GPIO_OUT_REG,
		(u8)value);
	if (ret)
	{
		dev_err(dev, "Error %d writing fpga i2c\n", ret);
		return -EIO;
	}
	
	return count;
}

/* cannot assign arbitrary permissions here, if these are not
 * appropriate, 'chmod' them afterwards.
 * */
static DEVICE_ATTR(gpio_out, S_IRUGO | S_IWUSR, exor_camera_gpio_out_show, exor_camera_gpio_out_store);

/* register **********************************************************/

unsigned int gi_reg = 0;

static ssize_t exor_camera_reg_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int len = 0;
	
	len = sprintf(buf, "%u\n", (unsigned int)gi_reg);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t exor_camera_reg_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	int ret = 0;
	
	ret = kstrtouint(buf, 0, &gi_reg);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	
	return count;
}

static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR, exor_camera_reg_show, exor_camera_reg_store);

/* value **********************************************************/

unsigned int gi_val = 0;

static ssize_t exor_camera_val_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	struct i2c_client *client = sensor->i2c_client;
	int len = 0;
	u32 value = 0;
	int ret = we20cam_read_reg32_device(
		sensor,
		client->addr,
		gi_reg,
		&value);

	if (ret)
	{
		dev_err(dev, "Error %d reading fpga i2c\n", ret);
		return -EIO;
	}
	
	len = sprintf(buf, "0x%x\n", (unsigned int)value);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t exor_camera_val_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	struct i2c_client *client = sensor->i2c_client;
	unsigned int value = 0;
	int ret = 0;
	
	ret = kstrtouint(buf, 0, &value);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	
	ret = we20cam_write_reg32_device(
		sensor,
		client->addr,
		gi_reg,
		(u32)value);
	if (ret)
	{
		dev_err(dev, "Error %d writing fpga i2c\n", ret);
		return -EIO;
	}
	
	return count;
}

/* cannot assign arbitrary permissions here, if these are not
 * appropriate, 'chmod' them afterwards.
 * */
static DEVICE_ATTR(val, S_IRUGO | S_IWUSR, exor_camera_val_show, exor_camera_val_store);

static void fpga_enable_all(struct we20cam_dev *sensor, const bool b_enable)
{
	int ret = 0;

	if (b_enable)
	{	// deassert reset
		u32 i_value = 0;
		ret = we20cam_read_reg32_device(
			sensor,
			sensor->i2c_client->addr,
			WE20CAM_VIDEO_SELECT_REG,
			&i_value);
		i_value &= ~(1 << 6);
		ret = we20cam_write_reg32_device(
			sensor,
			sensor->i2c_client->addr,
			WE20CAM_VIDEO_SELECT_REG,
			i_value);
	}

	if (b_enable)
		sensor->i_fpga_control_register |= FPGA_ROTATION_ENABLE;
	else
		sensor->i_fpga_control_register &= ~FPGA_ROTATION_ENABLE;
	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_ROTATION_CONTROL_REG,
		sensor->i_fpga_control_register);

	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_SCALER_CONTROL_REG,
		b_enable ? FPGA_SCALER_ENABLE : FPGA_SCALER_DISABLE);
	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_CLIPPER_CONTROL_REG,
		b_enable ? FPGA_CLIPPER_ENABLE : FPGA_CLIPPER_DISABLE);

	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_CLOCKED_VIDEO_OUTPUT_CONTROL_REG,
		b_enable ? FPGA_CVO_ENABLE : FPGA_CVO_DISABLE);

	if (!b_enable)
	{	// assert reset
		u32 i_value = 0;
		ret = we20cam_read_reg32_device(
			sensor,
			sensor->i2c_client->addr,
			WE20CAM_VIDEO_SELECT_REG,
			&i_value);
		i_value |= (1 << 6);
		ret = we20cam_write_reg32_device(
			sensor,
			sensor->i2c_client->addr,
			WE20CAM_VIDEO_SELECT_REG,
			i_value);
	}
}

static ssize_t exor_camera_width_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int len = 0;

	len = sprintf(buf, "%d\n", sensor->i_fpga_width);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t exor_camera_width_store(
	struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t count)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int ret = 0;

	ret = kstrtouint(buf, 0, &sensor->i_fpga_width);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	fpga_enable_all(sensor, false);

	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_SCALER_WIDTH_REG,
		sensor->i_fpga_width);

	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_CLIPPER_WIDTH_REG,
		sensor->i_fpga_width);
	usleep_range(10 * 1000, 20 * 1000);
	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_CLIPPER_BLANKING_WIDTH_REG,
		DEFAULT_WIDTH - sensor->i_fpga_width);
	usleep_range(10 * 1000, 20 * 1000);

//	fpga_enable_all(state, true);

	return count;
}

static ssize_t exor_camera_height_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int len = 0;

	len = sprintf(buf, "%d\n", sensor->i_fpga_height);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t exor_camera_height_store(
	struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t count)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int ret = 0;

	ret = kstrtouint(buf, 0, &sensor->i_fpga_height);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}

//	fpga_enable_all(state, false);

	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_SCALER_HEIGHT_REG,
		sensor->i_fpga_height);

	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_CLIPPER_HEIGHT_REG,
		sensor->i_fpga_height);
	usleep_range(10 * 1000, 20 * 1000);
	ret = we20cam_write_reg32_device(
		sensor,
		sensor->i2c_client->addr,
		FPGA_CLIPPER_BLANKING_HEIGHT_REG,
		DEFAULT_HEIGHT - sensor->i_fpga_height);
	usleep_range(10 * 1000, 20 * 1000);

	fpga_enable_all(sensor, true);

	return count;
}

static ssize_t exor_camera_rotation_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int len = 0;

	len = sprintf(buf, "%d\n", sensor->i_fpga_rotation);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t exor_camera_rotation_store(
	struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t count)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int ret = 0;

	ret = kstrtouint(buf, 0, &sensor->i_fpga_rotation);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	fpga_enable_all(sensor, false);

	sensor->i_fpga_control_register &= ~FPGA_ROTATION_ANGLE_MASK;
	switch (sensor->i_fpga_rotation)
	{
	case 0: sensor->i_fpga_control_register |= (0x0 << 1); break;
	case 90: sensor->i_fpga_control_register |= (0x1 << 1); break;
	case 180: sensor->i_fpga_control_register |= (0x2 << 1); break;
	case 270: sensor->i_fpga_control_register |= (0x3 << 1); break;
	default: sensor->i_fpga_control_register |= (0x0 << 1); break;
	}
	
	fpga_enable_all(sensor, true);

	return count;
}

static ssize_t exor_camera_mirror_horizontal_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int len = 0;

	len = sprintf(buf, "%d\n", sensor->i_fpga_mirror_horizontal);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t exor_camera_mirror_horizontal_store(
	struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t count)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int ret = 0;

	ret = kstrtouint(buf, 0, &sensor->i_fpga_mirror_horizontal);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	fpga_enable_all(sensor, false);

	if (sensor->i_fpga_mirror_horizontal)
	{
		sensor->i_fpga_control_register |= FPGA_MIRROR_HORIZONTAL_ENABLE;
	}
	else
	{
		sensor->i_fpga_control_register &= ~FPGA_MIRROR_HORIZONTAL_ENABLE;
	}
	fpga_enable_all(sensor, true);

	return count;
}

static ssize_t exor_camera_mirror_vertical_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int len = 0;

	len = sprintf(buf, "%d\n", sensor->i_fpga_mirror_vertical);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t exor_camera_mirror_vertical_store(
	struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t count)
{
	struct we20cam_dev *sensor = dev_get_drvdata(dev);
	int ret = 0;

	ret = kstrtouint(buf, 0, &sensor->i_fpga_mirror_vertical);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	fpga_enable_all(sensor, false);

	if (sensor->i_fpga_mirror_vertical)
	{
		sensor->i_fpga_control_register |= FPGA_MIRROR_VERTICAL_ENABLE;
	}
	else
	{
		sensor->i_fpga_control_register &= ~FPGA_MIRROR_VERTICAL_ENABLE;
	}
	fpga_enable_all(sensor, true);

	return count;
}

static DEVICE_ATTR(width, S_IRUGO | S_IWUSR, exor_camera_width_show, exor_camera_width_store);
static DEVICE_ATTR(height, S_IRUGO | S_IWUSR, exor_camera_height_show, exor_camera_height_store);
static DEVICE_ATTR(rotation, S_IRUGO | S_IWUSR, exor_camera_rotation_show, exor_camera_rotation_store);
static DEVICE_ATTR(mirror_horizontal, S_IRUGO | S_IWUSR, exor_camera_mirror_horizontal_show, exor_camera_mirror_horizontal_store);
static DEVICE_ATTR(mirror_vertical, S_IRUGO | S_IWUSR, exor_camera_mirror_vertical_show, exor_camera_mirror_vertical_store);

static struct attribute *we20cam_attrs[] =
{
	& dev_attr_gpio_in.attr,
	& dev_attr_gpio_out.attr,
	& dev_attr_reg.attr,
	& dev_attr_val.attr,
	& dev_attr_width.attr,
	& dev_attr_height.attr,
	& dev_attr_rotation.attr,
	& dev_attr_mirror_horizontal.attr,
	& dev_attr_mirror_vertical.attr,
	NULL
};

static struct attribute_group we20cam_group =
{
    .name = NULL, // do not create new folder, we already have one
    .attrs = we20cam_attrs,
};

/* probe and remove ***************************************************/

static int we20cam_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct we20cam_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	/*
	 * default init sequence initialize sensor to
	 * YUV422 UYVY VGA@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_RGB565_2X8_LE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 640;
	fmt->height = 480;
	fmt->field = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = we20cam_framerates[WE20CAM_30_FPS];
	sensor->current_fr = WE20CAM_30_FPS;
	sensor->current_mode =
		&we20cam_mode_data[WE20CAM_MODE_640_480];
	sensor->last_mode = sensor->current_mode;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &we20cam_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = we20cam_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto free_ctrls;

	sensor->dev_folder = root_device_register("exor_camera");
	if (IS_ERR(sensor->dev_folder)) {
		dev_err(dev, "sysfs folder creation failed\n");
		goto free_ctrls;
	}
	dev_set_drvdata(sensor->dev_folder, sensor);
	
	ret = sysfs_create_group(&sensor->dev_folder->kobj, &we20cam_group);
	if (ret) {
		dev_err(dev, "sysfs attributes creation failed\n");
		goto free_ctrls;
	}
	
	sensor->controls_initialized = true;
		
	return 0;

free_ctrls:
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static int we20cam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct we20cam_dev *sensor = to_we20cam_dev(sd);
	struct device *dev = &client->dev;

	sysfs_remove_group(&dev->kobj, &we20cam_group);
	if (sensor->dev_folder)
		root_device_unregister(sensor->dev_folder);
	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	we20cam_exit_controls(sensor);

	return 0;
}

static const struct i2c_device_id we20cam_id[] = {
	{"we20cam", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, we20cam_id);

static const struct of_device_id we20cam_dt_ids[] = {
	{ .compatible = "exor,we20cam" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, we20cam_dt_ids);

static struct i2c_driver we20cam_i2c_driver = {
	.driver = {
		.name  = "we20cam",
		.of_match_table	= we20cam_dt_ids,
// this is supposedly better way, but it causes driver to crash
//		.groups = we20cam_groups,
	},
	.id_table = we20cam_id,
	.probe    = we20cam_probe,
	.remove   = we20cam_remove,
};

module_i2c_driver(we20cam_i2c_driver);

MODULE_DESCRIPTION("WE20CAM Camera Subdev Driver");
MODULE_LICENSE("GPL");

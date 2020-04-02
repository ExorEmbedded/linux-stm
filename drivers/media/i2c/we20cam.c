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
	WE20CAM_MODE_VGA_640_480,
	WE20CAM_NUM_MODES,
};

enum we20cam_frame_rate {
	WE20CAM_15_FPS = 0,
	WE20CAM_30_FPS,
	WE20CAM_NUM_FRAMERATES,
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
	[WE20CAM_15_FPS] = 15,
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
};

static inline struct we20cam_dev *to_we20cam_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct we20cam_dev, sd);
}

static const struct we20cam_mode_info
we20cam_mode_data[WE20CAM_NUM_MODES] = {
	{WE20CAM_MODE_VGA_640_480,
	 640, 480},
};

#if 0	// will be useful when we need to talk to FPGA

static int we20cam_write_reg_device(
	struct we20cam_dev *sensor,
	u8 dev_addr,
	u8 reg,
	u8 val)
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

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
			__func__, reg, val);
		return ret;
	}

	return 0;
}

static int we20cam_read_reg_device(
	struct we20cam_dev *sensor,
	u8 dev_addr,
	u8 reg,
	u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[1];
	u8 buf[1];
	int ret;

	buf[0] = reg;

	msg[0].addr = dev_addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x\n",
			__func__, reg);
		return ret;
	}

	*val = buf[0];
	return 0;
}

static int we20cam_mod_reg(struct we20cam_dev *sensor, u16 reg,
			  u8 mask, u8 val)
{
	u8 readval;
	int ret;

	ret = we20cam_read_reg(sensor, reg, &readval);
	if (ret)
		return ret;

	readval &= ~mask;
	val &= mask;
	val |= readval;

	return we20cam_write_reg(sensor, reg, val);
}
#endif

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

	minfps = we20cam_framerates[WE20CAM_15_FPS];
	maxfps = we20cam_framerates[WE20CAM_30_FPS];

	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		return WE20CAM_30_FPS;
	}

	fps = DIV_ROUND_CLOSEST(fi->denominator, fi->numerator);

	fi->numerator = 1;
	if (fps > maxfps)
		fi->denominator = maxfps;
	else if (fps < minfps)
		fi->denominator = minfps;
	else if (2 * fps >= 2 * minfps + (maxfps - minfps))
		fi->denominator = maxfps;
	else
		fi->denominator = minfps;

	ret = (fi->denominator == minfps) ? WE20CAM_15_FPS : WE20CAM_30_FPS;

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
	return adv7180_we20_command(WE20_CMD_SET_CONTROL, ctrl->id, ctrl->val);
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
		&we20cam_mode_data[WE20CAM_MODE_VGA_640_480];
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
	},
	.id_table = we20cam_id,
	.probe    = we20cam_probe,
	.remove   = we20cam_remove,
};

module_i2c_driver(we20cam_i2c_driver);

MODULE_DESCRIPTION("WE20CAM MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");

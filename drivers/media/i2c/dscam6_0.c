// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
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
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>



#define SENSOR_NAME "dscam"
//for internel driver debug
#define DEV_DBG_EN 1
#if (DEV_DBG_EN == 1)
#define csi_dev_dbg(x, arg...)                                                 \
	printk("[CAM_DEBUG][%s]"                                               \
	       "[%06d]" x,                                                     \
	       SENSOR_NAME, __LINE__, ##arg)
#else
#define csi_dev_dbg(x, arg...)
#endif
#define csi_dev_err(x, arg...)                                                 \
	printk(KERN_ERR "[CAM_ERR][$s]"                                        \
			"[%06d]" x,                                            \
	       SENSOR_NAME, __LINE__, ##arg)
#define csi_dev_print(x, arg...)                                               \
	printk(KERN_INFO "[CAM][%s][%d]" x, SENSOR_NAME, __LINE__, ##arg)

#define MAX_FPS 30
#define MIN_FPS 15

enum dscam_mode_id {
	DSCAM_MODE_FHD_1920_1080 = 0,
	DSCAM_MODE_UHD_3840_2160_30 = 1,
	DSCAM_MODE_UHD_3840_2160_15 = 2,
	DSCAM_MODE_UHD_3840_2168_30 = 3,
	DSCAM_MODE_UHD_3840_2168_15 = 4,
	DSCAM_NUM_MODES,
};

enum dscam_frame_rate {
	DSCAM_15_FPS 	= 0,
	DSCAM_30_FPS  	= 1,
	DSCAM_NUM_FRAMERATES,
};

enum dscam_format_mux {
	DSCAM_FMT_MUX_YUV422 = 0,
	DSCAM_FMT_MUX_RGB,
	DSCAM_FMT_MUX_DITHER,
	DSCAM_FMT_MUX_RAW_DPC,
	DSCAM_FMT_MUX_SNR_RAW,
	DSCAM_FMT_MUX_RAW_CIP,
};

union _HWORD {
	u8 c[2];
	u16 h;
};

union _WORD {
	u8 c[4];
	u16 hw[2];
	u32 w;
};


struct dscam_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct dscam_pixfmt dscam_formats[] = {
	{
		MEDIA_BUS_FMT_UYVY8_2X8,
		V4L2_COLORSPACE_SRGB,
	},
	{
		MEDIA_BUS_FMT_YUYV8_2X8,
		V4L2_COLORSPACE_SRGB,
	},
};

/*
 * FIXME: remove this when a subdev API becomes available
 * to set the MIPI CSI-2 virtual channel.
 */
static unsigned int virtual_channel;
module_param(virtual_channel, uint, 0444);
MODULE_PARM_DESC(virtual_channel,
		 "MIPI CSI-2 virtual channel (0..3), default 0");

static const int dscam_framerates[] = {
	[DSCAM_15_FPS] = 15,
	[DSCAM_30_FPS] = 30,
};

struct reg_value {
	u16 reg_addr;
	u32 val;
	u32 mask;
	u32 delay_ms;
};

struct dscam_mode_info {
	enum dscam_mode_id id;
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
	const struct reg_value *reg_data;
	u32 reg_data_size;
	u32 max_fps;
};

struct dscam_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *pixel_rate;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *light_freq;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

struct dscam_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	/* lock to protect all members below */
	struct mutex lock;

	int power_count;

	struct v4l2_mbus_framefmt fmt;
	struct v4l2_captureparm streamcap;
	struct v4l2_fract frame_interval;
	bool pending_fmt_change;

	const struct dscam_mode_info *current_mode;
	const struct dscam_mode_info *last_mode;
	enum dscam_frame_rate current_fr;

	struct dscam_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	bool pending_mode_change;
	bool streaming;
};

static inline struct dscam_dev *to_dscam_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct dscam_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct dscam_dev, ctrls.handler)->sd;
}

/* power-on sensor init reg table */
static const struct dscam_mode_info dscam_mode_init_data = {
	DSCAM_MODE_UHD_3840_2160_30, 3840, 4400, 2160, 2250, NULL, 0, DSCAM_30_FPS,
};

static const struct dscam_mode_info    dscam_mode_data[DSCAM_NUM_MODES] = {
	{ DSCAM_MODE_FHD_1920_1080, 1920, 1920, 1080, 1088, NULL, 0,  DSCAM_30_FPS },
	{ DSCAM_MODE_UHD_3840_2160_30, 3840, 4400, 2160, 2250, NULL, 0,  DSCAM_30_FPS },
	{ DSCAM_MODE_UHD_3840_2160_15, 3840, 4400, 2160, 2250, NULL, 0,  DSCAM_15_FPS },
	{ DSCAM_MODE_UHD_3840_2168_30, 3840, 4400, 2168, 2250, NULL, 0,  DSCAM_30_FPS },
	{ DSCAM_MODE_UHD_3840_2168_15, 3840, 4400, 2168, 2250, NULL, 0,  DSCAM_15_FPS }
};



static int dscam_check_valid_mode(struct dscam_dev *sensor,
				  const struct dscam_mode_info *mode,
				  enum dscam_frame_rate rate)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;
	csi_dev_dbg("enter %s\n",__func__);
	switch (mode->id) {
	
	case DSCAM_MODE_FHD_1920_1080:
		if (rate !=DSCAM_30_FPS)
			ret = -EINVAL;
		break;


	case DSCAM_MODE_UHD_3840_2160_15:
		if (rate !=DSCAM_15_FPS)
			ret = -EINVAL;
		break;
	case DSCAM_MODE_UHD_3840_2160_30:
		if (rate !=DSCAM_30_FPS)
			ret = -EINVAL;
		break;
	case DSCAM_MODE_UHD_3840_2168_30:
		if(rate != DSCAM_30_FPS)
			ret = -EINVAL;
		break;
	case DSCAM_MODE_UHD_3840_2168_15:
		if(rate != DSCAM_15_FPS)
			ret = -EINVAL;
		break;

	default:
		dev_err(&client->dev, "Invalid mode (%d)\n", mode->id);
		ret = -EINVAL;
	}
	csi_dev_dbg("exit %s with ret:%d\n",__func__,ret);
	return ret;
}


static int dscam_set_autoexposure(struct dscam_dev *sensor, bool on)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
	return dscam_mod_reg(sensor, DSCAM_REG_AEC_PK_MANUAL, BIT(0),
			     on ? 0 : BIT(0))
#endif
}

/* read exposure, in number of line periods */
static int dscam_get_exposure(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
	int exp, ret;
	u8 temp;

	ret = dscam_read_reg(sensor, DSCAM_REG_AEC_PK_EXPOSURE_HI, &temp);
	if (ret)
		return ret;
	exp = ((int)temp & 0x0f) << 16;
	ret = dscam_read_reg(sensor, DSCAM_REG_AEC_PK_EXPOSURE_MED, &temp);
	if (ret)
		return ret;
	exp |= ((int)temp << 8);
	ret = dscam_read_reg(sensor, DSCAM_REG_AEC_PK_EXPOSURE_LO, &temp);
	if (ret)
		return ret;
	exp |= (int)temp;

	return exp >> 4;
#endif
}

/* write exposure, given number of line periods */
static int dscam_set_exposure(struct dscam_dev *sensor, u32 exposure)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
	int ret;

	exposure <<= 4;

	ret = dscam_write_reg(sensor, DSCAM_REG_AEC_PK_EXPOSURE_LO,
			      exposure & 0xff);
	if (ret)
		return ret;
	ret = dscam_write_reg(sensor, DSCAM_REG_AEC_PK_EXPOSURE_MED,
			      (exposure >> 8) & 0xff);
	if (ret)
		return ret;
	return dscam_write_reg(sensor, DSCAM_REG_AEC_PK_EXPOSURE_HI,
			       (exposure >> 16) & 0x0f);
#endif
}

static int dscam_get_gain(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
	u16 gain;
	int ret;

	ret = dscam_read_reg16(sensor, DSCAM_REG_AEC_PK_REAL_GAIN, &gain);
	if (ret)
		return ret;

	return gain & 0x3ff;
#endif
}

static int dscam_set_gain(struct dscam_dev *sensor, int gain)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
	return dscam_write_reg16(sensor, DSCAM_REG_AEC_PK_REAL_GAIN,
				 (u16)gain & 0x3ff);
#endif
}

static int dscam_set_autogain(struct dscam_dev *sensor, bool on)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
	return dscam_mod_reg(sensor, DSCAM_REG_AEC_PK_MANUAL, BIT(1),
			     on ? 0 : BIT(1));
#endif
}

static int dscam_set_stream_dvp(struct dscam_dev *sensor, bool on)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return -EINVAL;
#else
#endif
}

static int dscam_set_stream_mipi(struct dscam_dev *sensor, bool on)
{
#if 1
	csi_dev_dbg("enter %s with %s \n", __func__,on ? "on" : "off");
	return 0;
#else
#endif
}

static int dscam_get_sysclk(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_night_mode(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_get_hts(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_get_vts(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_vts(struct dscam_dev *sensor, int vts)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_get_light_freq(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else	
#endif
}

static int dscam_set_bandingfilter(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else	
#endif
}

static int dscam_set_ae_target(struct dscam_dev *sensor, int target)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_get_binning(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_binning(struct dscam_dev *sensor, bool enable)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_virtual_channel(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static const struct dscam_mode_info *dscam_find_mode(struct dscam_dev *sensor,
						     enum dscam_frame_rate fr,
						     int width, int height,
						     bool nearest)
{
	const struct dscam_mode_info *mode;
	csi_dev_dbg("enter %s\n",__func__);

	csi_dev_dbg("[%s] width=%d,height=%d,rate=%d\n",__func__,width,height,fr);	
#if 1 
	mode = v4l2_find_nearest_size(dscam_mode_data,
				      ARRAY_SIZE(dscam_mode_data), hact, vact,
				      width, height);
#else
#endif 

	csi_dev_dbg("[%s] mode->id =%d,mode->hact =%d,mode->vact =%d \n",__func__,mode->id,mode->hact,mode->vact);
	if (!mode ||
	    (!nearest && (mode->hact != width || mode->vact != height))) {
		return NULL;
	}

	csi_dev_dbg("exit %s\n",__func__);
	return mode;
}

static u64 dscam_calc_pixel_rate(struct dscam_dev *sensor)
{
	u64 rate;
	csi_dev_dbg("enter %s\n",__func__);

	rate = sensor->current_mode->vtot * sensor->current_mode->htot;
	rate *= dscam_framerates[sensor->current_fr];

	csi_dev_dbg("exit %s\n",__func__);
	return rate;
}

/*
 * sensor changes between scaling and subsampling, go through
 * exposure calculation
 */
static int dscam_set_mode_exposure_calc(struct dscam_dev *sensor,
					const struct dscam_mode_info *mode)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

/*
 * if sensor changes inside scaling or subsampling
 * change mode directly
 */
static int dscam_set_mode_direct(struct dscam_dev *sensor,
				 const struct dscam_mode_info *mode)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_mode(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else	
#endif
}

static int dscam_set_framefmt(struct dscam_dev *sensor,
			      struct v4l2_mbus_framefmt *format);

/* restore the last set video mode after chip power-on */
static int dscam_restore_mode(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	sensor->last_mode = &dscam_mode_init_data;
	return dscam_set_framefmt(sensor, &sensor->fmt);
#else
#endif
}

static void dscam_power(struct dscam_dev *sensor, bool enable)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
#else
#endif
}

static void dscam_reset(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
#else
#endif
}

static int dscam_set_power_on(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	dscam_power(sensor, true);
	return 0;
#else
#endif
}

static void dscam_set_power_off(struct dscam_dev *sensor)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	dscam_power(sensor, false);
	sensor->streaming = false;
#else
#endif
}

static int dscam_set_power_mipi(struct dscam_dev *sensor, bool on)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_power_dvp(struct dscam_dev *sensor, bool on)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_power(struct dscam_dev *sensor, bool on)
{
#if 1
	int ret =0;
	csi_dev_dbg("enter %s\n",__func__);
	if (on) {
		ret = dscam_set_power_on(sensor);
		if (ret)
			return ret;

		ret = dscam_restore_mode(sensor);
		if (ret)
			return ret;
	}
	ret= dscam_set_power_mipi(sensor, on);
	if(ret)
	{
		dscam_set_power_off(sensor);
		return ret;
	}
	if( !on )
	{
		dscam_set_power_off(sensor);	
	}
	return 0;
#else	
#endif
}

/* --------------- Subdev Operations --------------- */

static int dscam_s_power(struct v4l2_subdev *sd, int on)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	int ret = 0;
	csi_dev_dbg("enter %s\n",__func__);
	mutex_lock(&sensor->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		ret = dscam_set_power(sensor, !!on);
		if (ret)
			goto out;
	}

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);

	if (on && !ret && sensor->power_count == 1) {
		/* restore controls */
		ret = v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	}

	return ret;
}

static int dscam_try_frame_interval(struct dscam_dev *sensor,
				    struct v4l2_fract *fi, u32 width,
				    u32 height)
{
	const struct dscam_mode_info *mode;
	enum dscam_frame_rate rate = DSCAM_30_FPS;
	int minfps, maxfps, best_fps, fps;
	int i;
	csi_dev_dbg("enter %s\n",__func__);

	minfps = dscam_framerates[DSCAM_30_FPS];
	maxfps = dscam_framerates[DSCAM_30_FPS];

	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		rate = DSCAM_30_FPS;
		goto find_mode;
	}
	csi_dev_dbg("[%s] -------------- \n",__func__);
	fps = clamp_val(DIV_ROUND_CLOSEST(fi->denominator, fi->numerator),
			minfps, maxfps);

	best_fps = minfps;
	csi_dev_dbg("[%s] fps=%d, best_fps=%d\n",__func__,fps,best_fps);
	for (i = 0; i < ARRAY_SIZE(dscam_framerates); i++) {
		int curr_fps = dscam_framerates[i];

		if (abs(curr_fps - fps) < abs(best_fps - fps)) {
			best_fps = curr_fps;
			rate = i;
		}
	}
	fi->numerator = 1;
	fi->denominator = best_fps;
	csi_dev_dbg("[%s] denominator = %u \n",__func__,fi->denominator);

find_mode:
	mode = dscam_find_mode(sensor, rate, width, height, false);
	csi_dev_dbg("exit %s with %p[rate:%d]\n",__func__,mode,rate);
	if( mode )
	{
		return rate;
	}
	else
	{
		return -EINVAL;
	}
	// return mode ? rate : -EINVAL;
}

static int dscam_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	struct v4l2_mbus_framefmt *fmt;
	csi_dev_dbg("enter %s\n",__func__);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		csi_dev_dbg("[%s] ----- \n",__func__);
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg, format->pad);
	}
	else{
		csi_dev_dbg("[%s]----- \n",__func__);
		csi_dev_dbg("[%s] sensor->fmt.code = 0x%x \n",__func__,sensor->fmt.code);
		csi_dev_dbg("[%s] sensor->fmt.width = %d\n",__func__,sensor->fmt.width);
		csi_dev_dbg("[%s] sensor->fmt.height = %d\n",__func__,sensor->fmt.height);
		fmt = &sensor->fmt;
	}


	fmt->reserved[1] = (sensor->current_fr == DSCAM_30_FPS) ? 30 : 15;
	csi_dev_dbg("[%s] fmt->reserved[1] = %d\n",__func__,fmt->reserved[1]);

	format->format = *fmt;
	csi_dev_dbg("[%s] format->pad = %d\n",__func__,format->pad);
	csi_dev_dbg("[%s] format->format.width = %d\n",__func__,format->format.width);
	csi_dev_dbg("[%s] format->format.height = %d\n",__func__,format->format.height);

	mutex_unlock(&sensor->lock);

	return 0;
}

static int dscam_try_fmt_internal(struct v4l2_subdev *sd,
				  struct v4l2_mbus_framefmt *fmt,
				  enum dscam_frame_rate fr,
				  const struct dscam_mode_info **new_mode)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	const struct dscam_mode_info *mode;
	int i;
	csi_dev_dbg("enter %s\n",__func__);

	mode = dscam_find_mode(sensor, fr, fmt->width, fmt->height, true);
	if (!mode)
		return -EINVAL;
	fmt->width = mode->hact;
	fmt->height = mode->vact;
	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	if (new_mode)
		*new_mode = mode;

	for (i = 0; i < ARRAY_SIZE(dscam_formats); i++)
		if (dscam_formats[i].code == fmt->code)
			break;
	if (i >= ARRAY_SIZE(dscam_formats))
		i = 0;

	fmt->code = dscam_formats[i].code;
	fmt->colorspace = dscam_formats[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	csi_dev_dbg("exit %s\n",__func__);
	return 0;
}

static int dscam_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	const struct dscam_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	csi_dev_dbg("enter %s\n",__func__);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = dscam_try_fmt_internal(sd, mbus_fmt, sensor->current_fr,
				     &new_mode);
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

	__v4l2_ctrl_s_ctrl_int64(sensor->ctrls.pixel_rate,
				 dscam_calc_pixel_rate(sensor));

	if (sensor->pending_mode_change || sensor->pending_fmt_change)
		sensor->fmt = *mbus_fmt;
out:
	mutex_unlock(&sensor->lock);
	csi_dev_dbg("exit %s with %d\n",__func__,ret);
	return ret;
}

static int dscam_set_framefmt(struct dscam_dev *sensor,
			      struct v4l2_mbus_framefmt *format)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	int ret = 0;
	bool is_jpeg = false;
	u8 fmt, mux;

	switch (format->code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		/* YUV422, UYVY */
		fmt = 0x3f;
		mux = DSCAM_FMT_MUX_YUV422;
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		/* YUV422, YUYV */
		fmt = 0x30;
		mux = DSCAM_FMT_MUX_YUV422;
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		/* RGB565 {g[2:0],b[4:0]},{r[4:0],g[5:3]} */
		fmt = 0x6F;
		mux = DSCAM_FMT_MUX_RGB;
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_BE:
		/* RGB565 {r[4:0],g[5:3]},{g[2:0],b[4:0]} */
		fmt = 0x61;
		mux = DSCAM_FMT_MUX_RGB;
		break;
	case MEDIA_BUS_FMT_JPEG_1X8:
		/* YUV422, YUYV */
		fmt = 0x30;
		mux = DSCAM_FMT_MUX_YUV422;
		is_jpeg = true;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		/* Raw, BGBG... / GRGR... */
		fmt = 0x00;
		mux = DSCAM_FMT_MUX_RAW_DPC;
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
		/* Raw bayer, GBGB... / RGRG... */
		fmt = 0x01;
		mux = DSCAM_FMT_MUX_RAW_DPC;
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		/* Raw bayer, GRGR... / BGBG... */
		fmt = 0x02;
		mux = DSCAM_FMT_MUX_RAW_DPC;
		break;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		/* Raw bayer, RGRG... / GBGB... */
		fmt = 0x03;
		mux = DSCAM_FMT_MUX_RAW_DPC;
		break;
	default:
		return -EINVAL;
	}

	return 0;
#else
	int ret = 0;
	bool is_jpeg = false;
	u8 fmt, mux;

	switch (format->code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		/* YUV422, UYVY */
		fmt = 0x3f;
		mux = DSCAM_FMT_MUX_YUV422;
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		/* YUV422, YUYV */
		fmt = 0x30;
		mux = DSCAM_FMT_MUX_YUV422;
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_LE:
		/* RGB565 {g[2:0],b[4:0]},{r[4:0],g[5:3]} */
		fmt = 0x6F;
		mux = DSCAM_FMT_MUX_RGB;
		break;
	case MEDIA_BUS_FMT_RGB565_2X8_BE:
		/* RGB565 {r[4:0],g[5:3]},{g[2:0],b[4:0]} */
		fmt = 0x61;
		mux = DSCAM_FMT_MUX_RGB;
		break;
	case MEDIA_BUS_FMT_JPEG_1X8:
		/* YUV422, YUYV */
		fmt = 0x30;
		mux = DSCAM_FMT_MUX_YUV422;
		is_jpeg = true;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		/* Raw, BGBG... / GRGR... */
		fmt = 0x00;
		mux = DSCAM_FMT_MUX_RAW_DPC;
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
		/* Raw bayer, GBGB... / RGRG... */
		fmt = 0x01;
		mux = DSCAM_FMT_MUX_RAW_DPC;
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		/* Raw bayer, GRGR... / BGBG... */
		fmt = 0x02;
		mux = DSCAM_FMT_MUX_RAW_DPC;
		break;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		/* Raw bayer, RGRG... / GBGB... */
		fmt = 0x03;
		mux = DSCAM_FMT_MUX_RAW_DPC;
		break;
	default:
		return -EINVAL;
	}

	/* FORMAT CONTROL00: YUV and RGB formatting */
	ret = dscam_write_reg(sensor, DSCAM_REG_FORMAT_CONTROL00, fmt);
	if (ret)
		return ret;

	/* FORMAT MUX CONTROL: ISP YUV or RGB */
	ret = dscam_write_reg(sensor, DSCAM_REG_ISP_FORMAT_MUX_CTRL, mux);
	if (ret)
		return ret;

	/*
	 * TIMING TC REG21:
	 * - [5]:	JPEG enable
	 */
	ret = dscam_mod_reg(sensor, DSCAM_REG_TIMING_TC_REG21, BIT(5),
			    is_jpeg ? BIT(5) : 0);
	if (ret)
		return ret;

	/*
	 * SYSTEM RESET02:
	 * - [4]:	Reset JFIFO
	 * - [3]:	Reset SFIFO
	 * - [2]:	Reset JPEG
	 */
	ret = dscam_mod_reg(sensor, DSCAM_REG_SYS_RESET02,
			    BIT(4) | BIT(3) | BIT(2),
			    is_jpeg ? 0 : (BIT(4) | BIT(3) | BIT(2)));
	if (ret)
		return ret;

	/*
	 * CLOCK ENABLE02:
	 * - [5]:	Enable JPEG 2x clock
	 * - [3]:	Enable JPEG clock
	 */
	return dscam_mod_reg(sensor, DSCAM_REG_SYS_CLOCK_ENABLE02,
			     BIT(5) | BIT(3), is_jpeg ? (BIT(5) | BIT(3)) : 0);
#endif
}

/*
 * Sensor Controls.
 */

static int dscam_set_ctrl_hue(struct dscam_dev *sensor, int value)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_ctrl_contrast(struct dscam_dev *sensor, int value)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_ctrl_saturation(struct dscam_dev *sensor, int value)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_ctrl_white_balance(struct dscam_dev *sensor, int awb)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_ctrl_exposure(struct dscam_dev *sensor,
				   enum v4l2_exposure_auto_type auto_exposure)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);

	return 0;
#else

#endif
}

static int dscam_set_ctrl_gain(struct dscam_dev *sensor, bool auto_gain)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
	struct dscam_ctrls *ctrls = &sensor->ctrls;
	int ret = 0;

	if (ctrls->auto_gain->is_new) {
		ret = dscam_set_autogain(sensor, auto_gain);
		if (ret)
			return ret;
	}

	if (!auto_gain && ctrls->gain->is_new)
		ret = dscam_set_gain(sensor, ctrls->gain->val);

	return ret;
#endif
}


static int dscam_set_ctrl_test_pattern(struct dscam_dev *sensor, int value)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_ctrl_light_freq(struct dscam_dev *sensor, int value)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_ctrl_hflip(struct dscam_dev *sensor, int value)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_set_ctrl_vflip(struct dscam_dev *sensor, int value)
{
#if 1
	csi_dev_dbg("enter %s\n",__func__);
	return 0;
#else
#endif
}

static int dscam_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct dscam_dev *sensor = to_dscam_dev(sd);
	int val;
	csi_dev_dbg("enter %s\n",__func__);
	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		val = dscam_get_gain(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.gain->val = val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		val = dscam_get_exposure(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.exposure->val = val;
		break;
	}

	return 0;
}

static int dscam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct dscam_dev *sensor = to_dscam_dev(sd);
	int ret;

	csi_dev_dbg("enter %s\n",__func__);
	/* v4l2_ctrl_lock() locks our own mutex */

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */
	if (sensor->power_count == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = dscam_set_ctrl_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = dscam_set_ctrl_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = dscam_set_ctrl_white_balance(sensor, ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = dscam_set_ctrl_hue(sensor, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = dscam_set_ctrl_contrast(sensor, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = dscam_set_ctrl_saturation(sensor, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = dscam_set_ctrl_test_pattern(sensor, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = dscam_set_ctrl_light_freq(sensor, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = dscam_set_ctrl_hflip(sensor, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = dscam_set_ctrl_vflip(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops dscam_ctrl_ops = {
	.g_volatile_ctrl = dscam_g_volatile_ctrl,
	.s_ctrl = dscam_s_ctrl,
};

static int dscam_init_controls(struct dscam_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &dscam_ctrl_ops;
	struct dscam_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	csi_dev_dbg("enter %s\n",__func__);
	v4l2_ctrl_handler_init(hdl, 32);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Clock related controls */
	ctrls->pixel_rate =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE, 0, INT_MAX, 1,
				  dscam_calc_pixel_rate(sensor));

	/* Auto/manual white balance */
	ctrls->auto_wb = v4l2_ctrl_new_std(
		hdl, ops, V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 4095, 1, 0);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 4095, 1, 0);
	/* Auto/manual exposure */
	ctrls->auto_exp =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_AUTO,
				       V4L2_EXPOSURE_MANUAL, 0,
				       V4L2_EXPOSURE_AUTO);
	ctrls->exposure =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0, 65535, 1, 0);
	/* Auto/manual gain */
	ctrls->auto_gain =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN, 0, 1023, 1, 0);

	ctrls->saturation =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION, 0, 255, 1, 64);
	ctrls->hue = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE, 0, 359, 1, 0);
	ctrls->contrast =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST, 0, 255, 1, 0);
	ctrls->test_pattern =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_pattern_menu) - 1,
					     0, 0, test_pattern_menu);
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	ctrls->light_freq =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_POWER_LINE_FREQUENCY,
				       V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
				       V4L2_CID_POWER_LINE_FREQUENCY_50HZ);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrls->gain->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_auto_cluster(3, &ctrls->auto_wb, 0, false);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, true);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_exp, 1, true);

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int dscam_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_frame_size_enum *fse)
{
	csi_dev_dbg("enter %s\n",__func__);
	csi_dev_dbg("[%s] fse->pad=0x%x, fse->index=0x%x\n",__func__,fse->pad,fse->index);
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= DSCAM_NUM_MODES)
		return -EINVAL;

	csi_dev_dbg("[%s]---------------------",__func__);
	fse->min_width = dscam_mode_data[fse->index].hact;
	fse->max_width = fse->min_width;
	fse->min_height = dscam_mode_data[fse->index].vact;
	fse->max_height = fse->min_height;

	csi_dev_dbg("exit %s",__func__);
	return 0;
}

static int
dscam_enum_frame_interval(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	int i, j, count;

	csi_dev_dbg("enter %s\n",__func__);

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= DSCAM_NUM_FRAMERATES)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 || fie->code == 0) {
		pr_warn("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < DSCAM_NUM_FRAMERATES; i++) {
		for (j = 0; j < DSCAM_NUM_MODES; j++) {
			if (fie->width == dscam_mode_data[j].hact &&
			    fie->height == dscam_mode_data[j].vact &&
			    !dscam_check_valid_mode(sensor, &dscam_mode_data[j],
						    i))
				count++;

			if (fie->index == (count - 1)) {
				fie->interval.denominator = dscam_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int dscam_g_frame_interval(struct v4l2_subdev *sd,
				  struct v4l2_subdev_frame_interval *fi)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	csi_dev_dbg("enter %s\n",__func__);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int dscam_s_frame_interval(struct v4l2_subdev *sd,
				  struct v4l2_subdev_frame_interval *fi)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	const struct dscam_mode_info *mode;
	int frame_rate, ret = 0;

	csi_dev_dbg("enter %s\n",__func__);

	if (fi->pad != 0)
		return -EINVAL;

	csi_dev_dbg("[%s] instance of sensor=%p\n",__func__,sensor);

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;
	csi_dev_dbg("[%s] mode->hact=%d\n",__func__,mode->hact);	
	csi_dev_dbg("[%s] mode->vact=%d\n",__func__,mode->vact);	
	csi_dev_dbg("[%s] mode->htot=%d\n",__func__,mode->htot);	
	csi_dev_dbg("[%s] mode->vtot=%d\n",__func__,mode->vtot);	
	csi_dev_dbg("[%s] sensor->current_mode=0x%x\n",__func__,sensor->current_mode);	
	csi_dev_dbg("[%s] sensor->current_fr=%d \n",__func__, sensor->current_fr );	

	sensor->frame_interval = fi->interval;

	#if 1 

	frame_rate = dscam_try_frame_interval(sensor, &fi->interval, mode->hact,
					      mode->vact);
	if (frame_rate < 0) {
		/* Always return a valid frame interval value */
		fi->interval = sensor->frame_interval;
		goto out;
	}

	mode = dscam_find_mode(sensor, frame_rate, mode->hact, mode->vact,
			       true);
	if (!mode) {
		ret = -EINVAL;
		goto out;
	}
	#endif 

	csi_dev_dbg("[%s] ----------\n",__func__);	
	if (mode != sensor->current_mode || frame_rate != sensor->current_fr) {
		csi_dev_dbg("[%s] ----------\n",__func__);	
		sensor->current_fr = frame_rate;
		sensor->frame_interval = fi->interval;
		sensor->current_mode = mode;
		sensor->pending_mode_change = true;

		__v4l2_ctrl_s_ctrl_int64(sensor->ctrls.pixel_rate,
					 dscam_calc_pixel_rate(sensor));
	}
out:
	mutex_unlock(&sensor->lock);
	csi_dev_dbg("[%s] sensor->current_fr=%d \n",__func__, sensor->current_fr );	
	csi_dev_dbg("[%s] sensor->frame->interval=%d \n",__func__, sensor->frame_interval );	
	csi_dev_dbg("[%s] sensor->frame->interval=%d \n",__func__, sensor->frame_interval );	
	csi_dev_dbg("[%s] fi->interval.denominator=%d \n",__func__, fi->interval.denominator );	
	csi_dev_dbg("[%s] fi->interval.numerator=%d \n",__func__, fi->interval.numerator );	
	csi_dev_dbg("[%s] fi->pad=%d \n",__func__, fi->pad );	
	csi_dev_dbg(" exit %s with %d \n",__func__, ret );	
	return ret;
}

static int dscam_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	csi_dev_dbg("enter %s\n",__func__);
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(dscam_formats))
		return -EINVAL;

	code->code = dscam_formats[code->index].code;
	return 0;
}

static int dscam_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;
	csi_dev_dbg("enter %s\n",__func__);
	mutex_lock(&sensor->lock);

	csi_dev_dbg("[%s] sensor->streaming =%d \n",__func__,sensor->streaming);
	if (sensor->streaming == !enable) {
		ret = dscam_check_valid_mode(sensor, sensor->current_mode,
					     sensor->current_fr);
		if (ret) {
			dev_err(&client->dev, "Not support WxH@fps=%dx%d@%d\n",
				sensor->current_mode->hact,
				sensor->current_mode->vact,
				dscam_framerates[sensor->current_fr]);
			goto out;
		}

		if (enable && sensor->pending_mode_change) {
			ret = dscam_set_mode(sensor);
			if (ret)
				goto out;
		}

		if (enable && sensor->pending_fmt_change) {
			ret = dscam_set_framefmt(sensor, &sensor->fmt);
			if (ret)
				goto out;
			sensor->pending_fmt_change = false;
		}

		if (sensor->ep.bus_type == V4L2_MBUS_CSI2_DPHY)
			ret = dscam_set_stream_mipi(sensor, enable);
		else
			ret = dscam_set_stream_dvp(sensor, enable);

		if (!ret)
			sensor->streaming = enable;
	}

out:
	mutex_unlock(&sensor->lock);
	csi_dev_dbg("exit %s with ret:%d\n",__func__,ret);
	return ret;
}

/*!
 * dscam_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int dscam_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	csi_dev_dbg("enter %s\n", __func__);

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
		dev_warn(dev, "Type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}
	csi_dev_dbg("exit %s",__func__);

	return ret;
}

static int dscam_init_mode(struct dscam_dev *sensor,
			   enum dscam_frame_rate frame_rate,
			   enum dscam_mode_id mode, enum dscam_mode_id orig_mode)
{

	csi_dev_dbg("enter %s\n", __func__);
	if( mode == DSCAM_MODE_UHD_3840_2160_30)
	{
		sensor->fmt.width = 3840;
		sensor->fmt.height = 2160;
	}
	return 0;
}

static int dscam_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct dscam_dev *sensor = to_dscam_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps; /* target frames per secound */
	enum dscam_frame_rate frame_rate;
	enum dscam_mode_id orig_mode_id;
	int ret = 0;
	csi_dev_dbg("enter %s\n", __func__);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = dscam_framerates[frame_rate];
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator / timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator / timeperframe->numerator;

		if (tgt_fps == 15)
		 	frame_rate = DSCAM_15_FPS;
		else if (tgt_fps == 30)
			frame_rate = DSCAM_30_FPS;
		else {
		 	dev_warn(dev,
		 		 "The camera frame rate is not supported!\n");
		 	return -EINVAL;
		}

		orig_mode_id = sensor->streamcap.capturemode;
		ret = dscam_init_mode(sensor, frame_rate,
				      (u32)a->parm.capture.capturemode,
				      orig_mode_id);
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
		dev_warn(dev,
			 "Type is not V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			 a->type);
		ret = -EINVAL;
		break;

	default:
		dev_warn(dev, "Type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}
	csi_dev_dbg("exit %s",__func__);

	return ret;
}


static const struct v4l2_subdev_core_ops dscam_core_ops = {
	.s_power = dscam_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops dscam_video_ops = {
	.g_frame_interval = dscam_g_frame_interval,
	.s_frame_interval = dscam_s_frame_interval,
	.g_parm = dscam_g_parm,
	.s_parm = dscam_s_parm,
	.s_stream = dscam_s_stream,
};

static const struct v4l2_subdev_pad_ops dscam_pad_ops = {
	.enum_mbus_code = dscam_enum_mbus_code,
	.get_fmt = dscam_get_fmt,
	.set_fmt = dscam_set_fmt,
	.enum_frame_size = dscam_enum_frame_size,
	.enum_frame_interval = dscam_enum_frame_interval,
};

static const struct v4l2_subdev_ops dscam_subdev_ops = {
	.core = &dscam_core_ops,
	.video = &dscam_video_ops,
	.pad = &dscam_pad_ops,
};

static int dscam_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations dscam_sd_media_ops = {
	.link_setup = dscam_link_setup,
};


static int dscam_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct dscam_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	u32 rotation;
	int ret;

	csi_dev_dbg("enter %s\n",__func__);
	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	csi_dev_dbg("[%s] instance of sensor=%p\n",__func__,sensor);

	sensor->i2c_client = client;

	/*
	 * default init sequence initialize sensor to
	 * YUV422 YUYV UHD@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 3840;
	fmt->height = 2160;
	fmt->field = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = dscam_framerates[DSCAM_30_FPS];
	sensor->current_fr = DSCAM_30_FPS;
	sensor->current_mode = &dscam_mode_data[DSCAM_MODE_UHD_3840_2160_30];
	sensor->last_mode = sensor->current_mode;

	sensor->ae_target = 52;

	endpoint =
		fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
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

	if (sensor->ep.bus_type != V4L2_MBUS_PARALLEL &&
	    sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY &&
	    sensor->ep.bus_type != V4L2_MBUS_BT656) {
		dev_err(dev, "Unsupported bus type %d\n", sensor->ep.bus_type);
		return -EINVAL;
	}



	v4l2_i2c_subdev_init(&sensor->sd, client, &dscam_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &dscam_sd_media_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;



	mutex_init(&sensor->lock);

	ret = dscam_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
	if (ret)
		goto free_ctrls;

	csi_dev_dbg("exit %s\n",__func__);
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int dscam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dscam_dev *sensor = to_dscam_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);

	return 0;
}

static const struct i2c_device_id dscam_id[] = {
	{ "dscam", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, dscam_id);

static const struct of_device_id dscam_dt_ids[] = { { .compatible =
							      "abyz,dscam" },
						    { /* sentinel */ } };
MODULE_DEVICE_TABLE(of, dscam_dt_ids);

static struct i2c_driver dscam_i2c_driver = {
	.driver = {
		.name  = "dscam",
		.of_match_table	= dscam_dt_ids,
	},
	.id_table = dscam_id,
	.probe_new = dscam_probe,
	.remove   = dscam_remove,
};

module_i2c_driver(dscam_i2c_driver);

MODULE_DESCRIPTION("H&Abyz X ray detector dscam MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>



#define DSCAM_I2C_ADDR 0x3C

#define CAM_DRVIE_NAME "dscam"

// for internel driver debug
#define DEV_DBG_EN 1

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

#if (DEV_DBG_EN == 1)
#define csi_dev_dbg(x, arg...)                                                                                                                                 \
    printk(KERN_DEBUG  "[CAM_DEBUG][%s]"                                                                                                                                   \
           "[%06d]" x,                                                                                                                                         \
           CAM_DRVIE_NAME, __LINE__, ##arg)
#else
#define csi_dev_dbg(x, arg...)
#endif
#define csi_dev_err(x, arg...)                                                                                                                                 \
    printk(KERN_ERR "[CAM_ERR][$s]"                                                                                                                            \
                    "[%06d]" x,                                                                                                                                \
           CAM_DRVIE_NAME, __LINE__, ##arg)
#define csi_dev_print(x, arg...) printk(KERN_INFO "[CAM][%s][%d]" x, CAM_DRVIE_NAME, __LINE__, ##arg)

enum dscam_format_mux{
    DSCAM_FMT_MUX_YUYV=0
};

enum dscam_resolution { 
    DSCAM_RES_1024x768 = 0, 
    DSCAM_RES_768x1024, 
    DSCAM_RES_MAX 
};

struct dscam_dev {
    struct i2c_client *i2c_client;
    struct v4l2_subdev sd;
    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_mbus_framefmt format;
    struct media_pad pad;
    struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
    struct mutex lock;
    enum dscam_resolution res;
};

static const struct v4l2_mbus_framefmt dscam_mipi_default_fmt = {
    .code = MEDIA_BUS_FMT_UYVY8_1X16,
    .width = 1024,
    .height = 768,
    .colorspace = V4L2_COLORSPACE_SRGB,
    .ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SRGB),
    .quantization = V4L2_QUANTIZATION_FULL_RANGE,
    .xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_SRGB),
    .field = V4L2_FIELD_NONE,
};


static inline struct dscam_dev *to_dscam_dev(struct v4l2_subdev *sd) { return container_of(sd, struct dscam_dev, sd); }


static void dscam_set_resolution(struct dscam_dev *camera, enum dscam_resolution res) {
    csi_dev_dbg("enter %s\n", __func__);
    switch (res) {
    case DSCAM_RES_1024x768:
        camera->format.width = 1024;
        camera->format.height = 768;
        break;
    case DSCAM_RES_768x1024:
        camera->format.width = 768;
        camera->format.height = 1024;
        break;
    default:
        camera->format.width = 1024;
        camera->format.height = 768;
        break;
    }
    camera->format.code = V4L2_PIX_FMT_YUYV; // YUYV format
}

static int dscam_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_mbus_code_enum *code) {
    csi_dev_dbg("enter %s\n", __func__);
    if (code->index >= 1)
        return -EINVAL;

    code->code = V4L2_PIX_FMT_YUYV;
    return 0;
}

static int dscam_i2c_read(struct i2c_client *client, u8 reg) {
    csi_dev_dbg("enter %s\n", __func__);
    pr_debug("dscam: Reading I2C register 0x%02x\n", reg);
    return 0; // Virtual I2C read operation
}

static int dscam_i2c_write(struct i2c_client *client, u8 reg, u8 value) {
    csi_dev_dbg("enter %s\n", __func__);
    pr_debug("dscam: Writing 0x%02x to I2C register 0x%02x\n", value, reg);
    return 0; // Virtual I2C write operation
}

static int dscam_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format) {
    struct dscam_dev *sensor = to_dscam_dev(sd);
    struct v4l2_mbus_framefmt *mf = &format->format;
    csi_dev_dbg("enter %s\n", __func__);

    mutex_lock(&sensor->lock);

    if (mf->width == 1024 && mf->height == 768) {
        sensor->res = DSCAM_RES_1024x768;
    } else if (mf->width == 768 && mf->height == 1024) {
        sensor->res = DSCAM_RES_768x1024;
    } else {
        sensor->res = DSCAM_RES_1024x768;
    }

    sensor->format.code = V4L2_PIX_FMT_YUYV;  // Ensure format is set correctly
    sensor->format.width = mf->width;
    sensor->format.height = mf->height;

    *mf = sensor->format;

    mutex_unlock(&sensor->lock);
    return 0;
}

static int dscam_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format) {
    struct dscam_dev *sensor = to_dscam_dev(sd);
    struct v4l2_mbus_framefmt *fmt;
    csi_dev_dbg("enter %s\n", __func__);

    if (format->pad != 0)
        return -EINVAL;
#if 0 
    mutex_lock(&sensor->lock);
    if (format->which == V4L2_SUBDEV_FORMAT_TRY)
        fmt = v4l2_subdev_get_try_format(&sensor->sd, state, format->pad);
    else
        fmt = &sensor->format;
    fmt->reserved[1] = 30;
    format->format = *fmt;

    mutex_unlock(&sensor->lock);
#else

    mutex_lock(&sensor->lock);

    // Set default format if not already set
    if (sensor->format.code == 0) {
        sensor->format.code = V4L2_PIX_FMT_YUYV;
        sensor->format.width = 1024;
        sensor->format.height = 768;
    }

    format->format = sensor->format;
    mutex_unlock(&sensor->lock);
#endif
    csi_dev_dbg("format->format code=0x%x\n", format->format.code);
    csi_dev_dbg("format->format hight=%d\n", format->format.height);
    csi_dev_dbg("format->format width=%d\n", format->format.width);
    csi_dev_dbg("exit %s\n", __func__);
    return 0;
}

static int dscam_s_power(struct v4l2_subdev *sd, int on) {
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    csi_dev_dbg("enter %s\n", __func__);

    if (on) {
        dscam_i2c_write(client, 0x00, 0x01); // Virtual power on
    } else {
        dscam_i2c_write(client, 0x00, 0x00); // Virtual power off
    }

    return 0;
}

static int dscam_s_stream(struct v4l2_subdev *sd, int enable) {
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    csi_dev_dbg("enter %s\n", __func__);

    if (enable) {
        ret = dscam_i2c_write(client, 0x01, 0x01); // Virtual stream start
        if (ret < 0)
            return ret;
    } else {
        ret = dscam_i2c_write(client, 0x01, 0x00); // Virtual stream stop
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int dscam_parse_fwnode(struct dscam_dev *sensor) {
    struct fwnode_handle *ep = NULL;
    struct v4l2_fwnode_endpoint endpoint = {.bus_type = V4L2_MBUS_UNKNOWN};
    struct v4l2_subdev *sd = &sensor->sd;

    csi_dev_dbg("enter %s\n", __func__);

    ep = fwnode_graph_get_next_endpoint(dev_fwnode(sd->dev), NULL);
    if (!ep)
        return -ENODEV;

    v4l2_fwnode_endpoint_parse(ep, &endpoint);
    fwnode_handle_put(ep);

    return 0;
}

static int dscam_g_volatile_ctrl(struct v4l2_ctrl *ctrl) {
    csi_dev_dbg("enter %s\n", __func__);
    return 0;
}

static int dscam_s_ctrl(struct v4l2_ctrl *ctrl) {
    csi_dev_dbg("enter %s\n", __func__);
    return 0;
}

static int dscam_init_cfg(struct v4l2_subdev *sd, struct v4l2_subdev_state *state) {
    struct dscam_dev *sensor = to_dscam_dev(sd);
    struct v4l2_mbus_framefmt *fmt = v4l2_subdev_get_try_format(sd, state, 0);
    
    csi_dev_dbg("enter %s\n", __func__);

    *fmt = dscam_mipi_default_fmt;

    csi_dev_dbg("fmt code=0x%x\n", fmt->code);
    csi_dev_dbg("fmt hight=%d\n", fmt->height);
    csi_dev_dbg("fmt width=%d\n", fmt->width);

    csi_dev_dbg("exit %s\n", __func__);
    
    return 0;
}

static const struct v4l2_subdev_core_ops dscam_core_ops = {
    .s_power = dscam_s_power,
};

static const struct v4l2_subdev_video_ops dscam_video_ops = {
    .s_stream = dscam_s_stream,
};

static const struct v4l2_subdev_pad_ops dscam_pad_ops = {
    .init_cfg = dscam_init_cfg,
    .enum_mbus_code = dscam_enum_mbus_code,
    .set_fmt = dscam_set_fmt,
    .get_fmt = dscam_get_fmt,
};

static const struct v4l2_subdev_ops dscam_ops = {
    .core = &dscam_core_ops,
    .video = &dscam_video_ops,
    .pad = &dscam_pad_ops,
};

static const struct v4l2_ctrl_ops dscam_ctrl_ops = {
    .g_volatile_ctrl = dscam_g_volatile_ctrl,
    .s_ctrl = dscam_s_ctrl,
};

static int dscam_link_setup(struct media_entity *entity, const struct media_pad *local, const struct media_pad *remote, u32 flags) { return 0; }

static const struct media_entity_operations dscam_media_ops = {
    .link_setup = dscam_link_setup,
};



static int dscam_init_controls(struct dscam_dev *sensor) {

    const struct v4l2_ctrl_ops *ops = &dscam_ctrl_ops;
    struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;
    struct v4l2_fwnode_device_properties props;
    int ret;

    csi_dev_dbg("enter %s\n", __func__);

    v4l2_ctrl_handler_init(hdl, 32);
    hdl->lock = &sensor->lock;

    if (hdl->error) {
        ret = hdl->error;
        goto free_ctrls;
    }

    ret = v4l2_fwnode_device_parse(&sensor->i2c_client->dev, &props);
    if (ret)
        goto free_ctrls;

    ret = v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);
    if (ret)
        goto free_ctrls;

    sensor->sd.ctrl_handler = hdl;
    csi_dev_dbg("exit %s\n", __func__);
    return 0;
free_ctrls:
    v4l2_ctrl_handler_free(hdl);
    return ret;
}

static int dscam_probe(struct i2c_client *client) {
    struct device *dev = &client->dev;
    struct fwnode_handle *endpoint;
    struct dscam_dev *sensor;
    int ret;

    csi_dev_dbg("enter %s\n", __func__);

    sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;

    sensor->i2c_client = client;

    sensor->res = DSCAM_RES_1024x768;
    dscam_set_resolution(sensor, sensor->res);


    endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
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
    v4l2_i2c_subdev_init(&sensor->sd, client, &dscam_ops);

    sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
    sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
    sensor->sd.entity.ops = &dscam_media_ops;
    sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
    if (ret)
        return ret;

    mutex_init(&sensor->lock);

    ret = dscam_init_controls(sensor);
    if (ret)
        goto entity_cleanup;

    ret = v4l2_async_register_subdev_sensor(&sensor->sd);
    if (ret)
        goto entity_cleanup;

    csi_dev_dbg("exit %s\n", __func__);
    return 0;
entity_cleanup:
    media_entity_cleanup(&sensor->sd.entity);
    mutex_destroy(&sensor->lock);
    return ret;
}

static void dscam_remove(struct i2c_client *client) {
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct dscam_dev *sensor = to_dscam_dev(sd);

    csi_dev_dbg("enter %s\n", __func__);

    v4l2_ctrl_handler_free(&sensor->ctrl_handler);
    media_entity_cleanup(&sd->entity);
    v4l2_device_unregister_subdev(sd);
    mutex_destroy(&sensor->lock);
}

static const struct i2c_device_id dscam_id[] = {{"dscam", 0}, {}};
MODULE_DEVICE_TABLE(i2c, dscam_id);

static const struct of_device_id dscam_dt_ids[] = {
    {.compatible = "abyz,dscam"},
    {/* sentinel */},
};
MODULE_DEVICE_TABLE(of, dscam_dt_ids);

static struct i2c_driver dscam_i2c_driver = {
    .driver =
        {
            .name = "dscam",
            .of_match_table = dscam_dt_ids,
        },
    .id_table = dscam_id,
    .probe_new = dscam_probe,
    .remove = dscam_remove,
};

module_i2c_driver(dscam_i2c_driver);

MODULE_DESCRIPTION("DSCAM Camera Driver with Virtual I2C, RAW16, and Streaming Support");
MODULE_AUTHOR("david.kang");
MODULE_LICENSE("GPL");

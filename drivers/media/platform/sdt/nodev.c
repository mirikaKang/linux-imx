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
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>


#define DRIVER_NAME	"nodev"

//for internel driver debug
#define DEV_DBG_EN 1
#if (DEV_DBG_EN == 1)
#define csi_dev_dbg(x, arg...)                                                 \
	printk("[CAM_DEBUG][%s]"                                               \
	       "[%06d]" x,                                                     \
	       DRIVER_NAME, __LINE__, ##arg)
#else
#define csi_dev_dbg(x, arg...)
#endif
#define csi_dev_err(x, arg...)                                                 \
	printk(KERN_ERR "[CAM_ERR][$s]"                                        \
			"[%06d]" x,                                            \
	       DRIVER_NAME, __LINE__, ##arg)
#define csi_dev_print(x, arg...)                                               \
	printk(KERN_INFO "[CAM][%s][%d]" x, DRIVER_NAME, __LINE__, ##arg)


/*
 * FIXME: remove this when a subdev API becomes available
 * to set the MIPI CSI-2 virtual channel.
 */
static unsigned int virtual_channel;
module_param(virtual_channel, uint, 0444);
MODULE_PARM_DESC(virtual_channel,
         "MIPI CSI-2 virtual channel (0..3), default 0");

struct node_v_dev {
    struct v4l2_device      v4l2_dev;
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
    int id; /* sensors can be multiple. */
    struct platform_device *pdev;

    struct clk *xclk; /* Hmmm, there is no system clock to this camera */
    u32 xclk_freq;

    /* lock to protect all members below ?  */
    struct mutex lock;


    bool streaming;
};

static inline struct node_v_dev *to_node_v_dev(struct v4l2_subdev *sd)
{
    return container_of(sd, struct node_v_dev, sd);
}

/*
static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
    return &container_of(ctrl->handler, struct node_v_dev,
                 ctrls.handler)->sd;
}*/

static inline bool node_v_is_csi2(const struct node_v_dev *sensor)
{
    return sensor->ep.bus_type == V4L2_MBUS_CSI2_DPHY;
}

/*
    functions for  subdev_core
*/
static int node_v_s_power(struct v4l2_subdev *sd, int on)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}

/* 
    functions for subdev video
*/
static int  node_v_g_frame_interval(struct v4l2_subdev *sd,
                   struct v4l2_subdev_frame_interval *fi)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}

static int node_v_s_frame_interval(struct v4l2_subdev *sd,
                   struct v4l2_subdev_frame_interval *fi)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}

static int node_v_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}

/* 
    functions for subdev pads
*/
static int node_v_init_cfg(struct v4l2_subdev *sd,
               struct v4l2_subdev_state *state)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;

}
static int node_v_enum_mbus_code(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_mbus_code_enum *code)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}
static int node_v_get_fmt(struct v4l2_subdev *sd,
              struct v4l2_subdev_state *sd_state,
              struct v4l2_subdev_format *format)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}

static int node_v_set_fmt(struct v4l2_subdev *sd,
              struct v4l2_subdev_state *sd_state,
              struct v4l2_subdev_format *format)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}

static int node_v_get_selection(struct v4l2_subdev *sd,
                struct v4l2_subdev_state *sd_state,
                struct v4l2_subdev_selection *sel)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}
static int node_v_enum_frame_size(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *sd_state,
                  struct v4l2_subdev_frame_size_enum *fse)
{
    struct node_v_dev *sensor = to_node_v_dev(sd);

    dev_info(&sensor->pdev->dev, "%s enter\n", __func__);
    return 0;
}

static int node_v_enum_frame_interval(struct v4l2_subdev *sd,
    struct v4l2_subdev_state *sd_state,
    struct v4l2_subdev_frame_interval_enum *fie)
{
    return 0;
}

static const struct v4l2_subdev_core_ops node_v_core_ops = {
    .s_power = node_v_s_power,
    .log_status = v4l2_ctrl_subdev_log_status,
    .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
    .unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops node_v_video_ops = {
    .g_frame_interval = node_v_g_frame_interval,
    .s_frame_interval = node_v_s_frame_interval,
    .s_stream = node_v_s_stream,
};

static const struct v4l2_subdev_pad_ops node_v_pad_ops = {
    .init_cfg = node_v_init_cfg,
    .enum_mbus_code = node_v_enum_mbus_code,
    .get_fmt = node_v_get_fmt,
    .set_fmt = node_v_set_fmt,
    .get_selection = node_v_get_selection,
    .enum_frame_size = node_v_enum_frame_size,
    .enum_frame_interval = node_v_enum_frame_interval,
};

static const struct v4l2_subdev_ops camdev_subdev_ops = {
    .core = &node_v_core_ops,
    .video = &node_v_video_ops,
    .pad = &node_v_pad_ops,
};

static int node_v_link_setup(struct media_entity *entity,
               const struct media_pad *local,
               const struct media_pad *remote, u32 flags)
{
    return 0;
}

static const struct media_entity_operations node_v_sd_media_ops = {
    .link_setup = node_v_link_setup,
};

static int sdt_cam_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct node_v_dev *sensor;
    struct fwnode_handle *endpoint;
    int ret = -ENOMEM;

    dev_info(&pdev->dev, "%s\n", __func__);
    sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;

    sensor->pdev = pdev;

    // Parse DT?
    endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
    if (!endpoint) {
        dev_err(dev, "endpoint node not found\n");
        return -EINVAL;
    }

    //printk("dscam6::probe 2\n");

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


    v4l2_subdev_init(&sensor->sd, &camdev_subdev_ops);

    sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
                V4L2_SUBDEV_FL_HAS_EVENTS;
    sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
    sensor->sd.entity.ops = &node_v_sd_media_ops;
    sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
    if (ret)
        return ret;

    sensor->sd.owner = THIS_MODULE;
    snprintf(sensor->sd.name, sizeof(sensor->sd.name), "%s.%d",
         DRIVER_NAME, sensor->id);

    sensor->sd.dev = &pdev->dev;

    ret = v4l2_async_register_subdev_sensor(&sensor->sd);
    if (ret)
        goto media_entity_cleanup;

    dev_info(dev, "probe end\n");

    sensor->streaming = 0;
    //sensor->flags = MXC_MIPI_CSI2_PM_POWERED;
    pm_runtime_enable(&pdev->dev);

    return 0;

media_entity_cleanup:
    media_entity_cleanup(&sensor->sd.entity);

    return ret;
}

static int sdt_cam_remove(struct platform_device *pdev)
{
    struct v4l2_subdev *sd = platform_get_drvdata(pdev);
    struct node_v_dev *sensor = to_node_v_dev(sd);

    v4l2_async_unregister_subdev(&sensor->sd);
    media_entity_cleanup(&sensor->sd.entity);
    //v4l2_ctrl_handler_free(&sensor->ctrls.handler); // There is no handler
    //mutex_destroy(&sensor->lock); // There need no lock.
    kfree(sensor);

    return 0;
}

static int __maybe_unused sdt_cam_pm_runtime_resume(struct device *dev)
{
    // Do nothing yet

    return 0;
}

static int __maybe_unused sdt_cam_runtime_pm_suspend(struct device *dev)
{
    // Do nothing yet.

    return 0;
}

static int __maybe_unused sdt_cam_pm_suspend(struct device *dev)
{
    // Do nothing yet.

    return 0;
}

static int __maybe_unused sdt_cam_pm_resume(struct device *dev)
{
    // Do nothing yet.

    return 0;
}

static const struct dev_pm_ops sdt_cam_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(sdt_cam_pm_suspend, sdt_cam_pm_resume)
    SET_RUNTIME_PM_OPS(sdt_cam_runtime_pm_suspend,
                sdt_cam_pm_runtime_resume,
                NULL)
};

static const struct of_device_id sdt_4k_cam_of_match[] = {
    {.compatible = "sdt,mipi_4lane_4k_cam",},
    { /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, sdt_4k_cam_of_match);

static struct platform_driver sdt_4k_mipi_cam_driver = {
    .driver = {
           .name = "nodev",
           .of_match_table = of_match_ptr(sdt_4k_cam_of_match),
           .pm = &sdt_cam_pm_ops,
           },
    .probe = sdt_cam_probe,
    .remove = sdt_cam_remove,
};

module_platform_driver(sdt_4k_mipi_cam_driver);

MODULE_DESCRIPTION("NODE V MIPI 4Lane 4K Camera Subdev Driver");
MODULE_LICENSE("GPL");

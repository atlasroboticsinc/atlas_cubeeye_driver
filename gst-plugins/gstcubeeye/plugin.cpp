/**
 * plugin.cpp - CubeEye GStreamer Plugin Registration
 *
 * Registers all CubeEye GStreamer elements.
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include "gstcubeeyesrc.h"
#include "gstcubeeye_src.h"
#include "gstcubeeye_depth.h"

#define PACKAGE "gstcubeeye"
#define VERSION "1.0.0"

static gboolean plugin_init(GstPlugin *plugin) {
    gboolean ret = TRUE;

    /* Register cubeeyesrc - combined capture + extraction (recommended) */
    ret &= gst_element_register(plugin, "cubeeyesrc", GST_RANK_PRIMARY,
                                 GST_TYPE_CUBEEYESRC);

    /* Register legacy elements (for flexibility/debugging) */
    ret &= gst_element_register(plugin, "cubeeye_src", GST_RANK_NONE,
                                 GST_TYPE_CUBEEYE_SRC);
    ret &= gst_element_register(plugin, "cubeeye_depth", GST_RANK_NONE,
                                 GST_TYPE_CUBEEYE_DEPTH);

    return ret;
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    cubeeye,
    "CubeEye I200D ToF Camera Elements",
    plugin_init,
    VERSION,
    "LGPL",
    "Atlas CubeEye Driver",
    "https://github.com/atlasroboticsinc/atlas_cubeeye_driver"
)

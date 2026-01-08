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

#define PACKAGE "gstcubeeye"
#define VERSION "1.0.0"

static gboolean plugin_init(GstPlugin *plugin) {
    /* Register cubeeyesrc - combined capture + depth/amplitude extraction */
    return gst_element_register(plugin, "cubeeyesrc", GST_RANK_PRIMARY,
                                 GST_TYPE_CUBEEYESRC);
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

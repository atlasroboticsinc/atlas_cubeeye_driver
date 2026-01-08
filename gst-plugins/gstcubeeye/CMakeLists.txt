# CMakeLists.txt for CubeEye GStreamer Plugin
#
# Builds the gstcubeeye plugin containing:
#   - cubeeye_src: V4L2 source with UVC XU initialization
#   - cubeeye_depth: CUDA depth extraction (future)
#
# Copyright (c) 2025 Atlas Robotics Inc.

cmake_minimum_required(VERSION 3.16)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
    gstreamer-1.0
    gstreamer-base-1.0
    gstreamer-video-1.0
)

# GStreamer plugin shared library
add_library(gstcubeeye SHARED
    plugin.cpp
    gstcubeeye_src.cpp
)

target_include_directories(gstcubeeye PRIVATE
    ${GST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_directories(gstcubeeye PRIVATE
    ${GST_LIBRARY_DIRS}
)

target_link_libraries(gstcubeeye PRIVATE
    ${GST_LIBRARIES}
    cubeeye_device
)

target_compile_options(gstcubeeye PRIVATE
    ${GST_CFLAGS_OTHER}
)

# Set output name without 'lib' prefix (GStreamer convention)
set_target_properties(gstcubeeye PROPERTIES
    PREFIX ""
    OUTPUT_NAME "gstcubeeye"
)

# Install to GStreamer plugin directory (optional)
# pkg_get_variable(GST_PLUGIN_DIR gstreamer-1.0 pluginsdir)
# install(TARGETS gstcubeeye DESTINATION ${GST_PLUGIN_DIR})

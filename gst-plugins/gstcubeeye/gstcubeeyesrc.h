/**
 * gstcubeeyesrc.h - CubeEye I200D ToF Camera GStreamer Source Element
 *
 * Combined source element that captures from CubeEye I200D and outputs
 * extracted depth/amplitude frames. Does V4L2 capture + depth extraction
 * in a single element.
 *
 * Output: video/x-raw, format=GRAY16_LE, width=640, height=480
 *
 * Properties:
 *   serial              - Camera serial number for device selection
 *   device              - V4L2 device path (auto-detected from serial)
 *   output-type         - "depth" or "amplitude"
 *   gradient-correction - Apply polynomial gradient correction
 *   normalize           - Scale output for display (0-max_depth -> 0-65535)
 *   max-depth           - Maximum depth in mm for normalization
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#ifndef __GST_CUBEEYESRC_H__
#define __GST_CUBEEYESRC_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

G_BEGIN_DECLS

#define GST_TYPE_CUBEEYESRC (gst_cubeeyesrc_get_type())
#define GST_CUBEEYESRC(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_CUBEEYESRC, GstCubeEyeSrc))
#define GST_CUBEEYESRC_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_CUBEEYESRC, GstCubeEyeSrcClass))
#define GST_IS_CUBEEYESRC(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_CUBEEYESRC))
#define GST_IS_CUBEEYESRC_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_CUBEEYESRC))

typedef struct _GstCubeEyeSrc GstCubeEyeSrc;
typedef struct _GstCubeEyeSrcClass GstCubeEyeSrcClass;
typedef struct _GstCubeEyeSrcPrivate GstCubeEyeSrcPrivate;

/* Raw frame constants */
#define CUBEEYESRC_RAW_WIDTH      1600
#define CUBEEYESRC_RAW_HEIGHT     241
#define CUBEEYESRC_RAW_BPP        2
#define CUBEEYESRC_RAW_STRIDE     (CUBEEYESRC_RAW_WIDTH * CUBEEYESRC_RAW_BPP)
#define CUBEEYESRC_RAW_SIZE       (CUBEEYESRC_RAW_STRIDE * CUBEEYESRC_RAW_HEIGHT)

/* Output frame constants */
#define CUBEEYESRC_OUT_WIDTH      640
#define CUBEEYESRC_OUT_HEIGHT     480
#define CUBEEYESRC_OUT_SIZE       (CUBEEYESRC_OUT_WIDTH * CUBEEYESRC_OUT_HEIGHT * 2)

/* Output type enum */
typedef enum {
    CUBEEYESRC_OUTPUT_DEPTH = 0,
    CUBEEYESRC_OUTPUT_AMPLITUDE = 1,
} GstCubeEyeSrcOutputType;

struct _GstCubeEyeSrc {
    GstPushSrc parent;

    /* Properties */
    gchar *serial;
    gchar *device;
    gint device_index;
    GstCubeEyeSrcOutputType output_type;
    gboolean gradient_correction;
    gboolean normalize;
    gint max_depth;

    /* V4L2 state */
    gint fd;
    gboolean is_streaming;

    /* Buffer management */
    guint num_buffers;
    struct {
        void *start;
        gsize length;
    } *buffers;

    /* Frame info */
    guint64 frame_count;
    GstClockTime base_time;

    /* Private (depth extractor, etc.) */
    GstCubeEyeSrcPrivate *priv;
};

struct _GstCubeEyeSrcClass {
    GstPushSrcClass parent_class;
};

GType gst_cubeeyesrc_get_type(void);

G_END_DECLS

#endif /* __GST_CUBEEYESRC_H__ */

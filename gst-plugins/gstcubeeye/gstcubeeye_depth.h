/**
 * gstcubeeye_depth.h - CubeEye CUDA Depth Extraction GStreamer Element
 *
 * Transform element that extracts depth and amplitude from raw CubeEye frames
 * using CUDA acceleration.
 *
 * Input:  video/x-raw, format=GRAY8, width=3200, height=241 (raw frame)
 * Output: video/x-raw, format=GRAY16_LE, width=640, height=480 (depth/amplitude)
 *
 * Properties:
 *   enable-depth       - Enable depth output pad (default: true)
 *   enable-amplitude   - Enable amplitude output pad (default: false)
 *   gradient-correction - Apply polynomial gradient correction (default: true)
 *   interpolate        - 2x vertical interpolation (default: true)
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#ifndef __GST_CUBEEYE_DEPTH_H__
#define __GST_CUBEEYE_DEPTH_H__

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

G_BEGIN_DECLS

#define GST_TYPE_CUBEEYE_DEPTH             (gst_cubeeye_depth_get_type())
#define GST_CUBEEYE_DEPTH(obj)             (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_CUBEEYE_DEPTH, GstCubeEyeDepth))
#define GST_CUBEEYE_DEPTH_CLASS(klass)     (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_CUBEEYE_DEPTH, GstCubeEyeDepthClass))
#define GST_IS_CUBEEYE_DEPTH(obj)          (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_CUBEEYE_DEPTH))
#define GST_IS_CUBEEYE_DEPTH_CLASS(klass)  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_CUBEEYE_DEPTH))
#define GST_CUBEEYE_DEPTH_GET_CLASS(obj)   (G_TYPE_INSTANCE_GET_CLASS((obj), GST_TYPE_CUBEEYE_DEPTH, GstCubeEyeDepthClass))

typedef struct _GstCubeEyeDepth GstCubeEyeDepth;
typedef struct _GstCubeEyeDepthClass GstCubeEyeDepthClass;
typedef struct _GstCubeEyeDepthPrivate GstCubeEyeDepthPrivate;

/* Input/output dimensions */
#define CUBEEYE_DEPTH_RAW_WIDTH       3200
#define CUBEEYE_DEPTH_RAW_HEIGHT      241
#define CUBEEYE_DEPTH_RAW_SIZE        (CUBEEYE_DEPTH_RAW_WIDTH * CUBEEYE_DEPTH_RAW_HEIGHT)

#define CUBEEYE_DEPTH_OUT_WIDTH       640
#define CUBEEYE_DEPTH_OUT_HEIGHT_480  480
#define CUBEEYE_DEPTH_OUT_HEIGHT_240  240

struct _GstCubeEyeDepth {
    GstBaseTransform parent;

    /* Properties */
    gboolean enable_depth;
    gboolean enable_amplitude;
    gboolean gradient_correction;
    gboolean interpolate;

    /* Output dimensions */
    gint output_width;
    gint output_height;

    /* Private implementation (CUDA context, etc.) */
    GstCubeEyeDepthPrivate *priv;
};

struct _GstCubeEyeDepthClass {
    GstBaseTransformClass parent_class;
};

GType gst_cubeeye_depth_get_type(void);

G_END_DECLS

#endif /* __GST_CUBEEYE_DEPTH_H__ */

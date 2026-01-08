/**
 * gstcubeeye_src.h - CubeEye I200D GStreamer Source Element
 *
 * V4L2-based source element with UVC XU initialization for CubeEye ToF cameras.
 * Supports serial number-based device selection.
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#ifndef __GST_CUBEEYE_SRC_H__
#define __GST_CUBEEYE_SRC_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

G_BEGIN_DECLS

#define GST_TYPE_CUBEEYE_SRC (gst_cubeeye_src_get_type())
#define GST_CUBEEYE_SRC(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_CUBEEYE_SRC, GstCubeEyeSrc))
#define GST_CUBEEYE_SRC_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_CUBEEYE_SRC, GstCubeEyeSrcClass))
#define GST_IS_CUBEEYE_SRC(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_CUBEEYE_SRC))
#define GST_IS_CUBEEYE_SRC_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_CUBEEYE_SRC))

typedef struct _GstCubeEyeSrc GstCubeEyeSrc;
typedef struct _GstCubeEyeSrcClass GstCubeEyeSrcClass;
typedef struct _GstCubeEyeSrcPrivate GstCubeEyeSrcPrivate;

/**
 * GstCubeEyeSrc:
 *
 * CubeEye I200D source element instance structure.
 */
struct _GstCubeEyeSrc {
    GstPushSrc parent;

    /*< private >*/
    GstCubeEyeSrcPrivate *priv;

    /* Properties */
    gchar *serial;              /* Camera serial number */
    gchar *device;              /* Device path (e.g., /dev/video0) */
    gint device_index;          /* Device index (-1 = auto) */
    gint integration_time;      /* Sensor integration time (us) */
    gboolean auto_exposure;     /* Enable auto exposure */

    /* V4L2 state */
    gint fd;                    /* File descriptor */
    gboolean is_streaming;      /* Streaming state */

    /* Buffer management */
    guint num_buffers;          /* Number of MMAP buffers */
    struct {
        void *start;
        gsize length;
    } *buffers;

    /* Frame info */
    guint frame_width;
    guint frame_height;
    guint frame_size;
    guint64 frame_count;

    /* Timing */
    GstClockTime base_time;
    GstClockTime last_frame_time;
};

struct _GstCubeEyeSrcClass {
    GstPushSrcClass parent_class;
};

GType gst_cubeeye_src_get_type(void);

/* Frame format constants */
#define CUBEEYE_RAW_WIDTH   1600
#define CUBEEYE_RAW_HEIGHT  241
#define CUBEEYE_RAW_BPP     2       /* YUYV = 2 bytes per pixel */
#define CUBEEYE_RAW_STRIDE  (CUBEEYE_RAW_WIDTH * CUBEEYE_RAW_BPP)
#define CUBEEYE_RAW_SIZE    (CUBEEYE_RAW_STRIDE * CUBEEYE_RAW_HEIGHT)

/* Output format (raw bytes as GRAY8) */
#define CUBEEYE_OUT_WIDTH   3200    /* 1600 * 2 bytes = 3200 */
#define CUBEEYE_OUT_HEIGHT  241
#define CUBEEYE_OUT_SIZE    (CUBEEYE_OUT_WIDTH * CUBEEYE_OUT_HEIGHT)

G_END_DECLS

#endif /* __GST_CUBEEYE_SRC_H__ */

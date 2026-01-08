/**
 * gstcubeeye_src.cpp - CubeEye I200D GStreamer Source Element
 *
 * V4L2-based source element with UVC XU initialization for CubeEye ToF cameras.
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstcubeeye_src.h"
#include "../../src/cubeeye_device.h"

#include <gst/video/video.h>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

GST_DEBUG_CATEGORY_STATIC(gst_cubeeye_src_debug);
#define GST_CAT_DEFAULT gst_cubeeye_src_debug

/* Properties */
enum {
    PROP_0,
    PROP_SERIAL,
    PROP_DEVICE,
    PROP_DEVICE_INDEX,
    PROP_INTEGRATION_TIME,
    PROP_AUTO_EXPOSURE,
};

/* Default values */
#define DEFAULT_SERIAL          ""
#define DEFAULT_DEVICE          ""
#define DEFAULT_DEVICE_INDEX    -1
#define DEFAULT_INTEGRATION_TIME 1000
#define DEFAULT_AUTO_EXPOSURE   TRUE
#define DEFAULT_NUM_BUFFERS     4

/* Pad template */
static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS(
        "video/x-raw, "
        "format = (string) GRAY8, "
        "width = (int) " G_STRINGIFY(CUBEEYE_OUT_WIDTH) ", "
        "height = (int) " G_STRINGIFY(CUBEEYE_OUT_HEIGHT) ", "
        "framerate = (fraction) 15/1"
    )
);

#define gst_cubeeye_src_parent_class parent_class
G_DEFINE_TYPE(GstCubeEyeSrc, gst_cubeeye_src, GST_TYPE_PUSH_SRC);

/* Forward declarations */
static void gst_cubeeye_src_finalize(GObject *object);
static void gst_cubeeye_src_set_property(GObject *object, guint prop_id,
                                          const GValue *value, GParamSpec *pspec);
static void gst_cubeeye_src_get_property(GObject *object, guint prop_id,
                                          GValue *value, GParamSpec *pspec);
static gboolean gst_cubeeye_src_start(GstBaseSrc *src);
static gboolean gst_cubeeye_src_stop(GstBaseSrc *src);
static GstFlowReturn gst_cubeeye_src_create(GstPushSrc *src, GstBuffer **buf);
static gboolean gst_cubeeye_src_unlock(GstBaseSrc *src);
static gboolean gst_cubeeye_src_unlock_stop(GstBaseSrc *src);

/* Helper for ioctl with retry */
static int xioctl(int fd, unsigned long request, void *arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

/* Class initialization */
static void gst_cubeeye_src_class_init(GstCubeEyeSrcClass *klass) {
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
    GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS(klass);
    GstPushSrcClass *pushsrc_class = GST_PUSH_SRC_CLASS(klass);

    gobject_class->finalize = gst_cubeeye_src_finalize;
    gobject_class->set_property = gst_cubeeye_src_set_property;
    gobject_class->get_property = gst_cubeeye_src_get_property;

    basesrc_class->start = GST_DEBUG_FUNCPTR(gst_cubeeye_src_start);
    basesrc_class->stop = GST_DEBUG_FUNCPTR(gst_cubeeye_src_stop);
    basesrc_class->unlock = GST_DEBUG_FUNCPTR(gst_cubeeye_src_unlock);
    basesrc_class->unlock_stop = GST_DEBUG_FUNCPTR(gst_cubeeye_src_unlock_stop);

    pushsrc_class->create = GST_DEBUG_FUNCPTR(gst_cubeeye_src_create);

    /* Install properties */
    g_object_class_install_property(gobject_class, PROP_SERIAL,
        g_param_spec_string("serial", "Serial Number",
            "Camera serial number for device selection",
            DEFAULT_SERIAL,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_DEVICE,
        g_param_spec_string("device", "Device",
            "V4L2 device path (e.g., /dev/video0)",
            DEFAULT_DEVICE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_DEVICE_INDEX,
        g_param_spec_int("device-index", "Device Index",
            "V4L2 device index (-1 = auto from serial)",
            -1, 63, DEFAULT_DEVICE_INDEX,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_INTEGRATION_TIME,
        g_param_spec_int("integration-time", "Integration Time",
            "Sensor integration time in microseconds",
            100, 2000, DEFAULT_INTEGRATION_TIME,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_AUTO_EXPOSURE,
        g_param_spec_boolean("auto-exposure", "Auto Exposure",
            "Enable automatic exposure control",
            DEFAULT_AUTO_EXPOSURE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    /* Set element metadata */
    gst_element_class_set_static_metadata(element_class,
        "CubeEye I200D Source",
        "Source/Video",
        "Captures raw frames from CubeEye I200D ToF camera",
        "Atlas Robotics Inc.");

    gst_element_class_add_static_pad_template(element_class, &src_template);

    GST_DEBUG_CATEGORY_INIT(gst_cubeeye_src_debug, "cubeeye_src", 0,
        "CubeEye I200D source element");
}

/* Instance initialization */
static void gst_cubeeye_src_init(GstCubeEyeSrc *self) {
    self->serial = g_strdup(DEFAULT_SERIAL);
    self->device = g_strdup(DEFAULT_DEVICE);
    self->device_index = DEFAULT_DEVICE_INDEX;
    self->integration_time = DEFAULT_INTEGRATION_TIME;
    self->auto_exposure = DEFAULT_AUTO_EXPOSURE;

    self->fd = -1;
    self->is_streaming = FALSE;
    self->buffers = NULL;
    self->num_buffers = DEFAULT_NUM_BUFFERS;

    self->frame_width = CUBEEYE_RAW_WIDTH;
    self->frame_height = CUBEEYE_RAW_HEIGHT;
    self->frame_size = CUBEEYE_RAW_SIZE;
    self->frame_count = 0;

    self->base_time = GST_CLOCK_TIME_NONE;
    self->last_frame_time = GST_CLOCK_TIME_NONE;

    /* Configure as live source */
    gst_base_src_set_live(GST_BASE_SRC(self), TRUE);
    gst_base_src_set_format(GST_BASE_SRC(self), GST_FORMAT_TIME);
}

static void gst_cubeeye_src_finalize(GObject *object) {
    GstCubeEyeSrc *self = GST_CUBEEYE_SRC(object);

    g_free(self->serial);
    g_free(self->device);

    G_OBJECT_CLASS(parent_class)->finalize(object);
}

static void gst_cubeeye_src_set_property(GObject *object, guint prop_id,
                                          const GValue *value, GParamSpec *pspec) {
    GstCubeEyeSrc *self = GST_CUBEEYE_SRC(object);

    switch (prop_id) {
        case PROP_SERIAL:
            g_free(self->serial);
            self->serial = g_value_dup_string(value);
            break;
        case PROP_DEVICE:
            g_free(self->device);
            self->device = g_value_dup_string(value);
            break;
        case PROP_DEVICE_INDEX:
            self->device_index = g_value_get_int(value);
            break;
        case PROP_INTEGRATION_TIME:
            self->integration_time = g_value_get_int(value);
            break;
        case PROP_AUTO_EXPOSURE:
            self->auto_exposure = g_value_get_boolean(value);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static void gst_cubeeye_src_get_property(GObject *object, guint prop_id,
                                          GValue *value, GParamSpec *pspec) {
    GstCubeEyeSrc *self = GST_CUBEEYE_SRC(object);

    switch (prop_id) {
        case PROP_SERIAL:
            g_value_set_string(value, self->serial);
            break;
        case PROP_DEVICE:
            g_value_set_string(value, self->device);
            break;
        case PROP_DEVICE_INDEX:
            g_value_set_int(value, self->device_index);
            break;
        case PROP_INTEGRATION_TIME:
            g_value_set_int(value, self->integration_time);
            break;
        case PROP_AUTO_EXPOSURE:
            g_value_set_boolean(value, self->auto_exposure);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

/* Find and open the device */
static gboolean gst_cubeeye_src_open_device(GstCubeEyeSrc *self) {
    std::string device_path;

    /* Resolve device path from serial or use provided path */
    if (self->serial && strlen(self->serial) > 0) {
        auto dev = cubeeye::DeviceEnumerator::findBySerial(self->serial);
        if (!dev) {
            GST_ERROR_OBJECT(self, "Device with serial '%s' not found", self->serial);
            return FALSE;
        }
        device_path = dev->device_path;
        GST_INFO_OBJECT(self, "Found device '%s' at %s", self->serial, device_path.c_str());
    } else if (self->device && strlen(self->device) > 0) {
        device_path = self->device;
    } else if (self->device_index >= 0) {
        device_path = "/dev/video" + std::to_string(self->device_index);
    } else {
        /* Auto-detect first CubeEye */
        auto dev = cubeeye::DeviceEnumerator::findFirst();
        if (!dev) {
            GST_ERROR_OBJECT(self, "No CubeEye device found");
            return FALSE;
        }
        device_path = dev->device_path;
        GST_INFO_OBJECT(self, "Auto-detected CubeEye at %s (serial: %s)",
                        device_path.c_str(), dev->serial_number.c_str());
    }

    /* Open the device */
    self->fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK);
    if (self->fd < 0) {
        GST_ERROR_OBJECT(self, "Cannot open %s: %s", device_path.c_str(), strerror(errno));
        return FALSE;
    }

    /* Verify it's a video capture device */
    struct v4l2_capability cap;
    if (xioctl(self->fd, VIDIOC_QUERYCAP, &cap) < 0) {
        GST_ERROR_OBJECT(self, "VIDIOC_QUERYCAP failed: %s", strerror(errno));
        close(self->fd);
        self->fd = -1;
        return FALSE;
    }

    if (!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) {
        GST_ERROR_OBJECT(self, "Device is not a video capture device");
        close(self->fd);
        self->fd = -1;
        return FALSE;
    }

    GST_INFO_OBJECT(self, "Opened device: %s (%s)", device_path.c_str(), cap.card);
    return TRUE;
}

/* Initialize the sensor via UVC XU commands */
static gboolean gst_cubeeye_src_init_sensor(GstCubeEyeSrc *self) {
    GST_INFO_OBJECT(self, "Initializing CubeEye sensor");

    if (!cubeeye::UvcControl::initialize(self->fd)) {
        GST_ERROR_OBJECT(self, "Failed to initialize sensor via UVC XU");
        return FALSE;
    }

    GST_INFO_OBJECT(self, "Sensor initialized successfully");
    return TRUE;
}

/* Set up V4L2 format and buffers */
static gboolean gst_cubeeye_src_setup_capture(GstCubeEyeSrc *self) {
    /* Set format */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = CUBEEYE_RAW_WIDTH;
    fmt.fmt.pix.height = CUBEEYE_RAW_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(self->fd, VIDIOC_S_FMT, &fmt) < 0) {
        GST_ERROR_OBJECT(self, "VIDIOC_S_FMT failed: %s", strerror(errno));
        return FALSE;
    }

    GST_INFO_OBJECT(self, "Format: %ux%u, stride=%u, size=%u",
                    fmt.fmt.pix.width, fmt.fmt.pix.height,
                    fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);

    self->frame_size = fmt.fmt.pix.sizeimage;

    /* Request buffers */
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = self->num_buffers;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(self->fd, VIDIOC_REQBUFS, &req) < 0) {
        GST_ERROR_OBJECT(self, "VIDIOC_REQBUFS failed: %s", strerror(errno));
        return FALSE;
    }

    if (req.count < 2) {
        GST_ERROR_OBJECT(self, "Insufficient buffer memory");
        return FALSE;
    }

    self->num_buffers = req.count;
    GST_INFO_OBJECT(self, "Allocated %u buffers", self->num_buffers);

    /* Allocate buffer info array */
    self->buffers = (decltype(self->buffers))g_malloc0(
        self->num_buffers * sizeof(*self->buffers));

    /* Map buffers */
    for (guint i = 0; i < self->num_buffers; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(self->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            GST_ERROR_OBJECT(self, "VIDIOC_QUERYBUF failed: %s", strerror(errno));
            return FALSE;
        }

        self->buffers[i].length = buf.length;
        self->buffers[i].start = mmap(NULL, buf.length,
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED, self->fd, buf.m.offset);

        if (self->buffers[i].start == MAP_FAILED) {
            GST_ERROR_OBJECT(self, "mmap failed: %s", strerror(errno));
            return FALSE;
        }
    }

    /* Queue buffers */
    for (guint i = 0; i < self->num_buffers; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(self->fd, VIDIOC_QBUF, &buf) < 0) {
            GST_ERROR_OBJECT(self, "VIDIOC_QBUF failed: %s", strerror(errno));
            return FALSE;
        }
    }

    return TRUE;
}

/* Start streaming */
static gboolean gst_cubeeye_src_start_streaming(GstCubeEyeSrc *self) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(self->fd, VIDIOC_STREAMON, &type) < 0) {
        GST_ERROR_OBJECT(self, "VIDIOC_STREAMON failed: %s", strerror(errno));
        return FALSE;
    }

    self->is_streaming = TRUE;
    self->frame_count = 0;
    self->base_time = GST_CLOCK_TIME_NONE;

    GST_INFO_OBJECT(self, "Streaming started");
    return TRUE;
}

/* Stop streaming */
static void gst_cubeeye_src_stop_streaming(GstCubeEyeSrc *self) {
    if (!self->is_streaming)
        return;

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(self->fd, VIDIOC_STREAMOFF, &type);

    self->is_streaming = FALSE;
    GST_INFO_OBJECT(self, "Streaming stopped");
}

/* Clean up buffers */
static void gst_cubeeye_src_cleanup_buffers(GstCubeEyeSrc *self) {
    if (self->buffers) {
        for (guint i = 0; i < self->num_buffers; i++) {
            if (self->buffers[i].start && self->buffers[i].start != MAP_FAILED) {
                munmap(self->buffers[i].start, self->buffers[i].length);
            }
        }
        g_free(self->buffers);
        self->buffers = NULL;
    }
}

/* Shutdown sensor */
static void gst_cubeeye_src_shutdown_sensor(GstCubeEyeSrc *self) {
    if (self->fd >= 0) {
        GST_INFO_OBJECT(self, "Shutting down sensor");
        cubeeye::UvcControl::shutdown(self->fd);
    }
}

/* Start element */
static gboolean gst_cubeeye_src_start(GstBaseSrc *src) {
    GstCubeEyeSrc *self = GST_CUBEEYE_SRC(src);

    GST_INFO_OBJECT(self, "Starting CubeEye source");

    /* Open device */
    if (!gst_cubeeye_src_open_device(self)) {
        return FALSE;
    }

    /* Initialize sensor */
    if (!gst_cubeeye_src_init_sensor(self)) {
        close(self->fd);
        self->fd = -1;
        return FALSE;
    }

    /* Set up capture */
    if (!gst_cubeeye_src_setup_capture(self)) {
        gst_cubeeye_src_shutdown_sensor(self);
        close(self->fd);
        self->fd = -1;
        return FALSE;
    }

    /* Start streaming */
    if (!gst_cubeeye_src_start_streaming(self)) {
        gst_cubeeye_src_cleanup_buffers(self);
        gst_cubeeye_src_shutdown_sensor(self);
        close(self->fd);
        self->fd = -1;
        return FALSE;
    }

    return TRUE;
}

/* Stop element */
static gboolean gst_cubeeye_src_stop(GstBaseSrc *src) {
    GstCubeEyeSrc *self = GST_CUBEEYE_SRC(src);

    GST_INFO_OBJECT(self, "Stopping CubeEye source");

    gst_cubeeye_src_stop_streaming(self);
    gst_cubeeye_src_cleanup_buffers(self);
    gst_cubeeye_src_shutdown_sensor(self);

    if (self->fd >= 0) {
        close(self->fd);
        self->fd = -1;
    }

    return TRUE;
}

/* Create a buffer (capture a frame) */
static GstFlowReturn gst_cubeeye_src_create(GstPushSrc *src, GstBuffer **buf) {
    GstCubeEyeSrc *self = GST_CUBEEYE_SRC(src);
    GstBuffer *outbuf;
    GstMapInfo map;

    /* Loop until we get a valid frame */
    while (TRUE) {
        /* Wait for frame with timeout */
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(self->fd, &fds);

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int r = select(self->fd + 1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            if (errno == EINTR) {
                continue;  /* Interrupted, try again */
            }
            GST_ERROR_OBJECT(self, "select failed: %s", strerror(errno));
            return GST_FLOW_ERROR;
        } else if (r == 0) {
            GST_WARNING_OBJECT(self, "Timeout waiting for frame");
            return GST_FLOW_ERROR;
        }

        /* Dequeue buffer */
        struct v4l2_buffer v4l2_buf;
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(self->fd, VIDIOC_DQBUF, &v4l2_buf) < 0) {
            if (errno == EAGAIN) {
                continue;  /* No frame available yet, try again */
            }
            GST_ERROR_OBJECT(self, "VIDIOC_DQBUF failed: %s", strerror(errno));
            return GST_FLOW_ERROR;
        }

        /* Check frame validity */
        if (v4l2_buf.bytesused == 0) {
            /* Empty frame, re-queue and try again */
            xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);
            continue;
        }

        /* Skip frames that are all zeros (warmup frames) */
        guint8 *frame_data = (guint8*)self->buffers[v4l2_buf.index].start;
        gboolean has_data = FALSE;
        for (guint i = 0; i < 1000 && !has_data; i++) {
            if (frame_data[i] != 0) has_data = TRUE;
        }

        if (!has_data) {
            /* Warmup frame, skip */
            GST_DEBUG_OBJECT(self, "Skipping warmup frame");
            xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);
            continue;
        }

        /* Valid frame - process it */

        /* Allocate output buffer */
        outbuf = gst_buffer_new_allocate(NULL, CUBEEYE_OUT_SIZE, NULL);
        if (!outbuf) {
            GST_ERROR_OBJECT(self, "Failed to allocate buffer");
            xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);
            return GST_FLOW_ERROR;
        }

        /* Copy frame data */
        gst_buffer_map(outbuf, &map, GST_MAP_WRITE);
        memcpy(map.data, frame_data, MIN(v4l2_buf.bytesused, CUBEEYE_OUT_SIZE));
        gst_buffer_unmap(outbuf, &map);

        /* Set timestamps */
        GstClock *clock = gst_element_get_clock(GST_ELEMENT(self));
        GstClockTime now = GST_CLOCK_TIME_NONE;

        if (clock) {
            now = gst_clock_get_time(clock);
            gst_object_unref(clock);
        }

        if (self->base_time == GST_CLOCK_TIME_NONE) {
            self->base_time = now;
        }

        GstClockTime pts = (now != GST_CLOCK_TIME_NONE && self->base_time != GST_CLOCK_TIME_NONE)
                           ? (now - self->base_time) : (self->frame_count * GST_SECOND / 15);

        GST_BUFFER_PTS(outbuf) = pts;
        GST_BUFFER_DTS(outbuf) = pts;
        GST_BUFFER_DURATION(outbuf) = GST_SECOND / 15;  /* ~66.7ms at 15fps */
        GST_BUFFER_OFFSET(outbuf) = self->frame_count;
        GST_BUFFER_OFFSET_END(outbuf) = self->frame_count + 1;

        self->frame_count++;
        self->last_frame_time = now;

        /* Re-queue V4L2 buffer */
        if (xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf) < 0) {
            GST_WARNING_OBJECT(self, "VIDIOC_QBUF failed: %s", strerror(errno));
        }

        GST_LOG_OBJECT(self, "Captured frame %lu, pts=%" GST_TIME_FORMAT,
                       (unsigned long)self->frame_count, GST_TIME_ARGS(pts));

        *buf = outbuf;
        return GST_FLOW_OK;
    }  /* end while(TRUE) */
}

/* Unlock (for flushing) */
static gboolean gst_cubeeye_src_unlock(GstBaseSrc *src) {
    /* Signal any waiting operations to stop */
    return TRUE;
}

static gboolean gst_cubeeye_src_unlock_stop(GstBaseSrc *src) {
    /* Resume normal operation */
    return TRUE;
}

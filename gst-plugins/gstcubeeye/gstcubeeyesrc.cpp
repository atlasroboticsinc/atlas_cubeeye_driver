/**
 * gstcubeeyesrc.cpp - CubeEye I200D ToF Camera GStreamer Source Element
 *
 * Multi-pad source with separate depth and amplitude outputs.
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstcubeeyesrc.h"
#include "cubeeye_device.h"
#include "cubeeye_depth.h"

#ifdef HAVE_CUDA
#include "cubeeye_depth_cuda.h"
#endif

#include <gst/video/video.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <cstring>
#include <cerrno>

GST_DEBUG_CATEGORY_STATIC(gst_cubeeyesrc_debug);
#define GST_CAT_DEFAULT gst_cubeeyesrc_debug

/* Private data */
struct _GstCubeEyeSrcPrivate {
#ifdef HAVE_CUDA
    cubeeye::cuda::CudaDepthExtractor *cuda_extractor;
    gboolean cuda_available;
#endif
    cubeeye::DepthExtractor *cpu_extractor;
    uint8_t *raw_buffer;
    uint16_t *depth_buffer;
    uint16_t *amplitude_buffer;
};

/* Properties */
enum {
    PROP_0,
    PROP_SERIAL,
    PROP_DEVICE,
    PROP_ENABLE_AMPLITUDE,
    PROP_GRADIENT_CORRECTION,
    PROP_NORMALIZE,
    PROP_MAX_DEPTH,
    PROP_MAX_AMPLITUDE,
};

#define DEFAULT_ENABLE_AMP      FALSE
#define DEFAULT_GRADIENT_CORR   TRUE
#define DEFAULT_NORMALIZE       FALSE
#define DEFAULT_MAX_DEPTH       5000
#define DEFAULT_MAX_AMPLITUDE   1000

/* Pad templates */
static GstStaticPadTemplate depth_src_template = GST_STATIC_PAD_TEMPLATE(
    "depth",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS(
        "video/x-raw, "
        "format = (string) GRAY16_LE, "
        "width = (int) 640, "
        "height = (int) 480, "
        "framerate = (fraction) 15/1"
    )
);

static GstStaticPadTemplate amplitude_src_template = GST_STATIC_PAD_TEMPLATE(
    "amplitude",
    GST_PAD_SRC,
    GST_PAD_SOMETIMES,
    GST_STATIC_CAPS(
        "video/x-raw, "
        "format = (string) GRAY16_LE, "
        "width = (int) 640, "
        "height = (int) 480, "
        "framerate = (fraction) 15/1"
    )
);

#define gst_cubeeyesrc_parent_class parent_class
G_DEFINE_TYPE_WITH_PRIVATE(GstCubeEyeSrc, gst_cubeeyesrc, GST_TYPE_ELEMENT);

/* Forward declarations */
static void gst_cubeeyesrc_set_property(GObject *object, guint prop_id,
                                         const GValue *value, GParamSpec *pspec);
static void gst_cubeeyesrc_get_property(GObject *object, guint prop_id,
                                         GValue *value, GParamSpec *pspec);
static void gst_cubeeyesrc_finalize(GObject *object);
static GstStateChangeReturn gst_cubeeyesrc_change_state(GstElement *element,
                                                         GstStateChange transition);
static gpointer gst_cubeeyesrc_thread_func(gpointer data);

/* Helper for ioctl with retry */
static int xioctl(int fd, unsigned long request, void *arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

static void gst_cubeeyesrc_class_init(GstCubeEyeSrcClass *klass) {
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);

    GST_DEBUG_CATEGORY_INIT(gst_cubeeyesrc_debug, "cubeeyesrc", 0,
                            "CubeEye I200D ToF Camera Source");

    gobject_class->set_property = gst_cubeeyesrc_set_property;
    gobject_class->get_property = gst_cubeeyesrc_get_property;
    gobject_class->finalize = gst_cubeeyesrc_finalize;

    element_class->change_state = GST_DEBUG_FUNCPTR(gst_cubeeyesrc_change_state);

    /* Properties */
    g_object_class_install_property(gobject_class, PROP_SERIAL,
        g_param_spec_string("serial", "Serial Number",
            "Camera serial number (e.g., I200DU2427000032)",
            NULL, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_DEVICE,
        g_param_spec_string("device", "Device",
            "V4L2 device path (auto-detected from serial if not set)",
            NULL, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_ENABLE_AMPLITUDE,
        g_param_spec_boolean("enable-amplitude", "Enable Amplitude",
            "Enable amplitude output pad",
            DEFAULT_ENABLE_AMP, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_GRADIENT_CORRECTION,
        g_param_spec_boolean("gradient-correction", "Gradient Correction",
            "Apply polynomial gradient correction to depth",
            DEFAULT_GRADIENT_CORR, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_NORMALIZE,
        g_param_spec_boolean("normalize", "Normalize",
            "Scale depth to full 16-bit range for display",
            DEFAULT_NORMALIZE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_MAX_DEPTH,
        g_param_spec_int("max-depth", "Max Depth",
            "Maximum depth in mm (for normalization)",
            100, 10000, DEFAULT_MAX_DEPTH,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_MAX_AMPLITUDE,
        g_param_spec_int("max-amplitude", "Max Amplitude",
            "Maximum amplitude value (for normalization)",
            10, 4095, DEFAULT_MAX_AMPLITUDE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    /* Element metadata */
    gst_element_class_set_static_metadata(element_class,
        "CubeEye ToF Camera Source",
        "Source/Video",
        "Captures depth and amplitude from CubeEye I200D ToF camera",
        "Atlas Robotics Inc.");

    /* Pad templates */
    gst_element_class_add_static_pad_template(element_class, &depth_src_template);
    gst_element_class_add_static_pad_template(element_class, &amplitude_src_template);
}

static void gst_cubeeyesrc_init(GstCubeEyeSrc *self) {
    self->priv = (GstCubeEyeSrcPrivate *)
        gst_cubeeyesrc_get_instance_private(self);

    /* Create depth pad (always present) */
    self->depth_pad = gst_pad_new_from_static_template(&depth_src_template, "depth");
    gst_pad_set_active(self->depth_pad, TRUE);
    gst_element_add_pad(GST_ELEMENT(self), self->depth_pad);

    /* Amplitude pad created dynamically when enabled */
    self->amplitude_pad = NULL;

    /* Default property values */
    self->serial = NULL;
    self->device = NULL;
    self->enable_amplitude = DEFAULT_ENABLE_AMP;
    self->gradient_correction = DEFAULT_GRADIENT_CORR;
    self->normalize = DEFAULT_NORMALIZE;
    self->max_depth = DEFAULT_MAX_DEPTH;
    self->max_amplitude = DEFAULT_MAX_AMPLITUDE;

    /* State */
    self->fd = -1;
    self->is_streaming = FALSE;
    self->num_buffers = 4;
    self->buffers = NULL;
    self->frame_count = 0;
    self->base_time = GST_CLOCK_TIME_NONE;

    /* Thread */
    self->thread = NULL;
    self->running = FALSE;
    g_mutex_init(&self->lock);

    /* Private */
    self->priv->cpu_extractor = NULL;
#ifdef HAVE_CUDA
    self->priv->cuda_extractor = NULL;
    self->priv->cuda_available = FALSE;
#endif
    self->priv->raw_buffer = NULL;
    self->priv->depth_buffer = NULL;
    self->priv->amplitude_buffer = NULL;
}

static void gst_cubeeyesrc_finalize(GObject *object) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(object);

    g_mutex_clear(&self->lock);
    g_free(self->serial);
    g_free(self->device);

    if (self->priv->raw_buffer) g_free(self->priv->raw_buffer);
    if (self->priv->depth_buffer) g_free(self->priv->depth_buffer);
    if (self->priv->amplitude_buffer) g_free(self->priv->amplitude_buffer);

#ifdef HAVE_CUDA
    if (self->priv->cuda_extractor) delete self->priv->cuda_extractor;
#endif
    if (self->priv->cpu_extractor) delete self->priv->cpu_extractor;

    G_OBJECT_CLASS(parent_class)->finalize(object);
}

static void gst_cubeeyesrc_set_property(GObject *object, guint prop_id,
                                         const GValue *value, GParamSpec *pspec) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(object);

    switch (prop_id) {
        case PROP_SERIAL:
            g_free(self->serial);
            self->serial = g_value_dup_string(value);
            break;
        case PROP_DEVICE:
            g_free(self->device);
            self->device = g_value_dup_string(value);
            break;
        case PROP_ENABLE_AMPLITUDE:
            self->enable_amplitude = g_value_get_boolean(value);
            /* Add/remove amplitude pad */
            if (self->enable_amplitude && !self->amplitude_pad) {
                self->amplitude_pad = gst_pad_new_from_static_template(
                    &amplitude_src_template, "amplitude");
                gst_pad_set_active(self->amplitude_pad, TRUE);
                gst_element_add_pad(GST_ELEMENT(self), self->amplitude_pad);
            } else if (!self->enable_amplitude && self->amplitude_pad) {
                gst_element_remove_pad(GST_ELEMENT(self), self->amplitude_pad);
                self->amplitude_pad = NULL;
            }
            break;
        case PROP_GRADIENT_CORRECTION:
            self->gradient_correction = g_value_get_boolean(value);
            if (self->priv->cpu_extractor) {
                self->priv->cpu_extractor->SetGradientCorrection(self->gradient_correction);
            }
            break;
        case PROP_NORMALIZE:
            self->normalize = g_value_get_boolean(value);
            break;
        case PROP_MAX_DEPTH:
            self->max_depth = g_value_get_int(value);
            break;
        case PROP_MAX_AMPLITUDE:
            self->max_amplitude = g_value_get_int(value);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static void gst_cubeeyesrc_get_property(GObject *object, guint prop_id,
                                         GValue *value, GParamSpec *pspec) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(object);

    switch (prop_id) {
        case PROP_SERIAL:
            g_value_set_string(value, self->serial);
            break;
        case PROP_DEVICE:
            g_value_set_string(value, self->device);
            break;
        case PROP_ENABLE_AMPLITUDE:
            g_value_set_boolean(value, self->enable_amplitude);
            break;
        case PROP_GRADIENT_CORRECTION:
            g_value_set_boolean(value, self->gradient_correction);
            break;
        case PROP_NORMALIZE:
            g_value_set_boolean(value, self->normalize);
            break;
        case PROP_MAX_DEPTH:
            g_value_set_int(value, self->max_depth);
            break;
        case PROP_MAX_AMPLITUDE:
            g_value_set_int(value, self->max_amplitude);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static gboolean gst_cubeeyesrc_start(GstCubeEyeSrc *self) {
    std::string device_path;

    GST_DEBUG_OBJECT(self, "Starting cubeeyesrc");

    /* Find device by serial or use specified device */
    if (self->serial && strlen(self->serial) > 0) {
        auto device = cubeeye::DeviceEnumerator::findBySerial(self->serial);
        if (!device) {
            GST_ERROR_OBJECT(self, "Device with serial %s not found", self->serial);
            return FALSE;
        }
        device_path = device->device_path;
        GST_INFO_OBJECT(self, "Found device %s for serial %s",
                        device_path.c_str(), self->serial);
    } else if (self->device && strlen(self->device) > 0) {
        device_path = self->device;
    } else {
        auto devices = cubeeye::DeviceEnumerator::enumerate();
        if (devices.empty()) {
            GST_ERROR_OBJECT(self, "No CubeEye devices found");
            return FALSE;
        }
        device_path = devices[0].device_path;
        GST_INFO_OBJECT(self, "Using first available device: %s", device_path.c_str());
    }

    /* Open device */
    self->fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK);
    if (self->fd < 0) {
        GST_ERROR_OBJECT(self, "Cannot open %s: %s", device_path.c_str(), strerror(errno));
        return FALSE;
    }

    /* Initialize UVC extension unit */
    if (!cubeeye::UvcControl::initialize(self->fd)) {
        GST_WARNING_OBJECT(self, "UVC initialization failed, continuing anyway");
    }

    /* Set format */
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = CUBEEYESRC_RAW_WIDTH;
    fmt.fmt.pix.height = CUBEEYESRC_RAW_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(self->fd, VIDIOC_S_FMT, &fmt) < 0) {
        GST_ERROR_OBJECT(self, "VIDIOC_S_FMT failed: %s", strerror(errno));
        close(self->fd);
        self->fd = -1;
        return FALSE;
    }

    /* Request buffers */
    struct v4l2_requestbuffers req = {};
    req.count = self->num_buffers;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(self->fd, VIDIOC_REQBUFS, &req) < 0) {
        GST_ERROR_OBJECT(self, "VIDIOC_REQBUFS failed: %s", strerror(errno));
        close(self->fd);
        self->fd = -1;
        return FALSE;
    }

    self->num_buffers = req.count;
    self->buffers = g_new0(typeof(*self->buffers), self->num_buffers);

    /* Map buffers */
    for (guint i = 0; i < self->num_buffers; i++) {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(self->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            GST_ERROR_OBJECT(self, "VIDIOC_QUERYBUF failed");
            goto error;
        }

        self->buffers[i].length = buf.length;
        self->buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, self->fd, buf.m.offset);

        if (self->buffers[i].start == MAP_FAILED) {
            GST_ERROR_OBJECT(self, "mmap failed");
            goto error;
        }
    }

    /* Queue buffers */
    for (guint i = 0; i < self->num_buffers; i++) {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(self->fd, VIDIOC_QBUF, &buf) < 0) {
            GST_ERROR_OBJECT(self, "VIDIOC_QBUF failed");
            goto error;
        }
    }

    /* Start streaming */
    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(self->fd, VIDIOC_STREAMON, &type) < 0) {
            GST_ERROR_OBJECT(self, "VIDIOC_STREAMON failed: %s", strerror(errno));
            goto error;
        }
        self->is_streaming = TRUE;
    }

    /* Initialize depth extractor */
#ifdef HAVE_CUDA
    if (cubeeye::cuda::CudaDepthExtractor::IsCudaAvailable()) {
        try {
            self->priv->cuda_extractor = new cubeeye::cuda::CudaDepthExtractor(
                self->gradient_correction, true);
            self->priv->cuda_available = TRUE;
            GST_INFO_OBJECT(self, "CUDA depth extraction enabled: %s",
                           cubeeye::cuda::CudaDepthExtractor::GetDeviceInfo());
        } catch (const std::exception &e) {
            GST_WARNING_OBJECT(self, "CUDA init failed: %s", e.what());
            self->priv->cuda_available = FALSE;
        }
    }
#endif

    self->priv->cpu_extractor = new cubeeye::DepthExtractor(self->gradient_correction);
    self->priv->raw_buffer = (uint8_t *)g_malloc(CUBEEYESRC_RAW_SIZE);
    self->priv->depth_buffer = (uint16_t *)g_malloc(CUBEEYESRC_OUT_SIZE);
    self->priv->amplitude_buffer = (uint16_t *)g_malloc(CUBEEYESRC_OUT_SIZE);

    self->frame_count = 0;
    self->base_time = GST_CLOCK_TIME_NONE;

    /* Start streaming thread */
    self->running = TRUE;
    self->thread = g_thread_new("cubeeyesrc", gst_cubeeyesrc_thread_func, self);

    GST_INFO_OBJECT(self, "Started successfully on %s", device_path.c_str());
    return TRUE;

error:
    for (guint i = 0; i < self->num_buffers && self->buffers; i++) {
        if (self->buffers[i].start && self->buffers[i].start != MAP_FAILED) {
            munmap(self->buffers[i].start, self->buffers[i].length);
        }
    }
    g_free(self->buffers);
    self->buffers = NULL;
    close(self->fd);
    self->fd = -1;
    return FALSE;
}

static gboolean gst_cubeeyesrc_stop(GstCubeEyeSrc *self) {
    GST_DEBUG_OBJECT(self, "Stopping cubeeyesrc");

    /* Stop thread */
    if (self->thread) {
        self->running = FALSE;
        g_thread_join(self->thread);
        self->thread = NULL;
    }

    if (self->is_streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(self->fd, VIDIOC_STREAMOFF, &type);
        self->is_streaming = FALSE;
    }

    if (self->fd >= 0) {
        cubeeye::UvcControl::shutdown(self->fd);
    }

    for (guint i = 0; i < self->num_buffers && self->buffers; i++) {
        if (self->buffers[i].start && self->buffers[i].start != MAP_FAILED) {
            munmap(self->buffers[i].start, self->buffers[i].length);
        }
    }
    g_free(self->buffers);
    self->buffers = NULL;

    if (self->fd >= 0) {
        close(self->fd);
        self->fd = -1;
    }

#ifdef HAVE_CUDA
    if (self->priv->cuda_extractor) {
        delete self->priv->cuda_extractor;
        self->priv->cuda_extractor = NULL;
    }
#endif

    if (self->priv->cpu_extractor) {
        delete self->priv->cpu_extractor;
        self->priv->cpu_extractor = NULL;
    }

    if (self->priv->raw_buffer) {
        g_free(self->priv->raw_buffer);
        self->priv->raw_buffer = NULL;
    }
    if (self->priv->depth_buffer) {
        g_free(self->priv->depth_buffer);
        self->priv->depth_buffer = NULL;
    }
    if (self->priv->amplitude_buffer) {
        g_free(self->priv->amplitude_buffer);
        self->priv->amplitude_buffer = NULL;
    }

    return TRUE;
}

static GstStateChangeReturn gst_cubeeyesrc_change_state(GstElement *element,
                                                         GstStateChange transition) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(element);
    GstStateChangeReturn ret;

    switch (transition) {
        case GST_STATE_CHANGE_NULL_TO_READY:
            break;
        case GST_STATE_CHANGE_READY_TO_PAUSED:
            if (!gst_cubeeyesrc_start(self)) {
                return GST_STATE_CHANGE_FAILURE;
            }
            break;
        default:
            break;
    }

    ret = GST_ELEMENT_CLASS(parent_class)->change_state(element, transition);

    switch (transition) {
        case GST_STATE_CHANGE_PAUSED_TO_READY:
            gst_cubeeyesrc_stop(self);
            break;
        case GST_STATE_CHANGE_READY_TO_NULL:
            break;
        default:
            break;
    }

    return ret;
}

static gboolean gst_cubeeyesrc_send_stream_events(GstCubeEyeSrc *self, GstPad *pad,
                                                    const gchar *stream_id_suffix) {
    /* Create and send stream-start event */
    gchar *stream_id = gst_pad_create_stream_id(pad, GST_ELEMENT(self), stream_id_suffix);
    GstEvent *stream_start = gst_event_new_stream_start(stream_id);
    gst_event_set_group_id(stream_start, gst_util_group_id_next());
    g_free(stream_id);

    if (!gst_pad_push_event(pad, stream_start)) {
        GST_WARNING_OBJECT(self, "Failed to push stream-start on %s", GST_PAD_NAME(pad));
        return FALSE;
    }

    /* Create and send caps event */
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "GRAY16_LE",
        "width", G_TYPE_INT, CUBEEYESRC_OUT_WIDTH,
        "height", G_TYPE_INT, CUBEEYESRC_OUT_HEIGHT,
        "framerate", GST_TYPE_FRACTION, 15, 1,
        NULL);

    if (!gst_pad_push_event(pad, gst_event_new_caps(caps))) {
        GST_WARNING_OBJECT(self, "Failed to push caps on %s", GST_PAD_NAME(pad));
        gst_caps_unref(caps);
        return FALSE;
    }
    gst_caps_unref(caps);

    /* Create and send segment event */
    GstSegment segment;
    gst_segment_init(&segment, GST_FORMAT_TIME);
    segment.rate = 1.0;
    segment.start = 0;
    segment.time = 0;

    if (!gst_pad_push_event(pad, gst_event_new_segment(&segment))) {
        GST_WARNING_OBJECT(self, "Failed to push segment on %s", GST_PAD_NAME(pad));
        return FALSE;
    }

    GST_DEBUG_OBJECT(self, "Stream events sent on %s", GST_PAD_NAME(pad));
    return TRUE;
}

static gpointer gst_cubeeyesrc_thread_func(gpointer data) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(data);
    gboolean events_sent = FALSE;

    GST_DEBUG_OBJECT(self, "Streaming thread started");

    while (self->running) {
        /* Wait for frame */
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(self->fd, &fds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000;  /* 200ms timeout */

        int r = select(self->fd + 1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            if (errno == EINTR) continue;
            GST_ERROR_OBJECT(self, "select error: %s", strerror(errno));
            break;
        }
        if (r == 0) continue;  /* Timeout, check running flag */

        /* Dequeue buffer */
        struct v4l2_buffer v4l2_buf = {};
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(self->fd, VIDIOC_DQBUF, &v4l2_buf) < 0) {
            if (errno == EAGAIN) continue;
            GST_ERROR_OBJECT(self, "VIDIOC_DQBUF failed: %s", strerror(errno));
            break;
        }

        /* Check for valid frame */
        if (v4l2_buf.bytesused == 0) {
            xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);
            continue;
        }

        /* Check for warmup frames */
        uint8_t *raw_data = (uint8_t *)self->buffers[v4l2_buf.index].start;
        gboolean has_data = FALSE;
        for (guint i = 0; i < 1000 && !has_data; i++) {
            if (raw_data[i] != 0) has_data = TRUE;
        }

        if (!has_data) {
            xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);
            continue;
        }

        /* Copy raw frame */
        memcpy(self->priv->raw_buffer, raw_data, CUBEEYESRC_RAW_SIZE);

        /* Re-queue V4L2 buffer */
        xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);

        /* Extract depth and amplitude */
        gboolean extract_amp = (self->enable_amplitude && self->amplitude_pad);

#ifdef HAVE_CUDA
        if (self->priv->cuda_available && self->priv->cuda_extractor) {
            if (extract_amp) {
                self->priv->cuda_extractor->ExtractDepthAndAmplitude(
                    self->priv->raw_buffer, CUBEEYESRC_RAW_SIZE,
                    self->priv->depth_buffer, self->priv->amplitude_buffer, TRUE);
            } else {
                self->priv->cuda_extractor->ExtractDepth(
                    self->priv->raw_buffer, CUBEEYESRC_RAW_SIZE,
                    self->priv->depth_buffer, TRUE);
            }
        } else
#endif
        {
            if (extract_amp) {
                self->priv->cpu_extractor->ExtractDepthAndAmplitude(
                    self->priv->raw_buffer, CUBEEYESRC_RAW_SIZE,
                    self->priv->depth_buffer, self->priv->amplitude_buffer, TRUE);
            } else {
                self->priv->cpu_extractor->ExtractDepth(
                    self->priv->raw_buffer, CUBEEYESRC_RAW_SIZE,
                    self->priv->depth_buffer, TRUE);
            }
        }

        /* Apply normalization to depth */
        if (self->normalize) {
            gint max_val = self->max_depth;
            gint num_pixels = CUBEEYESRC_OUT_WIDTH * CUBEEYESRC_OUT_HEIGHT;
            for (gint i = 0; i < num_pixels; i++) {
                guint32 val = self->priv->depth_buffer[i];
                if (val > (guint32)max_val) val = max_val;
                self->priv->depth_buffer[i] = (uint16_t)((val * 65535) / max_val);
            }
        }

        /* Apply normalization to amplitude */
        if (self->normalize && extract_amp) {
            gint max_val = self->max_amplitude;
            gint num_pixels = CUBEEYESRC_OUT_WIDTH * CUBEEYESRC_OUT_HEIGHT;
            for (gint i = 0; i < num_pixels; i++) {
                guint32 val = self->priv->amplitude_buffer[i];
                if (val > (guint32)max_val) val = max_val;
                self->priv->amplitude_buffer[i] = (uint16_t)((val * 65535) / max_val);
            }
        }

        /* Send stream events before first buffer */
        if (!events_sent) {
            if (!gst_cubeeyesrc_send_stream_events(self, self->depth_pad, "depth")) {
                GST_ERROR_OBJECT(self, "Failed to send depth stream events");
                break;
            }
            if (extract_amp && self->amplitude_pad) {
                if (!gst_cubeeyesrc_send_stream_events(self, self->amplitude_pad, "amplitude")) {
                    GST_ERROR_OBJECT(self, "Failed to send amplitude stream events");
                    break;
                }
            }
            events_sent = TRUE;
            GST_INFO_OBJECT(self, "Stream events sent, starting data flow");
        }

        /* Get timestamp */
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

        /* Push depth buffer */
        GstBuffer *depth_buf = gst_buffer_new_allocate(NULL, CUBEEYESRC_OUT_SIZE, NULL);
        gst_buffer_fill(depth_buf, 0, self->priv->depth_buffer, CUBEEYESRC_OUT_SIZE);
        GST_BUFFER_PTS(depth_buf) = pts;
        GST_BUFFER_DTS(depth_buf) = pts;
        GST_BUFFER_DURATION(depth_buf) = GST_SECOND / 15;

        GstFlowReturn ret = gst_pad_push(self->depth_pad, depth_buf);
        if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
            GST_WARNING_OBJECT(self, "Depth pad push failed: %s", gst_flow_get_name(ret));
        }

        /* Push amplitude buffer if enabled */
        if (extract_amp) {
            GstBuffer *amp_buf = gst_buffer_new_allocate(NULL, CUBEEYESRC_OUT_SIZE, NULL);
            gst_buffer_fill(amp_buf, 0, self->priv->amplitude_buffer, CUBEEYESRC_OUT_SIZE);
            GST_BUFFER_PTS(amp_buf) = pts;
            GST_BUFFER_DTS(amp_buf) = pts;
            GST_BUFFER_DURATION(amp_buf) = GST_SECOND / 15;

            ret = gst_pad_push(self->amplitude_pad, amp_buf);
            if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
                GST_WARNING_OBJECT(self, "Amplitude pad push failed: %s", gst_flow_get_name(ret));
            }
        }

        self->frame_count++;
    }

    GST_DEBUG_OBJECT(self, "Streaming thread stopped");
    return NULL;
}

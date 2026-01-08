/**
 * gstcubeeyesrc.cpp - CubeEye I200D ToF Camera GStreamer Source Element
 *
 * Combined V4L2 capture + depth extraction in a single element.
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
    uint8_t *raw_buffer;  /* Temp buffer for raw frame */
};

/* Properties */
enum {
    PROP_0,
    PROP_SERIAL,
    PROP_DEVICE,
    PROP_OUTPUT_TYPE,
    PROP_GRADIENT_CORRECTION,
    PROP_NORMALIZE,
    PROP_MAX_DEPTH,
};

#define DEFAULT_OUTPUT_TYPE     CUBEEYESRC_OUTPUT_DEPTH
#define DEFAULT_GRADIENT_CORR   TRUE
#define DEFAULT_NORMALIZE       FALSE
#define DEFAULT_MAX_DEPTH       5000

/* Output type enum registration */
#define GST_TYPE_CUBEEYESRC_OUTPUT_TYPE (gst_cubeeyesrc_output_type_get_type())
static GType gst_cubeeyesrc_output_type_get_type(void) {
    static GType type = 0;
    if (!type) {
        static const GEnumValue values[] = {
            { CUBEEYESRC_OUTPUT_DEPTH, "Output depth", "depth" },
            { CUBEEYESRC_OUTPUT_AMPLITUDE, "Output amplitude", "amplitude" },
            { 0, NULL, NULL }
        };
        type = g_enum_register_static("GstCubeEyeSrcOutputType", values);
    }
    return type;
}

/* Pad template */
static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src",
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

#define gst_cubeeyesrc_parent_class parent_class
G_DEFINE_TYPE_WITH_PRIVATE(GstCubeEyeSrc, gst_cubeeyesrc, GST_TYPE_PUSH_SRC);

/* Forward declarations */
static void gst_cubeeyesrc_set_property(GObject *object, guint prop_id,
                                         const GValue *value, GParamSpec *pspec);
static void gst_cubeeyesrc_get_property(GObject *object, guint prop_id,
                                         GValue *value, GParamSpec *pspec);
static void gst_cubeeyesrc_finalize(GObject *object);
static gboolean gst_cubeeyesrc_start(GstBaseSrc *src);
static gboolean gst_cubeeyesrc_stop(GstBaseSrc *src);
static GstCaps *gst_cubeeyesrc_get_caps(GstBaseSrc *src, GstCaps *filter);
static gboolean gst_cubeeyesrc_set_caps(GstBaseSrc *src, GstCaps *caps);
static GstFlowReturn gst_cubeeyesrc_create(GstPushSrc *src, GstBuffer **buf);

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
    GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS(klass);
    GstPushSrcClass *pushsrc_class = GST_PUSH_SRC_CLASS(klass);

    GST_DEBUG_CATEGORY_INIT(gst_cubeeyesrc_debug, "cubeeyesrc", 0,
                            "CubeEye I200D ToF Camera Source");

    gobject_class->set_property = gst_cubeeyesrc_set_property;
    gobject_class->get_property = gst_cubeeyesrc_get_property;
    gobject_class->finalize = gst_cubeeyesrc_finalize;

    /* Properties */
    g_object_class_install_property(gobject_class, PROP_SERIAL,
        g_param_spec_string("serial", "Serial Number",
            "Camera serial number (e.g., I200DU2427000032)",
            NULL, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_DEVICE,
        g_param_spec_string("device", "Device",
            "V4L2 device path (auto-detected from serial if not set)",
            NULL, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_OUTPUT_TYPE,
        g_param_spec_enum("output-type", "Output Type",
            "Type of output (depth or amplitude)",
            GST_TYPE_CUBEEYESRC_OUTPUT_TYPE, DEFAULT_OUTPUT_TYPE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_GRADIENT_CORRECTION,
        g_param_spec_boolean("gradient-correction", "Gradient Correction",
            "Apply polynomial gradient correction to depth",
            DEFAULT_GRADIENT_CORR, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_NORMALIZE,
        g_param_spec_boolean("normalize", "Normalize",
            "Scale output to full 16-bit range for display",
            DEFAULT_NORMALIZE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_MAX_DEPTH,
        g_param_spec_int("max-depth", "Max Depth",
            "Maximum depth in mm (for normalization)",
            100, 10000, DEFAULT_MAX_DEPTH,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    /* Element metadata */
    gst_element_class_set_static_metadata(element_class,
        "CubeEye ToF Camera Source",
        "Source/Video",
        "Captures depth/amplitude from CubeEye I200D ToF camera",
        "Atlas Robotics Inc.");

    gst_element_class_add_static_pad_template(element_class, &src_template);

    /* Virtual methods */
    basesrc_class->start = GST_DEBUG_FUNCPTR(gst_cubeeyesrc_start);
    basesrc_class->stop = GST_DEBUG_FUNCPTR(gst_cubeeyesrc_stop);
    basesrc_class->get_caps = GST_DEBUG_FUNCPTR(gst_cubeeyesrc_get_caps);
    basesrc_class->set_caps = GST_DEBUG_FUNCPTR(gst_cubeeyesrc_set_caps);
    pushsrc_class->create = GST_DEBUG_FUNCPTR(gst_cubeeyesrc_create);
}

static void gst_cubeeyesrc_init(GstCubeEyeSrc *self) {
    self->priv = (GstCubeEyeSrcPrivate *)
        gst_cubeeyesrc_get_instance_private(self);

    /* Default property values */
    self->serial = NULL;
    self->device = NULL;
    self->device_index = -1;
    self->output_type = DEFAULT_OUTPUT_TYPE;
    self->gradient_correction = DEFAULT_GRADIENT_CORR;
    self->normalize = DEFAULT_NORMALIZE;
    self->max_depth = DEFAULT_MAX_DEPTH;

    /* State */
    self->fd = -1;
    self->is_streaming = FALSE;
    self->num_buffers = 4;
    self->buffers = NULL;
    self->frame_count = 0;
    self->base_time = GST_CLOCK_TIME_NONE;

    /* Private */
    self->priv->cpu_extractor = NULL;
#ifdef HAVE_CUDA
    self->priv->cuda_extractor = NULL;
    self->priv->cuda_available = FALSE;
#endif
    self->priv->raw_buffer = NULL;

    /* Live source */
    gst_base_src_set_live(GST_BASE_SRC(self), TRUE);
    gst_base_src_set_format(GST_BASE_SRC(self), GST_FORMAT_TIME);
}

static void gst_cubeeyesrc_finalize(GObject *object) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(object);

    g_free(self->serial);
    g_free(self->device);

    if (self->priv->raw_buffer) {
        g_free(self->priv->raw_buffer);
    }

#ifdef HAVE_CUDA
    if (self->priv->cuda_extractor) {
        delete self->priv->cuda_extractor;
    }
#endif

    if (self->priv->cpu_extractor) {
        delete self->priv->cpu_extractor;
    }

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
        case PROP_OUTPUT_TYPE:
            self->output_type = (GstCubeEyeSrcOutputType)g_value_get_enum(value);
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
        case PROP_OUTPUT_TYPE:
            g_value_set_enum(value, self->output_type);
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
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static gboolean gst_cubeeyesrc_start(GstBaseSrc *src) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(src);
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
        /* Find first available CubeEye */
        auto devices = cubeeye::DeviceEnumerator::enumerate();
        if (devices.empty()) {
            GST_ERROR_OBJECT(self, "No CubeEye devices found");
            return FALSE;
        }
        device_path = devices[0].device_path;
        GST_INFO_OBJECT(self, "Using first available device: %s (serial: %s)",
                        device_path.c_str(), devices[0].serial_number.c_str());
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
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(self->fd, VIDIOC_STREAMON, &type) < 0) {
        GST_ERROR_OBJECT(self, "VIDIOC_STREAMON failed: %s", strerror(errno));
        goto error;
    }
    self->is_streaming = TRUE;

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

    self->frame_count = 0;
    self->base_time = GST_CLOCK_TIME_NONE;

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

static gboolean gst_cubeeyesrc_stop(GstBaseSrc *src) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(src);

    GST_DEBUG_OBJECT(self, "Stopping cubeeyesrc");

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

    return TRUE;
}

static GstCaps *gst_cubeeyesrc_get_caps(GstBaseSrc *src, GstCaps *filter) {
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "GRAY16_LE",
        "width", G_TYPE_INT, CUBEEYESRC_OUT_WIDTH,
        "height", G_TYPE_INT, CUBEEYESRC_OUT_HEIGHT,
        "framerate", GST_TYPE_FRACTION, 15, 1,
        NULL);

    if (filter) {
        GstCaps *tmp = gst_caps_intersect_full(filter, caps, GST_CAPS_INTERSECT_FIRST);
        gst_caps_unref(caps);
        caps = tmp;
    }

    return caps;
}

static gboolean gst_cubeeyesrc_set_caps(GstBaseSrc *src, GstCaps *caps) {
    GST_DEBUG_OBJECT(src, "Set caps: %" GST_PTR_FORMAT, caps);
    return TRUE;
}

static GstFlowReturn gst_cubeeyesrc_create(GstPushSrc *src, GstBuffer **buf) {
    GstCubeEyeSrc *self = GST_CUBEEYESRC(src);
    GstBuffer *outbuf;
    GstMapInfo map;

    while (TRUE) {
        /* Wait for frame */
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(self->fd, &fds);

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int r = select(self->fd + 1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            if (errno == EINTR) continue;
            GST_ERROR_OBJECT(self, "select error: %s", strerror(errno));
            return GST_FLOW_ERROR;
        }
        if (r == 0) {
            GST_ERROR_OBJECT(self, "select timeout");
            return GST_FLOW_ERROR;
        }

        /* Dequeue buffer */
        struct v4l2_buffer v4l2_buf = {};
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(self->fd, VIDIOC_DQBUF, &v4l2_buf) < 0) {
            if (errno == EAGAIN) continue;
            GST_ERROR_OBJECT(self, "VIDIOC_DQBUF failed: %s", strerror(errno));
            return GST_FLOW_ERROR;
        }

        /* Check for valid frame */
        if (v4l2_buf.bytesused == 0) {
            xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);
            continue;
        }

        /* Check for warmup frames (all zeros) */
        uint8_t *raw_data = (uint8_t *)self->buffers[v4l2_buf.index].start;
        gboolean has_data = FALSE;
        for (guint i = 0; i < 1000 && !has_data; i++) {
            if (raw_data[i] != 0) has_data = TRUE;
        }

        if (!has_data) {
            xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);
            continue;
        }

        /* Copy raw frame to temp buffer */
        memcpy(self->priv->raw_buffer, raw_data, CUBEEYESRC_RAW_SIZE);

        /* Re-queue V4L2 buffer immediately */
        xioctl(self->fd, VIDIOC_QBUF, &v4l2_buf);

        /* Allocate output buffer */
        outbuf = gst_buffer_new_allocate(NULL, CUBEEYESRC_OUT_SIZE, NULL);
        if (!outbuf) {
            GST_ERROR_OBJECT(self, "Failed to allocate buffer");
            return GST_FLOW_ERROR;
        }

        gst_buffer_map(outbuf, &map, GST_MAP_WRITE);

        /* Extract depth or amplitude */
        gboolean success = FALSE;

#ifdef HAVE_CUDA
        if (self->priv->cuda_available && self->priv->cuda_extractor) {
            if (self->output_type == CUBEEYESRC_OUTPUT_AMPLITUDE) {
                /* CUDA amplitude not implemented, fall back to CPU */
                success = self->priv->cpu_extractor->ExtractAmplitude(
                    self->priv->raw_buffer, CUBEEYESRC_RAW_SIZE,
                    (uint16_t *)map.data, TRUE);
            } else {
                success = self->priv->cuda_extractor->ExtractDepth(
                    self->priv->raw_buffer, CUBEEYESRC_RAW_SIZE,
                    (uint16_t *)map.data, TRUE);
            }
        } else
#endif
        {
            if (self->output_type == CUBEEYESRC_OUTPUT_AMPLITUDE) {
                success = self->priv->cpu_extractor->ExtractAmplitude(
                    self->priv->raw_buffer, CUBEEYESRC_RAW_SIZE,
                    (uint16_t *)map.data, TRUE);
            } else {
                success = self->priv->cpu_extractor->ExtractDepth(
                    self->priv->raw_buffer, CUBEEYESRC_RAW_SIZE,
                    (uint16_t *)map.data, TRUE);
            }
        }

        if (!success) {
            gst_buffer_unmap(outbuf, &map);
            gst_buffer_unref(outbuf);
            GST_ERROR_OBJECT(self, "Depth extraction failed");
            return GST_FLOW_ERROR;
        }

        /* Apply normalization for display */
        if (self->normalize) {
            uint16_t *data = (uint16_t *)map.data;
            gint num_pixels = CUBEEYESRC_OUT_WIDTH * CUBEEYESRC_OUT_HEIGHT;
            gint max_val = self->max_depth;

            for (gint i = 0; i < num_pixels; i++) {
                guint32 val = data[i];
                if (val > (guint32)max_val) val = max_val;
                data[i] = (uint16_t)((val * 65535) / max_val);
            }
        }

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
        GST_BUFFER_DURATION(outbuf) = GST_SECOND / 15;
        GST_BUFFER_OFFSET(outbuf) = self->frame_count;
        GST_BUFFER_OFFSET_END(outbuf) = self->frame_count + 1;

        self->frame_count++;
        *buf = outbuf;

        return GST_FLOW_OK;
    }
}

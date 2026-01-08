/**
 * gstcubeeye_depth.cpp - CubeEye CUDA Depth Extraction GStreamer Element
 *
 * Transforms raw CubeEye frames into depth images using CUDA acceleration.
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstcubeeye_depth.h"
#include "cubeeye_depth.h"

#ifdef HAVE_CUDA
#include "cubeeye_depth_cuda.h"
#endif

#include <gst/video/video.h>
#include <cstring>

GST_DEBUG_CATEGORY_STATIC(gst_cubeeye_depth_debug);
#define GST_CAT_DEFAULT gst_cubeeye_depth_debug

/* Private data structure */
struct _GstCubeEyeDepthPrivate {
#ifdef HAVE_CUDA
    cubeeye::cuda::CudaDepthExtractor *cuda_extractor;
    gboolean cuda_available;
#endif
    cubeeye::DepthExtractor *cpu_extractor;
    gboolean initialized;

    /* Output buffer for amplitude when both are enabled */
    uint16_t *amplitude_buffer;
};

/* Properties */
enum {
    PROP_0,
    PROP_ENABLE_DEPTH,
    PROP_ENABLE_AMPLITUDE,
    PROP_GRADIENT_CORRECTION,
    PROP_INTERPOLATE,
    PROP_MAX_DEPTH,
    PROP_NORMALIZE,
};

/* Pad templates */
static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE(
    "sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS(
        "video/x-raw, "
        "format = (string) GRAY8, "
        "width = (int) 3200, "
        "height = (int) 241, "
        "framerate = (fraction) [ 0/1, 30/1 ]"
    )
);

/* Output caps - support both 480 and 240 height */
static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS(
        "video/x-raw, "
        "format = (string) GRAY16_LE, "
        "width = (int) 640, "
        "height = (int) { 240, 480 }, "
        "framerate = (fraction) [ 0/1, 30/1 ]"
    )
);

#define gst_cubeeye_depth_parent_class parent_class
G_DEFINE_TYPE_WITH_PRIVATE(GstCubeEyeDepth, gst_cubeeye_depth, GST_TYPE_BASE_TRANSFORM);

/* Forward declarations */
static void gst_cubeeye_depth_set_property(GObject *object, guint prop_id,
                                            const GValue *value, GParamSpec *pspec);
static void gst_cubeeye_depth_get_property(GObject *object, guint prop_id,
                                            GValue *value, GParamSpec *pspec);
static void gst_cubeeye_depth_finalize(GObject *object);

static gboolean gst_cubeeye_depth_start(GstBaseTransform *trans);
static gboolean gst_cubeeye_depth_stop(GstBaseTransform *trans);
static GstCaps *gst_cubeeye_depth_transform_caps(GstBaseTransform *trans,
                                                   GstPadDirection direction,
                                                   GstCaps *caps, GstCaps *filter);
static gboolean gst_cubeeye_depth_set_caps(GstBaseTransform *trans,
                                            GstCaps *incaps, GstCaps *outcaps);
static GstFlowReturn gst_cubeeye_depth_transform(GstBaseTransform *trans,
                                                   GstBuffer *inbuf, GstBuffer *outbuf);
static gboolean gst_cubeeye_depth_get_unit_size(GstBaseTransform *trans,
                                                  GstCaps *caps, gsize *size);

static void gst_cubeeye_depth_class_init(GstCubeEyeDepthClass *klass) {
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
    GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS(klass);

    GST_DEBUG_CATEGORY_INIT(gst_cubeeye_depth_debug, "cubeeye_depth", 0,
                            "CubeEye CUDA Depth Extraction");

    gobject_class->set_property = gst_cubeeye_depth_set_property;
    gobject_class->get_property = gst_cubeeye_depth_get_property;
    gobject_class->finalize = gst_cubeeye_depth_finalize;

    /* Properties */
    g_object_class_install_property(gobject_class, PROP_ENABLE_DEPTH,
        g_param_spec_boolean("enable-depth", "Enable Depth",
            "Enable depth output (always true for now)",
            TRUE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_ENABLE_AMPLITUDE,
        g_param_spec_boolean("enable-amplitude", "Enable Amplitude",
            "Output amplitude instead of depth",
            FALSE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_GRADIENT_CORRECTION,
        g_param_spec_boolean("gradient-correction", "Gradient Correction",
            "Apply polynomial gradient correction to depth",
            TRUE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_INTERPOLATE,
        g_param_spec_boolean("interpolate", "Interpolate",
            "2x vertical interpolation (240->480)",
            TRUE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_MAX_DEPTH,
        g_param_spec_int("max-depth", "Max Depth",
            "Maximum depth in mm (for normalization)",
            100, 10000, 5000, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_NORMALIZE,
        g_param_spec_boolean("normalize", "Normalize",
            "Scale depth to full 16-bit range for display (0-max_depth -> 0-65535)",
            FALSE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    /* Element metadata */
    gst_element_class_set_static_metadata(element_class,
        "CubeEye Depth Extraction",
        "Filter/Video",
        "Extracts depth from CubeEye I200D raw frames using CUDA",
        "Atlas Robotics Inc.");

    /* Pad templates */
    gst_element_class_add_static_pad_template(element_class, &sink_template);
    gst_element_class_add_static_pad_template(element_class, &src_template);

    /* Transform class methods */
    trans_class->start = GST_DEBUG_FUNCPTR(gst_cubeeye_depth_start);
    trans_class->stop = GST_DEBUG_FUNCPTR(gst_cubeeye_depth_stop);
    trans_class->transform_caps = GST_DEBUG_FUNCPTR(gst_cubeeye_depth_transform_caps);
    trans_class->set_caps = GST_DEBUG_FUNCPTR(gst_cubeeye_depth_set_caps);
    trans_class->transform = GST_DEBUG_FUNCPTR(gst_cubeeye_depth_transform);
    trans_class->get_unit_size = GST_DEBUG_FUNCPTR(gst_cubeeye_depth_get_unit_size);

    /* We change the format, so not passthrough */
    trans_class->passthrough_on_same_caps = FALSE;
}

static void gst_cubeeye_depth_init(GstCubeEyeDepth *self) {
    self->priv = (GstCubeEyeDepthPrivate *)
        gst_cubeeye_depth_get_instance_private(self);

    /* Default property values */
    self->enable_depth = TRUE;
    self->enable_amplitude = FALSE;
    self->gradient_correction = TRUE;
    self->interpolate = TRUE;
    self->max_depth = 5000;
    self->normalize = FALSE;
    self->output_width = CUBEEYE_DEPTH_OUT_WIDTH;
    self->output_height = CUBEEYE_DEPTH_OUT_HEIGHT_480;

    /* Private state */
    self->priv->initialized = FALSE;
#ifdef HAVE_CUDA
    self->priv->cuda_extractor = NULL;
    self->priv->cuda_available = FALSE;
#endif
    self->priv->cpu_extractor = NULL;
    self->priv->amplitude_buffer = NULL;
}

static void gst_cubeeye_depth_finalize(GObject *object) {
    GstCubeEyeDepth *self = GST_CUBEEYE_DEPTH(object);

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

    if (self->priv->amplitude_buffer) {
        g_free(self->priv->amplitude_buffer);
        self->priv->amplitude_buffer = NULL;
    }

    G_OBJECT_CLASS(parent_class)->finalize(object);
}

static void gst_cubeeye_depth_set_property(GObject *object, guint prop_id,
                                            const GValue *value, GParamSpec *pspec) {
    GstCubeEyeDepth *self = GST_CUBEEYE_DEPTH(object);

    switch (prop_id) {
        case PROP_ENABLE_DEPTH:
            self->enable_depth = g_value_get_boolean(value);
            break;
        case PROP_ENABLE_AMPLITUDE:
            self->enable_amplitude = g_value_get_boolean(value);
            break;
        case PROP_GRADIENT_CORRECTION:
            self->gradient_correction = g_value_get_boolean(value);
            /* Update CPU extractor if already initialized */
            if (self->priv->cpu_extractor) {
                self->priv->cpu_extractor->SetGradientCorrection(self->gradient_correction);
            }
            break;
        case PROP_INTERPOLATE:
            self->interpolate = g_value_get_boolean(value);
            self->output_height = self->interpolate ?
                CUBEEYE_DEPTH_OUT_HEIGHT_480 : CUBEEYE_DEPTH_OUT_HEIGHT_240;
            break;
        case PROP_MAX_DEPTH:
            self->max_depth = g_value_get_int(value);
            break;
        case PROP_NORMALIZE:
            self->normalize = g_value_get_boolean(value);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static void gst_cubeeye_depth_get_property(GObject *object, guint prop_id,
                                            GValue *value, GParamSpec *pspec) {
    GstCubeEyeDepth *self = GST_CUBEEYE_DEPTH(object);

    switch (prop_id) {
        case PROP_ENABLE_DEPTH:
            g_value_set_boolean(value, self->enable_depth);
            break;
        case PROP_ENABLE_AMPLITUDE:
            g_value_set_boolean(value, self->enable_amplitude);
            break;
        case PROP_GRADIENT_CORRECTION:
            g_value_set_boolean(value, self->gradient_correction);
            break;
        case PROP_INTERPOLATE:
            g_value_set_boolean(value, self->interpolate);
            break;
        case PROP_MAX_DEPTH:
            g_value_set_int(value, self->max_depth);
            break;
        case PROP_NORMALIZE:
            g_value_set_boolean(value, self->normalize);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static gboolean gst_cubeeye_depth_start(GstBaseTransform *trans) {
    GstCubeEyeDepth *self = GST_CUBEEYE_DEPTH(trans);

    GST_DEBUG_OBJECT(self, "Starting depth extraction");

#ifdef HAVE_CUDA
    /* Try CUDA first */
    if (cubeeye::cuda::CudaDepthExtractor::IsCudaAvailable()) {
        try {
            self->priv->cuda_extractor = new cubeeye::cuda::CudaDepthExtractor(
                self->gradient_correction, true);
            self->priv->cuda_available = TRUE;
            GST_INFO_OBJECT(self, "CUDA depth extraction enabled: %s",
                           cubeeye::cuda::CudaDepthExtractor::GetDeviceInfo());
        } catch (const std::exception &e) {
            GST_WARNING_OBJECT(self, "CUDA init failed: %s, falling back to CPU", e.what());
            self->priv->cuda_available = FALSE;
        }
    } else {
        GST_INFO_OBJECT(self, "CUDA not available, using CPU extraction");
        self->priv->cuda_available = FALSE;
    }
#endif

    /* Always create CPU extractor as fallback */
    self->priv->cpu_extractor = new cubeeye::DepthExtractor(self->gradient_correction);

    self->priv->initialized = TRUE;

    return TRUE;
}

static gboolean gst_cubeeye_depth_stop(GstBaseTransform *trans) {
    GstCubeEyeDepth *self = GST_CUBEEYE_DEPTH(trans);

    GST_DEBUG_OBJECT(self, "Stopping depth extraction");

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

    self->priv->initialized = FALSE;

    return TRUE;
}

static GstCaps *gst_cubeeye_depth_transform_caps(GstBaseTransform *trans,
                                                   GstPadDirection direction,
                                                   GstCaps *caps, GstCaps *filter) {
    GstCubeEyeDepth *self = GST_CUBEEYE_DEPTH(trans);
    GstCaps *result = NULL;

    if (direction == GST_PAD_SINK) {
        /* Transform sink caps to src caps */
        gint height = self->interpolate ?
            CUBEEYE_DEPTH_OUT_HEIGHT_480 : CUBEEYE_DEPTH_OUT_HEIGHT_240;

        result = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "GRAY16_LE",
            "width", G_TYPE_INT, CUBEEYE_DEPTH_OUT_WIDTH,
            "height", G_TYPE_INT, height,
            NULL);

        /* Copy framerate from input if present */
        GstStructure *s = gst_caps_get_structure(caps, 0);
        const GValue *framerate = gst_structure_get_value(s, "framerate");
        if (framerate) {
            gst_caps_set_value(result, "framerate", framerate);
        }
    } else {
        /* Transform src caps to sink caps */
        result = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "GRAY8",
            "width", G_TYPE_INT, CUBEEYE_DEPTH_RAW_WIDTH,
            "height", G_TYPE_INT, CUBEEYE_DEPTH_RAW_HEIGHT,
            NULL);

        GstStructure *s = gst_caps_get_structure(caps, 0);
        const GValue *framerate = gst_structure_get_value(s, "framerate");
        if (framerate) {
            gst_caps_set_value(result, "framerate", framerate);
        }
    }

    if (filter) {
        GstCaps *tmp = gst_caps_intersect_full(filter, result, GST_CAPS_INTERSECT_FIRST);
        gst_caps_unref(result);
        result = tmp;
    }

    GST_DEBUG_OBJECT(self, "Transform caps %s: %" GST_PTR_FORMAT " -> %" GST_PTR_FORMAT,
                     direction == GST_PAD_SINK ? "sink->src" : "src->sink",
                     caps, result);

    return result;
}

static gboolean gst_cubeeye_depth_set_caps(GstBaseTransform *trans,
                                            GstCaps *incaps, GstCaps *outcaps) {
    GstCubeEyeDepth *self = GST_CUBEEYE_DEPTH(trans);

    GST_DEBUG_OBJECT(self, "Set caps: in=%" GST_PTR_FORMAT " out=%" GST_PTR_FORMAT,
                     incaps, outcaps);

    /* Extract output dimensions from outcaps */
    GstStructure *s = gst_caps_get_structure(outcaps, 0);
    gst_structure_get_int(s, "width", &self->output_width);
    gst_structure_get_int(s, "height", &self->output_height);

    GST_INFO_OBJECT(self, "Output dimensions: %dx%d",
                    self->output_width, self->output_height);

    return TRUE;
}

static gboolean gst_cubeeye_depth_get_unit_size(GstBaseTransform *trans,
                                                  GstCaps *caps, gsize *size) {
    GstStructure *s = gst_caps_get_structure(caps, 0);
    gint width, height;
    const gchar *format;

    format = gst_structure_get_string(s, "format");
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    if (g_strcmp0(format, "GRAY8") == 0) {
        *size = width * height * 1;
    } else if (g_strcmp0(format, "GRAY16_LE") == 0) {
        *size = width * height * 2;
    } else {
        return FALSE;
    }

    return TRUE;
}

static GstFlowReturn gst_cubeeye_depth_transform(GstBaseTransform *trans,
                                                   GstBuffer *inbuf, GstBuffer *outbuf) {
    GstCubeEyeDepth *self = GST_CUBEEYE_DEPTH(trans);
    GstMapInfo in_map, out_map;
    gboolean success = FALSE;

    if (!self->priv->initialized) {
        GST_ERROR_OBJECT(self, "Not initialized");
        return GST_FLOW_ERROR;
    }

    if (!gst_buffer_map(inbuf, &in_map, GST_MAP_READ)) {
        GST_ERROR_OBJECT(self, "Failed to map input buffer");
        return GST_FLOW_ERROR;
    }

    if (!gst_buffer_map(outbuf, &out_map, GST_MAP_WRITE)) {
        GST_ERROR_OBJECT(self, "Failed to map output buffer");
        gst_buffer_unmap(inbuf, &in_map);
        return GST_FLOW_ERROR;
    }

    /* Validate input size */
    if (in_map.size < CUBEEYE_DEPTH_RAW_SIZE) {
        GST_ERROR_OBJECT(self, "Input buffer too small: %zu < %d",
                         in_map.size, CUBEEYE_DEPTH_RAW_SIZE);
        gst_buffer_unmap(outbuf, &out_map);
        gst_buffer_unmap(inbuf, &in_map);
        return GST_FLOW_ERROR;
    }

    gboolean interpolate = (self->output_height == CUBEEYE_DEPTH_OUT_HEIGHT_480);

#ifdef HAVE_CUDA
    if (self->priv->cuda_available && self->priv->cuda_extractor) {
        /* Use CUDA extraction */
        if (self->enable_amplitude && !self->enable_depth) {
            /* Amplitude only - not yet supported by CUDA, fall back to CPU */
            success = self->priv->cpu_extractor->ExtractAmplitude(
                in_map.data, in_map.size,
                (uint16_t *)out_map.data, interpolate);
        } else {
            success = self->priv->cuda_extractor->ExtractDepth(
                in_map.data, in_map.size,
                (uint16_t *)out_map.data, interpolate);
        }

        if (success) {
            GST_LOG_OBJECT(self, "CUDA extraction: %.2f ms",
                          self->priv->cuda_extractor->GetLastTotalTimeMs());
        }
    } else
#endif
    {
        /* CPU fallback */
        if (self->enable_amplitude && !self->enable_depth) {
            success = self->priv->cpu_extractor->ExtractAmplitude(
                in_map.data, in_map.size,
                (uint16_t *)out_map.data, interpolate);
        } else {
            success = self->priv->cpu_extractor->ExtractDepth(
                in_map.data, in_map.size,
                (uint16_t *)out_map.data, interpolate);
        }
    }

    /* Apply normalization if enabled (scale 0-max_depth to 0-65535 for display) */
    if (success && self->normalize) {
        uint16_t *depth_data = (uint16_t *)out_map.data;
        gint num_pixels = self->output_width * self->output_height;
        gint max_depth = self->max_depth;

        for (gint i = 0; i < num_pixels; i++) {
            guint32 val = depth_data[i];
            if (val > (guint32)max_depth) val = max_depth;
            depth_data[i] = (uint16_t)((val * 65535) / max_depth);
        }
    }

    gst_buffer_unmap(outbuf, &out_map);
    gst_buffer_unmap(inbuf, &in_map);

    if (!success) {
        GST_ERROR_OBJECT(self, "Depth extraction failed");
        return GST_FLOW_ERROR;
    }

    /* Copy timestamps */
    GST_BUFFER_PTS(outbuf) = GST_BUFFER_PTS(inbuf);
    GST_BUFFER_DTS(outbuf) = GST_BUFFER_DTS(inbuf);
    GST_BUFFER_DURATION(outbuf) = GST_BUFFER_DURATION(inbuf);

    return GST_FLOW_OK;
}

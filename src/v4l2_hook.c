/**
 * v4l2_hook.c - LD_PRELOAD library to intercept V4L2 reads
 *
 * Hooks into V4L2 DQBUF ioctl to capture raw frames before SDK processes them.
 *
 * Build:
 *   gcc -shared -fPIC -o libv4l2_hook.so v4l2_hook.c -ldl
 *
 * Usage:
 *   LD_PRELOAD=./libv4l2_hook.so ./sdk_capture
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <pthread.h>

// UVC XU control query ioctl and constants
#ifndef UVCIOC_CTRL_QUERY
struct uvc_xu_control_query {
    __u8 unit;
    __u8 selector;
    __u8 query;
    __u16 size;
    __u8 *data;
};
#define UVCIOC_CTRL_QUERY _IOWR('u', 0x21, struct uvc_xu_control_query)
#endif

// UVC query types
#define UVC_SET_CUR  0x01
#define UVC_GET_CUR  0x81
#define UVC_GET_MIN  0x82
#define UVC_GET_MAX  0x83
#define UVC_GET_RES  0x84
#define UVC_GET_LEN  0x85
#define UVC_GET_INFO 0x86
#define UVC_GET_DEF  0x87

// Frame dimensions for CubeEye I200D raw data
#define RAW_WIDTH  1600
#define RAW_HEIGHT 241
#define RAW_BYTES_PER_PIXEL 2
#define RAW_FRAME_SIZE (RAW_WIDTH * RAW_HEIGHT * RAW_BYTES_PER_PIXEL)

// How many frames to save (increase for longer benchmarks)
#define MAX_SAVED_FRAMES 200

// State
static int g_frame_count = 0;
static int g_video_fds[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int g_num_video_fds = 0;
static void* g_buffer_ptrs[8] = {0};
static size_t g_buffer_lens[8] = {0};
static int g_num_buffers = 0;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static const char* g_output_dir = "data";
static int g_hook_enabled = 1;

// Check if fd is a tracked video fd
static int is_video_fd(int fd) {
    for (int i = 0; i < g_num_video_fds; i++) {
        if (g_video_fds[i] == fd) return 1;
    }
    return 0;
}

// Add fd to tracked video fds
static void add_video_fd(int fd) {
    for (int i = 0; i < g_num_video_fds; i++) {
        if (g_video_fds[i] == fd) return; // Already tracked
    }
    if (g_num_video_fds < 8) {
        g_video_fds[g_num_video_fds++] = fd;
    }
}

// Original function pointers
static int (*real_ioctl)(int fd, unsigned long request, ...) = NULL;
static void* (*real_mmap)(void *addr, size_t length, int prot, int flags, int fd, off_t offset) = NULL;
static FILE* (*real_fopen)(const char* path, const char* mode) = NULL;
static int (*real_open)(const char* path, int flags, ...) = NULL;
static ssize_t (*real_read)(int fd, void* buf, size_t count) = NULL;

// Track file descriptors for FPPN files
static int g_fppn_fd = -1;
static char g_fppn_path[512] = {0};

// Initialize hooks
static void __attribute__((constructor)) init_hooks(void) {
    real_ioctl = dlsym(RTLD_NEXT, "ioctl");
    real_mmap = dlsym(RTLD_NEXT, "mmap");
    real_fopen = dlsym(RTLD_NEXT, "fopen");
    real_open = dlsym(RTLD_NEXT, "open");
    real_read = dlsym(RTLD_NEXT, "read");

    // Check env for output directory
    const char* out = getenv("V4L2_HOOK_OUTPUT");
    if (out) g_output_dir = out;

    // Check if hook should be disabled
    const char* disable = getenv("V4L2_HOOK_DISABLE");
    if (disable && disable[0] == '1') g_hook_enabled = 0;

    fprintf(stderr, "[V4L2_HOOK] Initialized, output=%s, enabled=%d\n",
            g_output_dir, g_hook_enabled);
}

// Save raw frame to file
static void save_raw_frame(const void* data, size_t size, int frame_num) {
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/raw_%04d.raw",
             g_output_dir, frame_num);

    FILE* f = fopen(filename, "wb");
    if (f) {
        fwrite(data, 1, size, f);
        fclose(f);
        fprintf(stderr, "[V4L2_HOOK] Saved raw frame %d (%zu bytes)\n",
                frame_num, size);
    } else {
        fprintf(stderr, "[V4L2_HOOK] Failed to save frame %d to %s\n",
                frame_num, filename);
    }
}

// Analyze frame header and first few pixels
static void analyze_frame(const uint16_t* data, size_t size, int frame_num) {
    if (size < 32) return;

    fprintf(stderr, "[V4L2_HOOK] Frame %d: header = %04x %04x %04x %04x ... ",
            frame_num, data[0], data[1], data[2], data[3]);

    // Check if frame has actual data (not zeros)
    int non_zero = 0;
    for (size_t i = 0; i < size/2 && i < 1000; i++) {
        if (data[i] != 0) non_zero++;
    }
    fprintf(stderr, "non_zero pixels: %d/1000\n", non_zero);
}

// Hooked mmap - track buffer mappings
void* mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset) {
    void* result = real_mmap(addr, length, prot, flags, fd, offset);

    // Track buffers that look like V4L2 buffers
    if (result != MAP_FAILED && length >= RAW_FRAME_SIZE/2) {
        pthread_mutex_lock(&g_mutex);
        if (g_num_buffers < 8) {
            g_buffer_ptrs[g_num_buffers] = result;
            g_buffer_lens[g_num_buffers] = length;
            g_num_buffers++;
            fprintf(stderr, "[V4L2_HOOK] Tracked mmap buffer %d: ptr=%p, len=%zu\n",
                    g_num_buffers-1, result, length);
        }
        pthread_mutex_unlock(&g_mutex);
    }

    return result;
}

// Log counter for rate limiting
static int g_ioctl_count = 0;

// Hooked ioctl - intercept V4L2 operations
int ioctl(int fd, unsigned long request, ...) {
    va_list args;
    va_start(args, request);
    void* arg = va_arg(args, void*);
    va_end(args);

    // Convert to 32-bit for V4L2 ioctl comparison (fix sign extension issue)
    unsigned int req32 = (unsigned int)request;

    int result = real_ioctl(fd, request, arg);

    if (!g_hook_enabled) return result;

    // Track all video file descriptors via QUERYCAP
    if (req32 == VIDIOC_QUERYCAP && result == 0) {
        struct v4l2_capability* cap = (struct v4l2_capability*)arg;
        fprintf(stderr, "[V4L2_HOOK] QUERYCAP fd=%d: card='%s', driver='%s'\n",
                fd, cap->card, cap->driver);
        // Track any UVC video capture device
        if ((cap->device_caps & V4L2_CAP_VIDEO_CAPTURE) &&
            (strstr((char*)cap->card, "Cube Eye") != NULL ||
             strstr((char*)cap->card, "CubeEye") != NULL)) {
            add_video_fd(fd);
            fprintf(stderr, "[V4L2_HOOK] Tracking video fd=%d (total=%d)\n", fd, g_num_video_fds);
        }
    }

    // Also track fd if we see STREAMON (in case we missed QUERYCAP)
    if (req32 == VIDIOC_STREAMON && result == 0) {
        add_video_fd(fd);
        fprintf(stderr, "[V4L2_HOOK] STREAMON on fd=%d, now tracking\n", fd);
    }

    // Log stream format configuration
    if (req32 == VIDIOC_S_FMT && result == 0) {
        struct v4l2_format* fmt = (struct v4l2_format*)arg;
        if (fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            fprintf(stderr, "[V4L2_HOOK] S_FMT: %dx%d, pixfmt=0x%08x\n",
                    fmt->fmt.pix.width, fmt->fmt.pix.height, fmt->fmt.pix.pixelformat);
        }
    }

    // Log V4L2 control changes
    if (req32 == VIDIOC_S_CTRL) {
        struct v4l2_control* ctrl = (struct v4l2_control*)arg;
        fprintf(stderr, "[V4L2_HOOK] S_CTRL: id=0x%08x, value=%d (result=%d)\n",
                ctrl->id, ctrl->value, result);
    }

    // Log extended controls
    if (req32 == VIDIOC_S_EXT_CTRLS) {
        struct v4l2_ext_controls* ctrls = (struct v4l2_ext_controls*)arg;
        fprintf(stderr, "[V4L2_HOOK] S_EXT_CTRLS: class=0x%08x, count=%d (result=%d)\n",
                ctrls->ctrl_class, ctrls->count, result);
        for (unsigned int i = 0; i < ctrls->count && i < 5; i++) {
            fprintf(stderr, "  [%d] id=0x%08x, size=%d\n",
                    i, ctrls->controls[i].id, ctrls->controls[i].size);
        }
    }

    // Log UVC XU (extension unit) controls - common for ToF sensors
    if (req32 == VIDIOC_G_EXT_CTRLS) {
        struct v4l2_ext_controls* ctrls = (struct v4l2_ext_controls*)arg;
        if (result == 0 && ctrls->count > 0) {
            fprintf(stderr, "[V4L2_HOOK] G_EXT_CTRLS: class=0x%08x, count=%d\n",
                    ctrls->ctrl_class, ctrls->count);
        }
    }

    // Log UVC Extension Unit Control Queries (UVCIOC_CTRL_QUERY)
    // This is how ToF sensors are typically configured
    if (req32 == (unsigned int)UVCIOC_CTRL_QUERY) {
        struct uvc_xu_control_query* xu = (struct uvc_xu_control_query*)arg;
        const char* query_name = "UNKNOWN";
        switch (xu->query) {
            case UVC_SET_CUR: query_name = "SET_CUR"; break;
            case UVC_GET_CUR: query_name = "GET_CUR"; break;
            case UVC_GET_MIN: query_name = "GET_MIN"; break;
            case UVC_GET_MAX: query_name = "GET_MAX"; break;
            case UVC_GET_RES: query_name = "GET_RES"; break;
            case UVC_GET_LEN: query_name = "GET_LEN"; break;
            case UVC_GET_INFO: query_name = "GET_INFO"; break;
            case UVC_GET_DEF: query_name = "GET_DEF"; break;
        }
        fprintf(stderr, "[V4L2_HOOK] UVC_XU: unit=%d, selector=%d, query=%s(0x%02x), size=%d, result=%d\n",
                xu->unit, xu->selector, query_name, xu->query, xu->size, result);

        // Dump the data payload for SET_CUR commands (sensor configuration)
        if (xu->query == UVC_SET_CUR && xu->data && xu->size > 0) {
            fprintf(stderr, "[V4L2_HOOK]   SET_CUR DATA: ");
            for (int i = 0; i < xu->size && i < 32; i++) {
                fprintf(stderr, "%02x ", xu->data[i]);
            }
            if (xu->size > 32) fprintf(stderr, "...");
            fprintf(stderr, "\n");

            // Save selector 4 SET_CUR data to understand page requests
            if (xu->selector == 4 && xu->size >= 4) {
                static int set_dump_count = 0;
                char filename[256];
                snprintf(filename, sizeof(filename), "%s/set_cur_%03d_sel%d.bin",
                         g_output_dir, set_dump_count++, xu->selector);
                FILE* f = fopen(filename, "wb");
                if (f) {
                    fwrite(xu->data, 1, xu->size, f);
                    fclose(f);
                }
            }
        }

        // CRITICAL: Also capture GET_CUR responses for calibration data (Selector 4)
        if (xu->query == UVC_GET_CUR && result == 0 && xu->data && xu->size > 0) {
            // Log first 64 bytes
            fprintf(stderr, "[V4L2_HOOK]   GET_CUR DATA: ");
            for (int i = 0; i < xu->size && i < 64; i++) {
                fprintf(stderr, "%02x ", xu->data[i]);
            }
            if (xu->size > 64) fprintf(stderr, "... (%d more)", xu->size - 64);
            fprintf(stderr, "\n");

            // Save Selector 4 calibration data to file
            if (xu->selector == 4 && xu->size > 32) {
                static int cal_dump_count = 0;
                char filename[256];
                snprintf(filename, sizeof(filename), "%s/calibration_%03d_sel%d.bin",
                         g_output_dir, cal_dump_count++, xu->selector);
                FILE* f = fopen(filename, "wb");
                if (f) {
                    fwrite(xu->data, 1, xu->size, f);
                    fclose(f);
                    fprintf(stderr, "[V4L2_HOOK]   Saved calibration data to %s\n", filename);
                }
            }
        }
    }

    // Intercept DQBUF to capture raw frame data
    if (req32 == VIDIOC_DQBUF && result == 0) {
        struct v4l2_buffer* buf = (struct v4l2_buffer*)arg;

        // Auto-track this fd if we see a valid DQBUF
        if (buf->bytesused > 0) {
            add_video_fd(fd);
        }

        // Log DQBUF for debugging
        if (g_frame_count < MAX_SAVED_FRAMES * 2) {
            fprintf(stderr, "[V4L2_HOOK] DQBUF fd=%d idx=%d bytesused=%d\n",
                    fd, buf->index, buf->bytesused);
        }
    }

    // Capture raw frame data from DQBUF
    if (req32 == VIDIOC_DQBUF && result == 0) {
        struct v4l2_buffer* buf = (struct v4l2_buffer*)arg;

        pthread_mutex_lock(&g_mutex);

        if (buf->index < g_num_buffers && g_frame_count < MAX_SAVED_FRAMES) {
            void* frame_data = g_buffer_ptrs[buf->index];
            size_t frame_size = buf->bytesused > 0 ? buf->bytesused : g_buffer_lens[buf->index];

            if (frame_data && frame_size > 0) {
                // Analyze the frame
                analyze_frame((uint16_t*)frame_data, frame_size, g_frame_count);

                // Save the frame
                save_raw_frame(frame_data, frame_size, g_frame_count);
                g_frame_count++;
            }
        }

        pthread_mutex_unlock(&g_mutex);
    }

    return result;
}

// Check if path is interesting (calibration/fppn related)
static int is_interesting_path(const char* path) {
    if (!path) return 0;
    return (strstr(path, "fppn") != NULL ||
            strstr(path, "FPPN") != NULL ||
            strstr(path, "FPPNData") != NULL ||
            strstr(path, "calibration") != NULL ||
            strstr(path, "calstore") != NULL ||
            strstr(path, ".cal") != NULL ||
            strstr(path, "I200") != NULL);
}

// Hooked fopen - intercept file opens
FILE* fopen(const char* path, const char* mode) {
    FILE* result = real_fopen(path, mode);

    if (g_hook_enabled && path) {
        // Log all file opens for debugging
        if (is_interesting_path(path)) {
            fprintf(stderr, "[FILE_HOOK] fopen('%s', '%s') = %p\n",
                    path, mode, (void*)result);

            // If this is an FPPN file that was successfully opened, dump it
            if (result && (strstr(path, "fppn") || strstr(path, "FPPN"))) {
                fprintf(stderr, "[FILE_HOOK] *** FPPN FILE FOUND: %s ***\n", path);

                // Read and save the file contents
                fseek(result, 0, SEEK_END);
                long size = ftell(result);
                fseek(result, 0, SEEK_SET);

                if (size > 0 && size < 1024*1024) {  // Max 1MB
                    void* data = malloc(size);
                    if (data) {
                        size_t read_size = fread(data, 1, size, result);
                        fseek(result, 0, SEEK_SET);  // Reset for SDK to read

                        // Save to output directory
                        char filename[512];
                        const char* basename = strrchr(path, '/');
                        basename = basename ? basename + 1 : path;
                        snprintf(filename, sizeof(filename), "%s/captured_%s.bin",
                                 g_output_dir, basename);

                        FILE* out = real_fopen(filename, "wb");
                        if (out) {
                            fwrite(data, 1, read_size, out);
                            fclose(out);
                            fprintf(stderr, "[FILE_HOOK] Saved FPPN data to %s (%ld bytes)\n",
                                    filename, size);
                        }
                        free(data);
                    }
                }
            }
        }
    }

    return result;
}

// Hooked open - intercept file opens (some libs use open instead of fopen)
int open(const char* path, int flags, ...) {
    va_list args;
    va_start(args, flags);
    mode_t mode = 0;
    if (flags & O_CREAT) {
        mode = va_arg(args, mode_t);
    }
    va_end(args);

    int result = real_open(path, flags, mode);

    if (g_hook_enabled && path && is_interesting_path(path)) {
        fprintf(stderr, "[FILE_HOOK] open('%s', 0x%x) = %d\n", path, flags, result);

        // Track FPPN file descriptors
        if (result >= 0 && (strstr(path, "fppn") || strstr(path, "FPPN"))) {
            g_fppn_fd = result;
            strncpy(g_fppn_path, path, sizeof(g_fppn_path) - 1);
            fprintf(stderr, "[FILE_HOOK] *** Tracking FPPN fd=%d: %s ***\n", result, path);
        }
    }

    return result;
}

// Hooked read - capture FPPN data when read
ssize_t read(int fd, void* buf, size_t count) {
    ssize_t result = real_read(fd, buf, count);

    if (g_hook_enabled && fd == g_fppn_fd && fd >= 0 && result > 0) {
        fprintf(stderr, "[FILE_HOOK] read(fd=%d, count=%zu) = %zd (FPPN data)\n",
                fd, count, result);

        // Save the data
        static int fppn_chunk = 0;
        char filename[512];
        snprintf(filename, sizeof(filename), "%s/fppn_chunk_%03d.bin",
                 g_output_dir, fppn_chunk++);

        FILE* out = real_fopen(filename, "wb");
        if (out) {
            fwrite(buf, 1, result, out);
            fclose(out);
            fprintf(stderr, "[FILE_HOOK] Saved FPPN chunk to %s\n", filename);
        }

        // Clear tracking after first read
        g_fppn_fd = -1;
    }

    return result;
}

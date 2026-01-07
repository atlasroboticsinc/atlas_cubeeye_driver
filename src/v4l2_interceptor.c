/**
 * v4l2_interceptor.c - Intercept V4L2 ioctl calls to capture raw frames
 *
 * Compile: gcc -shared -fPIC -o libv4l2_interceptor.so v4l2_interceptor.c -ldl
 * Usage: LD_PRELOAD=./libv4l2_interceptor.so ./build/sdk_capture 5
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

// Original function pointers
static int (*orig_ioctl)(int fd, unsigned long request, ...) = NULL;

// Capture state
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_frame_count = 0;
static int g_max_frames = 5;
static char g_output_dir[256] = "data/ioctl_capture";
static int g_video_fd = -1;

// Buffer tracking for mmap'd buffers
#define MAX_BUFFERS 8
static struct {
    void* addr;
    size_t length;
    int used;
} g_buffers[MAX_BUFFERS];

__attribute__((constructor))
static void init(void) {
    orig_ioctl = dlsym(RTLD_NEXT, "ioctl");
    if (!orig_ioctl) {
        fprintf(stderr, "[V4L2_INTERCEPT] ERROR: Failed to find ioctl\n");
        return;
    }

    char* env_dir = getenv("V4L2_CAPTURE_DIR");
    if (env_dir) strncpy(g_output_dir, env_dir, sizeof(g_output_dir)-1);

    char* env_frames = getenv("V4L2_CAPTURE_FRAMES");
    if (env_frames) g_max_frames = atoi(env_frames);

    char cmd[512];
    snprintf(cmd, sizeof(cmd), "mkdir -p %s", g_output_dir);
    system(cmd);

    fprintf(stderr, "[V4L2_INTERCEPT] Initialized - capturing to %s (max %d frames)\n",
            g_output_dir, g_max_frames);
}

__attribute__((destructor))
static void cleanup(void) {
    fprintf(stderr, "[V4L2_INTERCEPT] Captured %d frames\n", g_frame_count);
}

static void save_frame_data(int buffer_index) {
    pthread_mutex_lock(&g_mutex);

    if (g_frame_count < g_max_frames && buffer_index >= 0 && buffer_index < MAX_BUFFERS) {
        if (g_buffers[buffer_index].used && g_buffers[buffer_index].addr) {
            char filename[512];
            snprintf(filename, sizeof(filename), "%s/v4l2_raw_%04d.raw",
                     g_output_dir, g_frame_count);

            FILE* f = fopen(filename, "wb");
            if (f) {
                fwrite(g_buffers[buffer_index].addr, 1, g_buffers[buffer_index].length, f);
                fclose(f);
                fprintf(stderr, "[V4L2_INTERCEPT] Frame %d: saved %zu bytes to %s\n",
                        g_frame_count, g_buffers[buffer_index].length, filename);
            }
            g_frame_count++;
        }
    }

    pthread_mutex_unlock(&g_mutex);
}

// Also intercept mmap to track buffer addresses
void* mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset) {
    static void* (*orig_mmap)(void*, size_t, int, int, int, off_t) = NULL;
    if (!orig_mmap) orig_mmap = dlsym(RTLD_NEXT, "mmap");

    void* result = orig_mmap(addr, length, prot, flags, fd, offset);

    // Track video device mmaps
    if (result != MAP_FAILED && fd == g_video_fd && length > 100000) {
        pthread_mutex_lock(&g_mutex);
        for (int i = 0; i < MAX_BUFFERS; i++) {
            if (!g_buffers[i].used) {
                g_buffers[i].addr = result;
                g_buffers[i].length = length;
                g_buffers[i].used = 1;
                fprintf(stderr, "[V4L2_INTERCEPT] Mapped buffer %d: %p, %zu bytes\n",
                        i, result, length);
                break;
            }
        }
        pthread_mutex_unlock(&g_mutex);
    }

    return result;
}

int ioctl(int fd, unsigned long request, ...) {
    va_list args;
    va_start(args, request);
    void* arg = va_arg(args, void*);
    va_end(args);

    // Track video device opens
    if (request == VIDIOC_QUERYCAP) {
        g_video_fd = fd;
        fprintf(stderr, "[V4L2_INTERCEPT] Video device opened: fd=%d\n", fd);
    }

    // Call original
    int result = orig_ioctl(fd, request, arg);

    // Intercept DQBUF (dequeue buffer) - this is when a frame is ready
    if (fd == g_video_fd && request == VIDIOC_DQBUF && result == 0) {
        struct v4l2_buffer* buf = (struct v4l2_buffer*)arg;
        fprintf(stderr, "[V4L2_INTERCEPT] DQBUF: index=%d, bytesused=%d\n",
                buf->index, buf->bytesused);

        // Update buffer length with actual bytes used
        if (buf->index < MAX_BUFFERS && g_buffers[buf->index].used) {
            if (buf->bytesused > 0) {
                g_buffers[buf->index].length = buf->bytesused;
            }
            save_frame_data(buf->index);
        }
    }

    // Track REQBUFS to know buffer count
    if (fd == g_video_fd && request == VIDIOC_REQBUFS && result == 0) {
        struct v4l2_requestbuffers* req = (struct v4l2_requestbuffers*)arg;
        fprintf(stderr, "[V4L2_INTERCEPT] REQBUFS: count=%d, memory=%d\n",
                req->count, req->memory);
    }

    return result;
}

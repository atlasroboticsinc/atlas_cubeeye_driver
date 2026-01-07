/**
 * simple_v4l2_hook.c - Minimal V4L2 frame capture hook
 *
 * Intercepts ioctl and mmap with protection against early calls during ld.so
 *
 * Build: gcc -shared -fPIC -o libsimple_hook.so simple_v4l2_hook.c -ldl -lpthread
 * Usage: LD_PRELOAD=./libsimple_hook.so ./sdk_capture
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/syscall.h>

#define MAX_SAVED_FRAMES 500
#define MAX_BUFFERS 8

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_frame_count = 0;
static int g_video_fd = -1;
static void* g_buffers[MAX_BUFFERS] = {0};
static size_t g_buffer_sizes[MAX_BUFFERS] = {0};
static uint32_t g_buffer_offsets[MAX_BUFFERS] = {0};
static int g_num_buffers = 0;
static volatile int g_ready = 0;

static void save_frame(const void* data, size_t size, int idx) {
    const char* dir = getenv("HOOK_OUTPUT");
    if (!dir) dir = "data/sync_capture";

    char filename[512];
    snprintf(filename, sizeof(filename), "%s/raw_%04d.bin", dir, idx);

    FILE* f = fopen(filename, "wb");
    if (f) {
        fwrite(data, 1, size, f);
        fclose(f);

        const uint16_t* d16 = (const uint16_t*)data;
        int nonzero = 0;
        for (size_t i = 0; i < size/2 && i < 10000; i++) {
            if (d16[i] != 0) nonzero++;
        }
        fprintf(stderr, "[HOOK] Saved frame %d (%zu bytes, %d/10000 non-zero)\n",
                idx, size, nonzero);
    }
}

// Raw syscalls for early use
static long raw_ioctl(int fd, unsigned long request, void* arg) {
    return syscall(SYS_ioctl, fd, request, arg);
}

static void* raw_mmap(void* addr, size_t length, int prot, int flags, int fd, off_t offset) {
    return (void*)syscall(SYS_mmap, addr, length, prot, flags, fd, offset);
}

void* mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset) {
    static void* (*real_mmap)(void*, size_t, int, int, int, off_t) = NULL;

    // Use syscall during early loading
    if (!g_ready) {
        return raw_mmap(addr, length, prot, flags, fd, offset);
    }

    if (!real_mmap) {
        real_mmap = dlsym(RTLD_NEXT, "mmap");
        if (!real_mmap) {
            return raw_mmap(addr, length, prot, flags, fd, offset);
        }
    }

    void* result = real_mmap(addr, length, prot, flags, fd, offset);

    // Track mmap for video device
    if (result != MAP_FAILED && fd == g_video_fd && length > 100000) {
        pthread_mutex_lock(&g_mutex);
        // Find buffer by offset
        for (int i = 0; i < g_num_buffers; i++) {
            if (g_buffer_sizes[i] == length && g_buffer_offsets[i] == (uint32_t)offset && !g_buffers[i]) {
                g_buffers[i] = result;
                fprintf(stderr, "[HOOK] mmap buffer %d at %p (len=%zu, offset=%ld)\n",
                        i, result, length, offset);
                break;
            }
        }
        pthread_mutex_unlock(&g_mutex);
    }

    return result;
}

int ioctl(int fd, unsigned long request, ...) {
    static int (*real_ioctl)(int, unsigned long, ...) = NULL;

    va_list args;
    va_start(args, request);
    void* arg = va_arg(args, void*);
    va_end(args);

    // Use syscall during early loading
    if (!g_ready) {
        return raw_ioctl(fd, request, arg);
    }

    if (!real_ioctl) {
        real_ioctl = dlsym(RTLD_NEXT, "ioctl");
        if (!real_ioctl) {
            return raw_ioctl(fd, request, arg);
        }
        fprintf(stderr, "[HOOK] Initialized\n");
    }

    int result = real_ioctl(fd, request, arg);

    unsigned int req32 = (unsigned int)request;

    // Track video device via QUERYCAP
    if (req32 == VIDIOC_QUERYCAP && result == 0) {
        struct v4l2_capability* cap = (struct v4l2_capability*)arg;
        if (strstr((char*)cap->card, "Cube") || strstr((char*)cap->card, "Eye")) {
            pthread_mutex_lock(&g_mutex);
            g_video_fd = fd;
            pthread_mutex_unlock(&g_mutex);
            fprintf(stderr, "[HOOK] Found CubeEye on fd=%d ('%s')\n", fd, cap->card);
        }
    }

    // Track buffer info from QUERYBUF
    if (req32 == VIDIOC_QUERYBUF && result == 0 && fd == g_video_fd) {
        struct v4l2_buffer* buf = (struct v4l2_buffer*)arg;
        if (buf->index < MAX_BUFFERS && buf->memory == V4L2_MEMORY_MMAP) {
            pthread_mutex_lock(&g_mutex);
            g_buffer_sizes[buf->index] = buf->length;
            g_buffer_offsets[buf->index] = buf->m.offset;
            pthread_mutex_unlock(&g_mutex);
            fprintf(stderr, "[HOOK] QUERYBUF idx=%d len=%d offset=%u\n",
                    buf->index, buf->length, buf->m.offset);
        }
    }

    // Track buffer count from REQBUFS
    if (req32 == VIDIOC_REQBUFS && result == 0 && fd == g_video_fd) {
        struct v4l2_requestbuffers* req = (struct v4l2_requestbuffers*)arg;
        if (req->count > 0) {
            pthread_mutex_lock(&g_mutex);
            g_num_buffers = req->count < MAX_BUFFERS ? req->count : MAX_BUFFERS;
            // Clear buffer pointers for new allocation
            for (int i = 0; i < MAX_BUFFERS; i++) {
                g_buffers[i] = NULL;
            }
            pthread_mutex_unlock(&g_mutex);
            fprintf(stderr, "[HOOK] REQBUFS count=%d memory=%d\n", req->count, req->memory);
        }
    }

    // Capture frames from DQBUF
    if (req32 == VIDIOC_DQBUF && result == 0 && fd == g_video_fd) {
        struct v4l2_buffer* buf = (struct v4l2_buffer*)arg;

        pthread_mutex_lock(&g_mutex);

        if (g_frame_count < MAX_SAVED_FRAMES &&
            buf->index < g_num_buffers &&
            g_buffers[buf->index]) {

            size_t size = buf->bytesused > 0 ? buf->bytesused : g_buffer_sizes[buf->index];
            save_frame(g_buffers[buf->index], size, g_frame_count);
            g_frame_count++;
        }

        pthread_mutex_unlock(&g_mutex);
    }

    return result;
}

// Mark hook as ready after constructors have run
__attribute__((constructor(65535)))
static void hook_ready(void) {
    g_ready = 1;
    fprintf(stderr, "[HOOK] Ready\n");
}

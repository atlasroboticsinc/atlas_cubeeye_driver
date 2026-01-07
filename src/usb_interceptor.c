/**
 * usb_interceptor.c - Intercept libusb bulk transfers to capture raw ToF data
 *
 * Compile: gcc -shared -fPIC -o libusb_interceptor.so usb_interceptor.c -ldl
 * Usage: LD_PRELOAD=./libusb_interceptor.so ./build/sdk_capture 5
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

// libusb types
typedef struct libusb_device_handle libusb_device_handle;

// Original function pointer
static int (*orig_libusb_bulk_transfer)(libusb_device_handle *dev_handle,
    unsigned char endpoint, unsigned char *data, int length,
    int *actual_length, unsigned int timeout) = NULL;

// Output file
static FILE* g_raw_file = NULL;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_frame_count = 0;
static int g_bytes_this_frame = 0;
static const int EXPECTED_FRAME_SIZE = 771200;  // 241 * 3200 bytes
static uint8_t* g_frame_buffer = NULL;
static int g_max_frames = 5;
static char g_output_dir[256] = "data/usb_capture";

__attribute__((constructor))
static void init(void) {
    // Get original function
    orig_libusb_bulk_transfer = dlsym(RTLD_NEXT, "libusb_bulk_transfer");
    if (!orig_libusb_bulk_transfer) {
        fprintf(stderr, "[USB_INTERCEPT] Failed to find libusb_bulk_transfer\n");
        return;
    }

    // Check environment for config
    char* env_dir = getenv("USB_CAPTURE_DIR");
    if (env_dir) {
        strncpy(g_output_dir, env_dir, sizeof(g_output_dir)-1);
    }

    char* env_frames = getenv("USB_CAPTURE_FRAMES");
    if (env_frames) {
        g_max_frames = atoi(env_frames);
    }

    // Allocate frame buffer
    g_frame_buffer = (uint8_t*)malloc(EXPECTED_FRAME_SIZE);
    if (!g_frame_buffer) {
        fprintf(stderr, "[USB_INTERCEPT] Failed to allocate frame buffer\n");
        return;
    }

    // Create output directory
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "mkdir -p %s", g_output_dir);
    system(cmd);

    fprintf(stderr, "[USB_INTERCEPT] Initialized - capturing to %s (max %d frames)\n",
            g_output_dir, g_max_frames);
}

__attribute__((destructor))
static void cleanup(void) {
    if (g_frame_buffer) {
        free(g_frame_buffer);
        g_frame_buffer = NULL;
    }
    fprintf(stderr, "[USB_INTERCEPT] Captured %d complete frames\n", g_frame_count);
}

int libusb_bulk_transfer(libusb_device_handle *dev_handle,
    unsigned char endpoint, unsigned char *data, int length,
    int *actual_length, unsigned int timeout)
{
    // Call original function first
    int result = orig_libusb_bulk_transfer(dev_handle, endpoint, data, length,
                                           actual_length, timeout);

    // Only intercept incoming data on endpoint 0x81 (IN endpoint)
    if (result == 0 && (endpoint & 0x80) && actual_length && *actual_length > 0) {
        pthread_mutex_lock(&g_mutex);

        if (g_frame_count < g_max_frames && g_frame_buffer) {
            int bytes_to_copy = *actual_length;

            // Check if this would overflow buffer
            if (g_bytes_this_frame + bytes_to_copy <= EXPECTED_FRAME_SIZE) {
                memcpy(g_frame_buffer + g_bytes_this_frame, data, bytes_to_copy);
                g_bytes_this_frame += bytes_to_copy;
            }

            // Check if we have a complete frame
            if (g_bytes_this_frame >= EXPECTED_FRAME_SIZE) {
                // Save frame
                char filename[512];
                snprintf(filename, sizeof(filename), "%s/usb_raw_%04d.raw",
                         g_output_dir, g_frame_count);

                FILE* f = fopen(filename, "wb");
                if (f) {
                    fwrite(g_frame_buffer, 1, EXPECTED_FRAME_SIZE, f);
                    fclose(f);
                    fprintf(stderr, "[USB_INTERCEPT] Saved frame %d to %s\n",
                            g_frame_count, filename);
                }

                g_frame_count++;
                g_bytes_this_frame = 0;
            }
        }

        pthread_mutex_unlock(&g_mutex);
    }

    return result;
}

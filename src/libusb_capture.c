/**
 * libusb_capture.c - Direct libusb capture for CubeEye I200D
 *
 * Captures raw USB bulk transfers directly, bypassing V4L2/UVC transformation.
 * This should get the raw 5-byte packed data that the SDK formula expects.
 *
 * Build: gcc -o libusb_capture src/libusb_capture.c -I${SDK_PATH}/thirdparty/libusb/include -L${SDK_PATH}/thirdparty/libusb/lib -lusb-1.0 -lpthread
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <libusb-1.0/libusb.h>

#define CUBEEYE_VID 0x3674
#define CUBEEYE_PID 0x0200

#define BULK_EP_IN 0x83  // Bulk IN endpoint for data
#define FRAME_SIZE 771200  // Expected frame size (241 × 3200 bytes)
#define TRANSFER_SIZE 16384  // USB transfer chunk size

static volatile int g_running = 1;
static int g_frame_count = 0;
static int g_max_frames = 5;
static char g_output_dir[256] = "data/libusb_capture";

void signal_handler(int sig) {
    printf("\nReceived signal %d, stopping...\n", sig);
    g_running = 0;
}

int find_and_claim_device(libusb_context *ctx, libusb_device_handle **handle) {
    libusb_device **devs;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);

    if (cnt < 0) {
        fprintf(stderr, "Failed to get device list\n");
        return -1;
    }

    for (ssize_t i = 0; i < cnt; i++) {
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0) continue;

        if (desc.idVendor == CUBEEYE_VID && desc.idProduct == CUBEEYE_PID) {
            printf("Found CubeEye device: VID=%04x PID=%04x\n",
                   desc.idVendor, desc.idProduct);

            r = libusb_open(devs[i], handle);
            if (r < 0) {
                fprintf(stderr, "Failed to open device: %s\n", libusb_error_name(r));
                libusb_free_device_list(devs, 1);
                return -1;
            }

            // Check if kernel driver is active
            if (libusb_kernel_driver_active(*handle, 1) == 1) {
                printf("Kernel driver active on interface 1, detaching...\n");
                r = libusb_detach_kernel_driver(*handle, 1);
                if (r < 0) {
                    fprintf(stderr, "Failed to detach kernel driver: %s\n",
                            libusb_error_name(r));
                    // Continue anyway - might work
                }
            }

            // Claim interface 1 (Video Streaming)
            r = libusb_claim_interface(*handle, 1);
            if (r < 0) {
                fprintf(stderr, "Failed to claim interface: %s\n",
                        libusb_error_name(r));
                libusb_close(*handle);
                libusb_free_device_list(devs, 1);
                return -1;
            }

            printf("Successfully claimed interface 1\n");
            libusb_free_device_list(devs, 1);
            return 0;
        }
    }

    fprintf(stderr, "CubeEye device not found\n");
    libusb_free_device_list(devs, 1);
    return -1;
}

int configure_streaming(libusb_device_handle *handle) {
    // Set alternate interface for streaming
    // UVC uses alternate settings to control bandwidth
    int r = libusb_set_interface_alt_setting(handle, 1, 1);
    if (r < 0) {
        fprintf(stderr, "Failed to set alt setting: %s\n", libusb_error_name(r));
        return -1;
    }
    printf("Set alternate interface 1:1 for streaming\n");
    return 0;
}

void save_frame(const uint8_t *data, size_t size, int frame_idx) {
    char filename[512];
    snprintf(filename, sizeof(filename), "%s/usb_raw_%04d.bin",
             g_output_dir, frame_idx);

    FILE *f = fopen(filename, "wb");
    if (f) {
        fwrite(data, 1, size, f);
        fclose(f);

        // Analyze data
        int nonzero = 0;
        for (size_t i = 0; i < size && i < 20000; i++) {
            if (data[i] != 0) nonzero++;
        }

        printf("Saved frame %d: %zu bytes (%d/20000 non-zero)\n",
               frame_idx, size, nonzero);
    } else {
        fprintf(stderr, "Failed to save frame %d\n", frame_idx);
    }
}

int capture_frames(libusb_device_handle *handle) {
    uint8_t *frame_buffer = malloc(FRAME_SIZE);
    uint8_t *transfer_buffer = malloc(TRANSFER_SIZE);

    if (!frame_buffer || !transfer_buffer) {
        fprintf(stderr, "Failed to allocate buffers\n");
        free(frame_buffer);
        free(transfer_buffer);
        return -1;
    }

    size_t frame_offset = 0;
    int transferred;

    printf("Starting capture (max %d frames)...\n", g_max_frames);

    while (g_running && g_frame_count < g_max_frames) {
        int r = libusb_bulk_transfer(handle, BULK_EP_IN,
                                     transfer_buffer, TRANSFER_SIZE,
                                     &transferred, 1000);

        if (r == 0 && transferred > 0) {
            // Copy to frame buffer
            size_t to_copy = transferred;
            if (frame_offset + to_copy > FRAME_SIZE) {
                to_copy = FRAME_SIZE - frame_offset;
            }

            memcpy(frame_buffer + frame_offset, transfer_buffer, to_copy);
            frame_offset += to_copy;

            // Check if frame complete
            if (frame_offset >= FRAME_SIZE) {
                save_frame(frame_buffer, FRAME_SIZE, g_frame_count);
                g_frame_count++;
                frame_offset = 0;
            }
        } else if (r == LIBUSB_ERROR_TIMEOUT) {
            printf("Transfer timeout (frame_offset=%zu)\n", frame_offset);
        } else if (r != 0) {
            fprintf(stderr, "Bulk transfer error: %s\n", libusb_error_name(r));
            break;
        }
    }

    free(frame_buffer);
    free(transfer_buffer);
    return g_frame_count;
}

int main(int argc, char *argv[]) {
    if (argc > 1) {
        g_max_frames = atoi(argv[1]);
    }
    if (argc > 2) {
        strncpy(g_output_dir, argv[2], sizeof(g_output_dir) - 1);
    }

    // Create output directory
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "mkdir -p %s", g_output_dir);
    system(cmd);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("CubeEye Direct USB Capture\n");
    printf("==========================\n");
    printf("Max frames: %d\n", g_max_frames);
    printf("Output: %s\n\n", g_output_dir);

    libusb_context *ctx = NULL;
    libusb_device_handle *handle = NULL;

    int r = libusb_init(&ctx);
    if (r < 0) {
        fprintf(stderr, "Failed to init libusb: %s\n", libusb_error_name(r));
        return 1;
    }

    // Enable debug output
    libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);

    if (find_and_claim_device(ctx, &handle) < 0) {
        libusb_exit(ctx);
        return 1;
    }

    if (configure_streaming(handle) < 0) {
        libusb_release_interface(handle, 1);
        libusb_close(handle);
        libusb_exit(ctx);
        return 1;
    }

    int captured = capture_frames(handle);
    printf("\nCaptured %d frames\n", captured);

    // Cleanup
    libusb_set_interface_alt_setting(handle, 1, 0);  // Reset to alt 0
    libusb_release_interface(handle, 1);
    libusb_attach_kernel_driver(handle, 1);  // Reattach kernel driver
    libusb_close(handle);
    libusb_exit(ctx);

    return 0;
}

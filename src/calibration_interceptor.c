/**
 * calibration_interceptor.c - Comprehensive SDK Calibration Interceptor
 *
 * LD_PRELOAD hook to intercept ALL calibration data reads from SDK,
 * capturing exact bytes before and after SDK processes them.
 *
 * Compile:
 *   gcc -shared -fPIC -o libcal_intercept.so calibration_interceptor.c -ldl -lpthread
 *
 * Usage:
 *   LD_PRELOAD=./libcal_intercept.so ./benchmark_capture 1
 *
 * Output:
 *   - cal_intercept_output/xu_reads.log - All UVC XU page reads
 *   - cal_intercept_output/page_XXXX.bin - Raw page data
 *   - cal_intercept_output/fppn_assembled.bin - Assembled FPPN data
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

// UVC XU ioctl code
#define UVCIOC_CTRL_QUERY 0xc0107521

// Output configuration
static const char* OUTPUT_DIR = "cal_intercept_output";
static FILE* g_log = NULL;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_initialized = 0;

// FPPN page range
#define FPPN_START_PAGE 0x021C
#define FPPN_END_PAGE   0x0716
#define PAGE_DATA_SIZE  254

// FPPN assembly buffer
#define FPPN_WIDTH  320
#define FPPN_HEIGHT 240
#define FPPN_TOTAL  (FPPN_WIDTH * FPPN_HEIGHT)
static int16_t g_fppn_buffer[FPPN_TOTAL];
static int g_fppn_count = 0;
static int g_last_fppn_page = -1;

// Statistics
static int g_total_xu_reads = 0;
static int g_fppn_page_reads = 0;

// Real ioctl function
static int (*real_ioctl)(int, unsigned long, ...) = NULL;

// Initialize hook
static void init_hook(void) {
    if (g_initialized) return;

    pthread_mutex_lock(&g_mutex);
    if (g_initialized) {
        pthread_mutex_unlock(&g_mutex);
        return;
    }

    // Get real ioctl
    real_ioctl = dlsym(RTLD_NEXT, "ioctl");
    if (!real_ioctl) {
        fprintf(stderr, "[CAL_INTERCEPT] Failed to get real ioctl\n");
        pthread_mutex_unlock(&g_mutex);
        return;
    }

    // Create output directory
    mkdir(OUTPUT_DIR, 0755);

    // Open log file
    char log_path[256];
    snprintf(log_path, sizeof(log_path), "%s/xu_reads.log", OUTPUT_DIR);
    g_log = fopen(log_path, "w");
    if (g_log) {
        fprintf(g_log, "# CubeEye SDK Calibration Page Reads\n");
        fprintf(g_log, "# Format: page_num (hex), selector, query_type, size, hex_data\n\n");
        fflush(g_log);
    }

    // Initialize FPPN buffer to known invalid value
    memset(g_fppn_buffer, 0x00, sizeof(g_fppn_buffer));

    fprintf(stderr, "[CAL_INTERCEPT] Initialized, output dir: %s\n", OUTPUT_DIR);

    g_initialized = 1;
    pthread_mutex_unlock(&g_mutex);
}

// Check if value looks like FPPN
static int is_fppn_value(int16_t val) {
    return val > -950 && val < -450;
}

// Process FPPN page data
static void process_fppn_page(int page_num, const uint8_t* data, int size) {
    // Skip header (6 bytes: 00 20 HH LL 01 00)
    if (size < 10) return;

    const uint8_t* payload = data + 6;
    int payload_size = size - 6;
    int num_values = payload_size / 2;

    // Parse as big-endian int16
    for (int i = 0; i < num_values && g_fppn_count < FPPN_TOTAL; i++) {
        int16_t val = (int16_t)((payload[i*2] << 8) | payload[i*2 + 1]);

        if (is_fppn_value(val)) {
            g_fppn_buffer[g_fppn_count++] = val;
        }
    }

    g_last_fppn_page = page_num;
    g_fppn_page_reads++;

    if (g_log) {
        fprintf(g_log, "# FPPN page 0x%04X: %d values extracted, total=%d\n",
                page_num, num_values, g_fppn_count);
        fflush(g_log);
    }
}

// Save assembled FPPN
static void save_assembled_fppn(void) {
    pthread_mutex_lock(&g_mutex);

    if (g_fppn_count == 0) {
        pthread_mutex_unlock(&g_mutex);
        return;
    }

    char path[256];

    // Save as big-endian (SDK format)
    snprintf(path, sizeof(path), "%s/fppn_assembled_be.bin", OUTPUT_DIR);
    FILE* f = fopen(path, "wb");
    if (f) {
        // Convert to big-endian for saving
        for (int i = 0; i < FPPN_TOTAL; i++) {
            uint8_t be[2];
            be[0] = (g_fppn_buffer[i] >> 8) & 0xFF;
            be[1] = g_fppn_buffer[i] & 0xFF;
            fwrite(be, 1, 2, f);
        }
        fclose(f);
    }

    // Save as little-endian (native)
    snprintf(path, sizeof(path), "%s/fppn_assembled_le.bin", OUTPUT_DIR);
    f = fopen(path, "wb");
    if (f) {
        fwrite(g_fppn_buffer, sizeof(int16_t), FPPN_TOTAL, f);
        fclose(f);
    }

    // Save metadata
    snprintf(path, sizeof(path), "%s/fppn_metadata.txt", OUTPUT_DIR);
    f = fopen(path, "w");
    if (f) {
        fprintf(f, "SDK FPPN Assembly Results\n");
        fprintf(f, "=========================\n\n");
        fprintf(f, "Total FPPN pages read: %d\n", g_fppn_page_reads);
        fprintf(f, "Total FPPN values: %d / %d (%.1f%%)\n",
                g_fppn_count, FPPN_TOTAL, 100.0 * g_fppn_count / FPPN_TOTAL);
        fprintf(f, "Last FPPN page: 0x%04X\n", g_last_fppn_page);

        // Calculate stats
        int16_t min_val = 0, max_val = 0;
        int64_t sum = 0;
        int valid_count = 0;

        for (int i = 0; i < g_fppn_count; i++) {
            int16_t v = g_fppn_buffer[i];
            if (is_fppn_value(v)) {
                if (valid_count == 0 || v < min_val) min_val = v;
                if (valid_count == 0 || v > max_val) max_val = v;
                sum += v;
                valid_count++;
            }
        }

        if (valid_count > 0) {
            fprintf(f, "\nValue Statistics:\n");
            fprintf(f, "  Min: %d\n", min_val);
            fprintf(f, "  Max: %d\n", max_val);
            fprintf(f, "  Mean: %.1f\n", (double)sum / valid_count);
        }

        // Check for gaps
        int missing = FPPN_TOTAL - g_fppn_count;
        if (missing > 0) {
            fprintf(f, "\nMissing pixels: %d\n", missing);
            fprintf(f, "Missing rows: %.1f\n", (double)missing / FPPN_WIDTH);
        }

        fclose(f);
    }

    fprintf(stderr, "[CAL_INTERCEPT] Saved assembled FPPN: %d/%d values\n",
            g_fppn_count, FPPN_TOTAL);

    pthread_mutex_unlock(&g_mutex);
}

// Hooked ioctl
int ioctl(int fd, unsigned long request, ...) {
    init_hook();

    va_list args;
    va_start(args, request);
    void* arg = va_arg(args, void*);
    va_end(args);

    if (!real_ioctl) {
        errno = ENOSYS;
        return -1;
    }

    // Call real ioctl first
    int result = real_ioctl(fd, request, arg);

    // Check if this is UVC XU query
    if (request == UVCIOC_CTRL_QUERY && arg && result >= 0) {
        struct uvc_xu_control_query* xu = (struct uvc_xu_control_query*)arg;

        // Selector 4 is calibration data
        if (xu->selector == 4 && xu->query == UVC_GET_CUR && xu->data) {
            pthread_mutex_lock(&g_mutex);
            g_total_xu_reads++;

            // Parse page number from response
            // Format: 00 20 HH LL 01 00 [data...]
            uint8_t* data = xu->data;
            if (xu->size >= 4) {
                int page_num = (data[2] << 8) | data[3];

                // Log this read
                if (g_log) {
                    fprintf(g_log, "0x%04X sel=%d query=GET_CUR size=%d data=",
                            page_num, xu->selector, xu->size);
                    for (int i = 0; i < (xu->size < 32 ? xu->size : 32); i++) {
                        fprintf(g_log, "%02X", data[i]);
                    }
                    if (xu->size > 32) fprintf(g_log, "...");
                    fprintf(g_log, "\n");
                    fflush(g_log);
                }

                // Save raw page
                char page_path[256];
                snprintf(page_path, sizeof(page_path), "%s/page_%04x.bin",
                         OUTPUT_DIR, page_num);
                FILE* pf = fopen(page_path, "wb");
                if (pf) {
                    fwrite(data, 1, xu->size, pf);
                    fclose(pf);
                }

                // Process FPPN pages
                if (page_num >= FPPN_START_PAGE && page_num <= FPPN_END_PAGE) {
                    process_fppn_page(page_num, data, xu->size);
                }
            }

            pthread_mutex_unlock(&g_mutex);
        }
    }

    return result;
}

// Cleanup on library unload
__attribute__((destructor))
static void cleanup(void) {
    if (g_log) {
        fprintf(g_log, "\n# Summary:\n");
        fprintf(g_log, "# Total XU reads: %d\n", g_total_xu_reads);
        fprintf(g_log, "# FPPN pages read: %d\n", g_fppn_page_reads);
        fprintf(g_log, "# FPPN values assembled: %d/%d\n", g_fppn_count, FPPN_TOTAL);
        fclose(g_log);
        g_log = NULL;
    }

    save_assembled_fppn();

    fprintf(stderr, "\n[CAL_INTERCEPT] Summary:\n");
    fprintf(stderr, "  Total XU reads: %d\n", g_total_xu_reads);
    fprintf(stderr, "  FPPN pages read: %d\n", g_fppn_page_reads);
    fprintf(stderr, "  FPPN values: %d/%d (%.1f%%)\n",
            g_fppn_count, FPPN_TOTAL, 100.0 * g_fppn_count / FPPN_TOTAL);
}

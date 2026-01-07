/**
 * calibration_dump.c - Dump all calibration pages from CubeEye camera
 *
 * Queries UVC XU selector 4 for all available calibration pages.
 * FPPN data is expected to be ~153,600 bytes (320*240*2) per frequency.
 *
 * Build:
 *   gcc -o calibration_dump calibration_dump.c -I../include $(pkg-config --cflags --libs libusb-1.0)
 *
 * Usage:
 *   ./calibration_dump [output_dir]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>

// UVC XU definitions
#ifndef UVCIOC_CTRL_QUERY
struct uvc_xu_control_query {
    __u8 unit;
    __u8 selector;
    __u8 query;
    __u16 size;
    __u8 *data;
} __attribute__((packed));
#define UVCIOC_CTRL_QUERY _IOWR('u', 0x21, struct uvc_xu_control_query)
#endif

#define UVC_SET_CUR  0x01
#define UVC_GET_CUR  0x81

#define XU_UNIT      3
#define XU_SELECTOR  4
#define PAGE_SIZE    260

// Known page ranges to scan
typedef struct {
    uint16_t start;
    uint16_t end;
    const char* name;
} PageRange;

static PageRange page_ranges[] = {
    {0x0000, 0x0010, "device_info"},
    {0x0005, 0x0006, "lens_calibration"},
    {0x0080, 0x0090, "sensor_config"},
    {0x0100, 0x0400, "fppn_range1"},       // Possible FPPN location
    {0x1000, 0x1400, "fppn_range2"},       // Another possible range
    {0x2000, 0x2400, "fppn_range3"},       // Another possible range
    {0, 0, NULL}
};

// Find CubeEye camera device
static int find_cubeeye_device(char* path, size_t path_size) {
    DIR* dir = opendir("/dev");
    if (!dir) return -1;

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "video", 5) != 0) continue;

        snprintf(path, path_size, "/dev/%s", entry->d_name);
        int fd = open(path, O_RDWR);
        if (fd < 0) continue;

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
            if (strstr((char*)cap.card, "Cube Eye") != NULL) {
                close(fd);
                closedir(dir);
                return 0;
            }
        }
        close(fd);
    }

    closedir(dir);
    return -1;
}

// Query a calibration page
static int query_page(int fd, uint16_t page_num, uint8_t* data) {
    uint8_t request[PAGE_SIZE] = {0};

    // Set up page request
    // Format: 00 20 LL HH (where HHLL is page number in little-endian)
    request[0] = 0x00;
    request[1] = 0x20;
    request[2] = page_num & 0xFF;
    request[3] = (page_num >> 8) & 0xFF;

    struct uvc_xu_control_query xu_set = {
        .unit = XU_UNIT,
        .selector = XU_SELECTOR,
        .query = UVC_SET_CUR,
        .size = PAGE_SIZE,
        .data = request
    };

    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_set) < 0) {
        return -1;
    }

    // Get the response
    struct uvc_xu_control_query xu_get = {
        .unit = XU_UNIT,
        .selector = XU_SELECTOR,
        .query = UVC_GET_CUR,
        .size = PAGE_SIZE,
        .data = data
    };

    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_get) < 0) {
        return -1;
    }

    // Check if page is valid (header should be 00 20)
    if (data[0] != 0x00 || data[1] != 0x20) {
        return -2;  // Invalid page
    }

    // Check if page number matches
    uint16_t returned_page = data[2] | (data[3] << 8);
    if (returned_page != page_num) {
        return -3;  // Page number mismatch
    }

    return 0;
}

// Check if page has non-zero data
static int page_has_data(uint8_t* data) {
    for (int i = 4; i < PAGE_SIZE; i++) {
        if (data[i] != 0) return 1;
    }
    return 0;
}

int main(int argc, char* argv[]) {
    const char* output_dir = argc > 1 ? argv[1] : "./cal_dump";
    char device_path[256];
    uint8_t page_data[PAGE_SIZE];
    int total_pages = 0;
    int valid_pages = 0;

    printf("CubeEye Calibration Page Dumper\n");
    printf("================================\n");
    printf("Output directory: %s\n\n", output_dir);

    // Create output directory
    mkdir(output_dir, 0755);

    // Find camera
    if (find_cubeeye_device(device_path, sizeof(device_path)) != 0) {
        fprintf(stderr, "Error: No CubeEye camera found\n");
        return 1;
    }
    printf("Found camera: %s\n\n", device_path);

    int fd = open(device_path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Error: Cannot open %s\n", device_path);
        return 1;
    }

    // Open combined output file
    char combined_path[512];
    snprintf(combined_path, sizeof(combined_path), "%s/all_pages.bin", output_dir);
    FILE* combined = fopen(combined_path, "wb");

    // Scan known page ranges
    for (int r = 0; page_ranges[r].name != NULL; r++) {
        printf("Scanning range %s (0x%04X - 0x%04X)...\n",
               page_ranges[r].name, page_ranges[r].start, page_ranges[r].end);

        for (uint16_t page = page_ranges[r].start; page <= page_ranges[r].end; page++) {
            total_pages++;
            memset(page_data, 0, PAGE_SIZE);

            int ret = query_page(fd, page, page_data);
            if (ret == 0 && page_has_data(page_data)) {
                valid_pages++;

                // Save individual page
                char page_path[512];
                snprintf(page_path, sizeof(page_path), "%s/page_%04x.bin",
                         output_dir, page);
                FILE* f = fopen(page_path, "wb");
                if (f) {
                    fwrite(page_data, 1, PAGE_SIZE, f);
                    fclose(f);
                }

                // Append to combined file
                if (combined) {
                    fwrite(page_data, 1, PAGE_SIZE, combined);
                }

                printf("  Page 0x%04X: %d bytes of data\n", page, PAGE_SIZE - 4);
            }
        }
    }

    // Also try a brute-force scan of first 256 pages
    printf("\nBrute-force scanning pages 0x0000 - 0x00FF...\n");
    for (uint16_t page = 0; page <= 0xFF; page++) {
        memset(page_data, 0, PAGE_SIZE);
        int ret = query_page(fd, page, page_data);
        if (ret == 0 && page_has_data(page_data)) {
            // Check if we already have this page
            char page_path[512];
            snprintf(page_path, sizeof(page_path), "%s/page_%04x.bin", output_dir, page);
            if (access(page_path, F_OK) != 0) {
                valid_pages++;
                FILE* f = fopen(page_path, "wb");
                if (f) {
                    fwrite(page_data, 1, PAGE_SIZE, f);
                    fclose(f);
                }
                if (combined) {
                    fwrite(page_data, 1, PAGE_SIZE, combined);
                }
                printf("  Page 0x%04X: NEW - %d bytes\n", page, PAGE_SIZE - 4);
            }
        }
    }

    if (combined) fclose(combined);
    close(fd);

    printf("\n================================\n");
    printf("Scanned %d pages, found %d with data\n", total_pages + 256, valid_pages);
    printf("Output saved to: %s\n", output_dir);

    return 0;
}

/**
 * cal_scan.c - Comprehensive calibration page scanner
 * Scans pages 0x0000 to 0x0FFF and saves pages with actual data
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <dirent.h>

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
#define PAGE_SIZE    260

int main(int argc, char* argv[]) {
    const char* output_dir = argc > 1 ? argv[1] : "./cal_scan_output";
    char device_path[256] = "";

    // Find CubeEye Video Capture device
    DIR* dir = opendir("/dev");
    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "video", 5) != 0) continue;
        snprintf(device_path, sizeof(device_path), "/dev/%s", entry->d_name);
        int fd = open(device_path, O_RDWR);
        if (fd < 0) continue;
        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
            strstr((char*)cap.card, "Cube Eye") != NULL &&
            (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) {
            close(fd);
            break;
        }
        close(fd);
        device_path[0] = 0;
    }
    closedir(dir);

    if (device_path[0] == 0) {
        fprintf(stderr, "No CubeEye camera found\n");
        return 1;
    }

    printf("Found: %s\n", device_path);
    printf("Output: %s\n\n", output_dir);

    mkdir(output_dir, 0755);

    int fd = open(device_path, O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    uint8_t request[PAGE_SIZE] = {0};
    uint8_t response[PAGE_SIZE] = {0};
    uint8_t prev_response[PAGE_SIZE] = {0};

    struct uvc_xu_control_query xu_set = {
        .unit = 3, .selector = 4, .query = UVC_SET_CUR, .size = PAGE_SIZE, .data = request
    };
    struct uvc_xu_control_query xu_get = {
        .unit = 3, .selector = 4, .query = UVC_GET_CUR, .size = PAGE_SIZE, .data = response
    };

    // First get page 0 as reference (many invalid pages return this)
    request[0] = 0x00; request[1] = 0x20; request[2] = 0x00; request[3] = 0x00;
    ioctl(fd, UVCIOC_CTRL_QUERY, &xu_set);
    ioctl(fd, UVCIOC_CTRL_QUERY, &xu_get);
    memcpy(prev_response, response, PAGE_SIZE);

    int found_pages = 0;
    int total_data = 0;

    // Open combined output file
    char combined_path[512];
    snprintf(combined_path, sizeof(combined_path), "%s/all_valid_pages.bin", output_dir);
    FILE* combined = fopen(combined_path, "wb");

    // Scan pages 0x0000 to 0x0FFF
    printf("Scanning pages 0x0000 - 0x0FFF...\n");
    for (uint16_t page = 0; page <= 0x0FFF; page++) {
        memset(request, 0, PAGE_SIZE);
        memset(response, 0, PAGE_SIZE);

        request[0] = 0x00;
        request[1] = 0x20;
        request[2] = (page >> 8) & 0xFF;
        request[3] = page & 0xFF;

        if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_set) < 0) continue;
        if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_get) < 0) continue;

        // Check if this page has unique data (not same as page 0 device info)
        int is_unique = 0;
        int is_all_ff = 1;
        int has_data = 0;

        for (int i = 4; i < PAGE_SIZE; i++) {
            if (response[i] != 0xFF) is_all_ff = 0;
            if (response[i] != 0) has_data = 1;
            if (response[i] != prev_response[i]) is_unique = 1;
        }

        // Skip if it's just returning page 0 data or all 0xFF
        if (!is_unique && page > 0) continue;
        if (is_all_ff) continue;
        if (!has_data) continue;

        // This page has unique data - save it
        found_pages++;
        total_data += PAGE_SIZE - 4;

        char page_path[512];
        snprintf(page_path, sizeof(page_path), "%s/page_%04x.bin", output_dir, page);
        FILE* f = fopen(page_path, "wb");
        if (f) {
            fwrite(response, 1, PAGE_SIZE, f);
            fclose(f);
        }

        if (combined) {
            fwrite(response, 1, PAGE_SIZE, combined);
        }

        printf("Page 0x%04X: ", page);
        for (int i = 4; i < 20 && i < PAGE_SIZE; i++) {
            printf("%02x ", response[i]);
        }
        printf("...\n");

        // Progress indicator
        if (page % 256 == 0 && page > 0) {
            printf("  ... scanned %d pages, found %d unique\n", page, found_pages);
        }
    }

    if (combined) fclose(combined);
    close(fd);

    printf("\n=== Summary ===\n");
    printf("Pages scanned: 4096\n");
    printf("Unique pages found: %d\n", found_pages);
    printf("Total data: %d bytes\n", total_data);
    printf("Output saved to: %s\n", output_dir);

    // Check if we found enough data for FPPN
    if (total_data >= 150000) {
        printf("\n*** Possible FPPN data found! ***\n");
        printf("Expected FPPN size: 153600 bytes per frequency\n");
    }

    return 0;
}

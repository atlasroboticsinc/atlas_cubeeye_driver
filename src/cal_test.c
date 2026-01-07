/**
 * cal_test.c - Simple test to query a known calibration page
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

int main() {
    // Find CubeEye camera
    char device_path[256] = "";
    DIR* dir = opendir("/dev");
    struct dirent* entry;

    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "video", 5) != 0) continue;
        snprintf(device_path, sizeof(device_path), "/dev/%s", entry->d_name);
        int fd = open(device_path, O_RDWR);
        if (fd < 0) continue;

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
            if (strstr((char*)cap.card, "Cube Eye") != NULL &&
                (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) {
                printf("Found: %s (%s) - Video Capture\n", device_path, cap.card);
                close(fd);
                break;
            }
        }
        close(fd);
        device_path[0] = 0;
    }
    closedir(dir);

    if (device_path[0] == 0) {
        fprintf(stderr, "No CubeEye camera found\n");
        return 1;
    }

    int fd = open(device_path, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    // Try querying page 0 (device info) - this is what SDK does
    uint8_t request[260] = {0};
    uint8_t response[260] = {0};

    // Looking at the SDK log:
    // SET_CUR DATA: 00 20 00 00 ... (page 0x0000)
    // GET_CUR DATA: 00 20 00 00 49 32 30 30 44 ... (I200D)
    // Format: 00 20 HH LL (big-endian page number)
    request[0] = 0x00;
    request[1] = 0x20;
    request[2] = 0x00;  // Page high byte (big-endian)
    request[3] = 0x00;  // Page low byte

    printf("\nTesting page 0x0000 (device info)...\n");
    printf("SET_CUR request: %02x %02x %02x %02x\n",
           request[0], request[1], request[2], request[3]);

    struct uvc_xu_control_query xu_set = {
        .unit = 3,
        .selector = 4,
        .query = UVC_SET_CUR,
        .size = 260,
        .data = request
    };

    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_set) < 0) {
        perror("SET_CUR failed");
    } else {
        printf("SET_CUR success\n");
    }

    struct uvc_xu_control_query xu_get = {
        .unit = 3,
        .selector = 4,
        .query = UVC_GET_CUR,
        .size = 260,
        .data = response
    };

    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_get) < 0) {
        perror("GET_CUR failed");
    } else {
        printf("GET_CUR success\n");
        printf("Response: ");
        for (int i = 0; i < 64; i++) {
            printf("%02x ", response[i]);
        }
        printf("...\n");

        // Check for I200D string
        if (strstr((char*)response + 4, "I200D") != NULL) {
            printf("\nFOUND I200D in response - page query works!\n");
        }
    }

    // Now try page 5 (lens calibration) - big-endian: 00 05
    printf("\nTesting page 0x0005 (lens calibration)...\n");
    memset(request, 0, 260);
    memset(response, 0, 260);
    request[0] = 0x00;
    request[1] = 0x20;
    request[2] = 0x00;  // Page high byte
    request[3] = 0x05;  // Page low byte

    xu_set.data = request;
    xu_get.data = response;

    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_set) < 0) {
        perror("SET_CUR page 5");
    }
    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_get) < 0) {
        perror("GET_CUR page 5");
    } else {
        printf("Response: ");
        for (int i = 0; i < 64; i++) {
            printf("%02x ", response[i]);
        }
        printf("...\n");

        // Check floats at offset 4
        float* fx = (float*)(response + 4);
        printf("fx = %.2f (expected ~393)\n", *fx);
    }

    // Try page 0x82 (sensor config)
    printf("\nTesting page 0x0082 (sensor config)...\n");
    memset(request, 0, 260);
    memset(response, 0, 260);
    request[0] = 0x00;
    request[1] = 0x20;
    request[2] = 0x00;
    request[3] = 0x82;

    xu_set.data = request;
    xu_get.data = response;

    ioctl(fd, UVCIOC_CTRL_QUERY, &xu_set);
    if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_get) == 0) {
        printf("Response: ");
        for (int i = 0; i < 64; i++) {
            printf("%02x ", response[i]);
        }
        printf("...\n");
    }

    // Scan for potential FPPN pages (try various ranges)
    printf("\nScanning for FPPN pages...\n");
    uint16_t test_pages[] = {
        0x0001, 0x0002, 0x0003, 0x0004, 0x0006, 0x0007, 0x0008,
        0x0010, 0x0020, 0x0030, 0x0040, 0x0050, 0x0060, 0x0070, 0x0080, 0x0081, 0x0083,
        0x0100, 0x0200, 0x0300, 0x0400,
        0x1000, 0x2000, 0x3000, 0x4000,
        0xFFFF
    };

    for (int i = 0; test_pages[i] != 0xFFFF; i++) {
        uint16_t page = test_pages[i];
        memset(request, 0, 260);
        memset(response, 0, 260);
        request[0] = 0x00;
        request[1] = 0x20;
        request[2] = (page >> 8) & 0xFF;  // High byte
        request[3] = page & 0xFF;          // Low byte

        xu_set.data = request;
        xu_get.data = response;

        if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_set) == 0 &&
            ioctl(fd, UVCIOC_CTRL_QUERY, &xu_get) == 0) {
            // Check if response has data (not all zeros after header)
            int has_data = 0;
            for (int j = 4; j < 64; j++) {
                if (response[j] != 0) { has_data = 1; break; }
            }
            if (has_data) {
                printf("Page 0x%04X has data: %02x %02x %02x %02x %02x %02x %02x %02x...\n",
                       page, response[4], response[5], response[6], response[7],
                       response[8], response[9], response[10], response[11]);
            }
        }
    }

    close(fd);
    return 0;
}

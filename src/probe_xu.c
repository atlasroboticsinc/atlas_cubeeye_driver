/**
 * probe_xu.c - Probe CubeEye UVC Extension Unit controls
 *
 * This tool queries all 5 XU controls to understand what they do.
 *
 * Build:
 *   gcc -o probe_xu src/probe_xu.c
 *
 * Usage:
 *   sudo ./probe_xu /dev/video0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>

#define XU_UNIT_ID 3
#define MAX_CONTROL_SIZE 512

// UVC XU control query types
#define UVC_GET_CUR  0x81
#define UVC_GET_MIN  0x82
#define UVC_GET_MAX  0x83
#define UVC_GET_RES  0x84
#define UVC_GET_LEN  0x85
#define UVC_GET_INFO 0x86
#define UVC_GET_DEF  0x87
#define UVC_SET_CUR  0x01

void print_hex(const char* label, uint8_t* data, int len) {
    printf("%s: ", label);
    for (int i = 0; i < len && i < 32; i++) {
        printf("%02x ", data[i]);
    }
    if (len > 32) printf("...");
    printf("\n");
}

int query_xu(int fd, int unit, int selector, int query, uint8_t* data, int size) {
    struct uvc_xu_control_query xu = {
        .unit = unit,
        .selector = selector,
        .query = query,
        .size = size,
        .data = data
    };

    memset(data, 0, size);

    int ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if (ret < 0) {
        return -errno;
    }
    return 0;
}

void probe_selector(int fd, int selector) {
    uint8_t data[MAX_CONTROL_SIZE];
    uint8_t len_data[2] = {0};
    uint8_t info_data[1] = {0};
    int ret;

    printf("\n=== Selector %d ===\n", selector);

    // First get the length
    ret = query_xu(fd, XU_UNIT_ID, selector, UVC_GET_LEN, len_data, 2);
    if (ret < 0) {
        printf("  GET_LEN failed: %s (%d)\n", strerror(-ret), -ret);
        return;
    }
    int len = len_data[0] | (len_data[1] << 8);
    printf("  Length: %d bytes\n", len);

    if (len == 0) {
        printf("  Zero length, skipping\n");
        return;
    }
    if (len > MAX_CONTROL_SIZE) {
        printf("  Length %d > buffer size %d, truncating\n", len, MAX_CONTROL_SIZE);
        len = MAX_CONTROL_SIZE;
    }

    // Get info (capabilities)
    ret = query_xu(fd, XU_UNIT_ID, selector, UVC_GET_INFO, info_data, 1);
    if (ret == 0) {
        printf("  Info: 0x%02x", info_data[0]);
        if (info_data[0] & 0x01) printf(" [GET]");
        if (info_data[0] & 0x02) printf(" [SET]");
        if (info_data[0] & 0x04) printf(" [DISABLED]");
        if (info_data[0] & 0x08) printf(" [AUTOUPDATE]");
        if (info_data[0] & 0x10) printf(" [ASYNC]");
        printf("\n");
    }

    // Get current value
    ret = query_xu(fd, XU_UNIT_ID, selector, UVC_GET_CUR, data, len);
    if (ret == 0) {
        print_hex("  Current", data, len);
    } else {
        printf("  GET_CUR failed: %s\n", strerror(-ret));
    }

    // Get min
    ret = query_xu(fd, XU_UNIT_ID, selector, UVC_GET_MIN, data, len);
    if (ret == 0) {
        print_hex("  Min", data, len);
    }

    // Get max
    ret = query_xu(fd, XU_UNIT_ID, selector, UVC_GET_MAX, data, len);
    if (ret == 0) {
        print_hex("  Max", data, len);
    }

    // Get default
    ret = query_xu(fd, XU_UNIT_ID, selector, UVC_GET_DEF, data, len);
    if (ret == 0) {
        print_hex("  Default", data, len);
    }
}

int main(int argc, char* argv[]) {
    const char* device = "/dev/video0";

    if (argc > 1) {
        device = argv[1];
    }

    printf("CubeEye UVC XU Probe Tool\n");
    printf("========================\n");
    printf("Device: %s\n", device);
    printf("XU Unit ID: %d\n", XU_UNIT_ID);

    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    // Query device capabilities
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        printf("Card: %s\n", cap.card);
        printf("Driver: %s\n", cap.driver);
    }

    // Probe all 5 XU selectors (based on bmControls = 0x1f)
    for (int selector = 1; selector <= 5; selector++) {
        probe_selector(fd, selector);
    }

    close(fd);

    printf("\n=== Done ===\n");
    return 0;
}

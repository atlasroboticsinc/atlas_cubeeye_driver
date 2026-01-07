/**
 * set_mode.c - Set CubeEye sensor mode via UVC XU
 *
 * Based on captured SDK traffic:
 *   Selector 1 with data: 02 00 00 <mode>
 *   where mode = 1, 2, or 3 (likely frequency modes)
 *
 * Build:
 *   gcc -o set_mode src/set_mode.c
 *
 * Usage:
 *   sudo ./set_mode /dev/video0 <mode>
 *   where mode is 0, 1, 2, or 3
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>

#define XU_UNIT_ID 3
#define UVC_SET_CUR 0x01
#define UVC_GET_CUR 0x81

int set_xu(int fd, int unit, int selector, uint8_t* data, int size) {
    struct uvc_xu_control_query xu = {
        .unit = unit,
        .selector = selector,
        .query = UVC_SET_CUR,
        .size = size,
        .data = data
    };
    return ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
}

int get_xu(int fd, int unit, int selector, uint8_t* data, int size) {
    struct uvc_xu_control_query xu = {
        .unit = unit,
        .selector = selector,
        .query = UVC_GET_CUR,
        .size = size,
        .data = data
    };
    memset(data, 0, size);
    return ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
}

void print_hex(const char* label, uint8_t* data, int len) {
    printf("%s: ", label);
    for (int i = 0; i < len; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printf("Usage: %s <device> <mode>\n", argv[0]);
        printf("  device: /dev/video0\n");
        printf("  mode: 0=100MHz, 1=80MHz, 2=Dual, 3=?\n");
        printf("\nBased on SDK traffic:\n");
        printf("  Selector 1: 02 00 00 <mode> (mode=1,2,3)\n");
        printf("  Selector 2: XX XX XX XX (register access)\n");
        return 1;
    }

    const char* device = argv[1];
    int mode = atoi(argv[2]);

    printf("CubeEye Mode Setter\n");
    printf("==================\n");
    printf("Device: %s\n", device);
    printf("Target mode: %d\n", mode);

    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    // Prepare Selector 1 command (20 bytes)
    uint8_t cmd1[20] = {0};
    cmd1[0] = 0x02;  // Command type
    cmd1[1] = 0x00;
    cmd1[2] = 0x00;
    cmd1[3] = mode & 0xFF;

    printf("\nSending to Selector 1:\n");
    print_hex("  Data", cmd1, 20);

    // Get current value first
    uint8_t current[20] = {0};
    if (get_xu(fd, XU_UNIT_ID, 1, current, 20) == 0) {
        print_hex("  Before", current, 20);
    }

    // Set new mode
    if (set_xu(fd, XU_UNIT_ID, 1, cmd1, 20) < 0) {
        perror("  SET_CUR failed");
        close(fd);
        return 1;
    }
    printf("  SET_CUR: OK\n");

    // Read back
    if (get_xu(fd, XU_UNIT_ID, 1, current, 20) == 0) {
        print_hex("  After", current, 20);
    }

    // Also check selector 5 (short status)
    uint8_t status[8] = {0};
    if (get_xu(fd, XU_UNIT_ID, 5, status, 8) == 0) {
        print_hex("\nSelector 5 status", status, 8);
    }

    close(fd);

    printf("\nDone. Capture frames to see if data format changed.\n");
    return 0;
}

/**
 * reg_access.c - CubeEye register access via UVC XU Selector 2
 *
 * Based on SDK traffic:
 *   Selector 2 format: CMD ADDR_LO ADDR_HI VALUE [extra...]
 *   CMD: 0x00 = read, 0x01 = write
 *
 * Build:
 *   gcc -o reg_access src/reg_access.c
 *
 * Usage:
 *   sudo ./reg_access /dev/video0 read <addr>
 *   sudo ./reg_access /dev/video0 write <addr> <value>
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
#define SEL_REG 2
#define UVC_SET_CUR 0x01
#define UVC_GET_CUR 0x81

int set_xu(int fd, int selector, uint8_t* data, int size) {
    struct uvc_xu_control_query xu = {
        .unit = XU_UNIT_ID,
        .selector = selector,
        .query = UVC_SET_CUR,
        .size = size,
        .data = data
    };
    return ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
}

int get_xu(int fd, int selector, uint8_t* data, int size) {
    struct uvc_xu_control_query xu = {
        .unit = XU_UNIT_ID,
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
    for (int i = 0; i < len && i < 20; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");
}

int read_register(int fd, uint16_t addr) {
    uint8_t cmd[20] = {0};
    cmd[0] = 0x00;  // Read command
    cmd[1] = addr & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;

    printf("Reading register 0x%04X\n", addr);
    print_hex("  Request", cmd, 20);

    if (set_xu(fd, SEL_REG, cmd, 20) < 0) {
        perror("  SET_CUR failed");
        return -1;
    }

    uint8_t response[20] = {0};
    if (get_xu(fd, SEL_REG, response, 20) < 0) {
        perror("  GET_CUR failed");
        return -1;
    }

    print_hex("  Response", response, 20);
    return (int)(response[3] | (response[4] << 8));
}

int write_register(int fd, uint16_t addr, uint16_t value) {
    uint8_t cmd[20] = {0};
    cmd[0] = 0x01;  // Write command
    cmd[1] = addr & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = value & 0xFF;
    cmd[4] = (value >> 8) & 0xFF;

    printf("Writing register 0x%04X = 0x%04X\n", addr, value);
    print_hex("  Request", cmd, 20);

    if (set_xu(fd, SEL_REG, cmd, 20) < 0) {
        perror("  SET_CUR failed");
        return -1;
    }

    // Verify by reading back
    uint8_t response[20] = {0};
    if (get_xu(fd, SEL_REG, response, 20) < 0) {
        perror("  GET_CUR failed");
        return -1;
    }

    print_hex("  Response", response, 20);
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        printf("Usage: %s <device> <read|write> <addr> [value]\n", argv[0]);
        printf("  addr: hex register address (e.g., 0x0004)\n");
        printf("  value: hex value for write (e.g., 0x0081)\n");
        printf("\nKnown registers from SDK traffic:\n");
        printf("  0x0004: Read 0x81 (sensor ID?)\n");
        printf("  0x0010: Read 0x0d (config?)\n");
        printf("  0x0101: Write 0xd0 (mode?)\n");
        printf("  0x0294: Write with extra byte (stream control?)\n");
        return 1;
    }

    const char* device = argv[1];
    const char* op = argv[2];
    uint16_t addr = strtol(argv[3], NULL, 0);

    printf("CubeEye Register Access\n");
    printf("======================\n");
    printf("Device: %s\n", device);

    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    if (strcmp(op, "read") == 0) {
        int value = read_register(fd, addr);
        if (value >= 0) {
            printf("\nResult: 0x%04X = %d\n", value, value);
        }
    } else if (strcmp(op, "write") == 0) {
        if (argc < 5) {
            printf("Error: write requires value\n");
            close(fd);
            return 1;
        }
        uint16_t value = strtol(argv[4], NULL, 0);
        write_register(fd, addr, value);
    } else {
        printf("Unknown operation: %s\n", op);
    }

    close(fd);
    return 0;
}

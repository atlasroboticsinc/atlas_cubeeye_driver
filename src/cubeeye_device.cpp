/**
 * cubeeye_device.cpp - CubeEye I200D Device Enumeration Implementation
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#include "cubeeye_device.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

namespace cubeeye {

// UVC query types
#define UVC_SET_CUR  0x01
#define UVC_GET_CUR  0x81
#define UVC_GET_LEN  0x85

// Helper for ioctl with retry on EINTR
static int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

// ============================================================================
// UvcControl Implementation
// ============================================================================

int UvcControl::query(int fd, int selector, int query_type, uint8_t* data, int size) {
    struct uvc_xu_control_query xu = {};
    xu.unit = UNIT_ID;
    xu.selector = static_cast<__u8>(selector);
    xu.query = static_cast<__u8>(query_type);
    xu.size = static_cast<__u16>(size);
    xu.data = data;

    return ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
}

int UvcControl::getLength(int fd, int selector) {
    uint8_t len_data[2] = {0};
    if (query(fd, selector, UVC_GET_LEN, len_data, 2) < 0) {
        return -1;
    }
    return len_data[0] | (len_data[1] << 8);
}

std::string UvcControl::querySerial(int fd) {
    int len = getLength(fd, SELECTOR_INFO);
    if (len <= 0) {
        return "";
    }

    std::vector<uint8_t> data(len, 0);

    // Send query command for serial number
    data[0] = INFO_CMD_QUERY;
    data[3] = INFO_SUBCMD_SERIAL;

    if (query(fd, SELECTOR_INFO, UVC_SET_CUR, data.data(), len) < 0) {
        return "";
    }

    // Small delay for device to process
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Read response
    std::fill(data.begin(), data.end(), 0);
    if (query(fd, SELECTOR_INFO, UVC_GET_CUR, data.data(), len) < 0) {
        return "";
    }

    // Serial is at offset 4, max 16 characters
    char serial[17] = {0};
    std::memcpy(serial, &data[4], 16);

    // Trim any trailing whitespace or nulls
    std::string result(serial);
    while (!result.empty() && (result.back() == ' ' || result.back() == '\0')) {
        result.pop_back();
    }

    return result;
}

std::string UvcControl::queryFirmwareVersion(int fd) {
    int len = getLength(fd, SELECTOR_INFO);
    if (len <= 0) {
        return "";
    }

    std::vector<uint8_t> data(len, 0);

    // Send query command for firmware version
    data[0] = INFO_CMD_QUERY;
    data[3] = INFO_SUBCMD_FIRMWARE;

    if (query(fd, SELECTOR_INFO, UVC_SET_CUR, data.data(), len) < 0) {
        return "";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::fill(data.begin(), data.end(), 0);
    if (query(fd, SELECTOR_INFO, UVC_GET_CUR, data.data(), len) < 0) {
        return "";
    }

    // Format firmware version (at offset 4)
    std::ostringstream oss;
    oss << "0x" << std::hex << std::setfill('0') << std::setw(2)
        << static_cast<int>(data[4]);

    return oss.str();
}

bool UvcControl::enableStreaming(int fd, bool enable) {
    int len = getLength(fd, SELECTOR_REG);
    if (len <= 0) len = 20;

    std::vector<uint8_t> cmd(len, 0);

    // Stream enable/disable command (discovered from SDK analysis):
    // Selector 2: 01 02 94 00 01 → Enable streaming
    // Selector 2: 01 02 94 00 00 → Disable streaming
    cmd[0] = 0x01;  // Write command
    cmd[1] = 0x02;  // Register bank/type
    cmd[2] = 0x94;  // Register address low
    cmd[3] = 0x00;  // Register address high
    cmd[4] = enable ? 0x01 : 0x00;

    return query(fd, SELECTOR_REG, UVC_SET_CUR, cmd.data(), len) == 0;
}

bool UvcControl::enableSensor(int fd) {
    int len = getLength(fd, SELECTOR_REG);
    if (len <= 0) len = 20;

    std::vector<uint8_t> cmd(len, 0);

    // Enable sensor/illuminator (write 0xD0 to register 0x0001)
    cmd[0] = 0x01;  // Write command
    cmd[1] = 0x01;  // Register address low
    cmd[2] = 0x00;  // Register address high
    cmd[3] = 0xd0;  // Value

    return query(fd, SELECTOR_REG, UVC_SET_CUR, cmd.data(), len) == 0;
}

bool UvcControl::disableIlluminator(int fd) {
    int len = getLength(fd, SELECTOR_STATUS);
    if (len <= 0) len = 8;

    std::vector<uint8_t> cmd(len, 0);

    // Disable illuminator: 01 80 00 16 00
    cmd[0] = 0x01;
    cmd[1] = 0x80;
    cmd[2] = 0x00;
    cmd[3] = 0x16;
    cmd[4] = 0x00;

    return query(fd, SELECTOR_STATUS, UVC_SET_CUR, cmd.data(), len) == 0;
}

bool UvcControl::initialize(int fd) {
    // Step 1: Enable sensor/illuminator
    if (!enableSensor(fd)) {
        return false;
    }

    // Step 2: Send status commands
    int len = getLength(fd, SELECTOR_STATUS);
    if (len <= 0) len = 8;

    std::vector<uint8_t> cmd(len, 0);
    cmd[0] = 0x01;
    cmd[1] = 0x80;
    cmd[2] = 0x00;
    cmd[3] = 0x16;

    // Status 0x00
    cmd[4] = 0x00;
    query(fd, SELECTOR_STATUS, UVC_SET_CUR, cmd.data(), len);

    // Status 0x01
    cmd[4] = 0x01;
    query(fd, SELECTOR_STATUS, UVC_SET_CUR, cmd.data(), len);

    // Step 3: Enable streaming
    if (!enableStreaming(fd, true)) {
        return false;
    }

    // Give sensor time to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return true;
}

void UvcControl::shutdown(int fd) {
    enableStreaming(fd, false);
    disableIlluminator(fd);
}

// ============================================================================
// DeviceEnumerator Implementation
// ============================================================================

bool DeviceEnumerator::probeDevice(const std::string& path, DeviceInfo& info) {
    int fd = open(path.c_str(), O_RDWR);
    if (fd < 0) {
        return false;
    }

    info.device_path = path;
    info.is_cubeeye = false;

    // Extract device index from path (e.g., /dev/video0 -> 0)
    size_t pos = path.find_last_not_of("0123456789");
    if (pos != std::string::npos && pos + 1 < path.length()) {
        info.device_index = std::stoi(path.substr(pos + 1));
    } else {
        info.device_index = -1;
    }

    // Query V4L2 capabilities
    struct v4l2_capability cap = {};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        close(fd);
        return false;
    }

    info.card_name = reinterpret_cast<const char*>(cap.card);

    // Check if it's a video capture device
    if (!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) {
        close(fd);
        return false;
    }

    // Check card name for CubeEye
    std::string card_lower = info.card_name;
    std::transform(card_lower.begin(), card_lower.end(), card_lower.begin(), ::tolower);

    if (card_lower.find("cube") == std::string::npos &&
        card_lower.find("meere") == std::string::npos) {
        // Not obviously a CubeEye, but try UVC query anyway
    }

    // Try to query serial number via UVC XU
    std::string serial = UvcControl::querySerial(fd);
    if (!serial.empty()) {
        info.serial_number = serial;
        info.is_cubeeye = true;

        // Determine model from serial prefix
        if (serial.find("I200D") != std::string::npos) {
            info.model = "I200D";
        } else if (serial.find("S100D") != std::string::npos) {
            info.model = "S100D";
        } else if (serial.find("S110D") != std::string::npos) {
            info.model = "S110D";
        } else {
            info.model = "CubeEye";
        }

        // Query firmware version
        info.firmware_version = UvcControl::queryFirmwareVersion(fd);
    }

    close(fd);
    return info.is_cubeeye;
}

std::vector<DeviceInfo> DeviceEnumerator::enumerate() {
    std::vector<DeviceInfo> devices;

    // Scan /dev/video* devices
    for (int i = 0; i < 64; i++) {
        std::string path = "/dev/video" + std::to_string(i);

        // Check if device exists
        if (access(path.c_str(), F_OK) != 0) {
            continue;
        }

        DeviceInfo info;
        if (probeDevice(path, info)) {
            devices.push_back(info);
        }
    }

    // Sort by device index
    std::sort(devices.begin(), devices.end(),
              [](const DeviceInfo& a, const DeviceInfo& b) {
                  return a.device_index < b.device_index;
              });

    return devices;
}

std::optional<DeviceInfo> DeviceEnumerator::findBySerial(const std::string& serial) {
    if (serial.empty()) {
        return std::nullopt;
    }

    auto devices = enumerate();
    for (const auto& dev : devices) {
        if (dev.serial_number == serial) {
            return dev;
        }
    }

    return std::nullopt;
}

std::optional<DeviceInfo> DeviceEnumerator::findByPath(const std::string& path) {
    DeviceInfo info;
    if (probeDevice(path, info)) {
        return info;
    }
    return std::nullopt;
}

std::optional<DeviceInfo> DeviceEnumerator::findFirst() {
    auto devices = enumerate();
    if (!devices.empty()) {
        return devices[0];
    }
    return std::nullopt;
}

int DeviceEnumerator::count() {
    return static_cast<int>(enumerate().size());
}

} // namespace cubeeye

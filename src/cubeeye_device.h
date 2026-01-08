/**
 * cubeeye_device.h - CubeEye I200D Device Enumeration
 *
 * Provides device discovery and serial number matching for CubeEye ToF cameras.
 * Used by both GStreamer elements and standalone capture tools.
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#ifndef CUBEEYE_DEVICE_H
#define CUBEEYE_DEVICE_H

#include <string>
#include <vector>
#include <optional>
#include <cstdint>

namespace cubeeye {

/**
 * Information about a discovered CubeEye device
 */
struct DeviceInfo {
    std::string device_path;      // "/dev/video0"
    int device_index;             // 0
    std::string serial_number;    // "I200DU2427000032"
    std::string firmware_version; // "1.2.3" or hex version
    std::string model;            // "I200D"
    std::string card_name;        // V4L2 card name
    bool is_cubeeye;              // true if verified CubeEye device
};

/**
 * UVC Extension Unit control interface
 */
class UvcControl {
public:
    // UVC XU Selector IDs (from reverse engineering)
    static constexpr int UNIT_ID = 3;
    static constexpr int SELECTOR_INFO = 1;       // Device info queries
    static constexpr int SELECTOR_REG = 2;        // Register read/write
    static constexpr int SELECTOR_CAL_WRITE = 3;  // Calibration write
    static constexpr int SELECTOR_CAL_READ = 4;   // Calibration read
    static constexpr int SELECTOR_STATUS = 5;     // Status/control

    // Info selector commands
    static constexpr uint8_t INFO_CMD_QUERY = 0x02;
    static constexpr uint8_t INFO_SUBCMD_SERIAL = 0x01;
    static constexpr uint8_t INFO_SUBCMD_FIRMWARE = 0x03;

    /**
     * Query a UVC XU control
     * @param fd Open file descriptor
     * @param selector XU selector ID
     * @param query UVC query type (GET_CUR, SET_CUR, etc.)
     * @param data Buffer for data
     * @param size Size of data buffer
     * @return 0 on success, -1 on error
     */
    static int query(int fd, int selector, int query_type, uint8_t* data, int size);

    /**
     * Get control length for a selector
     * @param fd Open file descriptor
     * @param selector XU selector ID
     * @return Control length in bytes, or -1 on error
     */
    static int getLength(int fd, int selector);

    /**
     * Query device serial number
     * @param fd Open file descriptor
     * @return Serial number string, or empty on error
     */
    static std::string querySerial(int fd);

    /**
     * Query firmware version
     * @param fd Open file descriptor
     * @return Firmware version string, or empty on error
     */
    static std::string queryFirmwareVersion(int fd);

    /**
     * Enable sensor streaming
     * @param fd Open file descriptor
     * @param enable true to enable, false to disable
     * @return true on success
     */
    static bool enableStreaming(int fd, bool enable);

    /**
     * Enable sensor/illuminator
     * @param fd Open file descriptor
     * @return true on success
     */
    static bool enableSensor(int fd);

    /**
     * Disable illuminator (for shutdown)
     * @param fd Open file descriptor
     * @return true on success
     */
    static bool disableIlluminator(int fd);

    /**
     * Full sensor initialization sequence
     * @param fd Open file descriptor
     * @return true on success
     */
    static bool initialize(int fd);

    /**
     * Full sensor shutdown sequence
     * @param fd Open file descriptor
     */
    static void shutdown(int fd);
};

/**
 * Device enumeration and management
 */
class DeviceEnumerator {
public:
    /**
     * Enumerate all CubeEye devices on the system
     * @return Vector of discovered devices
     */
    static std::vector<DeviceInfo> enumerate();

    /**
     * Find a device by serial number
     * @param serial Serial number to search for
     * @return DeviceInfo if found, nullopt otherwise
     */
    static std::optional<DeviceInfo> findBySerial(const std::string& serial);

    /**
     * Find a device by device path
     * @param path Device path (e.g., "/dev/video0")
     * @return DeviceInfo if found and is CubeEye, nullopt otherwise
     */
    static std::optional<DeviceInfo> findByPath(const std::string& path);

    /**
     * Find first available CubeEye device
     * @return DeviceInfo if any CubeEye found, nullopt otherwise
     */
    static std::optional<DeviceInfo> findFirst();

    /**
     * Get number of CubeEye devices
     * @return Count of CubeEye devices
     */
    static int count();

private:
    /**
     * Check if a V4L2 device is a CubeEye and get its info
     * @param path Device path
     * @param info Output device info
     * @return true if device is a CubeEye
     */
    static bool probeDevice(const std::string& path, DeviceInfo& info);
};

} // namespace cubeeye

#endif // CUBEEYE_DEVICE_H

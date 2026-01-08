/**
 * cubeeye.cpp - CubeEye I200D Multi-purpose CLI Tool
 *
 * Comprehensive tool for device discovery, calibration extraction,
 * intrinsics query, FPPN export, and sensor diagnostics.
 *
 * Usage:
 *   cubeeye list [--json]
 *   cubeeye info [--device <path>|--serial <sn>]
 *   cubeeye intrinsics [--device <path>|--serial <sn>] [--json]
 *   cubeeye calibration [--device <path>|--serial <sn>] --page <hex> [--export <file>]
 *   cubeeye fppn [--device <path>|--serial <sn>] --export <file>
 *   cubeeye register [--device <path>|--serial <sn>] --read <addr>
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <vector>
#include <map>
#include <optional>
#include <chrono>
#include <thread>
#include <algorithm>
#include <numeric>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>

#include "../src/cubeeye_device.h"

// ============================================================================
// Calibration Page Reader
// ============================================================================

class CalibrationReader {
public:
    static constexpr int XU_UNIT = 3;
    static constexpr int XU_SELECTOR_CAL = 4;
    static constexpr int PAGE_SIZE = 260;

    // Known page types
    enum PageType {
        PAGE_DEVICE_INFO = 0x0000,
        PAGE_LENS_CALIB = 0x0005,
        PAGE_SENSOR_CONFIG_1 = 0x0082,
        PAGE_SENSOR_CONFIG_2 = 0x0083,
        PAGE_SENSOR_CONFIG_3 = 0x0084,
        PAGE_DEPTH_GRADIENT_START = 0x010C,
        PAGE_DEPTH_GRADIENT_END = 0x016A,
        PAGE_FPPN_START = 0x021C,
        PAGE_FPPN_END = 0x0716,
    };

    struct LensIntrinsics {
        float fx, fy;       // Focal length
        float cx, cy;       // Principal point
        float k1, k2, p1;   // Distortion coefficients
        float p2, k3;       // Additional distortion (may be zero)
        bool valid;
    };

    struct DeviceInfoEx {
        std::string product;
        std::string module_sn;
        std::string unit_sn;
        bool valid;
    };

    static bool queryPage(int fd, uint16_t page_num, uint8_t* data, int size = PAGE_SIZE) {
        uint8_t request[PAGE_SIZE] = {0};

        // Set up page request: 00 20 HH LL (big-endian page number)
        request[0] = 0x00;
        request[1] = 0x20;
        request[2] = (page_num >> 8) & 0xFF;  // High byte
        request[3] = page_num & 0xFF;         // Low byte

        struct uvc_xu_control_query xu_set = {};
        xu_set.unit = XU_UNIT;
        xu_set.selector = XU_SELECTOR_CAL;
        xu_set.query = 0x01;  // SET_CUR
        xu_set.size = PAGE_SIZE;
        xu_set.data = request;

        if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_set) < 0) {
            return false;
        }

        // Small delay for device to process
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // Get the response
        struct uvc_xu_control_query xu_get = {};
        xu_get.unit = XU_UNIT;
        xu_get.selector = XU_SELECTOR_CAL;
        xu_get.query = 0x81;  // GET_CUR
        xu_get.size = PAGE_SIZE;
        xu_get.data = data;

        if (ioctl(fd, UVCIOC_CTRL_QUERY, &xu_get) < 0) {
            return false;
        }

        // Check if page is valid (header should be 00 20)
        if (data[0] != 0x00 || data[1] != 0x20) {
            return false;
        }

        return true;
    }

    static LensIntrinsics readIntrinsics(int fd) {
        LensIntrinsics result = {};
        result.valid = false;

        uint8_t page_data[PAGE_SIZE];
        if (!queryPage(fd, PAGE_LENS_CALIB, page_data)) {
            return result;
        }

        // Intrinsics start at offset 4 (after 4-byte header)
        const float* floats = reinterpret_cast<const float*>(&page_data[4]);

        result.fx = floats[0];
        result.fy = floats[1];
        result.cx = floats[2];
        result.cy = floats[3];
        result.k1 = floats[4];
        result.k2 = floats[5];
        result.p1 = floats[6];
        result.p2 = (7 * sizeof(float) + 4 < PAGE_SIZE) ? floats[7] : 0.0f;
        result.k3 = (8 * sizeof(float) + 4 < PAGE_SIZE) ? floats[8] : 0.0f;
        result.valid = true;

        return result;
    }

    static DeviceInfoEx readDeviceInfoEx(int fd) {
        DeviceInfoEx result = {};
        result.valid = false;

        uint8_t page_data[PAGE_SIZE];
        if (!queryPage(fd, PAGE_DEVICE_INFO, page_data)) {
            return result;
        }

        // Parse device info page
        // Product name at offset 4
        char product[32] = {0};
        std::memcpy(product, &page_data[4], 16);
        result.product = product;

        // Module serial at offset 20
        char module_sn[32] = {0};
        std::memcpy(module_sn, &page_data[20], 16);
        result.module_sn = module_sn;

        // Unit serial at offset 36
        char unit_sn[32] = {0};
        std::memcpy(unit_sn, &page_data[36], 16);
        result.unit_sn = unit_sn;

        result.valid = true;
        return result;
    }

    static bool pageHasData(const uint8_t* data) {
        for (int i = 4; i < PAGE_SIZE; i++) {
            if (data[i] != 0) return true;
        }
        return false;
    }
};

// ============================================================================
// Command Handlers
// ============================================================================

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " <command> [options]\n"
              << "\n"
              << "Commands:\n"
              << "  list                     List all CubeEye devices\n"
              << "  info                     Show detailed device info\n"
              << "  intrinsics               Show camera intrinsics (lens calibration)\n"
              << "  calibration              Read calibration page(s)\n"
              << "  fppn                     Export FPPN correction data\n"
              << "  register                 Read/write sensor registers\n"
              << "\n"
              << "Common Options:\n"
              << "  --device <path>          Device path (e.g., /dev/video0)\n"
              << "  --serial <sn>            Device serial number\n"
              << "  --json                   Output in JSON format\n"
              << "  --help                   Show this help\n"
              << "\n"
              << "Command-specific Options:\n"
              << "  calibration:\n"
              << "    --page <hex>           Page number (e.g., 0x0005)\n"
              << "    --range <start>-<end>  Page range (e.g., 0x0000-0x00FF)\n"
              << "    --export <file>        Export to binary file\n"
              << "\n"
              << "  fppn:\n"
              << "    --export <file>        Export FPPN to file (required)\n"
              << "\n"
              << "  register:\n"
              << "    --read <addr>          Read register at address (hex)\n"
              << "\n"
              << "Examples:\n"
              << "  " << prog << " list\n"
              << "  " << prog << " info --serial I200DU2509000349\n"
              << "  " << prog << " intrinsics --json\n"
              << "  " << prog << " calibration --page 0x0005\n"
              << "  " << prog << " fppn --export fppn_data.bin\n"
              << std::endl;
}

int openDevice(const std::string& device_path, const std::string& serial) {
    std::optional<cubeeye::DeviceInfo> dev;

    if (!serial.empty()) {
        dev = cubeeye::DeviceEnumerator::findBySerial(serial);
        if (!dev) {
            std::cerr << "Error: Device with serial '" << serial << "' not found\n";
            return -1;
        }
    } else if (!device_path.empty()) {
        dev = cubeeye::DeviceEnumerator::findByPath(device_path);
        if (!dev) {
            std::cerr << "Error: Device '" << device_path << "' is not a CubeEye\n";
            return -1;
        }
    } else {
        dev = cubeeye::DeviceEnumerator::findFirst();
        if (!dev) {
            std::cerr << "Error: No CubeEye device found\n";
            return -1;
        }
    }

    int fd = open(dev->device_path.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Error: Cannot open " << dev->device_path << ": " << strerror(errno) << "\n";
        return -1;
    }

    return fd;
}

int cmdList(int argc, char* argv[]) {
    bool json_output = false;

    for (int i = 2; i < argc; i++) {
        if (std::strcmp(argv[i], "--json") == 0) {
            json_output = true;
        }
    }

    auto devices = cubeeye::DeviceEnumerator::enumerate();

    if (json_output) {
        std::cout << "{\n  \"count\": " << devices.size() << ",\n  \"devices\": [\n";
        for (size_t i = 0; i < devices.size(); i++) {
            const auto& dev = devices[i];
            std::cout << "    {\n"
                      << "      \"device_path\": \"" << dev.device_path << "\",\n"
                      << "      \"serial_number\": \"" << dev.serial_number << "\",\n"
                      << "      \"model\": \"" << dev.model << "\",\n"
                      << "      \"firmware_version\": \"" << dev.firmware_version << "\"\n"
                      << "    }" << (i < devices.size() - 1 ? "," : "") << "\n";
        }
        std::cout << "  ]\n}\n";
    } else {
        if (devices.empty()) {
            std::cout << "No CubeEye devices found.\n";
            return 1;
        }
        std::cout << "\nFound " << devices.size() << " CubeEye device(s):\n\n";
        std::cout << std::left
                  << std::setw(14) << "Device"
                  << std::setw(20) << "Serial"
                  << std::setw(8) << "Model"
                  << std::setw(10) << "Firmware"
                  << "\n" << std::string(52, '-') << "\n";
        for (const auto& dev : devices) {
            std::cout << std::left
                      << std::setw(14) << dev.device_path
                      << std::setw(20) << dev.serial_number
                      << std::setw(8) << dev.model
                      << std::setw(10) << dev.firmware_version
                      << "\n";
        }
    }
    return 0;
}

int cmdInfo(int argc, char* argv[]) {
    std::string device_path, serial;
    bool json_output = false;

    for (int i = 2; i < argc; i++) {
        if (std::strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            device_path = argv[++i];
        } else if (std::strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
            serial = argv[++i];
        } else if (std::strcmp(argv[i], "--json") == 0) {
            json_output = true;
        }
    }

    int fd = openDevice(device_path, serial);
    if (fd < 0) return 1;

    // Get basic info from our library
    auto dev = serial.empty()
        ? (device_path.empty() ? cubeeye::DeviceEnumerator::findFirst()
                               : cubeeye::DeviceEnumerator::findByPath(device_path))
        : cubeeye::DeviceEnumerator::findBySerial(serial);

    // Get extended info from calibration page
    auto ext_info = CalibrationReader::readDeviceInfoEx(fd);
    auto intrinsics = CalibrationReader::readIntrinsics(fd);

    close(fd);

    if (json_output) {
        std::cout << "{\n";
        std::cout << "  \"device_path\": \"" << dev->device_path << "\",\n";
        std::cout << "  \"serial_number\": \"" << dev->serial_number << "\",\n";
        std::cout << "  \"model\": \"" << dev->model << "\",\n";
        std::cout << "  \"firmware_version\": \"" << dev->firmware_version << "\",\n";
        if (ext_info.valid) {
            std::cout << "  \"product\": \"" << ext_info.product << "\",\n";
            std::cout << "  \"module_serial\": \"" << ext_info.module_sn << "\",\n";
            std::cout << "  \"unit_serial\": \"" << ext_info.unit_sn << "\",\n";
        }
        if (intrinsics.valid) {
            std::cout << "  \"intrinsics\": {\n";
            std::cout << "    \"fx\": " << intrinsics.fx << ",\n";
            std::cout << "    \"fy\": " << intrinsics.fy << ",\n";
            std::cout << "    \"cx\": " << intrinsics.cx << ",\n";
            std::cout << "    \"cy\": " << intrinsics.cy << ",\n";
            std::cout << "    \"k1\": " << intrinsics.k1 << ",\n";
            std::cout << "    \"k2\": " << intrinsics.k2 << ",\n";
            std::cout << "    \"p1\": " << intrinsics.p1 << "\n";
            std::cout << "  }\n";
        }
        std::cout << "}\n";
    } else {
        std::cout << "\n=== CubeEye Device Information ===\n\n";
        std::cout << "Device Path:      " << dev->device_path << "\n";
        std::cout << "Serial Number:    " << dev->serial_number << "\n";
        std::cout << "Model:            " << dev->model << "\n";
        std::cout << "Firmware:         " << dev->firmware_version << "\n";
        if (ext_info.valid) {
            std::cout << "\n--- Extended Info (from calibration) ---\n";
            std::cout << "Product:          " << ext_info.product << "\n";
            std::cout << "Module Serial:    " << ext_info.module_sn << "\n";
            std::cout << "Unit Serial:      " << ext_info.unit_sn << "\n";
        }
        if (intrinsics.valid) {
            std::cout << "\n--- Lens Intrinsics ---\n";
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "Focal Length:     fx=" << intrinsics.fx << ", fy=" << intrinsics.fy << "\n";
            std::cout << "Principal Point:  cx=" << intrinsics.cx << ", cy=" << intrinsics.cy << "\n";
            std::cout << "Radial Dist:      k1=" << intrinsics.k1 << ", k2=" << intrinsics.k2 << "\n";
            std::cout << "Tangential Dist:  p1=" << intrinsics.p1 << "\n";
        }
        std::cout << std::endl;
    }
    return 0;
}

int cmdIntrinsics(int argc, char* argv[]) {
    std::string device_path, serial;
    bool json_output = false;

    for (int i = 2; i < argc; i++) {
        if (std::strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            device_path = argv[++i];
        } else if (std::strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
            serial = argv[++i];
        } else if (std::strcmp(argv[i], "--json") == 0) {
            json_output = true;
        }
    }

    int fd = openDevice(device_path, serial);
    if (fd < 0) return 1;

    auto intrinsics = CalibrationReader::readIntrinsics(fd);
    close(fd);

    if (!intrinsics.valid) {
        std::cerr << "Error: Failed to read intrinsics\n";
        return 1;
    }

    if (json_output) {
        std::cout << "{\n";
        std::cout << "  \"fx\": " << intrinsics.fx << ",\n";
        std::cout << "  \"fy\": " << intrinsics.fy << ",\n";
        std::cout << "  \"cx\": " << intrinsics.cx << ",\n";
        std::cout << "  \"cy\": " << intrinsics.cy << ",\n";
        std::cout << "  \"k1\": " << intrinsics.k1 << ",\n";
        std::cout << "  \"k2\": " << intrinsics.k2 << ",\n";
        std::cout << "  \"p1\": " << intrinsics.p1 << ",\n";
        std::cout << "  \"p2\": " << intrinsics.p2 << ",\n";
        std::cout << "  \"k3\": " << intrinsics.k3 << ",\n";
        std::cout << "  \"camera_matrix\": [[" << intrinsics.fx << ", 0, " << intrinsics.cx << "], "
                  << "[0, " << intrinsics.fy << ", " << intrinsics.cy << "], [0, 0, 1]],\n";
        std::cout << "  \"distortion_coeffs\": [" << intrinsics.k1 << ", " << intrinsics.k2
                  << ", " << intrinsics.p1 << ", " << intrinsics.p2 << ", " << intrinsics.k3 << "]\n";
        std::cout << "}\n";
    } else {
        std::cout << "\n=== Camera Intrinsics ===\n\n";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Focal Length:\n";
        std::cout << "  fx = " << intrinsics.fx << "\n";
        std::cout << "  fy = " << intrinsics.fy << "\n";
        std::cout << "\nPrincipal Point:\n";
        std::cout << "  cx = " << intrinsics.cx << "\n";
        std::cout << "  cy = " << intrinsics.cy << "\n";
        std::cout << "\nDistortion Coefficients:\n";
        std::cout << "  k1 = " << intrinsics.k1 << "\n";
        std::cout << "  k2 = " << intrinsics.k2 << "\n";
        std::cout << "  p1 = " << intrinsics.p1 << "\n";
        std::cout << "  p2 = " << intrinsics.p2 << "\n";
        std::cout << "  k3 = " << intrinsics.k3 << "\n";
        std::cout << "\nOpenCV Format:\n";
        std::cout << "  K = [[" << intrinsics.fx << ", 0, " << intrinsics.cx << "],\n"
                  << "       [0, " << intrinsics.fy << ", " << intrinsics.cy << "],\n"
                  << "       [0, 0, 1]]\n";
        std::cout << "  D = [" << intrinsics.k1 << ", " << intrinsics.k2
                  << ", " << intrinsics.p1 << ", " << intrinsics.p2 << ", " << intrinsics.k3 << "]\n";
        std::cout << std::endl;
    }
    return 0;
}

int cmdCalibration(int argc, char* argv[]) {
    std::string device_path, serial, export_file;
    int page = -1;
    int range_start = -1, range_end = -1;

    for (int i = 2; i < argc; i++) {
        if (std::strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            device_path = argv[++i];
        } else if (std::strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
            serial = argv[++i];
        } else if (std::strcmp(argv[i], "--page") == 0 && i + 1 < argc) {
            page = std::stoi(argv[++i], nullptr, 0);
        } else if (std::strcmp(argv[i], "--range") == 0 && i + 1 < argc) {
            std::string range = argv[++i];
            size_t dash = range.find('-');
            if (dash != std::string::npos) {
                range_start = std::stoi(range.substr(0, dash), nullptr, 0);
                range_end = std::stoi(range.substr(dash + 1), nullptr, 0);
            }
        } else if (std::strcmp(argv[i], "--export") == 0 && i + 1 < argc) {
            export_file = argv[++i];
        }
    }

    if (page < 0 && range_start < 0) {
        std::cerr << "Error: Specify --page or --range\n";
        return 1;
    }

    int fd = openDevice(device_path, serial);
    if (fd < 0) return 1;

    std::vector<uint8_t> all_data;

    if (page >= 0) {
        uint8_t page_data[CalibrationReader::PAGE_SIZE];
        if (CalibrationReader::queryPage(fd, page, page_data)) {
            std::cout << "Page 0x" << std::hex << std::setw(4) << std::setfill('0') << page << ":\n";
            for (int i = 0; i < CalibrationReader::PAGE_SIZE; i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(page_data[i]) << " ";
                if ((i + 1) % 16 == 0) std::cout << "\n";
            }
            if (!export_file.empty()) {
                std::ofstream out(export_file, std::ios::binary);
                out.write(reinterpret_cast<char*>(page_data), CalibrationReader::PAGE_SIZE);
                std::cout << "\nExported to: " << export_file << "\n";
            }
        } else {
            std::cerr << "Failed to read page 0x" << std::hex << page << "\n";
        }
    } else {
        int valid_count = 0;
        for (int p = range_start; p <= range_end; p++) {
            uint8_t page_data[CalibrationReader::PAGE_SIZE];
            if (CalibrationReader::queryPage(fd, p, page_data) &&
                CalibrationReader::pageHasData(page_data)) {
                valid_count++;
                std::cout << "Page 0x" << std::hex << std::setw(4) << std::setfill('0')
                          << p << ": " << std::dec << (CalibrationReader::PAGE_SIZE - 4)
                          << " bytes\n";
                all_data.insert(all_data.end(), page_data, page_data + CalibrationReader::PAGE_SIZE);
            }
        }
        std::cout << "\nFound " << valid_count << " valid pages\n";
        if (!export_file.empty() && !all_data.empty()) {
            std::ofstream out(export_file, std::ios::binary);
            out.write(reinterpret_cast<char*>(all_data.data()), all_data.size());
            std::cout << "Exported " << all_data.size() << " bytes to: " << export_file << "\n";
        }
    }

    close(fd);
    return 0;
}

int cmdFppn(int argc, char* argv[]) {
    std::string device_path, serial, export_file;

    for (int i = 2; i < argc; i++) {
        if (std::strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            device_path = argv[++i];
        } else if (std::strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
            serial = argv[++i];
        } else if (std::strcmp(argv[i], "--export") == 0 && i + 1 < argc) {
            export_file = argv[++i];
        }
    }

    if (export_file.empty()) {
        std::cerr << "Error: --export <file> is required\n";
        return 1;
    }

    int fd = openDevice(device_path, serial);
    if (fd < 0) return 1;

    std::cout << "Extracting FPPN data from pages 0x021C-0x0716...\n";

    // FPPN is stored in pages 0x021C to 0x0716 (599 pages)
    // Each page has 254 bytes of data = 127 int16 values
    // Total: ~76,000 values for 320x240 image
    std::vector<int16_t> fppn_data;

    for (uint16_t page = 0x021C; page <= 0x0716; page += 2) {  // Even pages only
        uint8_t page_data[CalibrationReader::PAGE_SIZE];
        if (CalibrationReader::queryPage(fd, page, page_data)) {
            // Extract 127 int16 values from data portion (bytes 6-259)
            const int16_t* values = reinterpret_cast<const int16_t*>(&page_data[6]);
            for (int i = 0; i < 127; i++) {
                fppn_data.push_back(values[i]);
            }
        }
        // Progress indicator
        if ((page - 0x021C) % 100 == 0) {
            std::cout << "." << std::flush;
        }
    }

    close(fd);

    std::cout << "\nExtracted " << fppn_data.size() << " values\n";

    // Trim to 320x240 = 76800 values
    if (fppn_data.size() > 76800) {
        fppn_data.resize(76800);
    }

    // Pad if necessary
    while (fppn_data.size() < 76800) {
        // Use mean of last valid values for padding
        int16_t fill_val = fppn_data.empty() ? 0 : fppn_data.back();
        fppn_data.push_back(fill_val);
    }

    // Export
    std::ofstream out(export_file, std::ios::binary);
    out.write(reinterpret_cast<char*>(fppn_data.data()), fppn_data.size() * sizeof(int16_t));
    out.close();

    std::cout << "Exported 320x240 FPPN (" << (fppn_data.size() * 2) << " bytes) to: "
              << export_file << "\n";

    // Stats
    int16_t min_val = *std::min_element(fppn_data.begin(), fppn_data.end());
    int16_t max_val = *std::max_element(fppn_data.begin(), fppn_data.end());
    double sum = 0;
    for (auto v : fppn_data) sum += v;
    double mean = sum / fppn_data.size();

    std::cout << "Stats: min=" << min_val << ", max=" << max_val
              << ", mean=" << std::fixed << std::setprecision(1) << mean << "\n";

    return 0;
}

int cmdRegister(int argc, char* argv[]) {
    std::string device_path, serial;
    int read_addr = -1;

    for (int i = 2; i < argc; i++) {
        if (std::strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            device_path = argv[++i];
        } else if (std::strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
            serial = argv[++i];
        } else if (std::strcmp(argv[i], "--read") == 0 && i + 1 < argc) {
            read_addr = std::stoi(argv[++i], nullptr, 0);
        }
    }

    if (read_addr < 0) {
        std::cerr << "Error: Specify --read <addr>\n";
        return 1;
    }

    int fd = openDevice(device_path, serial);
    if (fd < 0) return 1;

    // Read register via XU Selector 2
    int len = cubeeye::UvcControl::getLength(fd, cubeeye::UvcControl::SELECTOR_REG);
    if (len <= 0) len = 20;

    std::vector<uint8_t> cmd(len, 0);
    cmd[0] = 0x00;  // Read command
    cmd[1] = read_addr & 0xFF;
    cmd[2] = (read_addr >> 8) & 0xFF;

    if (cubeeye::UvcControl::query(fd, cubeeye::UvcControl::SELECTOR_REG, 0x01, cmd.data(), len) < 0) {
        std::cerr << "Error: SET_CUR failed\n";
        close(fd);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::fill(cmd.begin(), cmd.end(), 0);
    if (cubeeye::UvcControl::query(fd, cubeeye::UvcControl::SELECTOR_REG, 0x81, cmd.data(), len) < 0) {
        std::cerr << "Error: GET_CUR failed\n";
        close(fd);
        return 1;
    }

    std::cout << "Register 0x" << std::hex << std::setw(4) << std::setfill('0') << read_addr << ":\n";
    for (int i = 0; i < len; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(cmd[i]) << " ";
        if ((i + 1) % 16 == 0) std::cout << "\n";
    }
    std::cout << std::dec << std::endl;

    close(fd);
    return 0;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string cmd = argv[1];

    if (cmd == "--help" || cmd == "-h") {
        printUsage(argv[0]);
        return 0;
    } else if (cmd == "list") {
        return cmdList(argc, argv);
    } else if (cmd == "info") {
        return cmdInfo(argc, argv);
    } else if (cmd == "intrinsics") {
        return cmdIntrinsics(argc, argv);
    } else if (cmd == "calibration") {
        return cmdCalibration(argc, argv);
    } else if (cmd == "fppn") {
        return cmdFppn(argc, argv);
    } else if (cmd == "register") {
        return cmdRegister(argc, argv);
    } else {
        std::cerr << "Unknown command: " << cmd << "\n";
        printUsage(argv[0]);
        return 1;
    }
}

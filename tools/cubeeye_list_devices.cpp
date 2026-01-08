/**
 * cubeeye_list_devices.cpp - List all CubeEye devices
 *
 * Enumerates all connected CubeEye ToF cameras and displays their
 * serial numbers, device paths, and firmware versions.
 *
 * Usage:
 *   cubeeye_list_devices [--json]
 *
 * Copyright (c) 2025 Atlas Robotics Inc.
 */

#include <iostream>
#include <iomanip>
#include <cstring>
#include "../src/cubeeye_device.h"

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS]\n"
              << "\n"
              << "Options:\n"
              << "  --json     Output in JSON format\n"
              << "  --help     Show this help message\n"
              << std::endl;
}

void print_table(const std::vector<cubeeye::DeviceInfo>& devices) {
    if (devices.empty()) {
        std::cout << "No CubeEye devices found.\n"
                  << "\nTips:\n"
                  << "  - Check USB connection\n"
                  << "  - Verify device permissions: ls -la /dev/video*\n"
                  << "  - Add user to 'video' group: sudo usermod -aG video $USER\n"
                  << std::endl;
        return;
    }

    std::cout << "\nFound " << devices.size() << " CubeEye device(s):\n\n";

    // Header
    std::cout << std::left
              << std::setw(6) << "Index"
              << std::setw(16) << "Device"
              << std::setw(20) << "Serial"
              << std::setw(10) << "Model"
              << std::setw(12) << "Firmware"
              << std::endl;

    std::cout << std::string(64, '-') << std::endl;

    // Rows
    for (size_t i = 0; i < devices.size(); i++) {
        const auto& dev = devices[i];
        std::cout << std::left
                  << std::setw(6) << i
                  << std::setw(16) << dev.device_path
                  << std::setw(20) << dev.serial_number
                  << std::setw(10) << dev.model
                  << std::setw(12) << dev.firmware_version
                  << std::endl;
    }

    std::cout << std::endl;
}

void print_json(const std::vector<cubeeye::DeviceInfo>& devices) {
    std::cout << "{\n";
    std::cout << "  \"count\": " << devices.size() << ",\n";
    std::cout << "  \"devices\": [\n";

    for (size_t i = 0; i < devices.size(); i++) {
        const auto& dev = devices[i];
        std::cout << "    {\n";
        std::cout << "      \"index\": " << i << ",\n";
        std::cout << "      \"device_path\": \"" << dev.device_path << "\",\n";
        std::cout << "      \"device_index\": " << dev.device_index << ",\n";
        std::cout << "      \"serial_number\": \"" << dev.serial_number << "\",\n";
        std::cout << "      \"model\": \"" << dev.model << "\",\n";
        std::cout << "      \"firmware_version\": \"" << dev.firmware_version << "\",\n";
        std::cout << "      \"card_name\": \"" << dev.card_name << "\"\n";
        std::cout << "    }" << (i < devices.size() - 1 ? "," : "") << "\n";
    }

    std::cout << "  ]\n";
    std::cout << "}\n";
}

int main(int argc, char* argv[]) {
    bool json_output = false;

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--json") == 0) {
            json_output = true;
        } else if (std::strcmp(argv[i], "--help") == 0 ||
                   std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << argv[i] << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    auto devices = cubeeye::DeviceEnumerator::enumerate();

    if (json_output) {
        print_json(devices);
    } else {
        print_table(devices);
    }

    return devices.empty() ? 1 : 0;
}

/**
 * benchmark_raw_capture.cpp - Raw SDK Capture with All Filters Disabled
 *
 * Captures depth/amplitude frames with SDK filters disabled to get
 * true raw output for comparison with our custom driver.
 *
 * Usage: ./benchmark_raw_capture [num_frames] [output_dir]
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstring>
#include <csignal>
#include <sys/stat.h>

#include "CubeEye/CubeEyeSink.h"
#include "CubeEye/CubeEyeCamera.h"
#include "CubeEye/CubeEyeBasicFrame.h"
#include "CubeEye/CubeEyeProperty.h"

namespace ms = meere::sensor;

static std::atomic<bool> g_running{true};
static std::atomic<int> g_frame_count{0};
static int g_target_frames = 10;
static std::string g_output_dir = "raw_benchmark";

void signal_handler(int sig) {
    std::cout << "\nReceived signal " << sig << ", stopping..." << std::endl;
    g_running = false;
}

void ensure_dir(const std::string& path) {
    mkdir(path.c_str(), 0755);
}

// Disable all SDK filters
void disable_all_filters(ms::ptr_camera camera) {
    std::cout << "\n=== Disabling All SDK Filters ===" << std::endl;

    // List of boolean filter properties to disable
    const char* bool_filters[] = {
        "flying_pixel_remove_filter",
        "flying_filter",
        "depth_time_filter",
        "amplitude_time_filter",
        "dead_pixel_remove_filter",
        "outlier_remove_filter",
        "hybrid_median_filter",
        "depth_average_median_filter",
        "phase_noise_filter",
        nullptr
    };

    for (const char** filter = bool_filters; *filter != nullptr; ++filter) {
        auto prop = ms::make_property_bool(*filter, false);
        auto result = camera->setProperty(prop);
        if (result == ms::result::success) {
            std::cout << "  Disabled: " << *filter << std::endl;
        } else {
            std::cout << "  Failed to disable: " << *filter << " (may not exist)" << std::endl;
        }
    }

    // Set amplitude threshold to minimum (0) to not reject any pixels
    {
        auto prop = ms::make_property_16u("amplitude_threshold_min", 0);
        auto result = camera->setProperty(prop);
        std::cout << "  amplitude_threshold_min = 0: "
                  << (result == ms::result::success ? "OK" : "FAILED") << std::endl;
    }

    // Set amplitude threshold max to maximum
    {
        auto prop = ms::make_property_16u("amplitude_threshold_max", 65535);
        auto result = camera->setProperty(prop);
        std::cout << "  amplitude_threshold_max = 65535: "
                  << (result == ms::result::success ? "OK" : "FAILED") << std::endl;
    }

    // Disable scattering filter
    {
        auto prop = ms::make_property_16u("scattering_threshold", 0);
        auto result = camera->setProperty(prop);
        std::cout << "  scattering_threshold = 0: "
                  << (result == ms::result::success ? "OK" : "FAILED") << std::endl;
    }

    // Disable phase filter
    {
        auto prop = ms::make_property_8u("phase_filter_level", 0);
        auto result = camera->setProperty(prop);
        std::cout << "  phase_filter_level = 0: "
                  << (result == ms::result::success ? "OK" : "FAILED") << std::endl;
    }

    std::cout << "=================================\n" << std::endl;
}

// Print current filter settings
void print_filter_settings(ms::ptr_camera camera) {
    std::cout << "\n=== Current Filter Settings ===" << std::endl;

    const char* props[] = {
        "flying_pixel_remove_filter",
        "depth_time_filter",
        "amplitude_time_filter",
        "dead_pixel_remove_filter",
        "outlier_remove_filter",
        "hybrid_median_filter",
        "amplitude_threshold_min",
        "amplitude_threshold_max",
        "scattering_threshold",
        "phase_filter_level",
        nullptr
    };

    for (const char** prop_name = props; *prop_name != nullptr; ++prop_name) {
        auto [result, prop] = camera->getProperty(*prop_name);
        if (result == ms::result::success && prop) {
            // Try to get value as different types
            bool bool_val = prop->asBoolean(false);
            int16_t int_val = prop->asInt16u(0);
            std::cout << "  " << *prop_name << " = " << int_val
                      << " (bool: " << (bool_val ? "true" : "false") << ")" << std::endl;
        } else {
            std::cout << "  " << *prop_name << " = <not available>" << std::endl;
        }
    }

    std::cout << "================================\n" << std::endl;
}

class RawCaptureSink : public ms::sink, public ms::prepared_listener {
public:
    std::string name() const override {
        return "RawCaptureSink";
    }

    void onCubeEyeCameraState(const ms::ptr_source source, ms::CameraState state) override {
        const char* state_str = "Unknown";
        switch (state) {
            case ms::CameraState::Released: state_str = "Released"; break;
            case ms::CameraState::Prepared: state_str = "Prepared"; break;
            case ms::CameraState::Stopped: state_str = "Stopped"; break;
            case ms::CameraState::Running: state_str = "Running"; break;
        }
        std::cout << "[State] " << state_str << std::endl;
    }

    void onCubeEyeCameraError(const ms::ptr_source source, ms::CameraError error) override {
        std::cerr << "[Error] Camera error: " << static_cast<int>(error) << std::endl;
    }

    void onCubeEyeFrameList(const ms::ptr_source source, const ms::sptr_frame_list& frames) override {
        if (!g_running || g_frame_count >= g_target_frames) {
            return;
        }

        int frame_num = g_frame_count++;

        for (const auto& frame : (*frames)) {
            int width = frame->frameWidth();
            int height = frame->frameHeight();

            if (frame->frameType() == ms::FrameType::Depth) {
                if (frame->frameDataType() == ms::DataType::U16) {
                    auto basic_frame = ms::frame_cast_basic16u(frame);
                    auto data = basic_frame->frameData();

                    // Count zeros
                    int zero_count = 0;
                    for (size_t i = 0; i < data->size(); i++) {
                        if ((*data)[i] == 0) zero_count++;
                    }

                    int center_idx = (height / 2) * width + (width / 2);
                    float center_depth = static_cast<float>((*data)[center_idx]);

                    std::cout << "[Frame " << std::setw(3) << frame_num << "] "
                              << "Depth: " << std::fixed << std::setprecision(1)
                              << center_depth << "mm, "
                              << "Size: " << width << "x" << height << ", "
                              << "Zeros: " << zero_count << "/" << data->size()
                              << std::endl;

                    // Save depth frame
                    std::ostringstream filename;
                    filename << g_output_dir << "/raw_depth_"
                             << std::setfill('0') << std::setw(4) << frame_num << ".raw";
                    std::ofstream file(filename.str(), std::ios::binary);
                    if (file) {
                        file.write(reinterpret_cast<const char*>(data->data()),
                                   data->size() * sizeof(uint16_t));
                    }
                }
            }
            else if (frame->frameType() == ms::FrameType::Amplitude) {
                if (frame->frameDataType() == ms::DataType::U16) {
                    auto basic_frame = ms::frame_cast_basic16u(frame);
                    auto data = basic_frame->frameData();

                    // Save amplitude frame
                    std::ostringstream filename;
                    filename << g_output_dir << "/raw_amp_"
                             << std::setfill('0') << std::setw(4) << frame_num << ".raw";
                    std::ofstream file(filename.str(), std::ios::binary);
                    if (file) {
                        file.write(reinterpret_cast<const char*>(data->data()),
                                   data->size() * sizeof(uint16_t));
                    }
                }
            }
        }

        if (frame_num >= g_target_frames - 1) {
            g_running = false;
        }
    }

    void onCubeEyeCameraPrepared(const ms::ptr_camera camera) override {
        std::cout << "[Prepared] Camera ready" << std::endl;
    }
};

static RawCaptureSink g_sink;

int main(int argc, char* argv[]) {
    if (argc > 1) {
        g_target_frames = std::atoi(argv[1]);
        if (g_target_frames <= 0) g_target_frames = 10;
    }
    if (argc > 2) {
        g_output_dir = argv[2];
    }

    std::cout << "========================================" << std::endl;
    std::cout << "CubeEye RAW Capture (Filters Disabled)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Target frames: " << g_target_frames << std::endl;
    std::cout << "Output dir:    " << g_output_dir << std::endl;

    ensure_dir(g_output_dir);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Search for camera
    std::cout << "\n[Search] Looking for CubeEye cameras..." << std::endl;
    auto source_list = ms::search_camera_source();

    if (!source_list || source_list->size() == 0) {
        std::cerr << "[Error] No CubeEye camera found!" << std::endl;
        return 1;
    }

    ms::sptr_source selected_source = (*source_list)[0];
    std::cout << "[Found] " << selected_source->name()
              << " (SN: " << selected_source->serialNumber() << ")" << std::endl;

    // Create camera
    auto camera = ms::create_camera(selected_source);
    if (!camera) {
        std::cerr << "[Error] Failed to create camera!" << std::endl;
        return 1;
    }

    ms::add_prepared_listener(&g_sink);
    camera->addSink(&g_sink);

    // Prepare camera
    auto result = camera->prepare();
    if (result != ms::result::success) {
        std::cerr << "[Error] Failed to prepare camera" << std::endl;
        ms::destroy_camera(camera);
        return 1;
    }

    // Print default filter settings
    print_filter_settings(camera.get());

    // DISABLE ALL FILTERS
    disable_all_filters(camera.get());

    // Print settings after disabling
    print_filter_settings(camera.get());

    // Start capture
    int wanted_frames = ms::FrameType::Depth | ms::FrameType::Amplitude;
    result = camera->run(wanted_frames);
    if (result != ms::result::success) {
        std::cerr << "[Error] Failed to start camera" << std::endl;
        ms::destroy_camera(camera);
        return 1;
    }

    // Wait for capture
    auto start_time = std::chrono::steady_clock::now();
    while (g_running && g_frame_count < g_target_frames) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 60) {
            std::cout << "[Timeout] 60 seconds reached" << std::endl;
            break;
        }
    }

    // Cleanup
    std::cout << "\n[Stop] Stopping camera..." << std::endl;
    camera->stop();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Capture Complete" << std::endl;
    std::cout << "Frames: " << g_frame_count.load() << std::endl;
    std::cout << "Output: " << g_output_dir << "/" << std::endl;
    std::cout << "========================================" << std::endl;

    ms::destroy_camera(camera);
    return 0;
}

/**
 * sdk_capture.cpp - CubeEye SDK Capture Tool
 *
 * Captures depth and amplitude frames from CubeEye ToF sensor using
 * the official SDK as ground truth reference for custom driver development.
 *
 * Usage: ./sdk_capture [num_frames]
 *   - Searches for CubeEye camera
 *   - Captures specified number of frames (default: 100)
 *   - Prints depth statistics for center pixel
 *   - Optionally saves raw frame data
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <csignal>

#include "CubeEye/CubeEyeSink.h"
#include "CubeEye/CubeEyeCamera.h"
#include "CubeEye/CubeEyeBasicFrame.h"

namespace ms = meere::sensor;

// Global state
static std::atomic<bool> g_running{true};
static std::atomic<int> g_frame_count{0};
static int g_target_frames = 100;
static std::string g_data_dir = "data";

// Statistics
static std::vector<float> g_depth_samples;
static std::vector<float> g_amplitude_samples;
static std::mutex g_stats_mutex;

// Signal handler
void signal_handler(int sig) {
    std::cout << "\nReceived signal " << sig << ", stopping..." << std::endl;
    g_running = false;
}

// Frame receiver sink
class FrameCaptureSink : public ms::sink, public ms::prepared_listener {
public:
    std::string name() const override {
        return "FrameCaptureSink";
    }

    void onCubeEyeCameraState(const ms::ptr_source source, ms::CameraState state) override {
        const char* state_str = "Unknown";
        switch (state) {
            case ms::CameraState::Released: state_str = "Released"; break;
            case ms::CameraState::Prepared: state_str = "Prepared"; break;
            case ms::CameraState::Stopped: state_str = "Stopped"; break;
            case ms::CameraState::Running: state_str = "Running"; break;
        }
        std::cout << "[State] Camera: " << state_str << std::endl;
    }

    void onCubeEyeCameraError(const ms::ptr_source source, ms::CameraError error) override {
        const char* error_str = "Unknown";
        switch (error) {
            case ms::CameraError::IO: error_str = "IO"; break;
            case ms::CameraError::AccessDenied: error_str = "AccessDenied"; break;
            case ms::CameraError::NoSuchDevice: error_str = "NoSuchDevice"; break;
            case ms::CameraError::Busy: error_str = "Busy"; break;
            case ms::CameraError::Timeout: error_str = "Timeout"; break;
            case ms::CameraError::Overflow: error_str = "Overflow"; break;
            case ms::CameraError::Interrupted: error_str = "Interrupted"; break;
            case ms::CameraError::Internal: error_str = "Internal"; break;
            case ms::CameraError::FrameDropped: error_str = "FrameDropped"; break;
            default: break;
        }
        std::cerr << "[Error] Camera error: " << error_str << std::endl;
    }

    void onCubeEyeFrameList(const ms::ptr_source source, const ms::sptr_frame_list& frames) override {
        if (!g_running || g_frame_count >= g_target_frames) {
            return;
        }

        int frame_num = g_frame_count++;

        float depth_center = 0;
        float amplitude_center = 0;
        int depth_width = 0, depth_height = 0;

        // Debug: print all frame types received on first frame
        if (frame_num == 0) {
            std::cout << "[Frame 0] Received " << frames->size() << " frame types: ";
            for (const auto& frame : (*frames)) {
                int ft = static_cast<int>(frame->frameType());
                std::cout << "0x" << std::hex << ft << std::dec << " ";
            }
            std::cout << std::endl;
        }

        for (const auto& frame : (*frames)) {
            int width = frame->frameWidth();
            int height = frame->frameHeight();
            int center_x = width / 2;
            int center_y = height / 2;
            int center_idx = center_y * width + center_x;

            // Check for Raw frame type
            if (frame->frameType() == ms::FrameType::Raw) {
                std::cout << "[Raw Frame " << frame_num << "] Width=" << width << ", Height=" << height
                          << ", DataType=" << static_cast<int>(frame->frameDataType())
                          << ", Format=" << frame->frameFormat() << std::endl;
            }
            else if (frame->frameType() == ms::FrameType::Depth) {
                depth_width = width;
                depth_height = height;

                if (frame->frameDataType() == ms::DataType::U16) {
                    auto basic_frame = ms::frame_cast_basic16u(frame);
                    auto data = basic_frame->frameData();
                    depth_center = static_cast<float>((*data)[center_idx]);
                } else if (frame->frameDataType() == ms::DataType::F32) {
                    auto basic_frame = ms::frame_cast_basic32f(frame);
                    auto data = basic_frame->frameData();
                    depth_center = (*data)[center_idx] * 1000.0f; // Convert m to mm
                }

                // Save frames for analysis (paired with Raw in same callback)
                if (frame_num < 300 && frame->frameDataType() == ms::DataType::U16) {
                    auto basic_frame = ms::frame_cast_basic16u(frame);
                    auto data = basic_frame->frameData();
                    std::string filename = g_data_dir + "/sync_depth_" + std::to_string(frame_num) + ".raw";
                    std::ofstream file(filename, std::ios::binary);
                    if (file) {
                        file.write(reinterpret_cast<const char*>(data->data()),
                                   data->size() * sizeof(uint16_t));
                        std::cout << "[Saved] Depth frame " << frame_num << " to " << filename
                                  << " (" << data->size() << " pixels, " << width << "x" << height << ")" << std::endl;
                    }

                    // Also save Raw frame from same callback if available
                    for (const auto& raw_frame : (*frames)) {
                        if (raw_frame->frameType() == ms::FrameType::Raw) {
                            auto raw_basic = ms::frame_cast_basic8u(raw_frame);
                            if (raw_basic) {
                                auto raw_data = raw_basic->frameData();
                                std::string raw_filename = g_data_dir + "/sync_raw_" + std::to_string(frame_num) + ".raw";
                                std::ofstream raw_file(raw_filename, std::ios::binary);
                                if (raw_file) {
                                    raw_file.write(reinterpret_cast<const char*>(raw_data->data()),
                                                   raw_data->size());
                                    std::cout << "[Saved] Raw frame " << frame_num << " to " << raw_filename
                                              << " (" << raw_data->size() << " bytes)" << std::endl;
                                }
                            }
                            break;
                        }
                    }
                }
            }
            else if (frame->frameType() == ms::FrameType::Amplitude) {
                if (frame->frameDataType() == ms::DataType::U16) {
                    auto basic_frame = ms::frame_cast_basic16u(frame);
                    auto data = basic_frame->frameData();
                    amplitude_center = static_cast<float>((*data)[center_idx]);
                } else if (frame->frameDataType() == ms::DataType::F32) {
                    auto basic_frame = ms::frame_cast_basic32f(frame);
                    auto data = basic_frame->frameData();
                    amplitude_center = (*data)[center_idx];
                }
            }
        }

        // Store samples for statistics
        {
            std::lock_guard<std::mutex> lock(g_stats_mutex);
            if (depth_center > 0) g_depth_samples.push_back(depth_center);
            if (amplitude_center > 0) g_amplitude_samples.push_back(amplitude_center);
        }

        // Print progress every 10 frames
        if (frame_num % 10 == 0 || frame_num < 5) {
            std::cout << "[Frame " << std::setw(4) << frame_num << "] "
                      << "Depth: " << std::fixed << std::setprecision(1) << depth_center << " mm, "
                      << "Amplitude: " << std::setprecision(0) << amplitude_center
                      << " (Size: " << depth_width << "x" << depth_height << ")"
                      << std::endl;
        }

        if (frame_num >= g_target_frames - 1) {
            g_running = false;
        }
    }

    void onCubeEyeCameraPrepared(const ms::ptr_camera camera) override {
        std::cout << "[Prepared] Camera ready: " << camera->source()->uri() << std::endl;
    }
};

static FrameCaptureSink g_sink;

void print_statistics() {
    std::lock_guard<std::mutex> lock(g_stats_mutex);

    std::cout << "\n=== Capture Statistics ===" << std::endl;
    std::cout << "Total frames captured: " << g_depth_samples.size() << std::endl;

    if (!g_depth_samples.empty()) {
        // Depth statistics
        float depth_sum = std::accumulate(g_depth_samples.begin(), g_depth_samples.end(), 0.0f);
        float depth_mean = depth_sum / g_depth_samples.size();

        std::vector<float> sorted_depth = g_depth_samples;
        std::sort(sorted_depth.begin(), sorted_depth.end());
        float depth_median = sorted_depth[sorted_depth.size() / 2];

        float depth_var = 0;
        for (float d : g_depth_samples) {
            depth_var += (d - depth_mean) * (d - depth_mean);
        }
        depth_var /= g_depth_samples.size();
        float depth_std = std::sqrt(depth_var);

        std::cout << "\nDepth (center pixel):" << std::endl;
        std::cout << "  Mean:   " << std::fixed << std::setprecision(2) << depth_mean << " mm" << std::endl;
        std::cout << "  Median: " << depth_median << " mm" << std::endl;
        std::cout << "  Std:    " << depth_std << " mm" << std::endl;
        std::cout << "  Min:    " << sorted_depth.front() << " mm" << std::endl;
        std::cout << "  Max:    " << sorted_depth.back() << " mm" << std::endl;
    }

    if (!g_amplitude_samples.empty()) {
        float amp_sum = std::accumulate(g_amplitude_samples.begin(), g_amplitude_samples.end(), 0.0f);
        float amp_mean = amp_sum / g_amplitude_samples.size();

        std::cout << "\nAmplitude (center pixel):" << std::endl;
        std::cout << "  Mean:   " << std::fixed << std::setprecision(1) << amp_mean << std::endl;
    }

    std::cout << "\n==========================" << std::endl;
}

int main(int argc, char* argv[]) {
    // Parse arguments
    if (argc > 1) {
        g_target_frames = std::atoi(argv[1]);
        if (g_target_frames <= 0) g_target_frames = 100;
    }

    std::cout << "CubeEye SDK Capture Tool" << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "Target frames: " << g_target_frames << std::endl;
    std::cout << "Press Ctrl+C to stop early\n" << std::endl;

    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Search for camera
    std::cout << "[Search] Looking for CubeEye cameras..." << std::endl;
    auto source_list = ms::search_camera_source();

    if (!source_list || source_list->size() == 0) {
        std::cerr << "[Error] No CubeEye camera found!" << std::endl;
        return 1;
    }

    // List all found cameras
    std::cout << "[Found] " << source_list->size() << " camera(s):" << std::endl;
    int idx = 0;
    ms::sptr_source selected_source;
    for (const auto& src : (*source_list)) {
        std::cout << "  [" << idx << "] " << src->name()
                  << " (SN: " << src->serialNumber() << ")"
                  << " URI: " << src->uri() << std::endl;
        if (!selected_source) {
            selected_source = src;
        }
        idx++;
    }

    if (!selected_source) {
        std::cerr << "[Error] No camera source available!" << std::endl;
        return 1;
    }

    // Create camera
    std::cout << "\n[Create] Creating camera for: " << selected_source->name() << std::endl;
    auto camera = ms::create_camera(selected_source);

    if (!camera) {
        std::cerr << "[Error] Failed to create camera!" << std::endl;
        return 1;
    }

    // Add sink and prepare
    ms::add_prepared_listener(&g_sink);
    camera->addSink(&g_sink);

    std::cout << "[Prepare] Preparing camera..." << std::endl;
    auto result = camera->prepare();
    if (result != ms::result::success) {
        std::cerr << "[Error] Failed to prepare camera: " << static_cast<int>(result) << std::endl;
        ms::destroy_camera(camera);
        return 1;
    }

    // Print camera info
    std::cout << "\n[Info] Camera parameters:" << std::endl;
    std::cout << "  Lenses: " << camera->lenses() << std::endl;

    auto fov = camera->fov(0);
    std::cout << "  FoV: " << std::get<0>(fov) << "° (H) x " << std::get<1>(fov) << "° (V)" << std::endl;

    ms::IntrinsicParameters intrinsic;
    if (camera->intrinsicParameters(intrinsic, 0) == ms::result::success) {
        std::cout << "  Focal Length: fx=" << intrinsic.focal.fx
                  << ", fy=" << intrinsic.focal.fy << std::endl;
        std::cout << "  Principal Point: cx=" << intrinsic.principal.cx
                  << ", cy=" << intrinsic.principal.cy << std::endl;
    }

    // Start capture
    std::cout << "\n[Run] Starting capture (Raw + Depth + Amplitude)..." << std::endl;
    int wanted_frames = ms::FrameType::Raw | ms::FrameType::Depth | ms::FrameType::Amplitude;
    result = camera->run(wanted_frames);

    if (result != ms::result::success) {
        std::cerr << "[Error] Failed to start camera: " << static_cast<int>(result) << std::endl;
        ms::destroy_camera(camera);
        return 1;
    }

    // Wait for capture to complete
    auto start_time = std::chrono::steady_clock::now();
    while (g_running && g_frame_count < g_target_frames) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Timeout after 30 seconds
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 30) {
            std::cout << "[Timeout] No frames received in 30 seconds" << std::endl;
            break;
        }
    }

    // Stop and cleanup
    std::cout << "\n[Stop] Stopping camera..." << std::endl;
    camera->stop();

    print_statistics();

    std::cout << "\n[Cleanup] Releasing camera..." << std::endl;
    ms::destroy_camera(camera);

    std::cout << "[Done]" << std::endl;
    return 0;
}

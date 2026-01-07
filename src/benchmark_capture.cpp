/**
 * benchmark_capture.cpp - Synchronized SDK + Raw Frame Benchmark Tool
 *
 * Captures depth frames via SDK while simultaneously capturing raw V4L2 frames
 * via LD_PRELOAD hook. Both frame types are saved with matching timestamps
 * for direct comparison.
 *
 * Usage: LD_PRELOAD=./build/libv4l2_hook.so ./build/benchmark_capture [num_frames] [output_dir]
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <csignal>
#include <sys/stat.h>

#include "CubeEye/CubeEyeSink.h"
#include "CubeEye/CubeEyeCamera.h"
#include "CubeEye/CubeEyeBasicFrame.h"

namespace ms = meere::sensor;

// Configuration
static std::atomic<bool> g_running{true};
static std::atomic<int> g_frame_count{0};
static int g_target_frames = 50;
static std::string g_output_dir = "benchmark";

// Statistics storage
struct FrameStats {
    int frame_num;
    float depth_center;
    float amplitude_center;
    uint64_t timestamp_us;
};

static std::vector<FrameStats> g_stats;
static std::mutex g_stats_mutex;

void signal_handler(int sig) {
    std::cout << "\nReceived signal " << sig << ", stopping..." << std::endl;
    g_running = false;
}

// Create output directory
void ensure_dir(const std::string& path) {
    mkdir(path.c_str(), 0755);
}

// Frame capture sink
class BenchmarkSink : public ms::sink, public ms::prepared_listener {
public:
    std::string name() const override {
        return "BenchmarkSink";
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
        auto now = std::chrono::high_resolution_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();

        FrameStats stats;
        stats.frame_num = frame_num;
        stats.timestamp_us = timestamp;
        stats.depth_center = 0;
        stats.amplitude_center = 0;

        for (const auto& frame : (*frames)) {
            int width = frame->frameWidth();
            int height = frame->frameHeight();
            int center_x = width / 2;
            int center_y = height / 2;
            int center_idx = center_y * width + center_x;

            if (frame->frameType() == ms::FrameType::Depth) {
                if (frame->frameDataType() == ms::DataType::U16) {
                    auto basic_frame = ms::frame_cast_basic16u(frame);
                    auto data = basic_frame->frameData();
                    stats.depth_center = static_cast<float>((*data)[center_idx]);

                    // Save every depth frame
                    std::ostringstream filename;
                    filename << g_output_dir << "/sdk_depth_"
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
                    stats.amplitude_center = static_cast<float>((*data)[center_idx]);

                    // Save every amplitude frame
                    std::ostringstream filename;
                    filename << g_output_dir << "/sdk_amp_"
                             << std::setfill('0') << std::setw(4) << frame_num << ".raw";
                    std::ofstream file(filename.str(), std::ios::binary);
                    if (file) {
                        file.write(reinterpret_cast<const char*>(data->data()),
                                   data->size() * sizeof(uint16_t));
                    }
                }
            }
        }

        // Store stats
        {
            std::lock_guard<std::mutex> lock(g_stats_mutex);
            g_stats.push_back(stats);
        }

        // Progress output
        if (frame_num % 10 == 0 || frame_num < 5) {
            std::cout << "[Frame " << std::setw(4) << frame_num << "] "
                      << "Depth: " << std::fixed << std::setprecision(1)
                      << stats.depth_center << " mm" << std::endl;
        }

        if (frame_num >= g_target_frames - 1) {
            g_running = false;
        }
    }

    void onCubeEyeCameraPrepared(const ms::ptr_camera camera) override {
        std::cout << "[Prepared] Camera ready" << std::endl;
    }
};

static BenchmarkSink g_sink;

void print_statistics() {
    std::lock_guard<std::mutex> lock(g_stats_mutex);

    std::cout << "\n===========================================" << std::endl;
    std::cout << "         BENCHMARK STATISTICS" << std::endl;
    std::cout << "===========================================" << std::endl;
    std::cout << "Total SDK frames: " << g_stats.size() << std::endl;

    if (g_stats.empty()) return;

    // Calculate depth stats
    std::vector<float> depths;
    for (const auto& s : g_stats) {
        if (s.depth_center > 0) depths.push_back(s.depth_center);
    }

    if (!depths.empty()) {
        float sum = std::accumulate(depths.begin(), depths.end(), 0.0f);
        float mean = sum / depths.size();

        std::vector<float> sorted = depths;
        std::sort(sorted.begin(), sorted.end());
        float median = sorted[sorted.size() / 2];

        float var = 0;
        for (float d : depths) {
            var += (d - mean) * (d - mean);
        }
        var /= depths.size();
        float std_dev = std::sqrt(var);

        std::cout << "\nSDK Depth (center pixel):" << std::endl;
        std::cout << "  Mean:   " << std::fixed << std::setprecision(2) << mean << " mm" << std::endl;
        std::cout << "  Median: " << median << " mm" << std::endl;
        std::cout << "  Std:    " << std_dev << " mm" << std::endl;
        std::cout << "  Min:    " << sorted.front() << " mm" << std::endl;
        std::cout << "  Max:    " << sorted.back() << " mm" << std::endl;
    }

    // Save stats to CSV
    std::string csv_file = g_output_dir + "/benchmark_stats.csv";
    std::ofstream csv(csv_file);
    if (csv) {
        csv << "frame,depth_mm,amplitude,timestamp_us\n";
        for (const auto& s : g_stats) {
            csv << s.frame_num << "," << s.depth_center << ","
                << s.amplitude_center << "," << s.timestamp_us << "\n";
        }
        std::cout << "\nStats saved to: " << csv_file << std::endl;
    }

    std::cout << "===========================================" << std::endl;
}

int main(int argc, char* argv[]) {
    // Parse arguments
    if (argc > 1) {
        g_target_frames = std::atoi(argv[1]);
        if (g_target_frames <= 0) g_target_frames = 50;
    }
    if (argc > 2) {
        g_output_dir = argv[2];
    }

    std::cout << "CubeEye Benchmark Capture Tool" << std::endl;
    std::cout << "==============================" << std::endl;
    std::cout << "Target frames: " << g_target_frames << std::endl;
    std::cout << "Output dir:    " << g_output_dir << std::endl;
    std::cout << "\nIMPORTANT: Run with LD_PRELOAD=./build/libv4l2_hook.so" << std::endl;
    std::cout << "           to capture raw frames simultaneously." << std::endl;
    std::cout << "\nPress Ctrl+C to stop early\n" << std::endl;

    // Create output directory
    ensure_dir(g_output_dir);

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

    ms::sptr_source selected_source = (*source_list)[0];
    std::cout << "[Found] " << selected_source->name()
              << " (SN: " << selected_source->serialNumber() << ")" << std::endl;

    // Create and prepare camera
    auto camera = ms::create_camera(selected_source);
    if (!camera) {
        std::cerr << "[Error] Failed to create camera!" << std::endl;
        return 1;
    }

    ms::add_prepared_listener(&g_sink);
    camera->addSink(&g_sink);

    auto result = camera->prepare();
    if (result != ms::result::success) {
        std::cerr << "[Error] Failed to prepare camera" << std::endl;
        ms::destroy_camera(camera);
        return 1;
    }

    // Start capture (Depth + Amplitude only, SDK doesn't give Raw)
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
    print_statistics();
    ms::destroy_camera(camera);

    std::cout << "\n[Done] Output saved to: " << g_output_dir << "/" << std::endl;
    return 0;
}

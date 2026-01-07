/**
 * test_depth_processor.cpp - Test the depth processor against SDK ground truth
 */

#include "depth_processor.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

bool load_raw_frame(const std::string& path, std::vector<uint16_t>& data) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    data.resize(size / sizeof(uint16_t));
    file.read(reinterpret_cast<char*>(data.data()), size);
    return true;
}

bool load_sdk_depth(const std::string& path, std::vector<uint16_t>& data) {
    return load_raw_frame(path, data);  // Same format
}

int main(int argc, char* argv[]) {
    std::string data_dir = "variable_dist";
    if (argc > 1) {
        data_dir = argv[1];
    }

    std::cout << "=== Depth Processor Test ===" << std::endl;
    std::cout << "Data directory: " << data_dir << std::endl;

    // Find raw and SDK files
    std::vector<std::string> raw_files, sdk_files;
    for (const auto& entry : fs::directory_iterator(data_dir)) {
        std::string name = entry.path().filename().string();
        if (name.find("raw_") == 0 && name.find(".raw") != std::string::npos) {
            // Skip warmup frames (0-3)
            int num = std::stoi(name.substr(4, 4));
            if (num >= 4) {
                raw_files.push_back(entry.path().string());
            }
        } else if (name.find("sdk_depth_") == 0) {
            sdk_files.push_back(entry.path().string());
        }
    }

    std::sort(raw_files.begin(), raw_files.end());
    std::sort(sdk_files.begin(), sdk_files.end());

    std::cout << "Found " << raw_files.size() << " raw frames, "
              << sdk_files.size() << " SDK frames" << std::endl;

    if (raw_files.empty() || sdk_files.empty()) {
        std::cerr << "No files found!" << std::endl;
        return 1;
    }

    cubeeye::DepthProcessor processor;

    // Process and compare
    double total_rmse = 0.0;
    double total_mae = 0.0;
    int total_compared = 0;
    float min_time = 1000.0f, max_time = 0.0f, total_time = 0.0f;

    std::vector<uint16_t> raw_data;
    std::vector<uint16_t> sdk_data;
    std::vector<float> depth_out(cubeeye::DEPTH_WIDTH * cubeeye::DEPTH_HEIGHT);

    int num_frames = std::min(raw_files.size(), sdk_files.size());

    for (int i = 0; i < num_frames; ++i) {
        if (!load_raw_frame(raw_files[i], raw_data)) {
            std::cerr << "Failed to load: " << raw_files[i] << std::endl;
            continue;
        }
        if (!load_sdk_depth(sdk_files[i], sdk_data)) {
            std::cerr << "Failed to load: " << sdk_files[i] << std::endl;
            continue;
        }

        // Process
        processor.Process(raw_data.data(), depth_out.data());
        const auto& stats = processor.GetLastStats();

        // Compare with SDK (SDK is 640x480, our output is 320x240)
        // Compare center region
        double se_sum = 0.0;
        double ae_sum = 0.0;
        int compare_count = 0;

        for (int y = 108; y < 132; ++y) {  // Center 24 rows
            for (int x = 148; x < 172; ++x) {  // Center 24 cols
                float our_depth = depth_out[y * cubeeye::DEPTH_WIDTH + x];
                if (our_depth <= 0) continue;

                // SDK uses 2x coordinates
                float sdk_depth = static_cast<float>(
                    sdk_data[y * 2 * 640 + x * 2]);

                float error = our_depth - sdk_depth;
                se_sum += error * error;
                ae_sum += std::abs(error);
                compare_count++;
            }
        }

        if (compare_count > 0) {
            double rmse = std::sqrt(se_sum / compare_count);
            double mae = ae_sum / compare_count;
            total_rmse += rmse;
            total_mae += mae;
            total_compared++;

            min_time = std::min(min_time, stats.process_time_ms);
            max_time = std::max(max_time, stats.process_time_ms);
            total_time += stats.process_time_ms;
        }

        if (i % 25 == 0 || i == num_frames - 1) {
            std::cout << "Frame " << i << "/" << num_frames
                      << ": depth=" << stats.mean_depth_mm << "mm"
                      << ", time=" << stats.process_time_ms << "ms"
                      << ", valid=" << stats.valid_pixels << std::endl;
        }
    }

    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "Frames processed: " << total_compared << std::endl;
    std::cout << "Average RMSE: " << (total_rmse / total_compared) << " mm" << std::endl;
    std::cout << "Average MAE:  " << (total_mae / total_compared) << " mm" << std::endl;
    std::cout << "Processing time: min=" << min_time << "ms, max=" << max_time
              << "ms, avg=" << (total_time / total_compared) << "ms" << std::endl;

    return 0;
}

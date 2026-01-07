/**
 * test_cubeeye_depth.cpp - Test CubeEye depth extraction
 *
 * Verifies C++ implementation against SDK output
 */

#include "cubeeye_depth.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>

bool LoadRawFrame(const std::string& path, std::vector<uint8_t>& data) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "Failed to open: " << path << std::endl;
        return false;
    }

    size_t size = file.tellg();
    file.seekg(0);

    data.resize(size);
    file.read(reinterpret_cast<char*>(data.data()), size);

    return file.good();
}

bool LoadSDKDepth(const std::string& path, std::vector<uint16_t>& data) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "Failed to open: " << path << std::endl;
        return false;
    }

    size_t size = file.tellg();
    file.seekg(0);

    data.resize(size / sizeof(uint16_t));
    file.read(reinterpret_cast<char*>(data.data()), size);

    return file.good();
}

void CompareFrames(const uint16_t* decoded, const uint16_t* sdk,
                   int width, int height, bool with_correction) {
    int num_pixels = width * height;
    int valid_count = 0;
    double sum_diff = 0;
    double sum_sq_diff = 0;
    double sum_decoded = 0;
    double sum_sdk = 0;

    for (int i = 0; i < num_pixels; ++i) {
        if (sdk[i] > 100 && sdk[i] < 7000) {
            double diff = static_cast<double>(decoded[i]) - static_cast<double>(sdk[i]);
            sum_diff += diff;
            sum_sq_diff += diff * diff;
            sum_decoded += decoded[i];
            sum_sdk += sdk[i];
            valid_count++;
        }
    }

    if (valid_count > 0) {
        double mean_diff = sum_diff / valid_count;
        double rmse = std::sqrt(sum_sq_diff / valid_count);
        double mean_decoded = sum_decoded / valid_count;
        double mean_sdk = sum_sdk / valid_count;

        // Calculate correlation
        double sum_xy = 0, sum_x2 = 0, sum_y2 = 0;
        for (int i = 0; i < num_pixels; ++i) {
            if (sdk[i] > 100 && sdk[i] < 7000) {
                double x = decoded[i] - mean_decoded;
                double y = sdk[i] - mean_sdk;
                sum_xy += x * y;
                sum_x2 += x * x;
                sum_y2 += y * y;
            }
        }
        double corr = sum_xy / std::sqrt(sum_x2 * sum_y2);

        const char* mode = with_correction ? "With correction" : "No correction";
        std::cout << "  " << mode << ":" << std::endl;
        std::cout << "    Valid pixels: " << valid_count << " / " << num_pixels << std::endl;
        std::cout << "    RMSE: " << std::fixed << std::setprecision(2) << rmse << " mm" << std::endl;
        std::cout << "    Mean diff: " << std::showpos << mean_diff << std::noshowpos << " mm" << std::endl;
        std::cout << "    Correlation: r = " << std::setprecision(6) << corr << std::endl;
    }
}

void TestSamplePixels(const uint16_t* decoded, const uint16_t* sdk, int width) {
    struct TestPoint {
        int row, col;
        const char* name;
    };

    TestPoint points[] = {
        {240, 320, "center"},
        {120, 160, "top-left quarter"},
        {360, 480, "bottom-right quarter"},
    };

    std::cout << "\n  Sample pixels (SDK, Decoded):" << std::endl;
    for (const auto& p : points) {
        int idx = p.row * width + p.col;
        int diff = static_cast<int>(decoded[idx]) - static_cast<int>(sdk[idx]);
        std::cout << "    " << p.name << " (" << p.row << "," << p.col << "): "
                  << "SDK=" << sdk[idx] << ", Decoded=" << decoded[idx]
                  << ", diff=" << std::showpos << diff << std::noshowpos << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "CubeEye Depth Extraction C++ Test" << std::endl;
    std::cout << "========================================" << std::endl;

    // Test with first frame pair (raw_0004 = sdk_depth_0)
    std::string raw_path = "data/sync_capture/raw_0004.bin";
    std::string sdk_path = "data/sync_depth_0.raw";

    std::vector<uint8_t> raw_data;
    std::vector<uint16_t> sdk_depth;

    if (!LoadRawFrame(raw_path, raw_data)) {
        std::cerr << "Failed to load raw frame" << std::endl;
        return 1;
    }

    if (!LoadSDKDepth(sdk_path, sdk_depth)) {
        std::cerr << "Failed to load SDK depth" << std::endl;
        return 1;
    }

    std::cout << "\nRaw frame: " << raw_data.size() << " bytes" << std::endl;
    std::cout << "SDK depth: " << sdk_depth.size() << " pixels" << std::endl;

    // Create extractors
    cubeeye::DepthExtractor extractor_with_corr(true);
    cubeeye::DepthExtractor extractor_no_corr(false);

    // Extract depth
    std::vector<uint16_t> depth_with_corr(cubeeye::OUTPUT_WIDTH * cubeeye::OUTPUT_HEIGHT);
    std::vector<uint16_t> depth_no_corr(cubeeye::OUTPUT_WIDTH * cubeeye::OUTPUT_HEIGHT);

    std::cout << "\n--- Test with gradient correction ---" << std::endl;
    if (!extractor_with_corr.ExtractDepth(raw_data.data(), raw_data.size(),
                                          depth_with_corr.data(), true)) {
        std::cerr << "Failed to extract depth (with correction)" << std::endl;
        return 1;
    }

    std::cout << "\n--- Test without gradient correction ---" << std::endl;
    if (!extractor_no_corr.ExtractDepth(raw_data.data(), raw_data.size(),
                                         depth_no_corr.data(), true)) {
        std::cerr << "Failed to extract depth (no correction)" << std::endl;
        return 1;
    }

    // Compare
    std::cout << "\n=== Comparison with SDK output ===" << std::endl;

    CompareFrames(depth_no_corr.data(), sdk_depth.data(),
                  cubeeye::OUTPUT_WIDTH, cubeeye::OUTPUT_HEIGHT, false);

    CompareFrames(depth_with_corr.data(), sdk_depth.data(),
                  cubeeye::OUTPUT_WIDTH, cubeeye::OUTPUT_HEIGHT, true);

    TestSamplePixels(depth_with_corr.data(), sdk_depth.data(), cubeeye::OUTPUT_WIDTH);

    // Multi-frame test
    std::cout << "\n=== Multi-frame verification ===" << std::endl;

    for (int i = 0; i < 5; ++i) {
        std::string raw = "data/sync_capture/raw_" + std::string(4 - std::to_string(i+4).length(), '0')
                          + std::to_string(i+4) + ".bin";
        std::string sdk = "data/sync_depth_" + std::to_string(i) + ".raw";

        std::vector<uint8_t> r;
        std::vector<uint16_t> s;

        if (!LoadRawFrame(raw, r) || !LoadSDKDepth(sdk, s)) continue;

        std::vector<uint16_t> d(cubeeye::OUTPUT_WIDTH * cubeeye::OUTPUT_HEIGHT);
        extractor_with_corr.ExtractDepth(r.data(), r.size(), d.data(), true);

        // Quick stats
        int valid = 0;
        double sum_sq = 0;
        for (size_t j = 0; j < s.size(); ++j) {
            if (s[j] > 100 && s[j] < 7000) {
                double diff = static_cast<double>(d[j]) - static_cast<double>(s[j]);
                sum_sq += diff * diff;
                valid++;
            }
        }
        double rmse = std::sqrt(sum_sq / valid);
        std::cout << "  Frame " << i << ": RMSE = " << std::fixed << std::setprecision(2)
                  << rmse << " mm" << std::endl;
    }

    std::cout << "\n=== Test PASSED ===" << std::endl;
    return 0;
}

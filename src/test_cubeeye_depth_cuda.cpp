/**
 * test_cubeeye_depth_cuda.cpp - CUDA Depth Extraction Test
 *
 * Tests CUDA implementation against CPU reference and SDK ground truth.
 * Benchmarks performance to verify GPU acceleration.
 *
 * Usage: ./test_cubeeye_depth_cuda [raw_file] [sdk_depth_file]
 *        ./test_cubeeye_depth_cuda --benchmark [raw_file] [iterations]
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstring>

#include "cubeeye_depth_cuda.h"
#include "cubeeye_depth.h"  // CPU reference

using namespace cubeeye;

// Test parameters
constexpr int RAW_FRAME_SIZE = 771200;
constexpr int OUTPUT_WIDTH = 640;
constexpr int OUTPUT_HEIGHT = 480;
constexpr int OUTPUT_SIZE = OUTPUT_WIDTH * OUTPUT_HEIGHT;

// Load raw frame from file
bool LoadRawFrame(const char* path, std::vector<uint8_t>& data) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot open " << path << std::endl;
        return false;
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (size != RAW_FRAME_SIZE) {
        std::cerr << "Error: Expected " << RAW_FRAME_SIZE << " bytes, got "
                  << size << std::endl;
        return false;
    }

    data.resize(size);
    file.read(reinterpret_cast<char*>(data.data()), size);
    return true;
}

// Load SDK depth reference
bool LoadSDKDepth(const char* path, std::vector<uint16_t>& data) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot open " << path << std::endl;
        return false;
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    size_t expected = OUTPUT_SIZE * sizeof(uint16_t);
    if (size != expected) {
        std::cerr << "Error: Expected " << expected << " bytes, got "
                  << size << std::endl;
        return false;
    }

    data.resize(OUTPUT_SIZE);
    file.read(reinterpret_cast<char*>(data.data()), size);
    return true;
}

// Calculate statistics
struct Stats {
    double rmse;
    double mean_error;
    double max_error;
    double correlation;
    int matching_pixels;
};

Stats CalculateStats(const uint16_t* decoded, const uint16_t* reference,
                     int width, int height, int min_depth = 50, int max_depth = 7000) {
    Stats stats = {0, 0, 0, 0, 0};

    double sum_diff = 0;
    double sum_sq_diff = 0;
    double max_diff = 0;
    int count = 0;

    // For correlation
    double sum_x = 0, sum_y = 0, sum_xy = 0;
    double sum_x2 = 0, sum_y2 = 0;

    for (int i = 0; i < width * height; i++) {
        int ref = reference[i];
        int dec = decoded[i];

        // Only count valid pixels
        if (ref >= min_depth && ref <= max_depth) {
            double diff = static_cast<double>(dec) - static_cast<double>(ref);
            sum_diff += diff;
            sum_sq_diff += diff * diff;
            max_diff = std::max(max_diff, std::abs(diff));
            count++;

            sum_x += dec;
            sum_y += ref;
            sum_xy += static_cast<double>(dec) * static_cast<double>(ref);
            sum_x2 += static_cast<double>(dec) * static_cast<double>(dec);
            sum_y2 += static_cast<double>(ref) * static_cast<double>(ref);

            // Count exact matches (within 1mm)
            if (std::abs(diff) <= 1) {
                stats.matching_pixels++;
            }
        }
    }

    if (count > 0) {
        stats.rmse = std::sqrt(sum_sq_diff / count);
        stats.mean_error = sum_diff / count;
        stats.max_error = max_diff;

        // Pearson correlation
        double n = count;
        double denom = std::sqrt((n * sum_x2 - sum_x * sum_x) *
                                  (n * sum_y2 - sum_y * sum_y));
        if (denom > 0) {
            stats.correlation = (n * sum_xy - sum_x * sum_y) / denom;
        }
    }

    return stats;
}

// Benchmark mode
void RunBenchmark(const std::vector<uint8_t>& raw_frame, int iterations) {
    std::cout << "\n========== CUDA Performance Benchmark ==========\n" << std::endl;

    // Check CUDA availability
    if (!cuda::CudaDepthExtractor::IsCudaAvailable()) {
        std::cerr << "CUDA not available on this system" << std::endl;
        return;
    }

    std::cout << "Device: " << cuda::CudaDepthExtractor::GetDeviceInfo() << std::endl;
    std::cout << "Iterations: " << iterations << std::endl;
    std::cout << std::endl;

    // Allocate output buffers
    std::vector<uint16_t> depth_cuda(OUTPUT_SIZE);
    std::vector<uint16_t> depth_cpu(OUTPUT_SIZE);

    // Initialize extractors
    cuda::CudaDepthExtractor cuda_extractor(true, true);
    DepthExtractor cpu_extractor(true);

    // Warmup
    std::cout << "Warming up..." << std::endl;
    for (int i = 0; i < 10; i++) {
        cuda_extractor.ExtractDepth(raw_frame.data(), raw_frame.size(),
                                     depth_cuda.data(), true);
    }

    // Benchmark CUDA
    std::cout << "Benchmarking CUDA..." << std::endl;
    auto cuda_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; i++) {
        cuda_extractor.ExtractDepth(raw_frame.data(), raw_frame.size(),
                                     depth_cuda.data(), true);
    }
    auto cuda_end = std::chrono::high_resolution_clock::now();
    double cuda_ms = std::chrono::duration<double, std::milli>(cuda_end - cuda_start).count();

    // Benchmark CPU
    std::cout << "Benchmarking CPU..." << std::endl;
    auto cpu_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; i++) {
        cpu_extractor.ExtractDepth(raw_frame.data(), raw_frame.size(),
                                    depth_cpu.data(), true);
    }
    auto cpu_end = std::chrono::high_resolution_clock::now();
    double cpu_ms = std::chrono::duration<double, std::milli>(cpu_end - cpu_start).count();

    // Results
    std::cout << "\n---------- Results ----------" << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    double cuda_per_frame = cuda_ms / iterations;
    double cpu_per_frame = cpu_ms / iterations;
    double speedup = cpu_per_frame / cuda_per_frame;

    std::cout << "CUDA: " << cuda_per_frame << " ms/frame ("
              << (1000.0 / cuda_per_frame) << " fps)" << std::endl;
    std::cout << "CPU:  " << cpu_per_frame << " ms/frame ("
              << (1000.0 / cpu_per_frame) << " fps)" << std::endl;
    std::cout << "Speedup: " << speedup << "x" << std::endl;

    // Verify CUDA vs CPU results match
    std::cout << "\n---------- Correctness Check ----------" << std::endl;
    Stats stats = CalculateStats(depth_cuda.data(), depth_cpu.data(),
                                  OUTPUT_WIDTH, OUTPUT_HEIGHT);
    std::cout << "CUDA vs CPU RMSE: " << stats.rmse << " mm" << std::endl;
    std::cout << "CUDA vs CPU correlation: " << std::setprecision(6)
              << stats.correlation << std::endl;

    if (stats.rmse < 1.0 && stats.correlation > 0.9999) {
        std::cout << "\n[PASS] CUDA and CPU implementations match" << std::endl;
    } else {
        std::cout << "\n[WARN] CUDA and CPU results differ" << std::endl;
    }

    std::cout << "\n============================================\n" << std::endl;
}

// Validation mode
void RunValidation(const std::vector<uint8_t>& raw_frame,
                   const std::vector<uint16_t>& sdk_depth) {
    std::cout << "\n========== CUDA Validation Test ==========\n" << std::endl;

    if (!cuda::CudaDepthExtractor::IsCudaAvailable()) {
        std::cerr << "CUDA not available on this system" << std::endl;
        return;
    }

    std::cout << "Device: " << cuda::CudaDepthExtractor::GetDeviceInfo() << std::endl;
    std::cout << std::endl;

    // Allocate output buffers
    std::vector<uint16_t> depth_cuda(OUTPUT_SIZE);
    std::vector<uint16_t> depth_cpu(OUTPUT_SIZE);
    std::vector<uint16_t> amplitude_cuda(OUTPUT_SIZE);

    // Initialize extractors
    cuda::CudaDepthExtractor cuda_extractor(true, true);
    DepthExtractor cpu_extractor(true);

    // Extract depth
    cuda_extractor.ExtractDepth(raw_frame.data(), raw_frame.size(),
                                 depth_cuda.data(), true);
    cpu_extractor.ExtractDepth(raw_frame.data(), raw_frame.size(),
                                depth_cpu.data(), true);

    std::cout << "Processing time: " << std::fixed << std::setprecision(3)
              << cuda_extractor.GetLastTotalTimeMs() << " ms" << std::endl;
    std::cout << std::endl;

    // Compare CUDA vs SDK
    std::cout << "---------- CUDA vs SDK ----------" << std::endl;
    Stats cuda_vs_sdk = CalculateStats(depth_cuda.data(), sdk_depth.data(),
                                        OUTPUT_WIDTH, OUTPUT_HEIGHT);
    std::cout << "RMSE:        " << std::fixed << std::setprecision(2)
              << cuda_vs_sdk.rmse << " mm" << std::endl;
    std::cout << "Mean error:  " << std::showpos << cuda_vs_sdk.mean_error
              << std::noshowpos << " mm" << std::endl;
    std::cout << "Max error:   " << cuda_vs_sdk.max_error << " mm" << std::endl;
    std::cout << "Correlation: " << std::setprecision(6)
              << cuda_vs_sdk.correlation << std::endl;

    // Compare CUDA vs CPU
    std::cout << "\n---------- CUDA vs CPU ----------" << std::endl;
    Stats cuda_vs_cpu = CalculateStats(depth_cuda.data(), depth_cpu.data(),
                                        OUTPUT_WIDTH, OUTPUT_HEIGHT);
    std::cout << "RMSE:        " << std::setprecision(2)
              << cuda_vs_cpu.rmse << " mm" << std::endl;
    std::cout << "Correlation: " << std::setprecision(6)
              << cuda_vs_cpu.correlation << std::endl;

    // Center pixel comparison
    int center_idx = 240 * OUTPUT_WIDTH + 320;
    std::cout << "\n---------- Center Pixel ----------" << std::endl;
    std::cout << "SDK:  " << sdk_depth[center_idx] << " mm" << std::endl;
    std::cout << "CUDA: " << depth_cuda[center_idx] << " mm" << std::endl;
    std::cout << "CPU:  " << depth_cpu[center_idx] << " mm" << std::endl;
    std::cout << "Error (CUDA vs SDK): "
              << (static_cast<int>(depth_cuda[center_idx]) -
                  static_cast<int>(sdk_depth[center_idx])) << " mm" << std::endl;

    // Test amplitude extraction
    std::cout << "\n---------- Amplitude Test ----------" << std::endl;
    cuda_extractor.ExtractDepthAndAmplitude(raw_frame.data(), raw_frame.size(),
                                             depth_cuda.data(), amplitude_cuda.data(),
                                             true);
    std::cout << "Center amplitude: " << amplitude_cuda[center_idx] << std::endl;
    std::cout << "Combined time: " << cuda_extractor.GetLastTotalTimeMs()
              << " ms" << std::endl;

    // Final verdict
    std::cout << "\n========================================" << std::endl;
    if (cuda_vs_sdk.correlation > 0.99 && cuda_vs_cpu.rmse < 1.0) {
        std::cout << "[PASS] CUDA implementation validated" << std::endl;
    } else if (cuda_vs_sdk.correlation > 0.95) {
        std::cout << "[WARN] CUDA results acceptable but degraded" << std::endl;
    } else {
        std::cout << "[FAIL] CUDA implementation needs debugging" << std::endl;
    }
    std::cout << "========================================\n" << std::endl;
}

void PrintUsage(const char* prog) {
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << prog << " <raw_file> <sdk_depth_file>  - Validation mode"
              << std::endl;
    std::cout << "  " << prog << " --benchmark <raw_file> [iterations]  - Benchmark mode"
              << std::endl;
    std::cout << "  " << prog << " --check  - Check CUDA availability" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "CubeEye CUDA Depth Extraction Test" << std::endl;
    std::cout << "===================================\n" << std::endl;

    if (argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    // Check mode
    if (strcmp(argv[1], "--check") == 0) {
        if (cuda::CudaDepthExtractor::IsCudaAvailable()) {
            std::cout << "[OK] CUDA is available" << std::endl;
            std::cout << "Device: " << cuda::CudaDepthExtractor::GetDeviceInfo()
                      << std::endl;
            return 0;
        } else {
            std::cout << "[ERROR] CUDA is not available" << std::endl;
            return 1;
        }
    }

    // Benchmark mode
    if (strcmp(argv[1], "--benchmark") == 0) {
        if (argc < 3) {
            std::cerr << "Error: --benchmark requires raw_file" << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }

        std::vector<uint8_t> raw_frame;
        if (!LoadRawFrame(argv[2], raw_frame)) {
            return 1;
        }

        int iterations = (argc > 3) ? std::atoi(argv[3]) : 100;
        if (iterations <= 0) iterations = 100;

        RunBenchmark(raw_frame, iterations);
        return 0;
    }

    // Validation mode
    if (argc < 3) {
        PrintUsage(argv[0]);
        return 1;
    }

    std::vector<uint8_t> raw_frame;
    std::vector<uint16_t> sdk_depth;

    if (!LoadRawFrame(argv[1], raw_frame)) {
        return 1;
    }

    if (!LoadSDKDepth(argv[2], sdk_depth)) {
        return 1;
    }

    RunValidation(raw_frame, sdk_depth);
    return 0;
}

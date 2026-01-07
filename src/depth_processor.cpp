/**
 * depth_processor.cpp - CubeEye I200D Depth Processor Implementation
 */

#include "depth_processor.h"
#include <chrono>
#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace cubeeye {

void DepthProcessor::Process(const uint16_t* raw_1600x241,
                             float* depth_320x240,
                             float* amplitude_320x240) {
    // Use default linear calibration
    ProcessWithCalibration(raw_1600x241, depth_320x240, CAL_SLOPE, CAL_OFFSET);

    // Extract amplitude if requested
    if (amplitude_320x240) {
        // Skip header row (row 0), data starts at row 1
        const uint16_t* pixel_data = raw_1600x241 + RAW_WIDTH;

        #pragma omp parallel for
        for (int y = 0; y < DEPTH_HEIGHT; ++y) {
            for (int x = 0; x < DEPTH_WIDTH; ++x) {
                int raw_idx = y * RAW_WIDTH + x * NUM_SUBPIXELS;
                // Sub[4] is amplitude
                amplitude_320x240[y * DEPTH_WIDTH + x] =
                    static_cast<float>(pixel_data[raw_idx + 4]);
            }
        }
    }
}

void DepthProcessor::ProcessWithCalibration(const uint16_t* raw_1600x241,
                                            float* depth_320x240,
                                            float slope, float offset) {
    auto start = std::chrono::high_resolution_clock::now();

    // Skip header row (row 0), data starts at row 1
    const uint16_t* pixel_data = raw_1600x241 + RAW_WIDTH;

    // First pass: compute mean amplitude for thresholding
    double sum_amp = 0.0;
    int count = 0;

    #pragma omp parallel for reduction(+:sum_amp,count)
    for (int y = 0; y < DEPTH_HEIGHT; ++y) {
        for (int x = 0; x < DEPTH_WIDTH; ++x) {
            int raw_idx = y * RAW_WIDTH + x * NUM_SUBPIXELS;
            uint16_t amp = pixel_data[raw_idx + 4];  // Sub[4] is amplitude
            if (amp > 0) {
                sum_amp += amp;
                count++;
            }
        }
    }

    float mean_amp = (count > 0) ? static_cast<float>(sum_amp / count) : 0.0f;
    float amp_threshold = mean_amp * AMP_THRESHOLD_FACTOR;

    // Second pass: compute depth
    double depth_sum = 0.0;
    double amp_sum = 0.0;
    int valid_count = 0;

    #pragma omp parallel for reduction(+:depth_sum,amp_sum,valid_count)
    for (int y = 0; y < DEPTH_HEIGHT; ++y) {
        for (int x = 0; x < DEPTH_WIDTH; ++x) {
            int raw_idx = y * RAW_WIDTH + x * NUM_SUBPIXELS;
            int out_idx = y * DEPTH_WIDTH + x;

            // Extract sub-pixels
            // Sub[0], Sub[1]: Black level reference (~121)
            // Sub[2], Sub[3]: Intensity measurements (correlate with depth)
            // Sub[4]: Amplitude
            uint16_t sub2 = pixel_data[raw_idx + 2];
            uint16_t sub3 = pixel_data[raw_idx + 3];
            uint16_t amp = pixel_data[raw_idx + 4];

            // Amplitude filtering
            if (amp < amp_threshold || sub2 == 0) {
                depth_320x240[out_idx] = 0.0f;  // Invalid pixel
                continue;
            }

            // Linear depth calculation using average of Sub[2] and Sub[3]
            // depth = slope * avg + offset
            float avg = (static_cast<float>(sub2) + static_cast<float>(sub3)) * 0.5f;
            float depth = slope * avg + offset;

            // Clamp to valid range
            if (depth < 0.0f) depth = 0.0f;
            if (depth > MAX_RANGE_MM) depth = MAX_RANGE_MM;

            depth_320x240[out_idx] = depth;

            depth_sum += depth;
            amp_sum += amp;
            valid_count++;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // Update stats
    last_stats_.mean_depth_mm = (valid_count > 0) ?
        static_cast<float>(depth_sum / valid_count) : 0.0f;
    last_stats_.mean_amplitude = (valid_count > 0) ?
        static_cast<float>(amp_sum / valid_count) : 0.0f;
    last_stats_.valid_pixels = valid_count;
    last_stats_.process_time_ms = static_cast<float>(duration.count()) / 1000.0f;
}

void DepthProcessor::ProcessQuadratic(const uint16_t* raw_1600x241,
                                       float* depth_320x240,
                                       float* amplitude_320x240) {
    auto start = std::chrono::high_resolution_clock::now();

    const uint16_t* pixel_data = raw_1600x241 + RAW_WIDTH;

    // First pass: compute mean amplitude
    double sum_amp = 0.0;
    int count = 0;

    #pragma omp parallel for reduction(+:sum_amp,count)
    for (int y = 0; y < DEPTH_HEIGHT; ++y) {
        for (int x = 0; x < DEPTH_WIDTH; ++x) {
            int raw_idx = y * RAW_WIDTH + x * NUM_SUBPIXELS;
            uint16_t amp = pixel_data[raw_idx + 4];
            if (amp > 0) {
                sum_amp += amp;
                count++;
            }
        }
    }

    float mean_amp = (count > 0) ? static_cast<float>(sum_amp / count) : 0.0f;
    float amp_threshold = mean_amp * AMP_THRESHOLD_FACTOR;

    // Second pass: compute depth with quadratic model
    double depth_sum = 0.0;
    double amp_sum = 0.0;
    int valid_count = 0;

    #pragma omp parallel for reduction(+:depth_sum,amp_sum,valid_count)
    for (int y = 0; y < DEPTH_HEIGHT; ++y) {
        for (int x = 0; x < DEPTH_WIDTH; ++x) {
            int raw_idx = y * RAW_WIDTH + x * NUM_SUBPIXELS;
            int out_idx = y * DEPTH_WIDTH + x;

            uint16_t sub2 = pixel_data[raw_idx + 2];
            uint16_t sub3 = pixel_data[raw_idx + 3];
            uint16_t amp = pixel_data[raw_idx + 4];

            if (amplitude_320x240) {
                amplitude_320x240[out_idx] = static_cast<float>(amp);
            }

            if (amp < amp_threshold || sub2 == 0) {
                depth_320x240[out_idx] = 0.0f;
                continue;
            }

            // Quadratic: depth = a * avg^2 + b * avg + c
            float avg = (static_cast<float>(sub2) + static_cast<float>(sub3)) * 0.5f;
            float depth = CAL_A * avg * avg + CAL_B * avg + CAL_C;

            if (depth < 0.0f) depth = 0.0f;
            if (depth > MAX_RANGE_MM) depth = MAX_RANGE_MM;

            depth_320x240[out_idx] = depth;
            depth_sum += depth;
            amp_sum += amp;
            valid_count++;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    last_stats_.mean_depth_mm = (valid_count > 0) ?
        static_cast<float>(depth_sum / valid_count) : 0.0f;
    last_stats_.mean_amplitude = (valid_count > 0) ?
        static_cast<float>(amp_sum / valid_count) : 0.0f;
    last_stats_.valid_pixels = valid_count;
    last_stats_.process_time_ms = static_cast<float>(duration.count()) / 1000.0f;
}

void DepthProcessor::ProcessMultiVar(const uint16_t* raw_1600x241,
                                      float* depth_320x240,
                                      float* amplitude_320x240) {
    auto start = std::chrono::high_resolution_clock::now();

    const uint16_t* pixel_data = raw_1600x241 + RAW_WIDTH;

    // Multi-variable calibration coefficients
    constexpr float K1 = 0.071687f;    // avg coefficient
    constexpr float K2 = -5.891034f;   // diff coefficient
    constexpr float K3 = -890.35f;     // offset

    // First pass: compute mean amplitude
    double sum_amp = 0.0;
    int count = 0;

    #pragma omp parallel for reduction(+:sum_amp,count)
    for (int y = 0; y < DEPTH_HEIGHT; ++y) {
        for (int x = 0; x < DEPTH_WIDTH; ++x) {
            int raw_idx = y * RAW_WIDTH + x * NUM_SUBPIXELS;
            uint16_t amp = pixel_data[raw_idx + 4];
            if (amp > 0) {
                sum_amp += amp;
                count++;
            }
        }
    }

    float mean_amp = (count > 0) ? static_cast<float>(sum_amp / count) : 0.0f;
    float amp_threshold = mean_amp * AMP_THRESHOLD_FACTOR;

    // Second pass: compute depth with multi-variable model
    double depth_sum = 0.0;
    double amp_sum = 0.0;
    int valid_count = 0;

    #pragma omp parallel for reduction(+:depth_sum,amp_sum,valid_count)
    for (int y = 0; y < DEPTH_HEIGHT; ++y) {
        for (int x = 0; x < DEPTH_WIDTH; ++x) {
            int raw_idx = y * RAW_WIDTH + x * NUM_SUBPIXELS;
            int out_idx = y * DEPTH_WIDTH + x;

            uint16_t sub2 = pixel_data[raw_idx + 2];
            uint16_t sub3 = pixel_data[raw_idx + 3];
            uint16_t amp = pixel_data[raw_idx + 4];

            if (amplitude_320x240) {
                amplitude_320x240[out_idx] = static_cast<float>(amp);
            }

            if (amp < amp_threshold || sub2 == 0) {
                depth_320x240[out_idx] = 0.0f;
                continue;
            }

            // Multi-variable: depth = k1 * avg + k2 * diff + k3
            float avg = (static_cast<float>(sub2) + static_cast<float>(sub3)) * 0.5f;
            float diff = static_cast<float>(sub2) - static_cast<float>(sub3);
            float depth = K1 * avg + K2 * diff + K3;

            if (depth < 0.0f) depth = 0.0f;
            if (depth > MAX_RANGE_MM) depth = MAX_RANGE_MM;

            depth_320x240[out_idx] = depth;
            depth_sum += depth;
            amp_sum += amp;
            valid_count++;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    last_stats_.mean_depth_mm = (valid_count > 0) ?
        static_cast<float>(depth_sum / valid_count) : 0.0f;
    last_stats_.mean_amplitude = (valid_count > 0) ?
        static_cast<float>(amp_sum / valid_count) : 0.0f;
    last_stats_.valid_pixels = valid_count;
    last_stats_.process_time_ms = static_cast<float>(duration.count()) / 1000.0f;
}

} // namespace cubeeye

/**
 * frame_analyzer.cpp - Raw Frame Data Analyzer
 *
 * Analyzes captured raw frames to determine sub-pixel layout
 * and extract phase/amplitude information.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <numeric>

constexpr int FRAME_WIDTH = 1600;
constexpr int FRAME_HEIGHT = 241;
constexpr int SPATIAL_WIDTH = 320;
constexpr int SUB_PIXELS = 5;

void analyze_file(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "Error: Cannot open " << filename << std::endl;
        return;
    }

    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint16_t> data(size / sizeof(uint16_t));
    file.read(reinterpret_cast<char*>(data.data()), size);

    std::cout << "\n=== Analyzing: " << filename << " ===" << std::endl;
    std::cout << "File size: " << size << " bytes (" << data.size() << " uint16 values)" << std::endl;
    std::cout << "Expected: " << FRAME_WIDTH * FRAME_HEIGHT * 2 << " bytes" << std::endl;

    if (data.size() != FRAME_WIDTH * FRAME_HEIGHT) {
        std::cerr << "Warning: Unexpected frame size!" << std::endl;
        return;
    }

    // Header analysis
    std::cout << "\n--- Header (Row 0) ---" << std::endl;
    std::cout << "First 32 values:" << std::endl;
    for (int i = 0; i < 32; i++) {
        if (i % 8 == 0) std::cout << "  [" << std::setw(2) << i << "]: ";
        std::cout << std::setw(6) << data[i] << " ";
        if (i % 8 == 7) std::cout << std::endl;
    }

    // Sub-pixel statistics (skip header row)
    std::cout << "\n--- Sub-pixel Statistics (rows 1-240) ---" << std::endl;

    std::vector<std::vector<uint16_t>> sub_data(SUB_PIXELS);

    for (int row = 1; row < FRAME_HEIGHT; row++) {
        for (int x = 0; x < SPATIAL_WIDTH; x++) {
            int base = row * FRAME_WIDTH + x * SUB_PIXELS;
            for (int s = 0; s < SUB_PIXELS; s++) {
                sub_data[s].push_back(data[base + s]);
            }
        }
    }

    for (int s = 0; s < SUB_PIXELS; s++) {
        auto& vals = sub_data[s];
        double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
        double mean = sum / vals.size();

        double var_sum = 0;
        for (uint16_t v : vals) {
            var_sum += (v - mean) * (v - mean);
        }
        double std_dev = std::sqrt(var_sum / vals.size());

        std::sort(vals.begin(), vals.end());
        uint16_t median = vals[vals.size() / 2];
        uint16_t min_val = vals.front();
        uint16_t max_val = vals.back();

        std::cout << "Sub[" << s << "]: mean=" << std::setw(8) << std::fixed << std::setprecision(1) << mean
                  << ", std=" << std::setw(7) << std_dev
                  << ", median=" << std::setw(6) << median
                  << ", min=" << std::setw(6) << min_val
                  << ", max=" << std::setw(6) << max_val << std::endl;
    }

    // Interpretation based on PLAN.md hypothesis
    std::cout << "\n--- Interpretation (Hypothesis) ---" << std::endl;
    std::cout << "Sub[0]: ~2000 mean - 80MHz phase/I component" << std::endl;
    std::cout << "Sub[1]: ~2000 mean - 80MHz Q component or amplitude" << std::endl;
    std::cout << "Sub[2]: ~6500 mean - 100MHz phase/I component" << std::endl;
    std::cout << "Sub[3]: ~19500 mean - 100MHz Q component or amplitude" << std::endl;
    std::cout << "Sub[4]: ~20000 mean - amplitude or metadata" << std::endl;

    // Center pixel detailed analysis
    int cx = SPATIAL_WIDTH / 2;
    int cy = FRAME_HEIGHT / 2;
    int center_base = cy * FRAME_WIDTH + cx * SUB_PIXELS;

    std::cout << "\n--- Center Pixel (" << cx << ", " << cy << ") ---" << std::endl;
    for (int s = 0; s < SUB_PIXELS; s++) {
        std::cout << "Sub[" << s << "]: " << data[center_base + s] << std::endl;
    }

    // Compute phase assuming I/Q pairs
    // If Sub[0]/Sub[1] are 80MHz I/Q and Sub[2]/Sub[3] are 100MHz I/Q:
    double I_80 = data[center_base + 0];
    double Q_80 = data[center_base + 1];
    double I_100 = data[center_base + 2];
    double Q_100 = data[center_base + 3];

    double phase_80 = std::atan2(Q_80, I_80);
    double phase_100 = std::atan2(Q_100, I_100);

    double amp_80 = std::sqrt(I_80 * I_80 + Q_80 * Q_80);
    double amp_100 = std::sqrt(I_100 * I_100 + Q_100 * Q_100);

    std::cout << "\n--- Phase/Amplitude Calculation (if I/Q hypothesis) ---" << std::endl;
    std::cout << "80MHz: phase=" << std::setprecision(3) << phase_80 << " rad ("
              << (phase_80 * 180 / M_PI) << " deg), amp=" << amp_80 << std::endl;
    std::cout << "100MHz: phase=" << phase_100 << " rad ("
              << (phase_100 * 180 / M_PI) << " deg), amp=" << amp_100 << std::endl;

    // Convert phase to depth
    constexpr double c = 299792458.0;  // speed of light m/s
    constexpr double f_80 = 80e6;
    constexpr double f_100 = 100e6;
    constexpr double range_80 = c / (2 * f_80) * 1000;   // 1874.95 mm
    constexpr double range_100 = c / (2 * f_100) * 1000; // 1499.96 mm

    // Phase is -pi to +pi, need to map to 0-2pi
    double phase_80_norm = (phase_80 < 0) ? (phase_80 + 2 * M_PI) : phase_80;
    double phase_100_norm = (phase_100 < 0) ? (phase_100 + 2 * M_PI) : phase_100;

    double depth_80 = phase_80_norm / (2 * M_PI) * range_80;
    double depth_100 = phase_100_norm / (2 * M_PI) * range_100;

    std::cout << "\n--- Single-Frequency Depth (wrapped) ---" << std::endl;
    std::cout << "80MHz depth (0-1875mm): " << std::setprecision(1) << depth_80 << " mm" << std::endl;
    std::cout << "100MHz depth (0-1500mm): " << depth_100 << " mm" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "CubeEye Frame Analyzer" << std::endl;
    std::cout << "======================" << std::endl;

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <frame.raw> [frame2.raw] ..." << std::endl;
        std::cout << "Analyzing default files in data/..." << std::endl;

        analyze_file("data/v4l2_frame_0.raw");
        analyze_file("data/sdk_depth_frame_0.raw");
    } else {
        for (int i = 1; i < argc; i++) {
            analyze_file(argv[i]);
        }
    }

    return 0;
}

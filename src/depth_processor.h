#pragma once
/**
 * depth_processor.h - CubeEye I200D Depth Processor
 *
 * Converts raw 1600x241 sensor data to 320x240 depth map.
 *
 * Sensor: Infineon IRS2976C in 320x240 binned mode
 * Current method: Linear intensity-to-depth mapping (R²=0.77)
 *
 * TODO: Implement proper phase-based calculation when format is understood
 */

#include <cstdint>
#include <cmath>

namespace cubeeye {

// Frame dimensions
constexpr int RAW_WIDTH = 1600;
constexpr int RAW_HEIGHT = 241;
constexpr int DEPTH_WIDTH = 320;
constexpr int DEPTH_HEIGHT = 240;
constexpr int NUM_SUBPIXELS = 5;

// Sub-pixel data format (confirmed from raw data analysis):
// Sub[0], Sub[1]: 9-bit black level references (range 0-509)
// Sub[2]: Full 16-bit intensity/depth data
// Sub[3]: 8-bit value in HIGH byte (Sub[3] >> 8 to extract)
// Sub[4]: 8-bit amplitude in HIGH byte (Sub[4] >> 8 to extract)
//
// IMPORTANT: Sub[3] and Sub[4] are 99.8% multiples of 256!
// The actual 8-bit value is stored in bits 15:8, not 7:0.

// Calibration constants from ruler measurements (0.5m, 1.0m, 1.5m)
// Using Sub[2] as primary depth indicator (full 16-bit precision)
//
// Linear model: depth_mm = CAL_SLOPE * Sub[2] + CAL_OFFSET
constexpr float CAL_SLOPE = 0.12881366f;    // mm per unit
constexpr float CAL_OFFSET = -2763.38f;     // mm offset

// Quadratic model: depth_mm = CAL_A * Sub[2]^2 + CAL_B * Sub[2] + CAL_C
constexpr float CAL_A = 0.0000284721f;
constexpr float CAL_B = -1.487768f;
constexpr float CAL_C = 19901.57f;

// Amplitude threshold for valid pixels (relative to mean)
constexpr float AMP_THRESHOLD_FACTOR = 0.3f;

// Modulation frequency (confirmed ~76.5 MHz)
constexpr float MOD_FREQ_HZ = 76.5e6f;
constexpr float SPEED_OF_LIGHT_MM = 299792458000.0f;
constexpr float MAX_RANGE_MM = SPEED_OF_LIGHT_MM / (2.0f * MOD_FREQ_HZ);  // ~1958mm

class DepthProcessor {
public:
    DepthProcessor() = default;

    /**
     * Process raw sensor data to depth map
     *
     * @param raw_1600x241  Raw frame (1600x241 uint16, includes header row)
     * @param depth_320x240 Output depth map (320x240 float, in mm)
     * @param amplitude_320x240 Optional output amplitude map (can be nullptr)
     */
    void Process(const uint16_t* raw_1600x241,
                 float* depth_320x240,
                 float* amplitude_320x240 = nullptr);

    /**
     * Process with custom linear calibration
     * depth = slope * avg + offset
     */
    void ProcessWithCalibration(const uint16_t* raw_1600x241,
                                float* depth_320x240,
                                float slope, float offset);

    /**
     * Process with quadratic calibration (better accuracy)
     * depth = a * avg^2 + b * avg + c
     */
    void ProcessQuadratic(const uint16_t* raw_1600x241,
                          float* depth_320x240,
                          float* amplitude_320x240 = nullptr);

    /**
     * Process with multi-variable calibration (best accuracy)
     * depth = k1 * avg + k2 * diff + k3
     * where avg = (Sub[2]+Sub[3])/2 and diff = Sub[2]-Sub[3]
     */
    void ProcessMultiVar(const uint16_t* raw_1600x241,
                         float* depth_320x240,
                         float* amplitude_320x240 = nullptr);

    /**
     * Get processing statistics from last frame
     */
    struct Stats {
        float mean_depth_mm;
        float mean_amplitude;
        int valid_pixels;
        float process_time_ms;
    };
    const Stats& GetLastStats() const { return last_stats_; }

private:
    Stats last_stats_{};
};

} // namespace cubeeye

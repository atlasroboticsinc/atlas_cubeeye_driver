/**
 * cubeeye_depth.h - CubeEye I200D Depth Extraction
 *
 * Extracts depth from V4L2 frames using validated 5-byte unpacking formula.
 *
 * Frame format:
 * - Total: 771,200 bytes (241 rows × 3,200 bytes/row)
 * - Row 0: Header (3,200 bytes)
 * - Rows 1-240: [amplitude_1600 bytes][depth_1600 bytes]
 *
 * 5-byte unpacking (produces 2 depth values from 5 bytes):
 *   Pixel 0: depth = ((((b4>>2)&0x03)|(b1<<2)) << 10) + ((b4&0x03)|(b0<<2))
 *   Pixel 1: depth = (((b4>>6)|(b3<<2)) << 10) + (((b4>>4)&0x03)|(b2<<2))
 *
 * Output: 640×480 depth in mm (2× vertical interpolation from 640×240)
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <memory>

namespace cubeeye {

// Frame dimensions
constexpr size_t RAW_FRAME_SIZE = 771200;    // Total bytes per frame
constexpr size_t HEADER_SIZE = 3200;          // Header row size
constexpr size_t BYTES_PER_ROW = 3200;        // Data row size
constexpr size_t NUM_DATA_ROWS = 240;         // Number of data rows
constexpr size_t DEPTH_SECTION_OFFSET = 1600; // Offset to depth data in each row
constexpr size_t DEPTH_SECTION_SIZE = 1600;   // Bytes of depth data per row

constexpr int OUTPUT_WIDTH = 640;
constexpr int OUTPUT_HEIGHT = 480;
constexpr int RAW_WIDTH = 640;                // After 5-byte unpacking
constexpr int RAW_HEIGHT = 240;               // Before interpolation

constexpr int MAX_DEPTH_MM = 7500;

// Gradient correction polynomial coefficients (13th degree)
constexpr double GRADIENT_COEFFS[14] = {
    -9.46306765e-08,   // x^13
    5.40828695e-06,    // x^12
    -0.000133821166,   // x^11
    0.00189463573,     // x^10
    -0.0170465988,     // x^9
    0.102187397,       // x^8
    -0.415584587,      // x^7
    1.14433615,        // x^6
    -2.09044109,       // x^5
    2.42940077,        // x^4
    -1.66835357,       // x^3
    0.587854516,       // x^2
    -0.076622637,      // x^1
    0.000344344841,    // x^0
};

/**
 * DepthExtractor - Extract depth from CubeEye I200D V4L2 frames
 */
class DepthExtractor {
public:
    DepthExtractor(bool apply_gradient_correction = true);
    ~DepthExtractor() = default;

    /**
     * Extract depth from raw V4L2 frame
     *
     * @param raw_frame Raw frame bytes (771,200 bytes)
     * @param frame_size Size of raw frame (should be 771,200)
     * @param depth_out Output depth buffer (640×480 uint16, in mm)
     * @param interpolate If true, output 640×480. If false, output 640×240.
     * @return true on success
     */
    bool ExtractDepth(const uint8_t* raw_frame, size_t frame_size,
                      uint16_t* depth_out, bool interpolate = true);

    /**
     * Extract amplitude from raw V4L2 frame
     *
     * @param raw_frame Raw frame bytes
     * @param frame_size Size of raw frame
     * @param amplitude_out Output amplitude buffer
     * @param interpolate If true, output 640×480
     * @return true on success
     */
    bool ExtractAmplitude(const uint8_t* raw_frame, size_t frame_size,
                          uint16_t* amplitude_out, bool interpolate = true);

    /**
     * Extract both depth and amplitude
     */
    bool ExtractDepthAndAmplitude(const uint8_t* raw_frame, size_t frame_size,
                                   uint16_t* depth_out, uint16_t* amplitude_out,
                                   bool interpolate = true);

    /**
     * Get output dimensions based on interpolation setting
     */
    static void GetOutputDimensions(bool interpolate, int& width, int& height) {
        width = OUTPUT_WIDTH;
        height = interpolate ? OUTPUT_HEIGHT : RAW_HEIGHT;
    }

    /**
     * Enable/disable gradient correction
     */
    void SetGradientCorrection(bool enable) { apply_gradient_correction_ = enable; }
    bool GetGradientCorrection() const { return apply_gradient_correction_; }

private:
    /**
     * Unpack depth section of a row using 5-byte formula
     * 1600 bytes → 640 depth values
     */
    void UnpackDepthRow(const uint8_t* depth_bytes, uint16_t* depth_out);

    /**
     * Apply gradient correction to depth frame
     */
    void ApplyGradientCorrection(uint16_t* depth_frame, int num_pixels);

    /**
     * Interpolate 240 rows to 480 rows (nearest neighbor)
     */
    void InterpolateVertical(const uint16_t* src_240, uint16_t* dst_480);

    bool apply_gradient_correction_;
    int16_t gradient_lut_[MAX_DEPTH_MM + 1];  // Precomputed correction LUT
};

/**
 * Inline 5-byte unpacking function (for use in CUDA kernels)
 */
inline void Unpack5Bytes(const uint8_t* bytes, uint16_t& depth0, uint16_t& depth1) {
    uint8_t b0 = bytes[0];
    uint8_t b1 = bytes[1];
    uint8_t b2 = bytes[2];
    uint8_t b3 = bytes[3];
    uint8_t b4 = bytes[4];

    // Pixel 0: coarse = ((b4>>2)&0x03)|(b1<<2), fine = (b4&0x03)|(b0<<2)
    uint16_t coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2);
    uint16_t fine0 = (b4 & 0x03) | (b0 << 2);
    depth0 = (coarse0 << 10) + fine0;

    // Pixel 1: coarse = (b4>>6)|(b3<<2), fine = ((b4>>4)&0x03)|(b2<<2)
    uint16_t coarse1 = (b4 >> 6) | (b3 << 2);
    uint16_t fine1 = ((b4 >> 4) & 0x03) | (b2 << 2);
    depth1 = (coarse1 << 10) + fine1;
}

} // namespace cubeeye

/**
 * cubeeye_depth.cpp - CubeEye I200D Depth Extraction Implementation
 */

#include "cubeeye_depth.h"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace cubeeye {

DepthExtractor::DepthExtractor(bool apply_gradient_correction)
    : apply_gradient_correction_(apply_gradient_correction)
{
    // Build gradient correction lookup table
    for (int depth_mm = 0; depth_mm <= MAX_DEPTH_MM; ++depth_mm) {
        double x = depth_mm / 1000.0;  // Convert to meters

        // Evaluate polynomial: sum(coeff[i] * x^(13-i)) for i=0..13
        double correction = 0.0;
        double x_power = 1.0;

        // Start from x^0 and work up
        for (int i = 13; i >= 0; --i) {
            correction += GRADIENT_COEFFS[i] * x_power;
            if (i > 0) x_power *= x;
        }

        // Correction is in meters, convert to mm and store as int16
        gradient_lut_[depth_mm] = static_cast<int16_t>(std::round(correction * 1000.0));
    }
}

void DepthExtractor::UnpackDepthRow(const uint8_t* depth_bytes, uint16_t* depth_out) {
    int out_idx = 0;

    // Process 5 bytes at a time, producing 2 depth values
    for (int i = 0; i < DEPTH_SECTION_SIZE - 4; i += 5) {
        uint16_t d0, d1;
        Unpack5Bytes(depth_bytes + i, d0, d1);

        if (out_idx < RAW_WIDTH) depth_out[out_idx++] = d0;
        if (out_idx < RAW_WIDTH) depth_out[out_idx++] = d1;
    }
}

void DepthExtractor::ApplyGradientCorrection(uint16_t* depth_frame, int num_pixels) {
    for (int i = 0; i < num_pixels; ++i) {
        int depth = depth_frame[i];
        if (depth > 0 && depth <= MAX_DEPTH_MM) {
            // Apply correction (add the correction value)
            int corrected = depth + gradient_lut_[depth];
            depth_frame[i] = static_cast<uint16_t>(std::clamp(corrected, 0, MAX_DEPTH_MM));
        }
    }
}

void DepthExtractor::InterpolateVertical(const uint16_t* src_240, uint16_t* dst_480) {
    // Simple nearest neighbor interpolation (duplicate rows)
    for (int row = 0; row < RAW_HEIGHT; ++row) {
        const uint16_t* src_row = src_240 + row * RAW_WIDTH;
        uint16_t* dst_row0 = dst_480 + (row * 2) * OUTPUT_WIDTH;
        uint16_t* dst_row1 = dst_480 + (row * 2 + 1) * OUTPUT_WIDTH;

        std::memcpy(dst_row0, src_row, RAW_WIDTH * sizeof(uint16_t));
        std::memcpy(dst_row1, src_row, RAW_WIDTH * sizeof(uint16_t));
    }
}

bool DepthExtractor::ExtractDepth(const uint8_t* raw_frame, size_t frame_size,
                                   uint16_t* depth_out, bool interpolate) {
    if (!raw_frame || !depth_out || frame_size < RAW_FRAME_SIZE) {
        return false;
    }

    // Skip header row
    const uint8_t* data_start = raw_frame + HEADER_SIZE;

    // Temporary buffer for 640x240 depth
    uint16_t depth_240[RAW_HEIGHT * RAW_WIDTH];

    // Process each data row
    for (int row = 0; row < NUM_DATA_ROWS; ++row) {
        const uint8_t* row_data = data_start + row * BYTES_PER_ROW;

        // Depth section is second half of row (bytes 1600-3199)
        const uint8_t* depth_section = row_data + DEPTH_SECTION_OFFSET;

        // Unpack to output
        UnpackDepthRow(depth_section, depth_240 + row * RAW_WIDTH);
    }

    // Apply gradient correction
    if (apply_gradient_correction_) {
        ApplyGradientCorrection(depth_240, RAW_HEIGHT * RAW_WIDTH);
    }

    // Output
    if (interpolate) {
        InterpolateVertical(depth_240, depth_out);
    } else {
        std::memcpy(depth_out, depth_240, RAW_HEIGHT * RAW_WIDTH * sizeof(uint16_t));
    }

    return true;
}

bool DepthExtractor::ExtractAmplitude(const uint8_t* raw_frame, size_t frame_size,
                                       uint16_t* amplitude_out, bool interpolate) {
    if (!raw_frame || !amplitude_out || frame_size < RAW_FRAME_SIZE) {
        return false;
    }

    const uint8_t* data_start = raw_frame + HEADER_SIZE;

    uint16_t amp_240[RAW_HEIGHT * RAW_WIDTH];

    for (int row = 0; row < NUM_DATA_ROWS; ++row) {
        const uint8_t* row_data = data_start + row * BYTES_PER_ROW;

        // Amplitude section is first half of row (bytes 0-1599)
        // Uses same 5-byte unpacking
        UnpackDepthRow(row_data, amp_240 + row * RAW_WIDTH);
    }

    if (interpolate) {
        InterpolateVertical(amp_240, amplitude_out);
    } else {
        std::memcpy(amplitude_out, amp_240, RAW_HEIGHT * RAW_WIDTH * sizeof(uint16_t));
    }

    return true;
}

bool DepthExtractor::ExtractDepthAndAmplitude(const uint8_t* raw_frame, size_t frame_size,
                                               uint16_t* depth_out, uint16_t* amplitude_out,
                                               bool interpolate) {
    if (!raw_frame || !depth_out || !amplitude_out || frame_size < RAW_FRAME_SIZE) {
        return false;
    }

    const uint8_t* data_start = raw_frame + HEADER_SIZE;

    uint16_t depth_240[RAW_HEIGHT * RAW_WIDTH];
    uint16_t amp_240[RAW_HEIGHT * RAW_WIDTH];

    for (int row = 0; row < NUM_DATA_ROWS; ++row) {
        const uint8_t* row_data = data_start + row * BYTES_PER_ROW;

        // Amplitude: first 1600 bytes
        UnpackDepthRow(row_data, amp_240 + row * RAW_WIDTH);

        // Depth: second 1600 bytes
        UnpackDepthRow(row_data + DEPTH_SECTION_OFFSET, depth_240 + row * RAW_WIDTH);
    }

    if (apply_gradient_correction_) {
        ApplyGradientCorrection(depth_240, RAW_HEIGHT * RAW_WIDTH);
    }

    if (interpolate) {
        InterpolateVertical(depth_240, depth_out);
        InterpolateVertical(amp_240, amplitude_out);
    } else {
        std::memcpy(depth_out, depth_240, RAW_HEIGHT * RAW_WIDTH * sizeof(uint16_t));
        std::memcpy(amplitude_out, amp_240, RAW_HEIGHT * RAW_WIDTH * sizeof(uint16_t));
    }

    return true;
}

} // namespace cubeeye

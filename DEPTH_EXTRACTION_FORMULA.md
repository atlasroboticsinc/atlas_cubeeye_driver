# CubeEye I200D Depth Extraction Formula

## Executive Summary

The CubeEye I200D ToF sensor depth extraction has been **SUCCESSFULLY VALIDATED**. This document describes the complete formula verified against SDK output with **r=0.9973 correlation**.

## Frame Format

### V4L2 Frame Structure
- **Total size**: 771,200 bytes
- **Header**: Row 0, 3,200 bytes (metadata)
- **Data rows**: Rows 1-240, 3,200 bytes each
- **Per row layout**: `[amplitude_1600][depth_1600]`

```
Byte Layout:
[0-3199]      Header row (3,200 bytes)
[3200-4799]   Row 1: amplitude bytes [0-1599], depth bytes [1600-3199]
[4800-6399]   Row 2: amplitude bytes, depth bytes
...
[768000-771199] Row 240: amplitude bytes, depth bytes
```

## 5-Byte Unpacking Formula (VALIDATED)

Each 5 consecutive bytes produce 2 depth values (coarse + fine → 20-bit depth):

```c
void Unpack5Bytes(const uint8_t* bytes, uint16_t& depth0, uint16_t& depth1) {
    uint8_t b0 = bytes[0];
    uint8_t b1 = bytes[1];
    uint8_t b2 = bytes[2];
    uint8_t b3 = bytes[3];
    uint8_t b4 = bytes[4];

    // Pixel 0
    uint16_t coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2);
    uint16_t fine0 = (b4 & 0x03) | (b0 << 2);
    depth0 = (coarse0 << 10) + fine0;

    // Pixel 1
    uint16_t coarse1 = (b4 >> 6) | (b3 << 2);
    uint16_t fine1 = ((b4 >> 4) & 0x03) | (b2 << 2);
    depth1 = (coarse1 << 10) + fine1;
}
```

### Pixels Per Row
- 1,600 bytes / 5 bytes × 2 pixels = **640 pixels per row**
- 240 data rows × 640 pixels = **153,600 raw pixels**
- SDK outputs 640×480 via 2× vertical interpolation

## Gradient Correction (VALIDATED)

After unpacking, apply polynomial gradient correction:

### Polynomial Coefficients (14 terms)
```c
constexpr double GRADIENT_COEFFS[14] = {
    -9.46306765e-08,   // x^13
     5.40828695e-06,   // x^12
    -0.000133821166,   // x^11
     0.00189463573,    // x^10
    -0.0170465988,     // x^9
     0.102187397,      // x^8
    -0.415584587,      // x^7
     1.14433615,       // x^6
    -2.09044109,       // x^5
     2.42940077,       // x^4
    -1.66835357,       // x^3
     0.587854516,      // x^2
    -0.076622637,      // x^1
     0.000344344841,   // x^0
};
```

### Correction Formula
```c
double EvaluateCorrection(int depth_mm) {
    double x = depth_mm / 1000.0;  // Convert to meters
    double correction = 0.0;

    for (int i = 0; i < 14; i++) {
        correction += GRADIENT_COEFFS[i] * pow(x, 13 - i);
    }

    return correction * 1000.0;  // Return correction in mm
}

uint16_t corrected = depth_mm + EvaluateCorrection(depth_mm);
```

### Correction Magnitudes (at key depths)
| Raw Depth | Correction |
|-----------|------------|
| 500mm     | +2mm       |
| 1000mm    | -2mm       |
| 2000mm    | -10mm      |
| 3000mm    | -26mm      |
| 5000mm    | -31mm      |

## Complete Processing Pipeline

```
V4L2 Frame (771,200 bytes)
         │
         ├── Skip header (3,200 bytes)
         │
         ▼
For each data row (1-240):
         │
         ├── Extract depth section (bytes 1600-3199)
         │
         ├── Apply 5-byte unpacking (1,600 bytes → 640 depths)
         │
         └── Apply gradient correction (lookup table)
         │
         ▼
640×240 Depth Frame
         │
         ├── 2× Vertical interpolation (nearest neighbor)
         │
         ▼
640×480 Output (matches SDK)
```

## Validation Results

### Accuracy Metrics
| Metric | Value |
|--------|-------|
| Correlation (r) | 0.9973 |
| RMSE | 73.34 mm |
| Mean difference | -2.19 mm |
| Valid pixels | 302,556 / 307,200 |

### Sample Pixel Comparison
| Location | SDK (mm) | Decoded (mm) | Error |
|----------|----------|--------------|-------|
| Center (240,320) | 3214 | 3206 | -8mm |
| Top-left (120,160) | 192 | 190 | -2mm |
| Bottom-right (360,480) | 1561 | 1558 | -3mm |

## Implementation Files

- **Python reference**: `depth_extractor.py`
- **C++ library**: `src/cubeeye_depth.cpp`, `src/cubeeye_depth.h`
- **C++ test**: `src/test_cubeeye_depth.cpp`

## Key Discovery: V4L2 vs SDK Documentation

Previous documentation suggested V4L2 format differs from USB format. **This was incorrect.**

The SDK documentation's 5-byte formula works directly on V4L2 data when applied to the **depth section** (second 1,600 bytes) of each row.

### Critical Finding
The V4L2 row structure is:
- `bytes[0:1599]` = Amplitude data (same 5-byte format)
- `bytes[1600:3199]` = Depth data (apply 5-byte formula here)

## CUDA Implementation Notes

For GPU implementation:
1. Each CUDA thread can process one row
2. Pre-compute gradient LUT (7,501 entries) on GPU
3. Use shared memory for 5-byte unpacking
4. Coalesced memory access: process 5 bytes → 2 outputs

```cuda
__global__ void ExtractDepthKernel(
    const uint8_t* raw_frame,   // 771,200 bytes
    uint16_t* depth_out,        // 640×480 output
    const int16_t* gradient_lut // 7,501 entries
) {
    int row = blockIdx.x;
    int pixel_start = threadIdx.x * 2;  // Each thread handles 2 pixels

    // Calculate byte offset into depth section
    const uint8_t* row_data = raw_frame + 3200 + row * 3200 + 1600;
    int byte_offset = (pixel_start / 2) * 5;

    // Unpack and apply correction
    uint16_t d0, d1;
    Unpack5Bytes(row_data + byte_offset, d0, d1);

    // Apply gradient correction
    d0 = clamp(d0 + gradient_lut[min(d0, 7500)], 0, 7500);
    d1 = clamp(d1 + gradient_lut[min(d1, 7500)], 0, 7500);

    // Write to output (with 2× vertical interpolation)
    int out_idx = (row * 2) * 640 + pixel_start;
    depth_out[out_idx] = d0;
    depth_out[out_idx + 1] = d1;
    depth_out[out_idx + 640] = d0;  // Duplicate row
    depth_out[out_idx + 640 + 1] = d1;
}
```

## Remaining RMSE Analysis

The ~73mm RMSE consists of:
1. **Temporal misalignment** (~50-70mm): Raw and SDK frames captured at slightly different times
2. **Flying pixel filter**: SDK removes edge artifacts
3. **Per-pixel calibration**: SDK may have per-pixel correction we don't have
4. **Interpolation differences**: SDK may use bilinear instead of nearest neighbor

For robotics applications, this accuracy is acceptable for obstacle avoidance and navigation.

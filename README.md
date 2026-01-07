# CubeEye I200D Custom Depth Extraction Driver

A reverse-engineered depth extraction implementation for the CubeEye I200D Time-of-Flight (ToF) sensor, bypassing the proprietary SDK for direct V4L2 access with GPU-ready processing.

## Overview

The CubeEye I200D is a ToF depth sensor that outputs 640x480 depth images at ~30fps. The official SDK (`libCubeEye.so`) is closed-source and performs all depth computation on the CPU. This project reverse-engineers the depth extraction algorithm to enable:

- **Direct V4L2 capture** without SDK dependency
- **GPU acceleration** via CUDA (planned)
- **Custom processing pipelines** for robotics applications
- **Lower latency** by eliminating SDK overhead

## Key Achievements

| Metric | Value |
|--------|-------|
| Correlation with SDK | r = 0.9957 |
| Center pixel accuracy | ±1mm typical |
| Close-range accuracy (<600mm) | ±0.7mm mean error |
| Full-frame RMSE | ~85mm (includes edge artifacts) |

## Quick Start

### Prerequisites

```bash
# Build tools
sudo apt-get install build-essential cmake

# The CubeEye SDK (for ground truth comparison only)
# Located at: ~/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0
```

### Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Run Depth Extraction Test

```bash
# Capture test data (requires connected CubeEye sensor)
./run_with_hook.sh data/test_capture 10

# Verify extraction accuracy
./build/test_cubeeye_depth
```

## Technical Details

### Frame Format

The CubeEye I200D delivers frames via V4L2/UVC in the following format:

```
Total Frame: 771,200 bytes
├── Header Row (3,200 bytes) - Metadata
└── Data Rows 1-240 (3,200 bytes each)
    ├── Amplitude Section (bytes 0-1599)
    └── Depth Section (bytes 1600-3199)
```

### 5-Byte Unpacking Algorithm

Each 5 consecutive bytes in the depth section produce 2 depth values:

```c
void Unpack5Bytes(const uint8_t* bytes, uint16_t& depth0, uint16_t& depth1) {
    uint8_t b0 = bytes[0], b1 = bytes[1], b2 = bytes[2];
    uint8_t b3 = bytes[3], b4 = bytes[4];

    // Pixel 0: coarse + fine → 20-bit depth
    uint16_t coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2);
    uint16_t fine0 = (b4 & 0x03) | (b0 << 2);
    depth0 = (coarse0 << 10) + fine0;

    // Pixel 1
    uint16_t coarse1 = (b4 >> 6) | (b3 << 2);
    uint16_t fine1 = ((b4 >> 4) & 0x03) | (b2 << 2);
    depth1 = (coarse1 << 10) + fine1;
}
```

### Gradient Correction

A 14-coefficient polynomial corrects systematic depth-dependent errors:

| Depth | Correction |
|-------|------------|
| 500mm | +2mm |
| 1000mm | -2mm |
| 2000mm | -10mm |
| 3000mm | -26mm |
| 5000mm | -31mm |

See `DEPTH_EXTRACTION_FORMULA.md` for complete polynomial coefficients.

### Processing Pipeline

```
V4L2 Frame (771,200 bytes)
         │
         ├── Skip header (3,200 bytes)
         │
         ▼
For each data row (240 rows):
         │
         ├── Extract depth section (bytes 1600-3199)
         ├── Apply 5-byte unpacking → 640 depth values
         └── Apply gradient correction (LUT)
         │
         ▼
640×240 Depth Frame
         │
         ├── 2× Vertical interpolation
         │
         ▼
640×480 Output (matches SDK)
```

## Project Structure

```
cubeeye_nano_driver/
├── README.md                      # This file
├── DEPTH_EXTRACTION_FORMULA.md    # Detailed algorithm documentation
├── CMakeLists.txt                 # Build configuration
│
├── src/
│   ├── cubeeye_depth.h            # C++ depth extraction API
│   ├── cubeeye_depth.cpp          # C++ implementation
│   ├── test_cubeeye_depth.cpp     # C++ validation test
│   ├── simple_v4l2_hook.c         # LD_PRELOAD hook for frame capture
│   ├── sdk_capture.cpp            # SDK-based capture (ground truth)
│   └── libusb_capture.c           # Direct USB capture (experimental)
│
├── depth_extractor.py             # Python reference implementation
├── analyze_hand_test.py           # Multi-frame validation script
├── verify_formula.py              # Formula verification
│
├── data/                          # Captured test data (not in git)
├── build/                         # Build output (not in git)
└── poly_coeffs.json               # Gradient correction coefficients
```

## API Reference

### C++ API

```cpp
#include "cubeeye_depth.h"

// Create extractor with gradient correction enabled
cubeeye::DepthExtractor extractor(true);

// Extract depth from raw V4L2 frame
uint8_t raw_frame[771200];  // From V4L2 capture
uint16_t depth[640 * 480];  // Output in mm

bool success = extractor.ExtractDepth(raw_frame, sizeof(raw_frame),
                                       depth, true /* interpolate */);

// Extract both depth and amplitude
uint16_t amplitude[640 * 480];
extractor.ExtractDepthAndAmplitude(raw_frame, sizeof(raw_frame),
                                    depth, amplitude, true);
```

### Python API

```python
from depth_extractor import DepthExtractor

extractor = DepthExtractor(apply_gradient_correction=True)

with open('raw_frame.bin', 'rb') as f:
    raw_data = f.read()

depth = extractor.extract_depth(raw_data, interpolate=True)
# depth is numpy array, shape (480, 640), dtype uint16, values in mm
```

## Validation Results

### Hand Waving Test (98 frames, 200-6400mm range)

```
Overall Statistics:
  RMSE:        mean=84.99mm, range=65.93-109.79mm
  Correlation: mean=0.9957, range=0.9933-0.9974

Accuracy by Depth Range (center pixel):
  200-500mm:   error = +0.5mm ± 0.9mm  (21 frames)
  500-1000mm:  error = -0.4mm ± 3.8mm  (5 frames)
  1000-2000mm: error = -9.3mm ± 7.3mm  (3 frames)
  3000-5000mm: error = -2.4mm ± 11.5mm (59 frames)
```

### Close-Range Accuracy (<600mm)

Critical for robot safety and obstacle detection:
- 23 frames analyzed
- Mean error: **+0.7mm**
- Standard deviation: **1.1mm**

## Verification Methodology

1. **Synchronized capture**: LD_PRELOAD hook captures raw V4L2 frames at the exact moment SDK processes them
2. **Pixel-by-pixel comparison**: Decoded depth vs SDK output
3. **Statistical validation**: Correlation, RMSE, per-depth-range analysis
4. **Multi-frame testing**: 98+ frames across full depth range

## Known Limitations

1. **RMSE includes edge artifacts**: The ~85mm RMSE is dominated by "flying pixels" at object boundaries. The SDK applies a FlyingPixelFilter we don't replicate.

2. **Temporal alignment**: ~1 frame offset possible between raw capture and SDK output due to buffering.

3. **Per-pixel calibration**: SDK may apply per-pixel corrections we don't have access to.

4. **Interpolation method**: We use nearest-neighbor; SDK may use bilinear.

## Future Work

- [ ] CUDA kernel implementation for GPU acceleration
- [ ] Flying pixel filter implementation
- [ ] ROS2 node integration
- [ ] Performance benchmarking vs SDK

## References

- [CubeEye I200D Product Page](https://www.meerecompany.com)
- SDK decompilation notes: `SDK_REVERSE_ENGINEERING_FINDINGS.md`
- Pipeline analysis: `SDK_PIPELINE_ANALYSIS.md`

## License

This reverse-engineering work is for educational and research purposes. The CubeEye SDK and hardware remain property of Meere Company.

# CubeEye I200D Driver Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        CubeEye I200D Sensor                      │
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ ToF Sensor   │→ │ On-chip DSP  │→ │ USB 3.0 Controller   │   │
│  │ (Infineon)   │  │ (CRT unwrap) │  │ (UVC Class)          │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ USB Bulk Transfer
                              │ 771,200 bytes/frame @ ~30fps
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                          Linux Host                              │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    UVC Kernel Driver                      │   │
│  │    /dev/video0  ←  V4L2 Interface  ←  USB Core           │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              │                                   │
│              ┌───────────────┴───────────────┐                  │
│              │                               │                   │
│              ▼                               ▼                   │
│  ┌───────────────────────┐     ┌───────────────────────────┐    │
│  │   CubeEye SDK         │     │   Custom Driver            │    │
│  │   (libCubeEye.so)     │     │   (This Project)           │    │
│  │                       │     │                             │    │
│  │  - V4L2 capture       │     │  - V4L2 capture            │    │
│  │  - 5-byte unpack      │     │  - 5-byte unpack           │    │
│  │  - Gradient correct   │     │  - Gradient correct        │    │
│  │  - Flying pixel filt  │     │  - (CUDA planned)          │    │
│  │  - 640×480 output     │     │  - 640×480 output          │    │
│  └───────────────────────┘     └───────────────────────────┘    │
│              │                               │                   │
│              ▼                               ▼                   │
│       uint16 depth[640×480]           uint16 depth[640×480]     │
│       (mm values)                     (mm values, r=0.997)      │
└─────────────────────────────────────────────────────────────────┘
```

## Data Flow

### 1. Sensor Output

The CubeEye I200D contains an Infineon IRS2976C ToF sensor that:
- Operates at dual modulation frequencies (100MHz, 80MHz)
- Performs on-chip CRT (Chinese Remainder Theorem) phase unwrapping
- Outputs pre-computed depth data (NOT raw phase)
- Maximum unambiguous range: ~7,500mm

### 2. USB Transfer

The sensor presents as a UVC (USB Video Class) device:
```
Bus 002 Device 004: ID 3674:0200 meere company Cube Eye Camera
  - Endpoint 0x83: Bulk IN, 1024 bytes max packet
  - Frame format: 1600×241 YUYV (actually ToF data)
  - Frame size: 771,200 bytes
```

### 3. Frame Structure

```
Offset      Size    Content
─────────────────────────────────────────
0x00000     3,200   Header row (metadata)
0x00C80     3,200   Data row 1: [amp 1600B][depth 1600B]
0x01900     3,200   Data row 2: [amp 1600B][depth 1600B]
...
0xBBD80     3,200   Data row 240: [amp 1600B][depth 1600B]
─────────────────────────────────────────
Total:      771,200 bytes
```

### 4. 5-Byte Packed Format

Each 5 bytes encode 2 depth values using coarse+fine components:

```
Byte layout:     [B0] [B1] [B2] [B3] [B4]

Pixel 0:
  coarse0 = ((B4 >> 2) & 0x03) | (B1 << 2)   // 10 bits
  fine0   = (B4 & 0x03) | (B0 << 2)          // 10 bits
  depth0  = (coarse0 << 10) + fine0          // 20 bits → mm

Pixel 1:
  coarse1 = (B4 >> 6) | (B3 << 2)            // 10 bits
  fine1   = ((B4 >> 4) & 0x03) | (B2 << 2)   // 10 bits
  depth1  = (coarse1 << 10) + fine1          // 20 bits → mm
```

### 5. Resolution Progression

```
Native sensor:    320 × 240 (76,800 pixels)
After unpacking:  640 × 240 (153,600 pixels) - horizontal 2×
After interp:     640 × 480 (307,200 pixels) - vertical 2×
```

## Gradient Correction

### Problem

Raw unpacked depth values have systematic errors that increase with distance:

```
Depth    Raw Error
───────────────────
500mm    +2mm
1000mm   +0mm
2000mm   +10mm
3000mm   +26mm
5000mm   +31mm
```

### Solution

A 13th-degree polynomial correction:

```c
// Coefficients (x in meters, correction in meters)
const double COEFFS[14] = {
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

// Apply correction
double x = depth_mm / 1000.0;
double correction = evaluate_polynomial(COEFFS, x);
corrected_depth = depth_mm + correction * 1000.0;
```

### Implementation

For efficiency, we pre-compute a lookup table (7,501 entries):

```c
int16_t gradient_lut[7501];  // correction in mm for each depth 0-7500

// At initialization:
for (int d = 0; d <= 7500; d++) {
    gradient_lut[d] = evaluate_correction(d);
}

// At runtime:
corrected = raw_depth + gradient_lut[raw_depth];
```

## Memory Layout

### Input Buffer (V4L2 mmap)

```
┌─────────────────────────────────────┐
│ Buffer 0: 771,200 bytes             │ ← mmap'd from kernel
├─────────────────────────────────────┤
│ Buffer 1: 771,200 bytes             │
├─────────────────────────────────────┤
│ Buffer 2: 771,200 bytes             │
├─────────────────────────────────────┤
│ Buffer 3: 771,200 bytes             │
└─────────────────────────────────────┘
```

### Output Buffer

```
┌─────────────────────────────────────┐
│ Depth: 640 × 480 × sizeof(uint16)   │ = 614,400 bytes
│        = 307,200 pixels             │
│        Values: 0-7500 (mm)          │
└─────────────────────────────────────┘
```

## CUDA Implementation

### Kernel Configuration

```
Grid:   240 blocks (one per raw data row)
Block:  320 threads (one per 5-byte group = 2 pixels)
Shared: 1,600 bytes (depth section of one row)
```

### Files

- `src/cubeeye_depth_cuda.h` - CUDA API header
- `src/cubeeye_depth_cuda.cu` - CUDA kernel implementation
- `src/test_cubeeye_depth_cuda.cpp` - Test/benchmark program

### Kernel Implementation

```cuda
__global__ void ExtractDepthKernel(
    const uint8_t* __restrict__ raw_frame,
    uint16_t* __restrict__ depth_out,
    const int16_t* __restrict__ gradient_lut,
    bool apply_correction,
    bool interpolate)
{
    const int row = blockIdx.x;
    const int group = threadIdx.x;  // 0-319

    // Shared memory for depth section of current row
    __shared__ uint8_t s_depth_data[1600];

    // Calculate offsets (skip header row, skip amplitude section)
    const int row_offset = (row + 1) * 3200;
    const int depth_offset = row_offset + 1600;

    // Cooperative load: each thread loads 5 bytes
    const int load_offset = group * 5;
    s_depth_data[load_offset + 0] = raw_frame[depth_offset + load_offset + 0];
    s_depth_data[load_offset + 1] = raw_frame[depth_offset + load_offset + 1];
    s_depth_data[load_offset + 2] = raw_frame[depth_offset + load_offset + 2];
    s_depth_data[load_offset + 3] = raw_frame[depth_offset + load_offset + 3];
    s_depth_data[load_offset + 4] = raw_frame[depth_offset + load_offset + 4];

    __syncthreads();

    // Unpack 5 bytes → 2 depths
    uint16_t d0, d1;
    Unpack5Bytes(&s_depth_data[group * 5], d0, d1);

    // Apply gradient correction from LUT
    if (apply_correction && gradient_lut != nullptr) {
        int idx0 = min((int)d0, 7500);
        int idx1 = min((int)d1, 7500);
        d0 = clamp((int)d0 + gradient_lut[idx0], 0, 7500);
        d1 = clamp((int)d1 + gradient_lut[idx1], 0, 7500);
    }

    // Write with optional 2× vertical duplication
    const int col = group * 2;
    if (interpolate) {
        const int out_row = row * 2;
        depth_out[out_row * 640 + col] = d0;
        depth_out[out_row * 640 + col + 1] = d1;
        depth_out[(out_row + 1) * 640 + col] = d0;
        depth_out[(out_row + 1) * 640 + col + 1] = d1;
    } else {
        depth_out[row * 640 + col] = d0;
        depth_out[row * 640 + col + 1] = d1;
    }
}
```

### Memory Management

```
Device Memory:
  - d_raw_frame_:      771,200 bytes (input)
  - d_depth_out_:      614,400 bytes (640×480×2)
  - d_amplitude_out_:  614,400 bytes (optional)
  - d_gradient_lut_:    15,002 bytes (7501 × int16)
  Total: ~2 MB

Pinned Host Memory (optional):
  - h_pinned_input_:   771,200 bytes
  - h_pinned_output_:  614,400 bytes
  Total: ~1.4 MB
```

### Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Kernel execution | <0.2 ms | Pure GPU compute |
| H2D transfer | ~0.1 ms | 771 KB @ ~8 GB/s |
| D2H transfer | ~0.08 ms | 614 KB @ ~8 GB/s |
| End-to-end | <0.5 ms | Including all transfers |
| Throughput | >60 fps | Sustainable rate |

### Usage

```cpp
#include "cubeeye_depth_cuda.h"

// Synchronous extraction
cubeeye::cuda::CudaDepthExtractor extractor(true, true);
extractor.ExtractDepth(raw_frame, 771200, depth_out, true);
std::cout << "Time: " << extractor.GetLastTotalTimeMs() << " ms\n";

// Asynchronous (for pipelining)
extractor.ExtractDepthAsync(raw_frame, 771200, depth_out, true);
// ... overlap with other work ...
extractor.Synchronize();
```

### Benchmark Command

```bash
./build/test_cubeeye_depth_cuda --benchmark data/raw_frame.bin 1000
```

## Verification Strategy

### Level 1: Pixel Comparison

```python
decoded = extract_depth(raw_frame)
sdk = load_sdk_output()
correlation = np.corrcoef(decoded.flat, sdk.flat)[0,1]
assert correlation > 0.99
```

### Level 2: Per-Depth-Range Validation

```python
for depth_range in [(200,500), (500,1000), (1000,2000), ...]:
    mask = (sdk >= depth_range[0]) & (sdk < depth_range[1])
    error = np.mean(decoded[mask] - sdk[mask])
    assert abs(error) < 15  # mm
```

### Level 3: Real-World Testing

- Hand waving at multiple distances
- Static scenes at known distances
- Moving objects (robot navigation scenarios)

## Error Analysis

### Sources of RMSE (~85mm)

1. **Edge artifacts** (~60mm contribution)
   - Flying pixels at object boundaries
   - SDK applies FlyingPixelFilter, we don't

2. **Temporal misalignment** (~30mm contribution)
   - 1-frame offset possible due to buffering
   - Motion between raw capture and SDK output

3. **Interpolation differences** (~10mm contribution)
   - We use nearest-neighbor
   - SDK may use bilinear

### Center Pixel Accuracy

For the center pixel (away from edges):
- Mean error: <1mm at close range
- Standard deviation: ~2mm
- This is the "true" algorithm accuracy

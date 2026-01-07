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

## CUDA Implementation Design (Planned)

### Kernel Configuration

```
Grid:   240 blocks (one per raw row)
Block:  320 threads (one per 5-byte group)
Shared: 3,200 bytes (one row of raw data)
```

### Kernel Pseudocode

```cuda
__global__ void ExtractDepthKernel(
    const uint8_t* __restrict__ raw_frame,
    uint16_t* __restrict__ depth_out,
    const int16_t* __restrict__ gradient_lut
) {
    int row = blockIdx.x;
    int group = threadIdx.x;  // 0-319, each handles 5 bytes → 2 pixels

    // Load row into shared memory (coalesced)
    __shared__ uint8_t row_data[3200];
    // ... cooperative load ...

    __syncthreads();

    // Unpack 5 bytes → 2 depths
    const uint8_t* bytes = row_data + 1600 + group * 5;
    uint16_t d0, d1;
    Unpack5Bytes(bytes, d0, d1);

    // Apply gradient correction
    d0 = clamp(d0 + gradient_lut[min(d0, 7500)], 0, 7500);
    d1 = clamp(d1 + gradient_lut[min(d1, 7500)], 0, 7500);

    // Write with 2× vertical duplication
    int col = group * 2;
    int out_row = row * 2;
    depth_out[out_row * 640 + col] = d0;
    depth_out[out_row * 640 + col + 1] = d1;
    depth_out[(out_row + 1) * 640 + col] = d0;
    depth_out[(out_row + 1) * 640 + col + 1] = d1;
}
```

### Expected Performance

- Input: 771,200 bytes
- Output: 307,200 × 2 bytes = 614,400 bytes
- Compute: 240 × 320 × 2 = 153,600 depth calculations
- Memory bandwidth limited: ~0.1ms theoretical on Jetson Orin

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

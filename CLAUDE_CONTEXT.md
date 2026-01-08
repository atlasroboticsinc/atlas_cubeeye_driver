# Claude Context - CubeEye I200D Driver

This file preserves context for Claude Code sessions working on this project.

## Project Overview

**Repository:** `atlasroboticsinc/atlas_cubeeye_driver`

SDK-free driver for CubeEye I200D ToF (Time-of-Flight) depth sensor. Reverse-engineered from proprietary SDK to run on Jetson Orin and x86 with CUDA acceleration.

## Key Technical Details

### Sensor Specifications
- **Raw frame:** 1600x241 pixels, YUYV format, 16-bit
- **Output:** 640x480 depth + 640x480 amplitude, GRAY16_LE
- **Frame rate:** 15 FPS
- **Depth range:** 0-10000mm (typical max ~3-5m usable)
- **Interface:** V4L2 + UVC Extension Unit for initialization

### Depth Extraction Algorithm
Raw frame contains interleaved depth data packed in 5-byte groups:
```
Byte layout: [A0_L] [A0_H | D0_L] [D0_H] [A1_L] [A1_H | D1_L] [D1_H] ...
- Amplitude: 12-bit (lower 4 bits of byte 1 + byte 0)
- Depth: 12-bit (byte 2 + upper 4 bits of byte 1)
```

### Gradient Correction Polynomial
Applied to correct depth distortion across the image:
```cpp
correction = 1.0 + c1*r + c2*r² + c3*r³ + c4*r⁴
// where r = distance from image center (normalized)
// Coefficients extracted from SDK calibration
```

### Camera Intrinsics (from calibration page 0x0005)
```cpp
fx = 393.25, fy = 393.42, cx = 321.48, cy = 239.92
k1 = -0.270483, k2 = 0.106138, p1 = -0.023670
```

## GStreamer Element: cubeeyesrc

**Single combined element** that does V4L2 capture AND depth/amplitude extraction.

### Architecture Decision
User specifically requested a monolithic element instead of separate `cubeeye_src` + `cubeeye_depth` elements because the capture and extraction are tightly coupled.

### Multi-pad Output
- **depth pad** (always present): 640x480 GRAY16_LE depth in mm
- **amplitude pad** (optional): 640x480 GRAY16_LE amplitude, enabled via `enable-amplitude=true`

### Properties
| Property | Type | Default | Description |
|----------|------|---------|-------------|
| serial | string | NULL | Camera serial (e.g., I200DU2509000349) |
| device | string | NULL | V4L2 path (auto-detected from serial) |
| enable-amplitude | bool | false | Enable amplitude output pad |
| gradient-correction | bool | true | Apply polynomial gradient correction |
| normalize | bool | false | Scale depth for display (0-max_depth -> 0-65535) |
| max-depth | int | 5000 | Maximum depth in mm for normalization |

### Key Implementation Details

1. **Element type:** GstElement (NOT GstPushSrc/GstBaseSrc) because multi-pad sources need manual thread management

2. **Streaming thread:** `gst_cubeeyesrc_thread_func` handles:
   - V4L2 buffer dequeue
   - Depth/amplitude extraction (CUDA or CPU)
   - Normalization if enabled
   - Buffer push to pads

3. **Required GStreamer events** (before first buffer):
   - stream-start event with unique stream ID
   - caps event with format negotiation
   - segment event for timing

4. **CUDA acceleration:** Enabled when `HAVE_CUDA=1` is defined and CUDA is available at runtime

### Source Files
```
gst-plugins/gstcubeeye/
├── CMakeLists.txt      # Build config, links cubeeye_device + cubeeye_depth
├── plugin.cpp          # Plugin registration (only cubeeyesrc)
├── gstcubeeyesrc.h     # Element header with struct definitions
└── gstcubeeyesrc.cpp   # Element implementation
```

## Build Instructions

**Must build inside devcontainer** (atlas_devcontainer):

```bash
cd /workspaces/atlas_levo/src/3rdparty/drivers/atlas_cubeeye_driver
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## Testing Commands

```bash
# Set plugin path
export GST_PLUGIN_PATH=/path/to/build

# Inspect element
gst-inspect-1.0 cubeeyesrc

# Basic depth display (with normalization for visibility)
gst-launch-1.0 cubeeyesrc serial=I200DU2509000349 normalize=true \
  ! queue ! videoconvert ! autovideosink sync=false

# Depth + amplitude (dual output)
gst-launch-1.0 cubeeyesrc serial=I200DU2509000349 enable-amplitude=true normalize=true name=src \
  src.depth ! queue ! videoconvert ! autovideosink sync=false \
  src.amplitude ! queue ! fakesink
```

## Common Issues & Fixes

### Issue: Black display with valid depth data
**Cause:** GRAY16_LE values 0-7500 appear nearly black in 0-65535 range
**Fix:** Use `normalize=true` property to scale depth to full 16-bit range

### Issue: "Got data flow before stream-start event" warnings
**Cause:** Missing GStreamer events before buffer push
**Fix:** Added `gst_cubeeyesrc_send_stream_events()` to send stream-start, caps, segment events before first buffer (commit 733c2e1)

### Issue: Struct name collision during build
**Cause:** Multiple header files defining `_GstCubeEyeSrc`
**Fix:** Removed legacy `gstcubeeye_src.*` and `gstcubeeye_depth.*` files, kept only `gstcubeeyesrc.*`

### Issue: Max depth values ~7400mm seem wrong
**Analysis:** Only ~40 pixels (0.01%) exceed 7000mm - these are noise at image edges. 95th percentile matches expected scene depth (~3.2m).

## Project Status

- [x] Phase 1: Device enumeration library
- [x] Phase 2: GStreamer source element
- [x] Phase 3: CUDA depth transform element
- [x] Phase 4: Plugin registration & build
- [x] Phase 5: Multi-camera testing (validated - one sensor has hardware issue)
- [x] Phase 6: Atlas integration (DS3D dataloader)

## Atlas DS3D Integration

CubeEye dataloader is integrated into `atlas_ds3d_components` for DeepStream 3D pipeline:

**Location:** `atlas_levo/src/perception/atlas_ds3d_components/`

### DS3D Dataloader Files
```
include/atlas_ds3d_components/dataloaders/
├── cubeeye_device.hpp      # Device enumeration, UVC control, calibration readout
├── cubeeye_kernels.hpp     # CUDA kernel declarations
└── cubeeye_loader.hpp      # DS3D DataLoader class

src/dataloaders/
├── cubeeye_device.cpp      # Device enumeration implementation
├── cubeeye_kernels.cu      # CUDA depth extraction + point cloud generation
└── cubeeye_loader.cpp      # DS3D DataLoader implementation

src/parameters/
└── cubeeye_loader.yaml     # Configuration schema
```

### DS3D Features
- **Automatic intrinsics readout** from sensor calibration page 0x0005
- **Fused CUDA kernel** for depth extraction + point cloud generation
- **Organized point cloud** output (640x480 with invalid points = z=0)
- **Multi-camera support** via serial number selection

### DS3D Output Keys
| Key | Type | Description |
|-----|------|-------------|
| `DS3D::LidarXYZI` | float4* (GPU) | Organized point cloud (640x480) |
| `DS3D::CubeEyeDepth` | uint16_t* (GPU) | Depth image in mm |
| `DS3D::CubeEyeAmplitude` | uint16_t* (GPU) | Amplitude image |

### DS3D Usage Example
```yaml
source_definitions:
  cubeeye_front:
    type: dataloader
    library: libatlas_ds3d_dataloader_cubeeye.so
    create_function: createCubeEyeLoader
    config:
      serial: "I200DU2509000349"
      frame_id: "cubeeye_front_optical"
      pointcloud_key: "DS3D::CubeEyeFront"
      max_depth_mm: 5000
      min_amplitude: 50
```

## Related Files

- `src/cubeeye_device.h/cpp` - Device enumeration, UVC control
- `src/cubeeye_depth.h/cpp` - CPU depth extraction
- `src/cubeeye_depth_cuda.h/cu` - CUDA depth extraction
- `SDK_REVERSE_ENGINEERING_FINDINGS.md` - Detailed protocol analysis

## Environment

- **Dev machine:** RTX 5080 with CUDA
- **Target:** Jetson Orin (atlas_levo)
- **Build system:** CMake + devcontainer
- **Container:** atlas_devcontainer

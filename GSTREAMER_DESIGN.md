# CubeEye I200D GStreamer Element Design

## Executive Summary

GPU-native GStreamer elements for CubeEye I200D ToF cameras, designed for direct integration with Atlas perception pipeline. Bypasses the SDK entirely, using our reverse-engineered depth extraction with CUDA acceleration.

**Target**: 3 cameras per robot at 15 FPS each, GPU-accelerated depth extraction, direct NVMM output for DeepStream/perception pipeline integration.

---

## 1. Architecture Overview

### 1.1 Element Design

Two GStreamer elements working together:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        GStreamer Pipeline                           │
│                                                                     │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐  │
│  │ cubeeye_src  │───▶│ cubeeye_depth    │───▶│ nvstreammux /    │  │
│  │              │    │                  │    │ perception       │  │
│  │ V4L2 + UVC   │    │ CUDA extraction  │    │ pipeline         │  │
│  │ Raw frames   │    │ Depth + Amp out  │    │                  │  │
│  └──────────────┘    └──────────────────┘    └──────────────────┘  │
│                                                                     │
│  Properties:          Properties:             Standard Atlas       │
│  - serial=ABC123      - output=depth|amp|both  pipeline elements   │
│  - device=/dev/videoX - undistort=true                             │
│  - framerate=15       - gradient-correction=true                   │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 Why Two Elements (Not One Bin)?

| Approach | Pros | Cons |
|----------|------|------|
| **Two separate elements** | Flexible pipeline composition, can insert debug/record elements between | Slightly more complex pipeline strings |
| **Single bin element** | Simple pipeline string | Less flexible, harder to debug |

**Decision**: Two separate elements for flexibility. Can create a convenience bin later if needed.

### 1.3 Data Flow

```
Camera Sensor
     │
     ▼ (USB 3.0, ~12 MB/s per camera)
┌─────────────────┐
│  V4L2 Device    │  /dev/videoX
│  771,200 bytes  │  per frame (241 rows × 3200 bytes)
└────────┬────────┘
         │
         ▼ (System memory)
┌─────────────────┐
│  cubeeye_src    │  GStreamer source element
│                 │  - UVC XU initialization
│                 │  - Serial number matching
│                 │  - V4L2 MMAP capture
│                 │  - Output: video/x-raw, format=GRAY8, 3200x241
└────────┬────────┘
         │
         ▼ (DMA to GPU / cudaMemcpyAsync)
┌─────────────────┐
│  cubeeye_depth  │  GStreamer transform element
│                 │  - CUDA 5-byte unpacking kernel
│                 │  - Gradient correction LUT
│                 │  - Optional undistortion
│                 │  - Output: video/x-raw(memory:NVMM), format=GRAY16_LE, 640x480
└────────┬────────┘
         │
         ▼ (NVMM / GPU memory)
┌─────────────────┐
│  Perception     │  nvstreammux, inference, etc.
│  Pipeline       │
└─────────────────┘
```

---

## 2. Element Specifications

### 2.1 `gstcubeeye_src` - Source Element

**Type**: `GstPushSrc` (extends `GstBaseSrc`)

**Capabilities**:
```
SRC pad:
  video/x-raw,
  format = GRAY8,
  width = 3200,
  height = 241,
  framerate = 15/1
```

**Properties**:

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `serial` | string | "" | Camera serial number (e.g., "I200DU2427000032"). If empty, uses first available. |
| `device` | string | "" | V4L2 device path (e.g., "/dev/video0"). Auto-detected if serial is set. |
| `device-index` | int | -1 | V4L2 device index. -1 = auto-detect from serial. |
| `framerate` | fraction | 15/1 | Capture frame rate (sensor supports up to 15 FPS) |
| `integration-time` | int | 1000 | Sensor integration time (100-2000 μs) |
| `auto-exposure` | bool | true | Enable auto exposure |

**Initialization Sequence**:
1. If `serial` set: enumerate V4L2 devices, query UVC XU Selector 1 for each, match serial
2. If `device` set: use directly
3. Open V4L2 device
4. Send UVC XU initialization commands (mode, integration time, etc.)
5. Setup MMAP buffers (4 buffers recommended)
6. Start streaming

**UVC XU Commands** (from our reverse engineering):
```c
// Selector 1: Device info (read serial, firmware version)
// Selector 2: Sensor control (mode, integration time, etc.)

struct uvc_xu_control_query xu_query = {
    .unit = 4,           // Extension unit ID
    .selector = 2,       // Control selector
    .query = UVC_SET_CUR,
    .size = 256,
    .data = control_data
};
```

### 2.2 `gstcubeeye_depth` - Transform Element

**Type**: `GstElement` (multi-pad element, not GstBaseTransform due to multiple src pads)

**Capabilities**:
```
SINK pad (always):
  video/x-raw,
  format = GRAY8,
  width = 3200,
  height = 241

SRC pad "depth" (can be disabled):
  video/x-raw(memory:NVMM),   # GPU memory
  format = GRAY16_LE,
  width = [320, 1280],        # Configurable
  height = [240, 960],        # Configurable
  framerate = [1/1, 30/1]     # Configurable

SRC pad "amplitude" (can be disabled):
  video/x-raw(memory:NVMM),
  format = GRAY16_LE,
  width = [320, 1280],
  height = [240, 960],
  framerate = [1/1, 30/1]
```

**Properties**:

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `enable-depth` | bool | true | Enable depth output pad |
| `enable-amplitude` | bool | false | Enable amplitude output pad |
| `output-width` | int | 640 | Output frame width (320-1280) |
| `output-height` | int | 480 | Output frame height (240-960) |
| `framerate` | fraction | 15/1 | Output frame rate |
| `gradient-correction` | bool | true | Apply polynomial gradient correction |
| `undistort` | bool | false | Apply lens undistortion |
| `max-depth-mm` | int | 7500 | Maximum depth value (for LUT) |

**Multi-Pad Design**:
- Two source pads: `depth` and `amplitude`
- Each pad can be independently enabled/disabled via properties
- Pads are created dynamically based on enable properties
- When both enabled, single CUDA kernel extracts both in one pass

**CUDA Processing Pipeline**:
```
1. cudaMemcpyAsync(device_input, host_frame, 771200, H2D, stream)
2. extract_depth_kernel<<<grid, block, 0, stream>>>(...)
3. If gradient_correction: apply_gradient_lut_kernel<<<...>>>
4. If undistort: cv::cuda::remap() or custom kernel
5. Output to NvBufSurface (NVMM)
```

**Kernel Design** (from existing `cubeeye_depth_cuda.cu`):
```cuda
__global__ void extract_depth_amplitude_kernel(
    const uint8_t* __restrict__ raw_frame,  // 771,200 bytes
    uint16_t* __restrict__ depth_out,       // 640x480
    uint16_t* __restrict__ amplitude_out,   // 640x480 (optional)
    const int16_t* __restrict__ gradient_lut, // 7501 entries
    bool apply_gradient,
    int max_depth
) {
    // Thread per output pixel pair (320 threads per row, 240 rows)
    // Each thread unpacks 2 depth + 2 amplitude values from 10 bytes

    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col_pair = blockIdx.x * blockDim.x + threadIdx.x;

    if (row >= 240 || col_pair >= 320) return;

    // Skip header row, offset to depth section
    int raw_offset = 3200 + row * 3200 + 1600 + col_pair * 5;

    // 5-byte unpack (vectorized load)
    uint8_t b0 = raw_frame[raw_offset + 0];
    uint8_t b1 = raw_frame[raw_offset + 1];
    uint8_t b2 = raw_frame[raw_offset + 2];
    uint8_t b3 = raw_frame[raw_offset + 3];
    uint8_t b4 = raw_frame[raw_offset + 4];

    // Pixel 0
    uint32_t coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2);
    uint32_t fine0 = (b4 & 0x03) | (b0 << 2);
    uint16_t depth0 = (coarse0 << 10) + fine0;

    // Pixel 1
    uint32_t coarse1 = (b4 >> 6) | (b3 << 2);
    uint32_t fine1 = ((b4 >> 4) & 0x03) | (b2 << 2);
    uint16_t depth1 = (coarse1 << 10) + fine1;

    // Apply gradient correction
    if (apply_gradient) {
        depth0 = clamp(depth0 + gradient_lut[min(depth0, max_depth)], 0, max_depth);
        depth1 = clamp(depth1 + gradient_lut[min(depth1, max_depth)], 0, max_depth);
    }

    // Write to output (row doubling for 480 height)
    int out_col = col_pair * 2;
    depth_out[row * 2 * 640 + out_col] = depth0;
    depth_out[row * 2 * 640 + out_col + 1] = depth1;
    depth_out[(row * 2 + 1) * 640 + out_col] = depth0;
    depth_out[(row * 2 + 1) * 640 + out_col + 1] = depth1;
}
```

---

## 3. Device Enumeration & Serial Matching

### 3.1 Device Enumeration Library

Shared code used by both GStreamer element and standalone tools.

**File**: `src/cubeeye_device.h / .cpp`

```cpp
namespace cubeeye {

struct DeviceInfo {
    std::string device_path;    // "/dev/video0"
    int device_index;           // 0
    std::string serial_number;  // "I200DU2427000032"
    std::string firmware_ver;   // "1.2.3"
    std::string model;          // "I200D"
    bool is_cubeeye;            // true if UVC XU responds correctly
};

class DeviceEnumerator {
public:
    // Enumerate all CubeEye devices on system
    static std::vector<DeviceInfo> enumerate();

    // Find device by serial number
    static std::optional<DeviceInfo> find_by_serial(const std::string& serial);

    // Find device by path
    static std::optional<DeviceInfo> find_by_path(const std::string& path);

private:
    // Query UVC XU Selector 1 for device info
    static bool query_device_info(int fd, DeviceInfo& info);
};

} // namespace cubeeye
```

### 3.2 Enumeration Algorithm

```cpp
std::vector<DeviceInfo> DeviceEnumerator::enumerate() {
    std::vector<DeviceInfo> devices;

    // Scan /dev/video* devices
    for (int i = 0; i < 20; i++) {
        std::string path = "/dev/video" + std::to_string(i);

        int fd = open(path.c_str(), O_RDWR);
        if (fd < 0) continue;

        // Check if it's a CubeEye by querying UVC XU
        DeviceInfo info;
        info.device_path = path;
        info.device_index = i;

        if (query_device_info(fd, info)) {
            info.is_cubeeye = true;
            devices.push_back(info);
        }

        close(fd);
    }

    return devices;
}
```

---

## 4. Multi-Camera Pipeline Configuration

### 4.1 Pipeline YAML Configuration

Following Atlas pattern from `levo.yaml`:

**File**: `levo_bringup/config/perception/pipeline/levo.yaml` (additions)

```yaml
source_definitions:
  # Existing RGB cameras...

  # CubeEye ToF Depth Cameras
  depth_shoulder_left:
    pipeline: "cubeeye_src serial=I200DU2427000032 ! cubeeye_depth output-type=depth gradient-correction=true"
    config_file: $(find-pkg-share levo_bringup)/config/perception/cameras/depth_shoulder_left.yaml

  depth_shoulder_right:
    pipeline: "cubeeye_src serial=I200DU2427000090 ! cubeeye_depth output-type=depth"
    config_file: $(find-pkg-share levo_bringup)/config/perception/cameras/depth_shoulder_right.yaml

  depth_chest:
    pipeline: "cubeeye_src serial=I200DU2427000041 ! cubeeye_depth output-type=depth"
    config_file: $(find-pkg-share levo_bringup)/config/perception/cameras/depth_chest.yaml
```

### 4.2 Camera Configuration Files

**File**: `levo_bringup/config/perception/cameras/depth_shoulder_left.yaml`

```yaml
camera:
  name: depth_shoulder_left
  type: tof_depth
  model: cubeeye_i200d

  # Intrinsics (from calibration)
  intrinsics:
    fx: 393.25
    fy: 393.42
    cx: 321.48
    cy: 239.92

  # Distortion coefficients
  distortion:
    model: brown_conrady
    k1: -0.270483
    k2: 0.106138
    p1: -0.023670
    p2: 0.0
    k3: 0.0

  # Sensor parameters
  sensor:
    serial: I200DU2427000032
    integration_time: 1000
    auto_exposure: true
    max_depth_mm: 7500

  # TF frame
  frame_id: depth_shoulder_left_optical_frame
```

### 4.3 Multi-Camera Batching (Optional)

If depth cameras need to feed into nvstreammux for batched processing:

```yaml
depth_mux:
  type: nvstreammux
  batch_size: 3
  width: 640
  height: 480
  sources:
    - depth_shoulder_left
    - depth_shoulder_right
    - depth_chest
```

---

## 5. Integration with Atlas Perception Pipeline

### 5.1 Source Setup Integration

The Atlas perception pipeline (`pipeline_source_setup.cpp`) auto-detects element types and injects conversion chains. We need to register our elements:

**Addition to** `pipeline_source_setup.cpp`:

```cpp
// CubeEye depth sources (already produce NVMM)
const std::set<std::string> kCubeEyeSources = {
    "cubeeye_src",
};

// In setupSourceBin():
if (kCubeEyeSources.contains(first_element)) {
    // cubeeye_src ! cubeeye_depth already outputs NVMM
    // No additional conversion needed
    final_pipeline = pipeline_string;
}
```

### 5.2 Output Format Compatibility

| Consumer | Expected Format | cubeeye_depth Output | Compatible? |
|----------|-----------------|---------------------|-------------|
| nvstreammux | video/x-raw(memory:NVMM) | video/x-raw(memory:NVMM), GRAY16_LE | Yes |
| DS3D depth | NvBufSurface, UINT16 | NvBufSurface via NVMM | Yes |
| ROS2 bridge | sensor_msgs/Image | Need appsink + publisher | Indirect |

### 5.3 Metadata Attachment

Attach camera intrinsics and frame info as GStreamer metadata:

```cpp
typedef struct _GstCubeEyeDepthMeta {
    GstMeta meta;

    // Camera identification
    gchar serial_number[32];
    gchar frame_id[64];

    // Intrinsics
    gdouble fx, fy, cx, cy;
    gdouble k1, k2, p1, p2, k3;

    // Frame info
    guint32 width, height;
    guint32 max_depth_mm;
    gboolean is_undistorted;

} GstCubeEyeDepthMeta;
```

---

## 6. File Structure

```
atlas_cubeeye_driver/               # This repository
├── gst-plugins/
│   └── gstcubeeye/
│       ├── CMakeLists.txt
│       ├── plugin.cpp              # Plugin registration
│       ├── gstcubeeye_src.h
│       ├── gstcubeeye_src.cpp      # V4L2 + UVC source element
│       ├── gstcubeeye_depth.h
│       ├── gstcubeeye_depth.cpp    # CUDA depth transform element
│       ├── gstcubeeye_depth.cu     # CUDA kernels
│       └── gstcubeeye_meta.h       # Custom metadata
│
├── src/
│   ├── cubeeye_device.h            # Device enumeration (shared)
│   ├── cubeeye_device.cpp
│   ├── cubeeye_depth.h             # Depth extraction (shared)
│   ├── cubeeye_depth.cpp
│   ├── cubeeye_depth_cuda.h
│   └── cubeeye_depth_cuda.cu       # Existing CUDA kernel
│
├── tools/
│   ├── cubeeye_list_devices.cpp    # CLI tool to list cameras
│   └── cubeeye_gst_test.cpp        # GStreamer pipeline test
│
└── config/
    └── cameras/                    # Camera config templates
        └── depth_camera.yaml.template
```

---

## 7. Implementation Plan

### Phase 1: Device Enumeration Library (1-2 hours)

**Deliverables**:
- `src/cubeeye_device.h` - Device enumeration interface
- `src/cubeeye_device.cpp` - Implementation
- `tools/cubeeye_list_devices.cpp` - CLI tool

**Tasks**:
1. Extract device enumeration code from `cubeeye_standalone_capture.cpp`
2. Create clean C++ API with serial number lookup
3. Add firmware version and model detection
4. Create list_devices CLI tool
5. Test with multiple cameras (if available) or single camera

**Verification**:
```bash
$ ./build/cubeeye_list_devices
Found 3 CubeEye devices:
  [0] /dev/video0  Serial: I200DU2427000032  Model: I200D  FW: 1.2.3
  [1] /dev/video2  Serial: I200DU2427000090  Model: I200D  FW: 1.2.3
  [2] /dev/video4  Serial: I200DU2427000041  Model: I200D  FW: 1.2.3
```

### Phase 2: GStreamer Source Element (2-3 hours)

**Deliverables**:
- `gst-plugins/gstcubeeye/gstcubeeye_src.cpp`
- `gst-plugins/gstcubeeye/gstcubeeye_src.h`

**Tasks**:
1. Create GstPushSrc subclass
2. Implement `serial` property with device lookup
3. Implement V4L2 MMAP capture in `create()` method
4. Add UVC XU initialization on `start()`
5. Implement `stop()` for clean shutdown
6. Register element with GStreamer

**Verification**:
```bash
# Basic capture test
$ gst-launch-1.0 cubeeye_src serial=I200DU2427000032 ! fakesink

# Save raw frames
$ gst-launch-1.0 cubeeye_src ! filesink location=raw_frames.bin
```

### Phase 3: CUDA Depth Transform Element (2-3 hours)

**Deliverables**:
- `gst-plugins/gstcubeeye/gstcubeeye_depth.cpp`
- `gst-plugins/gstcubeeye/gstcubeeye_depth.cu`
- `gst-plugins/gstcubeeye/gstcubeeye_depth.h`

**Tasks**:
1. Create GstBaseTransform subclass
2. Implement caps negotiation (GRAY8 3200x241 → GRAY16_LE 640x480)
3. Port existing CUDA kernel
4. Add NVMM output support (NvBufSurface allocation)
5. Implement gradient correction toggle
6. Add undistortion support (optional, can defer)

**Verification**:
```bash
# CPU visualization test (without NVMM)
$ gst-launch-1.0 cubeeye_src ! cubeeye_depth ! videoconvert ! autovideosink

# NVMM output test
$ gst-launch-1.0 cubeeye_src ! cubeeye_depth ! nvvideoconvert ! nveglglessink
```

### Phase 4: Plugin Registration & Build (1 hour)

**Deliverables**:
- `gst-plugins/gstcubeeye/CMakeLists.txt`
- `gst-plugins/gstcubeeye/plugin.cpp`
- Integration with atlas_levo build system

**Tasks**:
1. Create CMakeLists.txt with CUDA and GStreamer dependencies
2. Implement plugin_init() registration
3. Setup ament environment hooks for plugin discovery
4. Test plugin loading

**Verification**:
```bash
$ gst-inspect-1.0 cubeeye_src
$ gst-inspect-1.0 cubeeye_depth
```

### Phase 5: Multi-Camera Testing (1-2 hours)

**Deliverables**:
- USB bandwidth measurements
- Multi-camera pipeline test
- Documentation

**Tasks**:
1. Test 3-camera simultaneous capture
2. Measure USB bandwidth (expect ~36 MB/s for 3 cameras)
3. Verify no frame drops at 15 FPS × 3
4. Document any USB hub requirements

**Verification**:
```bash
# 3-camera pipeline
$ gst-launch-1.0 \
    cubeeye_src serial=I200DU2427000032 name=cam0 ! cubeeye_depth ! queue ! mux.sink_0 \
    cubeeye_src serial=I200DU2427000090 name=cam1 ! cubeeye_depth ! queue ! mux.sink_1 \
    cubeeye_src serial=I200DU2427000041 name=cam2 ! cubeeye_depth ! queue ! mux.sink_2 \
    nvstreammux name=mux batch-size=3 width=640 height=480 ! fakesink
```

### Phase 6: Atlas Integration (1-2 hours)

**Deliverables**:
- Pipeline YAML configuration
- Camera config files
- Integration test with perception pipeline

**Tasks**:
1. Add source definitions to levo.yaml
2. Create camera config files with intrinsics
3. Test with atlas_perception_pipeline
4. Verify depth data flows to consumers

---

## 8. Performance Targets

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Frame rate | 15 FPS per camera | gst-launch + fpsdisplaysink |
| Depth extraction latency | <1ms | CUDA events |
| Memory bandwidth (3 cams) | <40 MB/s USB | iotop / USB analyzer |
| GPU memory per camera | <10 MB | nvidia-smi |
| Total pipeline latency | <50ms | Timestamp delta |

---

## 9. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| USB bandwidth saturation (3 cams) | Medium | High | Use USB 3.0 hub, test early |
| NVMM allocation complexity | Medium | Medium | Reference atlasrosimagesrc implementation |
| UVC XU init failures | Low | High | Robust retry logic, error reporting |
| GStreamer version incompatibility | Low | Medium | Test on target Jetson image |

---

## 10. Dependencies

**Build Dependencies**:
- GStreamer 1.0 development files (`libgstreamer1.0-dev`)
- GStreamer base plugins (`libgstreamer-plugins-base1.0-dev`)
- CUDA Toolkit 11.4+ (for Jetson)
- OpenCV 4.x (for undistortion, optional)
- V4L2 headers (`v4l-utils`)

**Runtime Dependencies**:
- GStreamer 1.0 runtime
- NVIDIA DeepStream (for NVMM support)
- CUDA runtime

---

## 11. Design Decisions (Confirmed)

1. **Amplitude output**: **Separate pad with enable/disable**
   - Two src pads: `depth` and `amplitude`
   - Properties `enable-depth` and `enable-amplitude` control which pads are active
   - Single CUDA kernel extracts both when both enabled

2. **ROS2 bridge**: **Deferred, optional later**
   - First: Pure GStreamer implementation
   - Later: Optional ROS2 exposure following atlas_gstreamer_elements pattern (appsink → ROS publisher)

3. **Point cloud generation**: **Deferred to Phase 2**
   - Focus on depth/amplitude first
   - `cubeeye_pointcloud` element can be added later

4. **Resolution & framerate**: **Configurable**
   - `output-width`, `output-height` properties on depth element
   - `framerate` property on both source and depth elements
   - Supports scaling from native 640x480 to other resolutions

---

## 12. Acceptance Criteria

**Phase 1-4 Complete When**:
- [ ] `gst-inspect-1.0 cubeeye_src` shows element details
- [ ] `gst-inspect-1.0 cubeeye_depth` shows element details
- [ ] Single camera pipeline runs at 15 FPS
- [ ] Depth values match our Python extractor (within 1mm)
- [ ] Serial number selection works

**Phase 5-6 Complete When**:
- [ ] 3-camera pipeline runs without frame drops
- [ ] USB bandwidth documented
- [ ] Pipeline config added to levo.yaml
- [ ] Integration test passes with atlas_perception_pipeline

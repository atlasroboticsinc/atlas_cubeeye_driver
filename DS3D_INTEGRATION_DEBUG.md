# CubeEye DS3D Integration - Debug Notes

**Created:** 2025-01-08
**Purpose:** Context preservation for debugging compile errors after context compaction

## Current State

**STATUS: READY TO BUILD - EXPECTING COMPILE ERRORS**

We just created a CubeEye DS3D dataloader in `atlas_levo`. The code has been committed but NOT yet compiled. User is about to build and we expect compile errors that need debugging.

## What Was Built

### Location
`/home/cmericli/development/atlas/code/atlas_levo/src/perception/atlas_ds3d_components/`

### New Files (8 files, ~2079 lines)

| File | Purpose |
|------|---------|
| `include/.../dataloaders/cubeeye_device.hpp` | Device enumeration, UVC control, LensIntrinsics struct |
| `include/.../dataloaders/cubeeye_kernels.hpp` | CUDA kernel declarations, CubeEyeConfig/CubeEyeIntrinsics structs |
| `include/.../dataloaders/cubeeye_loader.hpp` | DS3D DataLoader class declaration |
| `src/dataloaders/cubeeye_device.cpp` | Device enumeration, UVC queries, calibration page readout |
| `src/dataloaders/cubeeye_kernels.cu` | CUDA kernels for depth extraction + point cloud generation |
| `src/dataloaders/cubeeye_loader.cpp` | Main loader implementation with V4L2 capture |
| `src/parameters/cubeeye_loader.yaml` | Parameter schema for generate_parameter_library |

### CMakeLists.txt Changes

Added after `atlas_ds3d_dataloader_econ_tof` section (around line 342):

```cmake
# CubeEye Kernels CUDA library
add_library(cubeeye_kernels SHARED src/dataloaders/cubeeye_kernels.cu)
target_include_directories(cubeeye_kernels PUBLIC ...)
target_link_libraries(cubeeye_kernels CUDA::cudart)

# CubeEyeLoader
generate_parameter_library(cubeeye_loader_parameters src/parameters/cubeeye_loader.yaml NO_ROS)

add_library(atlas_ds3d_dataloader_cubeeye SHARED
    src/dataloaders/cubeeye_loader.cpp
    src/dataloaders/cubeeye_device.cpp
)
target_link_libraries(atlas_ds3d_dataloader_cubeeye
    ds3d_helpers
    cubeeye_kernels
    cubeeye_loader_parameters
    yaml-cpp
    PkgConfig::gstreamer
    PkgConfig::gstreamer-app
    CUDA::cudart
    ${NVDS_3D_COMMON_LIB}
    ${NVDS_3D_GST_LIB}
)
```

Also added to install sections.

## Key Dependencies

### Headers We Include
```cpp
// DS3D framework
#include <ds3d/common/common.h>
#include <ds3d/common/impl/impl_dataloader.h>
#include <ds3d/gst/nvds3d_gst_plugin.h>

// Atlas helpers
#include "atlas_ds3d_components/common/ds3d_helpers.hpp"
#include "atlas_ds3d_components/common/organized_cloud_config.hpp"
#include "atlas_ds3d_components/wrappers/datamap_fixed.hpp"

// System
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <cuda_runtime.h>
#include <gst/gst.h>
#include <yaml-cpp/yaml.h>
```

### Functions We Use From ds3d_helpers.hpp
- `storeRosHeaderInDataMap()`
- `storeOrganizedCloudConfigInDataMap()`
- `RosHeader` struct

**IMPORTANT:** Check if these functions exist in ds3d_helpers.hpp. They may have different names or signatures.

## Likely Compile Errors

### 1. Missing ds3d_helpers functions
The loader uses:
```cpp
storeRosHeaderInDataMap(header, new_datamap);
storeOrganizedCloudConfigInDataMap(sensor_config, new_datamap);
```

These may not exist. Check `ds3d_helpers.hpp` for actual function names.

### 2. GuardDataMap API differences
We use:
```cpp
new_datamap.setGuardData(key, frame);
```

The actual API might be different. Check existing loaders like `econ_tof_loader.cpp` for correct usage.

### 3. Frame2DGuard usage
We use:
```cpp
::ds3d::Frame2DGuard xyzi_frame;
xyzi_frame.setDataFrame(...);
```

This might not be the correct type or method. Check existing code.

### 4. RosHeader struct
We use:
```cpp
RosHeader header;
header.frame_id = frame_id_;
header.stamp_ns = ...;
```

Check if RosHeader has these exact field names.

### 5. OrganizedCloudConfig
We use fields like `rows`, `cols`, `sensor_hz`, `sensor_brand`, `sensor_model`. Verify these match the actual struct.

## Reference Files to Check

When debugging, compare our code against these working implementations:

1. **econ_tof_loader.cpp** - Most similar (V4L2 ToF camera)
   - Path: `src/dataloaders/econ_tof_loader.cpp`
   - Check: V4L2 setup, buffer handling, datamap storage

2. **hesai_loader.cpp** - GPU data handling pattern
   - Path: `src/dataloaders/hesai_loader.cpp`
   - Check: How point clouds are stored in datamap

3. **ds3d_helpers.hpp** - Helper functions
   - Path: `include/.../common/ds3d_helpers.hpp`
   - Check: Available functions for datamap manipulation

4. **organized_cloud_config.hpp** - Sensor config struct
   - Path: `include/.../common/organized_cloud_config.hpp`
   - Check: Exact field names and types

## Build Commands

```bash
cd /workspaces/atlas_levo
colcon build --packages-select atlas_ds3d_components --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee build_output.txt
```

## Quick Fixes Checklist

If you get errors about:

- [ ] **"RosHeader not found"** → Check ds3d_helpers.hpp for actual name
- [ ] **"storeRosHeaderInDataMap not found"** → Find equivalent function in ds3d_helpers
- [ ] **"Frame2DGuard"** → May need to use FrameGuard or different type
- [ ] **"setGuardData"** → Check GuardDataMap API in ds3d/common/common.h
- [ ] **"setDataFrame"** → Check actual method name for frame storage
- [ ] **CUDA errors** → Check kernel launch parameters match cubeeye_kernels.hpp declarations

## Architecture Overview

```
CubeEyeLoader (DS3D DataLoader)
├── startImpl()
│   ├── Find device (by serial or auto-detect)
│   ├── initV4L2() → Open device, setup buffers
│   ├── Read intrinsics from sensor calibration page 0x0005
│   ├── Initialize CUDA (stream, buffers, gradient LUT)
│   ├── startStreaming() → Queue V4L2 buffers, STREAMON
│   └── Start capture thread
│
├── captureThread()
│   ├── select() wait for V4L2 frame
│   ├── DQBUF → get frame
│   ├── processFrame() → CUDA extraction + point cloud
│   └── Store in single-slot buffer, notify condition variable
│
├── readDataImpl()
│   ├── Wait on condition variable for frame
│   ├── Take latest frame from buffer
│   ├── Store in GuardDataMap:
│   │   ├── DS3D::LidarXYZI (point cloud)
│   │   ├── DS3D::CubeEyeDepth (depth image)
│   │   ├── DS3D::CubeEyeAmplitude (amplitude image)
│   │   ├── RosHeader (frame_id, timestamp)
│   │   └── OrganizedCloudConfig (rows, cols, sensor info)
│   └── Return datamap
│
└── stopImpl()
    ├── Stop capture thread
    ├── Stop V4L2 streaming
    ├── Free CUDA resources
    └── Cleanup V4L2 buffers
```

## Sensor Details

- **Model:** CubeEye I200D
- **Raw frame:** 1600x241 YUYV, 771,200 bytes
- **Output:** 640x480 depth (mm) + amplitude
- **Frame rate:** 15 FPS
- **Calibration:** Page 0x0005 contains lens intrinsics (fx, fy, cx, cy, k1, k2, p1, p2, k3)

## Git Commits

- `atlas_levo`: `69caaaf` - Add CubeEye DS3D dataloader
- `cubeeye_nano_driver`: `28e5a0e` - Update CLAUDE_CONTEXT.md

## After Fixing Compile Errors

Test with:
```bash
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$(ros2 pkg prefix atlas_ds3d_components)/lib/atlas_ds3d_components/gstreamer-1.0

GST_DEBUG=cubeeye_loader:5 gst-launch-1.0 \
  nvds3d_dataloader name=loader \
  library=libatlas_ds3d_dataloader_cubeeye.so \
  create-function=createCubeEyeLoader \
  config="cubeeye_loader: { serial: 'I200DU2509000349' }" \
  ! fakesink
```

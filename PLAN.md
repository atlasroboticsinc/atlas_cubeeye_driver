# CubeEye Custom ToF Driver with Dual-Frequency CRT Unwrapping

## Project Goal
Bypass the proprietary CubeEye SDK and implement a custom V4L2 + CUDA driver that performs dual-frequency Chinese Remainder Theorem (CRT) phase unwrapping to achieve accurate depth measurements beyond 1.5m.

**Target:** Match SDK depth accuracy (~2416mm at test distance) using raw sensor data.

---

## Background & Context

### The Problem
- CubeEye I200D ToF sensor uses **dual-frequency modulation** (80MHz + 100MHz)
- Single frequency 100MHz wraps at **1499mm** - insufficient for our use case
- Single frequency 80MHz wraps at **1875mm** - also insufficient
- Combined CRT unwrapping extends range to **~7.5m** (LCM of ranges)
- The proprietary SDK does this internally but we need direct control

### What We Discovered (from prior analysis on levo-0006)
1. **Raw frame format:** 1600x241 uint16 via V4L2
   - Row 0: Header (32 bytes meaningful)
   - Rows 1-240: Pixel data
   - 1600 = 5 × 320 (5 sub-pixels per spatial position)

2. **Sub-pixel layout hypothesis:**
   - Sub[0]: ~2000 mean - likely 80MHz phase/I component
   - Sub[1]: ~2000 mean - likely 80MHz Q component or amplitude
   - Sub[2]: ~6500 mean - likely 100MHz phase/I component
   - Sub[3]: ~19500 mean - likely 100MHz Q component or amplitude
   - Sub[4]: ~20000 mean - amplitude or metadata

3. **SDK internals (from decompiled libCubeEye.so):**
   - `ChineseReminderDualDepthUnwrapper::unwrap(phase1, amp1, phase2, amp2, *depth, *amp)`
   - `FastCRTDualDepthUnwrapper::calcDualAmplitudeDepth(...)`
   - Uses `DUAL_FREQ_TYPE` enum for frequency configuration

---

## Technical Parameters

### Frequency Constants
```
c = 299,792,458 m/s (speed of light)
f_80MHz  = 80,000,000 Hz  → range = c/(2*f) = 1.8748m = 1874.8mm
f_100MHz = 100,000,000 Hz → range = c/(2*f) = 1.4990m = 1499.0mm
Combined CRT range = LCM(1875, 1499) ≈ 7.5m
```

### Expected Wrapped Values for 2416mm
```
2416mm mod 1875mm = 541mm  → phase_80 ≈ 1.814 rad → 12-bit: ~1182
2416mm mod 1499mm = 917mm  → phase_100 ≈ 3.844 rad → 12-bit: ~2506
```

### CRT Algorithm
```
Given: depth_80 (wrapped 0-1875mm), depth_100 (wrapped 0-1499mm)
Find: true_depth where:
  true_depth ≡ depth_80 (mod 1875)
  true_depth ≡ depth_100 (mod 1499)

Solution uses Extended Euclidean Algorithm:
  M = 1875 * 1499 = 2,810,625
  M1 = 1499, M2 = 1875
  Find y1, y2 such that: M1*y1 ≡ 1 (mod 1875) and M2*y2 ≡ 1 (mod 1499)
  true_depth = (depth_80 * M1 * y1 + depth_100 * M2 * y2) mod M
```

---

## Key File Locations on feynman

```
SDK:
  ~/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0/
  ├── lib/libCubeEye.so          (x86_64)
  ├── include/CubeEye/*.h        (headers)
  └── thirdparty/                (opencv, live555 deps)

Decompiled SDK (reference):
  ~/development/atlas/code/libCubeEye_decompiled.c        (115K lines)
  ~/development/atlas/code/libCubeEye_decompiled_6.7b.c   (657K lines)

Existing ROS2 driver:
  ~/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/src/CubeEyeCameraNode.cpp
```

---

## Implementation Plan

### Phase 1: Sensor Verification
1. Power cycle CubeEye sensor
2. Verify USB enumeration: `lsusb | grep -i meere` or VID `2959`
3. Check video device: `ls /dev/video*`
4. Test SDK capture works (ground truth reference)

### Phase 2: Raw V4L2 Capture
1. Create standalone V4L2 capture tool
2. Capture raw 1600x241 frames while SDK keeps sensor streaming
3. Save frames to files for analysis
4. Validate header and pixel data structure

### Phase 3: Data Format Reverse Engineering
1. Analyze sub-pixel layout across multiple frames
2. Identify which sub-pixels contain:
   - 80MHz I/Q or phase
   - 100MHz I/Q or phase
   - Amplitude data
3. Verify by computing single-frequency depths and comparing to expected wrapped values

### Phase 4: CUDA CRT Implementation
1. Implement 4-phase IQ demodulation kernel (if needed)
2. Implement CRT unwrapping kernel:
   ```cuda
   __global__ void crt_unwrap_kernel(
       const uint16_t* raw_frame,
       float* depth_out,
       float* amplitude_out,
       int width, int height
   );
   ```
3. Handle edge cases: low amplitude, phase noise, wrap boundaries

### Phase 5: Validation
1. Compare CUDA output to SDK output
2. Target: <10mm error at 2.4m range
3. Test at multiple distances to verify CRT is working

---

## Project Structure (to create)

```
~/development/atlas/code/cubeeye_nano_driver/
├── src/
│   ├── sdk_capture.cpp       # SDK-based capture (ground truth)
│   ├── v4l2_capture.cpp      # Raw V4L2 capture
│   ├── crt_depth.cu          # CUDA CRT unwrapping kernel
│   ├── frame_analyzer.cpp    # Frame data analysis tools
│   └── compare_drivers.cpp   # Side-by-side comparison
├── include/
│   └── cubeeye_nano.h
├── build/
├── data/                     # Captured raw frames
├── CMakeLists.txt
└── README.md
```

---

## Hardware Setup

- **GPU:** NVIDIA RTX 5080 (CUDA 12.8)
- **Sensor:** CubeEye I200D (USB, VID possibly 2959 or check dmesg)
- **Video device:** Usually /dev/video0 or /dev/video2

---

## Commands Reference

```bash
# Check sensor
lsusb | grep -i 'meere\|cube\|2959'
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all

# Build CUDA
nvcc -o build/crt_depth src/crt_depth.cu -O3

# Run SDK with libs
SDK_PATH=~/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0
export LD_LIBRARY_PATH=$SDK_PATH/lib:$SDK_PATH/thirdparty/libopencv/lib:$LD_LIBRARY_PATH
./build/sdk_capture
```

---

## Success Criteria

1. ✅ Capture raw V4L2 frames independently
2. ✅ Decode dual-frequency phase data correctly
3. ✅ Implement CRT unwrapping in CUDA
4. ✅ Match SDK depth output within 10mm at 2.4m
5. ✅ Achieve >20 FPS processing rate

---

## Notes

- The SDK holds exclusive V4L2 access while running
- Sensor may stop streaming immediately when SDK is killed
- May need to use SDK to wake sensor, then quickly capture via V4L2
- Alternative: hook into SDK's raw frame callback if available

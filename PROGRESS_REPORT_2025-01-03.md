# CubeEye Custom Driver Project - Progress Report
**Date:** January 3, 2025 (Updated)
**Status:** Phase 2 Complete - Raw Frame Capture Working
**Machine:** feynman (x86_64, RTX 5080)

---

## Summary of This Session

Successfully created a working system to capture raw V4L2 frames from the CubeEye sensor while simultaneously capturing SDK depth output for comparison.

### Key Achievements

1. **Established New Ground Truth** (bench sensor at ~1.37m)
   - SDK Depth: **1367.15 mm mean** (std: 1.96mm)
   - Very stable readings: 1362-1371mm range

2. **Created LD_PRELOAD V4L2 Hook**
   - Intercepts V4L2 ioctl calls from within the SDK
   - Captures raw frames before SDK processing
   - Overcame sign-extension bug in ioctl request codes

3. **Built Benchmark Capture System**
   - `benchmark_capture` - Captures SDK depth + amplitude frames
   - `libv4l2_hook.so` - Intercepts raw V4L2 frames
   - Both save with matching frame numbers for comparison

4. **Analyzed Raw Frame Format**
   - Frame size: 1600×241 uint16 (771,200 bytes)
   - Row 0: Header (32 bytes meaningful)
   - Rows 1-240: Pixel data with 5 sub-pixels per spatial position

---

## Raw Data Format (Bench Sensor)

### Sub-Pixel Statistics (from center row)

| Sub-Pixel | Mean | Min | Max | Notes |
|-----------|------|-----|-----|-------|
| Sub[0] | ~57 | 0 | 511 | Low values, phase offset? |
| Sub[1] | ~57 | 0 | 511 | Low values, phase offset? |
| Sub[2] | ~14,670 | 0 | 65,535 | Main depth data |
| Sub[3] | ~14,571 | 0 | 65,280 | Main depth data |
| Sub[4] | ~16,022 | 0 | 65,281 | Amplitude/confidence? |

### Comparison with Previous Analysis (levo-0006)

The bench sensor shows a **different data layout** than the robot sensor:

| Sub | Bench (feynman) | Robot (levo-0006) |
|-----|-----------------|-------------------|
| 0 | ~57 | ~2000 |
| 1 | ~57 | ~2000 |
| 2 | ~14,670 | ~6500 |
| 3 | ~14,571 | ~19,500 |
| 4 | ~16,022 | ~20,000 |

This suggests either:
- Different firmware/configuration
- Different sensor model variant
- Different modulation frequencies

---

## Current File Structure

```
~/development/atlas/code/cubeeye_nano_driver/
├── PLAN.md                      # Original implementation plan
├── PROGRESS_REPORT_2025-01-03.md # This file
├── analyze_benchmark.py         # Python analysis script
├── capture_raw.sh               # Quick capture script
├── CMakeLists.txt
├── build/
│   ├── sdk_capture              # SDK-based capture
│   ├── benchmark_capture        # Benchmark tool
│   ├── v4l2_capture             # Standalone V4L2 capture
│   ├── frame_analyzer           # Frame analysis tool
│   └── libv4l2_hook.so          # LD_PRELOAD hook library
├── src/
│   ├── sdk_capture.cpp          # SDK capture source
│   ├── benchmark_capture.cpp    # Benchmark tool source
│   ├── v4l2_capture.cpp         # V4L2 capture source
│   ├── frame_analyzer.cpp       # Frame analyzer source
│   └── v4l2_hook.c              # V4L2 hook library
├── benchmark/                   # Latest benchmark capture
│   ├── benchmark_stats.csv      # SDK frame statistics
│   ├── sdk_depth_NNNN.raw       # SDK depth frames (640x480 uint16)
│   ├── sdk_amp_NNNN.raw         # SDK amplitude frames
│   └── raw_NNNN.raw             # Raw V4L2 frames (1600x241 uint16)
└── data/                        # Earlier test captures
```

---

## Commands Quick Reference

### Run Benchmark Capture
```bash
LD_LIBRARY_PATH="/path/to/cubeeye2.0/lib:..." \
LD_PRELOAD="./build/libv4l2_hook.so" \
V4L2_HOOK_OUTPUT="benchmark" \
./build/benchmark_capture 50 benchmark
```

### Analyze Benchmark
```bash
python3 analyze_benchmark.py benchmark
```

### Just SDK Capture (ground truth)
```bash
LD_LIBRARY_PATH="/path/to/cubeeye2.0/lib:..." \
./build/sdk_capture 50
```

---

## Next Steps

### Phase 3: Decode Raw Data Format
1. Understand the mapping between Sub[0-4] and dual-frequency phase data
2. Determine if this sensor uses different modulation frequencies
3. Check frame header for configuration information

### Phase 4: Implement CUDA CRT Processing
1. Create CUDA kernel for depth calculation from raw data
2. Implement CRT unwrapping algorithm
3. Handle calibration and sensor-specific parameters

### Phase 5: Validation
1. Compare CUDA output to SDK output frame-by-frame
2. Target: <10mm error at measured distance
3. Measure processing latency

---

## Technical Discoveries

### Sign Extension Bug
The LD_PRELOAD ioctl hook wasn't catching DQBUF calls due to sign extension.
The `request` parameter is `unsigned long` (64-bit), but V4L2 ioctl codes are 32-bit.
When passed through variadic args, they get sign-extended if the high bit is set.

**Fix:** Cast to 32-bit before comparison:
```c
unsigned int req32 = (unsigned int)request;
if (req32 == VIDIOC_DQBUF && result == 0) { ... }
```

### Frame Warmup
First 4 raw frames after STREAMON have `bytesused=0` and contain all zeros.
These are warmup frames while the sensor initializes. Valid data starts at frame 4.

### SDK Exclusivity
The SDK holds V4L2 exclusive access. Cannot capture raw frames directly.
Solution: LD_PRELOAD hook intercepts raw data before SDK processes it.

---

## Hardware Setup

```
GPU:     NVIDIA RTX 5080, CUDA 12.8
Sensor:  CubeEye I200D at /dev/video0
         VID: 3674, PID: 0200
         SN: I200DU2509000349
Driver:  uvcvideo
SDK:     ~/development/atlas/code/atlas_levo/src/3rdparty/drivers/
         ros2-cubeeye2.0/cubeeye2.0/
```

---

*Report updated: January 3, 2025*

# CubeEye SDK Pipeline Analysis

## Executive Summary

Through GDB/perf tracing and Ghidra reverse engineering, we have confirmed that:

1. **The CubeEye I200D sensor provides pre-computed depth values**
2. **The SDK does NOT perform phase unwrapping during frame processing**
3. **The SDK only applies post-processing corrections (filters, gradient table)**

## Evidence

### 1. GDB/Perf Tracing Results

Using `perf record` on the running SDK, we captured the hot functions:

```
16.27%  libCubeEye.so  meere::sensor::I200F::imageProcessProc
 2.44%  libCubeEye.so  meere::sensor::BaseFilter::FlyingPixelFilter2
 0.18%  libCubeEye.so  meere::sensor::I200F::readFrameProc
 0.14%  libm.so.6      __ieee754_pow_fma
```

**Notably ABSENT from the profile:**
- `ChineseReminderDualDepthUnwrapper::unwrap`
- Any CRT-based phase calculation functions
- Multi-frequency depth fusion

### 2. Disassembly Analysis

The `imageProcessProc` function (at offset 0x208ea0, size 12048 bytes) contains:

#### Frame Casting (no calculation)
```c
frame_cast_basic16u((sensor *)local_c8, (shared_ptr *)local_128);
```
The depth frame is directly cast to uint16 - no conversion from phase to depth.

#### Gradient Table Correction
```asm
+9777:  movzwl (%rax),%edx           # edx = depth[i]
+9780:  cmp    $0x1d4c,%dx           # if depth > 7500
+9790:  movzwl 0x468(%rsi,%rdx,2),%edx  # edx = gradient_table[depth]
+9798:  mov    %dx,(%rax)            # depth[i] = corrected
```

The gradient table at offset 0x468 is indexed BY DEPTH VALUE, meaning depth is already computed before this correction.

#### Flying Pixel Filter
```c
void BaseFilter::FlyingPixelFilter2(
    ushort *depth_data,  // Input/output depth buffer
    int width,
    int height,
    ushort threshold,
    ushort *output
)
```

This filter removes "flying pixels" at depth discontinuities - a post-processing operation.

### 3. Unwrap Function Exists But Not Called

The library contains unwrapping code:
```
_ZN5meere6sensor33ChineseReminderDualDepthUnwrapper6unwrapEttttPtS2_
_ZN5meere6sensor33ChineseReminderDualDepthUnwrapper20setDualFrequencyTypeENS1_14DUAL_FREQ_TYPEE
```

But this code is **never invoked** during standard frame acquisition. It appears to be:
- Legacy code for older sensor models
- Code for special multi-frequency modes not enabled by default

### 4. SDK Function Summary

| Function | Purpose | Calls Unwrap? |
|----------|---------|---------------|
| `readFrameProc` | Read frames from USB/V4L2 | No |
| `imageProcessProc` | Main processing | No |
| `FlyingPixelFilter2` | Remove flying pixels | No |
| Gradient table | Polynomial correction | No |

## SDK Data Flow

```
[Sensor Hardware]
       │
       │  USB bulk transfer (512 bytes/packet)
       │  Data is PRE-COMPUTED DEPTH (uint16)
       ▼
[readFrameProc]
       │
       │  Create StreamBuffer
       │  Queue for processing
       ▼
[imageProcessProc]
       │
       ├── frame_cast_basic16u() - Cast to uint16
       ├── FlyingPixelFilter2() - Remove flying pixels
       ├── Gradient table lookup - Polynomial correction
       └── Depth range filter - Clip to valid range
       │
       ▼
[Output to Application]
       Depth: uint16 mm values
       Amplitude: uint16 reflectivity
```

## Implications for Custom Driver

Since the sensor provides pre-computed depth:

1. **We do NOT need to implement:**
   - Phase unwrapping algorithms
   - CRT (Chinese Remainder Theorem) calculation
   - Multi-frequency fusion

2. **We MUST implement:**
   - USB bulk transfer protocol
   - Frame parsing (header + data extraction)
   - Gradient table correction (14-coefficient polynomial)
   - Optional: Flying pixel filter

3. **Formula for gradient correction:**
   ```
   gradient_table[i] = polynomial(i) for i in 0..7500
   where polynomial = sum(coeff[k] * i^k) for k in 0..13
   ```

## Open Questions

1. **Where exactly does the sensor compute depth?**
   - Most likely: On-chip DSP in the ToF sensor
   - The Melexis MLX75027 has on-chip depth calculation capability

2. **What is Sub[2] in the raw frame?**
   - Most likely: The pre-computed depth value before SDK corrections
   - Correlation with SDK output supports this

3. **Can we access raw phase data?**
   - Not in default mode
   - Would require special sensor firmware/configuration

## Test Methodology

Commands used for this analysis:

```bash
# Perf profiling
sudo sysctl kernel.perf_event_paranoid=-1
sudo perf record -g ./build/sdk_capture 100
sudo perf report

# GDB tracing
gdb -x gdb_script.txt ./build/sdk_capture

# Symbol extraction
strings libCubeEye.so | grep -i unwrap
nm -D libCubeEye.so | grep -i I200F
```

## Conclusion

The CubeEye I200D sensor operates as a **self-contained ToF camera** that performs all depth calculation internally. The SDK serves primarily as:
1. USB communication layer
2. Post-processing pipeline
3. Configuration interface

This significantly simplifies custom driver development - we only need to replicate the USB protocol and post-processing, not the fundamental ToF algorithms.

# Comprehensive Investigation Summary: CubeEye I200D Depth Pipeline

**Date:** 2026-01-06
**Objective:** Reverse-engineer how the CubeEye I200D SDK converts raw sensor data to depth values

## Executive Summary

This session focused on reverse-engineering how the CubeEye I200D SDK converts raw sensor data to depth values. The investigation revealed a **critical finding**: the I200D sensor performs phase unwrapping **on-chip**, meaning the raw V4L2 frames do NOT contain the wrapped phase data we assumed.

---

## 1. Red Team Analysis of Previous Assumptions

### 1.1 Previous Claims Found to be WRONG

| Previous Claim | Reality |
|----------------|---------|
| "Correlation 0.039 with SDK" | Based on **unsynchronized frames** - methodologically flawed |
| "TAP formula I=T4-T2, Q=T4-T0 works" | Gives **near-zero correlation** even with synchronized data |
| "0.1mm unit hypothesis" | **WRONG** - RMSE of 4094mm |
| "Sub[2] is raw wrapped phase" | **Partially true** but sensor unwraps on-chip |

### 1.2 Methodology Errors Identified

1. **Frame Synchronization Issue**: Earlier comparisons used `fppn_test/raw_*.raw` (Jan 6) with `data/sdk_depth_frame_0.raw` (Jan 3) - completely different capture sessions

2. **Zero-Filled Raw Captures**: The `raw_benchmark/` and `benchmark/` directories contained **ALL ZEROS** in raw files - the V4L2 hook wasn't capturing data properly when SDK filters were disabled

3. **Correct Synchronized Data**: The `variable_dist/` directory contains 150 properly synchronized raw+SDK frame pairs

---

## 2. Verified Raw Frame Format

### 2.1 Frame Structure
```
File size: 771,200 bytes
Dimensions: 241 rows × 1600 columns of uint16
Row 0: Metadata (13 non-zero values)
Rows 1-240: Pixel data for 320×240 output (5 sub-pixels per output pixel)
```

### 2.2 Sub-Pixel Layout (Interleaved)
For each output pixel at column `x`:
- `Sub[0]` at column `x*5 + 0`: Range 0-500, mean ~62
- `Sub[1]` at column `x*5 + 1`: Range 0-501, mean ~62
- `Sub[2]` at column `x*5 + 2`: Range 0-65535, mean ~15700 (PRIMARY DATA)
- `Sub[3]` at column `x*5 + 3`: 8-bit value in HIGH byte (multiples of 256)
- `Sub[4]` at column `x*5 + 4`: 8-bit value in HIGH byte (multiples of 256)

### 2.3 Metadata Row (Row 0) Analysis
```
Position 0: 2
Position 1: 768 (0x0300)
Position 2: 245 (height?)
Position 6: 60 (frequency indicator? 60 MHz?)
Position 7: 3864
```

---

## 3. Sub[2] Behavior Analysis

### 3.1 Non-Monotonic Relationship with Depth (Confirmed Phase Wrapping)
```
SDK Depth → Sub[2] Value
500mm    → 2,833
1000mm   → 11,109
1500mm   → 55,381 (jumps UP)
2000mm   → 2,579 (drops!)
2500mm   → 5,651
3000mm   → 10,544
```

**Phase wrap detected between 1600mm and 1800mm** - consistent with 81 MHz modulation (max range ~1852mm)

### 3.2 Correlation Analysis
| Test | Correlation |
|------|-------------|
| Sub[2] vs SDK depth (direct) | -0.2887 |
| All I/Q formulas tested | Near zero (-0.01 to -0.12) |
| Polynomial fit (degree 3) | 0.3688 |
| After phase unwrap | 0.1470 |
| FPPN correction effect | +0.001 (negligible) |

### 3.3 Key Observation
Despite Sub[2] showing phase-wrapping behavior, **no simple formula correlates with SDK depth above 0.4**

---

## 4. Polynomial Coefficients Extracted (DAT_00499da0)

### 4.1 Location
- File offset: `0x399DA0` in `libCubeEye.so`
- 14 double-precision coefficients for degree-13 polynomial

### 4.2 Coefficients
```
coeff[0]  = -9.463067650000e-08  (x^13)
coeff[1]  = +5.408286950000e-06  (x^12)
coeff[2]  = -1.338211660000e-04  (x^11)
coeff[3]  = +1.894635730000e-03  (x^10)
coeff[4]  = -1.704659880000e-02  (x^9)
coeff[5]  = +1.021873970000e-01  (x^8)
coeff[6]  = -4.155845870000e-01  (x^7)
coeff[7]  = +1.144336150000e+00  (x^6)
coeff[8]  = -2.090441090000e+00  (x^5)
coeff[9]  = +2.429400770000e+00  (x^4)
coeff[10] = -1.668353570000e+00  (x^3)
coeff[11] = +5.878545160000e-01  (x^2)
coeff[12] = -7.662263700000e-02  (x^1)
coeff[13] = +3.443448410000e-04  (x^0)
```

### 4.3 Formula (from `makeDepthGradientTable`)
```c
correction = Σ(coeff[i] × (depth_mm/1000)^(13-i)) for i=0..13
corrected_depth = depth + correction × 1000
```

### 4.4 Correction Magnitudes
```
Depth    Correction
500mm    +2.0mm
1000mm   -2.2mm
1500mm   -2.0mm
2000mm   -9.8mm
3000mm   -25.6mm
5000mm   -31.3mm
```

**Saved to: `poly_coeffs.json`**

---

## 5. CRT Frequency Configuration Discovered

### 5.1 Dual Frequency Type 0 (Default)
From `setDualFrequencyType()`:
```c
*(undefined8 *)(this + 8) = 0x4c9896804cbebc20;
// Decodes to:
// Frequency 1: 100 MHz (offset+8) → max_range = 1500mm
// Frequency 2: 80 MHz (offset+12) → max_range = 1875mm
```

### 5.2 Hypothesis Tables
- Table 1: `DAT_0051ba40`
- Table 2: `DAT_0051ba00`
- Used for CRT ambiguity resolution

### 5.3 Combined Unambiguous Range
LCM-based: **7500mm** (matches SDK max depth observation of 7454mm)

---

## 6. CRITICAL FINDING: No Unwrap Functions Called

### 6.1 LD_PRELOAD Interception Test
Created interceptors for all unwrap-related functions:
- `_ZN5meere6sensor33ChineseReminderDualDepthUnwrapper6unwrapEttttPtS2_`
- `_ZN5meere6sensor33ChineseReminderDualDepthUnwrapper22calcDualAmplitudeDepthE...`
- `_ZN5meere6sensor18Fast2DUnwrapFilter6unwrapEPtS2_tt`
- `_Z8unwrap2DPtiiS_t`
- `_Z13init_unwrap2Dii`

### 6.2 Test Conditions
- **Near-field test**: Sensor at ~1.5m (within single-frequency range)
- **Far-field test**: Sensor at ~3.2m (beyond 1.85m single-frequency limit)

### 6.3 Results
```
=== All Unwrap Interceptor Loaded ===
[No other function calls logged]

SDK depth output: 3210-3220mm (accurate per user measurement of 3.23m)
```

**NONE of the unwrap functions were called, even for depths requiring phase unwrapping!**

### 6.4 Verified Frame Patterns
Checked 16 consecutive frames for temporal multiplexing:
- All frames have nearly identical Sub[2] patterns
- No alternating frequency pattern detected
- **Conclusion: NOT temporally multiplexed**

---

## 7. Conclusions

### 7.1 Primary Finding
**The I200D sensor performs phase unwrapping ON-CHIP.** The SDK receives pre-processed depth data, not raw wrapped phase requiring software CRT.

### 7.2 Implications

1. **Sub[2] is NOT what we thought**: While it shows phase-wrapping-like patterns, it's not the raw phase the SDK uses for depth calculation

2. **CRT code exists but isn't used**: The `ChineseReminderDualDepthUnwrapper` class is instantiated but its `unwrap()` method is never called for I200D

3. **The sensor likely sends**:
   - Either pre-unwrapped depth in a format we haven't decoded
   - Or multiple frequency data that the on-chip processor combines

4. **FPPN is largely irrelevant**: The 1.12% missing pixels and fill strategy don't matter when we can't even compute the base depth

### 7.3 What Works
- Polynomial gradient correction coefficients: **EXTRACTED**
- CRT frequency configuration: **IDENTIFIED**
- SDK filter properties: **DOCUMENTED**
- Raw frame format: **UNDERSTOOD** (241×1600, 5 sub-pixels)

### 7.4 What Remains Unknown
1. **What Sub[2] actually represents** for I200D
2. **How the sensor encodes unwrapped depth** in the raw frame
3. **Whether a different field contains the depth** (not Sub[2])
4. **The on-chip processing algorithm**

---

## 8. Files Created/Modified This Session

| File | Description |
|------|-------------|
| `poly_coeffs.json` | 14 polynomial coefficients for gradient correction |
| `far_field_benchmark/sdk_depth_*.raw` | SDK depth at 3.2m distance |
| `far_field_benchmark/sdk_amp_*.raw` | SDK amplitude at 3.2m |
| `/tmp/libcubeeye_intercept.so` | LD_PRELOAD interceptor for unwrap functions |
| `/tmp/liball_unwrap.so` | Comprehensive unwrap function interceptor |

---

## 9. Recommended Next Steps

### Option A: Accept Findings and Pivot
- Use SDK for depth computation
- Focus on other driver components (camera control, frame capture, output formatting)
- Treat the sensor as a "black box" that outputs depth

### Option B: Deeper Investigation
- Examine what the sensor ACTUALLY sends by comparing raw frames at multiple known distances
- Look for a different field that might contain unwrapped depth
- Check if there's a "raw mode" that bypasses on-chip processing

### Option C: Contact Manufacturer
- The on-chip unwrapping is proprietary
- Only manufacturer documentation would reveal the true encoding

---

## 10. Key Technical Insights for Future Reference

1. **Symbol Name Mangling**: `ChineseReminderDualDepthUnwrapper` is 33 characters, so mangled name uses `33` not `31`

2. **LD_PRELOAD Works**: Verified with malloc interception test - the technique is valid

3. **SDK Max Depth**: 7454mm observed, consistent with 100/80 MHz dual-frequency CRT (LCM = 7500mm)

4. **Frame Rate**: All frames show consistent Sub[2] patterns - no temporal frequency multiplexing

5. **Phase Wrap Location**: Between 1600-1800mm, consistent with 81 MHz (~1852mm max range)

---

## Appendix A: Correlation Test Results (Synchronized Data)

Using `variable_dist/raw_0005.raw` + `variable_dist/sdk_depth_0005.raw`:

```python
# Direct correlations
TAP0 vs SDK: corr=-0.2859
TAP1 vs SDK: corr=-0.2845
TAP2 vs SDK: corr=-0.2880
TAP3 vs SDK: corr=-0.2855
TAP4 vs SDK: corr=-0.2635

# I/Q formula correlations
T0-T2, T1-T3     max=2000: corr=-0.0119
T2-T0, T3-T1     max=2000: corr=-0.0020
T4-T2, T4-T0     max=2000: corr=-0.0822
T3-T1, T0-T2     max=2000: corr=-0.0043
```

## Appendix B: LD_PRELOAD Interceptor Code

```c
// Key interceptor for CRT unwrap
void _ZN5meere6sensor33ChineseReminderDualDepthUnwrapper6unwrapEttttPtS2_(
    void* this_ptr, uint16_t phase1, uint16_t phase2,
    uint16_t amp1, uint16_t amp2, uint16_t* out_depth, uint16_t* out_amp) {

    log_msg("CRT unwrap called!");  // NEVER LOGGED

    static void (*orig)(...) = dlsym(RTLD_NEXT, "...");
    if (orig) orig(this_ptr, phase1, phase2, amp1, amp2, out_depth, out_amp);
}
```

## Appendix C: Polynomial Gradient Table Algorithm

From decompiled `makeDepthGradientTable()`:
```c
do {
    local_48 = 0.0;
    lVar4 = 0;
    do {
        dVar1 = (double)(&DAT_00499da0)[lVar4];  // coefficient
        lVar3 = 0xd - lVar4;                      // power: 13, 12, 11, ... 0
        lVar4 = lVar4 + 1;
        dVar6 = pow((double)((float)lVar5 / 1000.0), (double)lVar3);
        local_48 = dVar6 * dVar1 + local_48;
    } while (lVar4 != 0xe);  // 14 coefficients

    uVar2 = (ushort)(int)((double)lVar5 + local_48 * 1000.0);
    if (0x1d4c < uVar2) uVar2 = 0x1d4c;  // Clamp to 7500
    *(ushort *)(this + lVar5 * 2 + 0x468) = uVar2;
    lVar5 = lVar5 + 1;
} while (lVar5 != 0x1d4d);  // 7501 entries (0-7500mm)
```

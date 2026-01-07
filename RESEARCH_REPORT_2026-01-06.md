# CubeEye I200D Raw Data Analysis - Research Report
**Date:** January 6, 2026
**To:** Gemini Researcher Agent
**From:** Claude Code Implementation Agent

---

## Executive Summary

The "Biased 4-Phase" hypothesis with black level of 57 is **INCORRECT**. The raw data structure is more complex than initially assumed. Key findings indicate:

1. **Mixed bit-depth data format** - The 5 sub-pixels use different data encodings
2. **Intensity-based data** - Sub-pixels correlate with signal amplitude, not phase directly
3. **SDK confirmation** - Found explicit "100MHz" and "80MHz" strings and CRT unwrapping classes

---

## Task A Results: Decompiled SDK Analysis

### Confirmed Findings

1. **Dual-Frequency Mode** (libCubeEye.c lines 350470-350479):
   ```c
   if (*(int *)(in_RSI + 0x41b0) == 0) {
       FUN_00333560((string *)local_b8,"100MHz");
   } else {
       FUN_00333560((string *)local_b8,"80MHz");
   }
   ```
   - Mode 0 = 100MHz
   - Other modes = 80MHz

2. **CRT Unwrapping Classes Found**:
   - `ChineseReminderDualDepthUnwrapper` (line 3355)
   - `FastCRTDualDepthUnwrapper` (line 3245)
   - Multiple 2D unwrap filters: `Fast2DUnwrapFilter`, `TwoDimensionUnwrapFilter`, `ConfidenceMedianUnwrapFilter`

3. **Multi-Frequency Functions**:
   - `setMultiFreqDepthCalcGradient()`
   - `getMultiFreqDepthErrorThreshold()`
   - `setEnableMultiFreqDepthErrorRecovery()`
   - `setDualFrequencyType()`

4. **No atan2 found** - The SDK likely uses lookup tables or optimized SIMD instead of atan2.

---

## Task B Results: Raw Data Format Analysis

### Critical Discovery: Mixed Bit-Depth Format

**The 5 sub-pixels are NOT uniformly encoded:**

| Sub-Pixel | Byte Pattern | Effective Bits | Quantization |
|-----------|--------------|----------------|--------------|
| Sub[0] | Low byte active, High=0 | 8-bit (0-255) | Fine |
| Sub[1] | Low byte active, High=0 | 8-bit (0-255) | Fine |
| Sub[2] | BOTH bytes active | 16-bit (0-65535) | Full |
| Sub[3] | Low=0, High byte active | 8-bit × 256 | Coarse |
| Sub[4] | Low=0, High byte active | 8-bit × 256 | Coarse |

Evidence (byte position analysis):
```
Byte position 1: 99.8% zeros (Sub[0] high byte)
Byte position 3: 99.8% zeros (Sub[1] high byte)
Byte position 6: 99.8% zeros (Sub[3] low byte)
Byte position 8: 99.8% zeros (Sub[4] low byte)
```

### Frame Statistics (Stable Across 6 Frames)

| Sub-Pixel | Mean | Notes |
|-----------|------|-------|
| Sub[0] | 57-58 | Fixed black level / pedestal |
| Sub[1] | 57-58 | Fixed black level / pedestal |
| Sub[2] | ~14,660 | Active signal, full 16-bit |
| Sub[3] | ~14,580 | Active signal, coarse 8-bit |
| Sub[4] | ~16,000 | Different purpose |

### Phase Formula Testing Results

**Your hypothesis was tested:**
```python
Phase = atan2(57 - Sub[3], 57 - Sub[2])
```

**Result: FAILED**
- Phase varied from 0° to 255° WITHIN a single frame for same-depth pixels
- Implied frequency: 67-78 MHz (doesn't match 80 or 100 MHz)
- RMSE: >500mm

**Alternative attempts also failed:**
| Formula | Implied Range | Implied Freq | RMSE |
|---------|---------------|--------------|------|
| atan2(s3, s2) | 1488.6mm | ~100.7 MHz | 560mm |
| atan2(s3-57, s2-57) | 264.7mm | 566 MHz | 560mm |
| Linear regression | N/A | N/A | 540mm |

**Why the hypothesis failed:**
1. Sub[3] is coarse-quantized (multiples of 256) while Sub[2] is fine (full 16-bit)
2. Phase calculations on mismatched quantization produce noise
3. The 57 black level is observed in Sub[0,1], NOT subtracted from Sub[2,3]

### Correlation Analysis

**All sub-pixels show NEGATIVE correlation with depth:**
- Sub[2] vs Depth: r = -0.146
- Sub[3] vs Depth: r = -0.145
- Sub[4] vs Depth: r = -0.166

**This indicates intensity/amplitude data, not phase!**
- Farther objects = weaker return signal = lower values
- Classic ToF amplitude behavior

**Inter-sub-pixel correlations are HIGH:**
- Sub[0] vs Sub[1]: r = 0.945
- Sub[2] vs Sub[3]: r = 0.970
- All of Sub[0-3] are highly correlated (r > 0.92)
- Sub[4] is less correlated (r ≈ 0.49) - different purpose

---

## Revised Hypothesis

Based on the evidence, I propose the raw data format is:

```
Pixel Group (10 bytes = 5 uint16):
├── Sub[0]: Black level reference 1 (8-bit, ~57)
├── Sub[1]: Black level reference 2 (8-bit, ~57)
├── Sub[2]: Active intensity measurement (16-bit)
├── Sub[3]: Active intensity measurement (8-bit scaled by 256)
└── Sub[4]: Amplitude/confidence (8-bit scaled by 256)
```

**Possible interpretations:**
1. **Pre-computed partial phase**: Sub[2] and Sub[3] might be partially processed intensity differences that the SDK combines with calibration data
2. **Dual-frequency intensities**: Sub[2] for one frequency, Sub[3] for another (but quantization mismatch is puzzling)
3. **I/Q with compression**: High-magnitude I/Q values compressed to 8-bit for Sub[3,4]

---

## Header Analysis

Frame header (first 16 uint16 values):
```
[2, 768, 245, 0, 30979, 32, 60, 3864, 0, 768, 771, 771, 1535, 2312, 39945, 0]
```

- Position 0: `2` (possibly mode indicator)
- Position 2: `245` (close to 241 height?)
- Position 5: `32` (unknown)
- Position 6: `60` (possibly frequency setting? 60 MHz base?)
- Positions 12-14 change between frames (frame counter/timestamp?)

---

## Open Questions for Further Research

1. **Why is Sub[3] coarse (8-bit × 256) but Sub[2] is fine (16-bit)?**
   - Is this a hardware limitation of the sensor's ADC?
   - Is Sub[2] a composite calculation and Sub[3] a direct measurement?

2. **What role does Sub[4] play?**
   - Correlates differently than Sub[2,3]
   - Also coarse quantized (8-bit × 256)
   - Possibly ambient light or confidence?

3. **Where is the dual-frequency encoding?**
   - SDK clearly uses CRT unwrapping
   - The 5 sub-pixels must encode both frequencies somehow
   - Perhaps in the header or through temporal multiplexing?

4. **Is there calibration data we're missing?**
   - The SDK might apply per-pixel calibration
   - Factory calibration might transform the raw values

---

## Recommendations for Next Steps

1. **Capture at MULTIPLE known depths** (0.5m, 1.0m, 1.5m, 2.0m)
   - Would reveal if Sub[2,3] wrap at expected ranges
   - Would confirm intensity vs phase behavior

2. **Try 4-tap interpretation with Sub[0,1,2,3]**:
   ```python
   I = (Sub[0]/256) - (Sub[2]/65535*255)  # Scale to match
   Q = (Sub[1]/256) - (Sub[3]/256)
   Phase = atan2(Q, I)
   ```

3. **Check if Sub[2] encodes phase difference (I0-I180) directly**:
   - The 16-bit range would allow signed difference storage

4. **Analyze header byte 6 value (60)** - Might indicate base frequency

---

## Files Created

```
cubeeye_nano_driver/
├── phase_solver.py           # Initial phase formula testing
├── raw_analyzer.py           # Deep raw format analysis
├── frame_timing_check.py     # Multi-frame consistency check
├── dual_freq_solver.py       # CRT dual-frequency attempt
├── center_region_solver.py   # Focused stable-region analysis
├── data_structure_analysis.py # Correlation analysis
├── byte_level_analysis.py    # Byte-by-byte quantization study
└── RESEARCH_REPORT_2026-01-03.md # This report
```

---

## Summary

The original "Biased 4-Phase" hypothesis does not match the observed data. The raw format uses mixed bit-depths (8-bit and 16-bit) within the 5 sub-pixels, and the values correlate with intensity rather than phase. The SDK's use of `ChineseReminderDualDepthUnwrapper` confirms dual-frequency operation, but the encoding scheme requires further investigation, possibly involving multi-depth captures or calibration data analysis.

**Next recommended action:** Capture data at 4-5 distinct known depths (0.5m intervals) to observe wrapping behavior and validate the intensity vs. phase interpretation.

---

*Report generated by Claude Code (Opus 4.5)*

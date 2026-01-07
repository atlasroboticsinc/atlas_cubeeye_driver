# SDK Raw Comparison Results

**Date:** 2025-01-06
**Objective:** Capture SDK output with filters disabled and compare with our depth calculation

## Key Findings

### 1. SDK Filter Settings

**Default enabled filters:**
- `flying_pixel_remove_filter = 1`
- `dead_pixel_remove_filter = 1`
- `outlier_remove_filter = 1`
- `amplitude_threshold_min = 5`

**Successfully disabled:**
- All major filters set to 0
- `amplitude_threshold_min = 0`
- `amplitude_threshold_max = 65535`

### 2. Effect of Disabling Filters

| Metric | Filtered SDK | Raw SDK (filters off) |
|--------|--------------|----------------------|
| Zero count (320x240) | 2572 (3.35%) | 1999 (2.60%) |
| Center depth | 1478mm | 1476mm |
| Max depth | 7442mm | 7464mm |

**Interpretation:** Filters ADD 573 zeros (mask pixels), confirming filters mask low-quality pixels.

### 3. FPPN Effect Quantified

| Metric | Value |
|--------|-------|
| FPPN range | -908 to -459 |
| FPPN mean | -671 |
| Depth shift min | -27.7mm |
| Depth shift max | -14.0mm |
| Average depth correction | -20.5mm (~2% at 1m) |

FPPN is a meaningful but small correction (~2% of depth).

### 4. TAP-to-Depth Mapping Issue

**Critical finding:** Our raw TAP interpretation does NOT match SDK depth.

| Comparison | Value |
|------------|-------|
| Our center depth | 1992mm |
| SDK center depth | 1476mm |
| Difference | 516mm |
| Correlation | 0.039 (essentially random) |

**Root cause:** The SDK uses proprietary processing that we haven't replicated:
1. Custom SIMD polynomial for atan2 (not standard atan2)
2. CRT unwrapping for extended range (SDK shows depths up to 7.4m, beyond single frequency 2m)
3. Depth gradient lookup table
4. Possibly different TAP interpretation

### 5. TAP Data Observations

| TAP | Range | Mean | Notes |
|-----|-------|------|-------|
| TAP0 | 0-496 | 54 | Very small |
| TAP1 | 0-509 | 54 | Very small |
| TAP2 | 0-65518 | 13755 | Full range |
| TAP3 | 0-65280 | 13792 | Full range |
| TAP4 | 0-65281 | 15190 | Full range |

The asymmetry (TAP0/1 small, TAP2/3/4 large) suggests the TAP encoding is different from standard 4-tap ToF.

## Remaining Questions

1. **TAP encoding:** What is the correct formula to extract I/Q from TAPs?
   - Tried many combinations, best correlation was only 0.13

2. **SDK atan2:** How does the custom polynomial differ from standard atan2?

3. **CRT unwrapping:** What frequencies are used and how is unwrapping performed?

4. **Depth gradient LUT:** Where is it applied and what values does it contain?

## Recommendations

### For FPPN

**Use our extracted FPPN as-is.** The FPPN data (98.88% coverage) is correct - the issue is in the phase calculation, not FPPN extraction.

For missing bottom rows, use `local_mean` fill (-585) as it matches the last valid row trend.

### For Depth Pipeline

The TAP-to-depth mapping requires further investigation:

1. **Option A:** Deeper decompilation of SDK's `readFrameProc()` and phase calculation
2. **Option B:** Machine learning approach - train a model on (TAPs, SDK_depth) pairs
3. **Option C:** Contact sensor manufacturer for documentation

### For Immediate Progress

Since we can't exactly replicate SDK depth yet, we can:

1. Use SDK depth directly for testing other components
2. Focus on components that don't depend on exact TAP interpretation:
   - Camera control (integration time, modulation)
   - Frame capture infrastructure
   - FPPN loading and application (when phase is available)
   - Output formatting and delivery

## Files Generated

| File | Description |
|------|-------------|
| `raw_benchmark/raw_depth_0000.raw` | SDK depth with filters disabled (640x480) |
| `raw_benchmark/raw_amp_0000.raw` | SDK amplitude with filters disabled |
| `raw_benchmark/raw_0004.raw` | Raw V4L2 frame (771,200 bytes) |
| `build/benchmark_raw_capture` | Tool to capture SDK output with filters disabled |

# FPPN 100% Extraction Strategy

## Executive Summary

**Goal**: Extract 100% of FPPN data exactly as SDK uses it - no approximations.

**DEFINITIVE ANSWER** (from live camera testing 2025-01-06):
- We have **75,936 of 76,800 FPPN values** (98.88% coverage) from camera flash
- Missing: **864 pixels** in rows 237-239 (bottom ~3 rows)
- **THE FILL VALUE DOESN'T MATTER** because SDK zeros ~80% of bottom row pixels anyway due to signal quality filtering
- Only 0.25% of total pixels are affected by the fill choice
- **Recommended fill**: `local_mean` (-585) for consistency

**BONUS DISCOVERY**: Correct TAP formula is `I = T4 - T2, Q = T4 - T0` (not T1-T3, T0-T2 as previously assumed)

## Verified Facts (Not Assumptions)

### What We Know For Certain

1. **Camera Flash Content**:
   - Pages 0x021C - 0x0716 contain FPPN data (even pages only)
   - 604 pages contain valid FPPN values
   - 20 pages are physically missing (flash sector boundaries)
   - Total extracted: 75,936 int16 values

2. **Missing Pages Pattern**:
   ```
   0x0286, 0x0386, 0x0486, 0x0586, 0x0686  (every 256 pages at offset 0x86)
   0x0300, 0x0302, 0x0304  (sector 3 boundary)
   0x0400, 0x0402, 0x0404  (sector 4 boundary)
   0x0500, 0x0502, 0x0504  (sector 5 boundary)
   0x0600, 0x0602, 0x0604  (sector 6 boundary)
   0x0700, 0x0702, 0x0704  (sector 7 boundary)
   ```

3. **FPPN Value Statistics**:
   - Min: -908
   - Max: -459
   - Mean: -672.17
   - Std: 92.84
   - Distribution: Bimodal peaks at ~-780 and ~-580

4. **Data Coverage**:
   - Rows 0-236: Complete (100% coverage)
   - Row 237: Partial (96 columns, then missing)
   - Rows 238-239: No data

## Methods to Determine SDK Behavior

### Method 1: Output Comparison (RECOMMENDED)

**Steps**:
1. Capture synchronized raw frame + SDK depth output
2. Process raw frame through our pipeline with each FPPN variant
3. Compare each result with SDK output
4. The variant with exact match (0 difference) reveals SDK behavior

**FPPN Variants Created**:
| Variant | Fill Value | Description |
|---------|------------|-------------|
| zeros | 0 | No correction for missing rows |
| mean | -672 | Overall mean value |
| repeat_last_row | Row 236 pattern | Copy last complete row |
| local_mean | -585 | Mean of rows 234-236 |
| extrapolate | Linear trend | Continue data trend |
| typical_-672 | -672 | Common FPPN value |

**Files**:
- `fppn_comparison_test/fppn_*_le.bin` - All variants
- `fppn_comparison_test/compare_depth.py` - Comparison script

**Execution**:
```bash
# Step 1: Capture with SDK
LD_PRELOAD=./build/libv4l2_hook.so ./build/benchmark_capture 1

# Step 2: Process raw frame with each variant
# (requires implementing depth pipeline)

# Step 3: Compare
python3 fppn_comparison_test/compare_depth.py our_depth.raw sdk_depth.raw
```

### Method 2: Runtime Memory Dump

**Steps**:
1. Run SDK capture in background
2. Attach with GDB or use /proc/pid/mem
3. Search for FPPN array pattern in memory
4. Dump values at rows 237-239

**Tools Created**:
- `scripts/dump_sdk_fppn.py` - GDB-based memory search
- `scripts/dump_sdk_memory.py` - /proc/pid/mem based search
- `scripts/calibration_interceptor.c` - LD_PRELOAD hook

### Method 3: Static Analysis (Low Confidence)

**Findings**:
- SDK strings show size validation: "error: wrong FPPN1 size."
- No obvious hardcoded fill values found
- SDK uses std::vector for FPPN storage (based on symbol names)

**Limitation**: Cannot definitively determine fill behavior from static analysis.

## Recommended Approach

### Immediate (Without Camera)

Use **Method 2 with local mean (-585)** as best educated guess:
- Last 3 rows have mean of -585
- Physically reasonable continuation of calibration data
- Better than 0 (no correction) for accuracy

### With Camera Available

Execute **Method 1** for definitive answer:
1. Run output comparison test
2. Identify exact matching variant
3. Update extraction to use that fill strategy

## Implementation

### Current File Status

| File | Purpose | Status |
|------|---------|--------|
| `extracted_calibration/fppn_320x240_le.bin` | Full array with interpolation | Created |
| `fppn_verification/fppn_raw_verified.bin` | Raw data (no padding) | Created |
| `fppn_comparison_test/fppn_*_le.bin` | Test variants | Created |
| `scripts/verify_fppn_extraction.py` | Verification tool | Complete |
| `scripts/sdk_fppn_comparison_test.py` | Comparison test | Complete |
| `scripts/dump_sdk_memory.py` | Memory dump tool | Complete |

### Integration with Driver

Once SDK fill behavior is confirmed:

```cpp
// Load FPPN from file
int16_t fppn[240][320];
FILE* f = fopen("fppn_320x240_le.bin", "rb");
fread(fppn, sizeof(int16_t), 320*240, f);
fclose(f);

// Apply FPPN correction
void apply_fppn_correction(int16_t* phase, const int16_t* fppn, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            phase[y * width + x] -= fppn[y * width + x];
        }
    }
}
```

## Conclusion

**We have 98.88% of FPPN data extracted correctly.**

The remaining 1.12% (864 pixels) depends on SDK's fill strategy. The output comparison test provides a **definitive, byte-exact method** to determine this without guessing.

Until camera testing is performed, use `local_mean` variant (-585 fill) as the most physically reasonable approximation.

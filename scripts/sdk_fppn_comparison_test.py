#!/usr/bin/env python3
"""
SDK FPPN Behavior Determination Test

This test definitively determines how SDK handles missing FPPN pixels by:
1. Loading our extracted FPPN data
2. Creating variants with different fill strategies
3. Processing raw frames with each variant
4. Comparing against SDK output to find exact match

REQUIREMENT: Need synchronized raw frame + SDK depth output from same capture.
"""

import os
import struct
import numpy as np
from pathlib import Path

# Directories
DRIVER_DIR = Path("/home/cmericli/development/atlas/code/cubeeye_nano_driver")
FPPN_VERIFIED = DRIVER_DIR / "fppn_verification" / "fppn_raw_verified.bin"
OUTPUT_DIR = DRIVER_DIR / "fppn_comparison_test"

# Constants
WIDTH = 320
HEIGHT = 240
TOTAL_PIXELS = WIDTH * HEIGHT  # 76800


def load_raw_fppn():
    """Load our verified extracted FPPN (no padding)."""
    with open(FPPN_VERIFIED, "rb") as f:
        data = f.read()

    # Stored as big-endian
    num_values = len(data) // 2
    values = list(struct.unpack(f'>{num_values}h', data))

    print(f"Loaded {len(values)} raw FPPN values")
    print(f"  Min: {min(values)}, Max: {max(values)}, Mean: {np.mean(values):.1f}")

    return values


def create_fppn_variants(raw_values):
    """Create FPPN arrays with different fill strategies for missing pixels."""
    missing_count = TOTAL_PIXELS - len(raw_values)
    print(f"\nCreating variants to fill {missing_count} missing pixels...")

    variants = {}

    # Option 1: Fill with zeros
    fppn_zeros = np.zeros(TOTAL_PIXELS, dtype=np.int16)
    fppn_zeros[:len(raw_values)] = raw_values
    variants['zeros'] = fppn_zeros
    print(f"  zeros: Fill missing with 0")

    # Option 2: Fill with mean of valid data
    mean_val = int(np.mean(raw_values))
    fppn_mean = np.zeros(TOTAL_PIXELS, dtype=np.int16)
    fppn_mean[:len(raw_values)] = raw_values
    fppn_mean[len(raw_values):] = mean_val
    variants['mean'] = fppn_mean
    print(f"  mean: Fill missing with {mean_val}")

    # Option 3: Fill with last valid row repeated
    last_complete_row = len(raw_values) // WIDTH - 1
    if last_complete_row >= 0:
        last_row = raw_values[last_complete_row * WIDTH:(last_complete_row + 1) * WIDTH]
        fppn_repeat = np.zeros(TOTAL_PIXELS, dtype=np.int16)
        fppn_repeat[:len(raw_values)] = raw_values
        # Fill remaining rows by repeating last complete row
        for row in range(last_complete_row + 1, HEIGHT):
            for col in range(WIDTH):
                idx = row * WIDTH + col
                if idx >= len(raw_values):
                    fppn_repeat[idx] = last_row[col]
        variants['repeat_last_row'] = fppn_repeat
        print(f"  repeat_last_row: Repeat row {last_complete_row} pattern")

    # Option 4: Fill with mean of last 3 complete rows
    if last_complete_row >= 2:
        last_rows_mean = np.mean(raw_values[(last_complete_row-2)*WIDTH:(last_complete_row+1)*WIDTH])
        fppn_local_mean = np.zeros(TOTAL_PIXELS, dtype=np.int16)
        fppn_local_mean[:len(raw_values)] = raw_values
        fppn_local_mean[len(raw_values):] = int(last_rows_mean)
        variants['local_mean'] = fppn_local_mean
        print(f"  local_mean: Fill with mean of rows {last_complete_row-2}-{last_complete_row} = {last_rows_mean:.0f}")

    # Option 5: Linear extrapolation
    if last_complete_row >= 4:
        # Get mean of last few rows to estimate trend
        row_means = []
        for r in range(max(0, last_complete_row-4), last_complete_row+1):
            row_data = raw_values[r*WIDTH:(r+1)*WIDTH]
            row_means.append(np.mean(row_data))

        # Simple linear fit
        x = np.arange(len(row_means))
        slope, intercept = np.polyfit(x, row_means, 1)

        fppn_extrap = np.zeros(TOTAL_PIXELS, dtype=np.int16)
        fppn_extrap[:len(raw_values)] = raw_values
        for row in range(last_complete_row + 1, HEIGHT):
            extrap_mean = intercept + slope * (row - (last_complete_row - len(row_means) + 1))
            for col in range(WIDTH):
                idx = row * WIDTH + col
                if idx >= len(raw_values):
                    fppn_extrap[idx] = int(extrap_mean)
        variants['extrapolate'] = fppn_extrap
        print(f"  extrapolate: Linear extrapolation (slope={slope:.2f})")

    # Option 6: Fill with -672 (observed typical FPPN value)
    typical_val = -672
    fppn_typical = np.zeros(TOTAL_PIXELS, dtype=np.int16)
    fppn_typical[:len(raw_values)] = raw_values
    fppn_typical[len(raw_values):] = typical_val
    variants['typical_-672'] = fppn_typical
    print(f"  typical_-672: Fill with common value -672")

    return variants


def save_variants(variants):
    """Save all variants for testing."""
    OUTPUT_DIR.mkdir(exist_ok=True)

    for name, fppn in variants.items():
        # Reshape to 2D
        fppn_2d = fppn.reshape((HEIGHT, WIDTH))

        # Save as little-endian (for our driver)
        le_file = OUTPUT_DIR / f"fppn_{name}_le.bin"
        with open(le_file, "wb") as f:
            f.write(fppn_2d.astype('<i2').tobytes())

        # Save as big-endian (SDK format reference)
        be_file = OUTPUT_DIR / f"fppn_{name}_be.bin"
        with open(be_file, "wb") as f:
            f.write(fppn_2d.astype('>i2').tobytes())

        # Save as numpy for easy analysis
        np.save(OUTPUT_DIR / f"fppn_{name}.npy", fppn_2d)

        print(f"  Saved: {le_file.name}")


def create_comparison_script():
    """Create script to compare our depth output vs SDK."""
    script = OUTPUT_DIR / "compare_depth.py"
    with open(script, "w") as f:
        f.write('''#!/usr/bin/env python3
"""
Compare depth output from our pipeline vs SDK.

Usage: python3 compare_depth.py <our_depth.raw> <sdk_depth.raw> <fppn_variant>

This will:
1. Load both depth images
2. Calculate pixel-wise differences
3. Focus on bottom rows (237-239) where FPPN fill matters
4. Report statistics to determine which variant matches SDK
"""

import sys
import struct
import numpy as np

WIDTH = 320
HEIGHT = 240

def load_depth(filename):
    with open(filename, "rb") as f:
        data = f.read()
    return np.frombuffer(data, dtype=np.uint16).reshape((HEIGHT, WIDTH))

def compare(our_depth, sdk_depth, variant_name):
    diff = our_depth.astype(np.int32) - sdk_depth.astype(np.int32)

    print(f"\\nComparison for variant: {variant_name}")
    print(f"  Full image:")
    print(f"    Mean abs diff: {np.abs(diff).mean():.2f}")
    print(f"    Max abs diff: {np.abs(diff).max()}")
    print(f"    Exact matches: {np.sum(diff == 0)} / {WIDTH * HEIGHT}")

    # Focus on bottom rows where FPPN fill matters
    bottom_diff = diff[237:240, :]
    print(f"  Bottom rows (237-239):")
    print(f"    Mean abs diff: {np.abs(bottom_diff).mean():.2f}")
    print(f"    Max abs diff: {np.abs(bottom_diff).max()}")
    print(f"    Exact matches: {np.sum(bottom_diff == 0)} / {3 * WIDTH}")

    return np.abs(diff).sum()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: compare_depth.py <our_depth.raw> <sdk_depth.raw> [variant]")
        sys.exit(1)

    our_depth = load_depth(sys.argv[1])
    sdk_depth = load_depth(sys.argv[2])
    variant = sys.argv[3] if len(sys.argv) > 3 else "unknown"

    total_diff = compare(our_depth, sdk_depth, variant)
    print(f"\\nTotal absolute difference: {total_diff}")
''')
    print(f"\nComparison script saved: {script}")


def print_test_instructions():
    """Print instructions for running the comparison test."""
    print("\n" + "=" * 70)
    print("TEST PROCEDURE TO DETERMINE SDK FPPN BEHAVIOR")
    print("=" * 70)
    print("""
To definitively determine SDK's missing pixel handling:

STEP 1: Capture synchronized data
   # Start SDK capture with hook to get raw frames
   cd /home/cmericli/development/atlas/code/cubeeye_nano_driver
   LD_PRELOAD=./build/libv4l2_hook.so ./build/benchmark_capture 1

   This creates:
   - benchmark/sdk_depth_0000.raw (SDK processed depth)
   - V4L2 hook output with raw frames

STEP 2: Process raw frame with each FPPN variant
   For each variant (zeros, mean, repeat_last_row, etc.):
   - Apply FPPN correction using that variant
   - Run through depth pipeline
   - Save output

STEP 3: Compare each result with SDK output
   python3 fppn_comparison_test/compare_depth.py \\
       our_depth_zeros.raw sdk_depth_0000.raw zeros

   python3 fppn_comparison_test/compare_depth.py \\
       our_depth_mean.raw sdk_depth_0000.raw mean

   etc.

STEP 4: Identify exact match
   The variant with ZERO differences in bottom rows (237-239)
   is what the SDK uses.

ALTERNATIVE: Memory dump approach
   If camera is available, run:
   - Start SDK process
   - Attach with GDB: sudo gdb -p <pid>
   - Run: search-fppn (from dump_sdk_fppn.py)
   - Examine values at rows 237-239 directly
""")


def main():
    print("=" * 70)
    print("SDK FPPN FILL STRATEGY DETERMINATION")
    print("=" * 70)

    # Load raw FPPN
    if not FPPN_VERIFIED.exists():
        print(f"ERROR: FPPN data not found at {FPPN_VERIFIED}")
        print("Run verify_fppn_extraction.py first")
        return

    raw_values = load_raw_fppn()

    # Create variants
    variants = create_fppn_variants(raw_values)

    # Save variants
    print(f"\nSaving variants to {OUTPUT_DIR}")
    save_variants(variants)

    # Create comparison script
    create_comparison_script()

    # Print test instructions
    print_test_instructions()

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"""
Files created in {OUTPUT_DIR}:
  - fppn_zeros_le.bin        (fill missing with 0)
  - fppn_mean_le.bin         (fill with overall mean)
  - fppn_repeat_last_row_le.bin (repeat last complete row)
  - fppn_local_mean_le.bin   (fill with local mean)
  - fppn_extrapolate_le.bin  (linear extrapolation)
  - fppn_typical_-672_le.bin (fill with typical value)
  - compare_depth.py         (comparison script)

To determine exact SDK behavior:
  1. Capture raw + SDK depth simultaneously
  2. Test each variant against SDK output
  3. The matching variant IS what SDK does

This is DEFINITIVE - no guessing, byte-exact comparison.
""")


if __name__ == "__main__":
    main()

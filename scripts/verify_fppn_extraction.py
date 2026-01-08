#!/usr/bin/env python3
"""
FPPN Extraction Verification Tool

This tool provides DEFINITIVE proof of:
1. Which calibration pages contain FPPN data
2. Exactly which values are extracted from each page
3. Where gaps occur and why
4. What the SDK MUST do with missing data

NO GUESSING - only facts from actual data.
"""

import os
import struct
from pathlib import Path
from collections import defaultdict
import numpy as np

CAL_DIR = Path("/home/cmericli/development/atlas/code/cubeeye_nano_driver/cal_scan_output")
OUTPUT_DIR = Path("/home/cmericli/development/atlas/code/cubeeye_nano_driver/fppn_verification")

# Known FPPN parameters
WIDTH = 320
HEIGHT = 240
TOTAL_PIXELS = WIDTH * HEIGHT  # 76800

# FPPN page range (determined from SDK strings and analysis)
FPPN_START_PAGE = 0x021C  # From SDK behavior
FPPN_END_PAGE = 0x0716    # From SDK behavior


def read_page(page_num):
    """Read calibration page raw data."""
    filename = CAL_DIR / f"page_{page_num:04x}.bin"
    if not filename.exists():
        return None
    with open(filename, "rb") as f:
        return f.read()


def is_fppn_value(val):
    """Check if int16 value is in FPPN range."""
    return -950 < val < -450


def analyze_page(page_num, data):
    """Analyze a single page and extract FPPN-like values."""
    if not data or len(data) < 10:
        return None

    # Header format: 00 20 HH LL 01 00 [254 bytes data]
    header = data[:6]
    payload = data[6:]

    # Parse header
    page_in_header = (header[2] << 8) | header[3]

    # Parse payload as big-endian int16
    num_values = len(payload) // 2
    values = struct.unpack(f'>{num_values}h', payload[:num_values*2])

    # Count FPPN values
    fppn_count = sum(1 for v in values if is_fppn_value(v))
    fppn_values = [v for v in values if is_fppn_value(v)]

    return {
        'page': page_num,
        'page_in_header': page_in_header,
        'payload_size': len(payload),
        'num_values': num_values,
        'fppn_count': fppn_count,
        'fppn_values': fppn_values,
        'all_values': list(values),
        'header': header.hex(),
    }


def main():
    print("=" * 70)
    print("FPPN EXTRACTION VERIFICATION - DEFINITIVE ANALYSIS")
    print("=" * 70)
    print(f"\nSource: {CAL_DIR}")
    print(f"FPPN page range: 0x{FPPN_START_PAGE:04X} - 0x{FPPN_END_PAGE:04X}")
    print(f"Target: {WIDTH}x{HEIGHT} = {TOTAL_PIXELS} pixels")

    OUTPUT_DIR.mkdir(exist_ok=True)

    # Phase 1: Inventory all pages
    print("\n" + "=" * 70)
    print("PHASE 1: PAGE INVENTORY")
    print("=" * 70)

    all_pages = sorted([int(f.stem.split('_')[1], 16)
                        for f in CAL_DIR.glob("page_*.bin")])
    print(f"Total calibration pages available: {len(all_pages)}")

    fppn_range_pages = [p for p in all_pages
                        if FPPN_START_PAGE <= p <= FPPN_END_PAGE]
    print(f"Pages in FPPN range: {len(fppn_range_pages)}")

    # Check which expected pages are missing
    expected_pages = list(range(FPPN_START_PAGE, FPPN_END_PAGE + 1, 2))  # Even only
    missing_pages = [p for p in expected_pages if p not in fppn_range_pages]

    print(f"Expected pages (even): {len(expected_pages)}")
    print(f"Missing pages: {len(missing_pages)}")

    if missing_pages:
        print(f"\nMissing page numbers:")
        for i in range(0, len(missing_pages), 10):
            chunk = missing_pages[i:i+10]
            print(f"  {', '.join(f'0x{p:04X}' for p in chunk)}")

    # Phase 2: Analyze each FPPN page
    print("\n" + "=" * 70)
    print("PHASE 2: PAGE-BY-PAGE ANALYSIS")
    print("=" * 70)

    page_analyses = []
    total_fppn_values = 0
    fppn_raw = []  # All FPPN values in order

    for page_num in sorted(fppn_range_pages):
        data = read_page(page_num)
        analysis = analyze_page(page_num, data)

        if analysis and analysis['fppn_count'] > 0:
            page_analyses.append(analysis)
            total_fppn_values += analysis['fppn_count']

            # Add to raw array
            fppn_raw.extend(analysis['fppn_values'])

    print(f"Pages with FPPN data: {len(page_analyses)}")
    print(f"Total FPPN values extracted: {total_fppn_values}")
    print(f"Coverage: {100.0 * total_fppn_values / TOTAL_PIXELS:.2f}%")

    # Phase 3: Identify exact gaps
    print("\n" + "=" * 70)
    print("PHASE 3: GAP ANALYSIS")
    print("=" * 70)

    missing_values = TOTAL_PIXELS - len(fppn_raw)
    print(f"Missing FPPN values: {missing_values}")
    print(f"Missing rows: {missing_values / WIDTH:.2f}")

    if missing_values > 0:
        last_row_with_data = len(fppn_raw) // WIDTH
        last_col_with_data = len(fppn_raw) % WIDTH

        print(f"\nData coverage:")
        print(f"  Complete rows: {last_row_with_data} (0-{last_row_with_data-1})")
        if last_col_with_data > 0:
            print(f"  Partial row {last_row_with_data}: {last_col_with_data} columns")
        print(f"  Missing rows: {HEIGHT - last_row_with_data - (1 if last_col_with_data > 0 else 0)}")

    # Phase 4: Track where each pixel comes from
    print("\n" + "=" * 70)
    print("PHASE 4: PIXEL-TO-PAGE MAPPING")
    print("=" * 70)

    pixel_source = []  # (pixel_idx, page_num, value)
    pixel_idx = 0

    for analysis in page_analyses:
        for val in analysis['fppn_values']:
            if pixel_idx < TOTAL_PIXELS:
                pixel_source.append((pixel_idx, analysis['page'], val))
                pixel_idx += 1

    # Save detailed mapping
    mapping_file = OUTPUT_DIR / "pixel_to_page_mapping.csv"
    with open(mapping_file, "w") as f:
        f.write("pixel_idx,row,col,page_hex,value\n")
        for idx, page, val in pixel_source:
            row = idx // WIDTH
            col = idx % WIDTH
            f.write(f"{idx},{row},{col},0x{page:04X},{val}\n")

    print(f"Pixel-to-page mapping saved to: {mapping_file}")

    # Show first and last few pages
    print("\nFirst 5 FPPN pages:")
    for analysis in page_analyses[:5]:
        print(f"  0x{analysis['page']:04X}: {analysis['fppn_count']} values, "
              f"first={analysis['fppn_values'][0] if analysis['fppn_values'] else 'N/A'}")

    print("\nLast 5 FPPN pages:")
    for analysis in page_analyses[-5:]:
        print(f"  0x{analysis['page']:04X}: {analysis['fppn_count']} values, "
              f"last={analysis['fppn_values'][-1] if analysis['fppn_values'] else 'N/A'}")

    # Phase 5: Value statistics
    print("\n" + "=" * 70)
    print("PHASE 5: VALUE STATISTICS")
    print("=" * 70)

    fppn_array = np.array(fppn_raw, dtype=np.int16)

    print(f"Extracted values: {len(fppn_array)}")
    print(f"Min: {fppn_array.min()}")
    print(f"Max: {fppn_array.max()}")
    print(f"Mean: {fppn_array.mean():.2f}")
    print(f"Std: {fppn_array.std():.2f}")

    # Histogram of values
    print("\nValue distribution:")
    bins = [-900, -850, -800, -750, -700, -650, -600, -550, -500]
    hist, _ = np.histogram(fppn_array, bins=bins)
    for i, count in enumerate(hist):
        pct = 100.0 * count / len(fppn_array)
        bar = "#" * int(pct / 2)
        print(f"  [{bins[i]:4d} to {bins[i+1]:4d}]: {count:6d} ({pct:5.1f}%) {bar}")

    # Phase 6: Determine SDK behavior for missing data
    print("\n" + "=" * 70)
    print("PHASE 6: SDK MISSING DATA BEHAVIOR ANALYSIS")
    print("=" * 70)

    print("\nPOSSIBLE SDK BEHAVIORS for missing data:")
    print()

    # Option 1: Fill with zeros
    print("Option 1: Fill with zeros (0)")
    print("  - Would cause corrected_phase = raw_phase - 0 = raw_phase")
    print("  - Effect: No FPPN correction for bottom rows")
    print()

    # Option 2: Fill with mean
    mean_val = fppn_array.mean()
    print(f"Option 2: Fill with mean ({mean_val:.0f})")
    print(f"  - Would cause consistent correction across missing rows")
    print(f"  - Effect: Approximate FPPN correction")
    print()

    # Option 3: Copy last valid row
    if len(fppn_raw) >= WIDTH:
        last_row_start = (len(fppn_raw) // WIDTH - 1) * WIDTH
        last_row = fppn_raw[last_row_start:last_row_start + WIDTH]
        last_row_mean = np.mean(last_row)
        print(f"Option 3: Copy last valid row (mean {last_row_mean:.0f})")
        print(f"  - Would replicate row {len(fppn_raw) // WIDTH - 1} pattern")
        print()

    # Option 4: Extrapolate
    print("Option 4: Linear extrapolation from last rows")
    print("  - Would continue any trend in the data")
    print()

    print("=" * 70)
    print("CRITICAL QUESTION: How do we determine which option SDK uses?")
    print("=" * 70)
    print("""
DEFINITIVE METHODS to determine SDK behavior:

1. RUNTIME MEMORY DUMP (highest confidence)
   - Run SDK, dump FPPN array from memory
   - Compare bytes at positions 237-239 rows
   - This gives EXACT values SDK uses

2. OUTPUT COMPARISON (practical)
   - Capture raw frame + SDK depth output
   - Process raw frame with each FPPN option
   - Compare with SDK output to find match

3. DECOMPILATION (if available)
   - Find SDK's FPPN loading code
   - Analyze how it handles size < expected

RECOMMENDATION: Method 2 (Output Comparison) is most practical:
   - Doesn't require special permissions
   - Uses actual camera output
   - Provides byte-exact validation
""")

    # Save extracted FPPN (raw data only, no padding)
    raw_file = OUTPUT_DIR / "fppn_raw_verified.bin"
    with open(raw_file, "wb") as f:
        # Save as big-endian (SDK format)
        for val in fppn_raw:
            f.write(struct.pack('>h', val))
    print(f"\nRaw FPPN (no padding) saved to: {raw_file}")
    print(f"  Size: {len(fppn_raw) * 2} bytes ({len(fppn_raw)} values)")

    # Save verification report
    report_file = OUTPUT_DIR / "verification_report.txt"
    with open(report_file, "w") as f:
        f.write("FPPN EXTRACTION VERIFICATION REPORT\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"Source: {CAL_DIR}\n")
        f.write(f"Date: {os.popen('date').read().strip()}\n\n")
        f.write(f"RESULTS:\n")
        f.write(f"  Total pages analyzed: {len(page_analyses)}\n")
        f.write(f"  Total FPPN values: {len(fppn_raw)}\n")
        f.write(f"  Expected values: {TOTAL_PIXELS}\n")
        f.write(f"  Coverage: {100.0 * len(fppn_raw) / TOTAL_PIXELS:.2f}%\n")
        f.write(f"  Missing values: {TOTAL_PIXELS - len(fppn_raw)}\n\n")
        f.write(f"VALUE STATISTICS:\n")
        f.write(f"  Min: {fppn_array.min()}\n")
        f.write(f"  Max: {fppn_array.max()}\n")
        f.write(f"  Mean: {fppn_array.mean():.2f}\n")
        f.write(f"  Std: {fppn_array.std():.2f}\n\n")
        f.write(f"MISSING PAGES:\n")
        for p in missing_pages:
            f.write(f"  0x{p:04X}\n")
        f.write(f"\nCONCLUSION:\n")
        f.write(f"  Camera flash contains {100.0 * len(fppn_raw) / TOTAL_PIXELS:.1f}% of FPPN data.\n")
        f.write(f"  Bottom {HEIGHT - len(fppn_raw) // WIDTH} rows are missing.\n")
        f.write(f"  SDK MUST handle this gap - determine method via output comparison.\n")

    print(f"\nVerification report saved to: {report_file}")

    print("\n" + "=" * 70)
    print("VERIFICATION COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    main()

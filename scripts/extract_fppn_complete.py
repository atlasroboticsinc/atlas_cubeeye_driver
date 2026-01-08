#!/usr/bin/env python3
"""
Complete FPPN Extraction Tool

Extracts FPPN (Fixed Pattern Phase Noise) calibration data from CubeEye camera
calibration pages. Handles gaps and incomplete data.

FPPN is stored in camera flash pages 0x021C - 0x0716.
Format: Big-endian int16 values, typically in range -900 to -500.
"""

import os
import struct
import numpy as np
from pathlib import Path

CAL_DIR = Path("./cal_scan_output")
OUTPUT_DIR = Path("./extracted_calibration")

# FPPN constants
WIDTH = 320
HEIGHT = 240
TOTAL_PIXELS = WIDTH * HEIGHT  # 76800
BYTES_PER_PAGE = 254  # After 6-byte header

# FPPN page range (determined from analysis)
FPPN_START_PAGE = 0x021C
FPPN_END_PAGE = 0x0716


def read_page(page_num):
    """Read calibration page data (skipping 6-byte header)."""
    filename = CAL_DIR / f"page_{page_num:04x}.bin"
    if not filename.exists():
        return None
    with open(filename, "rb") as f:
        data = f.read()
    if len(data) < 10:
        return None
    # Skip 6-byte header: 00 20 HH LL 01 00
    return data[6:]


def is_fppn_value(val):
    """Check if value is in FPPN range."""
    return -950 < val < -450


def extract_fppn():
    """Extract FPPN data from calibration pages."""
    print("=" * 60)
    print("FPPN Extraction - Complete Analysis")
    print("=" * 60)

    # First pass: collect all FPPN pages
    fppn_pages = []
    total_values = 0

    for page in range(FPPN_START_PAGE, FPPN_END_PAGE + 2, 2):  # Even pages only
        data = read_page(page)
        if data is None:
            continue

        # Parse as big-endian int16
        num_vals = len(data) // 2
        vals = struct.unpack(f'>{num_vals}h', data[:num_vals*2])

        # Count FPPN-like values
        fppn_count = sum(1 for v in vals if is_fppn_value(v))

        if fppn_count > num_vals // 2:  # More than half are FPPN values
            fppn_pages.append({
                'page': page,
                'data': data,
                'values': vals,
                'fppn_count': fppn_count
            })
            total_values += fppn_count

    print(f"\nPhase 1: Page Analysis")
    print(f"  FPPN pages found: {len(fppn_pages)}")
    print(f"  First page: 0x{fppn_pages[0]['page']:04x}")
    print(f"  Last page: 0x{fppn_pages[-1]['page']:04x}")
    print(f"  FPPN values found: {total_values}")
    print(f"  Values needed: {TOTAL_PIXELS}")
    print(f"  Coverage: {100.0 * total_values / TOTAL_PIXELS:.1f}%")

    # Second pass: extract raw FPPN array
    fppn_raw = []

    for page_info in fppn_pages:
        vals = page_info['values']

        # Extract only FPPN values (skip markers and padding)
        for v in vals:
            if is_fppn_value(v):
                fppn_raw.append(v)
            elif len(fppn_raw) > 0 and len(fppn_raw) < TOTAL_PIXELS:
                # Once we start collecting, treat non-FPPN as end or gap
                # Could be a marker page or end of data
                pass

    print(f"\nPhase 2: Value Extraction")
    print(f"  Raw values collected: {len(fppn_raw)}")

    # Create full 320x240 array
    fppn_array = np.zeros((HEIGHT, WIDTH), dtype=np.int16)

    # Fill with extracted data
    for i, val in enumerate(fppn_raw[:TOTAL_PIXELS]):
        row = i // WIDTH
        col = i % WIDTH
        fppn_array[row, col] = val

    # Calculate statistics for filled portion
    filled_rows = min(len(fppn_raw) // WIDTH, HEIGHT)
    partial_col = len(fppn_raw) % WIDTH if len(fppn_raw) < TOTAL_PIXELS else 0

    print(f"  Complete rows filled: {filled_rows}")
    if partial_col > 0:
        print(f"  Partial row: {partial_col} pixels")

    # Handle missing data at the end
    missing_pixels = TOTAL_PIXELS - len(fppn_raw)
    if missing_pixels > 0:
        print(f"  Missing pixels: {missing_pixels}")

        # Fill missing with interpolation from last valid row
        if filled_rows > 0:
            last_valid_row = filled_rows - 1
            if partial_col > 0:
                last_valid_row = filled_rows

            # Get mean of last valid rows for interpolation
            if last_valid_row >= 2:
                interp_mean = fppn_array[last_valid_row-2:last_valid_row, :].mean()
            else:
                interp_mean = fppn_array[:last_valid_row+1, :].mean()

            print(f"  Interpolating missing rows with mean: {interp_mean:.1f}")

            # Fill remaining
            for i in range(len(fppn_raw), TOTAL_PIXELS):
                row = i // WIDTH
                col = i % WIDTH
                fppn_array[row, col] = int(interp_mean)

    # Final statistics
    print(f"\nPhase 3: Final FPPN Statistics")
    print(f"  Shape: {fppn_array.shape}")
    print(f"  Min: {fppn_array.min()}")
    print(f"  Max: {fppn_array.max()}")
    print(f"  Mean: {fppn_array.mean():.1f}")
    print(f"  Std: {fppn_array.std():.1f}")

    # Check for any anomalous rows
    print(f"\nRow statistics:")
    anomalous_rows = []
    for row in range(HEIGHT):
        row_mean = fppn_array[row, :].mean()
        row_std = fppn_array[row, :].std()
        if row_mean > -500 or row_mean < -900 or row_std > 100:
            anomalous_rows.append((row, row_mean, row_std))

    if anomalous_rows:
        print(f"  Anomalous rows (outside typical range):")
        for row, mean, std in anomalous_rows[:10]:
            print(f"    Row {row}: mean={mean:.1f}, std={std:.1f}")
    else:
        print(f"  All rows within expected range")

    return fppn_array, fppn_raw


def save_fppn(fppn_array, fppn_raw):
    """Save FPPN data in multiple formats."""
    OUTPUT_DIR.mkdir(exist_ok=True)

    # Save as little-endian binary (for our driver)
    fppn_le = fppn_array.astype('<i2')  # Little-endian
    with open(OUTPUT_DIR / "fppn_320x240_le.bin", "wb") as f:
        f.write(fppn_le.tobytes())

    # Save as big-endian binary (original SDK format)
    fppn_be = fppn_array.astype('>i2')  # Big-endian
    with open(OUTPUT_DIR / "fppn_320x240_be.bin", "wb") as f:
        f.write(fppn_be.tobytes())

    # Save as numpy array
    np.save(OUTPUT_DIR / "fppn_320x240.npy", fppn_array)

    # Save raw data (before padding)
    raw_array = np.array(fppn_raw, dtype='>i2')
    with open(OUTPUT_DIR / "fppn_raw.bin", "wb") as f:
        f.write(raw_array.tobytes())
    np.save(OUTPUT_DIR / "fppn_raw.npy", raw_array)

    # Save metadata
    with open(OUTPUT_DIR / "fppn_metadata.txt", "w") as f:
        f.write(f"CubeEye I200D FPPN Calibration Data\n")
        f.write(f"=" * 50 + "\n\n")
        f.write(f"Source: Camera flash via UVC XU selector 4\n")
        f.write(f"Pages: 0x{FPPN_START_PAGE:04X} - 0x{FPPN_END_PAGE:04X}\n\n")
        f.write(f"Format:\n")
        f.write(f"  Shape: {WIDTH}x{HEIGHT} (width x height)\n")
        f.write(f"  Type: int16 (signed 16-bit)\n")
        f.write(f"  Byte order: Little-endian in _le.bin, Big-endian in _be.bin\n\n")
        f.write(f"Statistics:\n")
        f.write(f"  Raw values extracted: {len(fppn_raw)}\n")
        f.write(f"  Total pixels: {TOTAL_PIXELS}\n")
        f.write(f"  Coverage: {100.0 * len(fppn_raw) / TOTAL_PIXELS:.1f}%\n")
        f.write(f"  Min: {fppn_array.min()}\n")
        f.write(f"  Max: {fppn_array.max()}\n")
        f.write(f"  Mean: {fppn_array.mean():.1f}\n\n")
        f.write(f"Usage:\n")
        f.write(f"  corrected_phase = raw_phase - fppn[y, x]\n\n")
        f.write(f"Files:\n")
        f.write(f"  fppn_320x240_le.bin - Full 320x240, little-endian int16\n")
        f.write(f"  fppn_320x240_be.bin - Full 320x240, big-endian int16\n")
        f.write(f"  fppn_320x240.npy - NumPy array format\n")
        f.write(f"  fppn_raw.bin - Raw extracted data (before padding)\n")
        f.write(f"  fppn_raw.npy - Raw data as NumPy array\n")

    print(f"\nFiles saved to {OUTPUT_DIR}:")
    for f in OUTPUT_DIR.glob("fppn_*"):
        print(f"  {f.name}: {f.stat().st_size} bytes")


def visualize_fppn(fppn_array):
    """Create visualization of FPPN data."""
    try:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # Main FPPN image
        im = axes[0, 0].imshow(fppn_array, cmap='viridis', aspect='auto')
        axes[0, 0].set_title('FPPN Correction Map')
        axes[0, 0].set_xlabel('X (pixels)')
        axes[0, 0].set_ylabel('Y (pixels)')
        plt.colorbar(im, ax=axes[0, 0], label='FPPN value')

        # Histogram
        axes[0, 1].hist(fppn_array.flatten(), bins=100, edgecolor='black')
        axes[0, 1].set_title('FPPN Value Distribution')
        axes[0, 1].set_xlabel('FPPN value')
        axes[0, 1].set_ylabel('Count')

        # Row means
        row_means = fppn_array.mean(axis=1)
        axes[1, 0].plot(row_means)
        axes[1, 0].set_title('Row-wise Mean FPPN')
        axes[1, 0].set_xlabel('Row')
        axes[1, 0].set_ylabel('Mean FPPN')
        axes[1, 0].grid(True)

        # Column means
        col_means = fppn_array.mean(axis=0)
        axes[1, 1].plot(col_means)
        axes[1, 1].set_title('Column-wise Mean FPPN')
        axes[1, 1].set_xlabel('Column')
        axes[1, 1].set_ylabel('Mean FPPN')
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.savefig(OUTPUT_DIR / 'fppn_visualization.png', dpi=150)
        print(f"\nVisualization saved to {OUTPUT_DIR / 'fppn_visualization.png'}")

    except ImportError:
        print("\nMatplotlib not available, skipping visualization")


if __name__ == "__main__":
    os.chdir(Path(__file__).parent.parent)  # Change to project root

    fppn_array, fppn_raw = extract_fppn()
    save_fppn(fppn_array, fppn_raw)
    visualize_fppn(fppn_array)

    print("\n" + "=" * 60)
    print("FPPN Extraction Complete")
    print("=" * 60)

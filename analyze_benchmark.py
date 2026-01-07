#!/usr/bin/env python3
"""
analyze_benchmark.py - Compare SDK depth output with raw V4L2 frames

Analyzes benchmark captures to understand the relationship between
raw sensor data and SDK-processed depth output.
"""

import numpy as np
import csv
import os
import sys
from pathlib import Path

# Frame dimensions
RAW_WIDTH = 1600
RAW_HEIGHT = 241
SPATIAL_WIDTH = 320  # 1600 / 5 sub-pixels
SPATIAL_HEIGHT = 240  # 241 - 1 header row

SDK_WIDTH = 640
SDK_HEIGHT = 480

# ToF constants
C_LIGHT = 299792458.0  # m/s
FREQ_80MHZ = 80e6
FREQ_100MHZ = 100e6
RANGE_80MHZ = C_LIGHT / (2 * FREQ_80MHZ) * 1000  # mm = 1874.8mm
RANGE_100MHZ = C_LIGHT / (2 * FREQ_100MHZ) * 1000  # mm = 1499.0mm

def load_raw_frame(path):
    """Load raw V4L2 frame (1600x241 uint16)"""
    data = np.fromfile(path, dtype='<u2')
    return data.reshape(RAW_HEIGHT, RAW_WIDTH)

def load_sdk_depth(path):
    """Load SDK depth frame (640x480 uint16)"""
    data = np.fromfile(path, dtype='<u2')
    return data.reshape(SDK_HEIGHT, SDK_WIDTH)

def analyze_raw_frame(frame, frame_num):
    """Analyze a raw frame's sub-pixel structure"""
    # Skip header row
    pixel_data = frame[1:, :]

    # Analyze sub-pixel channels
    print(f"\n=== Raw Frame {frame_num} Analysis ===")
    print(f"Shape: {frame.shape}")

    # Header info
    print(f"\nHeader (first 16 uint16): {frame[0, :16]}")

    # Analyze each sub-pixel channel
    print(f"\nSub-pixel statistics (full frame):")
    for sub in range(5):
        sub_data = pixel_data[:, sub::5]
        nz = np.count_nonzero(sub_data)
        total = sub_data.size
        print(f"  Sub[{sub}]: mean={sub_data.mean():8.1f}, std={sub_data.std():7.1f}, "
              f"min={sub_data.min():5}, max={sub_data.max():5}, nonzero={100*nz/total:.1f}%")

    # Center pixel analysis
    center_row = SPATIAL_HEIGHT // 2  # 120
    center_col = SPATIAL_WIDTH // 2   # 160
    row_data = pixel_data[center_row, :]
    center_vals = row_data[center_col * 5 : center_col * 5 + 5]
    print(f"\nCenter pixel (row={center_row}, col={center_col}):")
    print(f"  Sub values: {center_vals}")

    return {
        'sub_means': [pixel_data[:, s::5].mean() for s in range(5)],
        'center_vals': center_vals
    }

def try_depth_calculation(raw_frame, sdk_depth_center):
    """Try various depth calculation methods to match SDK"""
    pixel_data = raw_frame[1:, :]
    center_row = SPATIAL_HEIGHT // 2
    center_col = SPATIAL_WIDTH // 2

    # Extract sub-pixels for center pixel
    base = center_col * 5
    subs = pixel_data[center_row, base:base+5].astype(np.float64)

    print(f"\n=== Depth Calculation Attempts ===")
    print(f"SDK depth (target): {sdk_depth_center} mm")
    print(f"Sub-pixel values: {subs}")

    # Method 1: Direct phase interpretation (100MHz)
    # If sub[2] is phase for 100MHz, scaled to 12-bit
    if subs[2] > 0:
        phase_100 = subs[2] / 65535.0 * 2 * np.pi
        depth_100 = phase_100 / (2 * np.pi) * RANGE_100MHZ
        print(f"\n  Method 1a (Sub[2] as 100MHz phase):")
        print(f"    phase={np.degrees(phase_100):.1f}°, depth={depth_100:.1f}mm")

    # Method 2: I/Q interpretation
    # If Sub[0,1] are I,Q for one freq and Sub[2,3] for another
    if subs[0] > 0 or subs[1] > 0:
        I1, Q1 = subs[0], subs[1]
        phase1 = np.arctan2(Q1, I1) if (I1 != 0 or Q1 != 0) else 0
        if phase1 < 0: phase1 += 2 * np.pi
        depth_iq1 = phase1 / (2 * np.pi) * RANGE_80MHZ
        print(f"\n  Method 2a (Sub[0,1] as 80MHz I/Q):")
        print(f"    I={I1}, Q={Q1}, phase={np.degrees(phase1):.1f}°, depth={depth_iq1:.1f}mm")

    if subs[2] > 0 or subs[3] > 0:
        I2, Q2 = subs[2], subs[3]
        phase2 = np.arctan2(Q2, I2) if (I2 != 0 or Q2 != 0) else 0
        if phase2 < 0: phase2 += 2 * np.pi
        depth_iq2 = phase2 / (2 * np.pi) * RANGE_100MHZ
        print(f"\n  Method 2b (Sub[2,3] as 100MHz I/Q):")
        print(f"    I={I2}, Q={Q2}, phase={np.degrees(phase2):.1f}°, depth={depth_iq2:.1f}mm")

    # Method 3: Direct scaling (sub[2] might be depth directly)
    # Check if any sub-pixel is close to SDK depth
    for i, s in enumerate(subs):
        if s > 0:
            # Check various scalings
            for scale in [1.0, 0.1, 0.01, 16.0, 0.0625]:
                calc_depth = s * scale
                if abs(calc_depth - sdk_depth_center) < 50:
                    print(f"\n  Method 3: Sub[{i}] * {scale:.4f} = {calc_depth:.1f}mm (error: {calc_depth - sdk_depth_center:.1f}mm)")

def main():
    benchmark_dir = sys.argv[1] if len(sys.argv) > 1 else "benchmark"

    print("=" * 60)
    print("CubeEye Benchmark Analysis")
    print("=" * 60)
    print(f"Directory: {benchmark_dir}")

    # Load stats CSV
    stats_file = Path(benchmark_dir) / "benchmark_stats.csv"
    stats = []
    if stats_file.exists():
        with open(stats_file) as f:
            reader = csv.DictReader(f)
            stats = list(reader)
        depths = [float(s['depth_mm']) for s in stats]
        print(f"\nSDK Statistics:")
        print(f"  Frames: {len(stats)}")
        print(f"  Depth: mean={np.mean(depths):.2f}mm, std={np.std(depths):.2f}mm")
        print(f"  Range: {min(depths):.0f} - {max(depths):.0f}mm")
    else:
        print("\nNo stats file found")

    # Find raw frames (skip first 4 warmup frames)
    raw_files = sorted(Path(benchmark_dir).glob("raw_*.raw"))
    print(f"\nRaw frames found: {len(raw_files)}")

    # Analyze first valid raw frame (frame 4 or later)
    for raw_file in raw_files:
        frame_num = int(raw_file.stem.split('_')[1])
        if frame_num < 4:  # Skip warmup frames
            continue

        print(f"\n{'='*60}")
        print(f"Analyzing {raw_file.name}")
        print(f"{'='*60}")

        raw_frame = load_raw_frame(raw_file)
        analysis = analyze_raw_frame(raw_frame, frame_num)

        # Try to match with corresponding SDK frame
        # Raw frame 4 corresponds to SDK frame 0
        sdk_frame_num = frame_num - 4
        if stats and sdk_frame_num < len(stats):
            sdk_depth = float(stats[sdk_frame_num]['depth_mm'])
            try_depth_calculation(raw_frame, sdk_depth)

        # Only analyze first few valid frames
        if frame_num >= 6:
            break

    print(f"\n{'='*60}")
    print("Analysis complete")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()

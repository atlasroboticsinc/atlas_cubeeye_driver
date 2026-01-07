#!/usr/bin/env python3
"""
analyze_subpixels.py - Deep analysis of sub-pixel patterns in raw frames

Goal: Understand if the 5 sub-pixels contain phase information that can be
used for proper ToF depth calculation.
"""

import numpy as np
import os
from pathlib import Path

os.chdir(Path(__file__).parent)

def load_frame(filename):
    """Load raw frame and return 240x320x5 sub-pixel array"""
    data = np.fromfile(filename, dtype=np.uint16).reshape(241, 1600)
    return data[1:, :].reshape(240, 320, 5)

def analyze_bit_patterns(pixels, label):
    """Analyze bit patterns to understand encoding"""
    print(f"\n{'='*60}")
    print(f"  BIT PATTERN ANALYSIS: {label}")
    print(f"{'='*60}")

    for ch in range(5):
        vals = pixels[:, :, ch].flatten()

        # Check for quantization patterns
        mod_256 = np.sum(vals % 256 == 0) / len(vals) * 100
        mod_4 = np.sum(vals % 4 == 0) / len(vals) * 100
        mod_16 = np.sum(vals % 16 == 0) / len(vals) * 100

        # Check bit usage
        all_bits = np.bitwise_or.reduce(vals)
        used_bits = bin(all_bits).count('1')

        print(f"\nSub[{ch}]:")
        print(f"  Range: {vals.min():5d} - {vals.max():5d}")
        print(f"  Mod 256: {mod_256:.1f}%  Mod 16: {mod_16:.1f}%  Mod 4: {mod_4:.1f}%")
        print(f"  Used bits: {used_bits}/16  (mask: 0x{all_bits:04x})")

        # Check if values look like intensity or phase
        if vals.max() > 0:
            # For phase data, we expect values centered around mid-range
            # For intensity, values should be positive-biased
            center_ratio = np.mean(vals) / (vals.max() / 2)
            print(f"  Center ratio: {center_ratio:.3f} (1.0 = phase-like, <1 = intensity-like)")

def analyze_spatial_patterns(pixels, label):
    """Look for spatial patterns that might indicate multiple taps"""
    print(f"\n{'='*60}")
    print(f"  SPATIAL PATTERN ANALYSIS: {label}")
    print(f"{'='*60}")

    # Check even vs odd column differences
    for ch in range(5):
        even_cols = pixels[:, 0::2, ch].mean()
        odd_cols = pixels[:, 1::2, ch].mean()
        col_diff = even_cols - odd_cols

        even_rows = pixels[0::2, :, ch].mean()
        odd_rows = pixels[1::2, :, ch].mean()
        row_diff = even_rows - odd_rows

        print(f"Sub[{ch}]: Even-Odd col diff: {col_diff:.2f}, row diff: {row_diff:.2f}")

def analyze_phase_relationships(pixels, label):
    """Check if sub-pixels have phase relationships"""
    print(f"\n{'='*60}")
    print(f"  PHASE RELATIONSHIP ANALYSIS: {label}")
    print(f"{'='*60}")

    # In 4-phase ToF:
    # I = Tap0 - Tap2 (0° - 180°)
    # Q = Tap1 - Tap3 (90° - 270°)
    # Phase = atan2(Q, I)
    # Amplitude = sqrt(I² + Q²)

    # Try different sub-pixel assignments
    assignments = [
        ("Sub[0]-Sub[2], Sub[1]-Sub[3]", 0, 2, 1, 3),
        ("Sub[1]-Sub[3], Sub[0]-Sub[2]", 1, 3, 0, 2),
        ("Sub[0]-Sub[1], Sub[2]-Sub[3]", 0, 1, 2, 3),
        ("Sub[2]-Sub[0], Sub[3]-Sub[1]", 2, 0, 3, 1),
    ]

    for name, t0, t2, t1, t3 in assignments:
        I = pixels[:, :, t0].astype(np.float64) - pixels[:, :, t2].astype(np.float64)
        Q = pixels[:, :, t1].astype(np.float64) - pixels[:, :, t3].astype(np.float64)

        phase = np.arctan2(Q, I)
        amplitude = np.sqrt(I*I + Q*Q)

        # Valid pixels have good amplitude
        amp_thresh = np.mean(amplitude[amplitude > 0]) * 0.3
        valid = amplitude > amp_thresh

        if np.sum(valid) > 1000:
            phase_range = np.ptp(phase[valid])
            phase_std = np.std(phase[valid])
            print(f"\n{name}:")
            print(f"  I range: [{I[valid].min():.0f}, {I[valid].max():.0f}]")
            print(f"  Q range: [{Q[valid].min():.0f}, {Q[valid].max():.0f}]")
            print(f"  Phase range: {np.degrees(phase_range):.1f}° (std: {np.degrees(phase_std):.1f}°)")
            print(f"  Valid pixels: {np.sum(valid)}")

def visualize_frame(pixels, label):
    """Print a text visualization of sub-pixel values at center ROI"""
    print(f"\n{'='*60}")
    print(f"  CENTER ROI VALUES: {label}")
    print(f"{'='*60}")

    # 4x4 ROI at center
    cy, cx = 120, 160
    roi = pixels[cy-2:cy+2, cx-2:cx+2, :]

    print(f"\n4x4 ROI at ({cy},{cx}):")
    for y in range(4):
        for ch in range(5):
            if ch == 0:
                print(f"  Row {y}: ", end="")
            print(f"[", end="")
            for x in range(4):
                print(f"{roi[y,x,ch]:5d}", end=" ")
            print("]  ", end="")
        print()

def main():
    # Find available raw files
    raw_files = []
    for f in ["ruler_1.0m.raw", "ruler_0.5m.raw", "ruler_1.5m.raw"]:
        if os.path.exists(f):
            raw_files.append(f)

    if not raw_files:
        print("No ruler raw files found!")
        return

    for f in raw_files:
        print(f"\n{'#'*60}")
        print(f"  FILE: {f}")
        print(f"{'#'*60}")

        pixels = load_frame(f)

        analyze_bit_patterns(pixels, f)
        analyze_spatial_patterns(pixels, f)
        analyze_phase_relationships(pixels, f)
        visualize_frame(pixels, f)

if __name__ == "__main__":
    main()

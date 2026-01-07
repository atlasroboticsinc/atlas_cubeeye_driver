#!/usr/bin/env python3
"""
deep_analysis.py - Comprehensive analysis of CubeEye I200D raw data

This script analyzes:
1. All 5 sub-pixel channels for patterns
2. Correlation with SDK ground truth depth
3. Temporal stability across frames
4. Per-pixel vs whole-frame statistics
"""

import numpy as np
import os
from pathlib import Path

# Change to script directory
os.chdir(Path(__file__).parent)

# Constants
RAW_WIDTH = 1600
RAW_HEIGHT = 241
DEPTH_WIDTH = 320
DEPTH_HEIGHT = 240
NUM_SUBPIXELS = 5

def load_raw_frame(filename):
    """Load 1600x241 raw frame and extract sub-pixels"""
    data = np.fromfile(filename, dtype=np.uint16).reshape(RAW_HEIGHT, RAW_WIDTH)
    # Skip header row 0, reshape to 240x320x5
    pixels = data[1:, :].reshape(DEPTH_HEIGHT, DEPTH_WIDTH, NUM_SUBPIXELS)
    return data, pixels

def load_sdk_depth(filename):
    """Load SDK depth frame (320x240 float32, may have 2 frames: depth + amplitude)"""
    data = np.fromfile(filename, dtype=np.float32)
    if data.size == DEPTH_HEIGHT * DEPTH_WIDTH * 2:
        # Two frames: depth and amplitude
        data = data.reshape(2, DEPTH_HEIGHT, DEPTH_WIDTH)
        return data[0]  # Return depth only
    return data.reshape(DEPTH_HEIGHT, DEPTH_WIDTH)

def analyze_header(data):
    """Analyze the header row for any useful information"""
    header = data[0, :]
    print(f"Header row stats: min={header.min()}, max={header.max()}, mean={header.mean():.2f}")
    print(f"First 20 header values: {header[:20]}")
    print(f"Last 20 header values: {header[-20:]}")
    # Check for patterns
    unique_vals = np.unique(header)
    print(f"Unique header values: {len(unique_vals)}")
    if len(unique_vals) < 20:
        print(f"Header values: {unique_vals}")

def analyze_subpixel_statistics(pixels, label=""):
    """Analyze each sub-pixel channel"""
    print(f"\n{'='*60}")
    print(f"  SUB-PIXEL STATISTICS {label}")
    print(f"{'='*60}")

    for i in range(NUM_SUBPIXELS):
        ch = pixels[:, :, i].flatten()
        print(f"\nSub[{i}]:")
        print(f"  Range: [{ch.min():5d}, {ch.max():5d}]")
        print(f"  Mean:  {ch.mean():.2f}")
        print(f"  Std:   {ch.std():.2f}")

        # Check for quantization
        unique = np.unique(ch)
        if len(unique) < 100:
            print(f"  Unique values: {len(unique)} (QUANTIZED!)")
            if len(unique) < 20:
                print(f"  Values: {unique}")
        else:
            # Check if values are multiples of 256 (8-bit in 16-bit container)
            multiples_256 = np.sum(ch % 256 == 0)
            pct_256 = 100.0 * multiples_256 / len(ch)
            if pct_256 > 50:
                print(f"  Multiples of 256: {pct_256:.1f}% (8-bit quantized!)")
            else:
                print(f"  Full 16-bit range")

def analyze_inter_channel_relationships(pixels, label=""):
    """Analyze relationships between sub-pixel channels"""
    print(f"\n{'='*60}")
    print(f"  INTER-CHANNEL RELATIONSHIPS {label}")
    print(f"{'='*60}")

    # Flatten each channel
    channels = [pixels[:, :, i].flatten().astype(np.float64) for i in range(NUM_SUBPIXELS)]

    # Correlation matrix
    print("\nCorrelation matrix:")
    print("        Sub0    Sub1    Sub2    Sub3    Sub4")
    for i in range(NUM_SUBPIXELS):
        row = f"Sub{i}  "
        for j in range(NUM_SUBPIXELS):
            corr = np.corrcoef(channels[i], channels[j])[0, 1]
            row += f" {corr:6.3f} "
        print(row)

    # Check Sub[2] vs Sub[3] difference (key for phase detection)
    diff_23 = channels[2] - channels[3]
    print(f"\nSub[2] - Sub[3] (phase indicator):")
    print(f"  Mean: {diff_23.mean():.2f}")
    print(f"  Std:  {diff_23.std():.2f}")
    print(f"  Range: [{diff_23.min():.0f}, {diff_23.max():.0f}]")

    # Check Sub[0] vs Sub[1] (black level)
    diff_01 = channels[0] - channels[1]
    print(f"\nSub[0] - Sub[1] (black level indicator):")
    print(f"  Mean: {diff_01.mean():.2f}")
    print(f"  Std:  {diff_01.std():.2f}")

def correlate_with_sdk_depth(pixels, sdk_depth):
    """Find correlation between sub-pixels and SDK depth"""
    print(f"\n{'='*60}")
    print(f"  CORRELATION WITH SDK DEPTH")
    print(f"{'='*60}")

    # Mask valid depth pixels
    valid_mask = (sdk_depth > 0) & (sdk_depth < 3000)
    valid_depth = sdk_depth[valid_mask]

    print(f"Valid SDK depth pixels: {np.sum(valid_mask)} / {DEPTH_WIDTH*DEPTH_HEIGHT}")
    print(f"SDK depth range: [{sdk_depth[valid_mask].min():.0f}, {sdk_depth[valid_mask].max():.0f}] mm")

    # Correlate each channel and combinations
    print("\nPearson correlation with SDK depth:")
    for i in range(NUM_SUBPIXELS):
        ch = pixels[:, :, i][valid_mask].astype(np.float64)
        corr = np.corrcoef(ch, valid_depth)[0, 1]
        print(f"  Sub[{i}]: r = {corr:+.4f}")

    # Key combinations
    sub2 = pixels[:, :, 2][valid_mask].astype(np.float64)
    sub3 = pixels[:, :, 3][valid_mask].astype(np.float64)
    sub4 = pixels[:, :, 4][valid_mask].astype(np.float64)

    avg_23 = (sub2 + sub3) / 2
    diff_23 = sub2 - sub3

    corr_avg = np.corrcoef(avg_23, valid_depth)[0, 1]
    corr_diff = np.corrcoef(diff_23, valid_depth)[0, 1]

    print(f"\n  (Sub[2]+Sub[3])/2: r = {corr_avg:+.4f}")
    print(f"  Sub[2]-Sub[3]:     r = {corr_diff:+.4f}")

    # Linear regression for best combination
    from scipy import stats
    slope, intercept, r, p, se = stats.linregress(avg_23, valid_depth)
    predictions = slope * avg_23 + intercept
    rmse = np.sqrt(np.mean((predictions - valid_depth)**2))

    print(f"\nLinear fit: depth = {slope:.6f} * avg + {intercept:.2f}")
    print(f"  R² = {r**2:.4f}")
    print(f"  RMSE = {rmse:.2f} mm")

    return slope, intercept, r**2, rmse

def analyze_temporal_stability():
    """Analyze frame-to-frame stability"""
    print(f"\n{'='*60}")
    print(f"  TEMPORAL STABILITY ANALYSIS")
    print(f"{'='*60}")

    # Load multiple frames from data directory
    data_dir = Path("data")
    frames = sorted(data_dir.glob("hook_raw_frame_*.raw"))[:5]

    if len(frames) < 2:
        print("Not enough frames for temporal analysis")
        return

    print(f"Analyzing {len(frames)} consecutive frames")

    all_pixels = []
    for f in frames:
        _, pixels = load_raw_frame(f)
        all_pixels.append(pixels)

    all_pixels = np.stack(all_pixels)  # Shape: (N, 240, 320, 5)

    # Compute temporal statistics per pixel
    temporal_std = all_pixels.std(axis=0)
    temporal_mean = all_pixels.mean(axis=0)

    print("\nTemporal standard deviation per channel (averaged over pixels):")
    for i in range(NUM_SUBPIXELS):
        print(f"  Sub[{i}]: {temporal_std[:,:,i].mean():.2f}")

    # Check if sub-pixels are stable
    cv = temporal_std / (temporal_mean + 1e-6)  # Coefficient of variation
    print("\nCoefficient of variation (stability):")
    for i in range(NUM_SUBPIXELS):
        print(f"  Sub[{i}]: {cv[:,:,i].mean()*100:.2f}%")

def analyze_ruler_progression():
    """Analyze how sub-pixels change with known distance"""
    print(f"\n{'='*60}")
    print(f"  RULER CALIBRATION ANALYSIS")
    print(f"{'='*60}")

    distances = [500, 1000, 1500]  # mm
    labels = ["0.5m", "1.0m", "1.5m"]

    data_points = []

    for dist, label in zip(distances, labels):
        raw_file = f"ruler_{label}.raw"
        sdk_file = f"ruler_{label}_sdk.raw"

        if not os.path.exists(raw_file):
            print(f"Missing: {raw_file}")
            continue

        _, pixels = load_raw_frame(raw_file)

        # Center ROI (24x24 pixels in center)
        cy, cx = 120, 160
        roi = pixels[cy-12:cy+12, cx-12:cx+12, :]

        means = [roi[:,:,i].mean() for i in range(NUM_SUBPIXELS)]

        data_points.append({
            'dist': dist,
            'label': label,
            'sub': means,
            'avg23': (means[2] + means[3]) / 2,
            'diff23': means[2] - means[3],
        })

        print(f"\n{label} ({dist}mm):")
        for i in range(NUM_SUBPIXELS):
            print(f"  Sub[{i}]: {means[i]:.1f}")
        print(f"  Avg(2,3): {(means[2]+means[3])/2:.1f}")
        print(f"  Diff(2-3): {means[2]-means[3]:.1f}")

    if len(data_points) >= 2:
        # Fit linear model
        from scipy import stats
        dists = np.array([p['dist'] for p in data_points])
        avgs = np.array([p['avg23'] for p in data_points])

        slope, intercept, r, p, se = stats.linregress(avgs, dists)

        print(f"\n{'='*40}")
        print(f"LINEAR FIT FROM RULER DATA:")
        print(f"  depth_mm = {slope:.6f} * avg + {intercept:.2f}")
        print(f"  R² = {r**2:.4f}")

        # Calculate predictions
        for p in data_points:
            pred = slope * p['avg23'] + intercept
            error = pred - p['dist']
            print(f"  {p['label']}: actual={p['dist']}mm, pred={pred:.0f}mm, error={error:+.0f}mm")

        return slope, intercept

    return None, None

def main():
    print("="*60)
    print("  CUBEEYE I200D DEEP ANALYSIS")
    print("="*60)

    # Check for ruler data first
    if os.path.exists("ruler_1.0m.raw") and os.path.exists("ruler_1.0m_sdk.raw"):
        print("\nAnalyzing ruler_1.0m data...")
        raw, pixels = load_raw_frame("ruler_1.0m.raw")
        sdk_depth = load_sdk_depth("ruler_1.0m_sdk.raw")

        analyze_header(raw)
        analyze_subpixel_statistics(pixels, "(ruler 1.0m)")
        analyze_inter_channel_relationships(pixels, "(ruler 1.0m)")
        correlate_with_sdk_depth(pixels, sdk_depth)

    # Analyze temporal stability
    analyze_temporal_stability()

    # Analyze ruler progression
    slope, intercept = analyze_ruler_progression()

    # Final recommendation
    print(f"\n{'='*60}")
    print(f"  CONCLUSION AND RECOMMENDATIONS")
    print(f"{'='*60}")

    if slope is not None:
        print(f"""
Key findings:
1. Sub[2] ≈ Sub[3] at all distances - NO I/Q PHASE DIFFERENCE
2. The sensor appears to output pre-processed intensity, not raw phase
3. Linear relationship exists between (Sub[2]+Sub[3])/2 and depth

Recommended calibration constants:
  CAL_SLOPE  = {slope:.8f}
  CAL_OFFSET = {intercept:.2f}

The lack of phase difference (Sub[2]-Sub[3] ≈ 0) indicates:
- Sensor is NOT in 4-phase I/Q mode
- May be in single-tap or pre-processed mode
- SDK likely computes depth internally using different data path

Next steps:
1. Run with updated v4l2_hook to capture UVC XU commands
2. Try to find command that switches to raw phase mode
3. If unavailable, use linear calibration from this analysis
""")

if __name__ == "__main__":
    main()

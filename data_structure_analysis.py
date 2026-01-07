#!/usr/bin/env python3
"""
data_structure_analysis.py - Deep dive into actual data structure
"""

import numpy as np
import os
from scipy.stats import pearsonr

os.chdir("/home/cmericli/development/atlas/code/cubeeye_nano_driver")

def load_frame(frame_num):
    raw_file = f"benchmark/raw_{frame_num:04d}.raw"
    sdk_file = f"benchmark/sdk_depth_{frame_num-4:04d}.raw"
    amp_file = f"benchmark/sdk_amp_{frame_num-4:04d}.raw"

    raw = np.fromfile(raw_file, dtype='<u2').reshape(241, 1600)
    raw = raw[1:, :].reshape(240, 320, 5)

    sdk = np.fromfile(sdk_file, dtype='<u2').reshape(480, 640)
    amp = np.fromfile(amp_file, dtype='<u2').reshape(480, 640)
    return raw, sdk, amp

print("=" * 70)
print("Deep Data Structure Analysis")
print("=" * 70)

# Load one frame
raw, sdk, amp = load_frame(4)

# Flatten for correlation analysis
# Use whole frame, filter zeros
mask = (raw[:, :, 2] > 100) & (raw[:, :, 3] > 100)

sub0 = raw[:, :, 0][mask].astype(np.float64)
sub1 = raw[:, :, 1][mask].astype(np.float64)
sub2 = raw[:, :, 2][mask].astype(np.float64)
sub3 = raw[:, :, 3][mask].astype(np.float64)
sub4 = raw[:, :, 4][mask].astype(np.float64)

sdk_ds = sdk[::2, ::2][mask].astype(np.float64)
amp_ds = amp[::2, ::2][mask].astype(np.float64)

print(f"\nCorrelation with SDK depth:")
for i, sub in enumerate([sub0, sub1, sub2, sub3, sub4]):
    r, p = pearsonr(sub, sdk_ds)
    print(f"  Sub[{i}] vs Depth: r={r:.4f} (p={p:.2e})")

print(f"\nCorrelation with SDK amplitude:")
for i, sub in enumerate([sub0, sub1, sub2, sub3, sub4]):
    r, p = pearsonr(sub, amp_ds)
    print(f"  Sub[{i}] vs Amp:   r={r:.4f} (p={p:.2e})")

print(f"\nInter-sub-pixel correlations:")
for i in range(5):
    for j in range(i+1, 5):
        sub_i = [sub0, sub1, sub2, sub3, sub4][i]
        sub_j = [sub0, sub1, sub2, sub3, sub4][j]
        r, p = pearsonr(sub_i, sub_j)
        print(f"  Sub[{i}] vs Sub[{j}]: r={r:.4f}")

# Check if any transformation of sub-pixels matches depth
print("\n" + "=" * 70)
print("Transformation Search")
print("=" * 70)

# Maybe depth is stored directly in Sub[2] with some scaling?
# depth = Sub[2] * scale + offset
# Solve for scale and offset using least squares
from numpy.linalg import lstsq

for sub, name in [(sub2, "Sub[2]"), (sub3, "Sub[3]"), (sub2-sub3, "Sub[2]-Sub[3]"),
                   (sub2+sub3, "Sub[2]+Sub[3]"), (sub2/sub3, "Sub[2]/Sub[3]")]:
    if name == "Sub[2]/Sub[3]":
        sub = sub2 / (sub3 + 0.001)  # Avoid div by zero

    A = np.column_stack([sub, np.ones_like(sub)])
    coeffs, _, _, _ = lstsq(A, sdk_ds, rcond=None)
    pred = A @ coeffs
    rmse = np.sqrt(np.mean((pred - sdk_ds)**2))
    r = np.corrcoef(sub, sdk_ds)[0, 1]

    print(f"\n{name}:")
    print(f"  depth = {coeffs[0]:.6f} * {name} + {coeffs[1]:.2f}")
    print(f"  r={r:.4f}, RMSE={rmse:.1f}mm")

# Check Sub[4] as amplitude
print("\n" + "=" * 70)
print("Sub[4] vs SDK Amplitude Analysis")
print("=" * 70)

# Maybe Sub[4] needs different scaling
for scale in [1, 256, 4096]:
    sub4_scaled = sub4 / scale
    r = np.corrcoef(sub4_scaled, amp_ds)[0, 1]
    print(f"Sub[4]/{scale} vs amp: r={r:.4f}, ranges [{sub4_scaled.min():.1f}, {sub4_scaled.max():.1f}]")

# Check data distribution
print("\n" + "=" * 70)
print("Value Distribution Analysis")
print("=" * 70)

for i, sub in enumerate([sub0, sub1, sub2, sub3, sub4]):
    # Check if values cluster at certain points
    hist, edges = np.histogram(sub, bins=50)
    peak_idx = np.argmax(hist)
    peak_val = (edges[peak_idx] + edges[peak_idx+1]) / 2

    print(f"Sub[{i}]: range [{sub.min():.0f}, {sub.max():.0f}], "
          f"mean={sub.mean():.0f}, median={np.median(sub):.0f}, "
          f"peak≈{peak_val:.0f}")

# Check if values are quantized (8-bit data in 16-bit container)
print("\n" + "=" * 70)
print("Quantization Analysis")
print("=" * 70)

for i, sub in enumerate([sub0, sub1, sub2, sub3, sub4]):
    # Check low byte distribution
    low_bytes = (sub.astype(np.int32) & 0xFF)
    unique_low = len(np.unique(low_bytes))

    # Check if values are multiples of 256
    multiples_of_256 = np.sum(low_bytes == 0) / len(low_bytes) * 100

    print(f"Sub[{i}]: {unique_low} unique low-byte values, "
          f"{multiples_of_256:.1f}% are multiples of 256")

# Look at specific pixels with known good depth
print("\n" + "=" * 70)
print("Sample Pixel Analysis (varying depth)")
print("=" * 70)

# Find pixels at different depths
depths = sdk[::2, ::2]
depth_bins = [(1200, 1300), (1300, 1400), (1400, 1500), (1500, 1600), (1600, 1700)]

for d_min, d_max in depth_bins:
    mask_depth = (depths > d_min) & (depths < d_max) & (raw[:, :, 2] > 100)
    if np.sum(mask_depth) > 10:
        # Get average values in this depth band
        d_mean = depths[mask_depth].mean()
        s2_mean = raw[:, :, 2][mask_depth].mean()
        s3_mean = raw[:, :, 3][mask_depth].mean()
        s4_mean = raw[:, :, 4][mask_depth].mean()
        n = np.sum(mask_depth)

        print(f"Depth {d_min}-{d_max}mm (n={n}): SDK={d_mean:.0f}mm, "
              f"Sub2={s2_mean:.0f}, Sub3={s3_mean:.0f}, Sub4={s4_mean:.0f}")

# Maybe Sub[2]-Sub[3] difference encodes depth?
print("\n" + "=" * 70)
print("Sub[2] - Sub[3] vs Depth")
print("=" * 70)

diff = sub2 - sub3
r = np.corrcoef(diff, sdk_ds)[0, 1]
print(f"Correlation: r={r:.4f}")
print(f"Difference range: [{diff.min():.0f}, {diff.max():.0f}], mean={diff.mean():.0f}")

# Linear fit
A = np.column_stack([diff, np.ones_like(diff)])
coeffs, _, _, _ = lstsq(A, sdk_ds, rcond=None)
pred = A @ coeffs
rmse = np.sqrt(np.mean((pred - sdk_ds)**2))
print(f"Linear fit: depth = {coeffs[0]:.6f} * diff + {coeffs[1]:.2f}")
print(f"RMSE: {rmse:.1f}mm")

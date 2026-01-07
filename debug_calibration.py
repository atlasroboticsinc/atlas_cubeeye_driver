#!/usr/bin/env python3
"""
debug_calibration.py - Debug why calibration is failing
"""
import numpy as np
import glob
import os

os.chdir("/home/cmericli/development/atlas/code/cubeeye_nano_driver")

raw_files = sorted(glob.glob("variable_dist/raw_*.raw"))
sdk_files = sorted(glob.glob("variable_dist/sdk_depth_*.raw"))

print(f"Raw files: {len(raw_files)}, SDK files: {len(sdk_files)}")

# Skip first 4 warmup frames
raw_files = [f for f in raw_files if int(f.split('_')[-1].split('.')[0]) >= 4]
print(f"Valid raw files (after warmup): {len(raw_files)}")

# Check frame number alignment
for i in range(min(5, len(raw_files))):
    raw_num = int(raw_files[i].split('_')[-1].split('.')[0])
    sdk_num = int(sdk_files[i].split('_')[-1].split('.')[0])
    print(f"  raw_{raw_num:04d} <-> sdk_depth_{sdk_num:04d}")

# Load a few frames and check distributions
print("\n=== Sample Frame Analysis ===")
for frame_idx in [0, 50, 100]:
    if frame_idx >= len(raw_files): continue

    raw = np.fromfile(raw_files[frame_idx], dtype=np.uint16).reshape(241, 1600)[1:].reshape(240, 320, 5)
    sdk = np.fromfile(sdk_files[frame_idx], dtype=np.uint16).reshape(480, 640)

    # Center ROI
    roi_raw = raw[108:132, 148:172, :]
    roi_sdk = sdk[216:264, 296:344]

    print(f"\nFrame {frame_idx}:")
    print(f"  SDK center depth: {roi_sdk.mean():.1f} mm (range {roi_sdk.min()}-{roi_sdk.max()})")
    for s in range(5):
        sub = roi_raw[:,:,s]
        print(f"  Sub[{s}]: mean={sub.mean():.1f}, min={sub.min()}, max={sub.max()}")

    # Try biased 4-phase
    I = 57.0 - roi_raw[:,:,2].astype(float)
    Q = 57.0 - roi_raw[:,:,3].astype(float)
    phase = np.arctan2(Q, I)
    phase[phase < 0] += 2 * np.pi
    print(f"  Phase (biased): mean={np.degrees(phase.mean()):.1f}°, std={np.degrees(phase.std()):.1f}°")

    # Try classic 4-phase: (Sub0-Sub2, Sub1-Sub3)
    I2 = roi_raw[:,:,0].astype(float) - roi_raw[:,:,2].astype(float)
    Q2 = roi_raw[:,:,1].astype(float) - roi_raw[:,:,3].astype(float)
    phase2 = np.arctan2(Q2, I2)
    phase2[phase2 < 0] += 2 * np.pi
    print(f"  Phase (classic): mean={np.degrees(phase2.mean()):.1f}°, std={np.degrees(phase2.std()):.1f}°")

# Check depth variation across all frames
print("\n=== Depth Variation Across Frames ===")
sdk_depths = []
for f in sdk_files[:150]:
    sdk = np.fromfile(f, dtype=np.uint16).reshape(480, 640)
    roi = sdk[216:264, 296:344]
    sdk_depths.append(roi.mean())

sdk_depths = np.array(sdk_depths)
print(f"SDK depth range: {sdk_depths.min():.0f} - {sdk_depths.max():.0f} mm")
print(f"SDK depth mean: {sdk_depths.mean():.0f} mm, std: {sdk_depths.std():.0f} mm")

# Check if Sub[2]/Sub[3] vary with depth
print("\n=== Sub[2] vs SDK Depth Correlation ===")
sub2_vals = []
sub3_vals = []
for i, f in enumerate(raw_files[:150]):
    if i >= len(sdk_files): break
    raw = np.fromfile(f, dtype=np.uint16).reshape(241, 1600)[1:].reshape(240, 320, 5)
    roi = raw[108:132, 148:172, :]
    sub2_vals.append(roi[:,:,2].mean())
    sub3_vals.append(roi[:,:,3].mean())

sub2_vals = np.array(sub2_vals)
sub3_vals = np.array(sub3_vals)
print(f"Sub[2] range: {sub2_vals.min():.0f} - {sub2_vals.max():.0f}")
print(f"Sub[3] range: {sub3_vals.min():.0f} - {sub3_vals.max():.0f}")

from scipy.stats import pearsonr
r2, _ = pearsonr(sub2_vals, sdk_depths[:len(sub2_vals)])
r3, _ = pearsonr(sub3_vals, sdk_depths[:len(sub3_vals)])
print(f"Correlation Sub[2] vs depth: {r2:.4f}")
print(f"Correlation Sub[3] vs depth: {r3:.4f}")

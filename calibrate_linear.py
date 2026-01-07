#!/usr/bin/env python3
"""
calibrate_linear.py - Direct linear calibration using Sub[2] intensity
Since Sub[2] correlates strongly with depth (r=0.94), use linear regression.
"""
import numpy as np
import matplotlib.pyplot as plt
import glob
import os

os.chdir("/home/cmericli/development/atlas/code/cubeeye_nano_driver")

def main():
    raw_files = sorted(glob.glob("variable_dist/raw_*.raw"))
    sdk_files = sorted(glob.glob("variable_dist/sdk_depth_*.raw"))

    # Skip warmup frames
    raw_files = [f for f in raw_files if int(f.split('_')[-1].split('.')[0]) >= 4]

    sub2_vals = []
    sub3_vals = []
    sdk_depths = []

    print(f"Processing {min(len(raw_files), len(sdk_files))} frames...")

    for i, (r_f, s_f) in enumerate(zip(raw_files, sdk_files)):
        raw = np.fromfile(r_f, dtype=np.uint16).reshape(241, 1600)[1:].reshape(240, 320, 5)
        sdk = np.fromfile(s_f, dtype=np.uint16).reshape(480, 640)

        # Center ROI with amplitude filtering
        roi_raw = raw[108:132, 148:172, :]
        roi_sdk = sdk[216:264, 296:344]

        # Use Sub[4] as amplitude for filtering
        amp = roi_raw[:,:,4].astype(float)
        threshold = np.mean(amp) + 0.5 * np.std(amp)
        mask = amp > threshold

        if np.sum(mask) < 10:
            continue

        sub2_vals.append(roi_raw[:,:,2][mask].mean())
        sub3_vals.append(roi_raw[:,:,3][mask].mean())
        sdk_depths.append(roi_sdk.mean())

    sub2_vals = np.array(sub2_vals)
    sub3_vals = np.array(sub3_vals)
    sdk_depths = np.array(sdk_depths)

    print(f"Valid frames: {len(sdk_depths)}")
    print(f"Depth range: {sdk_depths.min():.0f} - {sdk_depths.max():.0f} mm")

    # Linear regression: depth = slope * sub2 + offset
    m, b = np.polyfit(sub2_vals, sdk_depths, 1)
    pred = m * sub2_vals + b
    rmse = np.sqrt(np.mean((pred - sdk_depths)**2))
    r2 = np.corrcoef(sub2_vals, sdk_depths)[0,1]**2

    print("\n" + "="*40)
    print("  LINEAR CALIBRATION (Sub[2] -> Depth)")
    print("="*40)
    print(f"Formula: depth = {m:.6f} * Sub[2] + {b:.2f}")
    print(f"R² = {r2:.5f}")
    print(f"RMSE = {rmse:.2f} mm")
    print("="*40)

    # Also try with Sub[2] + Sub[3] combined
    combined = (sub2_vals + sub3_vals) / 2
    m2, b2 = np.polyfit(combined, sdk_depths, 1)
    pred2 = m2 * combined + b2
    rmse2 = np.sqrt(np.mean((pred2 - sdk_depths)**2))
    r2_2 = np.corrcoef(combined, sdk_depths)[0,1]**2

    print("\n" + "="*40)
    print("  LINEAR CALIBRATION ((Sub[2]+Sub[3])/2)")
    print("="*40)
    print(f"Formula: depth = {m2:.6f} * avg + {b2:.2f}")
    print(f"R² = {r2_2:.5f}")
    print(f"RMSE = {rmse2:.2f} mm")
    print("="*40)

    # Plot
    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)
    plt.scatter(sub2_vals, sdk_depths, alpha=0.5, s=10)
    plt.plot(sub2_vals, pred, 'r-', linewidth=2, label=f'Fit (R²={r2:.3f})')
    plt.xlabel("Sub[2]")
    plt.ylabel("SDK Depth (mm)")
    plt.title("Sub[2] vs Depth")
    plt.legend()
    plt.grid()

    plt.subplot(1, 2, 2)
    plt.scatter(combined, sdk_depths, alpha=0.5, s=10)
    plt.plot(combined, pred2, 'r-', linewidth=2, label=f'Fit (R²={r2_2:.3f})')
    plt.xlabel("(Sub[2]+Sub[3])/2")
    plt.ylabel("SDK Depth (mm)")
    plt.title("Combined vs Depth")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig("calibration_linear.png")
    print("\nSaved calibration_linear.png")

    # Now try the ACTUAL correct phase calculation
    # If Sub[0,1] are black level (~121) and Sub[2,3] are I,Q...
    print("\n" + "="*40)
    print("  PHASE CALIBRATION (with correct black level)")
    print("="*40)

    phases = []
    for i, (r_f, s_f) in enumerate(zip(raw_files[:len(sdk_files)], sdk_files)):
        raw = np.fromfile(r_f, dtype=np.uint16).reshape(241, 1600)[1:].reshape(240, 320, 5)

        roi = raw[108:132, 148:172, :]

        # Use actual black level from Sub[0,1]
        black_level = (roi[:,:,0].mean() + roi[:,:,1].mean()) / 2

        # I = black - Sub[2], Q = black - Sub[3]
        I = black_level - roi[:,:,2].astype(float)
        Q = black_level - roi[:,:,3].astype(float)

        phase = np.arctan2(Q, I)
        phase[phase < 0] += 2 * np.pi
        phases.append(phase.mean())

    phases = np.array(phases)
    print(f"Phase range: {np.degrees(phases.min()):.1f}° - {np.degrees(phases.max()):.1f}°")
    print(f"Phase std: {np.degrees(phases.std()):.2f}°")

    # Check phase vs depth correlation
    r_phase = np.corrcoef(phases, sdk_depths[:len(phases)])[0,1]
    print(f"Phase vs Depth correlation: {r_phase:.4f}")

    if abs(r_phase) > 0.5:
        m3, b3 = np.polyfit(phases, sdk_depths[:len(phases)], 1)
        print(f"\nPhase formula: depth = {m3:.4f} * phase + {b3:.2f}")
        print(f"Max range (2π): {m3 * 2 * np.pi:.2f} mm")
        freq = 299792458000 / (4 * np.pi * m3)
        print(f"Implied frequency: {freq/1e6:.2f} MHz")

if __name__ == "__main__":
    main()

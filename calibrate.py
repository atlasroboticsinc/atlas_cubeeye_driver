#!/usr/bin/env python3
"""
calibrate.py - Derive golden constants for depth calculation
"""
import numpy as np
import matplotlib.pyplot as plt
import glob
import os

os.chdir("/home/cmericli/development/atlas/code/cubeeye_nano_driver")

def main():
    # Use variable_dist dataset (clipboard moving back and forth)
    raw_files = sorted(glob.glob("variable_dist/raw_*.raw"))
    sdk_files = sorted(glob.glob("variable_dist/sdk_depth_*.raw"))

    if not raw_files:
        print("Error: No benchmark files found.")
        return

    phases = []
    depths = []

    print(f"Processing {len(raw_files)} frames...")

    for r_f, s_f in zip(raw_files, sdk_files):
        # 1. Load Data
        # Raw: 1600x241 -> Skip header -> Reshape to 240x320x5
        raw = np.fromfile(r_f, dtype=np.uint16).reshape(241, 1600)[1:].reshape(240, 320, 5)
        # SDK: 640x480 (Ground Truth)
        sdk = np.fromfile(s_f, dtype=np.uint16).reshape(480, 640)

        # 2. Define ROI (Center 10%)
        # Raw ROI: [108:132, 148:172] (24x24 box in center)
        roi_raw = raw[108:132, 148:172, :]

        # 3. Compute Phase & Amplitude (Biased 4-Phase Logic)
        # I = 57 - Sub[2], Q = 57 - Sub[3]
        i_val = 57.0 - roi_raw[:,:,2].astype(float)
        q_val = 57.0 - roi_raw[:,:,3].astype(float)

        amp = np.sqrt(i_val**2 + q_val**2)
        phase_map = np.arctan2(q_val, i_val)
        phase_map[phase_map < 0] += 2 * np.pi

        # 4. Filter for Clipboard (High Amplitude Only)
        # Dynamic threshold: Mean + 0.5 * StdDev
        threshold = np.mean(amp) + 0.5 * np.std(amp)
        mask = amp > threshold

        if np.sum(mask) < 10: continue # Skip empty/bad frames

        # 5. Get Matching Ground Truth
        # Map 320x240 ROI to 640x480 ROI (multiply indices by 2)
        roi_sdk = sdk[216:264, 296:344] # Corresponding center area

        avg_phase = np.mean(phase_map[mask])
        avg_depth = np.mean(roi_sdk) # Simple mean of center for now

        phases.append(avg_phase)
        depths.append(avg_depth)

    # 6. Solve for Constants
    phases = np.array(phases)
    depths = np.array(depths)

    # Linear Regression: Depth = Slope * Phase + Offset
    m, b = np.polyfit(phases, depths, 1)

    # Physics Extraction
    c = 299792458000 # mm/s
    freq_hz = c / (4 * np.pi * m)
    max_range = m * 2 * np.pi

    print("\n" + "="*30)
    print("  GOLDEN CONSTANTS ACQUIRED")
    print("="*30)
    print(f"Slope (m):       {m:.4f}")
    print(f"Offset (b):      {b:.4f}")
    print(f"Mod Frequency:   {freq_hz/1e6:.2f} MHz")
    print(f"Max Range:       {max_range:.2f} mm")
    print(f"Linearity (R^2): {np.corrcoef(phases, depths)[0,1]**2:.5f}")
    print("="*30 + "\n")

    plt.figure(figsize=(10, 6))
    plt.scatter(phases, depths, alpha=0.5, label='Data')
    plt.plot(phases, m*phases + b, 'r', label='Fit')
    plt.xlabel("Phase (rad)"); plt.ylabel("Depth (mm)")
    plt.title(f"Driver Calibration (Freq: {freq_hz/1e6:.1f} MHz)")
    plt.legend(); plt.grid()
    plt.savefig("calibration_result.png")
    print("Saved calibration_result.png")

if __name__ == "__main__":
    main()

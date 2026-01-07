#!/usr/bin/env python3
"""
analyze_ruler.py - Analyze ruler calibration captures
"""
import numpy as np
import os

os.chdir("/home/cmericli/development/atlas/code/cubeeye_nano_driver")

# Known distances in mm
distances = {
    "ruler_0.5m.raw": 500,
    "ruler_1.0m.raw": 1000,
    "ruler_1.5m.raw": 1500,
}

print("=" * 60)
print("  RULER CALIBRATION ANALYSIS")
print("=" * 60)

data_points = []

for filename, dist_mm in distances.items():
    if not os.path.exists(filename):
        print(f"Missing: {filename}")
        continue

    raw = np.fromfile(filename, dtype=np.uint16).reshape(241, 1600)
    pixel_data = raw[1:, :].reshape(240, 320, 5)

    # Center ROI
    roi = pixel_data[108:132, 148:172, :]

    s0 = roi[:,:,0].mean()
    s1 = roi[:,:,1].mean()
    s2 = roi[:,:,2].mean()
    s3 = roi[:,:,3].mean()
    s4 = roi[:,:,4].mean()

    avg_23 = (s2 + s3) / 2

    data_points.append({
        'dist': dist_mm,
        's0': s0, 's1': s1, 's2': s2, 's3': s3, 's4': s4,
        'avg': avg_23,
        'diff': s2 - s3,
    })

    print(f"\n{filename} ({dist_mm}mm):")
    print(f"  Sub[0,1] (black): {s0:.1f}, {s1:.1f}")
    print(f"  Sub[2,3] (signal): {s2:.1f}, {s3:.1f} (diff={s2-s3:.1f})")
    print(f"  Sub[4] (amp): {s4:.1f}")
    print(f"  Average(2,3): {avg_23:.1f}")

# Linear regression: distance = slope * avg + offset
dists = np.array([p['dist'] for p in data_points])
avgs = np.array([p['avg'] for p in data_points])
s2s = np.array([p['s2'] for p in data_points])

# Fit: dist = m * avg + b
m, b = np.polyfit(avgs, dists, 1)
pred = m * avgs + b
rmse = np.sqrt(np.mean((pred - dists)**2))

print("\n" + "=" * 60)
print("  LINEAR CALIBRATION")
print("=" * 60)
print(f"\nUsing (Sub[2]+Sub[3])/2:")
print(f"  depth_mm = {m:.6f} * avg + {b:.2f}")
print(f"  RMSE = {rmse:.2f} mm")

# Check fit quality
for i, p in enumerate(data_points):
    predicted = m * p['avg'] + b
    error = predicted - p['dist']
    print(f"  {p['dist']}mm: predicted={predicted:.1f}mm, error={error:+.1f}mm")

# Also check if Sub[0]/Sub[1] carry information
s0s = np.array([p['s0'] for p in data_points])
m0, b0 = np.polyfit(s0s, dists, 1)
pred0 = m0 * s0s + b0
rmse0 = np.sqrt(np.mean((pred0 - dists)**2))
print(f"\nUsing Sub[0]:")
print(f"  depth_mm = {m0:.4f} * Sub[0] + {b0:.2f}")
print(f"  RMSE = {rmse0:.2f} mm")

# Combined model
print("\n" + "=" * 60)
print("  INSIGHTS")
print("=" * 60)

print(f"""
Key observations:
1. Sub[2] ≈ Sub[3] (diff < 70 out of ~30000) = NO PHASE INFORMATION
2. Both INCREASE with distance (unusual for ToF intensity!)
3. Sub[0,1] also increase with distance

This is NOT standard ToF output. Possible explanations:
- Pre-computed distance encoded as intensity
- Sensor in diagnostic/test mode
- Time-sequential mode capturing same phase twice

The linear relationship ({m:.6f} * avg + {b:.1f}) works but:
- Slope is positive (distance increases with signal)
- This is opposite to normal ToF amplitude behavior
""")

# Calculate what the sensor might be encoding
# If it's encoding distance directly with some scaling...
print("\nIf sensor encodes distance directly:")
print(f"  Scale factor: 1/{m:.6f} = {1/m:.2f} mm per unit")
print(f"  Zero offset: {-b/m:.1f} units at 0mm")

# Check the black level trend
print(f"\nBlack level (Sub[0]) trend:")
for p in data_points:
    print(f"  {p['dist']}mm: Sub[0]={p['s0']:.1f}")

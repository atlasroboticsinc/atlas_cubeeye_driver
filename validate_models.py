#!/usr/bin/env python3
"""
validate_models.py - Validate all calibration models against ruler data

Tests linear, quadratic, and multi-variable models to determine best approach.
"""

import numpy as np
import os
from pathlib import Path

os.chdir(Path(__file__).parent)

# Constants
DEPTH_WIDTH = 320
DEPTH_HEIGHT = 240
NUM_SUBPIXELS = 5

# Calibration constants from depth_processor.h
CAL_SLOPE = 0.12881366    # Linear
CAL_OFFSET = -2763.38
CAL_A = 0.0000284721      # Quadratic
CAL_B = -1.487768
CAL_C = 19901.57
K1 = 0.071687             # Multi-variable
K2 = -5.891034
K3 = -890.35

def load_raw_frame(filename):
    """Load and parse raw frame"""
    data = np.fromfile(filename, dtype=np.uint16).reshape(241, 1600)
    pixels = data[1:, :].reshape(DEPTH_HEIGHT, DEPTH_WIDTH, NUM_SUBPIXELS)
    return pixels

def compute_depth_linear(sub2, sub3):
    """Linear model: depth = slope * avg + offset"""
    avg = (sub2 + sub3) / 2
    return CAL_SLOPE * avg + CAL_OFFSET

def compute_depth_quadratic(sub2, sub3):
    """Quadratic model: depth = a * avg^2 + b * avg + c"""
    avg = (sub2 + sub3) / 2
    return CAL_A * avg * avg + CAL_B * avg + CAL_C

def compute_depth_multivar(sub2, sub3):
    """Multi-variable model: depth = k1 * avg + k2 * diff + k3"""
    avg = (sub2 + sub3) / 2
    diff = sub2 - sub3
    return K1 * avg + K2 * diff + K3

def validate_at_distance(filename, actual_dist):
    """Validate all models at a known distance"""
    pixels = load_raw_frame(filename)

    # Center ROI (24x24)
    cy, cx = 120, 160
    roi = pixels[cy-12:cy+12, cx-12:cx+12, :]

    sub2 = roi[:, :, 2].astype(np.float64)
    sub3 = roi[:, :, 3].astype(np.float64)
    sub4 = roi[:, :, 4].astype(np.float64)

    # Compute depths with each model
    depth_linear = compute_depth_linear(sub2, sub3)
    depth_quad = compute_depth_quadratic(sub2, sub3)
    depth_multi = compute_depth_multivar(sub2, sub3)

    # Calculate errors (center ROI mean)
    mean_linear = np.mean(depth_linear)
    mean_quad = np.mean(depth_quad)
    mean_multi = np.mean(depth_multi)

    error_linear = mean_linear - actual_dist
    error_quad = mean_quad - actual_dist
    error_multi = mean_multi - actual_dist

    # Per-pixel RMSE
    rmse_linear = np.sqrt(np.mean((depth_linear - actual_dist)**2))
    rmse_quad = np.sqrt(np.mean((depth_quad - actual_dist)**2))
    rmse_multi = np.sqrt(np.mean((depth_multi - actual_dist)**2))

    return {
        'actual': actual_dist,
        'linear': {'mean': mean_linear, 'error': error_linear, 'rmse': rmse_linear},
        'quad': {'mean': mean_quad, 'error': error_quad, 'rmse': rmse_quad},
        'multi': {'mean': mean_multi, 'error': error_multi, 'rmse': rmse_multi},
        'sub2_mean': np.mean(sub2),
        'sub3_mean': np.mean(sub3),
        'diff_mean': np.mean(sub2 - sub3),
        'amp_mean': np.mean(sub4),
    }

def main():
    print("="*70)
    print("  CALIBRATION MODEL VALIDATION")
    print("="*70)

    distances = [
        ("ruler_0.5m.raw", 500),
        ("ruler_1.0m.raw", 1000),
        ("ruler_1.5m.raw", 1500),
    ]

    results = []
    for filename, dist in distances:
        if os.path.exists(filename):
            result = validate_at_distance(filename, dist)
            results.append(result)

    # Print results table
    print("\nResults at each calibration distance:")
    print("-"*70)
    print(f"{'Distance':>10} | {'Linear':>20} | {'Quadratic':>20} | {'MultiVar':>20}")
    print(f"{'':>10} | {'Mean (Error)':>20} | {'Mean (Error)':>20} | {'Mean (Error)':>20}")
    print("-"*70)

    for r in results:
        print(f"{r['actual']:>8}mm | "
              f"{r['linear']['mean']:>7.0f}mm ({r['linear']['error']:>+4.0f}) | "
              f"{r['quad']['mean']:>7.0f}mm ({r['quad']['error']:>+4.0f}) | "
              f"{r['multi']['mean']:>7.0f}mm ({r['multi']['error']:>+4.0f})")

    # Overall statistics
    print("-"*70)

    linear_errors = [abs(r['linear']['error']) for r in results]
    quad_errors = [abs(r['quad']['error']) for r in results]
    multi_errors = [abs(r['multi']['error']) for r in results]

    print(f"{'MAE':>10} | {np.mean(linear_errors):>7.1f}mm              | "
          f"{np.mean(quad_errors):>7.1f}mm              | "
          f"{np.mean(multi_errors):>7.1f}mm")

    print(f"{'Max Error':>10} | {np.max(linear_errors):>7.1f}mm              | "
          f"{np.max(quad_errors):>7.1f}mm              | "
          f"{np.max(multi_errors):>7.1f}mm")

    # Raw data summary
    print("\n" + "="*70)
    print("  RAW DATA SUMMARY")
    print("="*70)
    print(f"{'Distance':>10} | {'Sub[2]':>10} | {'Sub[3]':>10} | {'Diff(2-3)':>10} | {'Amp':>10}")
    print("-"*70)
    for r in results:
        print(f"{r['actual']:>8}mm | {r['sub2_mean']:>10.0f} | {r['sub3_mean']:>10.0f} | "
              f"{r['diff_mean']:>10.1f} | {r['amp_mean']:>10.0f}")

    # Recommendation
    print("\n" + "="*70)
    print("  RECOMMENDATION")
    print("="*70)

    best_model = 'quadratic' if np.mean(quad_errors) < np.mean(linear_errors) else 'linear'
    if np.mean(multi_errors) < np.mean(quad_errors):
        best_model = 'multi-variable'

    print(f"""
Best performing model at calibration points: {best_model.upper()}

However, be aware:
- Quadratic and multi-variable models are fitted exactly to 3 points
- They may not generalize well outside the 0.5m - 1.5m range
- Linear model may be more robust for extrapolation

For production use:
1. If operating within 0.5m - 1.5m: Use quadratic or multi-variable
2. If operating outside this range: Use linear (more robust)
3. For best results: Capture more calibration points and re-fit

Current calibration constants:
  Linear:     depth = {CAL_SLOPE:.8f} * avg + {CAL_OFFSET:.2f}
  Quadratic:  depth = {CAL_A:.10f} * avg^2 + {CAL_B:.6f} * avg + {CAL_C:.2f}
  MultiVar:   depth = {K1:.6f} * avg + {K2:.6f} * diff + {K3:.2f}
""")

if __name__ == "__main__":
    main()

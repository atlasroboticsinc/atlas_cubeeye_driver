#!/usr/bin/env python3
"""
Derive depth formula from raw V4L2 data and SDK depth output
"""

import numpy as np
from scipy import optimize
import warnings
warnings.filterwarnings('ignore')

def load_data(raw_path, sdk_path):
    """Load raw V4L2 frame and SDK depth"""
    with open(raw_path, 'rb') as f:
        raw_data = f.read()

    with open(sdk_path, 'rb') as f:
        sdk_data = f.read()

    # Parse raw frame: 241 rows × 1600 uint16
    raw_u16 = np.frombuffer(raw_data, dtype=np.uint16).reshape(241, 1600)

    # Skip header row, extract data rows
    data_rows = raw_u16[1:]  # 240 rows × 1600 uint16

    # Reshape to pixels: 240 rows × 320 pixels × 5 sub-values
    raw_pixels = data_rows.reshape(240, 320, 5)

    # SDK depth: 480 × 640 uint16
    sdk_depth = np.frombuffer(sdk_data, dtype=np.uint16).reshape(480, 640)

    # Subsample SDK to match raw resolution
    sdk_sub = sdk_depth[::2, ::2]  # 240 × 320

    return raw_pixels, sdk_sub

def analyze_correlation_matrix(raw_pixels, sdk_depth):
    """Find best correlating combinations of sub-values"""
    print("Correlation analysis:")
    print("=" * 60)

    # Flatten for correlation
    depth_flat = sdk_depth.flatten().astype(float)
    valid_mask = depth_flat > 0  # Only correlate non-zero depths

    print(f"\nPixels with valid depth: {valid_mask.sum()} / {len(depth_flat)}")

    # Analyze each sub-value
    for i in range(5):
        sub_flat = raw_pixels[:, :, i].flatten().astype(float)
        if valid_mask.sum() > 0:
            corr = np.corrcoef(sub_flat[valid_mask], depth_flat[valid_mask])[0, 1]
            print(f"Sub[{i}] vs depth: r = {corr:.4f}")

            # Also try byte-level
            sub_low = (raw_pixels[:, :, i] & 0xFF).flatten().astype(float)
            sub_high = (raw_pixels[:, :, i] >> 8).flatten().astype(float)
            corr_low = np.corrcoef(sub_low[valid_mask], depth_flat[valid_mask])[0, 1]
            corr_high = np.corrcoef(sub_high[valid_mask], depth_flat[valid_mask])[0, 1]
            print(f"  Low byte: r = {corr_low:.4f}, High byte: r = {corr_high:.4f}")

def try_linear_model(raw_pixels, sdk_depth):
    """Try linear combination of sub-values"""
    print("\n\nLinear model fitting:")
    print("=" * 60)

    # Flatten
    depth = sdk_depth.flatten().astype(float)
    valid = depth > 100  # Valid depth range

    # Create feature matrix
    X = np.column_stack([
        raw_pixels[:, :, i].flatten().astype(float) for i in range(5)
    ])

    # Add constant term
    X = np.column_stack([np.ones(len(X)), X])

    # Solve least squares
    X_valid = X[valid]
    y_valid = depth[valid]

    try:
        coeffs, residuals, rank, s = np.linalg.lstsq(X_valid, y_valid, rcond=None)
        print(f"Linear coefficients (const, s0, s1, s2, s3, s4):")
        print(f"  {coeffs}")

        # Evaluate
        pred = X @ coeffs
        rmse = np.sqrt(np.mean((pred[valid] - y_valid) ** 2))
        corr = np.corrcoef(pred[valid], y_valid)[0, 1]
        print(f"Linear model: RMSE = {rmse:.2f} mm, r = {corr:.4f}")

    except Exception as e:
        print(f"Linear fit failed: {e}")

def try_byte_combinations(raw_pixels, sdk_depth):
    """Try various byte-level combinations"""
    print("\n\nByte combination analysis:")
    print("=" * 60)

    depth = sdk_depth.flatten().astype(float)
    valid = depth > 100

    # Extract bytes from each sub-value
    bytes_data = {}
    for i in range(5):
        bytes_data[f's{i}_lo'] = (raw_pixels[:, :, i] & 0xFF).flatten().astype(float)
        bytes_data[f's{i}_hi'] = (raw_pixels[:, :, i] >> 8).flatten().astype(float)

    # Try different combinations
    combinations = [
        # Simple combinations
        ("s2_hi * 100 + s2_lo", lambda: bytes_data['s2_hi'] * 100 + bytes_data['s2_lo']),
        ("s2_hi * 256 + s2_lo", lambda: bytes_data['s2_hi'] * 256 + bytes_data['s2_lo']),
        ("s2_full", lambda: raw_pixels[:, :, 2].flatten().astype(float)),
        ("s4_hi * 32", lambda: bytes_data['s4_hi'] * 32),
        ("s4_hi * 100", lambda: bytes_data['s4_hi'] * 100),

        # Cross sub-value combinations
        ("s0 + s2_hi * 10", lambda: bytes_data['s0_lo'] + bytes_data['s2_hi'] * 10),
        ("s0 * s4_hi", lambda: bytes_data['s0_lo'] * bytes_data['s4_hi']),
        ("(s2 + s4) / 2", lambda: (raw_pixels[:, :, 2].flatten().astype(float) +
                                   raw_pixels[:, :, 4].flatten().astype(float)) / 2),

        # Phase-like combinations
        ("atan2(s0, s1) scaled", lambda: np.arctan2(bytes_data['s0_lo'],
                                                     bytes_data['s1_lo']) * 1000),
        ("s2 / s0 * 100", lambda: np.where(bytes_data['s0_lo'] > 0,
                                           raw_pixels[:, :, 2].flatten().astype(float) /
                                           bytes_data['s0_lo'] * 100, 0)),
    ]

    for name, func in combinations:
        try:
            pred = func()
            if np.any(np.isnan(pred)):
                pred = np.nan_to_num(pred, nan=0)
            rmse = np.sqrt(np.mean((pred[valid] - depth[valid]) ** 2))
            corr = np.corrcoef(pred[valid], depth[valid])[0, 1]
            print(f"{name:30s}: RMSE = {rmse:8.2f} mm, r = {corr:+.4f}")
        except Exception as e:
            print(f"{name:30s}: Error - {e}")

def analyze_pixel_structure(raw_pixels, sdk_depth):
    """Analyze structure of individual pixels"""
    print("\n\nPixel structure analysis:")
    print("=" * 60)

    # Sample pixels at known depths
    test_points = [
        (120, 160, "center"),
        (60, 80, "top-left quarter"),
        (180, 240, "bottom-right quarter"),
    ]

    for row, col, name in test_points:
        pixel = raw_pixels[row, col]
        depth = sdk_depth[row, col]

        print(f"\n{name} ({row}, {col}):")
        print(f"  SDK depth: {depth} mm")
        print(f"  Raw values: S0={pixel[0]}, S1={pixel[1]}, S2={pixel[2]}, S3={pixel[3]}, S4={pixel[4]}")

        # Byte breakdown
        for i in range(5):
            lo = pixel[i] & 0xFF
            hi = pixel[i] >> 8
            print(f"  S{i}: {pixel[i]:5d} = {hi:3d}*256 + {lo:3d} (hi={hi}, lo={lo})")

def try_nonlinear_fit(raw_pixels, sdk_depth):
    """Try nonlinear curve fitting"""
    print("\n\nNonlinear model fitting:")
    print("=" * 60)

    depth = sdk_depth.flatten().astype(float)
    valid = (depth > 100) & (depth < 7000)

    # Use Sub[2] as primary input (seemed to have some correlation)
    s2 = raw_pixels[:, :, 2].flatten().astype(float)
    s0 = raw_pixels[:, :, 0].flatten().astype(float)
    s4_hi = (raw_pixels[:, :, 4] >> 8).flatten().astype(float)

    # Try polynomial fit on S2
    print("\nPolynomial fit on S2:")
    for degree in [1, 2, 3, 4]:
        try:
            coeffs = np.polyfit(s2[valid], depth[valid], degree)
            pred = np.polyval(coeffs, s2)
            rmse = np.sqrt(np.mean((pred[valid] - depth[valid]) ** 2))
            corr = np.corrcoef(pred[valid], depth[valid])[0, 1]
            print(f"  Degree {degree}: RMSE = {rmse:.2f} mm, r = {corr:.4f}")
        except Exception as e:
            print(f"  Degree {degree}: Error - {e}")

    # Try with S4 high byte
    print("\nLinear fit on S4 high byte:")
    try:
        coeffs = np.polyfit(s4_hi[valid], depth[valid], 1)
        pred = np.polyval(coeffs, s4_hi)
        rmse = np.sqrt(np.mean((pred[valid] - depth[valid]) ** 2))
        corr = np.corrcoef(pred[valid], depth[valid])[0, 1]
        print(f"  Coefficients: {coeffs}")
        print(f"  RMSE = {rmse:.2f} mm, r = {corr:.4f}")
    except Exception as e:
        print(f"  Error - {e}")

def analyze_frame_differences(raw_paths, sdk_paths):
    """Analyze consistency across multiple frames"""
    print("\n\nMulti-frame consistency analysis:")
    print("=" * 60)

    all_raw = []
    all_sdk = []

    for raw_path, sdk_path in zip(raw_paths, sdk_paths):
        raw_pixels, sdk_depth = load_data(raw_path, sdk_path)
        all_raw.append(raw_pixels)
        all_sdk.append(sdk_depth)

    # Compare center pixels across frames
    print("\nCenter pixel variation across frames:")
    for i, (raw, sdk) in enumerate(zip(all_raw, all_sdk)):
        pixel = raw[120, 160]
        depth = sdk[120, 160]
        print(f"  Frame {i}: depth={depth}mm, S0={pixel[0]}, S1={pixel[1]}, "
              f"S2={pixel[2]}, S3={pixel[3]}, S4={pixel[4]}")

if __name__ == "__main__":
    # Use synchronized frames (raw 4 corresponds to SDK 0)
    raw_paths = [f"data/sync_capture/raw_{i:04d}.bin" for i in range(4, 9)]
    sdk_paths = [f"data/sync_depth_{i}.raw" for i in range(5)]

    # Load first frame pair for detailed analysis
    raw_pixels, sdk_depth = load_data(raw_paths[0], sdk_paths[0])

    print("Data shapes:")
    print(f"  Raw: {raw_pixels.shape}")
    print(f"  SDK: {sdk_depth.shape}")

    analyze_correlation_matrix(raw_pixels, sdk_depth)
    try_linear_model(raw_pixels, sdk_depth)
    try_byte_combinations(raw_pixels, sdk_depth)
    analyze_pixel_structure(raw_pixels, sdk_depth)
    try_nonlinear_fit(raw_pixels, sdk_depth)
    analyze_frame_differences(raw_paths, sdk_paths)

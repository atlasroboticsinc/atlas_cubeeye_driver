#!/usr/bin/env python3
"""
Analyze raw V4L2 frame format from CubeEye I200D
"""

import numpy as np
import struct
import sys

def analyze_frame(raw_path, sdk_path=None):
    """Analyze raw frame structure and compare with SDK output if available"""

    # Read raw frame
    with open(raw_path, 'rb') as f:
        raw_data = f.read()

    print(f"Raw frame: {raw_path}")
    print(f"  Size: {len(raw_data)} bytes")

    # Frame structure: 241 rows × 3200 bytes
    ROWS = 241
    BYTES_PER_ROW = 3200
    expected_size = ROWS * BYTES_PER_ROW
    print(f"  Expected: {expected_size} bytes ({ROWS} rows × {BYTES_PER_ROW} bytes/row)")

    # Parse as uint16 array (V4L2 YUYV format = 2 bytes per sample)
    raw_u16 = np.frombuffer(raw_data, dtype=np.uint16)
    print(f"  As uint16: {len(raw_u16)} values")
    print(f"  Value range: {raw_u16.min()} - {raw_u16.max()}")

    # Reshape to row format
    raw_rows = raw_u16.reshape(ROWS, -1)
    print(f"  Reshaped: {raw_rows.shape} ({raw_rows.shape[1]} uint16 per row)")

    # Analyze first row (header?)
    header_row = raw_rows[0]
    print(f"\n  Header row (row 0):")
    print(f"    Non-zero values: {np.count_nonzero(header_row)}")
    print(f"    First 20 values: {header_row[:20]}")

    # Analyze data rows
    data_rows = raw_rows[1:]
    print(f"\n  Data rows (rows 1-240):")
    print(f"    Shape: {data_rows.shape}")
    print(f"    Non-zero values: {np.count_nonzero(data_rows)}")
    print(f"    Value range: {data_rows.min()} - {data_rows.max()}")

    # Each row has 1600 uint16 values
    # If each pixel has 5 uint16 values, that's 320 pixels per row
    # 240 rows × 320 pixels = 76,800 pixels
    # SDK outputs 640×480 = 307,200 pixels (4× more, likely interpolated)

    PIXELS_PER_ROW = 320
    VALUES_PER_PIXEL = 5  # Based on previous analysis

    print(f"\n  Per-pixel analysis (assuming {VALUES_PER_PIXEL} uint16 per pixel):")

    # Extract sub-values for center row
    center_row_idx = 120  # Row 120 of data
    center_row = data_rows[center_row_idx]

    # Reshape row to pixels
    row_pixels = center_row.reshape(-1, VALUES_PER_PIXEL)
    print(f"    Row shape: {row_pixels.shape} ({row_pixels.shape[0]} pixels)")

    # Analyze center pixel
    center_px_idx = 160  # Center of 320-pixel row
    center_pixel = row_pixels[center_px_idx]
    print(f"    Center pixel sub-values: {center_pixel}")

    # Analyze sub-value statistics
    for i in range(VALUES_PER_PIXEL):
        sub_vals = row_pixels[:, i]
        print(f"    Sub[{i}]: min={sub_vals.min()}, max={sub_vals.max()}, mean={sub_vals.mean():.1f}")

    # If SDK depth available, compare
    if sdk_path:
        with open(sdk_path, 'rb') as f:
            sdk_data = f.read()
        sdk_depth = np.frombuffer(sdk_data, dtype=np.uint16).reshape(480, 640)

        print(f"\n  SDK depth: {sdk_path}")
        print(f"    Shape: {sdk_depth.shape}")
        print(f"    Value range: {sdk_depth.min()} - {sdk_depth.max()}")

        # Center pixel (SDK coordinates)
        sdk_center = sdk_depth[240, 320]
        print(f"    Center pixel depth: {sdk_center} mm")

        # Try to find correlation between raw sub-values and SDK depth
        print(f"\n  Attempting to correlate raw values with SDK depth:")

        # SDK is 640×480, raw is 320×240, so SDK uses 2× interpolation
        # Map raw pixel (160, 120) to SDK pixel (320, 240)
        # The SDK center (320, 240) should come from raw (160, 120)

        # Extract depth values from 320×240 region of SDK
        sdk_subsampled = sdk_depth[::2, ::2]  # Every other pixel
        print(f"    SDK subsampled: {sdk_subsampled.shape}")
        sdk_center_sub = sdk_subsampled[120, 160]
        print(f"    SDK subsampled center: {sdk_center_sub} mm")

        # Try different formulas for raw → depth
        # Formula 1: Simple sub-value combination
        d0 = center_pixel[0]
        d1 = center_pixel[1]
        d2 = center_pixel[2]
        d3 = center_pixel[3]
        d4 = center_pixel[4]

        print(f"\n    Raw pixel values: {d0}, {d1}, {d2}, {d3}, {d4}")
        print(f"    Target SDK depth: {sdk_center_sub} mm")

        # Try various combinations
        formulas = [
            ("Sub[2] (low byte)", d2 & 0xFF),
            ("Sub[2] >> 8", d2 >> 8),
            ("Sub[2] full", d2),
            ("Sub[0]", d0),
            ("Sub[4] >> 8", d4 >> 8),
            ("(Sub[4] >> 8) * 10", (d4 >> 8) * 10),
            ("(Sub[2] low) + (Sub[4] high) * 256", (d2 & 0xFF) + ((d4 >> 8) << 8)),
        ]

        print("\n    Formula attempts:")
        for name, value in formulas:
            error = abs(value - sdk_center_sub)
            print(f"      {name}: {value} (error: {error} mm)")

        # Correlation analysis across all pixels
        print("\n    Correlation analysis (row 120):")
        sdk_row = sdk_subsampled[120, :]

        for i in range(VALUES_PER_PIXEL):
            raw_sub = row_pixels[:, i].astype(float)
            corr = np.corrcoef(raw_sub, sdk_row)[0, 1]
            print(f"      Sub[{i}] vs SDK: r={corr:.4f}")

        # Try byte-level correlations
        print("\n    Byte-level correlation (row 120):")
        for i in range(VALUES_PER_PIXEL):
            raw_low = (row_pixels[:, i] & 0xFF).astype(float)
            raw_high = (row_pixels[:, i] >> 8).astype(float)
            corr_low = np.corrcoef(raw_low, sdk_row)[0, 1]
            corr_high = np.corrcoef(raw_high, sdk_row)[0, 1]
            print(f"      Sub[{i}] low byte: r={corr_low:.4f}, high byte: r={corr_high:.4f}")

def analyze_5byte_packed(raw_path, sdk_path):
    """Try 5-byte packed format from SDK documentation"""

    with open(raw_path, 'rb') as f:
        raw_data = f.read()

    with open(sdk_path, 'rb') as f:
        sdk_data = f.read()
    sdk_depth = np.frombuffer(sdk_data, dtype=np.uint16).reshape(480, 640)

    print(f"\n5-byte packed analysis:")

    # Skip header row (3200 bytes)
    data_start = 3200

    # Each row: 1600 bytes depth + 1600 bytes amplitude
    # 1600 bytes / (5 bytes / 2 pixels) = 640 pixels per row

    # But that doesn't match 771,200 bytes...
    # 771,200 bytes = 241 rows × 3200 bytes/row
    # 3200 bytes/row = 1600 uint16 values/row

    # Actually the raw data is already uint16, not packed bytes
    # So the "5-byte packed" format is NOT what V4L2 delivers

    # V4L2/UVC delivers: 1600 uint16 per row = 3200 bytes
    # SDK expects: 5 bytes per 2 pixels (packed)

    # This means V4L2 has ALREADY unpacked the data!
    # Or the format is completely different

    print(f"  V4L2 frame: {len(raw_data)} bytes")
    print(f"  Rows: 241 (1 header + 240 data)")
    print(f"  Bytes per row: 3200")
    print(f"  uint16 per row: 1600")

    # If 5 uint16 per pixel (as previously observed):
    print(f"  Pixels per row (at 5 uint16/pixel): 320")
    print(f"  Total pixels: 320 × 240 = 76,800")
    print(f"  SDK output: 640 × 480 = 307,200 (4× more)")

    # Try 10-byte (5 uint16) structure per pixel
    raw_u8 = np.frombuffer(raw_data, dtype=np.uint8)

    # Skip header row
    data_u8 = raw_u8[3200:]

    # Reshape to 240 rows × 3200 bytes
    rows_u8 = data_u8.reshape(240, 3200)

    # Try SDK 5-byte packed formula on first half of each row (depth section)
    # Row structure might be: [depth_packed][amplitude_packed]
    # 1600 bytes depth / 5 bytes = 320 pixels

    print("\n  Trying 5-byte packed formula on depth section:")

    center_row = rows_u8[120]
    depth_section = center_row[:1600]  # First 1600 bytes = depth

    # Unpack using SDK formula
    unpacked_depth = []
    for i in range(0, len(depth_section), 5):
        if i + 4 >= len(depth_section):
            break
        b0 = depth_section[i]
        b1 = depth_section[i + 1]
        b2 = depth_section[i + 2]
        b3 = depth_section[i + 3]
        b4 = depth_section[i + 4]

        # Pixel 0
        coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2)
        fine0 = (b4 & 0x03) | (b0 << 2)
        depth0 = (coarse0 << 10) + fine0

        # Pixel 1
        coarse1 = (b4 >> 6) | (b3 << 2)
        fine1 = ((b4 >> 4) & 0x03) | (b2 << 2)
        depth1 = (coarse1 << 10) + fine1

        unpacked_depth.append(depth0)
        unpacked_depth.append(depth1)

    unpacked_depth = np.array(unpacked_depth)
    print(f"  Unpacked {len(unpacked_depth)} depth values")
    print(f"  Value range: {unpacked_depth.min()} - {unpacked_depth.max()}")

    # Compare with SDK
    sdk_row = sdk_depth[240, ::2]  # Row 240 (from raw row 120), every other pixel
    print(f"  SDK row (subsampled): {len(sdk_row)} values")
    print(f"  SDK range: {sdk_row.min()} - {sdk_row.max()}")

    # Correlation
    min_len = min(len(unpacked_depth), len(sdk_row))
    corr = np.corrcoef(unpacked_depth[:min_len], sdk_row[:min_len])[0, 1]
    print(f"  Correlation: r={corr:.4f}")

    # Check center pixel
    center_idx = 160
    if center_idx < len(unpacked_depth):
        print(f"\n  Center pixel comparison:")
        print(f"    Unpacked depth: {unpacked_depth[center_idx]}")
        print(f"    SDK depth (240, 320): {sdk_depth[240, 320]} mm")

if __name__ == "__main__":
    # Use raw frame 4 (first with data) and SDK depth 0
    raw_path = "data/sync_capture/raw_0004.bin"
    sdk_path = "data/sync_depth_0.raw"

    analyze_frame(raw_path, sdk_path)
    analyze_5byte_packed(raw_path, sdk_path)

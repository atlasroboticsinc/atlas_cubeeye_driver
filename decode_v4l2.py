#!/usr/bin/env python3
"""
Decode V4L2 data using SDK's 5-byte packed formula
The V4L2 frame might need to be interpreted as raw bytes, not uint16
"""

import numpy as np

def unpack_5byte(data, num_pixels):
    """
    Apply SDK 5-byte unpacking formula
    5 bytes → 2 pixels (coarse + fine → depth)
    """
    depths = []
    for i in range(0, len(data) - 4, 5):
        b0, b1, b2, b3, b4 = data[i:i+5]

        # Pixel 0
        coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2)
        fine0 = (b4 & 0x03) | (b0 << 2)
        depth0 = (coarse0 << 10) + fine0

        # Pixel 1
        coarse1 = (b4 >> 6) | (b3 << 2)
        fine1 = ((b4 >> 4) & 0x03) | (b2 << 2)
        depth1 = (coarse1 << 10) + fine1

        depths.extend([depth0, depth1])

        if len(depths) >= num_pixels:
            break

    return np.array(depths[:num_pixels], dtype=np.uint16)

def decode_row_as_bytes(raw_path, sdk_path):
    """Try decoding V4L2 row data as raw bytes"""

    # Read files
    with open(raw_path, 'rb') as f:
        raw_data = f.read()

    with open(sdk_path, 'rb') as f:
        sdk_data = f.read()

    # Parse
    raw_u8 = np.frombuffer(raw_data, dtype=np.uint8)
    sdk_depth = np.frombuffer(sdk_data, dtype=np.uint16).reshape(480, 640)

    print(f"Raw data: {len(raw_u8)} bytes")
    print(f"SDK depth: {sdk_depth.shape}")

    # Frame structure:
    # - Row 0: Header (3200 bytes)
    # - Rows 1-240: Data (3200 bytes each)
    HEADER_SIZE = 3200
    BYTES_PER_ROW = 3200

    # Skip header
    data_bytes = raw_u8[HEADER_SIZE:]

    print(f"\nData section: {len(data_bytes)} bytes ({len(data_bytes)//BYTES_PER_ROW} rows)")

    # Each data row might be: [depth_section][amplitude_section]
    # or interleaved data

    # Try interpreting each row as:
    # Option 1: First 1600 bytes = depth, next 1600 bytes = amplitude
    # Option 2: Interleaved (YUYV-like: D0, A0, D1, A1, ...)
    # Option 3: Full row is depth + amplitude interleaved at pixel level

    print("\n=== Option 1: Split row (first half depth, second half amplitude) ===")
    decode_split_row(data_bytes, sdk_depth, BYTES_PER_ROW)

    print("\n=== Option 2: Interleaved DADA... ===")
    decode_interleaved(data_bytes, sdk_depth, BYTES_PER_ROW)

    print("\n=== Option 3: 10 bytes per pixel (D+A) ===")
    decode_10byte_pixel(data_bytes, sdk_depth, BYTES_PER_ROW)

    print("\n=== Option 4: Raw bytes as-is through 5-byte formula ===")
    decode_raw_5byte(data_bytes, sdk_depth, BYTES_PER_ROW)

def decode_split_row(data_bytes, sdk_depth, bytes_per_row):
    """Interpret row as [depth_1600][amplitude_1600]"""
    try:
        # Row 120 (center)
        row_idx = 120
        row_start = row_idx * bytes_per_row
        row_data = data_bytes[row_start:row_start + bytes_per_row]

        # First 1600 bytes = depth section
        depth_section = row_data[:1600]

        # Apply 5-byte unpacking: 1600 bytes / 5 = 320 pairs = 640 pixels
        decoded = unpack_5byte(depth_section, 640)

        print(f"Decoded row: {len(decoded)} pixels")
        print(f"Decoded range: {decoded.min()} - {decoded.max()}")

        # Compare with SDK row (row 240 = 2*120)
        sdk_row = sdk_depth[240, :]
        print(f"SDK row range: {sdk_row.min()} - {sdk_row.max()}")

        # Correlation
        corr = np.corrcoef(decoded.astype(float), sdk_row.astype(float))[0, 1]
        print(f"Correlation: r = {corr:.4f}")

        # RMSE
        rmse = np.sqrt(np.mean((decoded.astype(float) - sdk_row.astype(float)) ** 2))
        print(f"RMSE: {rmse:.2f} mm")

        # Check center pixels
        print(f"Center: decoded={decoded[320]}, SDK={sdk_row[320]}")

    except Exception as e:
        print(f"Error: {e}")

def decode_interleaved(data_bytes, sdk_depth, bytes_per_row):
    """Interpret row as DADADADA... (alternating depth/amplitude bytes)"""
    try:
        row_idx = 120
        row_start = row_idx * bytes_per_row
        row_data = data_bytes[row_start:row_start + bytes_per_row]

        # Extract even bytes (depth) and odd bytes (amplitude)
        depth_bytes = row_data[::2]  # 1600 bytes
        amp_bytes = row_data[1::2]   # 1600 bytes

        # Apply 5-byte formula
        decoded = unpack_5byte(depth_bytes, 640)

        print(f"Decoded row: {len(decoded)} pixels")
        print(f"Decoded range: {decoded.min()} - {decoded.max()}")

        sdk_row = sdk_depth[240, :]
        corr = np.corrcoef(decoded.astype(float), sdk_row.astype(float))[0, 1]
        print(f"Correlation: r = {corr:.4f}")

        rmse = np.sqrt(np.mean((decoded.astype(float) - sdk_row.astype(float)) ** 2))
        print(f"RMSE: {rmse:.2f} mm")

    except Exception as e:
        print(f"Error: {e}")

def decode_10byte_pixel(data_bytes, sdk_depth, bytes_per_row):
    """Interpret 10 bytes per pixel (5 uint16 → combined depth/amp)"""
    try:
        row_idx = 120
        row_start = row_idx * bytes_per_row
        row_data = data_bytes[row_start:row_start + bytes_per_row]

        # 3200 bytes / 10 = 320 pixels
        # Need to extract depth from 10-byte structure

        decoded = []
        for i in range(0, len(row_data), 10):
            pixel_bytes = row_data[i:i+10]
            if len(pixel_bytes) < 10:
                break

            # Parse as 5 uint16 (little-endian)
            s0 = pixel_bytes[0] | (pixel_bytes[1] << 8)
            s1 = pixel_bytes[2] | (pixel_bytes[3] << 8)
            s2 = pixel_bytes[4] | (pixel_bytes[5] << 8)
            s3 = pixel_bytes[6] | (pixel_bytes[7] << 8)
            s4 = pixel_bytes[8] | (pixel_bytes[9] << 8)

            # Try various depth formulas
            # Formula: The low byte of S2 might contain phase info
            s2_lo = s2 & 0xFF
            s2_hi = s2 >> 8
            s4_hi = s4 >> 8

            # Attempt: phase + cycle * wavelength
            # Assume 100mm wavelength for CRT
            depth = s2_lo + s4_hi * 100

            decoded.append(depth)

        decoded = np.array(decoded, dtype=np.uint16)
        print(f"Decoded row: {len(decoded)} pixels")
        print(f"Decoded range: {decoded.min()} - {decoded.max()}")

        # SDK is 640 wide, our decode is 320 - subsample SDK
        sdk_row = sdk_depth[240, ::2]  # Every other pixel
        corr = np.corrcoef(decoded.astype(float), sdk_row.astype(float))[0, 1]
        print(f"Correlation: r = {corr:.4f}")

        rmse = np.sqrt(np.mean((decoded.astype(float) - sdk_row.astype(float)) ** 2))
        print(f"RMSE: {rmse:.2f} mm")

        print(f"Sample: decoded[160]={decoded[160]}, SDK[160]={sdk_row[160]}")

    except Exception as e:
        print(f"Error: {e}")

def decode_raw_5byte(data_bytes, sdk_depth, bytes_per_row):
    """Apply 5-byte formula directly to raw data"""
    try:
        row_idx = 120
        row_start = row_idx * bytes_per_row
        row_data = data_bytes[row_start:row_start + bytes_per_row]

        # Apply 5-byte formula to entire row: 3200/5 * 2 = 1280 values
        decoded = unpack_5byte(row_data, 1280)

        print(f"Decoded row: {len(decoded)} pixels")
        print(f"Decoded range: {decoded.min()} - {decoded.max()}")

        # Take every other pixel to get 640 (matching SDK width)
        decoded_640 = decoded[::2]
        sdk_row = sdk_depth[240, :]

        corr = np.corrcoef(decoded_640.astype(float), sdk_row.astype(float))[0, 1]
        print(f"Correlation (every 2nd pixel): r = {corr:.4f}")

        # Try taking first 640
        decoded_first = decoded[:640]
        corr_first = np.corrcoef(decoded_first.astype(float), sdk_row.astype(float))[0, 1]
        print(f"Correlation (first 640): r = {corr_first:.4f}")

        # Try second 640
        decoded_second = decoded[640:1280]
        corr_second = np.corrcoef(decoded_second.astype(float), sdk_row.astype(float))[0, 1]
        print(f"Correlation (second 640): r = {corr_second:.4f}")

        print(f"Sample decoded: {decoded[:10]}")
        print(f"Sample SDK: {sdk_row[:10]}")

    except Exception as e:
        print(f"Error: {e}")

def analyze_byte_patterns(raw_path):
    """Look for patterns in the raw byte data"""
    with open(raw_path, 'rb') as f:
        raw_data = f.read()

    raw_u8 = np.frombuffer(raw_data, dtype=np.uint8)
    data_bytes = raw_u8[3200:]  # Skip header

    print("\n=== Byte pattern analysis ===")

    # Analyze row 120
    row_data = data_bytes[120*3200:121*3200]

    # Look at byte distribution
    print(f"\nByte statistics (row 120):")
    print(f"  Min: {row_data.min()}, Max: {row_data.max()}, Mean: {row_data.mean():.1f}")

    # Look at even/odd byte patterns
    even_bytes = row_data[::2]
    odd_bytes = row_data[1::2]
    print(f"  Even bytes: min={even_bytes.min()}, max={even_bytes.max()}, mean={even_bytes.mean():.1f}")
    print(f"  Odd bytes: min={odd_bytes.min()}, max={odd_bytes.max()}, mean={odd_bytes.mean():.1f}")

    # Look at patterns in groups of 10 (5 uint16)
    print(f"\nPer-pixel byte pattern (pixel 160):")
    pixel_start = 160 * 10
    for i in range(10):
        print(f"  Byte {i}: {row_data[pixel_start + i]}")

    # Print as uint16
    print(f"\nAs uint16:")
    for i in range(5):
        lo = row_data[pixel_start + i*2]
        hi = row_data[pixel_start + i*2 + 1]
        val = lo | (hi << 8)
        print(f"  S{i}: {val} (lo={lo}, hi={hi})")

if __name__ == "__main__":
    raw_path = "data/sync_capture/raw_0004.bin"
    sdk_path = "data/sync_depth_0.raw"

    decode_row_as_bytes(raw_path, sdk_path)
    analyze_byte_patterns(raw_path)

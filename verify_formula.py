#!/usr/bin/env python3
"""
Verify the depth extraction formula:
- Row structure: [amplitude_1600][depth_1600]
- Apply 5-byte unpacking to depth section
- 1600 bytes / 5 * 2 = 640 depth pixels per row
"""

import numpy as np

def unpack_5byte(data):
    """
    Apply SDK 5-byte unpacking formula
    5 bytes → 2 pixels
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

    return np.array(depths, dtype=np.uint16)

def decode_frame(raw_path):
    """Decode entire frame using discovered formula"""
    with open(raw_path, 'rb') as f:
        raw_data = f.read()

    raw_u8 = np.frombuffer(raw_data, dtype=np.uint8)

    # Skip header (3200 bytes)
    data_bytes = raw_u8[3200:]

    # Each row is 3200 bytes: [amp_1600][depth_1600]
    BYTES_PER_ROW = 3200

    decoded_frame = []
    for row_idx in range(240):
        row_start = row_idx * BYTES_PER_ROW
        row_data = data_bytes[row_start:row_start + BYTES_PER_ROW]

        # Extract depth section (second 1600 bytes)
        depth_section = row_data[1600:3200]

        # Apply 5-byte formula
        decoded_row = unpack_5byte(depth_section)

        decoded_frame.append(decoded_row)

    return np.array(decoded_frame, dtype=np.uint16)

def interpolate_to_480(decoded_240):
    """2× vertical interpolation to match SDK output"""
    # Simple nearest neighbor interpolation
    interpolated = np.repeat(decoded_240, 2, axis=0)
    return interpolated

def load_sdk_depth(sdk_path):
    """Load SDK depth frame"""
    with open(sdk_path, 'rb') as f:
        sdk_data = f.read()
    return np.frombuffer(sdk_data, dtype=np.uint16).reshape(480, 640)

def compare_frames(decoded, sdk_depth):
    """Compare decoded frame with SDK output"""
    print(f"Decoded shape: {decoded.shape}")
    print(f"SDK shape: {sdk_depth.shape}")

    # If decoded is 240 rows, interpolate to 480
    if decoded.shape[0] == 240:
        decoded_full = interpolate_to_480(decoded)
        print(f"Interpolated shape: {decoded_full.shape}")
    else:
        decoded_full = decoded

    # Calculate statistics
    valid_mask = (sdk_depth > 0) & (sdk_depth < 7500)
    print(f"Valid pixels: {valid_mask.sum()} / {valid_mask.size}")

    # Compare
    diff = decoded_full.astype(float) - sdk_depth.astype(float)

    print(f"\nDecoded range: {decoded_full.min()} - {decoded_full.max()}")
    print(f"SDK range: {sdk_depth.min()} - {sdk_depth.max()}")

    valid_diff = diff[valid_mask]
    print(f"\nDifference (where SDK valid):")
    print(f"  Mean: {np.mean(valid_diff):.2f} mm")
    print(f"  Std: {np.std(valid_diff):.2f} mm")
    print(f"  Min: {np.min(valid_diff):.2f} mm")
    print(f"  Max: {np.max(valid_diff):.2f} mm")
    print(f"  RMSE: {np.sqrt(np.mean(valid_diff**2)):.2f} mm")

    # Correlation
    corr = np.corrcoef(decoded_full[valid_mask].flatten(),
                       sdk_depth[valid_mask].flatten())[0, 1]
    print(f"\nCorrelation: r = {corr:.6f}")

    # Sample pixels
    print(f"\nSample pixels:")
    test_points = [
        (240, 320, "center"),
        (120, 160, "top-left quarter"),
        (360, 480, "bottom-right quarter"),
    ]
    for row, col, name in test_points:
        dec = decoded_full[row, col]
        sdk = sdk_depth[row, col]
        print(f"  {name}: decoded={dec}, SDK={sdk}, diff={dec-sdk}")

    return decoded_full, diff

def analyze_gradient_correction(decoded, sdk_depth):
    """Check if gradient correction is needed"""
    print("\n" + "="*60)
    print("Gradient correction analysis:")
    print("="*60)

    valid_mask = (sdk_depth > 100) & (sdk_depth < 7000)

    # Compare at different depth ranges
    ranges = [(100, 500), (500, 1000), (1000, 2000), (2000, 3000), (3000, 5000)]

    for d_min, d_max in ranges:
        range_mask = valid_mask & (sdk_depth >= d_min) & (sdk_depth < d_max)
        if range_mask.sum() > 100:
            dec_vals = decoded[range_mask]
            sdk_vals = sdk_depth[range_mask]
            mean_diff = np.mean(dec_vals.astype(float) - sdk_vals.astype(float))
            print(f"  {d_min}-{d_max}mm: mean_diff = {mean_diff:+.2f}mm "
                  f"(n={range_mask.sum()})")

def process_multiple_frames(raw_paths, sdk_paths):
    """Process multiple frames to verify consistency"""
    print("\n" + "="*60)
    print("Multi-frame verification:")
    print("="*60)

    all_rmse = []
    all_corr = []

    for i, (raw_path, sdk_path) in enumerate(zip(raw_paths, sdk_paths)):
        decoded = decode_frame(raw_path)
        sdk_depth = load_sdk_depth(sdk_path)

        decoded_full = interpolate_to_480(decoded)
        valid_mask = (sdk_depth > 0) & (sdk_depth < 7500)

        diff = decoded_full.astype(float) - sdk_depth.astype(float)
        rmse = np.sqrt(np.mean(diff[valid_mask]**2))
        corr = np.corrcoef(decoded_full[valid_mask].flatten(),
                          sdk_depth[valid_mask].flatten())[0, 1]

        all_rmse.append(rmse)
        all_corr.append(corr)

        print(f"  Frame {i}: RMSE={rmse:.2f}mm, r={corr:.6f}")

    print(f"\n  Average RMSE: {np.mean(all_rmse):.2f} mm")
    print(f"  Average correlation: {np.mean(all_corr):.6f}")

if __name__ == "__main__":
    # Process first frame pair
    raw_path = "data/sync_capture/raw_0004.bin"
    sdk_path = "data/sync_depth_0.raw"

    print("="*60)
    print("Depth extraction formula verification")
    print("="*60)

    decoded = decode_frame(raw_path)
    sdk_depth = load_sdk_depth(sdk_path)

    decoded_full, diff = compare_frames(decoded, sdk_depth)

    analyze_gradient_correction(decoded_full, sdk_depth)

    # Multi-frame verification
    raw_paths = [f"data/sync_capture/raw_{i:04d}.bin" for i in range(4, 9)]
    sdk_paths = [f"data/sync_depth_{i}.raw" for i in range(5)]
    process_multiple_frames(raw_paths, sdk_paths)

    # Save decoded frame for visualization
    np.save("data/decoded_depth.npy", decoded_full)
    print(f"\nSaved decoded frame to data/decoded_depth.npy")

#!/usr/bin/env python3
"""
Analyze hand waving test data - compare decoded depth with SDK output
"""

import numpy as np
import os
from depth_extractor import DepthExtractor

def load_raw_frame(path):
    with open(path, 'rb') as f:
        return f.read()

def load_sdk_depth(path):
    with open(path, 'rb') as f:
        return np.frombuffer(f.read(), dtype=np.uint16).reshape(480, 640)

def analyze_frame(raw_path, sdk_path, extractor, frame_idx):
    """Analyze a single frame pair"""
    raw_data = load_raw_frame(raw_path)
    sdk_depth = load_sdk_depth(sdk_path)

    # Extract depth using our formula
    decoded = extractor.extract_depth(raw_data, interpolate=True)

    # Compare only valid pixels
    valid_mask = (sdk_depth > 50) & (sdk_depth < 7000)

    if valid_mask.sum() < 1000:
        return None  # Not enough valid pixels

    diff = decoded.astype(float) - sdk_depth.astype(float)

    # Statistics
    rmse = np.sqrt(np.mean(diff[valid_mask]**2))
    mean_diff = np.mean(diff[valid_mask])
    corr = np.corrcoef(decoded[valid_mask].flatten(),
                       sdk_depth[valid_mask].flatten())[0, 1]

    # Center pixel (use int to avoid overflow)
    center_sdk = int(sdk_depth[240, 320])
    center_dec = int(decoded[240, 320])

    return {
        'frame': frame_idx,
        'rmse': rmse,
        'mean_diff': mean_diff,
        'correlation': corr,
        'center_sdk': center_sdk,
        'center_dec': center_dec,
        'center_err': center_dec - center_sdk,
        'valid_pixels': valid_mask.sum(),
        'depth_range': (int(sdk_depth[valid_mask].min()), int(sdk_depth[valid_mask].max()))
    }

def main():
    print("=" * 70)
    print("Hand Waving Test Analysis")
    print("=" * 70)

    extractor = DepthExtractor(apply_gradient_correction=True)

    # Find all frame pairs (raw frames 4+ correspond to SDK frames 0+)
    raw_dir = "data/hand_test/raw"
    raw_files = sorted([f for f in os.listdir(raw_dir) if f.endswith('.bin')])

    results = []
    depth_bins = {
        '200-500mm': [],
        '500-1000mm': [],
        '1000-2000mm': [],
        '2000-3000mm': [],
        '3000-5000mm': [],
        '5000-7000mm': [],
    }

    print(f"\nProcessing {len(raw_files)} raw frames...")
    print("-" * 70)

    for i, raw_file in enumerate(raw_files):
        raw_idx = int(raw_file.split('_')[1].split('.')[0])

        # Raw frames 0-3 are empty, raw frame 4 = SDK frame 0
        sdk_idx = raw_idx - 4
        if sdk_idx < 0:
            continue

        sdk_path = f"data/sync_depth_{sdk_idx}.raw"
        raw_path = os.path.join(raw_dir, raw_file)

        if not os.path.exists(sdk_path):
            continue

        result = analyze_frame(raw_path, sdk_path, extractor, sdk_idx)
        if result:
            results.append(result)

            # Categorize by depth (only if center depth is valid)
            center_depth = result['center_sdk']
            if center_depth > 50:  # Valid depth
                for bin_name, bin_list in depth_bins.items():
                    low, high = map(int, bin_name.replace('mm', '').split('-'))
                    if low <= center_depth < high:
                        bin_list.append(result)
                        break

            # Print every 10th frame
            if sdk_idx % 10 == 0:
                print(f"Frame {sdk_idx:3d}: SDK={result['center_sdk']:4d}mm, "
                      f"Decoded={result['center_dec']:4d}mm, "
                      f"Error={result['center_err']:+4d}mm, "
                      f"RMSE={result['rmse']:.1f}mm, r={result['correlation']:.4f}")

    print("-" * 70)
    print(f"\nProcessed {len(results)} frame pairs\n")

    # Overall statistics
    all_rmse = [r['rmse'] for r in results]
    all_corr = [r['correlation'] for r in results]
    all_center_err = [r['center_err'] for r in results]

    print("=" * 70)
    print("OVERALL STATISTICS")
    print("=" * 70)
    print(f"  Frames analyzed: {len(results)}")
    print(f"  RMSE:        mean={np.mean(all_rmse):.2f}mm, "
          f"min={np.min(all_rmse):.2f}mm, max={np.max(all_rmse):.2f}mm")
    print(f"  Correlation: mean={np.mean(all_corr):.6f}, "
          f"min={np.min(all_corr):.6f}, max={np.max(all_corr):.6f}")
    print(f"  Center err:  mean={np.mean(all_center_err):+.1f}mm, "
          f"std={np.std(all_center_err):.1f}mm")

    # Per-depth-range statistics
    print("\n" + "=" * 70)
    print("ACCURACY BY DEPTH RANGE")
    print("=" * 70)

    for bin_name, bin_results in depth_bins.items():
        if len(bin_results) > 0:
            rmse_list = [r['rmse'] for r in bin_results]
            err_list = [r['center_err'] for r in bin_results]
            corr_list = [r['correlation'] for r in bin_results]

            print(f"\n  {bin_name}: ({len(bin_results)} frames)")
            print(f"    RMSE:       mean={np.mean(rmse_list):.2f}mm")
            print(f"    Center err: mean={np.mean(err_list):+.1f}mm, std={np.std(err_list):.1f}mm")
            print(f"    Correlation: mean={np.mean(corr_list):.6f}")

    # Find best and worst frames
    print("\n" + "=" * 70)
    print("NOTABLE FRAMES")
    print("=" * 70)

    sorted_by_rmse = sorted(results, key=lambda x: x['rmse'])
    print("\n  Best RMSE:")
    for r in sorted_by_rmse[:3]:
        print(f"    Frame {r['frame']}: RMSE={r['rmse']:.2f}mm, depth={r['center_sdk']}mm")

    print("\n  Worst RMSE:")
    for r in sorted_by_rmse[-3:]:
        print(f"    Frame {r['frame']}: RMSE={r['rmse']:.2f}mm, depth={r['center_sdk']}mm")

    # Close-range accuracy (hand detection) - only where SDK has valid center depth
    close_frames = [r for r in results if 100 < r['center_sdk'] < 600]
    if close_frames:
        print("\n" + "=" * 70)
        print("CLOSE-RANGE ACCURACY (<600mm) - Hand Detection")
        print("=" * 70)
        close_rmse = [r['rmse'] for r in close_frames]
        close_err = [r['center_err'] for r in close_frames]
        print(f"  Frames: {len(close_frames)}")
        print(f"  RMSE: mean={np.mean(close_rmse):.2f}mm")
        print(f"  Center error: mean={np.mean(close_err):+.1f}mm, std={np.std(close_err):.1f}mm")

        print("\n  Sample close-range frames:")
        for r in close_frames[:5]:
            print(f"    Frame {r['frame']}: SDK={r['center_sdk']}mm, "
                  f"Decoded={r['center_dec']}mm, err={r['center_err']:+d}mm")

if __name__ == "__main__":
    main()

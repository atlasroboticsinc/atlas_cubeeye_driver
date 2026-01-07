#!/usr/bin/env python3
"""
Complete depth extraction from CubeEye I200D V4L2 frames
Including gradient correction

Frame format:
- Row 0: Header (3200 bytes)
- Rows 1-240: [amplitude_1600 bytes][depth_1600 bytes]
- Apply 5-byte unpacking to depth section

Gradient correction:
- 14-coefficient polynomial
- Applied per-pixel based on raw depth value
"""

import numpy as np
import json

# Polynomial coefficients for gradient correction
GRADIENT_COEFFS = [
    -9.46306765e-08,   # x^13
    5.40828695e-06,    # x^12
    -0.000133821166,   # x^11
    0.00189463573,     # x^10
    -0.0170465988,     # x^9
    0.102187397,       # x^8
    -0.415584587,      # x^7
    1.14433615,        # x^6
    -2.09044109,       # x^5
    2.42940077,        # x^4
    -1.66835357,       # x^3
    0.587854516,       # x^2
    -0.076622637,      # x^1
    0.000344344841,    # x^0
]

class DepthExtractor:
    def __init__(self, apply_gradient_correction=True, max_depth_mm=7500):
        self.apply_gradient = apply_gradient_correction
        self.max_depth = max_depth_mm

        # Pre-compute gradient correction lookup table for efficiency
        self.gradient_lut = self._build_gradient_lut()

    def _build_gradient_lut(self):
        """Build lookup table for gradient correction (7501 entries)"""
        lut = np.zeros(self.max_depth + 1, dtype=np.int16)

        for depth_mm in range(self.max_depth + 1):
            x = depth_mm / 1000.0  # Convert to meters

            # Evaluate polynomial
            correction = 0.0
            for i, coeff in enumerate(GRADIENT_COEFFS):
                correction += coeff * (x ** (13 - i))

            # Correction is in meters, convert to mm
            correction_mm = correction * 1000.0

            # Store as offset (will be subtracted from raw depth)
            lut[depth_mm] = int(round(correction_mm))

        return lut

    def unpack_5byte_row(self, depth_bytes):
        """
        Apply SDK 5-byte unpacking formula to a row of depth data
        1600 bytes → 640 depth values
        """
        depths = np.zeros(640, dtype=np.uint16)
        idx = 0

        for i in range(0, len(depth_bytes) - 4, 5):
            b0 = depth_bytes[i]
            b1 = depth_bytes[i + 1]
            b2 = depth_bytes[i + 2]
            b3 = depth_bytes[i + 3]
            b4 = depth_bytes[i + 4]

            # Pixel 0
            coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2)
            fine0 = (b4 & 0x03) | (b0 << 2)
            depth0 = (coarse0 << 10) + fine0

            # Pixel 1
            coarse1 = (b4 >> 6) | (b3 << 2)
            fine1 = ((b4 >> 4) & 0x03) | (b2 << 2)
            depth1 = (coarse1 << 10) + fine1

            if idx < 640:
                depths[idx] = depth0
                idx += 1
            if idx < 640:
                depths[idx] = depth1
                idx += 1

        return depths

    def apply_gradient_correction(self, depth_frame):
        """Apply gradient correction using lookup table"""
        corrected = depth_frame.astype(np.int32)

        # Clip to valid range for LUT lookup
        clipped = np.clip(corrected, 0, self.max_depth)

        # Apply correction - ADD the correction value
        # (correction is negative for larger depths, positive for smaller)
        correction = self.gradient_lut[clipped]
        corrected = corrected + correction

        # Clip result to valid range
        corrected = np.clip(corrected, 0, self.max_depth)

        return corrected.astype(np.uint16)

    def extract_depth(self, raw_frame_bytes, interpolate=True):
        """
        Extract depth from raw V4L2 frame

        Args:
            raw_frame_bytes: 771200 bytes of raw frame data
            interpolate: If True, interpolate to 640x480. If False, return 640x240

        Returns:
            depth_frame: uint16 depth values in mm
        """
        raw_u8 = np.frombuffer(raw_frame_bytes, dtype=np.uint8)

        # Skip header (3200 bytes)
        data_bytes = raw_u8[3200:]

        # Process each row
        BYTES_PER_ROW = 3200
        depth_frame = np.zeros((240, 640), dtype=np.uint16)

        for row_idx in range(240):
            row_start = row_idx * BYTES_PER_ROW
            row_data = data_bytes[row_start:row_start + BYTES_PER_ROW]

            # Extract depth section (second 1600 bytes)
            depth_section = row_data[1600:3200]

            # Apply 5-byte unpacking
            depth_frame[row_idx] = self.unpack_5byte_row(depth_section)

        # Apply gradient correction
        if self.apply_gradient:
            depth_frame = self.apply_gradient_correction(depth_frame)

        # Interpolate to 480 rows if requested
        if interpolate:
            depth_frame = np.repeat(depth_frame, 2, axis=0)

        return depth_frame

    def extract_amplitude(self, raw_frame_bytes, interpolate=True):
        """
        Extract amplitude from raw V4L2 frame

        Returns:
            amplitude_frame: uint16 amplitude values
        """
        raw_u8 = np.frombuffer(raw_frame_bytes, dtype=np.uint8)

        # Skip header
        data_bytes = raw_u8[3200:]

        BYTES_PER_ROW = 3200
        amplitude_frame = np.zeros((240, 640), dtype=np.uint16)

        for row_idx in range(240):
            row_start = row_idx * BYTES_PER_ROW
            row_data = data_bytes[row_start:row_start + BYTES_PER_ROW]

            # Extract amplitude section (first 1600 bytes)
            amp_section = row_data[:1600]

            # Amplitude uses same 5-byte unpacking
            amplitude_frame[row_idx] = self.unpack_5byte_row(amp_section)

        if interpolate:
            amplitude_frame = np.repeat(amplitude_frame, 2, axis=0)

        return amplitude_frame


def verify_extraction(raw_path, sdk_path, extractor):
    """Verify extraction against SDK output"""
    with open(raw_path, 'rb') as f:
        raw_data = f.read()

    with open(sdk_path, 'rb') as f:
        sdk_data = f.read()

    sdk_depth = np.frombuffer(sdk_data, dtype=np.uint16).reshape(480, 640)

    # Extract with and without gradient correction
    extractor_no_corr = DepthExtractor(apply_gradient_correction=False)
    depth_no_corr = extractor_no_corr.extract_depth(raw_data)
    depth_with_corr = extractor.extract_depth(raw_data)

    print(f"Raw: {raw_path}")
    print(f"SDK: {sdk_path}")
    print(f"Shapes: decoded={depth_with_corr.shape}, SDK={sdk_depth.shape}")

    valid_mask = (sdk_depth > 100) & (sdk_depth < 7000)

    # Compare without correction
    diff_no_corr = depth_no_corr.astype(float) - sdk_depth.astype(float)
    rmse_no_corr = np.sqrt(np.mean(diff_no_corr[valid_mask]**2))
    corr_no_corr = np.corrcoef(depth_no_corr[valid_mask].flatten(),
                               sdk_depth[valid_mask].flatten())[0, 1]

    # Compare with correction
    diff_with_corr = depth_with_corr.astype(float) - sdk_depth.astype(float)
    rmse_with_corr = np.sqrt(np.mean(diff_with_corr[valid_mask]**2))
    corr_with_corr = np.corrcoef(depth_with_corr[valid_mask].flatten(),
                                  sdk_depth[valid_mask].flatten())[0, 1]

    print(f"\nWithout gradient correction:")
    print(f"  RMSE: {rmse_no_corr:.2f} mm, r: {corr_no_corr:.6f}")
    print(f"  Mean diff: {np.mean(diff_no_corr[valid_mask]):.2f} mm")

    print(f"\nWith gradient correction:")
    print(f"  RMSE: {rmse_with_corr:.2f} mm, r: {corr_with_corr:.6f}")
    print(f"  Mean diff: {np.mean(diff_with_corr[valid_mask]):.2f} mm")

    # Gradient correction breakdown by depth
    print(f"\nGradient correction by depth range:")
    ranges = [(100, 500), (500, 1000), (1000, 2000), (2000, 3000), (3000, 5000)]
    for d_min, d_max in ranges:
        range_mask = valid_mask & (sdk_depth >= d_min) & (sdk_depth < d_max)
        if range_mask.sum() > 100:
            no_corr_diff = np.mean(diff_no_corr[range_mask])
            with_corr_diff = np.mean(diff_with_corr[range_mask])
            print(f"  {d_min}-{d_max}mm: no_corr={no_corr_diff:+.1f}mm, "
                  f"with_corr={with_corr_diff:+.1f}mm")

    # Sample pixels
    print(f"\nSample pixels (SDK, No Corr, With Corr):")
    test_points = [(240, 320), (120, 160), (360, 480)]
    for row, col in test_points:
        sdk = sdk_depth[row, col]
        no_c = depth_no_corr[row, col]
        with_c = depth_with_corr[row, col]
        print(f"  ({row},{col}): SDK={sdk}, NoCorr={no_c}, WithCorr={with_c}")

    return depth_with_corr, sdk_depth


if __name__ == "__main__":
    print("="*60)
    print("CubeEye I200D Depth Extraction Verification")
    print("="*60)

    extractor = DepthExtractor(apply_gradient_correction=True)

    # Verify on multiple frames
    for i in range(5):
        raw_path = f"data/sync_capture/raw_{i+4:04d}.bin"
        sdk_path = f"data/sync_depth_{i}.raw"

        print(f"\n{'='*60}")
        print(f"Frame {i}")
        print("="*60)

        try:
            verify_extraction(raw_path, sdk_path, extractor)
        except FileNotFoundError as e:
            print(f"Skipping: {e}")

    # Save the gradient LUT for use in CUDA kernel
    np.save("data/gradient_lut.npy", extractor.gradient_lut)
    print(f"\nSaved gradient LUT to data/gradient_lut.npy")

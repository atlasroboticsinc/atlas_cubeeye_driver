#!/usr/bin/env python3
"""
Fast vectorized depth extraction from CubeEye I200D V4L2 frames
Optimized for real-time performance using NumPy vectorization
"""

import numpy as np

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

class FastDepthExtractor:
    """Vectorized depth extractor for real-time performance"""

    def __init__(self, apply_gradient_correction=True, max_depth_mm=7500):
        self.apply_gradient = apply_gradient_correction
        self.max_depth = max_depth_mm
        self.gradient_lut = self._build_gradient_lut()

    def _build_gradient_lut(self):
        """Build lookup table for gradient correction"""
        depths = np.arange(self.max_depth + 1, dtype=np.float64) / 1000.0

        # Evaluate polynomial using Horner's method for efficiency
        correction = np.zeros_like(depths)
        for coeff in GRADIENT_COEFFS:
            correction = correction * depths + coeff

        return (correction * 1000.0).astype(np.int16)

    def extract_depth(self, raw_frame_bytes, interpolate=True):
        """
        Extract depth using fully vectorized operations

        Args:
            raw_frame_bytes: 771200 bytes of raw frame data
            interpolate: If True, return 640x480. If False, return 640x240

        Returns:
            depth_frame: uint16 depth values in mm
        """
        # Convert to uint8 array
        raw = np.frombuffer(raw_frame_bytes, dtype=np.uint8)

        # Skip header row (3200 bytes), reshape to rows
        data = raw[3200:].reshape(240, 3200)

        # Extract depth section (second half of each row: bytes 1600-3200)
        depth_bytes = data[:, 1600:3200]  # Shape: (240, 1600)

        # Reshape for 5-byte group processing
        # 1600 bytes = 320 groups of 5 bytes
        depth_bytes = depth_bytes.reshape(240, 320, 5)

        # Extract individual bytes (vectorized)
        b0 = depth_bytes[:, :, 0].astype(np.uint32)
        b1 = depth_bytes[:, :, 1].astype(np.uint32)
        b2 = depth_bytes[:, :, 2].astype(np.uint32)
        b3 = depth_bytes[:, :, 3].astype(np.uint32)
        b4 = depth_bytes[:, :, 4].astype(np.uint32)

        # Pixel 0 extraction (vectorized)
        coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2)
        fine0 = (b4 & 0x03) | (b0 << 2)
        depth0 = (coarse0 << 10) + fine0

        # Pixel 1 extraction (vectorized)
        coarse1 = (b4 >> 6) | (b3 << 2)
        fine1 = ((b4 >> 4) & 0x03) | (b2 << 2)
        depth1 = (coarse1 << 10) + fine1

        # Interleave pixel 0 and pixel 1 to get 640 pixels per row
        depth_frame = np.empty((240, 640), dtype=np.uint16)
        depth_frame[:, 0::2] = depth0.astype(np.uint16)
        depth_frame[:, 1::2] = depth1.astype(np.uint16)

        # Apply gradient correction
        if self.apply_gradient:
            depth_i32 = depth_frame.astype(np.int32)
            clipped = np.clip(depth_i32, 0, self.max_depth)
            depth_i32 = depth_i32 + self.gradient_lut[clipped]
            depth_frame = np.clip(depth_i32, 0, self.max_depth).astype(np.uint16)

        # Interpolate to 480 rows
        if interpolate:
            depth_frame = np.repeat(depth_frame, 2, axis=0)

        return depth_frame

    def extract_amplitude(self, raw_frame_bytes, interpolate=True):
        """Extract amplitude using vectorized operations"""
        raw = np.frombuffer(raw_frame_bytes, dtype=np.uint8)
        data = raw[3200:].reshape(240, 3200)

        # Amplitude is first half (bytes 0-1600)
        amp_bytes = data[:, :1600].reshape(240, 320, 5)

        b0 = amp_bytes[:, :, 0].astype(np.uint32)
        b1 = amp_bytes[:, :, 1].astype(np.uint32)
        b2 = amp_bytes[:, :, 2].astype(np.uint32)
        b3 = amp_bytes[:, :, 3].astype(np.uint32)
        b4 = amp_bytes[:, :, 4].astype(np.uint32)

        coarse0 = ((b4 >> 2) & 0x03) | (b1 << 2)
        fine0 = (b4 & 0x03) | (b0 << 2)
        amp0 = (coarse0 << 10) + fine0

        coarse1 = (b4 >> 6) | (b3 << 2)
        fine1 = ((b4 >> 4) & 0x03) | (b2 << 2)
        amp1 = (coarse1 << 10) + fine1

        amp_frame = np.empty((240, 640), dtype=np.uint16)
        amp_frame[:, 0::2] = amp0.astype(np.uint16)
        amp_frame[:, 1::2] = amp1.astype(np.uint16)

        if interpolate:
            amp_frame = np.repeat(amp_frame, 2, axis=0)

        return amp_frame


# Make it the default when imported
DepthExtractor = FastDepthExtractor


if __name__ == "__main__":
    import time
    import sys

    # Benchmark
    raw_file = sys.argv[1] if len(sys.argv) > 1 else "standalone_capture2/frame_0.bin"

    with open(raw_file, 'rb') as f:
        raw_data = f.read()

    print(f"Frame size: {len(raw_data)} bytes")

    extractor = FastDepthExtractor(apply_gradient_correction=True)

    # Warmup
    for _ in range(5):
        extractor.extract_depth(raw_data)

    # Benchmark
    times = []
    for _ in range(100):
        t0 = time.perf_counter()
        depth = extractor.extract_depth(raw_data)
        times.append(time.perf_counter() - t0)

    times = np.array(times) * 1000
    print(f"Extraction time: {times.mean():.2f} ms (std: {times.std():.2f})")
    print(f"Max FPS: {1000/times.mean():.1f}")
    print(f"Depth shape: {depth.shape}")
    print(f"Center depth: {depth[240, 320]} mm")

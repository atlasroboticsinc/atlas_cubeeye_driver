#!/usr/bin/env python3
"""
capture_ruler.py - Interactive capture at known distances for calibration

Usage:
  1. Run this script
  2. Place sensor at each distance when prompted
  3. Press Enter to capture
  4. Raw frames saved as ruler_0.5m.raw, ruler_1.0m.raw, ruler_1.5m.raw
"""

import subprocess
import os
import sys
import shutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(SCRIPT_DIR)

# Distances to capture (in meters)
DISTANCES = ["0.5m", "1.0m", "1.5m"]

# SDK paths
SDK_BASE = os.path.expanduser("~/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0")
SDK_LIB = f"{SDK_BASE}/lib/x86_64:{SDK_BASE}/thirdparty/libopencv/lib:{SDK_BASE}/thirdparty/libffmpeg/lib:{SDK_BASE}/thirdparty/liblive555/lib/Release"

def capture_single_frame(output_name):
    """Capture a single frame using benchmark_capture with hook"""

    # Create temp directory
    temp_dir = "ruler_temp"
    os.makedirs(temp_dir, exist_ok=True)

    # Clean temp dir
    for f in os.listdir(temp_dir):
        os.remove(os.path.join(temp_dir, f))

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = f"{SDK_LIB}:{env.get('LD_LIBRARY_PATH', '')}"
    env["LD_PRELOAD"] = f"{SCRIPT_DIR}/build/libv4l2_hook.so"
    env["V4L2_HOOK_OUTPUT"] = temp_dir

    # Capture 10 frames (first 4 are warmup, we'll use frame 5)
    print("  Capturing...")
    result = subprocess.run(
        [f"{SCRIPT_DIR}/build/benchmark_capture", "10", temp_dir],
        env=env,
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print(f"  ERROR: {result.stderr}")
        return False

    # Find the first valid raw frame (skip warmup 0-3)
    raw_file = os.path.join(temp_dir, "raw_0005.raw")
    sdk_file = os.path.join(temp_dir, "sdk_depth_0001.raw")

    if not os.path.exists(raw_file):
        print(f"  ERROR: No raw frame captured")
        return False

    # Copy to final location
    shutil.copy(raw_file, f"ruler_{output_name}.raw")
    if os.path.exists(sdk_file):
        shutil.copy(sdk_file, f"ruler_{output_name}_sdk.raw")

    print(f"  Saved: ruler_{output_name}.raw")
    return True

def analyze_frame(filename):
    """Quick analysis of captured frame"""
    import numpy as np

    if not os.path.exists(filename):
        return

    data = np.fromfile(filename, dtype=np.uint16).reshape(241, 1600)
    pixel_data = data[1:, :].reshape(240, 320, 5)

    # Center ROI
    roi = pixel_data[108:132, 148:172, :]

    print(f"  Sub[0]: {roi[:,:,0].mean():.1f}")
    print(f"  Sub[1]: {roi[:,:,1].mean():.1f}")
    print(f"  Sub[2]: {roi[:,:,2].mean():.1f}")
    print(f"  Sub[3]: {roi[:,:,3].mean():.1f}")
    print(f"  Sub[4]: {roi[:,:,4].mean():.1f}")
    print(f"  Sub[2]-Sub[3]: {(roi[:,:,2].mean() - roi[:,:,3].mean()):.1f}")

def main():
    print("=" * 50)
    print("  RULER CALIBRATION CAPTURE")
    print("=" * 50)
    print()
    print("This will capture frames at known distances.")
    print("Place the sensor pointing at a flat surface (wall/clipboard)")
    print("at each distance when prompted.")
    print()

    # Check if benchmark_capture exists
    if not os.path.exists(f"{SCRIPT_DIR}/build/benchmark_capture"):
        print("ERROR: benchmark_capture not found. Run 'make' first.")
        sys.exit(1)

    for dist in DISTANCES:
        print("-" * 50)
        print(f"Distance: {dist}")
        print(f"Position the sensor at exactly {dist} from the target.")
        input("Press ENTER when ready to capture...")

        if capture_single_frame(dist):
            print("Analysis:")
            analyze_frame(f"ruler_{dist}.raw")
        else:
            print("Capture failed!")
        print()

    print("=" * 50)
    print("  CAPTURE COMPLETE")
    print("=" * 50)
    print()
    print("Files saved:")
    for dist in DISTANCES:
        if os.path.exists(f"ruler_{dist}.raw"):
            print(f"  - ruler_{dist}.raw")

    # Final comparison
    print()
    print("Comparison of Sub[2] and Sub[3] across distances:")
    print("-" * 50)

    try:
        import numpy as np
        for dist in DISTANCES:
            filename = f"ruler_{dist}.raw"
            if os.path.exists(filename):
                data = np.fromfile(filename, dtype=np.uint16).reshape(241, 1600)
                pixel_data = data[1:, :].reshape(240, 320, 5)
                roi = pixel_data[108:132, 148:172, :]

                s2 = roi[:,:,2].mean()
                s3 = roi[:,:,3].mean()
                diff = s2 - s3
                ratio = s2 / s3 if s3 > 0 else 0

                print(f"{dist}: Sub[2]={s2:.0f}, Sub[3]={s3:.0f}, diff={diff:.0f}, ratio={ratio:.4f}")
    except ImportError:
        print("(numpy not available for analysis)")

if __name__ == "__main__":
    main()

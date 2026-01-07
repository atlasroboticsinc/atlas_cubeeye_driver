#!/bin/bash
# capture.sh - Capture raw V4L2 + SDK depth frames simultaneously
#
# Usage: ./capture.sh [num_frames] [output_dir]
#   num_frames: Number of frames to capture (default: 150 = 10 seconds)
#   output_dir: Output directory (default: benchmark)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NUM_FRAMES="${1:-150}"
OUTPUT_DIR="${2:-benchmark}"

# CubeEye SDK library paths
SDK_BASE="/home/cmericli/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0"
SDK_LIB="$SDK_BASE/lib"
SDK_LIB="$SDK_LIB:$SDK_BASE/thirdparty/libopencv/lib"
SDK_LIB="$SDK_LIB:$SDK_BASE/thirdparty/libffmpeg/lib"
SDK_LIB="$SDK_LIB:$SDK_BASE/thirdparty/liblive555/lib/Release"

echo "=== CubeEye Benchmark Capture ==="
echo "Frames: $NUM_FRAMES (~$((NUM_FRAMES / 15)) seconds)"
echo "Output: $OUTPUT_DIR/"
echo ""

# Create output directory
mkdir -p "$SCRIPT_DIR/$OUTPUT_DIR"

# Run capture with hook
LD_LIBRARY_PATH="$SDK_LIB:$LD_LIBRARY_PATH" \
LD_PRELOAD="$SCRIPT_DIR/build/libv4l2_hook.so" \
V4L2_HOOK_OUTPUT="$OUTPUT_DIR" \
"$SCRIPT_DIR/build/benchmark_capture" "$NUM_FRAMES" "$OUTPUT_DIR"

echo ""
echo "=== Capture Complete ==="
echo "Raw frames:   $(ls -1 "$SCRIPT_DIR/$OUTPUT_DIR"/raw_*.raw 2>/dev/null | wc -l)"
echo "SDK depth:    $(ls -1 "$SCRIPT_DIR/$OUTPUT_DIR"/sdk_depth_*.raw 2>/dev/null | wc -l)"
echo "SDK amplitude: $(ls -1 "$SCRIPT_DIR/$OUTPUT_DIR"/sdk_amp_*.raw 2>/dev/null | wc -l)"

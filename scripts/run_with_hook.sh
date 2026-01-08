#!/bin/bash
# Wrapper script to run benchmark_capture with V4L2/file hooks

SDK_BASE=/home/cmericli/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0
DRIVER_DIR=/home/cmericli/development/atlas/code/cubeeye_nano_driver

export LD_LIBRARY_PATH=$SDK_BASE/lib:$SDK_BASE/thirdparty/libopencv/lib:$SDK_BASE/thirdparty/libffmpeg/lib:$SDK_BASE/thirdparty/liblive555/lib/Release:$SDK_BASE/thirdparty/libusb/lib:$LD_LIBRARY_PATH

# Output directory for captured data
OUTPUT_DIR=${1:-$DRIVER_DIR/fppn_test}
FRAMES=${2:-5}

mkdir -p "$OUTPUT_DIR"

echo "Running benchmark_capture with hooks..."
echo "  Output: $OUTPUT_DIR"
echo "  Frames: $FRAMES"
echo ""

V4L2_HOOK_OUTPUT="$OUTPUT_DIR" LD_PRELOAD="$DRIVER_DIR/build/libv4l2_hook.so" "$DRIVER_DIR/build/benchmark_capture" "$FRAMES"

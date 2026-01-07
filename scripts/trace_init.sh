#!/bin/bash
# Trace complete SDK initialization sequence
# This captures ALL UVC XU commands and V4L2 ioctls from startup to first frame

set -e

DRIVER_DIR="/home/cmericli/development/atlas/code/cubeeye_nano_driver"
SDK_BASE="/home/cmericli/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0"
OUTPUT_DIR="$DRIVER_DIR/init_trace_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$OUTPUT_DIR"
echo "=== CubeEye SDK Initialization Trace ==="
echo "Output: $OUTPUT_DIR"
echo ""

# Set up library paths
export LD_LIBRARY_PATH="$SDK_BASE/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_BASE/thirdparty/libopencv/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_BASE/thirdparty/libffmpeg/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_BASE/thirdparty/liblive555/lib/Release"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_BASE/thirdparty/libusb/lib"

# Hook environment
export V4L2_HOOK_OUTPUT="$OUTPUT_DIR"
export V4L2_HOOK_VERBOSE=1

# Run with hook - capture only 3 frames to get init sequence
echo "Running SDK capture with V4L2 hook..."
echo "Capturing stderr to log file..."
echo ""

LD_PRELOAD="$DRIVER_DIR/build/libv4l2_hook.so" \
    "$DRIVER_DIR/build/benchmark_capture" 3 "$OUTPUT_DIR" 2>&1 | tee "$OUTPUT_DIR/trace.log"

echo ""
echo "=== Trace Analysis ==="
echo ""

# Extract UVC XU commands
echo "--- UVC XU Commands (UVCIOC_CTRL_QUERY) ---"
grep -E "UVC_XU|SET_CUR|GET_CUR" "$OUTPUT_DIR/trace.log" | head -50 || echo "No UVC XU commands found"

echo ""
echo "--- V4L2 Format/Stream Commands ---"
grep -E "S_FMT|STREAMON|S_CTRL" "$OUTPUT_DIR/trace.log" | head -20 || echo "No format commands found"

echo ""
echo "--- Summary ---"
echo "Total UVC XU commands: $(grep -c "UVC_XU" "$OUTPUT_DIR/trace.log" 2>/dev/null || echo 0)"
echo "SET_CUR commands: $(grep -c "SET_CUR" "$OUTPUT_DIR/trace.log" 2>/dev/null || echo 0)"
echo "GET_CUR commands: $(grep -c "GET_CUR" "$OUTPUT_DIR/trace.log" 2>/dev/null || echo 0)"
echo ""
echo "Full log: $OUTPUT_DIR/trace.log"

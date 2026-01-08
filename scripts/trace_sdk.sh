#!/bin/bash
# Trace SDK FPPN loading using LD_PRELOAD hook with detailed logging

DRIVER_DIR="/home/cmericli/development/atlas/code/cubeeye_nano_driver"
SDK_BASE="/home/cmericli/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0"
OUTPUT_DIR="$DRIVER_DIR/trace_output_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$OUTPUT_DIR"
echo "Output directory: $OUTPUT_DIR"

# Include ALL thirdparty library paths
export LD_LIBRARY_PATH="$SDK_BASE/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_BASE/thirdparty/libopencv/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_BASE/thirdparty/libffmpeg/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_BASE/thirdparty/liblive555/lib/Release"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SDK_BASE/thirdparty/libusb/lib"
export V4L2_HOOK_OUTPUT="$OUTPUT_DIR"
export V4L2_HOOK_VERBOSE=1

echo "Library path: $LD_LIBRARY_PATH"
echo ""

echo "Running SDK capture with hook..."
LD_PRELOAD="$DRIVER_DIR/build/libv4l2_hook.so" "$DRIVER_DIR/build/benchmark_capture" 1

echo ""
echo "=== Analysis of captured UVC commands ==="
echo ""

# Analyze SET_CUR commands (page requests)
if ls "$OUTPUT_DIR"/set_cur_*.bin 1>/dev/null 2>&1; then
    echo "SET_CUR commands captured (page requests):"
    for f in "$OUTPUT_DIR"/set_cur_*.bin; do
        # Each SET_CUR for selector 4 is: 00 20 HH LL (4 bytes, HH LL = big-endian page num)
        hex=$(xxd -p "$f" | head -c 8)
        if [ ${#hex} -ge 8 ]; then
            page_hi="${hex:4:2}"
            page_lo="${hex:6:2}"
            page_num=$((16#${page_hi}${page_lo}))
            echo "  $(basename $f): Page 0x${page_hi}${page_lo} ($page_num)"
        fi
    done | head -100
    
    # Count total and show range
    total=$(ls "$OUTPUT_DIR"/set_cur_*.bin 2>/dev/null | wc -l)
    echo ""
    echo "Total SET_CUR commands: $total"
    
    # Extract page numbers and find range
    echo ""
    echo "Page number summary:"
    for f in "$OUTPUT_DIR"/set_cur_*.bin; do
        hex=$(xxd -p "$f" | head -c 8)
        if [ ${#hex} -ge 8 ]; then
            page_hi="${hex:4:2}"
            page_lo="${hex:6:2}"
            echo "$((16#${page_hi}${page_lo}))"
        fi
    done | sort -n | uniq > "$OUTPUT_DIR/pages.txt"
    
    first=$(head -1 "$OUTPUT_DIR/pages.txt")
    last=$(tail -1 "$OUTPUT_DIR/pages.txt")
    count=$(wc -l < "$OUTPUT_DIR/pages.txt")
    echo "  First page: $first (0x$(printf '%04x' $first))"
    echo "  Last page: $last (0x$(printf '%04x' $last))"
    echo "  Unique pages: $count"
fi


#!/bin/bash
# Capture USB traffic during SDK operation to understand UVC XU commands

set -e

SDK_BASE="${HOME}/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0"
SDK_LIB="${SDK_BASE}/lib"
SDK_LIB="${SDK_LIB}:${SDK_BASE}/thirdparty/libopencv/lib"
SDK_LIB="${SDK_LIB}:${SDK_BASE}/thirdparty/libffmpeg/lib"
SDK_LIB="${SDK_LIB}:${SDK_BASE}/thirdparty/liblive555/lib/Release"

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
OUTPUT_DIR="${SCRIPT_DIR}/usb_capture"

mkdir -p "${OUTPUT_DIR}"

echo "=== USB Traffic Capture for CubeEye SDK ==="
echo ""
echo "This will capture USB traffic while running the SDK."
echo "Output will be saved to: ${OUTPUT_DIR}"
echo ""

# Find the USB bus for CubeEye
BUS=$(lsusb | grep "3674:0200" | cut -d' ' -f2)
if [ -z "$BUS" ]; then
    echo "ERROR: CubeEye camera not found"
    exit 1
fi
echo "Found CubeEye on USB bus ${BUS}"

# Start USB capture (requires tcpdump or tshark)
PCAP_FILE="${OUTPUT_DIR}/cubeeye_$(date +%Y%m%d_%H%M%S).pcap"

echo ""
echo "Starting USB capture to ${PCAP_FILE}"
echo "Press Ctrl+C when done with SDK test..."
echo ""

# Use tshark if available, otherwise use tcpdump
if command -v tshark &> /dev/null; then
    sudo tshark -i usbmon${BUS} -w "${PCAP_FILE}" -f "host 2.4" &
    CAPTURE_PID=$!
elif command -v tcpdump &> /dev/null; then
    sudo tcpdump -i usbmon${BUS} -w "${PCAP_FILE}" &
    CAPTURE_PID=$!
else
    echo "ERROR: Neither tshark nor tcpdump found. Install with:"
    echo "  sudo apt install tshark"
    exit 1
fi

sleep 2

# Run the SDK benchmark capture briefly
echo "Running SDK capture..."
export LD_LIBRARY_PATH="${SDK_LIB}:${LD_LIBRARY_PATH}"
export LD_PRELOAD="${SCRIPT_DIR}/build/libv4l2_hook.so"
export V4L2_HOOK_OUTPUT="${OUTPUT_DIR}"

# Run for 5 frames only
"${SCRIPT_DIR}/build/benchmark_capture" 5 "${OUTPUT_DIR}" 2>&1 | tee "${OUTPUT_DIR}/sdk_output.log" || true

echo ""
echo "Stopping USB capture..."
sudo kill ${CAPTURE_PID} 2>/dev/null || true
sleep 1

echo ""
echo "=== Capture complete ==="
echo "USB traffic saved to: ${PCAP_FILE}"
echo "SDK output saved to: ${OUTPUT_DIR}/sdk_output.log"
echo ""
echo "To analyze UVC commands:"
echo "  tshark -r ${PCAP_FILE} -Y 'usb.setup.bRequest==0x01' -T fields -e usb.urb_type -e usb.setup.bRequest -e usb.data_len"

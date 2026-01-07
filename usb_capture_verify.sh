#!/bin/bash
# USB vs V4L2 verification script

DEVICE=/dev/video0
USB_BUS=2
DURATION=3

echo "=== USB vs V4L2 Verification ==="
echo "This will capture raw USB traffic and compare to V4L2 frames"
echo ""

# Clean up any existing captures
rm -f usb_raw.pcap v4l2_frame.raw

# Start USB capture in background (using tshark with usbmon)
echo "[1] Starting USB capture on bus $USB_BUS..."
sudo -S timeout $DURATION tshark -i usbmon${USB_BUS} -w usb_raw.pcap 2>/dev/null &
USB_PID=$!
sleep 1

# Capture one V4L2 frame
echo "[2] Capturing V4L2 frame..."
v4l2-ctl -d $DEVICE --set-fmt-video=width=1600,height=241,pixelformat=YUYV
timeout 2 v4l2-ctl -d $DEVICE --stream-mmap --stream-count=1 --stream-to=v4l2_frame.raw 2>/dev/null

# Wait for USB capture to finish
wait $USB_PID 2>/dev/null

echo "[3] Capture complete"
echo ""

# Check file sizes
echo "=== File Sizes ==="
ls -la usb_raw.pcap v4l2_frame.raw 2>/dev/null

echo ""
echo "=== V4L2 Frame Analysis ==="
if [ -f v4l2_frame.raw ]; then
    SIZE=$(stat -c%s v4l2_frame.raw)
    echo "V4L2 frame size: $SIZE bytes"
    echo "Expected: 771200 bytes (1600 * 241 * 2)"
    echo ""
    echo "First 64 bytes (hex):"
    xxd -l 64 v4l2_frame.raw
fi

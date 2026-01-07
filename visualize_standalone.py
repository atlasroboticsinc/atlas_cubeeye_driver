#!/usr/bin/env python3
"""
SDK-Free Live Depth Visualization for CubeEye I200D

This is a COMPLETELY SDK-FREE implementation that:
1. Opens the V4L2 device directly
2. Sends UVC XU commands to enable streaming
3. Captures raw frames via V4L2
4. Extracts depth using our reverse-engineered algorithm

Usage:
    python visualize_standalone.py [--device /dev/video0]

Controls:
    Left-click: Show depth at cursor position
    'c': Toggle colormap
    'g': Toggle gradient correction
    's': Save current frame
    'q'/ESC: Quit

Prerequisites:
    pip install numpy opencv-python fcntl
"""

import numpy as np
import cv2
import sys
import time
import fcntl
import ctypes
import mmap
from pathlib import Path

# Add current directory for depth_extractor
sys.path.insert(0, str(Path(__file__).parent))
from depth_extractor_fast import FastDepthExtractor as DepthExtractor

# Frame format constants
FRAME_WIDTH = 1600
FRAME_HEIGHT = 241
FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT * 2  # YUYV = 2 bytes/pixel

OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480

# UVC Extension Unit constants
XU_UNIT_ID = 3
XU_SELECTOR_REG = 2

# ioctl codes (from Linux headers)
VIDIOC_QUERYCAP = 0x80685600
VIDIOC_S_FMT = 0xC0CC5605
VIDIOC_G_FMT = 0xC0CC5604
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0445609
VIDIOC_QBUF = 0xC044560F
VIDIOC_DQBUF = 0xC0445611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
UVCIOC_CTRL_QUERY = 0xC0107521

# V4L2 structures
class v4l2_capability(ctypes.Structure):
    _fields_ = [
        ('driver', ctypes.c_char * 16),
        ('card', ctypes.c_char * 32),
        ('bus_info', ctypes.c_char * 32),
        ('version', ctypes.c_uint32),
        ('capabilities', ctypes.c_uint32),
        ('device_caps', ctypes.c_uint32),
        ('reserved', ctypes.c_uint32 * 3),
    ]

class v4l2_pix_format(ctypes.Structure):
    _fields_ = [
        ('width', ctypes.c_uint32),
        ('height', ctypes.c_uint32),
        ('pixelformat', ctypes.c_uint32),
        ('field', ctypes.c_uint32),
        ('bytesperline', ctypes.c_uint32),
        ('sizeimage', ctypes.c_uint32),
        ('colorspace', ctypes.c_uint32),
        ('priv', ctypes.c_uint32),
        ('flags', ctypes.c_uint32),
        ('ycbcr_enc', ctypes.c_uint32),
        ('quantization', ctypes.c_uint32),
        ('xfer_func', ctypes.c_uint32),
    ]

class v4l2_format(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_uint32),
        ('fmt', v4l2_pix_format),
        ('padding', ctypes.c_uint8 * 152),  # Union padding
    ]

class v4l2_requestbuffers(ctypes.Structure):
    _fields_ = [
        ('count', ctypes.c_uint32),
        ('type', ctypes.c_uint32),
        ('memory', ctypes.c_uint32),
        ('capabilities', ctypes.c_uint32),
        ('flags', ctypes.c_uint8),
        ('reserved', ctypes.c_uint8 * 3),
    ]

class timeval(ctypes.Structure):
    _fields_ = [
        ('tv_sec', ctypes.c_long),
        ('tv_usec', ctypes.c_long),
    ]

class v4l2_timecode(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_uint32),
        ('flags', ctypes.c_uint32),
        ('frames', ctypes.c_uint8),
        ('seconds', ctypes.c_uint8),
        ('minutes', ctypes.c_uint8),
        ('hours', ctypes.c_uint8),
        ('userbits', ctypes.c_uint8 * 4),
    ]

class v4l2_buffer_union(ctypes.Union):
    _fields_ = [
        ('offset', ctypes.c_uint32),
        ('userptr', ctypes.c_ulong),
        ('planes', ctypes.c_void_p),
        ('fd', ctypes.c_int32),
    ]

class v4l2_buffer(ctypes.Structure):
    _fields_ = [
        ('index', ctypes.c_uint32),
        ('type', ctypes.c_uint32),
        ('bytesused', ctypes.c_uint32),
        ('flags', ctypes.c_uint32),
        ('field', ctypes.c_uint32),
        ('timestamp', timeval),
        ('timecode', v4l2_timecode),
        ('sequence', ctypes.c_uint32),
        ('memory', ctypes.c_uint32),
        ('m', v4l2_buffer_union),
        ('length', ctypes.c_uint32),
        ('reserved2', ctypes.c_uint32),
        ('request_fd_or_reserved', ctypes.c_int32),
    ]

class uvc_xu_control_query(ctypes.Structure):
    _fields_ = [
        ('unit', ctypes.c_uint8),
        ('selector', ctypes.c_uint8),
        ('query', ctypes.c_uint8),
        ('size', ctypes.c_uint16),
        ('data', ctypes.c_void_p),
    ]

# V4L2 constants
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_PIX_FMT_YUYV = 0x56595559  # 'YUYV'

# UVC constants
UVC_SET_CUR = 0x01
UVC_GET_LEN = 0x85


class SDKFreeVisualizer:
    def __init__(self, device="/dev/video0"):
        self.device = device
        self.fd = None
        self.buffers = []
        self.extractor = DepthExtractor(apply_gradient_correction=True)

        # State
        self.cursor_pos = (OUTPUT_WIDTH // 2, OUTPUT_HEIGHT // 2)
        self.colormap_idx = 0
        self.depth_range = (200, 5000)
        self.gradient_correction = True
        self.running = True

        # FPS tracking
        self.frame_times = []
        self.fps = 0.0

        self.colormaps = [
            ('JET', cv2.COLORMAP_JET),
            ('VIRIDIS', cv2.COLORMAP_VIRIDIS),
            ('TURBO', cv2.COLORMAP_TURBO),
        ]

    def open_device(self):
        """Open V4L2 device"""
        import os
        self.fd = os.open(self.device, os.O_RDWR)
        print(f"Opened {self.device} (fd={self.fd})")

        # Query capabilities
        cap = v4l2_capability()
        fcntl.ioctl(self.fd, VIDIOC_QUERYCAP, cap)
        print(f"Card: {cap.card.decode()}")
        print(f"Driver: {cap.driver.decode()}")

    def get_xu_length(self, selector):
        """Get UVC XU control length"""
        data = (ctypes.c_uint8 * 2)()
        xu = uvc_xu_control_query()
        xu.unit = XU_UNIT_ID
        xu.selector = selector
        xu.query = UVC_GET_LEN
        xu.size = 2
        xu.data = ctypes.addressof(data)

        try:
            fcntl.ioctl(self.fd, UVCIOC_CTRL_QUERY, xu)
            return data[0] | (data[1] << 8)
        except:
            return 0

    def send_xu_command(self, selector, data_bytes):
        """Send UVC XU command"""
        length = self.get_xu_length(selector)
        if length == 0:
            length = len(data_bytes)

        data = (ctypes.c_uint8 * length)()
        for i, b in enumerate(data_bytes):
            if i < length:
                data[i] = b

        xu = uvc_xu_control_query()
        xu.unit = XU_UNIT_ID
        xu.selector = selector
        xu.query = UVC_SET_CUR
        xu.size = length
        xu.data = ctypes.addressof(data)

        try:
            fcntl.ioctl(self.fd, UVCIOC_CTRL_QUERY, xu)
            return True
        except:
            return False

    def enable_sensor(self):
        """Enable sensor/illuminator (write 0xD0 to register 0x0001)"""
        print("Enabling sensor (reg 0x0001 = 0xD0)... ", end="")
        # Selector 2: 01 01 00 d0
        if self.send_xu_command(XU_SELECTOR_REG, [0x01, 0x01, 0x00, 0xd0]):
            print("OK")
            return True
        print("FAILED")
        return False

    def send_status_commands(self):
        """Send status toggle commands on Selector 5"""
        print("Sending status commands... ", end="")
        # Selector 5: 01 80 00 16 00, then 01 80 00 16 01
        self.send_xu_command(5, [0x01, 0x80, 0x00, 0x16, 0x00])
        self.send_xu_command(5, [0x01, 0x80, 0x00, 0x16, 0x01])
        print("OK")

    def enable_streaming(self, enable=True):
        """Send UVC XU command to enable/disable sensor streaming"""
        print(f"{'Enabling' if enable else 'Disabling'} streaming... ", end="")
        # Selector 2: 01 02 94 00 01 (enable) / 00 (disable)
        if self.send_xu_command(XU_SELECTOR_REG, [0x01, 0x02, 0x94, 0x00, 0x01 if enable else 0x00]):
            print("OK")
            return True
        print("FAILED")
        return False

    def setup_capture(self):
        """Setup capture using OpenCV (simpler than raw V4L2)"""
        # Close raw fd - OpenCV needs exclusive access
        import os
        if self.fd:
            os.close(self.fd)
            self.fd = None

        # Use OpenCV VideoCapture for frame grabbing
        # The XU commands have already initialized the sensor
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open {self.device}")

        # Set format
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # Get raw YUYV

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"OpenCV capture: {actual_w}x{actual_h}")

    def capture_frame(self):
        """Capture a single frame using OpenCV"""
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame.tobytes()

    def depth_to_color(self, depth_frame):
        """Convert depth to colorized image"""
        min_d, max_d = self.depth_range
        normalized = np.clip(depth_frame.astype(np.float32), min_d, max_d)
        normalized = ((normalized - min_d) / (max_d - min_d) * 255).astype(np.uint8)

        _, colormap = self.colormaps[self.colormap_idx]
        colored = cv2.applyColorMap(normalized, colormap)
        colored[depth_frame == 0] = [0, 0, 0]

        return colored

    def draw_overlay(self, image, depth_frame):
        """Draw information overlay"""
        h, w = image.shape[:2]
        x, y = self.cursor_pos
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))

        depth_val = depth_frame[y, x]

        # Crosshair
        cv2.line(image, (x - 20, y), (x + 20, y), (255, 255, 255), 1)
        cv2.line(image, (x, y - 20), (x, y + 20), (255, 255, 255), 1)

        # Info box
        cv2.rectangle(image, (5, 5), (240, 120), (0, 0, 0), -1)
        cv2.rectangle(image, (5, 5), (240, 120), (255, 255, 255), 1)

        # Title
        cv2.putText(image, "SDK-FREE CAPTURE", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        # FPS
        cv2.putText(image, f"FPS: {self.fps:.1f}", (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Position and depth
        cv2.putText(image, f"Pos: ({x}, {y})", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        depth_str = f"Depth: {depth_val} mm" if depth_val > 0 else "Depth: INVALID"
        color = (0, 255, 0) if depth_val > 0 else (0, 0, 255)
        cv2.putText(image, depth_str, (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if depth_val > 0:
            cv2.putText(image, f"       ({depth_val/1000:.2f} m)", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Controls hint
        cv2.putText(image, "C:colormap G:gradient S:save Q:quit", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        return image

    def mouse_callback(self, event, x, y, flags, param):
        if event in [cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN]:
            self.cursor_pos = (x, y)

    def update_fps(self):
        now = time.time()
        self.frame_times.append(now)
        while len(self.frame_times) > 30:
            self.frame_times.pop(0)
        if len(self.frame_times) > 1:
            dt = self.frame_times[-1] - self.frame_times[0]
            if dt > 0:
                self.fps = (len(self.frame_times) - 1) / dt

    def run(self):
        """Main capture loop"""
        import os

        print("\n=== SDK-Free CubeEye Live Capture ===\n")

        frame_count = 0
        valid_frames = 0

        try:
            self.open_device()

            # Full sensor initialization sequence
            print("\n=== Sensor Initialization ===")
            self.enable_sensor()
            self.send_status_commands()
            self.enable_streaming(True)
            time.sleep(0.2)

            self.setup_capture()

            cv2.namedWindow("SDK-Free Depth", cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback("SDK-Free Depth", self.mouse_callback)

            print("\nControls:")
            print("  C: Cycle colormap")
            print("  G: Toggle gradient correction")
            print("  S: Save frame")
            print("  Q: Quit\n")

            # Skip first few frames (warmup)
            for _ in range(5):
                self.capture_frame()

            frame_count = 0
            valid_frames = 0

            while self.running:
                raw_data = self.capture_frame()
                if raw_data is None or len(raw_data) != FRAME_SIZE:
                    continue

                frame_count += 1

                # Check if frame has data
                frame_array = np.frombuffer(raw_data[:1000], dtype=np.uint16)
                if np.all(frame_array == 0):
                    continue

                valid_frames += 1

                # Extract depth
                self.extractor.apply_gradient = self.gradient_correction
                try:
                    depth = self.extractor.extract_depth(raw_data, interpolate=True)
                except Exception as e:
                    print(f"Extraction error: {e}")
                    continue

                # Visualize
                display = self.depth_to_color(depth)
                display = self.draw_overlay(display, depth)

                self.update_fps()
                cv2.imshow("SDK-Free Depth", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    self.running = False
                elif key == ord('c'):
                    self.colormap_idx = (self.colormap_idx + 1) % len(self.colormaps)
                elif key == ord('g'):
                    self.gradient_correction = not self.gradient_correction
                    print(f"Gradient correction: {'ON' if self.gradient_correction else 'OFF'}")
                elif key == ord('s'):
                    filename = f"standalone_frame_{valid_frames:05d}.png"
                    cv2.imwrite(filename, display)
                    print(f"Saved: {filename}")

        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()

        finally:
            cv2.destroyAllWindows()

            # Release OpenCV capture and send shutdown commands
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
                # Reopen device briefly to send shutdown commands
                import os
                try:
                    shutdown_fd = os.open(self.device, os.O_RDWR)
                    self.fd = shutdown_fd
                    self.enable_streaming(False)
                    # Send illuminator off command
                    self.send_xu_command(5, [0x01, 0x80, 0x00, 0x16, 0x00])
                    print("Illuminator disabled")
                    os.close(shutdown_fd)
                    self.fd = None
                except Exception as e:
                    print(f"Warning: Could not send shutdown commands: {e}")

            elif self.fd:
                # Device still open with raw fd
                self.enable_streaming(False)
                import os
                os.close(self.fd)

            print(f"\nTotal frames: {frame_count}, Valid: {valid_frames}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="SDK-Free CubeEye Live Visualization")
    parser.add_argument("--device", "-d", default="/dev/video0", help="V4L2 device")
    args = parser.parse_args()

    viz = SDKFreeVisualizer(args.device)
    viz.run()


if __name__ == "__main__":
    main()

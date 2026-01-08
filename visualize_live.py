#!/usr/bin/env python3
"""
Live Depth Visualization for CubeEye I200D

Captures raw frames from the sensor via V4L2 and displays depth in real-time.

Usage:
    python visualize_live.py                    # Auto-detect device
    python visualize_live.py --device /dev/video0

Controls:
    Left-click: Show depth at cursor position
    'c': Toggle colormap
    's': Save current frame
    'f': Toggle FPS display
    'g': Toggle gradient correction
    'q'/ESC: Quit
"""

import numpy as np
import cv2
import sys
import time
from pathlib import Path

# Add current directory for depth_extractor
sys.path.insert(0, str(Path(__file__).parent))
from depth_extractor import DepthExtractor

# CubeEye frame constants
FRAME_WIDTH = 1600
FRAME_HEIGHT = 241
FRAME_SIZE = 771200  # 1600 * 241 * 2 (YUYV)
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
MAX_DEPTH_MM = 7500

# Colormaps
COLORMAPS = [
    ('JET', cv2.COLORMAP_JET),
    ('VIRIDIS', cv2.COLORMAP_VIRIDIS),
    ('PLASMA', cv2.COLORMAP_PLASMA),
    ('TURBO', cv2.COLORMAP_TURBO),
]


class LiveDepthVisualizer:
    def __init__(self, device=0):
        self.device = device
        self.cap = None
        self.extractor = DepthExtractor(apply_gradient_correction=True)

        # State
        self.cursor_pos = (OUTPUT_WIDTH // 2, OUTPUT_HEIGHT // 2)
        self.colormap_idx = 0
        self.depth_range = (200, 5000)
        self.show_fps = True
        self.gradient_correction = True
        self.running = True

        # FPS tracking
        self.frame_times = []
        self.fps = 0.0

        # Window
        self.window_name = "CubeEye Live Depth"

    def depth_to_color(self, depth_frame):
        """Convert depth to colorized image"""
        min_d, max_d = self.depth_range
        normalized = np.clip(depth_frame.astype(np.float32), min_d, max_d)
        normalized = ((normalized - min_d) / (max_d - min_d) * 255).astype(np.uint8)

        colormap_name, colormap = COLORMAPS[self.colormap_idx]
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
        cv2.circle(image, (x, y), 5, (255, 255, 255), 1)

        # Info box
        cv2.rectangle(image, (5, 5), (220, 120), (0, 0, 0), -1)
        cv2.rectangle(image, (5, 5), (220, 120), (255, 255, 255), 1)

        # FPS
        if self.show_fps:
            cv2.putText(image, f"FPS: {self.fps:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        # Position and depth
        cv2.putText(image, f"Pos: ({x}, {y})", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        depth_str = f"Depth: {depth_val} mm" if depth_val > 0 else "Depth: INVALID"
        color = (0, 255, 0) if depth_val > 0 else (0, 0, 255)
        cv2.putText(image, depth_str, (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if depth_val > 0:
            cv2.putText(image, f"       ({depth_val/1000:.2f} m)", (10, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Gradient correction status
        gc_text = "Gradient: ON" if self.gradient_correction else "Gradient: OFF"
        gc_color = (0, 255, 0) if self.gradient_correction else (0, 0, 255)
        cv2.putText(image, gc_text, (10, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, gc_color, 1)

        # Colorbar
        self.draw_colorbar(image)

        # Controls hint
        controls = "C:colormap G:gradient S:save Q:quit"
        cv2.putText(image, controls, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        return image

    def draw_colorbar(self, image):
        """Draw colorbar on right side"""
        h, w = image.shape[:2]
        bar_w, bar_h = 25, h - 80
        bar_x, bar_y = w - bar_w - 10, 40

        gradient = np.linspace(0, 255, bar_h).astype(np.uint8)[::-1].reshape(-1, 1)
        gradient = np.repeat(gradient, bar_w, axis=1)

        _, colormap = COLORMAPS[self.colormap_idx]
        colorbar = cv2.applyColorMap(gradient, colormap)

        image[bar_y:bar_y + bar_h, bar_x:bar_x + bar_w] = colorbar
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (255, 255, 255), 1)

        min_d, max_d = self.depth_range
        cv2.putText(image, f"{max_d/1000:.1f}m", (bar_x - 10, bar_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        cv2.putText(image, f"{min_d/1000:.1f}m", (bar_x - 10, bar_y + bar_h + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE or event == cv2.EVENT_LBUTTONDOWN:
            self.cursor_pos = (x, y)

    def update_fps(self):
        """Update FPS calculation"""
        now = time.time()
        self.frame_times.append(now)

        # Keep last 30 frames
        while len(self.frame_times) > 30:
            self.frame_times.pop(0)

        if len(self.frame_times) > 1:
            dt = self.frame_times[-1] - self.frame_times[0]
            if dt > 0:
                self.fps = (len(self.frame_times) - 1) / dt

    def run(self):
        """Main visualization loop"""
        print(f"\nOpening device: {self.device}")

        # Open with OpenCV
        if isinstance(self.device, str):
            self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print(f"Error: Cannot open {self.device}")
            return

        # Set format: 1600x241 YUYV
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # Get raw YUYV

        # Verify settings
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Capture format: {actual_w}x{actual_h}")

        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        print("\n=== Live Depth Viewer ===")
        print("Controls:")
        print("  Left-click: Show depth")
        print("  C: Cycle colormap")
        print("  G: Toggle gradient correction")
        print("  S: Save frame")
        print("  F: Toggle FPS")
        print("  +/-: Adjust depth range")
        print("  Q/ESC: Quit")
        print("=========================\n")

        frame_count = 0
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to read frame")
                time.sleep(0.1)
                continue

            # OpenCV returns BGR by default, but we asked for raw YUYV
            # The frame should be the raw sensor data
            raw_data = frame.tobytes()

            # Check size - should be 771200 bytes (1600 * 241 * 2)
            if len(raw_data) != FRAME_SIZE:
                # If not, try reshaping
                if frame.size * frame.itemsize == FRAME_SIZE:
                    raw_data = frame.tobytes()
                else:
                    print(f"Unexpected frame size: {len(raw_data)} (expected {FRAME_SIZE})")
                    continue

            # Update extractor settings
            self.extractor.apply_gradient_correction = self.gradient_correction

            # Extract depth
            try:
                depth = self.extractor.extract_depth(raw_data, interpolate=True)
            except Exception as e:
                print(f"Extraction error: {e}")
                continue

            # Convert to color
            display = self.depth_to_color(depth)

            # Draw overlay
            display = self.draw_overlay(display, depth)

            # Update FPS
            self.update_fps()
            frame_count += 1

            cv2.imshow(self.window_name, display)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                self.running = False
            elif key == ord('c'):
                self.colormap_idx = (self.colormap_idx + 1) % len(COLORMAPS)
                print(f"Colormap: {COLORMAPS[self.colormap_idx][0]}")
            elif key == ord('g'):
                self.gradient_correction = not self.gradient_correction
                self.extractor.apply_gradient_correction = self.gradient_correction
                print(f"Gradient correction: {'ON' if self.gradient_correction else 'OFF'}")
            elif key == ord('f'):
                self.show_fps = not self.show_fps
            elif key == ord('s'):
                filename = f"live_depth_{frame_count:05d}.png"
                cv2.imwrite(filename, display)
                # Also save raw
                raw_filename = f"live_raw_{frame_count:05d}.bin"
                with open(raw_filename, 'wb') as f:
                    f.write(raw_data)
                print(f"Saved: {filename}, {raw_filename}")
            elif key == ord('+') or key == ord('='):
                min_d, max_d = self.depth_range
                self.depth_range = (min_d, min(max_d + 500, MAX_DEPTH_MM))
            elif key == ord('-'):
                min_d, max_d = self.depth_range
                self.depth_range = (min_d, max(max_d - 500, min_d + 500))

        cv2.destroyAllWindows()
        self.cap.release()
        print(f"\nTotal frames: {frame_count}")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Live CubeEye Depth Visualization")
    parser.add_argument("--device", "-d", default="0",
                        help="V4L2 device index or path (default: 0)")
    parser.add_argument("--colormap", choices=["jet", "viridis", "plasma", "turbo"],
                        default="jet", help="Initial colormap")
    parser.add_argument("--range", type=str, default="200,5000",
                        help="Depth display range (min,max) in mm")

    args = parser.parse_args()

    # Parse device
    device = args.device
    if device.isdigit():
        device = int(device)

    viz = LiveDepthVisualizer(device)

    # Set colormap
    colormap_names = [c[0].lower() for c in COLORMAPS]
    if args.colormap in colormap_names:
        viz.colormap_idx = colormap_names.index(args.colormap)

    # Set depth range
    try:
        min_d, max_d = map(int, args.range.split(","))
        viz.depth_range = (min_d, max_d)
    except:
        pass

    viz.run()


if __name__ == "__main__":
    main()

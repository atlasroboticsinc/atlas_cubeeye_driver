#!/usr/bin/env python3
"""
Interactive Depth Visualization Tool for CubeEye I200D

Click on any point to see the depth value in millimeters.
Supports live capture, single frame, or comparison mode.

Usage:
    python visualize_depth.py raw_frame.bin                  # Single raw frame
    python visualize_depth.py raw_frame.bin sdk_depth.raw    # Compare with SDK
    python visualize_depth.py --live                         # Live capture (requires sensor)

Controls:
    Left-click: Show depth at cursor position
    Right-click: Toggle between decoded/SDK view (comparison mode)
    'c': Toggle colormap (jet/viridis/plasma/inferno)
    's': Save current frame as PNG
    'r': Reset view
    'q'/ESC: Quit
"""

import numpy as np
import cv2
import sys
import os
from pathlib import Path

# Add current directory to path for importing depth_extractor
sys.path.insert(0, str(Path(__file__).parent))
from depth_extractor import DepthExtractor

# Constants
RAW_FRAME_SIZE = 771200
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
MAX_DEPTH_MM = 7500

# Colormaps
COLORMAPS = [
    ('JET', cv2.COLORMAP_JET),
    ('VIRIDIS', cv2.COLORMAP_VIRIDIS),
    ('PLASMA', cv2.COLORMAP_PLASMA),
    ('INFERNO', cv2.COLORMAP_INFERNO),
    ('TURBO', cv2.COLORMAP_TURBO),
]


class DepthVisualizer:
    def __init__(self, raw_path=None, sdk_path=None):
        self.extractor = DepthExtractor(apply_gradient_correction=True)

        # State
        self.raw_frame = None
        self.decoded_depth = None
        self.sdk_depth = None
        self.amplitude = None
        self.showing_sdk = False
        self.colormap_idx = 0
        self.cursor_pos = (OUTPUT_WIDTH // 2, OUTPUT_HEIGHT // 2)
        self.depth_range = (200, 5000)  # Display range in mm

        # Window name
        self.window_name = "CubeEye Depth Viewer"

        # Load data if provided
        if raw_path:
            self.load_raw_frame(raw_path)
        if sdk_path:
            self.load_sdk_depth(sdk_path)

    def load_raw_frame(self, path):
        """Load and decode a raw V4L2 frame"""
        print(f"Loading raw frame: {path}")
        with open(path, 'rb') as f:
            self.raw_frame = f.read()

        if len(self.raw_frame) != RAW_FRAME_SIZE:
            print(f"Warning: Expected {RAW_FRAME_SIZE} bytes, got {len(self.raw_frame)}")
            return False

        # Decode depth
        self.decoded_depth = self.extractor.extract_depth(self.raw_frame, interpolate=True)

        # Also extract amplitude for display
        self.amplitude = self.extractor.extract_amplitude(self.raw_frame, interpolate=True)

        print(f"Decoded depth range: {self.decoded_depth.min()}-{self.decoded_depth.max()} mm")
        return True

    def load_sdk_depth(self, path):
        """Load SDK depth output for comparison"""
        print(f"Loading SDK depth: {path}")
        with open(path, 'rb') as f:
            data = f.read()

        expected_size = OUTPUT_WIDTH * OUTPUT_HEIGHT * 2
        if len(data) != expected_size:
            print(f"Warning: Expected {expected_size} bytes, got {len(data)}")
            return False

        self.sdk_depth = np.frombuffer(data, dtype=np.uint16).reshape(OUTPUT_HEIGHT, OUTPUT_WIDTH)
        print(f"SDK depth range: {self.sdk_depth.min()}-{self.sdk_depth.max()} mm")
        return True

    def depth_to_color(self, depth_frame):
        """Convert depth to colorized image"""
        # Normalize to display range
        min_d, max_d = self.depth_range
        normalized = np.clip(depth_frame.astype(np.float32), min_d, max_d)
        normalized = ((normalized - min_d) / (max_d - min_d) * 255).astype(np.uint8)

        # Apply colormap
        colormap_name, colormap = COLORMAPS[self.colormap_idx]
        colored = cv2.applyColorMap(normalized, colormap)

        # Mark invalid pixels (depth = 0) as black
        colored[depth_frame == 0] = [0, 0, 0]

        return colored

    def draw_info_overlay(self, image, depth_frame):
        """Draw information overlay on image"""
        h, w = image.shape[:2]
        x, y = self.cursor_pos

        # Ensure cursor is within bounds
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))

        # Get depth at cursor
        depth_val = depth_frame[y, x]

        # Draw crosshair at cursor
        cv2.line(image, (x - 20, y), (x + 20, y), (255, 255, 255), 1)
        cv2.line(image, (x, y - 20), (x, y + 20), (255, 255, 255), 1)
        cv2.circle(image, (x, y), 5, (255, 255, 255), 1)

        # Info box background
        info_box_h = 160
        cv2.rectangle(image, (5, 5), (250, info_box_h), (0, 0, 0), -1)
        cv2.rectangle(image, (5, 5), (250, info_box_h), (255, 255, 255), 1)

        # Source indicator
        if self.sdk_depth is not None:
            source = "SDK Output" if self.showing_sdk else "Decoded (Custom)"
        else:
            source = "Decoded (Custom)"
        cv2.putText(image, source, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        # Cursor position and depth
        cv2.putText(image, f"Position: ({x}, {y})", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        depth_str = f"Depth: {depth_val} mm" if depth_val > 0 else "Depth: INVALID"
        color = (0, 255, 0) if depth_val > 0 else (0, 0, 255)
        cv2.putText(image, depth_str, (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Depth in meters
        if depth_val > 0:
            cv2.putText(image, f"       ({depth_val/1000:.3f} m)", (10, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Comparison info
        if self.sdk_depth is not None and self.decoded_depth is not None:
            sdk_val = self.sdk_depth[y, x]
            dec_val = self.decoded_depth[y, x]
            diff = int(dec_val) - int(sdk_val)
            cv2.putText(image, f"SDK: {sdk_val} mm | Dec: {dec_val} mm", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            diff_color = (0, 255, 0) if abs(diff) < 10 else (0, 165, 255) if abs(diff) < 50 else (0, 0, 255)
            cv2.putText(image, f"Difference: {diff:+d} mm", (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, diff_color, 1)

        # Colormap indicator
        colormap_name, _ = COLORMAPS[self.colormap_idx]
        cv2.putText(image, f"Colormap: {colormap_name}", (10, info_box_h - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # Controls hint at bottom
        controls = "LClick: depth | RClick: toggle SDK | C: colormap | S: save | Q: quit"
        cv2.putText(image, controls, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # Draw colorbar
        self.draw_colorbar(image)

        return image

    def draw_colorbar(self, image):
        """Draw a colorbar on the right side"""
        h, w = image.shape[:2]
        bar_w = 30
        bar_h = h - 100
        bar_x = w - bar_w - 10
        bar_y = 50

        # Create colorbar gradient
        gradient = np.linspace(0, 255, bar_h).astype(np.uint8)
        gradient = gradient[::-1].reshape(-1, 1)  # Flip so high values are at top
        gradient = np.repeat(gradient, bar_w, axis=1)

        _, colormap = COLORMAPS[self.colormap_idx]
        colorbar = cv2.applyColorMap(gradient, colormap)

        # Place colorbar on image
        image[bar_y:bar_y + bar_h, bar_x:bar_x + bar_w] = colorbar
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (255, 255, 255), 1)

        # Labels
        min_d, max_d = self.depth_range
        cv2.putText(image, f"{max_d}mm", (bar_x - 5, bar_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(image, f"{min_d}mm", (bar_x - 5, bar_y + bar_h + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(image, f"{(max_d + min_d)//2}mm", (bar_x - 5, bar_y + bar_h//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events"""
        if event == cv2.EVENT_MOUSEMOVE or event == cv2.EVENT_LBUTTONDOWN:
            self.cursor_pos = (x, y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.sdk_depth is not None:
                self.showing_sdk = not self.showing_sdk

    def run(self):
        """Main visualization loop"""
        if self.decoded_depth is None:
            print("Error: No depth data loaded")
            return

        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        print("\n=== CubeEye Depth Viewer ===")
        print("Controls:")
        print("  Left-click: Show depth at cursor")
        print("  Right-click: Toggle decoded/SDK view")
        print("  C: Cycle colormap")
        print("  S: Save frame as PNG")
        print("  +/-: Adjust depth range")
        print("  R: Reset view")
        print("  Q/ESC: Quit")
        print("============================\n")

        while True:
            # Select depth source
            if self.showing_sdk and self.sdk_depth is not None:
                depth_frame = self.sdk_depth
            else:
                depth_frame = self.decoded_depth

            # Convert to color
            display = self.depth_to_color(depth_frame)

            # Add overlay
            display = self.draw_info_overlay(display, depth_frame)

            cv2.imshow(self.window_name, display)

            key = cv2.waitKey(30) & 0xFF

            if key == ord('q') or key == 27:  # q or ESC
                break
            elif key == ord('c'):
                self.colormap_idx = (self.colormap_idx + 1) % len(COLORMAPS)
                print(f"Colormap: {COLORMAPS[self.colormap_idx][0]}")
            elif key == ord('s'):
                filename = f"depth_capture_{self.cursor_pos[0]}_{self.cursor_pos[1]}.png"
                cv2.imwrite(filename, display)
                print(f"Saved: {filename}")
            elif key == ord('r'):
                self.depth_range = (200, 5000)
                self.cursor_pos = (OUTPUT_WIDTH // 2, OUTPUT_HEIGHT // 2)
            elif key == ord('+') or key == ord('='):
                min_d, max_d = self.depth_range
                self.depth_range = (min_d, min(max_d + 500, MAX_DEPTH_MM))
            elif key == ord('-'):
                min_d, max_d = self.depth_range
                self.depth_range = (min_d, max(max_d - 500, min_d + 500))

        cv2.destroyAllWindows()


def find_latest_files(data_dir="data"):
    """Find the most recent raw and SDK depth files"""
    raw_files = sorted(Path(data_dir).glob("**/raw_*.bin"), key=os.path.getmtime, reverse=True)
    sdk_files = sorted(Path(data_dir).glob("**/sync_depth_*.raw"), key=os.path.getmtime, reverse=True)

    raw_path = str(raw_files[0]) if raw_files else None
    sdk_path = str(sdk_files[0]) if sdk_files else None

    return raw_path, sdk_path


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Interactive CubeEye Depth Visualization")
    parser.add_argument("raw_file", nargs="?", help="Path to raw V4L2 frame (.bin)")
    parser.add_argument("sdk_file", nargs="?", help="Path to SDK depth output (.raw)")
    parser.add_argument("--latest", action="store_true", help="Use latest files in data/")
    parser.add_argument("--colormap", choices=["jet", "viridis", "plasma", "inferno", "turbo"],
                        default="jet", help="Initial colormap")
    parser.add_argument("--range", type=str, default="200,5000",
                        help="Depth display range in mm (min,max)")

    args = parser.parse_args()

    # Parse depth range
    try:
        min_d, max_d = map(int, args.range.split(","))
    except:
        min_d, max_d = 200, 5000

    # Find files
    raw_path = args.raw_file
    sdk_path = args.sdk_file

    if args.latest or (raw_path is None):
        auto_raw, auto_sdk = find_latest_files()
        if raw_path is None:
            raw_path = auto_raw
        if sdk_path is None:
            sdk_path = auto_sdk

    if raw_path is None:
        print("Error: No raw frame file specified or found in data/")
        print("Usage: python visualize_depth.py <raw_frame.bin> [sdk_depth.raw]")
        print("       python visualize_depth.py --latest")
        sys.exit(1)

    # Create visualizer
    viz = DepthVisualizer(raw_path, sdk_path)

    # Set initial colormap
    colormap_names = [c[0].lower() for c in COLORMAPS]
    if args.colormap in colormap_names:
        viz.colormap_idx = colormap_names.index(args.colormap)

    # Set depth range
    viz.depth_range = (min_d, max_d)

    # Run
    viz.run()


if __name__ == "__main__":
    main()

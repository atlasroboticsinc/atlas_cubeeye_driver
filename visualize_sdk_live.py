#!/usr/bin/env python3
"""
Live Depth Visualization using CubeEye SDK + Custom Extraction

This script runs the SDK capture in the background and visualizes
both SDK depth and our custom-decoded depth side by side.

Usage:
    python visualize_sdk_live.py

Requirements:
    - CubeEye SDK must be installed
    - build/sdk_capture must be built

Controls:
    Left-click: Show depth at cursor
    'c': Toggle colormap
    'm': Toggle mode (SDK/Custom/Side-by-side)
    's': Save frame
    'q'/ESC: Quit
"""

import numpy as np
import cv2
import sys
import os
import time
import subprocess
import threading
import queue
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from depth_extractor import DepthExtractor

# Constants
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
MAX_DEPTH_MM = 7500

COLORMAPS = [
    ('JET', cv2.COLORMAP_JET),
    ('VIRIDIS', cv2.COLORMAP_VIRIDIS),
    ('TURBO', cv2.COLORMAP_TURBO),
]


class SDKLiveVisualizer:
    def __init__(self):
        self.extractor = DepthExtractor(apply_gradient_correction=True)

        # State
        self.cursor_pos = (OUTPUT_WIDTH // 2, OUTPUT_HEIGHT // 2)
        self.colormap_idx = 0
        self.depth_range = (200, 5000)
        self.running = True
        self.mode = 'custom'  # 'sdk', 'custom', 'sidebyside'

        # Data
        self.sdk_depth = None
        self.custom_depth = None
        self.raw_frame = None

        # FPS
        self.frame_times = []
        self.fps = 0.0

        # Temp directory for frame exchange
        self.temp_dir = Path("/tmp/cubeeye_live")
        self.temp_dir.mkdir(exist_ok=True)

        # SDK process
        self.sdk_process = None

    def depth_to_color(self, depth_frame):
        """Convert depth to colorized image"""
        min_d, max_d = self.depth_range
        normalized = np.clip(depth_frame.astype(np.float32), min_d, max_d)
        normalized = ((normalized - min_d) / (max_d - min_d) * 255).astype(np.uint8)

        _, colormap = COLORMAPS[self.colormap_idx]
        colored = cv2.applyColorMap(normalized, colormap)
        colored[depth_frame == 0] = [0, 0, 0]

        return colored

    def draw_overlay(self, image, depth_frame, label=""):
        """Draw info overlay"""
        h, w = image.shape[:2]
        x, y = self.cursor_pos

        # Adjust for side-by-side mode
        if self.mode == 'sidebyside' and x >= OUTPUT_WIDTH:
            x = x - OUTPUT_WIDTH

        x = max(0, min(x, OUTPUT_WIDTH - 1))
        y = max(0, min(y, OUTPUT_HEIGHT - 1))

        if depth_frame is not None:
            depth_val = depth_frame[y, x]
        else:
            depth_val = 0

        # Crosshair (on the correct side)
        draw_x = x if self.mode != 'sidebyside' or self.cursor_pos[0] < OUTPUT_WIDTH else x + OUTPUT_WIDTH
        cv2.line(image, (draw_x - 15, y), (draw_x + 15, y), (255, 255, 255), 1)
        cv2.line(image, (draw_x, y - 15), (draw_x, y + 15), (255, 255, 255), 1)

        # Info box
        cv2.rectangle(image, (5, 5), (200, 100), (0, 0, 0), -1)
        cv2.rectangle(image, (5, 5), (200, 100), (255, 255, 255), 1)

        # Mode and FPS
        cv2.putText(image, f"{label} | FPS: {self.fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Depth
        cv2.putText(image, f"({x}, {y})", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        depth_str = f"{depth_val} mm ({depth_val/1000:.2f} m)" if depth_val > 0 else "INVALID"
        color = (0, 255, 0) if depth_val > 0 else (0, 0, 255)
        cv2.putText(image, depth_str, (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Controls
        cv2.putText(image, "M:mode C:color S:save Q:quit", (10, h - 10),
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

    def start_sdk_capture(self):
        """Start SDK capture process"""
        # Clean temp directory
        for f in self.temp_dir.glob("*.raw"):
            f.unlink()
        for f in self.temp_dir.glob("*.bin"):
            f.unlink()

        # Find sdk_capture binary
        sdk_capture = Path(__file__).parent / "build" / "sdk_capture"
        if not sdk_capture.exists():
            print(f"Error: {sdk_capture} not found. Build it first.")
            return False

        # Run with hook to capture raw frames
        hook_lib = Path(__file__).parent / "build" / "libsimple_hook.so"
        if not hook_lib.exists():
            print(f"Error: {hook_lib} not found. Build it first.")
            return False

        env = os.environ.copy()
        env['LD_PRELOAD'] = str(hook_lib)
        env['V4L2_HOOK_OUTPUT_DIR'] = str(self.temp_dir)
        env['V4L2_HOOK_MAX_FRAMES'] = "999999"  # Unlimited

        # Start SDK capture (captures 999999 frames = effectively unlimited)
        self.sdk_process = subprocess.Popen(
            [str(sdk_capture), "999999"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(self.temp_dir)
        )

        print(f"Started SDK capture (PID: {self.sdk_process.pid})")
        return True

    def stop_sdk_capture(self):
        """Stop SDK capture"""
        if self.sdk_process:
            self.sdk_process.terminate()
            try:
                self.sdk_process.wait(timeout=2)
            except:
                self.sdk_process.kill()
            print("SDK capture stopped")

    def get_latest_frames(self):
        """Get latest raw and SDK depth frames"""
        # Find latest raw frame
        raw_files = sorted(self.temp_dir.glob("raw_*.bin"),
                          key=lambda x: x.stat().st_mtime, reverse=True)
        sdk_files = sorted(self.temp_dir.glob("sync_depth_*.raw"),
                          key=lambda x: x.stat().st_mtime, reverse=True)

        raw_frame = None
        sdk_depth = None

        if raw_files:
            try:
                with open(raw_files[0], 'rb') as f:
                    data = f.read()
                if len(data) == 771200:
                    raw_frame = data
            except:
                pass

        if sdk_files:
            try:
                with open(sdk_files[0], 'rb') as f:
                    data = f.read()
                if len(data) == OUTPUT_WIDTH * OUTPUT_HEIGHT * 2:
                    sdk_depth = np.frombuffer(data, dtype=np.uint16).reshape(OUTPUT_HEIGHT, OUTPUT_WIDTH)
            except:
                pass

        return raw_frame, sdk_depth

    def run(self):
        """Main loop"""
        print("Starting CubeEye SDK Live Visualization")
        print("========================================")

        if not self.start_sdk_capture():
            return

        # Wait for first frames
        print("Waiting for frames...")
        for _ in range(50):  # Wait up to 5 seconds
            raw, sdk = self.get_latest_frames()
            if raw or sdk:
                break
            time.sleep(0.1)
        else:
            print("Timeout waiting for frames. Check if sensor is connected.")
            self.stop_sdk_capture()
            return

        cv2.namedWindow("CubeEye Live", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("CubeEye Live", self.mouse_callback)

        print("\nControls:")
        print("  M: Toggle mode (Custom/SDK/Side-by-side)")
        print("  C: Cycle colormap")
        print("  S: Save frame")
        print("  Q: Quit")
        print()

        frame_count = 0
        while self.running:
            # Get latest frames
            raw_frame, sdk_depth = self.get_latest_frames()

            # Extract custom depth if we have raw frame
            custom_depth = None
            if raw_frame:
                try:
                    custom_depth = self.extractor.extract_depth(raw_frame, interpolate=True)
                except:
                    pass

            # Create display based on mode
            if self.mode == 'custom' and custom_depth is not None:
                display = self.depth_to_color(custom_depth)
                display = self.draw_overlay(display, custom_depth, "CUSTOM")
            elif self.mode == 'sdk' and sdk_depth is not None:
                display = self.depth_to_color(sdk_depth)
                display = self.draw_overlay(display, sdk_depth, "SDK")
            elif self.mode == 'sidebyside':
                # Side by side comparison
                left = self.depth_to_color(custom_depth) if custom_depth is not None else np.zeros((OUTPUT_HEIGHT, OUTPUT_WIDTH, 3), dtype=np.uint8)
                right = self.depth_to_color(sdk_depth) if sdk_depth is not None else np.zeros((OUTPUT_HEIGHT, OUTPUT_WIDTH, 3), dtype=np.uint8)

                # Add labels
                cv2.putText(left, "CUSTOM", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(right, "SDK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                display = np.hstack([left, right])

                # Show depth values
                x, y = self.cursor_pos
                if x < OUTPUT_WIDTH and custom_depth is not None:
                    cv2.putText(display, f"Custom: {custom_depth[y, x]}mm", (10, OUTPUT_HEIGHT - 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                if sdk_depth is not None:
                    sdk_x = x if x < OUTPUT_WIDTH else x - OUTPUT_WIDTH
                    cv2.putText(display, f"SDK: {sdk_depth[y, sdk_x]}mm", (10, OUTPUT_HEIGHT - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                # Fallback - show what we have
                if custom_depth is not None:
                    display = self.depth_to_color(custom_depth)
                    display = self.draw_overlay(display, custom_depth, "CUSTOM")
                elif sdk_depth is not None:
                    display = self.depth_to_color(sdk_depth)
                    display = self.draw_overlay(display, sdk_depth, "SDK")
                else:
                    display = np.zeros((OUTPUT_HEIGHT, OUTPUT_WIDTH, 3), dtype=np.uint8)
                    cv2.putText(display, "Waiting for frames...", (150, 240),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            self.update_fps()
            frame_count += 1

            cv2.imshow("CubeEye Live", display)

            key = cv2.waitKey(30) & 0xFF

            if key == ord('q') or key == 27:
                self.running = False
            elif key == ord('m'):
                modes = ['custom', 'sdk', 'sidebyside']
                self.mode = modes[(modes.index(self.mode) + 1) % len(modes)]
                print(f"Mode: {self.mode}")
            elif key == ord('c'):
                self.colormap_idx = (self.colormap_idx + 1) % len(COLORMAPS)
            elif key == ord('s'):
                if custom_depth is not None:
                    cv2.imwrite(f"frame_{frame_count:05d}_custom.png",
                               self.depth_to_color(custom_depth))
                if sdk_depth is not None:
                    cv2.imwrite(f"frame_{frame_count:05d}_sdk.png",
                               self.depth_to_color(sdk_depth))
                print(f"Saved frame {frame_count}")

        cv2.destroyAllWindows()
        self.stop_sdk_capture()
        print(f"Total frames: {frame_count}")


def main():
    viz = SDKLiveVisualizer()
    viz.run()


if __name__ == "__main__":
    main()

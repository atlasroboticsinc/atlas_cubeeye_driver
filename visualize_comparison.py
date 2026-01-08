#!/usr/bin/env python3
"""
CubeEye SDK vs Custom Driver Comparison GUI

Side-by-side comparison of SDK output vs our custom depth extraction,
with live SDK filter parameter controls.

This version uses subprocess-based SDK capture since the SDK Python bindings
require Python 3.8 which may not be installed.

Usage:
    python3 visualize_comparison.py

Requirements:
    - build/sdk_capture (C++ SDK capture tool)
    - build/libsimple_hook.so (V4L2 frame capture hook)
    - numpy, opencv-python

Controls:
    Left-click: Show depth at cursor (both SDK and Custom values)
    D: Toggle Depth/Amplitude view
    C: Cycle colormap
    S: Save comparison frame
    Q/ESC: Quit

    SDK Filter Controls:
    1: Toggle Flying Pixel Filter
    2: Toggle Median Filter
    3: Toggle Outlier Remove Filter
    4: Toggle Phase Noise Filter
    5: Toggle Scattering Filter
    6: Toggle Auto Exposure
    +/-: Adjust Amplitude Threshold Min
    [/]: Adjust Integration Time
"""

import sys
import os
import subprocess
import threading
import time
import signal
import numpy as np
import cv2
from pathlib import Path
from collections import deque

# Add local path for depth extractor
sys.path.insert(0, str(Path(__file__).parent))
from depth_extractor_fast import FastDepthExtractor

# Constants
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
RAW_FRAME_SIZE = 771200
MAX_DEPTH_MM = 7500


class FilterConfig:
    """SDK filter configuration"""
    def __init__(self):
        # Boolean filters
        self.flying_pixel_remove_filter = True
        self.median_filter = False
        self.outlier_remove_filter = True
        self.phase_noise_filter = False
        self.scattering_filter = False
        self.auto_exposure = True
        self.depth_undistortion = False  # SDK undistortion
        # Numeric parameters
        self.amplitude_threshold_min = 0
        self.amplitude_threshold_max = 65535
        self.integration_time = 1000
        self.flying_pixel_remove_threshold = 3000
        self.scattering_filter_threshold = 700
        self.depth_range_min = 150
        self.depth_range_max = 65535

    def write_to_file(self, filepath):
        """Write config to file in key=value format"""
        with open(filepath, 'w') as f:
            f.write(f"# SDK Filter Configuration\n")
            f.write(f"flying_pixel_remove_filter={1 if self.flying_pixel_remove_filter else 0}\n")
            f.write(f"median_filter={1 if self.median_filter else 0}\n")
            f.write(f"outlier_remove_filter={1 if self.outlier_remove_filter else 0}\n")
            f.write(f"phase_noise_filter={1 if self.phase_noise_filter else 0}\n")
            f.write(f"scattering_filter={1 if self.scattering_filter else 0}\n")
            f.write(f"auto_exposure={1 if self.auto_exposure else 0}\n")
            f.write(f"depth_undistortion={1 if self.depth_undistortion else 0}\n")
            f.write(f"amplitude_threshold_min={self.amplitude_threshold_min}\n")
            f.write(f"amplitude_threshold_max={self.amplitude_threshold_max}\n")
            f.write(f"integration_time={self.integration_time}\n")
            f.write(f"flying_pixel_remove_threshold={self.flying_pixel_remove_threshold}\n")
            f.write(f"scattering_filter_threshold={self.scattering_filter_threshold}\n")
            f.write(f"depth_range_min={self.depth_range_min}\n")
            f.write(f"depth_range_max={self.depth_range_max}\n")

    def get_filter_status_str(self):
        """Get compact status string for display"""
        status = []
        if self.flying_pixel_remove_filter: status.append("FlyPx")
        if self.median_filter: status.append("Med")
        if self.outlier_remove_filter: status.append("Outl")
        if self.phase_noise_filter: status.append("PhNs")
        if self.scattering_filter: status.append("Scat")
        if self.auto_exposure: status.append("AE")
        if self.depth_undistortion: status.append("Undist")
        return " ".join(status) if status else "None"


COLORMAPS = [
    ('JET', cv2.COLORMAP_JET),
    ('TURBO', cv2.COLORMAP_TURBO),
    ('VIRIDIS', cv2.COLORMAP_VIRIDIS),
    ('INFERNO', cv2.COLORMAP_INFERNO),
]


class SDKCaptureProcess:
    """Manages SDK capture subprocess with V4L2 hook"""

    def __init__(self, output_dir, filter_config=None):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.process = None
        self.running = False

        # Paths
        self.project_dir = Path(__file__).parent
        self.sdk_capture = self.project_dir / "build" / "sdk_capture"
        self.hook_lib = self.project_dir / "build" / "libsimple_hook.so"

        # Track last-read frame numbers to detect new frames
        self.last_raw_frame_num = -1
        self.last_sdk_frame_num = -1
        self.last_sdk_amp_frame_num = -1

        # Filter configuration
        self.filter_config = filter_config or FilterConfig()
        self.config_file = self.output_dir / "filter_config.txt"

    def start(self, num_frames=999999):
        """Start SDK capture with V4L2 hook"""
        if not self.sdk_capture.exists():
            raise FileNotFoundError(f"SDK capture not found: {self.sdk_capture}")
        if not self.hook_lib.exists():
            raise FileNotFoundError(f"Hook library not found: {self.hook_lib}")

        # Clean old files
        for f in self.output_dir.glob("*.raw"):
            f.unlink()
        for f in self.output_dir.glob("*.bin"):
            f.unlink()
        # Also clean data subdirectory (where sdk_capture saves depth files)
        data_dir = self.output_dir / "data"
        data_dir.mkdir(exist_ok=True)
        for f in data_dir.glob("*.raw"):
            f.unlink()

        # SDK library paths
        sdk_path = Path.home() / "development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0"
        lib_paths = [
            str(sdk_path / "lib"),
            str(sdk_path / "thirdparty/libopencv/lib"),
            str(sdk_path / "thirdparty/libffmpeg/lib"),
            str(sdk_path / "thirdparty/liblive555/lib/Release"),
        ]

        # Write initial filter config
        self.filter_config.write_to_file(self.config_file)

        # Set up environment with hook and SDK libraries
        env = os.environ.copy()
        env['LD_PRELOAD'] = str(self.hook_lib)
        env['HOOK_OUTPUT'] = str(self.output_dir)  # Hook uses HOOK_OUTPUT
        env['LD_LIBRARY_PATH'] = ':'.join(lib_paths) + ':' + env.get('LD_LIBRARY_PATH', '')
        env['SDK_FILTER_CONFIG'] = str(self.config_file)  # Filter config file

        # Start SDK capture
        self.process = subprocess.Popen(
            [str(self.sdk_capture), str(num_frames)],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            cwd=str(self.output_dir),
            preexec_fn=os.setsid  # Create new process group for clean termination
        )
        self.running = True
        print(f"Started SDK capture (PID: {self.process.pid})")

        # Start output reader thread
        self.output_thread = threading.Thread(target=self._read_output, daemon=True)
        self.output_thread.start()

        return True

    def _read_output(self):
        """Read and print SDK capture output"""
        while self.running and self.process.poll() is None:
            try:
                line = self.process.stdout.readline()
                if line:
                    print(f"[SDK] {line.decode().strip()}")
            except:
                break

    def update_filter_config(self):
        """Write current filter config to file (triggers SDK reload)"""
        self.filter_config.write_to_file(self.config_file)

    def stop(self):
        """Stop SDK capture"""
        self.running = False
        if self.process:
            try:
                # Send SIGTERM to process group
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=3)
            except:
                try:
                    self.process.kill()
                except:
                    pass
            print("SDK capture stopped")

    def _extract_frame_num(self, filepath):
        """Extract frame number from filename like raw_000123.bin or sync_depth_000123.raw"""
        import re
        match = re.search(r'_(\d+)\.(?:bin|raw)$', filepath.name)
        if match:
            return int(match.group(1))
        return -1

    def get_latest_frames(self):
        """Get latest raw and SDK depth/amplitude frames (only returns NEW frames)"""
        raw_frame = None
        sdk_depth = None
        sdk_amplitude = None

        # Find latest raw frame from hook by frame number
        raw_files = list(self.output_dir.glob("raw_*.bin"))
        if raw_files:
            # Sort by frame number (extracted from filename)
            raw_files.sort(key=self._extract_frame_num, reverse=True)
            latest_file = raw_files[0]
            frame_num = self._extract_frame_num(latest_file)

            # Only read if this is a new frame
            if frame_num > self.last_raw_frame_num:
                try:
                    data = latest_file.read_bytes()
                    if len(data) == RAW_FRAME_SIZE:
                        raw_frame = data
                        self.last_raw_frame_num = frame_num
                except Exception as e:
                    pass

        # Find latest SDK depth (sdk_capture saves to data/ subdirectory)
        sdk_files = list(self.output_dir.glob("**/sync_depth_*.raw"))
        if sdk_files:
            # Sort by frame number
            sdk_files.sort(key=self._extract_frame_num, reverse=True)
            latest_file = sdk_files[0]
            frame_num = self._extract_frame_num(latest_file)

            # Only read if this is a new frame
            if frame_num > self.last_sdk_frame_num:
                try:
                    data = latest_file.read_bytes()
                    if len(data) == OUTPUT_WIDTH * OUTPUT_HEIGHT * 2:
                        sdk_depth = np.frombuffer(data, dtype=np.uint16).reshape(
                            OUTPUT_HEIGHT, OUTPUT_WIDTH)
                        self.last_sdk_frame_num = frame_num
                except Exception as e:
                    pass

        # Find latest SDK amplitude
        amp_files = list(self.output_dir.glob("**/sync_amplitude_*.raw"))
        if amp_files:
            amp_files.sort(key=self._extract_frame_num, reverse=True)
            latest_file = amp_files[0]
            frame_num = self._extract_frame_num(latest_file)

            if frame_num > self.last_sdk_amp_frame_num:
                try:
                    data = latest_file.read_bytes()
                    if len(data) == OUTPUT_WIDTH * OUTPUT_HEIGHT * 2:
                        sdk_amplitude = np.frombuffer(data, dtype=np.uint16).reshape(
                            OUTPUT_HEIGHT, OUTPUT_WIDTH)
                        self.last_sdk_amp_frame_num = frame_num
                except Exception as e:
                    pass

        return raw_frame, sdk_depth, sdk_amplitude

    def cleanup_old_files(self, keep_count=100):
        """Remove old files to prevent disk filling up"""
        # Clean raw files
        raw_files = list(self.output_dir.glob("raw_*.bin"))
        if len(raw_files) > keep_count:
            raw_files.sort(key=self._extract_frame_num)
            for f in raw_files[:-keep_count]:
                try:
                    f.unlink()
                except:
                    pass

        # Clean SDK depth files
        sdk_files = list(self.output_dir.glob("**/sync_depth_*.raw"))
        if len(sdk_files) > keep_count:
            sdk_files.sort(key=self._extract_frame_num)
            for f in sdk_files[:-keep_count]:
                try:
                    f.unlink()
                except:
                    pass

        # Clean SDK amplitude files
        amp_files = list(self.output_dir.glob("**/sync_amplitude_*.raw"))
        if len(amp_files) > keep_count:
            amp_files.sort(key=self._extract_frame_num)
            for f in amp_files[:-keep_count]:
                try:
                    f.unlink()
                except:
                    pass


class ComparisonGUI:
    """Side-by-side comparison GUI for SDK vs Custom extraction"""

    def __init__(self):
        self.extractor = FastDepthExtractor(apply_gradient_correction=True, apply_undistortion=False)
        self.sdk_capture = None
        self.temp_dir = Path("/tmp/cubeeye_comparison")
        self.filter_config = FilterConfig()
        self.our_undistortion = False  # Our driver's undistortion state

        # Current frames
        self.sdk_depth = None
        self.sdk_amplitude = None
        self.custom_depth = None
        self.custom_amplitude = None

        # State
        self.running = True
        self.view_mode = 'depth'  # 'depth' or 'amplitude'
        self.colormap_idx = 0
        self.depth_range = (200, 5000)
        self.cursor_pos = (OUTPUT_WIDTH // 2, OUTPUT_HEIGHT // 2)

        # FPS tracking
        self.frame_times = deque(maxlen=30)
        self.fps = 0.0

        # Statistics
        self.frame_count = 0
        self.diff_history = deque(maxlen=100)

    def depth_to_color(self, depth_frame):
        """Convert depth to colorized image"""
        if depth_frame is None:
            return np.zeros((OUTPUT_HEIGHT, OUTPUT_WIDTH, 3), dtype=np.uint8)

        min_d, max_d = self.depth_range
        normalized = np.clip(depth_frame.astype(np.float32), min_d, max_d)
        normalized = ((normalized - min_d) / (max_d - min_d) * 255).astype(np.uint8)

        _, colormap = COLORMAPS[self.colormap_idx]
        colored = cv2.applyColorMap(normalized, colormap)
        colored[depth_frame == 0] = [0, 0, 0]

        return colored

    def amplitude_to_gray(self, amplitude_frame):
        """Convert amplitude to grayscale image"""
        if amplitude_frame is None:
            return np.zeros((OUTPUT_HEIGHT, OUTPUT_WIDTH, 3), dtype=np.uint8)

        # Normalize amplitude (typically 0-4095 for 12-bit)
        normalized = np.clip(amplitude_frame.astype(np.float32) / 4095 * 255, 0, 255).astype(np.uint8)
        return cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)

    def compute_diff_image(self, sdk_depth, custom_depth):
        """Compute difference image between SDK and custom depth"""
        if sdk_depth is None or custom_depth is None:
            return np.zeros((OUTPUT_HEIGHT, OUTPUT_WIDTH, 3), dtype=np.uint8)

        # Compute absolute difference
        diff = np.abs(sdk_depth.astype(np.int32) - custom_depth.astype(np.int32))

        # Create heatmap (green = good, red = bad)
        diff_normalized = np.clip(diff / 100 * 255, 0, 255).astype(np.uint8)

        # Create color image: green where diff is small, red where large
        result = np.zeros((OUTPUT_HEIGHT, OUTPUT_WIDTH, 3), dtype=np.uint8)
        result[:, :, 1] = 255 - diff_normalized  # Green channel (inverse of diff)
        result[:, :, 2] = diff_normalized  # Red channel

        # Black out invalid pixels
        invalid = (sdk_depth == 0) | (custom_depth == 0)
        result[invalid] = [50, 50, 50]

        return result

    def draw_info_panel(self, image):
        """Draw info panel on image"""
        h, w = image.shape[:2]

        # Semi-transparent background (taller to show filter info + undistortion)
        overlay = image.copy()
        cv2.rectangle(overlay, (5, 5), (280, 280), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)

        # Cursor position and values
        cx, cy = self.cursor_pos
        # Adjust for side-by-side (left = SDK, right = Custom)
        if cx >= OUTPUT_WIDTH * 2:
            cx_adj = cx - OUTPUT_WIDTH * 2
            side = "diff"
        elif cx >= OUTPUT_WIDTH:
            cx_adj = cx - OUTPUT_WIDTH
            side = "custom"
        else:
            cx_adj = cx
            side = "sdk"

        cx_adj = max(0, min(cx_adj, OUTPUT_WIDTH - 1))
        cy = max(0, min(cy, OUTPUT_HEIGHT - 1))

        y_pos = 25
        cv2.putText(image, f"Mode: {self.view_mode.upper()}", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        y_pos += 20
        cv2.putText(image, f"Cursor: ({cx_adj}, {cy}) [{side}]", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        y_pos += 25
        sdk_val = self.sdk_depth[cy, cx_adj] if self.sdk_depth is not None else 0
        custom_val = self.custom_depth[cy, cx_adj] if self.custom_depth is not None else 0
        diff = abs(int(sdk_val) - int(custom_val))

        cv2.putText(image, f"SDK Depth: {sdk_val} mm", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_pos += 20
        cv2.putText(image, f"Custom Depth: {custom_val} mm", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        y_pos += 20

        diff_color = (0, 255, 0) if diff < 10 else (0, 165, 255) if diff < 50 else (0, 0, 255)
        cv2.putText(image, f"Difference: {diff} mm", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, diff_color, 1)

        y_pos += 25
        cv2.putText(image, f"FPS: {self.fps:.1f}", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        y_pos += 20
        cv2.putText(image, f"Display Frame: {self.frame_count}", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Show actual captured frame numbers
        if self.sdk_capture:
            y_pos += 20
            cv2.putText(image, f"Raw#: {self.sdk_capture.last_raw_frame_num}  SDK#: {self.sdk_capture.last_sdk_frame_num}",
                        (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 255), 1)

        # Statistics
        if len(self.diff_history) > 0:
            y_pos += 25
            mean_diff = np.mean(self.diff_history)
            cv2.putText(image, f"Avg Center Diff: {mean_diff:.1f} mm", (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        # Filter status
        y_pos += 25
        cv2.putText(image, "SDK Filters:", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 100), 1)
        y_pos += 18
        filter_str = self.filter_config.get_filter_status_str()
        cv2.putText(image, f"  {filter_str}", (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y_pos += 18
        cv2.putText(image, f"  AmpMin:{self.filter_config.amplitude_threshold_min} IntT:{self.filter_config.integration_time}",
                    (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Our undistortion status
        y_pos += 18
        undist_color = (0, 255, 0) if self.our_undistortion else (100, 100, 100)
        cv2.putText(image, f"Our Undist: {'ON' if self.our_undistortion else 'OFF'}",
                    (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.4, undist_color, 1)

        return image

    def draw_crosshair(self, image):
        """Draw crosshair at cursor position"""
        x, y = self.cursor_pos
        h, w = image.shape[:2]

        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))

        cv2.line(image, (x - 20, y), (x + 20, y), (255, 255, 255), 1)
        cv2.line(image, (x, y - 20), (x, y + 20), (255, 255, 255), 1)
        cv2.circle(image, (x, y), 5, (255, 255, 255), 1)

        return image

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events"""
        if event in [cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN]:
            self.cursor_pos = (x, y)

    def update_fps(self):
        """Update FPS counter"""
        now = time.time()
        self.frame_times.append(now)
        if len(self.frame_times) > 1:
            dt = self.frame_times[-1] - self.frame_times[0]
            if dt > 0:
                self.fps = (len(self.frame_times) - 1) / dt

    def run(self):
        """Main loop"""
        print("\n" + "=" * 60)
        print("CubeEye SDK vs Custom Driver Comparison")
        print("=" * 60)

        # Start SDK capture with filter config
        self.sdk_capture = SDKCaptureProcess(self.temp_dir, self.filter_config)
        try:
            self.sdk_capture.start()
        except FileNotFoundError as e:
            print(f"Error: {e}")
            print("\nMake sure to build the SDK capture tool:")
            print("  cd build && cmake .. && make sdk_capture libsimple_hook.so")
            return

        # Wait for first frames
        print("Waiting for frames...")
        for _ in range(100):
            raw, sdk, sdk_amp = self.sdk_capture.get_latest_frames()
            if raw or sdk:
                print("Frames received!")
                break
            time.sleep(0.1)
        else:
            print("Timeout waiting for frames. Check if sensor is connected.")
            self.sdk_capture.stop()
            return

        cv2.namedWindow("SDK vs Custom Comparison", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("SDK vs Custom Comparison", self.mouse_callback)

        print("\nControls:")
        print("  D: Toggle Depth/Amplitude view")
        print("  C: Cycle colormap")
        print("  S: Save comparison frame")
        print("  Q/ESC: Quit")
        print("\nSDK Filter Controls:")
        print("  1: Toggle Flying Pixel Filter")
        print("  2: Toggle Median Filter")
        print("  3: Toggle Outlier Remove Filter")
        print("  4: Toggle Phase Noise Filter")
        print("  5: Toggle Scattering Filter")
        print("  6: Toggle Auto Exposure")
        print("  7: Toggle SDK Undistortion")
        print("  8: Toggle Our Undistortion")
        print("  +/-: Adjust Amplitude Threshold Min")
        print("  [/]: Adjust Integration Time")
        print()

        cleanup_counter = 0
        while self.running:
            # Get latest frames
            raw_frame, sdk_depth, sdk_amplitude = self.sdk_capture.get_latest_frames()

            if sdk_depth is not None:
                self.sdk_depth = sdk_depth

            if sdk_amplitude is not None:
                self.sdk_amplitude = sdk_amplitude

            if raw_frame is not None:
                self.custom_depth = self.extractor.extract_depth(raw_frame, interpolate=True)
                self.custom_amplitude = self.extractor.extract_amplitude(raw_frame, interpolate=True)

            # Periodic cleanup (every 300 frames, ~20 seconds at 15fps)
            cleanup_counter += 1
            if cleanup_counter >= 300:
                self.sdk_capture.cleanup_old_files()
                cleanup_counter = 0

            # Create display
            if self.view_mode == 'depth':
                left = self.depth_to_color(self.sdk_depth)
                middle = self.depth_to_color(self.custom_depth)
                right = self.compute_diff_image(self.sdk_depth, self.custom_depth)

                cv2.putText(left, "SDK Depth", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(middle, "Custom Depth", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(right, "Difference", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                left = self.amplitude_to_gray(self.sdk_amplitude)
                middle = self.amplitude_to_gray(self.custom_amplitude)
                # Compute amplitude difference
                if self.sdk_amplitude is not None and self.custom_amplitude is not None:
                    diff = np.abs(self.sdk_amplitude.astype(np.int32) - self.custom_amplitude.astype(np.int32))
                    diff_normalized = np.clip(diff / 500 * 255, 0, 255).astype(np.uint8)
                    right = np.zeros((OUTPUT_HEIGHT, OUTPUT_WIDTH, 3), dtype=np.uint8)
                    right[:, :, 1] = 255 - diff_normalized
                    right[:, :, 2] = diff_normalized
                else:
                    right = np.zeros_like(left)

                cv2.putText(left, "SDK Amplitude", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(middle, "Custom Amplitude", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(right, "Amp Difference", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Combine three views
            display = np.hstack([left, middle, right])

            # Draw overlays
            display = self.draw_info_panel(display)
            display = self.draw_crosshair(display)

            # Controls at bottom
            cv2.putText(display, "D:depth/amp  C:colormap  S:save  Q:quit  |  1-6:filters  7:SDK-undist  8:our-undist  +/-:ampMin",
                       (10, display.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

            self.update_fps()
            self.frame_count += 1

            # Update statistics
            if self.sdk_depth is not None and self.custom_depth is not None:
                center_sdk = self.sdk_depth[OUTPUT_HEIGHT // 2, OUTPUT_WIDTH // 2]
                center_custom = self.custom_depth[OUTPUT_HEIGHT // 2, OUTPUT_WIDTH // 2]
                if center_sdk > 0 and center_custom > 0:
                    self.diff_history.append(abs(int(center_sdk) - int(center_custom)))

            cv2.imshow("SDK vs Custom Comparison", display)

            # Handle keyboard
            key = cv2.waitKey(30) & 0xFF

            if key == ord('q') or key == 27:
                self.running = False
            elif key == ord('d'):
                self.view_mode = 'amplitude' if self.view_mode == 'depth' else 'depth'
                print(f"View mode: {self.view_mode}")
            elif key == ord('c'):
                self.colormap_idx = (self.colormap_idx + 1) % len(COLORMAPS)
                print(f"Colormap: {COLORMAPS[self.colormap_idx][0]}")
            elif key == ord('s'):
                filename = f"comparison_{self.frame_count:05d}.png"
                cv2.imwrite(filename, display)
                print(f"Saved: {filename}")
            # Filter controls
            elif key == ord('1'):
                self.filter_config.flying_pixel_remove_filter = not self.filter_config.flying_pixel_remove_filter
                self.sdk_capture.update_filter_config()
                print(f"Flying Pixel Filter: {'ON' if self.filter_config.flying_pixel_remove_filter else 'OFF'}")
            elif key == ord('2'):
                self.filter_config.median_filter = not self.filter_config.median_filter
                self.sdk_capture.update_filter_config()
                print(f"Median Filter: {'ON' if self.filter_config.median_filter else 'OFF'}")
            elif key == ord('3'):
                self.filter_config.outlier_remove_filter = not self.filter_config.outlier_remove_filter
                self.sdk_capture.update_filter_config()
                print(f"Outlier Remove Filter: {'ON' if self.filter_config.outlier_remove_filter else 'OFF'}")
            elif key == ord('4'):
                self.filter_config.phase_noise_filter = not self.filter_config.phase_noise_filter
                self.sdk_capture.update_filter_config()
                print(f"Phase Noise Filter: {'ON' if self.filter_config.phase_noise_filter else 'OFF'}")
            elif key == ord('5'):
                self.filter_config.scattering_filter = not self.filter_config.scattering_filter
                self.sdk_capture.update_filter_config()
                print(f"Scattering Filter: {'ON' if self.filter_config.scattering_filter else 'OFF'}")
            elif key == ord('6'):
                self.filter_config.auto_exposure = not self.filter_config.auto_exposure
                self.sdk_capture.update_filter_config()
                print(f"Auto Exposure: {'ON' if self.filter_config.auto_exposure else 'OFF'}")
            elif key == ord('+') or key == ord('='):
                self.filter_config.amplitude_threshold_min = min(255, self.filter_config.amplitude_threshold_min + 10)
                self.sdk_capture.update_filter_config()
                print(f"Amplitude Threshold Min: {self.filter_config.amplitude_threshold_min}")
            elif key == ord('-') or key == ord('_'):
                self.filter_config.amplitude_threshold_min = max(0, self.filter_config.amplitude_threshold_min - 10)
                self.sdk_capture.update_filter_config()
                print(f"Amplitude Threshold Min: {self.filter_config.amplitude_threshold_min}")
            elif key == ord(']'):
                self.filter_config.integration_time = min(2000, self.filter_config.integration_time + 50)
                self.sdk_capture.update_filter_config()
                print(f"Integration Time: {self.filter_config.integration_time}")
            elif key == ord('['):
                self.filter_config.integration_time = max(100, self.filter_config.integration_time - 50)
                self.sdk_capture.update_filter_config()
                print(f"Integration Time: {self.filter_config.integration_time}")
            elif key == ord('7'):
                self.filter_config.depth_undistortion = not self.filter_config.depth_undistortion
                self.sdk_capture.update_filter_config()
                print(f"SDK Depth Undistortion: {'ON' if self.filter_config.depth_undistortion else 'OFF'}")
            elif key == ord('8'):
                self.our_undistortion = not self.our_undistortion
                self.extractor.set_undistortion(self.our_undistortion)
                print(f"Our Undistortion: {'ON' if self.our_undistortion else 'OFF'}")

        # Cleanup
        print("\nStopping...")
        self.sdk_capture.stop()
        cv2.destroyAllWindows()
        print(f"Total frames: {self.frame_count}")

        if len(self.diff_history) > 0:
            print(f"Average center pixel difference: {np.mean(self.diff_history):.1f} mm")


def main():
    gui = ComparisonGUI()
    gui.run()


if __name__ == "__main__":
    main()

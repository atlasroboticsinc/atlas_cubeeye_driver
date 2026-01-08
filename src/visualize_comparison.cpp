/**
 * visualize_comparison.cpp - SDK vs Custom Depth Comparison GUI
 *
 * Real-time side-by-side comparison of CubeEye SDK output vs our custom
 * depth extraction, with live SDK filter controls.
 *
 * Usage: ./visualize_comparison
 *
 * Controls:
 *   Mouse click: Show depth at cursor (both SDK and Custom)
 *   D: Toggle Depth/Amplitude view
 *   C: Cycle colormap
 *   G: Toggle gradient correction (custom)
 *   S: Save comparison frame
 *   1: Toggle flying_pixel_remove_filter
 *   2: Toggle median_filter
 *   3: Toggle outlier_remove_filter
 *   4: Toggle phase_noise_filter
 *   5: Toggle scattering_filter
 *   6: Toggle auto_exposure
 *   Q/ESC: Quit
 */

#include <iostream>
#include <iomanip>
#include <atomic>
#include <mutex>
#include <chrono>
#include <thread>
#include <csignal>
#include <deque>

#include <opencv2/opencv.hpp>

#include "CubeEye/CubeEyeSink.h"
#include "CubeEye/CubeEyeCamera.h"
#include "CubeEye/CubeEyeBasicFrame.h"
#include "CubeEye/CubeEyeProperty.h"

#include "cubeeye_depth.h"

namespace ms = meere::sensor;

// Constants
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int MIN_DEPTH = 200;
constexpr int MAX_DEPTH = 5000;

// Global state
static std::atomic<bool> g_running{true};
static std::mutex g_frame_mutex;

// Current frames (protected by mutex)
static cv::Mat g_sdk_depth;
static cv::Mat g_sdk_amplitude;
static cv::Mat g_raw_frame;
static bool g_new_frame = false;

// GUI state
static int g_cursor_x = WIDTH / 2;
static int g_cursor_y = HEIGHT / 2;
static int g_colormap = cv::COLORMAP_JET;
static bool g_show_depth = true;  // true = depth, false = amplitude
static int g_frame_count = 0;

// FPS tracking
static std::deque<double> g_frame_times;
static double g_fps = 0.0;

// SDK filter states
struct FilterState {
    std::string name;
    std::string key;
    bool enabled;
    bool is_bool;
};

static std::vector<FilterState> g_filters = {
    {"flying_pixel_remove_filter", "1", true, true},
    {"median_filter", "2", false, true},
    {"outlier_remove_filter", "3", true, true},
    {"phase_noise_filter", "4", false, true},
    {"scattering_filter", "5", false, true},
    {"auto_exposure", "6", true, true},
};

// Camera pointer for filter control
static ms::sptr_camera g_camera;

// Signal handler
void signal_handler(int sig) {
    std::cout << "\nReceived signal " << sig << ", stopping..." << std::endl;
    g_running = false;
}

// Frame sink
class ComparisonSink : public ms::sink {
public:
    std::string name() const override {
        return "ComparisonSink";
    }

    void onCubeEyeCameraState(const ms::ptr_source source, ms::CameraState state) override {
        const char* state_str = "Unknown";
        switch (state) {
            case ms::CameraState::Released: state_str = "Released"; break;
            case ms::CameraState::Prepared: state_str = "Prepared"; break;
            case ms::CameraState::Stopped: state_str = "Stopped"; break;
            case ms::CameraState::Running: state_str = "Running"; break;
        }
        std::cout << "[State] " << state_str << std::endl;
    }

    void onCubeEyeCameraError(const ms::ptr_source source, ms::CameraError error) override {
        std::cerr << "[Error] Camera error: " << static_cast<int>(error) << std::endl;
    }

    void onCubeEyeFrameList(const ms::ptr_source source, const ms::sptr_frame_list& frames) override {
        if (!g_running) return;

        std::lock_guard<std::mutex> lock(g_frame_mutex);

        for (const auto& frame : (*frames)) {
            int width = frame->frameWidth();
            int height = frame->frameHeight();

            if (frame->frameType() == ms::FrameType::Depth) {
                if (frame->frameDataType() == ms::DataType::U16) {
                    auto basic = ms::frame_cast_basic16u(frame);
                    auto data = basic->frameData();
                    g_sdk_depth = cv::Mat(height, width, CV_16UC1,
                                          const_cast<uint16_t*>(data->data())).clone();
                }
            }
            else if (frame->frameType() == ms::FrameType::Amplitude) {
                if (frame->frameDataType() == ms::DataType::U16) {
                    auto basic = ms::frame_cast_basic16u(frame);
                    auto data = basic->frameData();
                    g_sdk_amplitude = cv::Mat(height, width, CV_16UC1,
                                              const_cast<uint16_t*>(data->data())).clone();
                }
            }
            else if (frame->frameType() == ms::FrameType::Raw) {
                auto basic = ms::frame_cast_basic8u(frame);
                if (basic) {
                    auto data = basic->frameData();
                    g_raw_frame = cv::Mat(1, static_cast<int>(data->size()), CV_8UC1,
                                          const_cast<uint8_t*>(data->data())).clone();
                }
            }
        }
        g_new_frame = true;
    }
};

static ComparisonSink g_sink;

// Apply colormap to depth
cv::Mat depth_to_color(const cv::Mat& depth, int colormap) {
    if (depth.empty()) {
        return cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    }

    cv::Mat normalized;
    depth.convertTo(normalized, CV_32F);

    // Normalize to range
    normalized = (normalized - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH) * 255.0;
    cv::threshold(normalized, normalized, 255, 255, cv::THRESH_TRUNC);
    cv::threshold(normalized, normalized, 0, 0, cv::THRESH_TOZERO);

    normalized.convertTo(normalized, CV_8UC1);

    cv::Mat colored;
    cv::applyColorMap(normalized, colored, colormap);

    // Black out invalid pixels
    cv::Mat mask = (depth == 0);
    colored.setTo(cv::Scalar(0, 0, 0), mask);

    return colored;
}

// Apply grayscale to amplitude
cv::Mat amplitude_to_gray(const cv::Mat& amplitude) {
    if (amplitude.empty()) {
        return cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    }

    cv::Mat normalized;
    amplitude.convertTo(normalized, CV_32F);
    normalized = normalized / 4095.0 * 255.0;
    cv::threshold(normalized, normalized, 255, 255, cv::THRESH_TRUNC);
    normalized.convertTo(normalized, CV_8UC1);

    cv::Mat colored;
    cv::cvtColor(normalized, colored, cv::COLOR_GRAY2BGR);
    return colored;
}

// Compute difference heatmap
cv::Mat compute_diff(const cv::Mat& sdk, const cv::Mat& custom) {
    if (sdk.empty() || custom.empty()) {
        return cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    }

    cv::Mat sdk_f, custom_f;
    sdk.convertTo(sdk_f, CV_32F);
    custom.convertTo(custom_f, CV_32F);

    cv::Mat diff;
    cv::absdiff(sdk_f, custom_f, diff);

    // Scale: 0-100mm diff -> 0-255
    diff = diff / 100.0 * 255.0;
    cv::threshold(diff, diff, 255, 255, cv::THRESH_TRUNC);
    diff.convertTo(diff, CV_8UC1);

    // Create heatmap: green = good (low diff), red = bad (high diff)
    cv::Mat result = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    std::vector<cv::Mat> channels(3);
    channels[0] = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);  // Blue
    channels[1] = 255 - diff;  // Green (inverse of diff)
    channels[2] = diff;        // Red
    cv::merge(channels, result);

    // Gray out invalid pixels
    cv::Mat invalid = (sdk == 0) | (custom == 0);
    result.setTo(cv::Scalar(50, 50, 50), invalid);

    return result;
}

// Mouse callback
void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_MOUSEMOVE || event == cv::EVENT_LBUTTONDOWN) {
        g_cursor_x = x;
        g_cursor_y = y;
    }
}

// Toggle SDK filter
void toggle_filter(int index) {
    if (index < 0 || index >= static_cast<int>(g_filters.size())) return;
    if (!g_camera) return;

    auto& filter = g_filters[index];
    filter.enabled = !filter.enabled;

    auto prop = ms::make_property_bool(filter.name, filter.enabled);
    auto result = g_camera->setProperty(prop);

    std::cout << "[Filter] " << filter.name << " = "
              << (filter.enabled ? "ON" : "OFF")
              << " (result: " << static_cast<int>(result) << ")" << std::endl;
}

// Update FPS
void update_fps() {
    auto now = std::chrono::steady_clock::now();
    double ts = std::chrono::duration<double>(now.time_since_epoch()).count();

    g_frame_times.push_back(ts);
    while (g_frame_times.size() > 30) {
        g_frame_times.pop_front();
    }

    if (g_frame_times.size() > 1) {
        double dt = g_frame_times.back() - g_frame_times.front();
        if (dt > 0) {
            g_fps = (g_frame_times.size() - 1) / dt;
        }
    }
}

int main(int argc, char* argv[]) {
    std::cout << "CubeEye SDK vs Custom Comparison GUI" << std::endl;
    std::cout << "=====================================" << std::endl;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize depth extractor
    cubeeye::DepthExtractor extractor(true);  // With gradient correction

    // Search for camera
    std::cout << "[Search] Looking for CubeEye cameras..." << std::endl;
    auto source_list = ms::search_camera_source();

    if (!source_list || source_list->size() == 0) {
        std::cerr << "[Error] No CubeEye camera found!" << std::endl;
        return 1;
    }

    auto source = (*source_list)[0];
    std::cout << "[Found] " << source->name() << " (SN: " << source->serialNumber() << ")" << std::endl;

    // Create camera
    g_camera = ms::create_camera(source);
    if (!g_camera) {
        std::cerr << "[Error] Failed to create camera!" << std::endl;
        return 1;
    }

    g_camera->addSink(&g_sink);

    // Prepare camera
    std::cout << "[Prepare] Preparing camera..." << std::endl;
    auto result = g_camera->prepare();
    if (result != ms::result::success) {
        std::cerr << "[Error] Failed to prepare: " << static_cast<int>(result) << std::endl;
        ms::destroy_camera(g_camera);
        return 1;
    }

    // Start with Raw + Depth + Amplitude
    std::cout << "[Run] Starting capture..." << std::endl;
    int wanted = ms::FrameType::Raw | ms::FrameType::Depth | ms::FrameType::Amplitude;
    result = g_camera->run(wanted);
    if (result != ms::result::success) {
        std::cerr << "[Error] Failed to start: " << static_cast<int>(result) << std::endl;
        ms::destroy_camera(g_camera);
        return 1;
    }

    // Create window
    cv::namedWindow("SDK vs Custom", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("SDK vs Custom", mouse_callback);

    std::cout << "\nControls:" << std::endl;
    std::cout << "  D: Toggle Depth/Amplitude" << std::endl;
    std::cout << "  C: Cycle colormap" << std::endl;
    std::cout << "  G: Toggle gradient correction" << std::endl;
    std::cout << "  S: Save frame" << std::endl;
    std::cout << "  1-6: Toggle SDK filters" << std::endl;
    std::cout << "  Q/ESC: Quit" << std::endl;
    std::cout << std::endl;

    // Buffers for custom extraction
    std::vector<uint16_t> custom_depth(WIDTH * HEIGHT);
    std::vector<uint16_t> custom_amplitude(WIDTH * HEIGHT);

    while (g_running) {
        cv::Mat sdk_depth_local, sdk_amplitude_local, raw_local;
        bool has_new = false;

        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            if (g_new_frame) {
                sdk_depth_local = g_sdk_depth.clone();
                sdk_amplitude_local = g_sdk_amplitude.clone();
                raw_local = g_raw_frame.clone();
                g_new_frame = false;
                has_new = true;
            }
        }

        // Extract custom depth from raw
        cv::Mat custom_depth_mat, custom_amplitude_mat;
        if (!raw_local.empty() && raw_local.total() == cubeeye::RAW_FRAME_SIZE) {
            extractor.ExtractDepthAndAmplitude(
                raw_local.data, raw_local.total(),
                custom_depth.data(), custom_amplitude.data(), true);

            custom_depth_mat = cv::Mat(HEIGHT, WIDTH, CV_16UC1, custom_depth.data()).clone();
            custom_amplitude_mat = cv::Mat(HEIGHT, WIDTH, CV_16UC1, custom_amplitude.data()).clone();
        }

        // Create display
        cv::Mat left, middle, right;

        if (g_show_depth) {
            left = depth_to_color(sdk_depth_local, g_colormap);
            middle = depth_to_color(custom_depth_mat, g_colormap);
            right = compute_diff(sdk_depth_local, custom_depth_mat);

            cv::putText(left, "SDK Depth", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            cv::putText(middle, "Custom Depth", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            cv::putText(right, "Difference", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        } else {
            left = amplitude_to_gray(sdk_amplitude_local);
            middle = amplitude_to_gray(custom_amplitude_mat);
            right = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

            cv::putText(left, "SDK Amplitude", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            cv::putText(middle, "Custom Amplitude", cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        }

        // Combine horizontally
        cv::Mat display;
        cv::hconcat(std::vector<cv::Mat>{left, middle, right}, display);

        // Draw info panel
        cv::rectangle(display, cv::Point(5, 5), cv::Point(300, 220),
                     cv::Scalar(0, 0, 0), -1);

        int y = 25;
        std::string mode_str = g_show_depth ? "DEPTH" : "AMPLITUDE";
        cv::putText(display, "Mode: " + mode_str, cv::Point(10, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

        // Cursor info
        int cx = g_cursor_x % WIDTH;
        int cy = std::min(g_cursor_y, HEIGHT - 1);

        y += 20;
        std::ostringstream cursor_ss;
        cursor_ss << "Cursor: (" << cx << ", " << cy << ")";
        cv::putText(display, cursor_ss.str(), cv::Point(10, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        // Depth values at cursor
        y += 25;
        uint16_t sdk_val = 0, custom_val = 0;
        if (!sdk_depth_local.empty() && cy < sdk_depth_local.rows && cx < sdk_depth_local.cols) {
            sdk_val = sdk_depth_local.at<uint16_t>(cy, cx);
        }
        if (!custom_depth_mat.empty() && cy < custom_depth_mat.rows && cx < custom_depth_mat.cols) {
            custom_val = custom_depth_mat.at<uint16_t>(cy, cx);
        }

        std::ostringstream sdk_ss, custom_ss, diff_ss;
        sdk_ss << "SDK: " << sdk_val << " mm";
        custom_ss << "Custom: " << custom_val << " mm";
        int diff = std::abs(static_cast<int>(sdk_val) - static_cast<int>(custom_val));
        diff_ss << "Diff: " << diff << " mm";

        cv::putText(display, sdk_ss.str(), cv::Point(10, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        y += 20;
        cv::putText(display, custom_ss.str(), cv::Point(10, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 200, 255), 1);
        y += 20;
        cv::Scalar diff_color = diff < 10 ? cv::Scalar(0, 255, 0) :
                               diff < 50 ? cv::Scalar(0, 165, 255) :
                               cv::Scalar(0, 0, 255);
        cv::putText(display, diff_ss.str(), cv::Point(10, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, diff_color, 1);

        // FPS
        y += 25;
        std::ostringstream fps_ss;
        fps_ss << "FPS: " << std::fixed << std::setprecision(1) << g_fps;
        cv::putText(display, fps_ss.str(), cv::Point(10, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

        // Filter panel
        cv::rectangle(display, cv::Point(display.cols - 280, 5),
                     cv::Point(display.cols - 5, 180), cv::Scalar(0, 0, 0), -1);

        cv::putText(display, "SDK Filters (1-6):", cv::Point(display.cols - 275, 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);

        y = 45;
        for (size_t i = 0; i < g_filters.size(); i++) {
            auto& f = g_filters[i];
            std::string short_name = f.name;
            size_t pos = short_name.find("_filter");
            if (pos != std::string::npos) short_name = short_name.substr(0, pos);
            pos = short_name.find("_remove");
            if (pos != std::string::npos) short_name = short_name.substr(0, pos);

            std::ostringstream ss;
            ss << "[" << f.key << "] " << short_name << ": " << (f.enabled ? "ON" : "OFF");

            cv::Scalar color = f.enabled ? cv::Scalar(0, 255, 0) : cv::Scalar(100, 100, 100);
            cv::putText(display, ss.str(), cv::Point(display.cols - 275, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            y += 20;
        }

        // Gradient correction status
        y += 5;
        std::string gc_str = extractor.GetGradientCorrection() ? "ON" : "OFF";
        cv::putText(display, "[G] Gradient Corr: " + gc_str,
                   cv::Point(display.cols - 275, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4,
                   extractor.GetGradientCorrection() ? cv::Scalar(0, 255, 0) : cv::Scalar(100, 100, 100), 1);

        // Draw crosshair
        int draw_x = std::min(g_cursor_x, display.cols - 1);
        int draw_y = std::min(g_cursor_y, display.rows - 1);
        cv::line(display, cv::Point(draw_x - 20, draw_y), cv::Point(draw_x + 20, draw_y),
                cv::Scalar(255, 255, 255), 1);
        cv::line(display, cv::Point(draw_x, draw_y - 20), cv::Point(draw_x, draw_y + 20),
                cv::Scalar(255, 255, 255), 1);

        // Controls hint
        cv::putText(display, "D:mode C:color G:gradient S:save 1-6:filters Q:quit",
                   cv::Point(10, display.rows - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

        update_fps();
        g_frame_count++;

        cv::imshow("SDK vs Custom", display);

        int key = cv::waitKey(30) & 0xFF;
        if (key == 'q' || key == 27) {
            g_running = false;
        } else if (key == 'd') {
            g_show_depth = !g_show_depth;
            std::cout << "Mode: " << (g_show_depth ? "Depth" : "Amplitude") << std::endl;
        } else if (key == 'c') {
            static const int colormaps[] = {
                cv::COLORMAP_JET, cv::COLORMAP_TURBO,
                cv::COLORMAP_VIRIDIS, cv::COLORMAP_INFERNO
            };
            static int cm_idx = 0;
            cm_idx = (cm_idx + 1) % 4;
            g_colormap = colormaps[cm_idx];
        } else if (key == 'g') {
            extractor.SetGradientCorrection(!extractor.GetGradientCorrection());
            std::cout << "Gradient correction: "
                      << (extractor.GetGradientCorrection() ? "ON" : "OFF") << std::endl;
        } else if (key == 's') {
            // cv::imwrite requires opencv_imgcodecs which has missing dependencies
            std::cout << "Save disabled (use screenshot tool)" << std::endl;
        } else if (key >= '1' && key <= '6') {
            toggle_filter(key - '1');
        }
    }

    // Cleanup
    std::cout << "\n[Stop] Stopping camera..." << std::endl;
    g_camera->stop();

    std::cout << "[Cleanup] Releasing camera..." << std::endl;
    ms::destroy_camera(g_camera);

    cv::destroyAllWindows();

    std::cout << "[Done] Total frames: " << g_frame_count << std::endl;
    return 0;
}

/**
 * cubeeye_standalone_capture.cpp - SDK-Free CubeEye I200D Capture
 *
 * Captures raw frames directly from CubeEye sensor WITHOUT using the SDK.
 * This implements the reverse-engineered initialization sequence.
 *
 * Key discovery: The SDK sends a UVC XU command to enable streaming:
 *   Selector 2: 01 02 94 00 01 → Enable streaming
 *   Selector 2: 01 02 94 00 00 → Disable streaming
 *
 * Build:
 *   cmake .. && make cubeeye_standalone_capture
 *
 * Usage:
 *   ./cubeeye_standalone_capture [device] [num_frames]
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <cstdint>
#include <vector>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

// Frame format constants (verified from SDK analysis)
constexpr int FRAME_WIDTH = 1600;
constexpr int FRAME_HEIGHT = 241;
constexpr int FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT * 2;  // YUYV = 2 bytes/pixel

// UVC Extension Unit constants (from reverse engineering)
constexpr int XU_UNIT_ID = 3;
constexpr int XU_SELECTOR_INFO = 1;
constexpr int XU_SELECTOR_REG = 2;
constexpr int XU_SELECTOR_CAL_WRITE = 3;
constexpr int XU_SELECTOR_CAL_READ = 4;
constexpr int XU_SELECTOR_STATUS = 5;

// UVC XU query types
#define UVC_SET_CUR  0x01
#define UVC_GET_CUR  0x81
#define UVC_GET_LEN  0x85

// Buffer structure
struct Buffer {
    void* start;
    size_t length;
};

// Helper for ioctl with retry
int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

// Query UVC XU control
int query_xu(int fd, int selector, int query, uint8_t* data, int size) {
    struct uvc_xu_control_query xu = {
        .unit = XU_UNIT_ID,
        .selector = static_cast<__u8>(selector),
        .query = static_cast<__u8>(query),
        .size = static_cast<__u16>(size),
        .data = data
    };

    return ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
}

// Get control length
int get_xu_length(int fd, int selector) {
    uint8_t len_data[2] = {0};
    if (query_xu(fd, selector, UVC_GET_LEN, len_data, 2) < 0) {
        return -1;
    }
    return len_data[0] | (len_data[1] << 8);
}

// Print hex data
void print_hex(const char* label, const uint8_t* data, int len) {
    std::cout << label << ": ";
    for (int i = 0; i < len && i < 20; i++) {
        std::cout << std::hex << std::setfill('0') << std::setw(2)
                  << static_cast<int>(data[i]) << " ";
    }
    if (len > 20) std::cout << "...";
    std::cout << std::dec << std::endl;
}

// Query device info using Selector 1
bool query_device_info(int fd) {
    std::cout << "\n=== Device Info ===" << std::endl;

    int len = get_xu_length(fd, XU_SELECTOR_INFO);
    if (len <= 0) {
        std::cerr << "Failed to get info control length" << std::endl;
        return false;
    }

    std::vector<uint8_t> data(len);

    // Query serial number (command 0x01)
    data[0] = 0x02;  // Query command
    data[3] = 0x01;  // Serial number
    if (query_xu(fd, XU_SELECTOR_INFO, UVC_SET_CUR, data.data(), len) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (query_xu(fd, XU_SELECTOR_INFO, UVC_GET_CUR, data.data(), len) == 0) {
            char serial[17] = {0};
            memcpy(serial, &data[4], 16);
            std::cout << "Serial: " << serial << std::endl;
        }
    }

    // Query firmware version (command 0x03)
    memset(data.data(), 0, len);
    data[0] = 0x02;
    data[3] = 0x03;
    if (query_xu(fd, XU_SELECTOR_INFO, UVC_SET_CUR, data.data(), len) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (query_xu(fd, XU_SELECTOR_INFO, UVC_GET_CUR, data.data(), len) == 0) {
            std::cout << "FW Version: 0x" << std::hex << static_cast<int>(data[4])
                      << std::dec << std::endl;
        }
    }

    return true;
}

// Send XU command to Selector 5 (status)
bool send_status_command(int fd, uint8_t value) {
    int len = get_xu_length(fd, XU_SELECTOR_STATUS);
    if (len <= 0) len = 8;  // Default for selector 5

    std::vector<uint8_t> cmd(len, 0);
    cmd[0] = 0x01;
    cmd[1] = 0x80;
    cmd[2] = 0x00;
    cmd[3] = 0x16;
    cmd[4] = value;

    return query_xu(fd, XU_SELECTOR_STATUS, UVC_SET_CUR, cmd.data(), len) == 0;
}

// Enable sensor/illuminator via XU Selector 2
bool enable_sensor(int fd) {
    int len = get_xu_length(fd, XU_SELECTOR_REG);
    if (len <= 0) len = 20;

    std::vector<uint8_t> cmd(len, 0);

    // Key command discovered from SDK trace:
    // Selector 2: 01 01 00 d0 → Write 0xD0 to register 0x0001
    // This likely enables the sensor/illuminator
    cmd[0] = 0x01;  // Write command
    cmd[1] = 0x01;  // Register address low
    cmd[2] = 0x00;  // Register address high
    cmd[3] = 0xd0;  // Value

    std::cout << "Enabling sensor (reg 0x0001 = 0xD0)... ";

    if (query_xu(fd, XU_SELECTOR_REG, UVC_SET_CUR, cmd.data(), len) < 0) {
        std::cerr << "FAILED: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "OK" << std::endl;
    return true;
}

// Enable sensor streaming via XU Selector 2
bool enable_streaming(int fd, bool enable) {
    int len = get_xu_length(fd, XU_SELECTOR_REG);
    if (len <= 0) len = 20;

    std::vector<uint8_t> cmd(len, 0);

    // Stream enable command discovered from SDK analysis:
    // Selector 2: 01 02 94 00 01 → Enable streaming
    // Selector 2: 01 02 94 00 00 → Disable streaming
    cmd[0] = 0x01;  // Write command
    cmd[1] = 0x02;  // Register bank/type
    cmd[2] = 0x94;  // Register address low
    cmd[3] = 0x00;  // Register address high
    cmd[4] = enable ? 0x01 : 0x00;  // Value: 1=enable, 0=disable

    std::cout << (enable ? "Enabling" : "Disabling") << " streaming... ";

    if (query_xu(fd, XU_SELECTOR_REG, UVC_SET_CUR, cmd.data(), len) < 0) {
        std::cerr << "FAILED: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "OK" << std::endl;
    return true;
}

// Disable illuminator via XU Selector 5 (discovered from SDK shutdown trace)
// SDK shutdown sequence: 01 80 00 16 00 → Turn off illuminator
bool disable_illuminator(int fd) {
    std::cout << "Disabling illuminator... ";
    if (!send_status_command(fd, 0x00)) {
        std::cerr << "FAILED: " << strerror(errno) << std::endl;
        return false;
    }
    std::cout << "OK" << std::endl;
    return true;
}

// Full sensor shutdown (call before close())
// Discovered shutdown sequence:
//   1. Selector 2: 01 02 94 00 00 → Disable streaming
//   2. Selector 5: 01 80 00 16 00 → Turn off illuminator
void shutdown_sensor(int fd) {
    std::cout << "\n=== Sensor Shutdown ===" << std::endl;
    enable_streaming(fd, false);
    disable_illuminator(fd);
}

// Main capture function
int main(int argc, char* argv[]) {
    const char* device = "/dev/video0";
    int num_frames = 10;
    const char* output_dir = "standalone_capture";

    if (argc > 1) device = argv[1];
    if (argc > 2) num_frames = std::atoi(argv[2]);
    if (argc > 3) output_dir = argv[3];

    std::cout << "======================================" << std::endl;
    std::cout << "CubeEye SDK-Free Standalone Capture" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Device: " << device << std::endl;
    std::cout << "Frames: " << num_frames << std::endl;
    std::cout << "Output: " << output_dir << std::endl;

    // Create output directory
    std::string mkdir_cmd = std::string("mkdir -p ") + output_dir;
    system(mkdir_cmd.c_str());

    // Open device
    int fd = open(device, O_RDWR);
    if (fd == -1) {
        std::cerr << "Error: Cannot open " << device << ": " << strerror(errno) << std::endl;
        return 1;
    }

    // Query capabilities
    struct v4l2_capability cap;
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        std::cerr << "Error: VIDIOC_QUERYCAP failed" << std::endl;
        close(fd);
        return 1;
    }

    std::cout << "\nDriver: " << cap.driver << std::endl;
    std::cout << "Card: " << cap.card << std::endl;

    // Verify it's a CubeEye sensor
    if (strstr(reinterpret_cast<char*>(cap.card), "Cube Eye") == nullptr &&
        strstr(reinterpret_cast<char*>(cap.card), "CubeEye") == nullptr) {
        std::cerr << "Warning: Device may not be a CubeEye sensor" << std::endl;
    }

    // Query device info
    query_device_info(fd);

    // Full sensor initialization sequence (discovered from SDK trace)
    std::cout << "\n=== Sensor Initialization ===" << std::endl;

    // Step 1: Enable sensor/illuminator (write 0xD0 to register 0x0001)
    if (!enable_sensor(fd)) {
        std::cerr << "Warning: Failed to enable sensor" << std::endl;
    }

    // Step 2: Send status commands (Selector 5)
    std::cout << "Sending status commands... ";
    send_status_command(fd, 0x00);
    send_status_command(fd, 0x01);
    std::cout << "OK" << std::endl;

    // Step 3: Enable streaming
    if (!enable_streaming(fd, true)) {
        std::cerr << "Warning: Failed to enable streaming via XU, trying anyway..." << std::endl;
    }

    // Give sensor time to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Set format
    std::cout << "\n=== V4L2 Configuration ===" << std::endl;
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = FRAME_WIDTH;
    fmt.fmt.pix.height = FRAME_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        std::cerr << "Error: VIDIOC_S_FMT failed: " << strerror(errno) << std::endl;
        shutdown_sensor(fd);
        close(fd);
        return 1;
    }

    std::cout << "Format: " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << std::endl;
    std::cout << "Bytes/line: " << fmt.fmt.pix.bytesperline << std::endl;
    std::cout << "Image size: " << fmt.fmt.pix.sizeimage << std::endl;

    // Request buffers
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        std::cerr << "Error: VIDIOC_REQBUFS failed: " << strerror(errno) << std::endl;
        shutdown_sensor(fd);
        close(fd);
        return 1;
    }

    std::cout << "Allocated " << req.count << " buffers" << std::endl;

    // Map buffers
    std::vector<Buffer> buffers(req.count);
    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            std::cerr << "Error: VIDIOC_QUERYBUF failed" << std::endl;
            shutdown_sensor(fd);
            close(fd);
            return 1;
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, buf.m.offset);

        if (buffers[i].start == MAP_FAILED) {
            std::cerr << "Error: mmap failed" << std::endl;
            shutdown_sensor(fd);
            close(fd);
            return 1;
        }
    }

    // Queue buffers
    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            std::cerr << "Error: VIDIOC_QBUF failed" << std::endl;
            shutdown_sensor(fd);
            close(fd);
            return 1;
        }
    }

    // Start streaming
    std::cout << "\n=== Capture ===" << std::endl;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        std::cerr << "Error: VIDIOC_STREAMON failed: " << strerror(errno) << std::endl;
        shutdown_sensor(fd);
        close(fd);
        return 1;
    }

    std::cout << "Capturing " << num_frames << " frames..." << std::endl;

    // Capture loop
    int valid_frames = 0;
    int empty_frames = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < num_frames + 10; frame++) {  // Extra frames to skip warmup
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Wait for frame with timeout
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        struct timeval tv;
        tv.tv_sec = 5;
        tv.tv_usec = 0;

        int r = select(fd + 1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            std::cerr << "Error: select failed" << std::endl;
            break;
        } else if (r == 0) {
            std::cerr << "Error: Timeout waiting for frame" << std::endl;
            break;
        }

        if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            std::cerr << "Error: VIDIOC_DQBUF failed: " << strerror(errno) << std::endl;
            break;
        }

        const uint16_t* frame_data = static_cast<const uint16_t*>(buffers[buf.index].start);

        // Check if frame has valid data (not all zeros)
        bool has_data = false;
        for (int i = 0; i < 1000 && !has_data; i++) {
            if (frame_data[i] != 0) has_data = true;
        }

        if (buf.bytesused > 0 && has_data) {
            valid_frames++;

            // Save frame
            if (valid_frames <= num_frames) {
                std::string filename = std::string(output_dir) + "/frame_" +
                    std::to_string(valid_frames - 1) + ".bin";
                std::ofstream file(filename, std::ios::binary);
                if (file) {
                    file.write(static_cast<const char*>(buffers[buf.index].start), buf.bytesused);
                }

                // Print progress
                if (valid_frames == 1 || valid_frames % 10 == 0) {
                    int center_idx = 120 * FRAME_WIDTH + 160 * 5;
                    std::cout << "[Frame " << valid_frames << "] Size: " << buf.bytesused
                              << " bytes, Center pixel: "
                              << frame_data[center_idx + 2] << std::endl;
                }
            }
        } else {
            empty_frames++;
            if (empty_frames <= 5) {
                std::cout << "[Frame " << frame << "] Empty/zero (warmup)" << std::endl;
            }
        }

        // Re-queue buffer
        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            std::cerr << "Error: VIDIOC_QBUF failed" << std::endl;
            break;
        }

        if (valid_frames >= num_frames) break;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(end_time - start_time).count();

    // Stop streaming
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
        std::cerr << "Warning: VIDIOC_STREAMOFF failed" << std::endl;
    }

    // Proper sensor shutdown (streaming off + illuminator off)
    shutdown_sensor(fd);

    // Cleanup
    for (auto& buffer : buffers) {
        munmap(buffer.start, buffer.length);
    }

    close(fd);

    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Valid frames: " << valid_frames << std::endl;
    std::cout << "Empty frames: " << empty_frames << std::endl;
    std::cout << "Elapsed: " << std::fixed << std::setprecision(2) << elapsed << " s" << std::endl;
    std::cout << "FPS: " << std::fixed << std::setprecision(1) << (valid_frames / elapsed) << std::endl;

    return valid_frames > 0 ? 0 : 1;
}

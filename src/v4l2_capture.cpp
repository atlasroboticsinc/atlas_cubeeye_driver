/**
 * v4l2_capture.cpp - Raw V4L2 Frame Capture
 *
 * Captures raw frames directly from V4L2 device bypassing SDK.
 * Frame format: 1600x241 uint16 (YUYV format, 2 bytes per pixel)
 *
 * Layout (from PLAN.md):
 *   Row 0: Header (32 bytes meaningful)
 *   Rows 1-240: Pixel data
 *   1600 = 5 × 320 (5 sub-pixels per spatial position)
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <cstdint>
#include <vector>
#include <chrono>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

constexpr int FRAME_WIDTH = 1600;
constexpr int FRAME_HEIGHT = 241;
constexpr int BYTES_PER_PIXEL = 2;  // YUYV = 2 bytes per pixel (uint16)
constexpr int FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL;

// Sub-pixel layout
constexpr int SPATIAL_WIDTH = 320;   // 1600 / 5 = 320 spatial pixels
constexpr int SUB_PIXELS = 5;        // 5 sub-pixels per spatial position

struct Buffer {
    void* start;
    size_t length;
};

int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

void analyze_frame(const uint16_t* data, int frame_num) {
    // Analyze header (row 0)
    std::cout << "\n=== Frame " << frame_num << " Header Analysis ===" << std::endl;
    std::cout << "First 16 header values: ";
    for (int i = 0; i < 16; i++) {
        std::cout << data[i] << " ";
    }
    std::cout << std::endl;

    // Analyze sub-pixel means for row 120 (middle of frame)
    std::cout << "\n=== Sub-pixel Analysis (row 120) ===" << std::endl;

    int row = 120;
    double sub_means[SUB_PIXELS] = {0};
    double sub_mins[SUB_PIXELS];
    double sub_maxs[SUB_PIXELS];

    for (int s = 0; s < SUB_PIXELS; s++) {
        sub_mins[s] = 65535;
        sub_maxs[s] = 0;
    }

    for (int x = 0; x < SPATIAL_WIDTH; x++) {
        int base_idx = row * FRAME_WIDTH + x * SUB_PIXELS;
        for (int s = 0; s < SUB_PIXELS; s++) {
            uint16_t val = data[base_idx + s];
            sub_means[s] += val;
            if (val < sub_mins[s]) sub_mins[s] = val;
            if (val > sub_maxs[s]) sub_maxs[s] = val;
        }
    }

    for (int s = 0; s < SUB_PIXELS; s++) {
        sub_means[s] /= SPATIAL_WIDTH;
        std::cout << "  Sub[" << s << "]: mean=" << std::fixed << std::setprecision(1)
                  << sub_means[s] << ", min=" << sub_mins[s]
                  << ", max=" << sub_maxs[s] << std::endl;
    }

    // Center pixel analysis
    int center_x = SPATIAL_WIDTH / 2;
    int center_row = FRAME_HEIGHT / 2;
    int center_base = center_row * FRAME_WIDTH + center_x * SUB_PIXELS;

    std::cout << "\n=== Center Pixel (" << center_x << ", " << center_row << ") ===" << std::endl;
    for (int s = 0; s < SUB_PIXELS; s++) {
        std::cout << "  Sub[" << s << "]: " << data[center_base + s] << std::endl;
    }
}

int main(int argc, char* argv[]) {
    const char* device = "/dev/video0";
    int num_frames = 10;

    if (argc > 1) device = argv[1];
    if (argc > 2) num_frames = std::atoi(argv[2]);

    std::cout << "V4L2 Raw Capture Tool" << std::endl;
    std::cout << "=====================" << std::endl;
    std::cout << "Device: " << device << std::endl;
    std::cout << "Target frames: " << num_frames << std::endl;
    std::cout << "Expected format: " << FRAME_WIDTH << "x" << FRAME_HEIGHT
              << " uint16 (" << FRAME_SIZE << " bytes)" << std::endl;

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

    // Set format
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = FRAME_WIDTH;
    fmt.fmt.pix.height = FRAME_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        std::cerr << "Error: VIDIOC_S_FMT failed: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }

    std::cout << "Format set: " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height
              << " (bytes per line: " << fmt.fmt.pix.bytesperline
              << ", image size: " << fmt.fmt.pix.sizeimage << ")" << std::endl;

    // Request buffers
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        std::cerr << "Error: VIDIOC_REQBUFS failed: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }

    if (req.count < 2) {
        std::cerr << "Error: Insufficient buffer memory" << std::endl;
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
            close(fd);
            return 1;
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, buf.m.offset);

        if (buffers[i].start == MAP_FAILED) {
            std::cerr << "Error: mmap failed" << std::endl;
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
            close(fd);
            return 1;
        }
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        std::cerr << "Error: VIDIOC_STREAMON failed: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }

    std::cout << "\nCapturing " << num_frames << " frames..." << std::endl;

    // Capture frames
    for (int frame = 0; frame < num_frames; frame++) {
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

        // Process frame
        const uint16_t* frame_data = static_cast<const uint16_t*>(buffers[buf.index].start);

        if (frame == 0) {
            // Detailed analysis of first frame
            analyze_frame(frame_data, frame);

            // Save first frame
            std::string filename = "data/v4l2_frame_0.raw";
            std::ofstream file(filename, std::ios::binary);
            if (file) {
                file.write(static_cast<const char*>(buffers[buf.index].start), buf.bytesused);
                std::cout << "\n[Saved] Raw frame to " << filename
                          << " (" << buf.bytesused << " bytes)" << std::endl;
            }
        } else if (frame % 10 == 0) {
            // Quick stats for every 10th frame
            int center_x = SPATIAL_WIDTH / 2;
            int center_row = FRAME_HEIGHT / 2;
            int center_base = center_row * FRAME_WIDTH + center_x * SUB_PIXELS;
            std::cout << "[Frame " << frame << "] Center sub-pixels: ";
            for (int s = 0; s < SUB_PIXELS; s++) {
                std::cout << frame_data[center_base + s] << " ";
            }
            std::cout << std::endl;
        }

        // Re-queue buffer
        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            std::cerr << "Error: VIDIOC_QBUF failed" << std::endl;
            break;
        }
    }

    // Stop streaming
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
        std::cerr << "Warning: VIDIOC_STREAMOFF failed" << std::endl;
    }

    // Cleanup
    for (auto& buffer : buffers) {
        munmap(buffer.start, buffer.length);
    }

    close(fd);
    std::cout << "\n[Done]" << std::endl;

    return 0;
}

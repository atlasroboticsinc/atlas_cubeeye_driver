/**
 * cubeeye_depth_cuda.h - CUDA-accelerated CubeEye I200D Depth Extraction
 *
 * GPU-accelerated depth extraction for CubeEye I200D ToF sensor.
 * Achieves ~10x speedup over CPU implementation on Jetson Orin.
 *
 * Memory requirements:
 *   - Device: ~1.5MB (input + output + LUT)
 *   - Pinned host: ~1.4MB for zero-copy transfers
 *
 * Performance targets:
 *   - Kernel execution: <0.2ms
 *   - End-to-end (with transfers): <0.5ms
 *   - Throughput: >60 fps sustainable
 */

#ifndef CUBEEYE_DEPTH_CUDA_H
#define CUBEEYE_DEPTH_CUDA_H

#include <cstdint>
#include <cstddef>

namespace cubeeye {
namespace cuda {

// Frame dimensions
constexpr int RAW_FRAME_SIZE = 771200;     // Total bytes per V4L2 frame
constexpr int RAW_WIDTH = 1600;             // Bytes per row (amplitude + depth)
constexpr int RAW_HEIGHT = 241;             // Rows (1 header + 240 data)
constexpr int DATA_ROWS = 240;              // Actual data rows
constexpr int DEPTH_BYTES_PER_ROW = 1600;   // Depth section size
constexpr int OUTPUT_WIDTH = 640;           // Output pixels per row
constexpr int OUTPUT_HEIGHT = 480;          // Output rows (2x interpolated)
constexpr int MAX_DEPTH_MM = 7500;          // Maximum valid depth

/**
 * CUDA Depth Extractor
 *
 * Manages GPU memory and provides depth extraction from raw V4L2 frames.
 * Thread-safe for concurrent kernel launches on different streams.
 */
class CudaDepthExtractor {
public:
    /**
     * Constructor - allocates GPU memory and initializes gradient LUT
     *
     * @param apply_gradient_correction Enable polynomial depth correction
     * @param use_pinned_memory Use pinned host memory for faster transfers
     * @throws std::runtime_error if CUDA initialization fails
     */
    explicit CudaDepthExtractor(bool apply_gradient_correction = true,
                                 bool use_pinned_memory = true);

    /**
     * Destructor - frees GPU memory
     */
    ~CudaDepthExtractor();

    // Non-copyable
    CudaDepthExtractor(const CudaDepthExtractor&) = delete;
    CudaDepthExtractor& operator=(const CudaDepthExtractor&) = delete;

    // Movable
    CudaDepthExtractor(CudaDepthExtractor&& other) noexcept;
    CudaDepthExtractor& operator=(CudaDepthExtractor&& other) noexcept;

    /**
     * Extract depth from raw V4L2 frame (synchronous)
     *
     * @param raw_frame   Input raw frame data (771,200 bytes)
     * @param frame_size  Size of input (must be RAW_FRAME_SIZE)
     * @param depth_out   Output depth buffer (640x480 uint16, in mm)
     * @param interpolate Apply 2x vertical interpolation (240->480)
     * @return true on success
     */
    bool ExtractDepth(const uint8_t* raw_frame, size_t frame_size,
                      uint16_t* depth_out, bool interpolate = true);

    /**
     * Extract depth asynchronously (non-blocking)
     *
     * @param raw_frame   Input raw frame data
     * @param frame_size  Size of input
     * @param depth_out   Output depth buffer (must remain valid until sync)
     * @param interpolate Apply vertical interpolation
     * @return true if launch succeeded
     */
    bool ExtractDepthAsync(const uint8_t* raw_frame, size_t frame_size,
                           uint16_t* depth_out, bool interpolate = true);

    /**
     * Wait for async operation to complete
     */
    void Synchronize();

    /**
     * Extract both depth and amplitude
     *
     * @param raw_frame     Input raw frame
     * @param frame_size    Size of input
     * @param depth_out     Output depth (640x480 uint16)
     * @param amplitude_out Output amplitude (640x480 uint16)
     * @param interpolate   Apply vertical interpolation
     * @return true on success
     */
    bool ExtractDepthAndAmplitude(const uint8_t* raw_frame, size_t frame_size,
                                   uint16_t* depth_out, uint16_t* amplitude_out,
                                   bool interpolate = true);

    /**
     * Get last kernel execution time in milliseconds
     */
    float GetLastKernelTimeMs() const { return last_kernel_time_ms_; }

    /**
     * Get total processing time (transfers + kernel) in milliseconds
     */
    float GetLastTotalTimeMs() const { return last_total_time_ms_; }

    /**
     * Check if CUDA is available on this system
     */
    static bool IsCudaAvailable();

    /**
     * Get CUDA device properties string
     */
    static const char* GetDeviceInfo();

private:
    void InitializeGradientLUT();
    void AllocateDeviceMemory();
    void FreeDeviceMemory();

    // Configuration
    bool apply_gradient_correction_;
    bool use_pinned_memory_;

    // Device memory
    uint8_t* d_raw_frame_;      // Input frame on GPU
    uint16_t* d_depth_out_;     // Output depth on GPU
    uint16_t* d_amplitude_out_; // Output amplitude on GPU
    int16_t* d_gradient_lut_;   // Gradient correction LUT on GPU

    // Pinned host memory (optional)
    uint8_t* h_pinned_input_;
    uint16_t* h_pinned_output_;

    // CUDA stream for async operations
    void* stream_;  // cudaStream_t

    // Timing
    void* start_event_;  // cudaEvent_t
    void* stop_event_;   // cudaEvent_t
    float last_kernel_time_ms_;
    float last_total_time_ms_;

    // Device info cache
    static char device_info_[256];
    static bool device_info_initialized_;
};

/**
 * Low-level kernel launch (for advanced users)
 *
 * Directly launches the depth extraction kernel without memory management.
 * Caller is responsible for all memory allocation and transfers.
 *
 * Grid:  240 blocks (one per data row)
 * Block: 320 threads (one per 5-byte group)
 *
 * @param d_raw_frame       Device pointer to raw frame
 * @param d_depth_out       Device pointer to output (640x480 or 640x240)
 * @param d_gradient_lut    Device pointer to gradient LUT (7501 int16)
 * @param apply_correction  Whether to apply gradient correction
 * @param interpolate       Whether to 2x vertically interpolate
 * @param stream            CUDA stream (0 for default)
 */
void LaunchDepthExtractionKernel(
    const uint8_t* d_raw_frame,
    uint16_t* d_depth_out,
    const int16_t* d_gradient_lut,
    bool apply_correction,
    bool interpolate,
    void* stream = nullptr);

/**
 * Launch combined depth + amplitude extraction kernel
 */
void LaunchDepthAmplitudeKernel(
    const uint8_t* d_raw_frame,
    uint16_t* d_depth_out,
    uint16_t* d_amplitude_out,
    const int16_t* d_gradient_lut,
    bool apply_correction,
    bool interpolate,
    void* stream = nullptr);

}  // namespace cuda
}  // namespace cubeeye

#endif  // CUBEEYE_DEPTH_CUDA_H

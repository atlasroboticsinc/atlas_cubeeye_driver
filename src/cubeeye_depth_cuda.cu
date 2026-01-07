/**
 * cubeeye_depth_cuda.cu - CUDA Kernel Implementation
 *
 * GPU-accelerated depth extraction for CubeEye I200D ToF sensor.
 *
 * Kernel Configuration:
 *   Grid:   240 blocks (one per raw data row)
 *   Block:  320 threads (one per 5-byte group = 2 pixels)
 *   Shared: 1,600 bytes (depth section of one row)
 *
 * Memory Access Pattern:
 *   - Coalesced global reads (32 threads read 32 consecutive bytes)
 *   - Shared memory for row data (bank conflict free)
 *   - Coalesced global writes (2 pixels per thread, consecutive)
 *
 * Performance Characteristics:
 *   - Compute bound for small depths (polynomial evaluation)
 *   - Memory bound for large scenes (LUT lookups)
 *   - ~150K depth calculations per frame
 *   - Target: <0.2ms on Jetson Orin Nano
 */

#include "cubeeye_depth_cuda.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cstdio>
#include <stdexcept>
#include <cstring>

namespace cubeeye {
namespace cuda {

// Static member initialization
char CudaDepthExtractor::device_info_[256] = {0};
bool CudaDepthExtractor::device_info_initialized_ = false;

// Gradient correction polynomial coefficients (constant memory)
__constant__ double c_gradient_coeffs[14] = {
    -9.46306765e-08,   // x^13
     5.40828695e-06,   // x^12
    -0.000133821166,   // x^11
     0.00189463573,    // x^10
    -0.0170465988,     // x^9
     0.102187397,      // x^8
    -0.415584587,      // x^7
     1.14433615,       // x^6
    -2.09044109,       // x^5
     2.42940077,       // x^4
    -1.66835357,       // x^3
     0.587854516,      // x^2
    -0.076622637,      // x^1
     0.000344344841    // x^0
};

/**
 * Device function: Unpack 5 bytes into 2 depth values
 *
 * Bit layout:
 *   Byte 0: fine0[9:2]
 *   Byte 1: coarse0[9:2]
 *   Byte 2: fine1[9:2]
 *   Byte 3: coarse1[9:2]
 *   Byte 4: [coarse1[1:0]][fine1[1:0]][coarse0[1:0]][fine0[1:0]]
 */
__device__ __forceinline__ void Unpack5Bytes(
    const uint8_t* __restrict__ bytes,
    uint16_t& depth0,
    uint16_t& depth1)
{
    uint8_t b0 = bytes[0];
    uint8_t b1 = bytes[1];
    uint8_t b2 = bytes[2];
    uint8_t b3 = bytes[3];
    uint8_t b4 = bytes[4];

    // Pixel 0: coarse + fine components
    uint16_t coarse0 = ((b4 >> 2) & 0x03) | (static_cast<uint16_t>(b1) << 2);
    uint16_t fine0 = (b4 & 0x03) | (static_cast<uint16_t>(b0) << 2);
    depth0 = (coarse0 << 10) + fine0;

    // Pixel 1: coarse + fine components
    uint16_t coarse1 = (b4 >> 6) | (static_cast<uint16_t>(b3) << 2);
    uint16_t fine1 = ((b4 >> 4) & 0x03) | (static_cast<uint16_t>(b2) << 2);
    depth1 = (coarse1 << 10) + fine1;
}

/**
 * Depth extraction kernel - one block per row, one thread per 5-byte group
 *
 * @param raw_frame      Input frame (771,200 bytes)
 * @param depth_out      Output depth (640x480 or 640x240 uint16)
 * @param gradient_lut   Gradient correction LUT (7501 int16 values)
 * @param apply_correction Whether to apply gradient correction
 * @param interpolate    Whether to duplicate rows for 2x vertical
 */
__global__ void ExtractDepthKernel(
    const uint8_t* __restrict__ raw_frame,
    uint16_t* __restrict__ depth_out,
    const int16_t* __restrict__ gradient_lut,
    bool apply_correction,
    bool interpolate)
{
    // Row index (0-239 for data rows)
    const int row = blockIdx.x;

    // Thread handles one 5-byte group = 2 output pixels
    const int group = threadIdx.x;  // 0-319

    // Shared memory for depth section of current row (1600 bytes)
    __shared__ uint8_t s_depth_data[1600];

    // Calculate row offset in raw frame
    // Header row (row 0 in raw) + data rows start at byte 3200
    // Each row is 3200 bytes: [amplitude 1600][depth 1600]
    const int row_offset = (row + 1) * 3200;  // Skip header
    const int depth_offset = row_offset + 1600;  // Skip amplitude section

    // Cooperative load: each thread loads 5 bytes
    // Total: 320 threads * 5 bytes = 1600 bytes
    const int load_offset = group * 5;
    if (load_offset + 4 < 1600) {
        // Load 5 bytes per thread (coalesced for groups of 32)
        s_depth_data[load_offset + 0] = raw_frame[depth_offset + load_offset + 0];
        s_depth_data[load_offset + 1] = raw_frame[depth_offset + load_offset + 1];
        s_depth_data[load_offset + 2] = raw_frame[depth_offset + load_offset + 2];
        s_depth_data[load_offset + 3] = raw_frame[depth_offset + load_offset + 3];
        s_depth_data[load_offset + 4] = raw_frame[depth_offset + load_offset + 4];
    }

    __syncthreads();

    // Unpack 5 bytes -> 2 depth values
    uint16_t d0, d1;
    Unpack5Bytes(&s_depth_data[group * 5], d0, d1);

    // Apply gradient correction if enabled
    if (apply_correction && gradient_lut != nullptr) {
        // Clamp to valid LUT range
        int idx0 = min(static_cast<int>(d0), MAX_DEPTH_MM);
        int idx1 = min(static_cast<int>(d1), MAX_DEPTH_MM);

        // Apply correction (LUT values are signed corrections in mm)
        int corrected0 = static_cast<int>(d0) + gradient_lut[idx0];
        int corrected1 = static_cast<int>(d1) + gradient_lut[idx1];

        // Clamp to valid range
        d0 = static_cast<uint16_t>(max(0, min(corrected0, MAX_DEPTH_MM)));
        d1 = static_cast<uint16_t>(max(0, min(corrected1, MAX_DEPTH_MM)));
    }

    // Output pixel positions
    const int col = group * 2;

    if (interpolate) {
        // 2x vertical interpolation: write to two consecutive rows
        const int out_row = row * 2;
        const int out_idx0 = out_row * OUTPUT_WIDTH + col;
        const int out_idx1 = (out_row + 1) * OUTPUT_WIDTH + col;

        // Write row N
        depth_out[out_idx0] = d0;
        depth_out[out_idx0 + 1] = d1;

        // Write row N+1 (duplicate)
        depth_out[out_idx1] = d0;
        depth_out[out_idx1 + 1] = d1;
    } else {
        // No interpolation: 640x240 output
        const int out_idx = row * OUTPUT_WIDTH + col;
        depth_out[out_idx] = d0;
        depth_out[out_idx + 1] = d1;
    }
}

/**
 * Combined depth + amplitude extraction kernel
 */
__global__ void ExtractDepthAmplitudeKernel(
    const uint8_t* __restrict__ raw_frame,
    uint16_t* __restrict__ depth_out,
    uint16_t* __restrict__ amplitude_out,
    const int16_t* __restrict__ gradient_lut,
    bool apply_correction,
    bool interpolate)
{
    const int row = blockIdx.x;
    const int group = threadIdx.x;

    // Shared memory for both sections
    __shared__ uint8_t s_row_data[3200];

    // Load entire row (amplitude + depth)
    const int row_offset = (row + 1) * 3200;

    // Cooperative load: 320 threads, 10 bytes each = 3200 bytes
    const int load_offset = group * 10;
    if (load_offset + 9 < 3200) {
        #pragma unroll
        for (int i = 0; i < 10; i++) {
            s_row_data[load_offset + i] = raw_frame[row_offset + load_offset + i];
        }
    }

    __syncthreads();

    // Unpack depth (from second half of row)
    uint16_t d0, d1;
    Unpack5Bytes(&s_row_data[1600 + group * 5], d0, d1);

    // Unpack amplitude (from first half of row)
    uint16_t a0, a1;
    Unpack5Bytes(&s_row_data[group * 5], a0, a1);

    // Apply gradient correction to depth
    if (apply_correction && gradient_lut != nullptr) {
        int idx0 = min(static_cast<int>(d0), MAX_DEPTH_MM);
        int idx1 = min(static_cast<int>(d1), MAX_DEPTH_MM);
        int corrected0 = static_cast<int>(d0) + gradient_lut[idx0];
        int corrected1 = static_cast<int>(d1) + gradient_lut[idx1];
        d0 = static_cast<uint16_t>(max(0, min(corrected0, MAX_DEPTH_MM)));
        d1 = static_cast<uint16_t>(max(0, min(corrected1, MAX_DEPTH_MM)));
    }

    const int col = group * 2;

    if (interpolate) {
        const int out_row = row * 2;
        const int out_idx0 = out_row * OUTPUT_WIDTH + col;
        const int out_idx1 = (out_row + 1) * OUTPUT_WIDTH + col;

        // Depth
        depth_out[out_idx0] = d0;
        depth_out[out_idx0 + 1] = d1;
        depth_out[out_idx1] = d0;
        depth_out[out_idx1 + 1] = d1;

        // Amplitude
        amplitude_out[out_idx0] = a0;
        amplitude_out[out_idx0 + 1] = a1;
        amplitude_out[out_idx1] = a0;
        amplitude_out[out_idx1 + 1] = a1;
    } else {
        const int out_idx = row * OUTPUT_WIDTH + col;
        depth_out[out_idx] = d0;
        depth_out[out_idx + 1] = d1;
        amplitude_out[out_idx] = a0;
        amplitude_out[out_idx + 1] = a1;
    }
}

// Kernel launch wrappers
void LaunchDepthExtractionKernel(
    const uint8_t* d_raw_frame,
    uint16_t* d_depth_out,
    const int16_t* d_gradient_lut,
    bool apply_correction,
    bool interpolate,
    void* stream)
{
    dim3 grid(DATA_ROWS);   // 240 blocks
    dim3 block(320);        // 320 threads per block

    ExtractDepthKernel<<<grid, block, 0, static_cast<cudaStream_t>(stream)>>>(
        d_raw_frame,
        d_depth_out,
        d_gradient_lut,
        apply_correction,
        interpolate
    );
}

void LaunchDepthAmplitudeKernel(
    const uint8_t* d_raw_frame,
    uint16_t* d_depth_out,
    uint16_t* d_amplitude_out,
    const int16_t* d_gradient_lut,
    bool apply_correction,
    bool interpolate,
    void* stream)
{
    dim3 grid(DATA_ROWS);   // 240 blocks
    dim3 block(320);        // 320 threads per block

    ExtractDepthAmplitudeKernel<<<grid, block, 0, static_cast<cudaStream_t>(stream)>>>(
        d_raw_frame,
        d_depth_out,
        d_amplitude_out,
        d_gradient_lut,
        apply_correction,
        interpolate
    );
}

// CudaDepthExtractor implementation

CudaDepthExtractor::CudaDepthExtractor(bool apply_gradient_correction,
                                         bool use_pinned_memory)
    : apply_gradient_correction_(apply_gradient_correction)
    , use_pinned_memory_(use_pinned_memory)
    , d_raw_frame_(nullptr)
    , d_depth_out_(nullptr)
    , d_amplitude_out_(nullptr)
    , d_gradient_lut_(nullptr)
    , h_pinned_input_(nullptr)
    , h_pinned_output_(nullptr)
    , stream_(nullptr)
    , start_event_(nullptr)
    , stop_event_(nullptr)
    , last_kernel_time_ms_(0)
    , last_total_time_ms_(0)
{
    // Check CUDA availability
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    if (err != cudaSuccess || device_count == 0) {
        throw std::runtime_error("No CUDA-capable device found");
    }

    // Create stream and events
    cudaStream_t stream;
    cudaStreamCreate(&stream);
    stream_ = stream;

    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    start_event_ = start;
    stop_event_ = stop;

    AllocateDeviceMemory();
    InitializeGradientLUT();
}

CudaDepthExtractor::~CudaDepthExtractor() {
    FreeDeviceMemory();

    if (stream_) {
        cudaStreamDestroy(static_cast<cudaStream_t>(stream_));
    }
    if (start_event_) {
        cudaEventDestroy(static_cast<cudaEvent_t>(start_event_));
    }
    if (stop_event_) {
        cudaEventDestroy(static_cast<cudaEvent_t>(stop_event_));
    }
}

CudaDepthExtractor::CudaDepthExtractor(CudaDepthExtractor&& other) noexcept
    : apply_gradient_correction_(other.apply_gradient_correction_)
    , use_pinned_memory_(other.use_pinned_memory_)
    , d_raw_frame_(other.d_raw_frame_)
    , d_depth_out_(other.d_depth_out_)
    , d_amplitude_out_(other.d_amplitude_out_)
    , d_gradient_lut_(other.d_gradient_lut_)
    , h_pinned_input_(other.h_pinned_input_)
    , h_pinned_output_(other.h_pinned_output_)
    , stream_(other.stream_)
    , start_event_(other.start_event_)
    , stop_event_(other.stop_event_)
    , last_kernel_time_ms_(other.last_kernel_time_ms_)
    , last_total_time_ms_(other.last_total_time_ms_)
{
    other.d_raw_frame_ = nullptr;
    other.d_depth_out_ = nullptr;
    other.d_amplitude_out_ = nullptr;
    other.d_gradient_lut_ = nullptr;
    other.h_pinned_input_ = nullptr;
    other.h_pinned_output_ = nullptr;
    other.stream_ = nullptr;
    other.start_event_ = nullptr;
    other.stop_event_ = nullptr;
}

CudaDepthExtractor& CudaDepthExtractor::operator=(CudaDepthExtractor&& other) noexcept {
    if (this != &other) {
        FreeDeviceMemory();
        if (stream_) cudaStreamDestroy(static_cast<cudaStream_t>(stream_));
        if (start_event_) cudaEventDestroy(static_cast<cudaEvent_t>(start_event_));
        if (stop_event_) cudaEventDestroy(static_cast<cudaEvent_t>(stop_event_));

        apply_gradient_correction_ = other.apply_gradient_correction_;
        use_pinned_memory_ = other.use_pinned_memory_;
        d_raw_frame_ = other.d_raw_frame_;
        d_depth_out_ = other.d_depth_out_;
        d_amplitude_out_ = other.d_amplitude_out_;
        d_gradient_lut_ = other.d_gradient_lut_;
        h_pinned_input_ = other.h_pinned_input_;
        h_pinned_output_ = other.h_pinned_output_;
        stream_ = other.stream_;
        start_event_ = other.start_event_;
        stop_event_ = other.stop_event_;
        last_kernel_time_ms_ = other.last_kernel_time_ms_;
        last_total_time_ms_ = other.last_total_time_ms_;

        other.d_raw_frame_ = nullptr;
        other.d_depth_out_ = nullptr;
        other.d_amplitude_out_ = nullptr;
        other.d_gradient_lut_ = nullptr;
        other.h_pinned_input_ = nullptr;
        other.h_pinned_output_ = nullptr;
        other.stream_ = nullptr;
        other.start_event_ = nullptr;
        other.stop_event_ = nullptr;
    }
    return *this;
}

void CudaDepthExtractor::AllocateDeviceMemory() {
    cudaError_t err;

    // Device memory for raw frame input
    err = cudaMalloc(&d_raw_frame_, RAW_FRAME_SIZE);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate device memory for raw frame");
    }

    // Device memory for depth output (640x480 uint16)
    size_t depth_size = OUTPUT_WIDTH * OUTPUT_HEIGHT * sizeof(uint16_t);
    err = cudaMalloc(&d_depth_out_, depth_size);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate device memory for depth output");
    }

    // Device memory for amplitude output
    err = cudaMalloc(&d_amplitude_out_, depth_size);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate device memory for amplitude output");
    }

    // Device memory for gradient LUT
    err = cudaMalloc(&d_gradient_lut_, (MAX_DEPTH_MM + 1) * sizeof(int16_t));
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate device memory for gradient LUT");
    }

    // Pinned host memory for faster transfers
    if (use_pinned_memory_) {
        err = cudaMallocHost(&h_pinned_input_, RAW_FRAME_SIZE);
        if (err != cudaSuccess) {
            h_pinned_input_ = nullptr;  // Fall back to regular memory
        }

        err = cudaMallocHost(&h_pinned_output_, depth_size);
        if (err != cudaSuccess) {
            h_pinned_output_ = nullptr;
        }
    }
}

void CudaDepthExtractor::FreeDeviceMemory() {
    if (d_raw_frame_) cudaFree(d_raw_frame_);
    if (d_depth_out_) cudaFree(d_depth_out_);
    if (d_amplitude_out_) cudaFree(d_amplitude_out_);
    if (d_gradient_lut_) cudaFree(d_gradient_lut_);

    if (h_pinned_input_) cudaFreeHost(h_pinned_input_);
    if (h_pinned_output_) cudaFreeHost(h_pinned_output_);

    d_raw_frame_ = nullptr;
    d_depth_out_ = nullptr;
    d_amplitude_out_ = nullptr;
    d_gradient_lut_ = nullptr;
    h_pinned_input_ = nullptr;
    h_pinned_output_ = nullptr;
}

void CudaDepthExtractor::InitializeGradientLUT() {
    // Host-side polynomial coefficients
    const double coeffs[14] = {
        -9.46306765e-08,   // x^13
         5.40828695e-06,   // x^12
        -0.000133821166,   // x^11
         0.00189463573,    // x^10
        -0.0170465988,     // x^9
         0.102187397,      // x^8
        -0.415584587,      // x^7
         1.14433615,       // x^6
        -2.09044109,       // x^5
         2.42940077,       // x^4
        -1.66835357,       // x^3
         0.587854516,      // x^2
        -0.076622637,      // x^1
         0.000344344841    // x^0
    };

    // Build LUT on host
    int16_t* h_lut = new int16_t[MAX_DEPTH_MM + 1];

    for (int d = 0; d <= MAX_DEPTH_MM; d++) {
        double x = d / 1000.0;  // Convert to meters
        double correction = 0.0;

        // Evaluate polynomial using Horner's method
        for (int i = 0; i < 14; i++) {
            correction = correction * x + coeffs[i];
        }

        // Convert correction from meters to mm and store
        h_lut[d] = static_cast<int16_t>(correction * 1000.0);
    }

    // Copy to device
    cudaMemcpy(d_gradient_lut_, h_lut, (MAX_DEPTH_MM + 1) * sizeof(int16_t),
               cudaMemcpyHostToDevice);

    delete[] h_lut;
}

bool CudaDepthExtractor::ExtractDepth(const uint8_t* raw_frame, size_t frame_size,
                                       uint16_t* depth_out, bool interpolate) {
    if (frame_size != RAW_FRAME_SIZE) {
        return false;
    }

    cudaStream_t stream = static_cast<cudaStream_t>(stream_);
    cudaEvent_t start = static_cast<cudaEvent_t>(start_event_);
    cudaEvent_t stop = static_cast<cudaEvent_t>(stop_event_);

    // Record start time
    cudaEventRecord(start, stream);

    // Copy input to device
    const uint8_t* src = raw_frame;
    if (use_pinned_memory_ && h_pinned_input_) {
        memcpy(h_pinned_input_, raw_frame, RAW_FRAME_SIZE);
        src = h_pinned_input_;
    }
    cudaMemcpyAsync(d_raw_frame_, src, RAW_FRAME_SIZE,
                    cudaMemcpyHostToDevice, stream);

    // Launch kernel
    LaunchDepthExtractionKernel(
        d_raw_frame_,
        d_depth_out_,
        apply_gradient_correction_ ? d_gradient_lut_ : nullptr,
        apply_gradient_correction_,
        interpolate,
        stream_
    );

    // Copy output to host
    size_t output_size = OUTPUT_WIDTH *
                         (interpolate ? OUTPUT_HEIGHT : OUTPUT_HEIGHT / 2) *
                         sizeof(uint16_t);

    if (use_pinned_memory_ && h_pinned_output_) {
        cudaMemcpyAsync(h_pinned_output_, d_depth_out_, output_size,
                        cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);
        memcpy(depth_out, h_pinned_output_, output_size);
    } else {
        cudaMemcpyAsync(depth_out, d_depth_out_, output_size,
                        cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);
    }

    // Record stop time and compute duration
    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&last_total_time_ms_, start, stop);

    return cudaGetLastError() == cudaSuccess;
}

bool CudaDepthExtractor::ExtractDepthAsync(const uint8_t* raw_frame, size_t frame_size,
                                            uint16_t* depth_out, bool interpolate) {
    if (frame_size != RAW_FRAME_SIZE) {
        return false;
    }

    cudaStream_t stream = static_cast<cudaStream_t>(stream_);

    // Copy input to device (async)
    cudaMemcpyAsync(d_raw_frame_, raw_frame, RAW_FRAME_SIZE,
                    cudaMemcpyHostToDevice, stream);

    // Launch kernel
    LaunchDepthExtractionKernel(
        d_raw_frame_,
        d_depth_out_,
        apply_gradient_correction_ ? d_gradient_lut_ : nullptr,
        apply_gradient_correction_,
        interpolate,
        stream_
    );

    // Copy output to host (async)
    size_t output_size = OUTPUT_WIDTH *
                         (interpolate ? OUTPUT_HEIGHT : OUTPUT_HEIGHT / 2) *
                         sizeof(uint16_t);
    cudaMemcpyAsync(depth_out, d_depth_out_, output_size,
                    cudaMemcpyDeviceToHost, stream);

    return true;
}

void CudaDepthExtractor::Synchronize() {
    cudaStreamSynchronize(static_cast<cudaStream_t>(stream_));
}

bool CudaDepthExtractor::ExtractDepthAndAmplitude(
    const uint8_t* raw_frame, size_t frame_size,
    uint16_t* depth_out, uint16_t* amplitude_out, bool interpolate)
{
    if (frame_size != RAW_FRAME_SIZE) {
        return false;
    }

    cudaStream_t stream = static_cast<cudaStream_t>(stream_);
    cudaEvent_t start = static_cast<cudaEvent_t>(start_event_);
    cudaEvent_t stop = static_cast<cudaEvent_t>(stop_event_);

    cudaEventRecord(start, stream);

    // Copy input
    cudaMemcpyAsync(d_raw_frame_, raw_frame, RAW_FRAME_SIZE,
                    cudaMemcpyHostToDevice, stream);

    // Launch combined kernel
    LaunchDepthAmplitudeKernel(
        d_raw_frame_,
        d_depth_out_,
        d_amplitude_out_,
        apply_gradient_correction_ ? d_gradient_lut_ : nullptr,
        apply_gradient_correction_,
        interpolate,
        stream_
    );

    // Copy outputs
    size_t output_size = OUTPUT_WIDTH *
                         (interpolate ? OUTPUT_HEIGHT : OUTPUT_HEIGHT / 2) *
                         sizeof(uint16_t);
    cudaMemcpyAsync(depth_out, d_depth_out_, output_size,
                    cudaMemcpyDeviceToHost, stream);
    cudaMemcpyAsync(amplitude_out, d_amplitude_out_, output_size,
                    cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&last_total_time_ms_, start, stop);

    return cudaGetLastError() == cudaSuccess;
}

bool CudaDepthExtractor::IsCudaAvailable() {
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    return (err == cudaSuccess && device_count > 0);
}

const char* CudaDepthExtractor::GetDeviceInfo() {
    if (!device_info_initialized_) {
        int device_count = 0;
        cudaError_t err = cudaGetDeviceCount(&device_count);
        if (err != cudaSuccess || device_count == 0) {
            snprintf(device_info_, sizeof(device_info_), "No CUDA device found");
        } else {
            cudaDeviceProp prop;
            cudaGetDeviceProperties(&prop, 0);
            snprintf(device_info_, sizeof(device_info_),
                     "%s (SM %d.%d, %d SMs, %.1f GB)",
                     prop.name,
                     prop.major, prop.minor,
                     prop.multiProcessorCount,
                     prop.totalGlobalMem / (1024.0 * 1024.0 * 1024.0));
        }
        device_info_initialized_ = true;
    }
    return device_info_;
}

}  // namespace cuda
}  // namespace cubeeye

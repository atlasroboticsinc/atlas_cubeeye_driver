cmake_minimum_required(VERSION 3.16)
project(cubeeye_nano_driver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CubeEye SDK paths
set(CUBEEYE_SDK_PATH "$ENV{HOME}/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0")
set(CUBEEYE_INCLUDE_DIR "${CUBEEYE_SDK_PATH}/include")
set(CUBEEYE_LIB_DIR "${CUBEEYE_SDK_PATH}/lib")
set(CUBEEYE_OPENCV_LIB_DIR "${CUBEEYE_SDK_PATH}/thirdparty/libopencv/lib")
set(CUBEEYE_FFMPEG_LIB_DIR "${CUBEEYE_SDK_PATH}/thirdparty/libffmpeg/lib")
set(CUBEEYE_LIVE555_LIB_DIR "${CUBEEYE_SDK_PATH}/thirdparty/liblive555/lib/Release")

# Optional CUDA support (for Phase 4)
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    message(STATUS "CUDA found: ${CUDAToolkit_VERSION}")
else()
    message(STATUS "CUDA not found - CRT kernel will not be built")
endif()

# SDK Capture Tool (uses CubeEye SDK as ground truth)
add_executable(sdk_capture
    src/sdk_capture.cpp
)

target_include_directories(sdk_capture PRIVATE
    ${CUBEEYE_INCLUDE_DIR}
    include
)

target_link_directories(sdk_capture PRIVATE
    ${CUBEEYE_LIB_DIR}
    ${CUBEEYE_OPENCV_LIB_DIR}
    ${CUBEEYE_FFMPEG_LIB_DIR}
    ${CUBEEYE_LIVE555_LIB_DIR}
)

target_link_libraries(sdk_capture PRIVATE
    CubeEye
    Live555ClientSource
    liveMedia
    BasicUsageEnvironment
    UsageEnvironment
    groupsock
    avcodec
    avutil
    swscale
    pthread
)

# Set RPATH for SDK capture
set_target_properties(sdk_capture PROPERTIES
    BUILD_RPATH "${CUBEEYE_LIB_DIR};${CUBEEYE_OPENCV_LIB_DIR};${CUBEEYE_FFMPEG_LIB_DIR};${CUBEEYE_LIVE555_LIB_DIR}"
    INSTALL_RPATH "${CUBEEYE_LIB_DIR};${CUBEEYE_OPENCV_LIB_DIR};${CUBEEYE_FFMPEG_LIB_DIR};${CUBEEYE_LIVE555_LIB_DIR}"
)

# V4L2 Raw Capture Tool
add_executable(v4l2_capture
    src/v4l2_capture.cpp
)

target_include_directories(v4l2_capture PRIVATE
    include
)

target_link_libraries(v4l2_capture PRIVATE
    pthread
)

# Benchmark Capture Tool (SDK + Raw via hook)
add_executable(benchmark_capture
    src/benchmark_capture.cpp
)

target_include_directories(benchmark_capture PRIVATE
    ${CUBEEYE_INCLUDE_DIR}
    include
)

target_link_directories(benchmark_capture PRIVATE
    ${CUBEEYE_LIB_DIR}
    ${CUBEEYE_OPENCV_LIB_DIR}
    ${CUBEEYE_FFMPEG_LIB_DIR}
    ${CUBEEYE_LIVE555_LIB_DIR}
)

target_link_libraries(benchmark_capture PRIVATE
    CubeEye
    Live555ClientSource
    liveMedia
    BasicUsageEnvironment
    UsageEnvironment
    groupsock
    avcodec
    avutil
    swscale
    pthread
)

set_target_properties(benchmark_capture PROPERTIES
    BUILD_RPATH "${CUBEEYE_LIB_DIR};${CUBEEYE_OPENCV_LIB_DIR};${CUBEEYE_FFMPEG_LIB_DIR};${CUBEEYE_LIVE555_LIB_DIR}"
    INSTALL_RPATH "${CUBEEYE_LIB_DIR};${CUBEEYE_OPENCV_LIB_DIR};${CUBEEYE_FFMPEG_LIB_DIR};${CUBEEYE_LIVE555_LIB_DIR}"
)

# Raw Benchmark Capture Tool (SDK with filters disabled)
add_executable(benchmark_raw_capture
    src/benchmark_raw_capture.cpp
)

target_include_directories(benchmark_raw_capture PRIVATE
    ${CUBEEYE_INCLUDE_DIR}
    include
)

target_link_directories(benchmark_raw_capture PRIVATE
    ${CUBEEYE_LIB_DIR}
    ${CUBEEYE_OPENCV_LIB_DIR}
    ${CUBEEYE_FFMPEG_LIB_DIR}
    ${CUBEEYE_LIVE555_LIB_DIR}
)

target_link_libraries(benchmark_raw_capture PRIVATE
    CubeEye
    Live555ClientSource
    liveMedia
    BasicUsageEnvironment
    UsageEnvironment
    groupsock
    avcodec
    avutil
    swscale
    pthread
)

set_target_properties(benchmark_raw_capture PROPERTIES
    BUILD_RPATH "${CUBEEYE_LIB_DIR};${CUBEEYE_OPENCV_LIB_DIR};${CUBEEYE_FFMPEG_LIB_DIR};${CUBEEYE_LIVE555_LIB_DIR}"
    INSTALL_RPATH "${CUBEEYE_LIB_DIR};${CUBEEYE_OPENCV_LIB_DIR};${CUBEEYE_FFMPEG_LIB_DIR};${CUBEEYE_LIVE555_LIB_DIR}"
)

# Frame Analyzer Tool
add_executable(frame_analyzer
    src/frame_analyzer.cpp
)

target_include_directories(frame_analyzer PRIVATE
    include
)

# V4L2 Hook Library (LD_PRELOAD to intercept raw frames)
add_library(v4l2_hook SHARED
    src/v4l2_hook.c
)

target_link_libraries(v4l2_hook PRIVATE
    dl
    pthread
)

set_target_properties(v4l2_hook PROPERTIES
    PREFIX "lib"
    OUTPUT_NAME "v4l2_hook"
)

# Depth Processor Library
find_package(OpenMP QUIET)

add_library(depth_processor STATIC
    src/depth_processor.cpp
)

target_include_directories(depth_processor PUBLIC
    src
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(depth_processor PUBLIC OpenMP::OpenMP_CXX)
    message(STATUS "OpenMP found - depth processor will use parallel processing")
endif()

# Depth Processor Test
add_executable(test_depth_processor
    src/test_depth_processor.cpp
)

target_link_libraries(test_depth_processor PRIVATE
    depth_processor
)

# libusb paths from SDK
set(CUBEEYE_LIBUSB_INCLUDE "${CUBEEYE_SDK_PATH}/thirdparty/libusb/include")
set(CUBEEYE_LIBUSB_LIB "${CUBEEYE_SDK_PATH}/thirdparty/libusb/lib")

# Direct libusb Capture Tool
add_executable(libusb_capture
    src/libusb_capture.c
)

target_include_directories(libusb_capture PRIVATE
    ${CUBEEYE_LIBUSB_INCLUDE}
)

target_link_directories(libusb_capture PRIVATE
    ${CUBEEYE_LIBUSB_LIB}
)

target_link_libraries(libusb_capture PRIVATE
    usb-1.0
    pthread
)

set_target_properties(libusb_capture PROPERTIES
    BUILD_RPATH "${CUBEEYE_LIBUSB_LIB}"
    INSTALL_RPATH "${CUBEEYE_LIBUSB_LIB}"
)

# Simple V4L2 Hook Library
add_library(simple_hook SHARED
    src/simple_v4l2_hook.c
)

target_link_libraries(simple_hook PRIVATE
    dl
    pthread
)

set_target_properties(simple_hook PROPERTIES
    PREFIX "lib"
    OUTPUT_NAME "simple_hook"
)

# USB Interceptor Library
add_library(usb_interceptor SHARED
    src/usb_interceptor.c
)

target_link_libraries(usb_interceptor PRIVATE
    dl
    pthread
)

set_target_properties(usb_interceptor PROPERTIES
    PREFIX "lib"
    OUTPUT_NAME "usb_interceptor"
)

# V4L2 Interceptor Library
add_library(v4l2_interceptor SHARED
    src/v4l2_interceptor.c
)

target_link_libraries(v4l2_interceptor PRIVATE
    dl
    pthread
)

set_target_properties(v4l2_interceptor PROPERTIES
    PREFIX "lib"
    OUTPUT_NAME "v4l2_interceptor"
)

# CubeEye Depth Extraction Library
add_library(cubeeye_depth STATIC
    src/cubeeye_depth.cpp
)

target_include_directories(cubeeye_depth PUBLIC
    src
)

# CubeEye Depth Test
add_executable(test_cubeeye_depth
    src/test_cubeeye_depth.cpp
)

target_link_libraries(test_cubeeye_depth PRIVATE
    cubeeye_depth
)

# CUDA CRT Depth Processing (Phase 4 - only if CUDA is available)
if(CUDAToolkit_FOUND)
    # add_executable(crt_depth
    #     src/crt_depth.cu
    # )
    # target_link_libraries(crt_depth PRIVATE
    #     CUDA::cudart
    # )
endif()

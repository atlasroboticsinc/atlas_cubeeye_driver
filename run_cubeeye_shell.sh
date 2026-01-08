#!/bin/bash
# Launcher script for CubeEyeShell with correct library paths

SDK_PATH="$HOME/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0"
COMPAT_LIBS="$HOME/development/atlas/code/cubeeye_nano_driver/compat_libs"

export LD_LIBRARY_PATH="${COMPAT_LIBS}:${SDK_PATH}/lib:${SDK_PATH}/thirdparty/libopencv/lib:${SDK_PATH}/thirdparty/libffmpeg/lib:${SDK_PATH}/thirdparty/liblive555/lib/Release:${LD_LIBRARY_PATH}"

echo "Running CubeEyeShell..."
exec "${SDK_PATH}/bin/CubeEyeShell" "$@"

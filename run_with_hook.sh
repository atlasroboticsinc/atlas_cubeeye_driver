#!/bin/bash
export SDK_PATH=/home/cmericli/development/atlas/code/atlas_levo/src/3rdparty/drivers/ros2-cubeeye2.0/cubeeye2.0
export LD_LIBRARY_PATH="${SDK_PATH}/lib:${SDK_PATH}/thirdparty/libopencv/lib:${SDK_PATH}/thirdparty/liblive555/lib/Release:${SDK_PATH}/thirdparty/libffmpeg/lib:${LD_LIBRARY_PATH}"
export HOOK_OUTPUT="${1:-data/sync_capture}"
export LD_PRELOAD="./build/libsimple_hook.so"

mkdir -p "$HOOK_OUTPUT"
exec ./build/sdk_capture "${2:-10}"

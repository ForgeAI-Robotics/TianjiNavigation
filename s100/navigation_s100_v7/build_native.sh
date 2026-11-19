#!/usr/bin/env bash
# Build on the target device: Jetson and S100 have different libstdc++ ABIs.
# Default output is isolated; this script never replaces active executables.
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
BUILD_ROOT="${1:-$TASK_ROOT/experiments/native_build}"
mkdir -p "$BUILD_ROOT/bin" "$BUILD_ROOT/include"
BUILD_FLAGS=(-std=c++17 -O2 "-I$TASK_ROOT/include" -I/usr/include/eigen3)
if [[ ! -f /usr/include/nlohmann/json.hpp ]]; then
    JSON_SOURCE="$TASK_ROOT/../navigation_stack_v3/vendor/rootfs/usr/include/nlohmann"
    [[ -f "$JSON_SOURCE/json.hpp" ]] || { echo 'nlohmann/json headers unavailable' >&2; exit 1; }
    ln -sfn "$JSON_SOURCE" "$BUILD_ROOT/include/nlohmann"
    BUILD_FLAGS+=("-I$BUILD_ROOT/include")
fi
for TARGET_NAME in hybrid world_guard voxel_box_test; do
    SOURCE_FILE="$TASK_ROOT/native/$TARGET_NAME.cpp"
    [[ "$TARGET_NAME" != voxel_box_test ]] || SOURCE_FILE="$TASK_ROOT/tests/voxel_box_native.cpp"
    g++ "${BUILD_FLAGS[@]}" "$SOURCE_FILE" -o "$BUILD_ROOT/bin/$TARGET_NAME.tmp"
    mv "$BUILD_ROOT/bin/$TARGET_NAME.tmp" "$BUILD_ROOT/bin/$TARGET_NAME"
done
g++ -std=c++17 -O3 -shared -fPIC "$TASK_ROOT/native/components.cpp" -o "$BUILD_ROOT/bin/libcomponents.so"
"$BUILD_ROOT/bin/voxel_box_test"
echo "Built target-device binaries under $BUILD_ROOT/bin"

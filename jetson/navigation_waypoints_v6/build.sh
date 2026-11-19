#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
STACK_ROOT="$PROJECT_ROOT/../navigation_stack_v3"
g++ -std=c++17 -O3 -I/usr/include/eigen3 -I"$STACK_ROOT/vendor/rootfs/usr/include" -I"$STACK_ROOT/ros2_ws/src/tianji_navigation_3d/include" "$PROJECT_ROOT/native/hybrid.cpp" -o "$PROJECT_ROOT/native/hybrid"

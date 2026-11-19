#!/usr/bin/env bash
set -eo pipefail
MODULE_ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export CYCLONEDDS_URI="file://$MODULE_ROOT/config/input_cyclonedds.xml"
exec /usr/bin/python3 "$MODULE_ROOT/modules/onboard_navigation.py" "$@"

#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export CYCLONEDDS_URI="file:///home/sunrise/tianji_table_approach_dev/navigation_s100_v7/config/input_cyclonedds.xml"
exec /usr/bin/python3 "/home/sunrise/tianji_table_approach_dev/navigation_bus_refactor_v1_20261116/gui_runtime/modules/preview_plan.py" "$@"

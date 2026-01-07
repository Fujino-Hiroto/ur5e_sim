#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

set +u
source /opt/ros/humble/setup.bash
set -u
source install/setup.bash

ros2 launch ur_slam_bringup ur5e_gazebo.launch.py

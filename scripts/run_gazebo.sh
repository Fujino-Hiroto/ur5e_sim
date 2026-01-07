#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_slam_bringup ur5e_gazebo.launch.py

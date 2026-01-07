#!/usr/bin/env bash
set -euo pipefail

echo "== Key topics =="
ros2 topic list | egrep "/clock|/tf$|/tf_static|/camera" || true

echo
echo "== TF quick check (world -> camera_link) =="
ros2 run tf2_ros tf2_echo world camera_link -t 0 || \
  (echo "TF check failed. Try: ros2 run tf2_ros tf2_echo world tool0 -t 0")

echo
echo "== Camera hz (best effort) =="
ros2 topic hz /camera/color/image_raw --window 30 2>/dev/null || true
ros2 topic hz /camera/depth/image_raw --window 30 2>/dev/null || true

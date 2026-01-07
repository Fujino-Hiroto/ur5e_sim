#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "[OK] colcon build done"

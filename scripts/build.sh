#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install

echo "[OK] colcon build done"

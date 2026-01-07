#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "[OK] rosdep install done"

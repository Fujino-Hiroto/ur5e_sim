#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

# Gazebo が model://cucumber を見つけられるように追加（unset対策込み）
WS_ROOT="$(pwd)"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/usr/share/gazebo-11/models:$WS_ROOT/src/ur_slam_bringup/models"

set -u

# ---- Plant pose / leaf targets TF (tunable) ----
PLANT_X=0.0
PLANT_Y=0.9
PLANT_Z=0.0

# Blenderの「正面が -X」なので、植物を x=+0.5 に置けば yaw=0 で正面がロボット側を向く
# もし「正面を +X に揃えたい」なら yaw=180（下の行を有効化）
PLANT_QX=0; PLANT_QY=0; PLANT_QZ=0.7071068; PLANT_QW=0.7071068
# PLANT_QX=0; PLANT_QY=0; PLANT_QZ=1; PLANT_QW=0   # yaw=180deg

SHOT_OFFSET=0.05  # スタンドオフ距離 [m]（まずは5cm推奨）

# background TF publishers
pids=()
trap 'kill "${pids[@]}" 2>/dev/null || true' EXIT

# world -> plant_base（植物モデル根本の配置）
ros2 run tf2_ros static_transform_publisher \
  "$PLANT_X" "$PLANT_Y" "$PLANT_Z" \
  "$PLANT_QX" "$PLANT_QY" "$PLANT_QZ" "$PLANT_QW" \
  base_link plant_base &
pids+=($!)

# plant_base -> leaf_target（+0.1505m 上げる）
ros2 run tf2_ros static_transform_publisher \
  -0.14212 0.032125 0.63595 \
  0.245305 -0.946771 0.031645 -0.206033 \
  plant_base leaf_target &

# leaf_target -> leaf_target_shot（めり込み回避：leaf_target +Z 方向にオフセット）
ros2 run tf2_ros static_transform_publisher \
  0 0 "$SHOT_OFFSET" \
  0 0 0 1 \
  leaf_target leaf_target_shot &
pids+=($!)

# leaf_target_shot -> leaf_target_view（camera +Z = leaf_target -Z に合わせる：X軸回り180deg）
ros2 run tf2_ros static_transform_publisher \
  0 0 0 \
  1 0 0 0 \
  leaf_target_shot leaf_target_view &
pids+=($!)
# ---- end TF ----

ros2 launch ur_slam_bringup ur5e_gazebo.launch.py

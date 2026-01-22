#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

# --- ROS env ---
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

MODE="${1:-clean}"   # clean | weak | mid | strong
SEED="${2:-42}"      # noiser seed

RUN_ID="$(date +%Y%m%d_%H%M%S)_${MODE}"
DB_PATH="$HOME/.ros/rtabmap_ur5e_${RUN_ID}.db"
BAG_DIR="bags/${RUN_ID}"
mkdir -p "${BAG_DIR}"

echo "[INFO] MODE=${MODE}"
echo "[INFO] SEED=${SEED}"
echo "[INFO] RUN_ID=${RUN_ID}"
echo "[INFO] DB_PATH=${DB_PATH}"
echo "[INFO] BAG_DIR=${BAG_DIR}"

# --- PIDs (for cleanup) ---
SLAM_PID=""
BAG_PID=""
NOISER_PID=""

cleanup() {
  echo "[INFO] Cleanup..."
  # Stop rosbag
  if [[ -n "${BAG_PID}" ]]; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
    BAG_PID=""
  fi
  # Stop SLAM
  if [[ -n "${SLAM_PID}" ]]; then
    kill -INT "${SLAM_PID}" 2>/dev/null || true
    wait "${SLAM_PID}" 2>/dev/null || true
    SLAM_PID=""
  fi
  # Stop noiser
  if [[ -n "${NOISER_PID}" ]]; then
    kill -INT "${NOISER_PID}" 2>/dev/null || true
    wait "${NOISER_PID}" 2>/dev/null || true
    NOISER_PID=""
  fi
}
trap cleanup EXIT INT TERM

# --- Noise presets (tune here) ---
# Notes:
# - dropout_p: 点群欠損（NaN）
# - gauss_a/b: 深度依存ガウス（sigma = a + b*z^2）
# - spike_p: 偽の近距離リターン（zを縮める）
# - block_*: 画像面の穴（organized cloud のとき効く）
IN_TOPIC="/camera/depth/points"
OUT_TOPIC="/camera/depth/points_noisy"

# defaults
DROPOUT_P="0.00"
BLOCK_DROPOUT="false"
BLOCK_COUNT="0"
BLOCK_RADIUS_PX="0"
GAUSS_A="0.0"
GAUSS_B="0.0"
SPIKE_P="0.0"
SPIKE_MIN="0.3"
SPIKE_MAX="0.8"
ENABLE_FLICKER="false"
FLICKER_HZ="1.0"
FLICKER_AMP="0.5"
FRAME_DROP_P="0.0"
MIN_Z="0.0"
MAX_Z="5.0"

case "${MODE}" in
  clean)
    # no noiser
    ;;
  weak)
    DROPOUT_P="0.01"
    BLOCK_DROPOUT="true"
    BLOCK_COUNT="4"
    BLOCK_RADIUS_PX="4"
    GAUSS_A="0.0015"
    GAUSS_B="0.0006"
    SPIKE_P="0.001"
    ENABLE_FLICKER="false"
    ;;
  mid)
    DROPOUT_P="0.03"
    BLOCK_DROPOUT="true"
    BLOCK_COUNT="8"
    BLOCK_RADIUS_PX="6"
    GAUSS_A="0.0030"
    GAUSS_B="0.0012"
    SPIKE_P="0.004"
    ENABLE_FLICKER="true"
    FLICKER_HZ="1.0"
    FLICKER_AMP="0.6"
    ;;
  strong)
    DROPOUT_P="0.07"
    BLOCK_DROPOUT="true"
    BLOCK_COUNT="14"
    BLOCK_RADIUS_PX="8"
    GAUSS_A="0.0060"
    GAUSS_B="0.0025"
    SPIKE_P="0.010"
    ENABLE_FLICKER="true"
    FLICKER_HZ="1.2"
    FLICKER_AMP="0.9"
    FRAME_DROP_P="0.02"
    ;;
  *)
    echo "[ERROR] Unknown MODE: ${MODE} (use clean|weak|mid|strong)"
    exit 1
    ;;
esac

# --- Start PointCloudNoiser (background) if needed ---
if [[ "${MODE}" != "clean" ]]; then
  echo "[INFO] Starting PointCloudNoiser..."
  echo "[INFO]  ${IN_TOPIC} -> ${OUT_TOPIC}"
  echo "[INFO]  dropout_p=${DROPOUT_P} block=${BLOCK_DROPOUT} blocks=${BLOCK_COUNT} rad_px=${BLOCK_RADIUS_PX}"
  echo "[INFO]  gauss_a=${GAUSS_A} gauss_b=${GAUSS_B} spike_p=${SPIKE_P} flicker=${ENABLE_FLICKER}"
  ros2 run ur_slam_tools pointcloud_noiser.py --ros-args \
    -p in_topic:="${IN_TOPIC}" \
    -p out_topic:="${OUT_TOPIC}" \
    -p seed:="${SEED}" \
    -p dropout_p:="${DROPOUT_P}" \
    -p block_dropout:="${BLOCK_DROPOUT}" \
    -p block_count:="${BLOCK_COUNT}" \
    -p block_radius_px:="${BLOCK_RADIUS_PX}" \
    -p gauss_a:="${GAUSS_A}" \
    -p gauss_b:="${GAUSS_B}" \
    -p spike_p:="${SPIKE_P}" \
    -p spike_min_factor:="${SPIKE_MIN}" \
    -p spike_max_factor:="${SPIKE_MAX}" \
    -p enable_flicker:="${ENABLE_FLICKER}" \
    -p flicker_hz:="${FLICKER_HZ}" \
    -p flicker_amp:="${FLICKER_AMP}" \
    -p frame_drop_p:="${FRAME_DROP_P}" \
    -p min_z:="${MIN_Z}" \
    -p max_z:="${MAX_Z}" \
    > "${BAG_DIR}/noiser.log" 2>&1 &
  NOISER_PID=$!

  # Wait until published (best-effort)
  echo "[INFO] Waiting /camera/depth/points_noisy ..."
  for i in {1..40}; do
    if ros2 topic list 2>/dev/null | grep -q "^${OUT_TOPIC}$"; then
      break
    fi
    sleep 0.2
  done
fi

# --- Start SLAM (background) ---
# 注意:
# RTAB-Map が depth image を使っている構成だと、この noiser は SLAM結果に直接影響しません。
# ただし /cloud_map や /camera/depth/points(_noisy) を bag に残すので、CloudCompareでの比較は可能です。
echo "[INFO] Starting SLAM..."
ros2 launch ur_slam_bringup ur5e_slam.launch.py with_gazebo:=false \
  db_path:="${DB_PATH}" delete_db_on_start:=true \
  > "${BAG_DIR}/slam.log" 2>&1 &
SLAM_PID=$!

# --- Wait for action server ---
echo "[INFO] Waiting for FollowJointTrajectory action server..."
for i in {1..60}; do
  if ros2 action list 2>/dev/null | grep -q "/joint_trajectory_controller/follow_joint_trajectory"; then
    break
  fi
  sleep 0.5
done

# --- Start rosbag (background) ---
# 比較実験向けに「noisy も一緒に記録」します
echo "[INFO] Starting rosbag record..."
ros2 bag record -o "${BAG_DIR}/bag" \
  /tf /tf_static /clock \
  /joint_states \
  /camera/color/image_raw /camera/color/camera_info \
  /camera/depth/depth/image_raw /camera/depth/depth/camera_info \
  /camera/depth/points \
  /camera/depth/points_noisy \
  /cloud_map \
  > "${BAG_DIR}/rosbag.log" 2>&1 &
BAG_PID=$!

# --- (Optional) Dump params for reproducibility ---
# ノードが存在しない場合はスキップされます
ros2 param dump /rtabmap > "${BAG_DIR}/params_rtabmap.yaml" 2>/dev/null || true
ros2 param dump /pointcloud_noiser > "${BAG_DIR}/params_pointcloud_noiser.yaml" 2>/dev/null || true

# --- Send trajectory (pan sweep ±45° around -90°) ---
echo "[INFO] Sending trajectory..."
ros2 action send_goal --feedback \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{
    trajectory: {
      joint_names: [
        shoulder_pan_joint,
        shoulder_lift_joint,
        elbow_joint,
        wrist_1_joint,
        wrist_2_joint,
        wrist_3_joint
      ],
      points: [
        { positions: [-1.5707963268, -0.5235987756, -2.0943951024, -0.5235987756,  1.5707963268, 0.0872664626],
          time_from_start: {sec: 3, nanosec: 0} },

        { positions: [-1.5707963268, -0.5235987756, -2.0943951024, -0.5235987756,  1.5707963268, 0.0872664626],
          time_from_start: {sec: 6, nanosec: 0} },

        { positions: [-2.3561944902, -0.5235987756, -2.0943951024, -0.5235987756,  1.5707963268, 0.0872664626],
          time_from_start: {sec: 16, nanosec: 0} },

        { positions: [-0.7853981634,  -0.5235987756, -2.0943951024, -0.5235987756,  1.5707963268, 0.0872664626],
          time_from_start: {sec: 26, nanosec: 0} },

        { positions: [-1.5707963268,  -0.5235987756, -2.0943951024, -0.5235987756,  1.5707963268, 0.0872664626],
          time_from_start: {sec: 36, nanosec: 0} }
      ]
    }
  }'

echo "[INFO] Trajectory done. (rosbag/slam/noiser will stop via trap cleanup)"
echo "[OK] DB saved at: ${DB_PATH}"
echo "[OK] rosbag saved under: ${BAG_DIR}/bag"
echo "[OK] Logs: ${BAG_DIR}/slam.log , ${BAG_DIR}/rosbag.log , ${BAG_DIR}/noiser.log"

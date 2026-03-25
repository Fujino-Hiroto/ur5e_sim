#!/usr/bin/env bash
set -euo pipefail

SEED="${1:-46}"                      # ./scripts/run_all.sh 46
RUN_ID="${2:-ompldirect_v15_seed${SEED}}"

WS_ROOT="${HOME}/workspaces/ur_slam_ws"
CSV_PATH="results/leaf5_log.csv"
POST_LEAF_HOLD_SEC="${POST_LEAF_HOLD_SEC:-5}"

pgids=()

start_bg() {
  local name="$1"; shift
  echo "[run_all] start (bg): ${name}"
  setsid bash -lc "$*" >/tmp/run_all_${name}.log 2>&1 &
  local pid=$!
  pgids+=("${pid}")
  echo "[run_all]   ${name} pid/pgid=${pid} log=/tmp/run_all_${name}.log"
}

kill_all() {
  echo "[run_all] stopping all started processes..."

  for pg in "${pgids[@]:-}"; do
    if kill -0 "${pg}" 2>/dev/null; then
      kill -TERM "-${pg}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pg in "${pgids[@]:-}"; do
    if kill -0 "${pg}" 2>/dev/null; then
      kill -KILL "-${pg}" 2>/dev/null || true
    fi
  done

  pkill -TERM -f gzserver 2>/dev/null || true
  pkill -TERM -f gzclient 2>/dev/null || true
  pkill -TERM -f "gazebo" 2>/dev/null || true
  pkill -TERM -f "ign gazebo" 2>/dev/null || true
  sleep 1
  pkill -KILL -f gzserver 2>/dev/null || true
  pkill -KILL -f gzclient 2>/dev/null || true

  pkill -TERM -f "ros2 launch ur_slam_bringup ur5e_moveit.launch.py" 2>/dev/null || true
  pkill -TERM -f "ros2 launch ur_slam_bringup ur5e_slam.launch.py" 2>/dev/null || true
  pkill -TERM -f "depth_image_noiser.py" 2>/dev/null || true
  pkill -TERM -f "cloud_gate.py" 2>/dev/null || true
  pkill -TERM -f "arc_sweep_jointtraj.py" 2>/dev/null || true
  pkill -TERM -f "coarse_to_fine_go_to_leaf5" 2>/dev/null || true

  echo "[run_all] cleanup done."
}

trap 'kill_all; exit 130' INT
trap 'kill_all' TERM EXIT

# ---- source ROS env (avoid nounset crash) ----
set +u
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

mkdir -p "$(dirname "${CSV_PATH}")"

echo "[run_all] 1) Gazebo bringup (plants + robot spawn)"
start_bg "gazebo" "./scripts/run_gazebo.sh"
sleep 3

echo "[run_all] 2) MoveIt"
start_bg "moveit" "ros2 launch ur_slam_bringup ur5e_moveit.launch.py"
sleep 4

echo "[run_all] 3) SLAM"
start_bg "slam" "ros2 launch ur_slam_bringup ur5e_slam.launch.py"
sleep 4

echo "[run_all] 4) cloud_gate (bg)"
start_bg "cloud_gate" "ros2 run ur_slam_tools cloud_gate.py --ros-args -p use_sim_time:=true"
sleep 1

echo "[run_all] 5) depth_image_noiser (bg)"
start_bg "noiser" "ros2 run ur_slam_tools depth_image_noiser.py --ros-args \
  -p use_sim_time:=true \
  -p dropout_p:=0.05 \
  -p block_dropout:=true \
  -p block_count:=7 \
  -p block_radius_px:=20 \
  -p gauss_a:=0.010 \
  -p gauss_b:=0.0005 \
  -p enable_edge_spikes:=true \
  -p edge_grad_thr:=0.30 \
  -p edge_spike_p:=0.020 \
  -p edge_spike_min_factor:=0.93 \
  -p edge_spike_max_factor:=0.98 \
  -p seed:=${SEED}"
sleep 2

# echo "[run_all] 6) arc_sweep_jointtraj (blocking)"
# ARC_RC=0
# ros2 run ur_slam_tools arc_sweep_jointtraj.py --ros-args \
#   -p use_sim_time:=true \
#   -p time_per_segment:=2.0 \
#   -p hold_sec:=0.0 || ARC_RC=$?
# echo "[run_all] arc_sweep finished. exit_code=${ARC_RC}"

echo "[run_all] 7) coarse_to_fine_go_to_leaf5 (blocking; after arc)"
LEAF_RC=0
ros2 run ur_slam_tools coarse_to_fine_go_to_leaf5 --ros-args \
  -p planning_mode:=ompl_direct \
  -p use_sim_time:=true \
  -p world_frame:=base_link \
  -p ee_link:=tool0 \
  -p camera_frame:=camera_color_optical_frame \
  -p leaf_center_frame:=leaf_target \
  -p fine_goal_frame:=leaf_target_shot \
  -p view_goal_frame:=leaf_target_view \
  -p tf_timeout_sec:=10.0 \
  -p fine_planning_time:=15.0 \
  -p fine_vel_scale:=0.15 -p fine_acc_scale:=0.15 \
  -p fine_orient_mode:=lookat \
  -p csv_enable:=true -p csv_path:="${CSV_PATH}" \
  -p csv_run_id:="${RUN_ID}" || LEAF_RC=$?

echo "[run_all] leaf5 finished. exit_code=${LEAF_RC}"
echo "[run_all] hold ${POST_LEAF_HOLD_SEC}s for recording..."
sleep "${POST_LEAF_HOLD_SEC}"

echo "[run_all] resetting (stopping everything)..."
kill_all
exit "${LEAF_RC}"

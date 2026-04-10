#!/usr/bin/env bash
set -uo pipefail

# ============================================================
# run_bench.sh — 指定条件の実験をN回連続自動実行
# Usage: ./scripts/run_bench.sh <CONDITION> <NOISE> [N_TRIALS]
#
# CONDITION:
#   A = ompl_two_stage + blend alpha=0.7  (卒論ベースライン)
#   B = ompl_two_stage + fixed            (問題①を抑制)
#   C = hybrid + blend alpha=0.7          (問題①②を解決)
# NOISE: clean | weak | medium | strong
# ============================================================

# ---------- 引数パース ----------
CONDITION="${1:?Usage: $0 <CONDITION: A|B|C> <NOISE: clean|weak|medium|strong> [N_TRIALS]}"
NOISE="${2:?Usage: $0 <CONDITION: A|B|C> <NOISE: clean|weak|medium|strong> [N_TRIALS]}"
N_TRIALS="${3:-10}"

# ---------- バリデーション ----------
case "$CONDITION" in
  A|B|C) ;;
  *) echo "[ERROR] CONDITION must be A, B, or C (got: $CONDITION)" >&2; exit 1 ;;
esac

case "$NOISE" in
  clean|weak|medium|strong) ;;
  *) echo "[ERROR] NOISE must be clean, weak, medium, or strong (got: $NOISE)" >&2; exit 1 ;;
esac

if ! [[ "$N_TRIALS" =~ ^[0-9]+$ ]] || [ "$N_TRIALS" -lt 1 ]; then
  echo "[ERROR] N_TRIALS must be a positive integer (got: $N_TRIALS)" >&2; exit 1
fi

# ---------- 定数 ----------
WS_SETUP="/home/fujino/ur5e_sim/install/setup.bash"
RESULTS_DIR="/home/fujino/ur5e_sim/results"
WAIT_TIMEOUT=60   # トピック待機タイムアウト(秒)
CLEANUP_WAIT=10   # cleanup後の待機時間(秒)

# ---------- ワークスペース source ----------
if [ -f "$WS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$WS_SETUP"
else
  echo "[ERROR] Workspace setup not found: $WS_SETUP" >&2; exit 1
fi

# ---------- results/ 自動作成 ----------
mkdir -p "$RESULTS_DIR"

# ---------- CONDITION → planning_mode / fine_orient_mode ----------
case "$CONDITION" in
  A)
    PLANNING_MODE="ompl_two_stage"
    FINE_ORIENT_MODE="blend"
    FINE_ORIENT_ALPHA="0.7"
    ;;
  B)
    PLANNING_MODE="ompl_two_stage"
    FINE_ORIENT_MODE="fixed"
    FINE_ORIENT_ALPHA="0.7"  # fixed モードでは使われないが一応設定
    ;;
  C)
    PLANNING_MODE="hybrid"
    FINE_ORIENT_MODE="blend"
    FINE_ORIENT_ALPHA="0.7"
    ;;
esac

# ---------- NOISE パラメータ ----------
noise_args() {
  local common="-p seed:=42"
  case "$NOISE" in
    clean)
      echo "$common" \
        "-p dropout_p:=0.0" \
        "-p block_dropout:=false" \
        "-p gauss_a:=0.0 -p gauss_b:=0.0" \
        "-p enable_edge_spikes:=false"
      ;;
    weak)
      echo "$common" \
        "-p dropout_p:=0.01" \
        "-p block_dropout:=true" \
        "-p block_count:=2 -p block_radius_px:=12" \
        "-p gauss_a:=0.003 -p gauss_b:=0.0002" \
        "-p enable_edge_spikes:=true" \
        "-p edge_grad_thr:=0.40" \
        "-p edge_spike_p:=0.005" \
        "-p edge_spike_min_factor:=0.97 -p edge_spike_max_factor:=0.995"
      ;;
    medium)
      echo "$common" \
        "-p dropout_p:=0.02" \
        "-p block_dropout:=true" \
        "-p block_count:=5 -p block_radius_px:=18" \
        "-p gauss_a:=0.006 -p gauss_b:=0.0003" \
        "-p enable_edge_spikes:=true" \
        "-p edge_grad_thr:=0.35" \
        "-p edge_spike_p:=0.015" \
        "-p edge_spike_min_factor:=0.96 -p edge_spike_max_factor:=0.99"
      ;;
    strong)
      echo "$common" \
        "-p dropout_p:=0.05" \
        "-p block_dropout:=true" \
        "-p block_count:=7 -p block_radius_px:=20" \
        "-p gauss_a:=0.010 -p gauss_b:=0.0005" \
        "-p enable_edge_spikes:=true" \
        "-p edge_grad_thr:=0.30" \
        "-p edge_spike_p:=0.020" \
        "-p edge_spike_min_factor:=0.93 -p edge_spike_max_factor:=0.98"
      ;;
  esac
}

# ---------- ユーティリティ ----------
CHILD_PIDS=()

cleanup() {
  echo "[INFO] Cleaning up child processes..."
  for pid in "${CHILD_PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  # SIGINT で終わらないプロセスを SIGKILL
  sleep 3
  for pid in "${CHILD_PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${CHILD_PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
  CHILD_PIDS=()
  # 次の試行前にプロセスが完全に終了するのを待つ
  echo "[INFO] Waiting ${CLEANUP_WAIT}s for processes to fully terminate..."
  sleep "$CLEANUP_WAIT"
}

trap cleanup EXIT

wait_for_topic() {
  local topic="$1"
  local timeout="$2"
  echo "[INFO] Waiting for topic: $topic (timeout: ${timeout}s)"
  local deadline=$((SECONDS + timeout))
  while [ $SECONDS -lt $deadline ]; do
    if ros2 topic hz "$topic" --window 1 2>/dev/null | grep -q "average rate"; then
      echo "[INFO] Topic $topic is active."
      return 0
    fi
    sleep 2
  done
  echo "[WARN] Timeout waiting for topic: $topic" >&2
  return 1
}

# ---------- メインループ ----------
echo "========================================"
echo " run_bench.sh"
echo " CONDITION=$CONDITION ($PLANNING_MODE + $FINE_ORIENT_MODE)"
echo " NOISE=$NOISE  N_TRIALS=$N_TRIALS"
echo "========================================"

for i in $(seq 1 "$N_TRIALS"); do
  RUN_ID=$(printf "%s_%s_%03d" "$CONDITION" "$NOISE" "$i")
  echo ""
  echo "[TRIAL $i/$N_TRIALS] START  run_id=$RUN_ID"
  echo "----------------------------------------"
  TRIAL_OK=true

  # --- 1. Gazebo ---
  echo "[INFO] Launching Gazebo..."
  ros2 launch ur_slam_bringup ur5e_gazebo.launch.py &
  CHILD_PIDS+=($!)
  if ! wait_for_topic /clock "$WAIT_TIMEOUT"; then
    echo "[ERROR] Gazebo did not start. Skipping trial $i." >&2
    TRIAL_OK=false
  fi

  # --- 2. MoveIt2 ---
  if $TRIAL_OK; then
    echo "[INFO] Launching MoveIt2..."
    ros2 launch ur_slam_bringup ur5e_moveit.launch.py &
    CHILD_PIDS+=($!)
    if ! wait_for_topic /move_group/status "$WAIT_TIMEOUT"; then
      echo "[ERROR] MoveIt2 did not start. Skipping trial $i." >&2
      TRIAL_OK=false
    fi
  fi

  # --- 3. RTAB-Map ---
  if $TRIAL_OK; then
    echo "[INFO] Launching RTAB-Map..."
    ros2 launch ur_slam_bringup ur5e_slam.launch.py &
    CHILD_PIDS+=($!)
    if ! wait_for_topic /rtabmap/mapData "$WAIT_TIMEOUT"; then
      echo "[ERROR] RTAB-Map did not start. Skipping trial $i." >&2
      TRIAL_OK=false
    fi
  fi

  # --- 4. cloud_gate ---
  if $TRIAL_OK; then
    echo "[INFO] Launching cloud_gate..."
    ros2 run ur_slam_tools cloud_gate.py --ros-args -p use_sim_time:=true &
    CHILD_PIDS+=($!)
    if ! wait_for_topic /cloud_gate/status "$WAIT_TIMEOUT"; then
      echo "[WARN] cloud_gate topic not confirmed, continuing anyway."
    fi
  fi

  # --- 5. depth_image_noiser ---
  if $TRIAL_OK; then
    echo "[INFO] Launching depth_image_noiser (NOISE=$NOISE)..."
    # shellcheck disable=SC2046
    ros2 run ur_slam_tools depth_image_noiser.py --ros-args \
      -p use_sim_time:=true \
      $(noise_args) &
    CHILD_PIDS+=($!)
    if ! wait_for_topic /camera/depth/depth/image_raw_noisy "$WAIT_TIMEOUT"; then
      echo "[WARN] depth_image_noiser topic not confirmed, continuing anyway."
    fi
  fi

  # --- 6. arc_sweep_jointtraj ---
  if $TRIAL_OK; then
    echo "[INFO] Launching arc_sweep_jointtraj..."
    ros2 run ur_slam_tools arc_sweep_jointtraj.py --ros-args \
      -p use_sim_time:=true &
    ARC_PID=$!
    CHILD_PIDS+=($ARC_PID)
    echo "[INFO] Waiting for arc_sweep_jointtraj to finish..."
    wait "$ARC_PID" 2>/dev/null || true
  fi

  # --- 7. coarse_to_fine_go_to_leaf5 ---
  if $TRIAL_OK; then
    echo "[INFO] Launching coarse_to_fine_go_to_leaf5 (CONDITION=$CONDITION, run_id=$RUN_ID)..."
    ros2 run ur_slam_tools coarse_to_fine_go_to_leaf5 --ros-args \
      -p use_sim_time:=true \
      -p world_frame:=base_link \
      -p ee_link:=tool0 \
      -p camera_frame:=camera_color_optical_frame \
      -p leaf_center_frame:=leaf_target \
      -p fine_goal_frame:=leaf_target_shot \
      -p view_goal_frame:=leaf_target_view \
      -p tf_timeout_sec:=10.0 \
      -p pre_back_m:=0.20 -p pre_down_m:=0.20 \
      -p coarse_max_tries:=5 -p coarse_retry_sleep_sec:=0.2 \
      -p coarse_planning_time:=15.0 -p fine_planning_time:=10.0 -p view_planning_time:=5.0 \
      -p coarse_vel_scale:=0.05 -p coarse_acc_scale:=0.05 \
      -p fine_vel_scale:=0.05 -p fine_acc_scale:=0.05 \
      -p view_vel_scale:=0.05 -p view_acc_scale:=0.05 \
      -p fine_orient_mode:="$FINE_ORIENT_MODE" \
      -p fine_orient_blend_alpha:="$FINE_ORIENT_ALPHA" \
      -p fine_cartesian_eef_step:=0.002 \
      -p fine_cartesian_jump_threshold:=0.0 \
      -p fine_cartesian_min_fraction:=0.60 \
      -p fine_cartesian_avoid_collisions:=true \
      -p "fine_cartesian_eef_step_candidates:=[0.002, 0.005, 0.01]" \
      -p state_sync_sleep_ms:=300 \
      -p p0_warn_rad:=0.02 \
      -p csv_enable:=true \
      -p csv_path:="$RESULTS_DIR/leaf5_log.csv" \
      -p csv_run_id:="$RUN_ID" \
      -p planning_mode:="$PLANNING_MODE" &
    LEAF_PID=$!
    CHILD_PIDS+=($LEAF_PID)
    echo "[INFO] Waiting for coarse_to_fine_go_to_leaf5 to finish..."
    wait "$LEAF_PID" 2>/dev/null || true
  fi

  # --- 試行終了 → cleanup して次へ ---
  STATUS=$( $TRIAL_OK && echo "SUCCESS" || echo "SKIPPED" )
  echo "[TRIAL $i/$N_TRIALS] DONE  run_id=$RUN_ID  status=$STATUS"
  cleanup
done

echo ""
echo "========================================"
echo " All $N_TRIALS trials complete."
echo " Results: $RESULTS_DIR/leaf5_log.csv"
echo "========================================"
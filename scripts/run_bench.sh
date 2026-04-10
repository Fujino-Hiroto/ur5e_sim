#!/usr/bin/env bash
set -uo pipefail

# ============================================================
# run_bench.sh — 指定条件の実験をN回連続自動実行
# Usage: ./scripts/run_bench.sh [--dry-run] <CONDITION> <NOISE> [N_TRIALS]
#
# CONDITION:
#   A = ompl_two_stage + blend alpha=0.7  (卒論ベースライン)
#   B = ompl_two_stage + fixed            (問題①を抑制)
#   C = hybrid + blend alpha=0.7          (問題①②を解決)
# NOISE: clean | weak | medium | strong
# ============================================================

# ---------- 引数パース ----------
DRY_RUN=false
if [ "${1:-}" = "--dry-run" ]; then
  DRY_RUN=true
  shift
fi

CONDITION="${1:?Usage: $0 [--dry-run] <CONDITION: A|B|C> <NOISE: clean|weak|medium|strong> [N_TRIALS]}"
NOISE="${2:?Usage: $0 [--dry-run] <CONDITION: A|B|C> <NOISE: clean|weak|medium|strong> [N_TRIALS]}"
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
WS_ROOT="/home/fujino/ur5e_sim"
WS_SETUP="$WS_ROOT/install/setup.bash"
RESULTS_DIR="$WS_ROOT/results"
CLEANUP_WAIT=10   # cleanup後の待機時間(秒)

# 起動待機時間 (PC性能に合わせてチューニング)
GAZEBO_STARTUP_SEC=15
MOVEIT_STARTUP_SEC=15
SLAM_STARTUP_SEC=20
TOOL_STARTUP_SEC=5

# ---------- ログ関数 ----------
log() { echo "[$(date '+%H:%M:%S')] $*"; }

# ---------- ワークスペース source ----------
if [ -f "$WS_SETUP" ]; then
  # shellcheck disable=SC1090
  set +u
  source "$WS_SETUP"
  set -u
else
  echo "[ERROR] Workspace setup not found: $WS_SETUP" >&2; exit 1
fi

# ---------- GAZEBO_MODEL_PATH (run_gazebo.sh と同等) ----------
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:/usr/share/gazebo-11/models:$WS_ROOT/src/ur_slam_bringup/models"

# ---------- results/ 自動作成 ----------
mkdir -p "$RESULTS_DIR"

# ---------- Plant pose / leaf targets TF (run_gazebo.sh と同等) ----------
PLANT_X=0.0; PLANT_Y=0.83; PLANT_Z=0.0
PLANT_QX=0; PLANT_QY=0; PLANT_QZ=0.7071068; PLANT_QW=0.7071068
SHOT_OFFSET=0.07

start_tf_publishers() {
  setsid ros2 run tf2_ros static_transform_publisher \
    "$PLANT_X" "$PLANT_Y" "$PLANT_Z" \
    "$PLANT_QX" "$PLANT_QY" "$PLANT_QZ" "$PLANT_QW" \
    world plant_base &
  GROUP_PIDS+=($!)

  setsid ros2 run tf2_ros static_transform_publisher \
    -0.14212 0.032125 0.63595 \
    0.245305 -0.946771 0.031645 -0.206033 \
    plant_base leaf_target &
  GROUP_PIDS+=($!)

  setsid ros2 run tf2_ros static_transform_publisher \
    0 0 "$SHOT_OFFSET" \
    0 0 0 1 \
    leaf_target leaf_target_shot &
  GROUP_PIDS+=($!)

  setsid ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    1 0 0 0 \
    leaf_target_shot leaf_target_view &
  GROUP_PIDS+=($!)

  log "[INFO] TF publishers started (4 nodes)"
}

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

# ============================================================
# プロセス管理 — setsid + プロセスグループ kill
# ============================================================
GROUP_PIDS=()

cleanup() {
  log "[INFO] Cleaning up (killing ${#GROUP_PIDS[@]} process groups)..."
  # SIGINT でグループごと終了
  for gpid in "${GROUP_PIDS[@]:-}"; do
    kill -- -"$gpid" 2>/dev/null || true
  done
  sleep 3
  # 残っていれば SIGKILL
  for gpid in "${GROUP_PIDS[@]:-}"; do
    kill -9 -- -"$gpid" 2>/dev/null || true
  done
  GROUP_PIDS=()
  log "[INFO] Waiting ${CLEANUP_WAIT}s for processes to fully terminate..."
  sleep "$CLEANUP_WAIT"
}

trap cleanup EXIT

# ============================================================
# readiness 検出 — 固定sleep + topic 検証
# ============================================================
wait_ready() {
  local name="$1"
  local sleep_sec="$2"
  local check_topic="${3:-}"

  log "[INFO] Waiting ${sleep_sec}s for $name to start..."
  sleep "$sleep_sec"

  if [ -n "$check_topic" ]; then
    if timeout 10 ros2 topic echo "$check_topic" --once > /dev/null 2>&1; then
      log "[INFO] $name ready (topic $check_topic confirmed)."
    else
      log "[WARN] $name: topic $check_topic not detected after ${sleep_sec}s wait."
      return 1
    fi
  fi
  return 0
}

# ---------- サマリーカウンター ----------
N_SUCCESS=0
N_FAIL=0
N_SKIP=0

# ---------- dry-run ----------
if $DRY_RUN; then
  RUN_ID=$(printf "%s_%s_%03d" "$CONDITION" "$NOISE" 1)
  cat <<EOM
[DRY-RUN] CONDITION=$CONDITION ($PLANNING_MODE + $FINE_ORIENT_MODE)
[DRY-RUN] NOISE=$NOISE  N_TRIALS=$N_TRIALS  run_id=$RUN_ID
[DRY-RUN] LOG_FILE: dry-run mode, no log file created

# 0. TF publishers (4 nodes)
# 1. Gazebo  (wait ${GAZEBO_STARTUP_SEC}s + /clock check)
ros2 launch ur_slam_bringup ur5e_gazebo.launch.py

# 2. MoveIt2 (wait ${MOVEIT_STARTUP_SEC}s + /move_group/monitored_planning_scene check)
ros2 launch ur_slam_bringup ur5e_moveit.launch.py

# 3. RTAB-Map (wait ${SLAM_STARTUP_SEC}s + /rtabmap/mapData check)
ros2 launch ur_slam_bringup ur5e_slam.launch.py

# 4. cloud_gate (wait ${TOOL_STARTUP_SEC}s)
ros2 run ur_slam_tools cloud_gate.py --ros-args -p use_sim_time:=true

# 5. depth_image_noiser (wait ${TOOL_STARTUP_SEC}s)
ros2 run ur_slam_tools depth_image_noiser.py --ros-args \\
  -p use_sim_time:=true \\
  $(noise_args)

# 6. arc_sweep_jointtraj (wait for exit)
ros2 run ur_slam_tools arc_sweep_jointtraj.py --ros-args -p use_sim_time:=true

# 7. coarse_to_fine_go_to_leaf5 (wait for exit)
ros2 run ur_slam_tools coarse_to_fine_go_to_leaf5 --ros-args \\
  -p use_sim_time:=true \\
  -p world_frame:=base_link \\
  -p ee_link:=tool0 \\
  -p camera_frame:=camera_color_optical_frame \\
  -p leaf_center_frame:=leaf_target \\
  -p fine_goal_frame:=leaf_target_shot \\
  -p view_goal_frame:=leaf_target_view \\
  -p tf_timeout_sec:=10.0 \\
  -p pre_back_m:=0.20 -p pre_down_m:=0.20 \\
  -p coarse_max_tries:=5 -p coarse_retry_sleep_sec:=0.2 \\
  -p coarse_planning_time:=15.0 -p fine_planning_time:=10.0 -p view_planning_time:=5.0 \\
  -p coarse_vel_scale:=0.05 -p coarse_acc_scale:=0.05 \\
  -p fine_vel_scale:=0.05 -p fine_acc_scale:=0.05 \\
  -p view_vel_scale:=0.05 -p view_acc_scale:=0.05 \\
  -p fine_orient_mode:=$FINE_ORIENT_MODE \\
  -p fine_orient_blend_alpha:=$FINE_ORIENT_ALPHA \\
  -p fine_cartesian_eef_step:=0.002 \\
  -p fine_cartesian_jump_threshold:=0.0 \\
  -p fine_cartesian_min_fraction:=0.60 \\
  -p fine_cartesian_avoid_collisions:=true \\
  -p "fine_cartesian_eef_step_candidates:=[0.002, 0.005, 0.01]" \\
  -p state_sync_sleep_ms:=300 \\
  -p p0_warn_rad:=0.02 \\
  -p csv_enable:=true \\
  -p csv_path:=$RESULTS_DIR/leaf5_log.csv \\
  -p csv_run_id:=$RUN_ID \\
  -p planning_mode:=$PLANNING_MODE
EOM
  exit 0
fi

# ---------- ログファイル出力 (tee) ----------
LOG_FILE="$RESULTS_DIR/run_bench_$(date '+%Y%m%d_%H%M%S').log"
exec > >(tee -a "$LOG_FILE") 2>&1

# ---------- 環境情報ヘッダー ----------
log "========================================"
log " run_bench.sh"
log " CONDITION=$CONDITION ($PLANNING_MODE + $FINE_ORIENT_MODE)"
log " NOISE=$NOISE  N_TRIALS=$N_TRIALS"
log " ROS_DISTRO=${ROS_DISTRO:-unknown}"
log " WS=$WS_SETUP"
log " LOG=$LOG_FILE"
log "========================================"

# ============================================================
# メインループ
# ============================================================
for i in $(seq 1 "$N_TRIALS"); do
  RUN_ID=$(printf "%s_%s_%03d" "$CONDITION" "$NOISE" "$i")
  TRIAL_START=$SECONDS
  log ""
  log "[TRIAL $i/$N_TRIALS] START  run_id=$RUN_ID"
  log "----------------------------------------"
  TRIAL_OK=true
  ARC_EXIT=0
  LEAF_EXIT=0

  # --- 0. TF publishers ---
  start_tf_publishers

  # --- 1. Gazebo ---
  log "[INFO] Launching Gazebo..."
  setsid ros2 launch ur_slam_bringup ur5e_gazebo.launch.py &
  GROUP_PIDS+=($!)
  log "[INFO] Gazebo PGID=$!"
  if ! wait_ready "Gazebo" "$GAZEBO_STARTUP_SEC" /clock; then
    log "[ERROR] Gazebo did not start. Skipping trial $i." >&2
    TRIAL_OK=false
  fi

  # --- 2. MoveIt2 ---
  if $TRIAL_OK; then
    log "[INFO] Launching MoveIt2..."
    setsid ros2 launch ur_slam_bringup ur5e_moveit.launch.py &
    GROUP_PIDS+=($!)
    log "[INFO] MoveIt2 PGID=$!"
    if ! wait_ready "MoveIt2" "$MOVEIT_STARTUP_SEC" /monitored_planning_scene; then
      log "[ERROR] MoveIt2 did not start. Skipping trial $i." >&2
      TRIAL_OK=false
    fi
  fi

  # --- 2.5. Collision object (floor) ---
  if $TRIAL_OK; then
    log "[INFO] Loading floor collision from Rviz_floor.scene..."
    ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "{
      scene: {
        is_diff: true,
        world: {
          collision_objects: [{
            id: 'Box_0',
            header: {frame_id: 'world'},
            primitives: [{type: 1, dimensions: [4.0, 4.0, 0.02]}],
            primitive_poses: [{position: {x: 0.0, y: 0.0, z: -0.1}}],
            operation: 0
          }]
        }
      }
    }" > /dev/null 2>&1
    log "[INFO] Floor collision added."
  fi

  # --- 3. RTAB-Map ---
  if $TRIAL_OK; then
    log "[INFO] Launching RTAB-Map..."
    setsid ros2 launch ur_slam_bringup ur5e_slam.launch.py &
    GROUP_PIDS+=($!)
    log "[INFO] RTAB-Map PGID=$!"
    # mapData の publish は入力データ蓄積後なので検証スキップ
    wait_ready "RTAB-Map" "$SLAM_STARTUP_SEC"
  fi

  # --- 4. depth_image_noiser ---
  if $TRIAL_OK; then
    log "[INFO] Launching depth_image_noiser (NOISE=$NOISE)..."
    # shellcheck disable=SC2046
    setsid ros2 run ur_slam_tools depth_image_noiser.py --ros-args \
      -p use_sim_time:=true \
      $(noise_args) &
    GROUP_PIDS+=($!)
    log "[INFO] depth_image_noiser PGID=$!"
    wait_ready "depth_image_noiser" "$TOOL_STARTUP_SEC" || true
  fi

  # --- 5. cloud_gate ---
  if $TRIAL_OK; then
    log "[INFO] Launching cloud_gate..."
    setsid ros2 run ur_slam_tools cloud_gate.py --ros-args -p use_sim_time:=true &
    GROUP_PIDS+=($!)
    log "[INFO] cloud_gate PGID=$!"
    wait_ready "cloud_gate" "$TOOL_STARTUP_SEC" || true
  fi

  # --- 6. arc_sweep_jointtraj ---
  if $TRIAL_OK; then
    log "[INFO] Launching arc_sweep_jointtraj..."
    ros2 run ur_slam_tools arc_sweep_jointtraj.py --ros-args \
      -p use_sim_time:=true &
    ARC_PID=$!
    log "[INFO] arc_sweep PID=$ARC_PID — waiting for exit..."
    wait "$ARC_PID" 2>/dev/null; ARC_EXIT=$?
    log "[INFO] arc_sweep exited with code $ARC_EXIT"
    if [ "$ARC_EXIT" -ne 0 ]; then
      log "[ERROR] arc_sweep failed (exit=$ARC_EXIT). Marking trial as FAIL."
      TRIAL_OK=false
    fi
  fi

  # --- 7. coarse_to_fine_go_to_leaf5 ---
  if $TRIAL_OK; then
    log "[INFO] Launching coarse_to_fine_go_to_leaf5 (CONDITION=$CONDITION, run_id=$RUN_ID)..."
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
    log "[INFO] leaf5 PID=$LEAF_PID — waiting for exit..."
    wait "$LEAF_PID" 2>/dev/null; LEAF_EXIT=$?
    log "[INFO] leaf5 exited with code $LEAF_EXIT"
    if [ "$LEAF_EXIT" -ne 0 ]; then
      log "[ERROR] leaf5 failed (exit=$LEAF_EXIT). Marking trial as FAIL."
      TRIAL_OK=false
    fi
  fi

  # --- 試行終了 → cleanup して次へ ---
  TRIAL_DURATION=$((SECONDS - TRIAL_START))
  if ! $TRIAL_OK; then
    if [ "${ARC_EXIT:-0}" -ne 0 ] || [ "${LEAF_EXIT:-0}" -ne 0 ]; then
      STATUS="FAIL"; N_FAIL=$((N_FAIL + 1))
    else
      STATUS="SKIP"; N_SKIP=$((N_SKIP + 1))
    fi
  else
    STATUS="SUCCESS"; N_SUCCESS=$((N_SUCCESS + 1))
  fi
  log "[TRIAL $i/$N_TRIALS] DONE  run_id=$RUN_ID  status=$STATUS  duration=${TRIAL_DURATION}s"
  cleanup
done

log ""
log "========================================"
log " All $N_TRIALS trials complete."
log " SUCCESS=$N_SUCCESS  FAIL=$N_FAIL  SKIP=$N_SKIP"
log " CSV:  $RESULTS_DIR/leaf5_log.csv"
log " LOG:  $LOG_FILE"
log "========================================"

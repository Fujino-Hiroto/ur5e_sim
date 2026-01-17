#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

# --- ROS env ---
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

RUN_ID="$(date +%Y%m%d_%H%M%S)"
DB_PATH="$HOME/.ros/rtabmap_ur5e_${RUN_ID}.db"

echo "[INFO] RUN_ID=${RUN_ID}"
echo "[INFO] DB_PATH=${DB_PATH}"

# --- Start SLAM (background) ---
ros2 launch ur_slam_bringup ur5e_slam.launch.py with_gazebo:=false \
  db_path:="${DB_PATH}" delete_db_on_start:=true \
  > "runs_${RUN_ID}_slam.log" 2>&1 &
SLAM_PID=$!

# --- Wait for action server ---
echo "[INFO] Waiting for FollowJointTrajectory action server..."
for i in {1..60}; do
  if ros2 action list 2>/dev/null | grep -q "/joint_trajectory_controller/follow_joint_trajectory"; then
    break
  fi
  sleep 0.5
done

# --- Send trajectory (pan sweep ±45° around -90°) ---
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

echo "[INFO] Trajectory done. Stopping SLAM..."

# --- Stop SLAM ---
kill -INT "${SLAM_PID}" 2>/dev/null || true
wait "${SLAM_PID}" 2>/dev/null || true

echo "[OK] Finished. DB saved at: ${DB_PATH}"
echo "[OK] SLAM log: runs_${RUN_ID}_slam.log"

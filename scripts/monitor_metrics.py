#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_metrics.py (貼り替え版)

目的
- MoveIt の /check_state_validity で「衝突（または無効状態）」を監視し、CSVへ記録する
- TF から camera pose を取得し、leaf（PlanningScene上の collision object）の
  「距離」「視線角」「裏面側にいるか」を評価してCSVへ記録する

重要な修正点（あなたの環境向け）
1) TF Buffer を Node の clock（sim time）で動かす
   - Gazebo の use_sim_time=true では、tf2_ros.Buffer に node を渡さないと
     TF が「システム時間」で解釈され、lookup が不安定になることがあります。
2) leaf の pose を PlanningScene から取得するとき、co.header.frame_id を考慮し
   必ず args.world_frame（デフォルト base_link）へ TF 変換して保持する
   - これをしないと、leaf が world 基準で登録されている場合に評価が狂います。

使い方例
  ros2 run <your_pkg> monitor_metrics --csv trial.csv --leaf_id plant_leaf_0_3 \
    --world_frame base_link --camera_frame camera_color_optical_frame
"""

import math
import csv
import argparse
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetStateValidity, GetPlanningScene
from moveit_msgs.msg import RobotState, PlanningSceneComponents

import tf2_ros
from tf2_ros import TransformException


# ------------------------------
# Small math helpers
# ------------------------------
def quat_to_rot(qx, qy, qz, qw):
    """Rotation matrix from quaternion (x,y,z,w)."""
    xx = qx*qx; yy = qy*qy; zz = qz*qz
    xy = qx*qy; xz = qx*qz; yz = qy*qz
    wx = qw*qx; wy = qw*qy; wz = qw*qz

    r00 = 1 - 2*(yy+zz)
    r01 = 2*(xy - wz)
    r02 = 2*(xz + wy)

    r10 = 2*(xy + wz)
    r11 = 1 - 2*(xx+zz)
    r12 = 2*(yz - wx)

    r20 = 2*(xz - wy)
    r21 = 2*(yz + wx)
    r22 = 1 - 2*(xx+yy)
    return ((r00, r01, r02), (r10, r11, r12), (r20, r21, r22))


def rot_vec(R, v):
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )


def norm(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])


def unit(v):
    n = norm(v)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    return (v[0]/n, v[1]/n, v[2]/n)


def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


# ------------------------------
# Metrics monitor node
# ------------------------------
class MetricsMonitor(Node):
    def __init__(self, a):
        super().__init__("metrics_monitor")
        self.a = a

        # -------------------------
        # Joint states
        # -------------------------
        self.latest_js: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_js, qos_profile_sensor_data)

        # -------------------------
        # MoveIt services
        # -------------------------
        self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")

        if not self.valid_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Service /check_state_validity not available (move_group起動済み？)")
        if not self.scene_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Service /get_planning_scene not available (move_group起動済み？)")

        # -------------------------
        # TF (重要：sim time の clock を使う)
        # -------------------------
        # Gazebo/use_sim_time の場合、Buffer に node を渡して clock を共有するのが安全。
        # Humble で有効。
        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------------------------
        # Leaf pose cache (in args.world_frame)
        # -------------------------
        # leaf_pose: ((x,y,z), (qx,qy,qz,qw)) in a.world_frame
        self.leaf_pose = None

        # -------------------------
        # Counters
        # -------------------------
        self.in_collision = False
        self.collision_events = 0
        self.collision_time = 0.0

        self.success_events = 0
        self.success_hold = 0.0
        self.success_latched = False

        self.last_t = None

    def _on_js(self, msg: JointState):
        self.latest_js = msg

    # -------------------------
    # PlanningScene から leaf pose を一度だけ取得し、world_frameに揃える
    # -------------------------
    def fetch_leaf_pose_once(self):
        leaf_id = self.a.leaf_id
        if not leaf_id:
            return

        req = GetPlanningScene.Request()

        # request world object geometry/names（環境によりmaskが効かない場合もあるのでbest-effort）
        mask = 0
        for name in ["WORLD_OBJECT_GEOMETRY", "WORLD_OBJECT_NAMES"]:
            mask |= getattr(PlanningSceneComponents, name, 0)
        req.components.components = mask

        fut = self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            raise RuntimeError("get_planning_scene failed")

        found = False
        for co in res.scene.world.collision_objects:
            if co.id != leaf_id:
                continue

            if len(co.primitive_poses) == 0:
                raise RuntimeError(f"leaf_id={leaf_id} has no primitive_poses")

            # collision object pose is expressed in co.header.frame_id (may be 'world' etc.)
            src_frame = co.header.frame_id if co.header.frame_id else self.a.world_frame
            p = co.primitive_poses[0]

            leaf_ps = PoseStamped()
            leaf_ps.header.frame_id = src_frame
            leaf_ps.pose = p

            # Transform to args.world_frame (default: base_link)
            dst_frame = self.a.world_frame
            try:
                leaf_tf = self.tf_buffer.transform(
                    leaf_ps,
                    dst_frame,
                    timeout=Duration(seconds=1.0)
                )
            except TransformException as ex:
                raise RuntimeError(f"TF transform leaf pose failed: {src_frame} -> {dst_frame}: {ex}")

            pos = (leaf_tf.pose.position.x, leaf_tf.pose.position.y, leaf_tf.pose.position.z)
            quat = (leaf_tf.pose.orientation.x, leaf_tf.pose.orientation.y, leaf_tf.pose.orientation.z, leaf_tf.pose.orientation.w)
            self.leaf_pose = (pos, quat)

            found = True
            break

        if not found:
            raise RuntimeError(
                f"leaf_id '{leaf_id}' not found in planning scene. "
                f"RVizのScene GeometryでIDを確認してください。"
            )

        self.get_logger().info(
            f"Loaded leaf pose: id={leaf_id} (src_frame converted) -> {self.a.world_frame} pos={self.leaf_pose[0]}"
        )

    # -------------------------
    # TF: camera pose in world_frame
    # -------------------------
    def get_camera_world_tf(self) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        try:
            # lookup_transform(target, source) => target_T_source
            # i.e., transform from source into target
            tf = self.tf_buffer.lookup_transform(
                self.a.world_frame,
                self.a.camera_frame,
                rclpy.time.Time()
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            pos = (t.x, t.y, t.z)
            quat = (q.x, q.y, q.z, q.w)
            return pos, quat
        except TransformException:
            return None

    # -------------------------
    # MoveIt: collision check via /check_state_validity
    # -------------------------
    def check_collision(self) -> Tuple[bool, int, bool]:
        """returns: (in_collision, contact_count, valid_flag)"""
        if self.latest_js is None:
            return (False, 0, True)

        rs = RobotState()
        rs.joint_state = self.latest_js

        req = GetStateValidity.Request()
        req.robot_state = rs
        req.group_name = self.a.group

        fut = self.valid_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
        res = fut.result()
        if res is None:
            return (False, 0, True)

        # contacts が返る環境なら contacts を優先（より「衝突」に近い）
        contact_count = 0
        if hasattr(res, "contacts") and res.contacts is not None:
            contact_count = len(res.contacts)

        if contact_count > 0:
            return (True, contact_count, bool(getattr(res, "valid", True)))

        # contacts が無い場合のフォールバック
        valid_flag = bool(getattr(res, "valid", True))
        if not valid_flag:
            return (True, 0, valid_flag)

        return (False, 0, valid_flag)

    # -------------------------
    # View success criteria
    # -------------------------
    def check_view_success(self, cam_pos, cam_quat) -> Tuple[bool, float, float, bool]:
        """
        returns: (success_now, dist, look_angle_deg, underside_ok)
        * all computed in args.world_frame
        """
        if self.leaf_pose is None:
            return (False, float("nan"), float("nan"), False)

        (lx, ly, lz), (lqx, lqy, lqz, lqw) = self.leaf_pose
        cx, cy, cz = cam_pos
        (cqx, cqy, cqz, cqw) = cam_quat

        # distance (camera to leaf center)
        v_cl = (lx - cx, ly - cy, lz - cz)
        dist = norm(v_cl)
        dir_to_leaf = unit(v_cl)

        # camera forward in world: optical frame +Z
        Rc = quat_to_rot(cqx, cqy, cqz, cqw)
        cam_forward = unit(rot_vec(Rc, (0.0, 0.0, 1.0)))

        # look angle
        c = max(-1.0, min(1.0, dot(cam_forward, dir_to_leaf)))
        look_angle = math.degrees(math.acos(c))

        # leaf underside normal in world: leaf frame -Z を「裏向き法線」と定義
        Rl = quat_to_rot(lqx, lqy, lqz, lqw)
        underside_n = unit(rot_vec(Rl, (0.0, 0.0, -1.0)))

        # camera should be on underside side: dot(n, cam-leaf) > 0
        cam_minus_leaf = (cx - lx, cy - ly, cz - lz)
        underside_ok = (dot(underside_n, cam_minus_leaf) > 0.0)

        # thresholds
        dist_ok = (self.a.d_min <= dist <= self.a.d_max)
        ang_ok = (look_angle <= self.a.theta_deg)

        success_now = dist_ok and ang_ok and underside_ok
        return success_now, dist, look_angle, underside_ok

    # -------------------------
    # Main loop
    # -------------------------
    def run(self):
        # open csv
        with open(self.a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "t_sec",
                "in_collision", "contact_count", "collision_events", "collision_time_sec",
                "leaf_success_now", "success_events", "success_hold_sec",
                "dist_to_leaf_m", "look_angle_deg", "underside_ok",
            ])

            # preload leaf pose (needs TF ready to transform into world_frame)
            if self.a.leaf_id:
                self.fetch_leaf_pose_once()

            start_t = self.get_clock().now().nanoseconds / 1e9
            self.last_t = start_t

            rate = self.a.rate
            period = 1.0 / rate

            while rclpy.ok():
                # keep callbacks flowing (/joint_states, /tf, etc.)
                rclpy.spin_once(self, timeout_sec=0.01)

                now = self.get_clock().now().nanoseconds / 1e9
                dt = now - self.last_t
                if dt < period:
                    continue
                self.last_t = now

                # -------------------------
                # collision check
                # -------------------------
                in_col, contact_count, valid_flag = self.check_collision()

                # collision events/time
                if in_col and (not self.in_collision):
                    self.collision_events += 1
                if in_col:
                    self.collision_time += dt
                self.in_collision = in_col

                # -------------------------
                # view success
                # -------------------------
                cam_tf = self.get_camera_world_tf()
                if cam_tf is not None:
                    cam_pos, cam_quat = cam_tf
                    success_now, dist, ang, underside_ok = self.check_view_success(cam_pos, cam_quat)

                    # hold logic: need success for hold_sec continuously
                    if success_now:
                        self.success_hold += dt
                        if (not self.success_latched) and (self.success_hold >= self.a.hold_sec):
                            self.success_events += 1
                            self.success_latched = True
                    else:
                        self.success_hold = 0.0
                        self.success_latched = False
                else:
                    success_now, dist, ang, underside_ok = (False, float("nan"), float("nan"), False)
                    self.success_hold = 0.0
                    self.success_latched = False

                # write row
                w.writerow([
                    now - start_t,
                    int(self.in_collision), int(contact_count), int(self.collision_events), self.collision_time,
                    int(success_now), int(self.success_events), self.success_hold,
                    dist, ang, int(underside_ok),
                ])
                f.flush()

                # auto stop
                if self.a.duration > 0 and (now - start_t) >= self.a.duration:
                    break


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", default="trial_metrics.csv")
    p.add_argument("--rate", type=float, default=20.0, help="sampling rate (Hz)")
    p.add_argument("--duration", type=float, default=0.0, help="0=until Ctrl+C, else seconds")

    # moveit
    p.add_argument("--group", default="ur_manipulator")

    # TF frames
    p.add_argument("--world_frame", default="base_link")
    p.add_argument("--camera_frame", default="camera_color_optical_frame")

    # leaf success
    p.add_argument("--leaf_id", default="", help="e.g. plant_leaf_0_3")
    p.add_argument("--d_min", type=float, default=0.03)
    p.add_argument("--d_max", type=float, default=0.08)
    p.add_argument("--theta_deg", type=float, default=20.0, help="camera-forward vs leaf-center angle threshold")
    p.add_argument("--hold_sec", type=float, default=0.5, help="need to satisfy success for this duration to count as 1")

    args = p.parse_args()

    rclpy.init()
    node = MetricsMonitor(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

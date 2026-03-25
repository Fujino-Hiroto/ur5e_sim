#!/usr/bin/env python3
import math
import time
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from std_srvs.srv import SetBool




def _wrap_to_pi(x: float) -> float:
    # map to (-pi, pi]
    y = (x + math.pi) % (2.0 * math.pi) - math.pi
    return y


def _unwrap_like(ref: float, x: float) -> float:
    """
    Return x adjusted by +/-2pi so that it is closest to ref.
    Useful to avoid wrist joints spinning a full turn when crossing +/-pi.
    """
    # consider x + 2k*pi, choose k minimizing |(x+2k*pi)-ref|
    best = x
    best_err = abs(x - ref)
    for k in (-2, -1, 0, 1, 2):
        cand = x + 2.0 * math.pi * k
        err = abs(cand - ref)
        if err < best_err:
            best = cand
            best_err = err
    return best


class ArcSweepJointTraj(Node):
    """
    Replaced: This node no longer does TF / IK / arc generation.
    It simply sends joint waypoints: CENTER -> RIGHT -> CENTER -> LEFT -> CENTER
    via FollowJointTrajectory.
    """

    def __init__(self):
        super().__init__("arc_sweep_jointtraj")

        # Action name
        self.fjt_action = self.declare_parameter(
            "fjt_action", "/joint_trajectory_controller/follow_joint_trajectory"
        ).value

        # Timing
        self.time_per_segment = float(self.declare_parameter("time_per_segment", 6.0).value)
        self.hold_sec = float(self.declare_parameter("hold_sec", 0.5).value)

        # Use joint_states only as optional seed/reference for unwrap
        self.joint_states_topic = self.declare_parameter("joint_states_topic", "/joint_states").value
        self.use_unwrap = bool(self.declare_parameter("use_unwrap", True).value)

        # UR arm joints
        self.arm_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Waypoints (defaults: you can overwrite via ros-args -p ...)
        # CENTER
        self.center = self._declare_list6(
            "center",
            [-1.5707963268, -0.5235987756, -2.0943951024, -0.705551350, 1.5707963268, 0.0872664626]
        )
        # RIGHT
        self.right = self._declare_list6(
            "right",
            [-0.7853981634, -0.5235987756, -2.0943951024, -0.705551350, 1.5707963268, 0.0872664626]
        )
        # LEFT
        self.left = self._declare_list6(
            "left",
            [-2.3561944902, -0.5235987756, -2.0943951024, -0.705551350, 1.5707963268, 0.0872664626]
        )

        # Subscribers / action
        self.last_js: Optional[JointState] = None
        self.create_subscription(JointState, self.joint_states_topic, self._on_js, 10)
        self.action_cli = ActionClient(self, FollowJointTrajectory, self.fjt_action)

        self.gate_service = self.declare_parameter("gate_service", "/set_cloud_gate").value
        self.gate_cli = self.create_client(SetBool, self.gate_service)


    def _declare_list6(self, name: str, default: List[float]) -> List[float]:
        v = self.declare_parameter(name, default).value
        # rclpy sometimes returns tuple
        v = list(v)
        if len(v) != 6:
            raise RuntimeError(f"Parameter '{name}' must be length 6, got {len(v)}")
        return [float(x) for x in v]

    def _on_js(self, msg: JointState):
        self.last_js = msg

    def _js_map(self, js: JointState) -> Dict[str, float]:
        return {n: p for n, p in zip(js.name, js.position)}

    def wait_ready(self):
        # wait joint_states a bit (optional, only for unwrap reference)
        deadline = self.get_clock().now() + RclpyDuration(seconds=2.0)
        while (self.last_js is None) and (self.get_clock().now() < deadline):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.action_cli.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action server not available: {self.fjt_action}")
        
    def _set_gate(self, enabled: bool, timeout_sec: float = 2.0):
        if not self.gate_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Gate service not available: {self.gate_service}")
        req = SetBool.Request()
        req.data = bool(enabled)
        fut = self.gate_cli.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (time.time() - t0) > timeout_sec:
                raise RuntimeError("Timeout waiting for gate service response.")
        res = fut.result()
        if res is None or not res.success:
            raise RuntimeError(f"Failed to set gate: enabled={enabled} res={res}")
        self.get_logger().info(res.message)


    def _maybe_unwrap_waypoint(self, ref: List[float], wp: List[float]) -> List[float]:
        if not self.use_unwrap:
            return wp

        out = wp[:]
        # unwrap each joint to be closest to ref
        for i in range(6):
            out[i] = _unwrap_like(ref[i], out[i])
        return out

    def run(self):
        self.wait_ready()

        self._set_gate(True)
        try:


            # Build sequence
            seq_raw = [
                ("CENTER0", self.center),
                ("LEFT",    self.left),
                ("RIGHT",   self.right),
                ("CENTER2", self.center),
            ]

            # Reference for unwrap: current joints if available, else center
            ref = self.center[:]
            if self.last_js is not None:
                m = self._js_map(self.last_js)
                if all(j in m for j in self.arm_joints):
                    ref = [float(m[j]) for j in self.arm_joints]

            seq: List[List[float]] = []
            tags: List[str] = []
            cur = ref[:]
            for tag, wp in seq_raw:
                wp2 = self._maybe_unwrap_waypoint(cur, wp)
                seq.append(wp2)
                tags.append(tag)
                cur = wp2

            # Log
            self.get_logger().info("Waypoints (rad):")
            for tag, wp in zip(tags, seq):
                self.get_logger().info(f"  {tag}: " + ", ".join([f"{x:+.6f}" for x in wp]))

            # Build trajectory
            points: List[JointTrajectoryPoint] = []
            t = 0.0
            for wp in seq:
                pt = JointTrajectoryPoint()
                pt.positions = wp
                t += self.time_per_segment
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
                points.append(pt)

                if self.hold_sec > 1e-6:
                    pt2 = JointTrajectoryPoint()
                    pt2.positions = wp
                    t2 = t + self.hold_sec
                    pt2.time_from_start.sec = int(t2)
                    pt2.time_from_start.nanosec = int((t2 - int(t2)) * 1e9)
                    points.append(pt2)
                    t = t2

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(self.arm_joints)
            goal.trajectory.points = points

            self.get_logger().info(
                f"Sending trajectory: joints={len(goal.trajectory.joint_names)} "
                f"points={len(points)} duration~{t:.2f}s"
            )

            send_fut = self.action_cli.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_fut)
            gh = send_fut.result()
            if gh is None or not gh.accepted:
                raise RuntimeError("Trajectory goal was rejected.")

            res_fut = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut)
            self.get_logger().info("Done.")

        finally:
            try:
                self._set_gate(False)
            except Exception as e:
                self.get_logger().error(f"Error disabling gate in cleanup: {e}")
            


def main():
    rclpy.init()
    node = ArcSweepJointTraj()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random
import argparse

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def pose_xyz_rpy(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


class PlantSpawner(Node):
    def __init__(self, args):
        super().__init__("plant_collision_spawner")
        self.args = args
        self.cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Service /apply_planning_scene not available. Is move_group running?")

    def make_cylinder(self, obj_id, frame, x, y, z, height, radius, rpy=(0, 0, 0), op=CollisionObject.ADD):
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = frame
        co.operation = op
        if op == CollisionObject.REMOVE:
            return co

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [float(height), float(radius)]  # [height, radius]
        co.primitives = [prim]
        co.primitive_poses = [pose_xyz_rpy(x, y, z, *rpy)]
        return co

    def make_box(self, obj_id, frame, x, y, z, sx, sy, sz, rpy=(0, 0, 0), op=CollisionObject.ADD):
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = frame
        co.operation = op
        if op == CollisionObject.REMOVE:
            return co

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(sx), float(sy), float(sz)]
        co.primitives = [prim]
        co.primitive_poses = [pose_xyz_rpy(x, y, z, *rpy)]
        return co

    def _stem_positions(self, n):
        """Return list of (x, y, base_yaw)."""
        a = self.args
        if n <= 1:
            return [(a.x, a.y, 0.0)]
        if n == 2:
            return [
                (a.x, a.y, 0.0),
                (a.x + a.stem_dx, a.y + a.stem_dy, a.stem_yaw2),
            ]
        # n>=3: around a circle
        out = []
        R = max(0.12, a.cluster_radius)
        for i in range(n):
            ang = 2.0 * math.pi * i / n
            out.append((a.x + R * math.cos(ang), a.y + R * math.sin(ang), ang))
        return out

    def build_plant_objects(self):
        a = self.args
        rng = random.Random(a.seed)
        frame = a.frame

        objs_add = []
        ids = []

        stem_xy_yaw = self._stem_positions(a.stem_count)

        # stem heights with jitter
        heights = []
        for _ in range(a.stem_count):
            h = a.height * (1.0 + rng.uniform(-a.height_jitter, a.height_jitter))
            h = max(a.height_min, min(a.height_max, h))
            heights.append(h)

        self.get_logger().info(f"Stem heights: {['%.3f' % h for h in heights]} (seed={a.seed})")

        for i, (sx, sy, base_yaw) in enumerate(stem_xy_yaw):
            sh = heights[i]

            # center z for vertical cylinder
            stem_center_z = a.z0 + sh / 2.0

            # yaw jitter for natural look
            yaw = base_yaw + rng.uniform(-a.stem_yaw_jitter, a.stem_yaw_jitter)

            stem_id = f"{a.prefix}stem_{i}"
            ids.append(stem_id)
            objs_add.append(self.make_cylinder(
                stem_id, frame, sx, sy, stem_center_z,
                height=sh, radius=a.stem_radius, rpy=(0.0, 0.0, yaw)
            ))

            # z levels follow stem height so leaves don't float
            if a.levels <= 1:
                z_levels = [a.z0 + sh * 0.7]
            else:
                z_levels = [
                    a.z0 + sh * (a.z_start + (a.z_end - a.z_start) * (k / (a.levels - 1)))
                    for k in range(a.levels)
                ]

            for k, z in enumerate(z_levels):
                ang = (2.0 * math.pi * (k / max(1, a.levels))) + yaw + rng.uniform(-a.angle_jitter, a.angle_jitter)

                blen = a.branch_len * (1.0 + rng.uniform(-a.branch_len_jitter, a.branch_len_jitter))
                blen = max(0.05, blen)

                # branch center
                bx = sx + math.cos(ang) * (blen * 0.5)
                by = sy + math.sin(ang) * (blen * 0.5)

                branch_id = f"{a.prefix}branch_{i}_{k}"
                ids.append(branch_id)
                objs_add.append(self.make_cylinder(
                    branch_id, frame, bx, by, z,
                    height=blen, radius=a.branch_radius,
                    rpy=(0.0, math.pi / 2.0, ang)
                ))

                # leaf at branch end
                t = (k / max(1, a.levels - 1))
                scale = 1.0 - a.leaf_taper * t
                sx_leaf = a.leaf_sx * scale * (1.0 + rng.uniform(-a.leaf_size_jitter, a.leaf_size_jitter))
                sy_leaf = a.leaf_sy * scale * (1.0 + rng.uniform(-a.leaf_size_jitter, a.leaf_size_jitter))

                lx = sx + math.cos(ang) * (blen + a.leaf_offset)
                ly = sy + math.sin(ang) * (blen + a.leaf_offset)
                lz = z + a.leaf_z_offset

                leaf_roll = (a.leaf_roll * (1 if (k % 2 == 0) else -1)) + rng.uniform(-a.leaf_rpy_jitter, a.leaf_rpy_jitter)
                leaf_pitch = a.leaf_pitch + rng.uniform(-a.leaf_rpy_jitter, a.leaf_rpy_jitter)
                leaf_yaw = ang + rng.uniform(-a.leaf_rpy_jitter, a.leaf_rpy_jitter)

                leaf_id = f"{a.prefix}leaf_{i}_{k}"
                ids.append(leaf_id)
                objs_add.append(self.make_box(
                    leaf_id, frame, lx, ly, lz,
                    sx_leaf, sy_leaf, a.leaf_sz,
                    rpy=(leaf_roll, leaf_pitch, leaf_yaw)
                ))

        # replace: remove same IDs first
        objs = []
        if a.replace:
            for oid in ids:
                co = CollisionObject()
                co.id = oid
                co.header.frame_id = frame
                co.operation = CollisionObject.REMOVE
                objs.append(co)

        objs.extend(objs_add)
        return objs

    def apply(self, collision_objects):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects

        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

        if fut.result() is None:
            raise RuntimeError("Failed to call /apply_planning_scene")
        if not fut.result().success:
            raise RuntimeError("apply_planning_scene returned success=False")

        self.get_logger().info(f"Applied {len(collision_objects)} scene ops to PlanningScene.")


def main():
    p = argparse.ArgumentParser()

    # base frame / placement
    p.add_argument("--frame", default="world")
    p.add_argument("--x", type=float, default=1.2)
    p.add_argument("--y", type=float, default=0.0)
    p.add_argument("--z0", type=float, default=0.0)

    # stem placement
    p.add_argument("--stem_count", type=int, default=2)
    p.add_argument("--stem_dx", type=float, default=0.20)
    p.add_argument("--stem_dy", type=float, default=0.10)
    p.add_argument("--stem_yaw2", type=float, default=0.15)
    p.add_argument("--cluster_radius", type=float, default=0.22)

    # stem height randomness
    p.add_argument("--height", type=float, default=1.5)
    p.add_argument("--height_jitter", type=float, default=0.25, help="± ratio, e.g. 0.25 -> 1.5*(0.75..1.25)")
    p.add_argument("--height_min", type=float, default=0.9)
    p.add_argument("--height_max", type=float, default=1.7)
    p.add_argument("--seed", type=int, default=1)

    # NEW: stem yaw jitter (fix for your error)
    p.add_argument("--stem_yaw_jitter", type=float, default=0.20, help="Yaw jitter (rad) per stem for natural look")

    # radii
    p.add_argument("--stem_radius", type=float, default=0.025)
    p.add_argument("--branch_radius", type=float, default=0.012)

    # density
    p.add_argument("--levels", type=int, default=8)
    p.add_argument("--branch_len", type=float, default=0.55)
    p.add_argument("--branch_len_jitter", type=float, default=0.15)
    p.add_argument("--angle_jitter", type=float, default=0.35)

    # leaf size / pose
    p.add_argument("--leaf_sx", type=float, default=0.30)
    p.add_argument("--leaf_sy", type=float, default=0.18)
    p.add_argument("--leaf_sz", type=float, default=0.01)
    p.add_argument("--leaf_offset", type=float, default=0.05)
    p.add_argument("--leaf_z_offset", type=float, default=0.02)
    p.add_argument("--leaf_pitch", type=float, default=0.25)
    p.add_argument("--leaf_roll", type=float, default=0.15)
    p.add_argument("--leaf_rpy_jitter", type=float, default=0.20)
    p.add_argument("--leaf_size_jitter", type=float, default=0.10)
    p.add_argument("--leaf_taper", type=float, default=0.25)

    # leaf z distribution on stem (ratio)
    p.add_argument("--z_start", type=float, default=0.25)
    p.add_argument("--z_end", type=float, default=0.95)

    # ids
    p.add_argument("--replace", action="store_true")
    p.add_argument("--prefix", default="plant_")

    args = p.parse_args()

    rclpy.init()
    node = PlantSpawner(args)
    objs = node.build_plant_objects()
    node.apply(objs)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

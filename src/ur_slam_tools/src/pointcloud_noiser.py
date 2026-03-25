#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudNoiser(Node):
    """
    Subscribe:  /camera/depth/points (PointCloud2)
    Publish:    /camera/depth/points_noisy (PointCloud2)

    Noise model (configurable):
      - dropout (NaN) with optional blocky dropout
      - depth-dependent gaussian noise: z += N(0, (a + b z^2)^2)
      - spikes: a fraction of points get z multiplied by factor < 1 (fake near returns)
      - temporal flicker: scales dropout and sigma over time
    """

    def __init__(self):
        super().__init__('pointcloud_noiser')

        # Topics
        self.declare_parameter('in_topic', '/camera/depth/points')
        self.declare_parameter('out_topic', '/camera/depth/points_noisy')

        # Reproducibility
        self.declare_parameter('seed', 42)

        # Base noise knobs
        self.declare_parameter('dropout_p', 0.02)         # per-point dropout probability
        self.declare_parameter('block_dropout', True)     # blocky dropout in image plane (organized clouds)
        self.declare_parameter('block_count', 8)          # number of blocks per frame
        self.declare_parameter('block_radius_px', 6)      # block radius (px) for NaN holes (organized only)

        self.declare_parameter('gauss_a', 0.002)          # meters
        self.declare_parameter('gauss_b', 0.001)          # meters / m^2  (sigma = a + b*z^2)

        self.declare_parameter('spike_p', 0.005)          # fraction of points to spike
        self.declare_parameter('spike_min_factor', 0.3)   # z *= factor (fake closer)
        self.declare_parameter('spike_max_factor', 0.8)

        # Temporal flicker (optional)
        self.declare_parameter('enable_flicker', False)
        self.declare_parameter('flicker_hz', 1.0)
        self.declare_parameter('flicker_amp', 0.5)        # scales dropout/sigma by (1 + amp*sin)
        self.declare_parameter('frame_drop_p', 0.0)       # drop entire frame (publish nothing)

        # Limits / gating
        self.declare_parameter('min_z', 0.0)
        self.declare_parameter('max_z', 5.0)              # keep points beyond max_z as-is or NaN? (we NaN)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self.rng = np.random.default_rng(int(self.get_parameter('seed').value))

        # Input: match Gazebo publisher (often RELIABLE)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Output: match typical perception consumers (often BEST_EFFORT)
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, sub_qos)


        self.get_logger().info(f"PointCloudNoiser: {in_topic} -> {out_topic}")

    def _flicker_scale(self, stamp_sec: float):
        if not self.get_parameter('enable_flicker').value:
            return 1.0
        hz = float(self.get_parameter('flicker_hz').value)
        amp = float(self.get_parameter('flicker_amp').value)
        return 1.0 + amp * math.sin(2.0 * math.pi * hz * stamp_sec)

    def cb(self, msg: PointCloud2):
        # Optional frame drop
        frame_drop_p = float(self.get_parameter('frame_drop_p').value)
        if frame_drop_p > 0.0 and self.rng.random() < frame_drop_p:
            return

        stamp_sec = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        flick = self._flicker_scale(stamp_sec)

        dropout_p = float(self.get_parameter('dropout_p').value) * flick
        dropout_p = max(0.0, min(0.95, dropout_p))

        gauss_a = float(self.get_parameter('gauss_a').value) * flick
        gauss_b = float(self.get_parameter('gauss_b').value) * flick

        spike_p = float(self.get_parameter('spike_p').value)
        spike_min = float(self.get_parameter('spike_min_factor').value)
        spike_max = float(self.get_parameter('spike_max_factor').value)

        min_z = float(self.get_parameter('min_z').value)
        max_z = float(self.get_parameter('max_z').value)

        # Read points (x,y,z) as numpy array; keep organized shape if possible
        # sensor_msgs_py.point_cloud2 returns generator; we convert to ndarray
        # NOTE: This copies; for high-rate huge clouds you can optimize later.
        # Read points (x,y,z) into numpy
        pts = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)))

        if pts.size == 0:
            self.pub.publish(msg)
            return

        # Handle structured vs non-structured
        if pts.dtype.fields is not None:
            x = pts['x'].astype(np.float32, copy=False)
            y = pts['y'].astype(np.float32, copy=False)
            z = pts['z'].astype(np.float32, copy=False)
        else:
            pts = pts.astype(np.float32, copy=False)
            # shape should be (N,3)
            x = pts[:, 0]
            y = pts[:, 1]
            z = pts[:, 2]

        # Determine organized dims if present
        w = int(msg.width)
        h = int(msg.height)
        organized = (w > 0 and h > 0 and w * h == x.shape[0])



        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > min_z) & (z <= max_z)

        # A) Dropout per-point -> NaN
        if dropout_p > 0.0:
            drop_mask = (self.rng.random(z.shape[0]) < dropout_p) & valid
            x[drop_mask] = np.nan
            y[drop_mask] = np.nan
            z[drop_mask] = np.nan
            valid = valid & (~drop_mask)

        # A2) Blocky dropout (organized only)
        if organized and self.get_parameter('block_dropout').value:
            block_count = int(self.get_parameter('block_count').value)
            rad = int(self.get_parameter('block_radius_px').value)
            if block_count > 0 and rad > 0:
                # reshape to image for block operations
                x2 = x.reshape((h, w))
                y2 = y.reshape((h, w))
                z2 = z.reshape((h, w))
                valid2 = valid.reshape((h, w))

                for _ in range(block_count):
                    cy = self.rng.integers(0, h)
                    cx = self.rng.integers(0, w)
                    y0 = max(0, cy - rad)
                    y1 = min(h, cy + rad + 1)
                    x0 = max(0, cx - rad)
                    x1 = min(w, cx + rad + 1)
                    # apply only where currently valid
                    m = valid2[y0:y1, x0:x1]
                    x2[y0:y1, x0:x1][m] = np.nan
                    y2[y0:y1, x0:x1][m] = np.nan
                    z2[y0:y1, x0:x1][m] = np.nan

                # flatten back
                x[:] = x2.reshape((-1,))
                y[:] = y2.reshape((-1,))
                z[:] = z2.reshape((-1,))
                valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > min_z) & (z <= max_z)

        # B) Depth-dependent gaussian noise on z (and optionally x,y if you want)
        if (gauss_a > 0.0) or (gauss_b > 0.0):
            sigma = gauss_a + gauss_b * (z[valid] ** 2)
            dz = self.rng.normal(loc=0.0, scale=sigma).astype(np.float32)
            z[valid] = z[valid] + dz

            # After noise, invalidate outside range
            bad = (z <= min_z) | (z > max_z) | (~np.isfinite(z))
            x[bad] = np.nan
            y[bad] = np.nan
            z[bad] = np.nan
            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > min_z) & (z <= max_z)

        if (gauss_a > 0.0) or (gauss_b > 0.0):
            z_old = z[valid].copy()
            sigma = gauss_a + gauss_b * (z_old ** 2)
            dz = self.rng.normal(loc=0.0, scale=sigma).astype(np.float32)
            z_new = z_old + dz

            # レイ方向に乗せる：比率で x,y も更新
            scale = z_new / np.maximum(z_old, 1e-6)
            x[valid] *= scale
            y[valid] *= scale
            z[valid] = z_new


        # Rebuild PointCloud2 (x,y,z only). Keep header/frame/stamp.
        noisy_points = np.stack([x, y, z], axis=1).astype(np.float32)

        out = pc2.create_cloud_xyz32(msg.header, noisy_points.tolist())
        out.height = msg.height
        out.width = msg.width
        out.is_dense = False  # NaN included
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PointCloudNoiser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

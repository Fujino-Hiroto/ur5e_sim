#!/usr/bin/env python3
import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthImageNoiser(Node):
    """
    Subscribe:  /camera/depth/depth/image_raw   (sensor_msgs/Image, 32FC1)
    Publish:    /camera/depth/depth/image_raw_noisy (sensor_msgs/Image, 32FC1)

    Noise model (typical RGB-D artifacts):
      - Per-pixel dropout -> invalid depth (NaN or 0)
      - Block dropout holes (organized depth)
      - Depth-dependent gaussian: z += N(0, (a + b*z^2)^2)
      - Edge flying pixels: near depth discontinuities, z scaled to smaller (false near returns)
      - Temporal flicker: scales dropout/sigma over time
    """

    def __init__(self):
        super().__init__('depth_image_noiser')
        self.bridge = CvBridge()

        # Topics
        self.declare_parameter('in_topic', '/camera/depth/depth/image_raw')
        self.declare_parameter('out_topic', '/camera/depth/depth/image_raw_noisy')

        # Reproducibility
        self.declare_parameter('seed', 42)

        # QoS (avoid mismatches): default to RELIABLE for both sub/pub
        # reliability: "reliable" or "best_effort"
        self.declare_parameter('sub_reliability', 'reliable')
        self.declare_parameter('pub_reliability', 'reliable')
        self.declare_parameter('qos_depth', 5)

        # Invalid representation (for 32FC1, NaN is common)
        self.declare_parameter('invalid_is_nan', True)  # False -> set invalid to 0.0

        # Range gating
        self.declare_parameter('min_z', 0.1)   # meters
        self.declare_parameter('max_z', 5.0)   # meters

        # Dropout
        self.declare_parameter('dropout_p', 0.02)
        self.declare_parameter('block_dropout', True)
        self.declare_parameter('block_count', 8)
        self.declare_parameter('block_radius_px', 6)

        # Gaussian (depth-dependent)
        self.declare_parameter('gauss_a', 0.002)  # meters
        self.declare_parameter('gauss_b', 0.001)  # meters / m^2 (sigma = a + b*z^2)

        # Edge flying pixels (more realistic than uniform spikes)
        self.declare_parameter('enable_edge_spikes', True)
        self.declare_parameter('edge_grad_thr', 0.10)     # meters (depth discontinuity threshold)
        self.declare_parameter('edge_spike_p', 0.25)      # probability applied only on edge pixels
        self.declare_parameter('edge_spike_min_factor', 0.3)  # z *= factor (closer)
        self.declare_parameter('edge_spike_max_factor', 0.8)

        # Temporal flicker
        self.declare_parameter('enable_flicker', False)
        self.declare_parameter('flicker_hz', 1.0)
        self.declare_parameter('flicker_amp', 0.5)  # scales dropout/sigma by (1 + amp*sin)
        self.declare_parameter('frame_drop_p', 0.0) # drop entire frame (publish nothing)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self.rng = np.random.default_rng(int(self.get_parameter('seed').value))

        def _rel(s: str):
            s = str(s).strip().lower()
            if s in ('reliable', 'rel'):
                return QoSReliabilityPolicy.RELIABLE
            if s in ('best_effort', 'besteffort', 'be'):
                return QoSReliabilityPolicy.BEST_EFFORT
            # fallback
            return QoSReliabilityPolicy.RELIABLE

        depth = int(self.get_parameter('qos_depth').value)
        sub_rel = _rel(self.get_parameter('sub_reliability').value)
        pub_rel = _rel(self.get_parameter('pub_reliability').value)

        sub_qos = QoSProfile(
            reliability=sub_rel,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
        )
        pub_qos = QoSProfile(
            reliability=pub_rel,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
        )

        self.pub = self.create_publisher(Image, out_topic, pub_qos)
        self.sub = self.create_subscription(Image, in_topic, self.cb, sub_qos)

        self.get_logger().info(
            f"DepthImageNoiser: {in_topic} -> {out_topic} "
            f"(seed={int(self.get_parameter('seed').value)}, "
            f"sub_rel={self.get_parameter('sub_reliability').value}, "
            f"pub_rel={self.get_parameter('pub_reliability').value}, "
            f"depth={depth})"
        )

    def _flicker_scale(self, stamp_sec: float) -> float:
        if not self.get_parameter('enable_flicker').value:
            return 1.0
        hz = float(self.get_parameter('flicker_hz').value)
        amp = float(self.get_parameter('flicker_amp').value)
        return 1.0 + amp * math.sin(2.0 * math.pi * hz * stamp_sec)

    def _set_invalid(self, z: np.ndarray, mask: np.ndarray):
        if self.get_parameter('invalid_is_nan').value:
            z[mask] = np.nan
        else:
            z[mask] = 0.0

    def cb(self, msg: Image):
        # Frame drop
        frame_drop_p = float(self.get_parameter('frame_drop_p').value)
        if frame_drop_p > 0.0 and self.rng.random() < frame_drop_p:
            return

        # Sanity: we expect 32FC1
        if msg.encoding != '32FC1':
            self.get_logger().warn(f"Unexpected encoding {msg.encoding}, expected 32FC1. Publishing as-is.")
            self.pub.publish(msg)
            return

        stamp_sec = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        flick = self._flicker_scale(stamp_sec)

        dropout_p = float(self.get_parameter('dropout_p').value) * flick
        dropout_p = max(0.0, min(0.95, dropout_p))

        gauss_a = float(self.get_parameter('gauss_a').value) * flick
        gauss_b = float(self.get_parameter('gauss_b').value) * flick

        min_z = float(self.get_parameter('min_z').value)
        max_z = float(self.get_parameter('max_z').value)

        # Convert to numpy float32 image (HxW)
        z = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        z = np.asarray(z, dtype=np.float32)

        h, w = z.shape[:2]

        # valid mask (consider NaN invalid)
        valid = np.isfinite(z) & (z > min_z) & (z <= max_z)

        # A) per-pixel dropout
        if dropout_p > 0.0:
            drop = (self.rng.random((h, w)) < dropout_p) & valid
            self._set_invalid(z, drop)
            valid &= ~drop

        # A2) block dropout holes
        if self.get_parameter('block_dropout').value:
            block_count = int(self.get_parameter('block_count').value)
            rad = int(self.get_parameter('block_radius_px').value)
            if block_count > 0 and rad > 0:
                for _ in range(block_count):
                    cy = int(self.rng.integers(0, h))
                    cx = int(self.rng.integers(0, w))
                    y0 = max(0, cy - rad)
                    y1 = min(h, cy + rad + 1)
                    x0 = max(0, cx - rad)
                    x1 = min(w, cx + rad + 1)
                    m = valid[y0:y1, x0:x1]
                    if np.any(m):
                        self._set_invalid(z[y0:y1, x0:x1], m)
                        valid[y0:y1, x0:x1] &= ~m

        # B) depth-dependent gaussian noise (only on valid)
        if (gauss_a > 0.0) or (gauss_b > 0.0):
            vv = valid
            z_old = z[vv]
            sigma = gauss_a + gauss_b * (z_old ** 2)
            dz = self.rng.normal(loc=0.0, scale=sigma).astype(np.float32)
            z_new = z_old + dz
            z[vv] = z_new

            # range gate again
            bad = (~np.isfinite(z)) | (z <= min_z) | (z > max_z)
            self._set_invalid(z, bad)
            valid = np.isfinite(z) & (z > min_z) & (z <= max_z)

        # C) Edge flying pixels (depth discontinuities)
        if self.get_parameter('enable_edge_spikes').value:
            thr = float(self.get_parameter('edge_grad_thr').value)
            p = float(self.get_parameter('edge_spike_p').value)
            fmin = float(self.get_parameter('edge_spike_min_factor').value)
            fmax = float(self.get_parameter('edge_spike_max_factor').value)

            # Compute simple gradient on valid depths (ignore invalid by treating as 0)
            zz = np.where(np.isfinite(z), z, 0.0).astype(np.float32)
            gx = np.abs(zz[:, 1:] - zz[:, :-1])
            gy = np.abs(zz[1:, :] - zz[:-1, :])

            edge = np.zeros((h, w), dtype=bool)
            edge[:, 1:] |= gx > thr
            edge[:, :-1] |= gx > thr
            edge[1:, :] |= gy > thr
            edge[:-1, :] |= gy > thr

            edge &= valid  # only where valid

            if p > 0.0:
                fire = (self.rng.random((h, w)) < p) & edge
                if np.any(fire):
                    factors = self.rng.uniform(fmin, fmax, size=np.count_nonzero(fire)).astype(np.float32)
                    z[fire] = z[fire] * factors

                    # range gate
                    bad = (~np.isfinite(z)) | (z <= min_z) | (z > max_z)
                    self._set_invalid(z, bad)
                    valid = np.isfinite(z) & (z > min_z) & (z <= max_z)

        # Rebuild Image msg: keep header EXACTLY (stamp sync safety)
        out = self.bridge.cv2_to_imgmsg(z, encoding='32FC1')
        out.header = msg.header  # stamp/frame_id preserved
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthImageNoiser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

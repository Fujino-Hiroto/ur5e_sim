#!/usr/bin/env python3
import os
import csv
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import PointCloud2


def _get_field_offset(msg: PointCloud2, name: str):
    for f in msg.fields:
        if f.name == name:
            return int(f.offset)
    return None


def _xyz_frombuffer(msg: PointCloud2):
    """
    Fast, zero-copy-ish extractor for x,y,z using frombuffer with correct offsets.
    Assumes x/y/z are float32 (common for Gazebo/RealSense style clouds).
    """
    npts = int(msg.width) * int(msg.height)
    if npts <= 0 or msg.point_step <= 0 or len(msg.data) < npts * msg.point_step:
        return None, None, None, 0

    off_x = _get_field_offset(msg, "x")
    off_y = _get_field_offset(msg, "y")
    off_z = _get_field_offset(msg, "z")
    if off_x is None or off_y is None or off_z is None:
        return None, None, None, 0

    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [off_x, off_y, off_z],
            "itemsize": int(msg.point_step),
        }
    )

    arr = np.frombuffer(msg.data, dtype=dtype, count=npts)
    x = arr["x"].astype(np.float32, copy=False)
    y = arr["y"].astype(np.float32, copy=False)
    z = arr["z"].astype(np.float32, copy=False)
    return x, y, z, npts


class PointCloudMetricsCSV(Node):
    """
    Subscribe:  in_topic (PointCloud2)
    Output:     csv_path (append)

    Records per message:
      - stamp_sec, recv_sec, age_sec
      - n_points, n_valid, n_roi, min_z_roi
      - ratios
    ROI logic matches ForwardObstacleMonitor:
      finite(x,y,z) AND (z > min_z) AND (z <= max_z) AND (|x|<=roi_x) AND (|y|<=roi_y)
    """

    def __init__(self):
        super().__init__("pointcloud_metrics_csv")

        # Topics / file
        self.declare_parameter("in_topic", "/camera/depth/points_noisy")
        self.declare_parameter("csv_path", "/tmp/pointcloud_metrics.csv")
        self.declare_parameter("append", True)

        # ROI / gating (match your monitor knobs)
        self.declare_parameter("roi_x_m", 0.12)
        self.declare_parameter("roi_y_m", 0.12)
        self.declare_parameter("min_z", 0.0)
        self.declare_parameter("max_z", 1.20)

        # Optional downsample logging
        self.declare_parameter("log_every_n", 1)  # 1 = every message

        # QoS: match your pipeline (Gazebo publisher often RELIABLE)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.in_topic = self.get_parameter("in_topic").value
        self.csv_path = self.get_parameter("csv_path").value
        self.append = bool(self.get_parameter("append").value)

        self.roi_x = float(self.get_parameter("roi_x_m").value)
        self.roi_y = float(self.get_parameter("roi_y_m").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)

        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))
        self.msg_count = 0

        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

        file_exists = os.path.exists(self.csv_path)
        mode = "a" if self.append else "w"
        self.f = open(self.csv_path, mode, newline="")
        self.writer = csv.writer(self.f)

        self.header = [
            "idx",
            "frame_id",
            "stamp_sec",
            "recv_sec",
            "age_sec",
            "width",
            "height",
            "n_points",
            "n_valid",
            "valid_ratio",
            "n_roi",
            "roi_ratio_of_all",
            "roi_ratio_of_valid",
            "min_z_roi",
            "roi_x_m",
            "roi_y_m",
            "min_z",
            "max_z",
        ]

        if (not file_exists) or (not self.append):
            self.writer.writerow(self.header)
            self.f.flush()

        self.sub = self.create_subscription(PointCloud2, self.in_topic, self.cb, sub_qos)
        self.get_logger().info(f"PointCloudMetricsCSV: in={self.in_topic} csv={self.csv_path}")

    def cb(self, msg: PointCloud2):
        self.msg_count += 1
        if (self.msg_count % self.log_every_n) != 0:
            return

        # Times
        stamp_sec = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        recv_sec = self.get_clock().now().nanoseconds * 1e-9
        age_sec = recv_sec - stamp_sec

        # Extract xyz
        x, y, z, npts = _xyz_frombuffer(msg)
        if npts <= 0 or x is None:
            return

        w = int(msg.width)
        h = int(msg.height)

        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        # Valid range (match monitor-ish)
        in_range = (z > self.min_z) & (z <= self.max_z)
        valid = finite & in_range

        n_valid = int(np.count_nonzero(valid))
        valid_ratio = (n_valid / npts) if npts > 0 else 0.0

        # ROI
        roi = valid & (np.abs(x) <= self.roi_x) & (np.abs(y) <= self.roi_y)
        n_roi = int(np.count_nonzero(roi))
        roi_ratio_all = (n_roi / npts) if npts > 0 else 0.0
        roi_ratio_valid = (n_roi / n_valid) if n_valid > 0 else 0.0

        if n_roi > 0:
            min_z_roi = float(np.min(z[roi]))
        else:
            min_z_roi = math.inf

        row = [
            self.msg_count,
            msg.header.frame_id,
            f"{stamp_sec:.9f}",
            f"{recv_sec:.9f}",
            f"{age_sec:.6f}",
            w,
            h,
            npts,
            n_valid,
            f"{valid_ratio:.6f}",
            n_roi,
            f"{roi_ratio_all:.6f}",
            f"{roi_ratio_valid:.6f}",
            f"{min_z_roi:.6f}" if math.isfinite(min_z_roi) else "inf",
            f"{self.roi_x:.3f}",
            f"{self.roi_y:.3f}",
            f"{self.min_z:.3f}",
            f"{self.max_z:.3f}",
        ]

        self.writer.writerow(row)
        self.f.flush()

    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PointCloudMetricsCSV()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

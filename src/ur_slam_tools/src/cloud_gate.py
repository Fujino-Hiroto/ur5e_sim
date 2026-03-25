#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool


class CloudGate(Node):
    def __init__(self):
        super().__init__("cloud_gate")

        self.input_topic  = self.declare_parameter("input_topic",  "/cloud_obstacles").value
        self.output_topic = self.declare_parameter("output_topic", "/planning_cloud").value
        self.enabled      = bool(self.declare_parameter("enabled", False).value)

        # Subscriber QoS: rtabmap publishes RELIABLE/TRANSIENT_LOCAL.
        # VOLATILE subscriber is compatible; RELIABLE keeps it stable.
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publisher QoS: MoveIt側が受け取りやすいよう、RELIABLE/VOLATILEで出す
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, sub_qos)

        self.srv = self.create_service(SetBool, "set_cloud_gate", self.on_set_gate)

        self.get_logger().info(
            f"CloudGate ready: input={self.input_topic} output={self.output_topic} enabled={self.enabled}"
        )

    def on_set_gate(self, req: SetBool.Request, res: SetBool.Response):
        self.enabled = bool(req.data)
        res.success = True
        res.message = f"cloud_gate enabled={self.enabled}"
        self.get_logger().info(res.message)
        return res

    def on_cloud(self, msg: PointCloud2):
        if not self.enabled:
            return
        # pass-through (frame_id, stamp, fields all unchanged)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CloudGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

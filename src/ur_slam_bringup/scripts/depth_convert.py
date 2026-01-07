#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthConverter(Node):
    def __init__(self):
        super().__init__('depth_converter')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/camera/depth/image_converted', 10)
        self.get_logger().info('DepthConverter node started')

    def callback(self, msg):
        try:
            depth_float = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            depth_uint16 = np.clip(depth_float * 1000.0, 0, 65535).astype(np.uint16)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_uint16, encoding="16UC1")
            depth_msg.header = msg.header
            self.publisher_.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

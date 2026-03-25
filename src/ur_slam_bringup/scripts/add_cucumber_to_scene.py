#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh
from shape_msgs.msg import MeshTriangle
from geometric_shapes import shape_operations  # from moveit_ros_planning
import os

class AddCucumber(Node):
    def __init__(self):
        super().__init__("add_cucumber_to_scene")
        self.pub = self.create_publisher(CollisionObject, "collision_object", 10)

        # Change this if you want a different frame
        self.frame_id = "world"

        # Path to mesh inside your workspace
        ws = os.path.expanduser("~/workspaces/ur_slam_ws")
        self.mesh_path = os.path.join(
            ws, "src/ur_slam_bringup/models/cucumber/meshes/cucumber_branch2.dae"
        )

        # Pose: this should match your Gazebo include pose (x,y,z) for now
        # You said: <pose>0.6 0.0 0.0 0 0 0</pose> in world, and link pose handles grounding
        self.pose = PoseStamped()
        self.pose.header.frame_id = self.frame_id
        self.pose.pose.position.x = 0.6
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.w = 1.0

        # Scale: match model.sdf mesh scale
        self.scale = (0.3, 0.3, 0.3)

        self.timer = self.create_timer(1.0, self.once)
        self.sent = False

    def once(self):
        if self.sent:
            return

        if not os.path.exists(self.mesh_path):
            self.get_logger().error(f"Mesh not found: {self.mesh_path}")
            return

        self.pose.header.stamp = self.get_clock().now().to_msg()

        # Load mesh using moveit geometric_shapes
        try:
            mesh_msg = shape_operations.createMeshFromResource(
                "file://" + self.mesh_path, list(self.scale)
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load mesh: {e}")
            return

        co = CollisionObject()
        co.header.frame_id = self.frame_id
        co.id = "cucumber_mesh"
        co.meshes = [mesh_msg]
        co.mesh_poses = [self.pose.pose]
        co.operation = CollisionObject.ADD

        self.pub.publish(co)
        self.get_logger().info("Published cucumber_mesh collision object to PlanningScene")
        self.sent = True

def main():
    rclpy.init()
    node = AddCucumber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <thread>
#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go_to_leaf");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // TF
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // MoveIt
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPoseReferenceFrame("world");

  // TF取得（world -> leaf_target_view）
  geometry_msgs::msg::TransformStamped T;
  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    try {
      T = tf_buffer.lookupTransform("world", "leaf_target_view", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node->get_logger(), "Waiting TF world->leaf_target_view: %s", ex.what());
      rate.sleep();
    }
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.pose.position.x = T.transform.translation.x;
  goal.pose.position.y = T.transform.translation.y;
  goal.pose.position.z = T.transform.translation.z;
  goal.pose.orientation = T.transform.rotation;

  // 計画直前に現在状態へ更新
  move_group.setStartStateToCurrentState();

  // カメラEEで解く（ここが肝）
  move_group.setPoseTarget(goal, "camera_color_optical_frame");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
    exec.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Executing...");
  RCLCPP_INFO(node->get_logger(), "EE link used for target: %s", move_group.getEndEffectorLink().c_str());

  move_group.execute(plan);

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}

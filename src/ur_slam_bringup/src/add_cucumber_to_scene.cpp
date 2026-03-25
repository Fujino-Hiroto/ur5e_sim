// 中島さんのきゅうりをcollission objectとしてMoveItのシーンに追加するものだったがおもすぎて断念
#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <geometric_shapes/shape_operations.h>   // shapes::constructMsgFromShape
#include <geometric_shapes/shapes.h>             // shapes::Mesh, shapes::createMeshFromResource

#include <boost/variant/get.hpp>

class AddCucumberToScene : public rclcpp::Node
{
public:
  AddCucumberToScene() : Node("add_cucumber_to_scene")
  {
    pub_ = this->create_publisher<moveit_msgs::msg::CollisionObject>("collision_object", 10);

    // ---- parameters ----
    this->declare_parameter<std::string>("frame_id", "base_link");  // planning_frame
    this->declare_parameter<std::string>("id", "cucumber_mesh");

    // NOTE: file:// の絶対パスで渡す（package:// でもいいけどまずは確実な方）
    this->declare_parameter<std::string>(
      "mesh_resource",
      "file:///home/fujino/workspaces/ur_slam_ws/src/ur_slam_bringup/models/cucumber/meshes/cucumber_branch2.dae"
    );

    this->declare_parameter<double>("scale", 0.3);

    // Pose（base_link基準）
    this->declare_parameter<double>("x", 0.6);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("z", 0.3);
    this->declare_parameter<double>("qx", 0.0);
    this->declare_parameter<double>("qy", 0.0);
    this->declare_parameter<double>("qz", 0.0);
    this->declare_parameter<double>("qw", 1.0);

    // 何回か投げる（起動順でMoveItがまだ購読してない時があるので保険）
    this->declare_parameter<int>("repeat", 5);
    this->declare_parameter<double>("period_sec", 1.0);

    // timer
    repeat_ = this->get_parameter("repeat").as_int();
    period_sec_ = this->get_parameter("period_sec").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_sec_),
      std::bind(&AddCucumberToScene::onTimer, this)
    );

    RCLCPP_INFO(get_logger(), "AddCucumberToScene node started.");
  }

private:
  void onTimer()
  {
    if (sent_ >= repeat_) {
      RCLCPP_INFO(get_logger(), "Done publishing (%d times).", sent_);
      rclcpp::shutdown();
      return;
    }

    const auto frame_id = this->get_parameter("frame_id").as_string();
    const auto id = this->get_parameter("id").as_string();
    const auto mesh_resource = this->get_parameter("mesh_resource").as_string();
    const double s = this->get_parameter("scale").as_double();

    // ---- load mesh via geometric_shapes ----
    std::unique_ptr<shapes::Mesh> mesh(
      shapes::createMeshFromResource(mesh_resource, Eigen::Vector3d(s, s, s))
    );

    if (!mesh) {
      RCLCPP_ERROR(get_logger(), "Failed to load mesh from: %s", mesh_resource.c_str());
      return;
    }

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh.get(), shape_msg);

    shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    geometry_msgs::msg::Pose pose;
    pose.position.x = this->get_parameter("x").as_double();
    pose.position.y = this->get_parameter("y").as_double();
    pose.position.z = this->get_parameter("z").as_double();
    pose.orientation.x = this->get_parameter("qx").as_double();
    pose.orientation.y = this->get_parameter("qy").as_double();
    pose.orientation.z = this->get_parameter("qz").as_double();
    pose.orientation.w = this->get_parameter("qw").as_double();

    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = frame_id;
    co.id = id;
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    co.meshes.push_back(mesh_msg);
    co.mesh_poses.push_back(pose);

    pub_->publish(co);
    sent_++;

    RCLCPP_INFO(get_logger(), "Published CollisionObject '%s' in frame '%s' (%d/%d)",
                id.c_str(), frame_id.c_str(), sent_, repeat_);
  }

  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int repeat_{5};
  int sent_{0};
  double period_sec_{1.0};
};
 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddCucumberToScene>());
  rclcpp::shutdown();
  return 0;
}

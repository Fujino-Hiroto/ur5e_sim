cmake_minimum_required(VERSION 3.8)
project(ur_slam_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)  # ★ PointCloud2 を使うなら必須

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ----------------------------
# Executables
# ----------------------------
add_executable(go_to_leaf src/go_to_leaf.cpp)
ament_target_dependencies(go_to_leaf
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs moveit_ros_planning_interface
)

add_executable(coarse_to_fine_go_to_leaf src/coarse_to_fine_go_to_leaf.cpp)
ament_target_dependencies(coarse_to_fine_go_to_leaf
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs moveit_ros_planning_interface
)

# ★ 2 / 3 は「別ターゲット名」にする
add_executable(coarse_to_fine_go_to_leaf2 src/coarse_to_fine_go_to_leaf2.cpp)
ament_target_dependencies(coarse_to_fine_go_to_leaf2
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs moveit_ros_planning_interface sensor_msgs
)

add_executable(coarse_to_fine_go_to_leaf3 src/coarse_to_fine_go_to_leaf3.cpp)
ament_target_dependencies(coarse_to_fine_go_to_leaf3
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs moveit_ros_planning_interface sensor_msgs
)

add_executable(coarse_to_fine_go_to_leaf4 src/coarse_to_fine_go_to_leaf4.cpp)
ament_target_dependencies(coarse_to_fine_go_to_leaf4
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs moveit_ros_planning_interface sensor_msgs
)

add_executable(coarse_to_fine_go_to_leaf5 src/coarse_to_fine_go_to_leaf5.cpp)
ament_target_dependencies(coarse_to_fine_go_to_leaf5
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs moveit_ros_planning_interface sensor_msgs
)

# ----------------------------
# Install
# ----------------------------
install(TARGETS
  go_to_leaf
  coarse_to_fine_go_to_leaf
  coarse_to_fine_go_to_leaf2
  coarse_to_fine_go_to_leaf3
  coarse_to_fine_go_to_leaf4
  coarse_to_fine_go_to_leaf5

  DESTINATION lib/${PROJECT_NAME}
)

# Install python executable scripts
install(PROGRAMS
  src/pointcloud_noiser.py
  src/pointcloud_metrics_csv.py
  src/depth_image_noiser.py
  src/arc_sweep_jointtraj.py
  src/cloud_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



ament_package()

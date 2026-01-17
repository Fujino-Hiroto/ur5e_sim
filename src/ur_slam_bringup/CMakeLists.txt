cmake_minimum_required(VERSION 3.8)
project(ur_slam_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(add_cucumber_to_scene src/add_cucumber_to_scene.cpp)

target_link_libraries(add_cucumber_to_scene Eigen3::Eigen)

ament_target_dependencies(add_cucumber_to_scene
  rclcpp
  moveit_msgs
  shape_msgs
  geometry_msgs
  geometric_shapes
)

install(TARGETS add_cucumber_to_scene
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## Install ##

# launch ファイルをインストール
install(
  DIRECTORY launch urdf worlds scripts config
  DESTINATION share/${PROJECT_NAME}
)


# Python ノードをインストール（これは既に書いてあるやつ）
install(
  PROGRAMS
    scripts/depth_convert.py
    scripts/add_cucumber_to_scene.py
  DESTINATION lib/${PROJECT_NAME}
)





ament_package()

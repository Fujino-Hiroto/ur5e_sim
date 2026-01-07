from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 共通引数
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # ==== Launch Arguments ====
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("ur_slam_bringup"),
                "config",
                "slam_toolbox_params.yaml",  # 当時作っていたパラメータファイル名
            ]
        ),
        description="Full path to the ROS2 parameters file for slam_toolbox",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("ur_slam_bringup"),
                "rviz",
                "ur5e_slam.rviz",  # SLAM 用 RViz 設定
            ]
        ),
        description="Full path to the RVIZ config file",
    )

    # ==== UR5e + Gazebo(カメラ付き) ====
    ur5e_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_slam_bringup"),
                    "launch",
                    "ur5e_gazebo.launch.py",
                ]
            )
        ),
        # 必要なら引数も渡せる
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "false",  # こちらの RViz を使うので
        }.items(),
    )

    # ==== SLAM ノード（例：slam_toolbox）====
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",  # or async_slam_toolbox_node
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    # ==== RViz2 ====
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_slam_params,
            declare_rviz_config,
            ur5e_gazebo_launch,
            slam_node,
            rviz_node,
        ]
    )

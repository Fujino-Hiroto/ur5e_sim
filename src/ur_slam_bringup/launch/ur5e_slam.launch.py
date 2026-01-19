#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------
    # Launch arguments
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_gazebo = LaunchConfiguration("with_gazebo")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world = LaunchConfiguration("world")

    # Camera topics (Gazebo Classic plugin outputs)
    rgb_topic = LaunchConfiguration("rgb_topic")
    rgb_info_topic = LaunchConfiguration("rgb_info_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")

    # Frames
    odom_frame_id = LaunchConfiguration("odom_frame_id")   # TFからオドメトリを取る親
    base_frame_id = LaunchConfiguration("base_frame_id")   # “オドメトリ対象”の子（= ここではカメラ自身）
    map_frame_id = LaunchConfiguration("map_frame_id")

    # RTAB-Map database
    db_path = LaunchConfiguration("db_path")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")

    # QoS for sensor topics in ROS2
    qos = LaunchConfiguration("qos")
    qos_camera_info = LaunchConfiguration("qos_camera_info")

    # RViz / rtabmap_viz
    launch_rtabmap_viz = LaunchConfiguration("launch_rtabmap_viz")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("with_gazebo", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_slam_bringup"), "worlds", "white_room.world"]
            ),
        ),
        # Topics
        DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("rgb_info_topic", default_value="/camera/color/camera_info"),
        # Gazebo depth camera: 32FC1 は通常こちら
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/depth/image_raw"),
        DeclareLaunchArgument("depth_info_topic", default_value="/camera/depth/depth/camera_info"),
        # Frames
        # odom_frame_id を “空にしない” → TFからオドメトリ取得 :contentReference[oaicite:1]{index=1}
        DeclareLaunchArgument("odom_frame_id", default_value="world"),
        DeclareLaunchArgument("base_frame_id", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("map_frame_id", default_value="map"),
        # DB
        DeclareLaunchArgument(
            "db_path",
            default_value=os.path.join(os.path.expanduser("~"), ".ros", "rtabmap_ur5e.db"),
        ),
        DeclareLaunchArgument("delete_db_on_start", default_value="true"),
        # QoS
        DeclareLaunchArgument("qos", default_value="2"),
        DeclareLaunchArgument("qos_camera_info", default_value="2"),
        # Viz
        DeclareLaunchArgument("launch_rtabmap_viz", default_value="false"),
    ]

    # -------------------------
    # Gazebo bringup (optional)
    # -------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_slam_bringup"), "launch", "ur5e_gazebo.launch.py"]
            )
        ),
        condition=IfCondition(with_gazebo),
        launch_arguments={
            "world": world,
            "gazebo_gui": gazebo_gui,
            # MoveIt側RVizを使うことが多いので、ここでは起動しない想定
            "launch_rviz": "false",
        }.items(),
    )

    # -------------------------
    # RTAB-Map pipeline
    #   rgbd_sync -> rtabmap (TF/FK odom)
    #
    # rgbd_sync は rtabmap_sync パッケージの rgbd_sync を使う構成が一般的 :contentReference[oaicite:2]{index=2}
    # subscribe_rgbd と subscribe_depth の併用は避ける（警告が出る）:contentReference[oaicite:3]{index=3}
    # -------------------------
    rgbd_sync_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "approx_sync": True,
            "approx_sync_max_interval": 0.01,
            "sync_queue_size": 30,
            "topic_queue_size": 10,
            "qos": qos,
            "qos_camera_info": qos_camera_info,
        }],
        remappings=[
            ("rgb/image", rgb_topic),
            ("rgb/camera_info", rgb_info_topic),
            ("depth/image", depth_topic),
            ("depth/camera_info", depth_info_topic),
        ],
    )


    # rtabmap 本体：odom_frame_id を設定して TFからオドメトリ取得 :contentReference[oaicite:4]{index=4}
    # 3D占有（OctoMap系トピック）を出すための代表的パラメータ :contentReference[oaicite:5]{index=5}
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "database_path": db_path,

            # TF odom (TFからオドメトリ取得)
            "frame_id": base_frame_id,
            "odom_frame_id": odom_frame_id,   # default=world
            "map_frame_id": "world",          # まずは world 推奨

            # Input: RGBDImage
            "subscribe_rgbd": True,
            "subscribe_odom": False,
            "approx_sync": True,
            "sync_queue_size": 30,
            "qos": qos,
            "qos_camera_info": qos_camera_info,

            # TF publish（FK(TF)を壊さない）
            "publish_tf": False,
            "publish_tf_map": False,

            # ---- RTAB-Map内部パラメータ（文字列）----
            "RGBD/CreateOccupancyGrid": "true",
            "Grid/3D": "true",
            "Grid/FromDepth": "true",
            "Grid/Sensor": "1",
            "Grid/RayTracing": "true",
            "Grid/RangeMax": "5.0",

            # 軽め
            "Rtabmap/DetectionRate": "2.0",
        }],
        remappings=[
            ("rgbd_image", "rgbd_image"),
        ],
    )


    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        condition=IfCondition(launch_rtabmap_viz),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "frame_id": base_frame_id,
                "odom_frame_id": odom_frame_id,
                "map_frame_id": map_frame_id,
                "subscribe_rgbd": True,
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "subscribe_odom": False,
                "qos": qos,
                "qos_camera_info": qos_camera_info,
            }
        ],
        remappings=[
            ("rgbd_image", "rgbd_image"),
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_launch,
            rgbd_sync_node,
            rtabmap_node,
            rtabmap_viz_node,
        ]
    )

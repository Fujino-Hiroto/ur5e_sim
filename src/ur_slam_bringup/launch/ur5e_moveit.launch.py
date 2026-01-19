from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")

    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_gazebo = LaunchConfiguration("launch_gazebo")

    # (Optional) Launch Gazebo + spawn UR5e (your bringup)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_slam_bringup"), "/launch", "/ur5e_gazebo.launch.py"]
        ),
        condition=IfCondition(launch_gazebo),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "launch_rviz": "false",
            "gazebo_gui": "true",
        }.items(),
    )

    # MoveIt2 (upstream)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": launch_rviz,
            "use_fake_hardware": "false",

        }.items(),
    )

    return [gazebo_launch, ur_moveit_launch]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_gazebo",
            default_value="false",
            description="Also launch Gazebo + spawn robot (use true if you want one-shot launch).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR type",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enable safety limits controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_simulation_gazebo",
            description="Controller config package",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="Controllers YAML file",
        )
    )

    # IMPORTANT: use your custom description (with camera)
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_slam_bringup",
            description="Description package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur5e_with_camera.urdf.xacro",
            description="URDF/Xacro file (with camera)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/Xacro file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Joint name prefix",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 for MoveIt",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

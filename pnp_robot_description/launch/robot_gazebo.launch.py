#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    pkg_name = "pnp_robot_description"
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Paths to important files
    urdf_file = os.path.join(pkg_share, "urdf", "pnp_robot.urdf")
    world_file = os.path.join(pkg_share, "worlds", "pnp_world.sdf")
    rviz_config = os.path.join(pkg_share, "rviz", "view_robot.rviz")
    bridge_config = os.path.join(pkg_share, "config", "gazebo_bridge_config.yaml")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    world = LaunchConfiguration("world", default=world_file)

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world", default_value=world_file, description="Full path to world file to load"
    )

    # Read URDF file
    with open(urdf_file, "r") as infp:
        robot_description = infp.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={"gz_args": world}.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        parameters=[{"robot_description": robot_description}],
        arguments=[
            "-param",
            "robot_description",
            "-name",
            "pnp_robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.5",
        ],
    )

    # ROS-Gazebo Bridge using RosGzBridge action with YAML config
    ros_gz_bridge = RosGzBridge(
        bridge_name="ros_gz_bridge",
        config_file=bridge_config,
        bridge_params=[{"use_sim_time": use_sim_time}],
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Camera image viewer (optional)
    image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="camera_viewer",
        arguments=["/camera/image_raw"],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_world_cmd,
            robot_state_publisher,
            ros_gz_bridge,
            gazebo,
            spawn_robot,
            rviz,
            # image_view,  # Uncomment if you want automatic camera viewer
        ]
    )

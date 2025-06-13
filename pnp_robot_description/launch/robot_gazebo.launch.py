#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_name = "pnp_robot_description"  # Replace with your actual package name
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Paths to important files
    urdf_file = os.path.join(pkg_share, "urdf", "pnp_robot.urdf")
    world_file = os.path.join(pkg_share, "worlds", "pnp_world.sdf")
    rviz_config = os.path.join(pkg_share, "rviz", "view_robot.rviz")

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

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
    )

    # Joint State Publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
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
        launch_arguments={"gz_args": world_file}.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "default",
            "-param",
            "robot_description",
            "-name",
            "my_robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.5",
        ],
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
            joint_state_publisher_gui,
            static_tf_node,
            # gazebo,
            # spawn_robot,
            rviz,
            # image_view,  # Uncomment if you want automatic camera viewer
        ]
    )

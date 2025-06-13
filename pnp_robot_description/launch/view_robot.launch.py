#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "pnp_robot_description"
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Paths to important files
    urdf_file = os.path.join(pkg_share, "urdf", "pnp_robot.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "view_robot.rviz")

    # Read URDF file
    with open(urdf_file, "r") as infp:
        robot_description = infp.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
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
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
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
            robot_state_publisher,
            joint_state_publisher_gui,
            static_tf_node,
            rviz,
            # image_view,  # Uncomment if you want automatic camera viewer
        ]
    )

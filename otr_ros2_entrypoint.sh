#!/bin/bash

. "/opt/ros/${ROS_DISTRO}/setup.bash"

cd /workspace
ls -la "src/${ROBOT_PACKAGE}"
colcon build
. install/local_setup.bash

package_name=$(sed -n 's/.*<name>\(.*\)<\/name>.*/\1/p' "src/${ROBOT_PACKAGE}/package.xml")
ros2 launch "${package_name}" robot_gazebo.launch.py

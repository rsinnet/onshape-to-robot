ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-perception
SHELL ["/bin/bash", "-c"]
RUN apt-get update
RUN apt-get install -y ros-${ROS_DISTRO}-{rviz2,joint-state-publisher-gui,robot-state-publisher,tf2-ros,xacro}
RUN mkdir -p /workspace/src

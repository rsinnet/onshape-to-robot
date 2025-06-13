ARG GZ_DISTRO=ionic
ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-perception
SHELL ["/bin/bash", "-c"]

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

RUN apt-get update
RUN apt-get install -y ros-${ROS_DISTRO}-{rviz2,joint-state-publisher,joint-state-publisher-gui,robot-state-publisher,tf2-ros,xacro}

ARG GZ_DISTRO
RUN apt-get install -y gz-${GZ_DISTRO} ros-${ROS_DISTRO}-ros-gz
RUN mkdir -p /workspace/src

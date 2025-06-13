# PnP Robot Gazebo Simulation

This package contains a robot with Z-axis (vertical), XY-axes (planar), and theta (rotation) degrees of freedom, equipped with a camera for simulation in Gazebo with ROS2.

## Package Structure

```text
pnp_robot/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── robot_gazebo.launch.py
├── urdf/
│   └── pnp_robot.urdf
├── worlds/
│   └── robot_world.world
├── rviz/
│   └── robot_view.rviz
└── README.md
```

## Prerequisites

Make sure you have the following installed:

- ROS2 (Humble, Iron, or Rolling)
- Gazebo (Garden or Ignition)
- Required ROS2 packages:

  ```bash
  sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
  sudo apt install ros-$ROS_DISTRO-robot-state-publisher
  sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
  sudo apt install ros-$ROS_DISTRO-rviz2
  sudo apt install ros-$ROS_DISTRO-rqt-image-view
  ```

## Running the Simulation

1. **Launch the robot in Gazebo**:

   ```bash
   ros2 launch pnp_robot robot_gazebo.launch.py
   ```

   This will start:

   - Gazebo with the robot world
   - RViz for visualization
   - Joint State Publisher GUI for manual control
   - Robot State Publisher

2. **Control the robot**:

   - Use the Joint State Publisher GUI window to move the robot joints
   - Z-joint: Vertical motion (0 to 1.0 meters)
   - X-joint: Horizontal motion (-0.5 to 0.5 meters)
   - Y-joint: Horizontal motion (-0.5 to 0.5 meters)
   - Theta-joint: Rotation (-π to π radians)

3. **View camera feed**:

   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```

   Then select `/camera/image_raw` topic

## Available Topics

- `/joint_states` - Current joint positions
- `/robot_description` - Robot URDF description
- `/camera/image_raw` - Camera image data
- `/camera/camera_info` - Camera calibration info
- `/tf` - Transform tree

## Troubleshooting

1. **Gazebo won't start**:

   - Make sure you've sourced your ROS2 workspace: `source ~/ros2_ws/install/setup.bash`
   - Check if Gazebo is properly installed: `gazebo --version`

2. **Robot doesn't appear**:

   - Check the terminal for error messages
   - Verify all file paths in the launch file are correct

3. **Camera not working**:

   - Check if the camera topic is being published: `ros2 topic list | grep camera`
   - Verify the camera plugin is loaded in Gazebo

4. **RViz not showing robot**:
   - Make sure the robot_description topic is being published
   - Check the Fixed Frame in RViz (should be `base_link`)

## Customization

- **Modify robot dimensions**: Edit values in `urdf/pnp_robot.urdf`
- **Change joint limits**: Modify the `<limit>` tags in the URDF
- **Add more objects**: Edit `worlds/robot_world.world`
- **Adjust camera settings**: Modify the camera plugin parameters in the URDF

## Camera Specifications

- Resolution: 800x600
- Frame rate: 30 FPS
- Horizontal FOV: 80 degrees
- Topics: `/camera/image_raw`, `/camera/camera_info`

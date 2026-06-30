# BW Description

This package contains the URDF/Xacro model and controller configuration for the BW robot description.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to bw_description --symlink-install
```

## Visualize the robot

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bw_description visualize.launch.py
```

## Launch ROS2 Control

This simplified tree keeps OCS2 and RL controllers. The old guide-controller and chained leg-PD examples were removed.

### Mujoco Simulator or Real Robot

For real robots using the SDK2 hardware interface, configure `xacro/ros2_control.xacro`, disable any official locomotion controller before testing, and test in simulation first.

* OCS2 Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=bw_description
  ```
* RL Quadruped Controller
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch rl_quadruped_controller mujoco.launch.py pkg_description:=bw_description
  ```

### Joystick command input

Start the kept joystick input path in another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch joystick_input joystick.launch.py
```

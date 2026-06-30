# Joystick Input

This package converts `sensor_msgs/msg/Joy` from the standard ROS `joy` node into `control_input_msgs/msg/Inputs` on `/control_input`.

Tested environment:

* Ubuntu 24.04
  * ROS2 Jazzy
  * Logitech F310 Gamepad

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to joystick_input --symlink-install
```

## Launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch joystick_input joystick.launch.py
```

The launch file starts:

* `joy/joy_node`, reading `/dev/input/js0`
* `joystick_input`, publishing `/control_input`

## Input mapping

The node has two input modes selected by axis 7.

### Velocity input mode

When axis 7 is not `1`, the node publishes velocity-style inputs:

* `lx = -axes[0]`
* `ly = axes[1]`
* `rx = -axes[3]`
* `ry = axes[2]`
* `command = 0`

### Command selection mode

When axis 7 is `1`, the node clears `lx`, `ly`, `rx`, and `ry`, then selects `command` from axes 4, 5, and 6:

| axes[4] | axes[5] | axes[6] | command |
| --- | --- | --- | --- |
| -1 | -1 | -1 | 1 |
| 0 | -1 | -1 | 2 |
| 1 | -1 | -1 | 3 |
| -1 | 0 | -1 | 4 |
| 0 | 0 | -1 | 5 |
| 1 | 0 | -1 | 6 |
| -1 | 1 | -1 | 7 |
| 0 | 1 | -1 | 8 |
| -1 | -1 | 0 | 9 |

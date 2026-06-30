# Control Command Inputs

This directory contains the retained command input path used by the controllers.

* `control_input_msgs`: defines `control_input_msgs/msg/Inputs`.
* `joystick_input`: converts `sensor_msgs/msg/Joy` from the standard ROS `joy` node into `/control_input`.

## Message

`control_input_msgs/msg/Inputs` contains:

```text
int32 command
float32 lx
float32 ly
float32 rx
float32 ry
```

The controllers use `command` for mode or gait selection and the four float fields for joystick-style motion inputs.

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

See [joystick_input](joystick_input/) for the joystick axis mapping.

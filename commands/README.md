# Control Command Inputs

This simplified tree keeps the standard joystick command path:

* `control_input_msgs`: shared `/control_input` message definition.
* `joystick_input`: converts `sensor_msgs/msg/Joy` from the ROS `joy` node into `control_input_msgs/msg/Inputs`.

Launch joystick input with:

```bash
ros2 launch joystick_input joystick.launch.py
```

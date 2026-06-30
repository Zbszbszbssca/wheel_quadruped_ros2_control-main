# Hardware Unitree SDK2

This package provides the `ros2_control` hardware interface for Unitree SDK2 transport. It is used for real Unitree robots and for the Unitree Mujoco simulator.

For Mujoco simulation, use [unitree_mujoco](https://github.com/legubiao/unitree_mujoco). This fork expects the Unitree/Mujoco path to provide joint, IMU, and foot-force state interfaces.

* [x] **[2025-01-16]** Add odometer states for simulation.

## 1. Interfaces

Required hardware interfaces:

* command:
  * joint position
  * joint velocity
  * joint effort
  * KP
  * KD
* state:
  * joint effort
  * joint position
  * joint velocity
  * imu sensor
    * linear acceleration
    * angular velocity
    * orientation
  * foot force sensor

## 2. Build

Tested environment:

* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble

```bash
cd ~/ros2_ws
colcon build --packages-up-to hardware_unitree_sdk2 --symlink-install
```

## 3. Configure network and domain

Set the Unitree DDS domain and network interface in the robot description xacro before running against a real robot or the Unitree Mujoco transport:

```xml
<hardware>
  <plugin>hardware_unitree_sdk2/HardwareUnitree</plugin>
  <param name="domain">1</param>
  <param name="network_interface">lo</param>
</hardware>
```

For Mujoco on localhost, `lo` is commonly used. For a real robot, set `network_interface` to the network device connected to the robot.

## 4. Visualize hardware state

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hardware_unitree_sdk2 visualize.launch.py
```

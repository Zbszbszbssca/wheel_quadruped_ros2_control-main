# Robot Descriptions

This folder contains the URDF/Xacro files and controller configs for quadruped robots.

* My Dog
  * [BW](my_dog/bw_description/)

## 1. Steps to transfer URDF to Mujoco model

* Install [Mujoco](https://github.com/google-deepmind/mujoco).
* Transfer mesh files to a Mujoco-supported format, such as STL.
* Adjust the URDF file to match mesh scale after conversion.
* Use `xacro` to generate the URDF file:
  ```bash
  xacro robot.xacro > ../urdf/robot.urdf
  ```
* Use Mujoco to convert the URDF file to a Mujoco model:
  ```bash
  compile robot.urdf robot.xml
  ```

## 2. Dependencies for Gazebo Simulation

Gazebo Harmonic simulation is tested on ROS2 Jazzy.

* Gazebo Harmonic
  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```
* ros2-control for Gazebo
  ```bash
  sudo apt-get install ros-jazzy-gz-ros2-control
  ```

The simplified tree uses the kept Gazebo hardware interface and OCS2/RL controllers.

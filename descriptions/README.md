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

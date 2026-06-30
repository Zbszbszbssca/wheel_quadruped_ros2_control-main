# OCS2 Quadruped Controller

This is a ros2-control controller based on [legged_control](https://github.com/qiayuanl/legged_control)
and [ocs2_ros2](https://github.com/Zbszbszbssca/ocs2_ros2).

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy


* [x] **[2026-03-16]** Add support for wheeled robot.


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
    * feet force sensor

## 2. Build

### 2.1 Build Dependencies
Before install OCS2 ROS2, please follow the guide to install [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html). **you can use sudo apt install ros-jazzy-pinocchio**!，

**After installed Pinocchio**, follow below step to clone ocs2 ros2 library to src folder.

```bash
cd ~/ros2_ws/src
git clone https://github.com/Zbszbszbssca/ocs2_ros2

cd ocs2_ros2
git submodule update --init --recursive

cd ..
rosdep install --from-paths src --ignore-src -r -y
```

### 2.2 Build OCS2 Quadruped Controller

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_quadruped_controller  --symlink-install
```

## 3. Launch

supported robot description in this simplified tree:

* My Dog
    * bw_description

### 3.1 About OCS2 Shared Library
OCS2 Quadruped controller depends on the OCS2 library, it required c++ automatic differentiation shared library. When first launch the controller, it will compile the OCS2 model and generate the shared library. 

You may see something similar to:
```
[controller_manager] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_libcppadcg_tmp-27918274.so
[controller_manager] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_libcppadcg_tmp-27918274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_lib.so
[controller_manager] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_libcppadcg_tmp-94918274.so
[controller_manager] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_libcppadcg_tmp-94918274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_lib.so
[controller_manager] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_libcppadcg_tmp-83618274.so
[controller_manager] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_libcppadcg_tmp-83618274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_lib.so
```
The compilation process may take a few minutes. After the compilation, restart the controller and the robot should stand up.

To config the path for the cppAD shared library, you can modify the `modelFolderCppAd` item in `task.info` file, which located at the `config/ocs2` folder under robot description package. If the path is not start with `/`, it will be considered as **relative path to the Linux Home folder**.

### 3.2 Usage
#### Control Input State Switch
The controller listens to `/control_input`. With the kept joystick input path, command values select controller modes/gaits:
* command 1: Passive Mode
* command 2: OCS2 MPC Mode / stance
* command 3: trot
* command 4: standing_trot
* command 5: flying_trot

### 3.3 Launch Controller
#### Mujoco Simulation
> **Warm Reminder**: You need to launch [Unitree Mujoco C++ Simulation](https://github.com/legubiao/unitree_mujoco) before launch the controller.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=bw_description
```

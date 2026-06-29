//
// Created by biao on 24-9-12.
//

#include <iostream>
#include "controller_common/CtrlInterfaces.h"
#include "rl_quadruped_controller/robot/QuadrupedRobot.h"

// 构造函数，初始化四足机器人，解析机器人描述并构建机器人的各个部件
QuadrupedRobot::QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                               const std::vector<std::string> &feet_names,
                               const std::string &base_name) : ctrl_interfaces_(ctrl_interfaces) {
    KDL::Tree robot_tree;
    // 使用机器人描述字符串构建KDL树
    kdl_parser::treeFromString(robot_description, robot_tree);

    // 获取四条腿的关节链
    robot_tree.getChain(base_name, feet_names[0], fl_chain_);
    robot_tree.getChain(base_name, feet_names[1], fr_chain_);
    robot_tree.getChain(base_name, feet_names[2], rl_chain_);
    robot_tree.getChain(base_name, feet_names[3], rr_chain_);

    // 为四条腿创建RobotLeg对象
    robot_legs_.resize(4);
    robot_legs_[0] = std::make_shared<RobotLeg>(fl_chain_);
    robot_legs_[1] = std::make_shared<RobotLeg>(fr_chain_);
    robot_legs_[2] = std::make_shared<RobotLeg>(rl_chain_);
    robot_legs_[3] = std::make_shared<RobotLeg>(rr_chain_);

    // 初始化关节位置和速度
    current_joint_pos_.resize(4);
    current_joint_vel_.resize(4);

    std::cout << "robot_legs_.size(): " << robot_legs_.size() << std::endl;

    // 计算机器人的总质量
    mass_ = 0;
    for (const auto &[fst, snd]: robot_tree.getSegments()) {
        mass_ += snd.segment.getInertia().getMass();
    }

    // 初始化四条腿的标准站立位置
    feet_pos_normal_stand_ << 0.1881, 0.1881, -0.1881, -0.1881, -0.1300, 0.1300,
            -0.1300, 0.1300, -0.3200, -0.3200, -0.3200, -0.3200;
}

// 根据末端执行器位置列表计算目标关节角度
std::vector<KDL::JntArray> QuadrupedRobot::getQ(const std::vector<KDL::Frame> &pEe_list) const {
    std::vector<KDL::JntArray> result;
    result.resize(4);
    // 为每条腿计算对应的关节角度
    for (int i(0); i < 4; ++i) {
        result[i] = robot_legs_[i]->calcQ(pEe_list[i], current_joint_pos_[i]);
    }
    return result;
}

// 根据末端执行器的位置（Vec34）计算目标关节角度（Vec12）
Vec12 QuadrupedRobot::getQ(const Vec34 &vecP) const {
    Vec12 q;
    // 对每条腿，使用位置向量来计算关节角度
    for (int i(0); i < 4; ++i) {
        KDL::Frame frame;
        frame.p = KDL::Vector(vecP.col(i)[0], vecP.col(i)[1], vecP.col(i)[2]);
        frame.M = KDL::Rotation::Identity();
        q.segment(3 * i, 3) = robot_legs_[i]->calcQ(frame, current_joint_pos_[i]).data;
    }
    return q;
}

// 根据末端执行器的位置和速度计算目标关节速度
Vec12 QuadrupedRobot::getQd(const std::vector<KDL::Frame> &pos, const Vec34 &vel) {
    Vec12 qd;
    // 先根据位置计算关节角度
    const std::vector<KDL::JntArray> q = getQ(pos);
    // 对每条腿，通过雅可比矩阵计算关节速度
    for (int i(0); i < 4; ++i) {
        Mat3 jacobian = robot_legs_[i]->calcJaco(q[i]).data.topRows(3);
        qd.segment(3 * i, 3) = jacobian.inverse() * vel.col(i);
    }
    return qd;
}

// 获取每条腿的末端执行器相对于基坐标系的位姿
std::vector<KDL::Frame> QuadrupedRobot::getFeet2BPositions() const {
    std::vector<KDL::Frame> result;
    result.resize(4);
    // 计算每条腿的末端执行器位置
    for (int i = 0; i < 4; i++) {
        result[i] = robot_legs_[i]->calcPEe2B(current_joint_pos_[i]);
        result[i].M = KDL::Rotation::Identity();
    }
    return result;
}

// 获取指定索引腿的末端执行器位置
KDL::Frame QuadrupedRobot::getFeet2BPositions(const int index) const {
    return robot_legs_[index]->calcPEe2B(current_joint_pos_[index]);
}

// 获取指定索引腿的雅可比矩阵
KDL::Jacobian QuadrupedRobot::getJacobian(const int index) const {
    return robot_legs_[index]->calcJaco(current_joint_pos_[index]);
}

// 计算指定力在给定腿部上的力矩
KDL::JntArray QuadrupedRobot::getTorque(
    const Vec3 &force, const int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], force);
}

// 计算指定力在给定腿部上的力矩，力是KDL::Vector类型
KDL::JntArray QuadrupedRobot::getTorque(const KDL::Vector &force, int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], Vec3(force.data));
}

// 计算指定腿部的末端执行器速度
KDL::Vector QuadrupedRobot::getFeet2BVelocities(const int index) const {
    const Mat3 jacobian = getJacobian(index).data.topRows(3);
    Vec3 foot_velocity = jacobian * current_joint_vel_[index].data;
    return {foot_velocity(0), foot_velocity(1), foot_velocity(2)};
}

// 获取所有四条腿的末端执行器速度
std::vector<KDL::Vector> QuadrupedRobot::getFeet2BVelocities() const {
    std::vector<KDL::Vector> result;
    result.resize(4);
    // 计算每条腿的末端执行器速度
    for (int i = 0; i < 4; i++) {
        result[i] = getFeet2BVelocities(i);
    }
    return result;
}

// 更新机器人当前的关节位置和速度
void QuadrupedRobot::update() {
    if (mass_ == 0) return;
    // 遍历每条腿，更新关节位置和速度
    for (int i = 0; i < 4; i++) {
        KDL::JntArray pos_array(3);
        // 获取每条腿的关节位置
        pos_array(0) = ctrl_interfaces_.joint_position_state_interface_[i * 3].get().get_optional().value();
        pos_array(1) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 1].get().get_optional().value();
        pos_array(2) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 2].get().get_optional().value();
        current_joint_pos_[i] = pos_array;

        KDL::JntArray vel_array(3);
        // 获取每条腿的关节速度
        vel_array(0) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3].get().get_optional().value();
        vel_array(1) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 1].get().get_optional().value();
        vel_array(2) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 2].get().get_optional().value();
        current_joint_vel_[i] = vel_array;
    }
}
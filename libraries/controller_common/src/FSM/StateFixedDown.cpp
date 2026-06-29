//
// Created by tlab-uav on 24-9-11.
//

#include "controller_common/FSM/StateFixedDown.h"

#include <cmath>

// 定义轮子关节的起始索引（16个关节：0-11腿，12-15轮子）
#define WHEEL_JOINT_START_INDEX 12
// 轮子转向比例系数（调大=转向更快，调小=更慢）
#define WHEEL_STEER_SCALE_ly 2.5
#define WHEEL_STEER_SCALE 2.5

StateFixedDown::StateFixedDown(CtrlInterfaces& ctrl_interfaces,
                               const std::vector<double>& target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::FIXEDDOWN, "fixed down", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 16; i++)
    {
        target_pos_[i] = target_pos[i];
    }
}

void StateFixedDown::enter()
{
    for (int i = 0; i < 16; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional().value();
    }
    ctrl_interfaces_.control_inputs_.command = 0;
    for (int i = 0; i < 12; i++)
    {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }
    for (int i = 12; i < 16; i++)
    {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }    
}

void StateFixedDown::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);

    // 2. 前12个关节：腿部关节 → 保持【趴下插值运动】（不变）
    for (int i = 0; i < 12; i++)
    {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
    }

    // 3. 后4个关节：轮子关节 → 【指令控制转向】（新增功能）
    // 读取控制指令：ly=前向速度  rx=偏航旋转速度
    double ly = ctrl_interfaces_.control_inputs_.ly *WHEEL_STEER_SCALE_ly;
    double rx = ctrl_interfaces_.control_inputs_.rx * WHEEL_STEER_SCALE;

    // 计算轮子目标转向角度（前向+偏航耦合，标准四轮转向逻辑）
    double wheel_fl = ly + rx;  // 左前轮
    double wheel_fr = ly - rx;  // 右前轮
    double wheel_rl = ly + rx;  // 左后轮
    double wheel_rr = ly - rx;  // 右后轮
    // 直接控制4个轮子关节位置（实时响应指令）
    std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[12].get().set_value(wheel_fr);  // FR_foot
    std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[13].get().set_value(wheel_fl);  // FL_foot
    std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[14].get().set_value(wheel_rr);  // RR_foot
    std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[15].get().set_value(wheel_rl);  // RL_foot
}

void StateFixedDown::exit()
{
    percent_ = 0;
}

FSMStateName StateFixedDown::checkChange()
{
    if (percent_ < 1.5)
    {
        return FSMStateName::FIXEDDOWN;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDSTAND;
    default:
        return FSMStateName::FIXEDDOWN;
    }
}

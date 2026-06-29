//
// Created by tlab-uav on 24-9-24.
//

/**
 * @file Ocs2QuadrupedController.h
 * @brief OCS2四足机器人控制器实现文件
 * @note 该文件实现了基于OCS2 (Optimal Control for Switched Systems) 框架的四足机器人控制器，
 *       集成了状态机管理、硬件接口配置、传感器数据处理和控制指令生成等核心功能
 */

#include "Ocs2QuadrupedController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <angles/angles.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>

// OCS2四足机器人名空间
namespace ocs2::legged_robot
{
    // 类型别名：简化控制器接口配置类型的书写
    using config_type = controller_interface::interface_configuration_type;

    /**
     * @brief 配置控制器的命令接口
     * @return 包含命令接口配置信息的结构体
     * @note 命令接口用于向硬件发送控制指令（如关节位置/速度/力矩），
     *       这里根据配置的关节名称和接口类型动态生成接口列表
     */
    controller_interface::InterfaceConfiguration Ocs2QuadrupedController::command_interface_configuration() const
    {
        // 初始化接口配置：使用独立接口模式
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        // 预分配内存：关节数量 × 命令接口类型数量
        conf.names.reserve(joint_names_.size() * command_interface_types_.size() + 
        foot_joint_names_.size() * command_interface_types_.size()
        );
        
        // 遍历所有关节和接口类型，生成完整的接口名称
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    // 带命令前缀的接口名称（如：controller/joint1/position）
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" + interface_type);
                }
                else
                {
                    // 无前缀的接口名称（如：joint1/position）
                    conf.names.push_back(joint_name + "/" + interface_type);
                }
            }
        }


            // 2. 新增轮毂关节命令接口
        for (const auto& foot_joint_name : foot_joint_names_) {
            for (const auto& interface_type : command_interface_types_) {
                conf.names.push_back(command_prefix_.empty() ? 
                    (foot_joint_name + "/" + interface_type) : 
                    (command_prefix_ + "/" + foot_joint_name + "/" + interface_type));
            }
        }

        return conf;
    }

    /**
     * @brief 配置控制器的状态接口
     * @return 包含状态接口配置信息的结构体
     * @note 状态接口用于读取硬件状态（关节状态、IMU、里程计、足端力等），
     *       根据不同传感器类型分别配置对应的接口列表
     */
    controller_interface::InterfaceConfiguration Ocs2QuadrupedController::state_interface_configuration() const
    {
        // 初始化接口配置：使用独立接口模式
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        // 预分配内存：关节状态接口数量
        conf.names.reserve(joint_names_.size() * state_interface_types_.size()+ 
        foot_joint_names_.size() * state_interface_types_.size()
        );
        
        // 1. 添加关节状态接口（如：joint1/position, joint1/velocity）
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        // 新增轮毂关节状态接口
        for (const auto& foot_joint_name : foot_joint_names_) {
            for (const auto& interface_type : state_interface_types_) {
                conf.names.push_back(foot_joint_name + "/" + interface_type);
            }
        }

        // 2. 添加IMU传感器接口（如：imu/data/angular_velocity, imu/data/linear_acceleration）
        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" + interface_type);
        }

        // 3. 添加里程计接口（仅当使用地面真值估计器时）
        if (estimator_type_ == "ground_truth")
        {
            for (const auto& interface_type : odom_interface_types_)
            {
                conf.names.push_back(odom_name_ + "/" + interface_type);
            }
        }
        
        // 4. 添加足端力传感器接口（如：foot_force/force_x, foot_force/force_z）
        for (const auto& interface_type : foot_force_interface_types_)
        {
            conf.names.push_back(foot_force_name_ + "/" + interface_type);
        }

        return conf;
    }

    /**
     * @brief 控制器主更新函数（核心循环）
     * @param time: 当前ROS系统时间
     * @param period: 从上一次更新到现在的时间间隔
     * @return 控制器更新结果状态码
     * @note 该函数在每个控制周期被调用，完成：
     *       1. 状态估计更新
     *       2. 有限状态机(FSM)的状态管理和切换
     *       3. 当前状态的控制逻辑执行
     */
    controller_interface::return_type Ocs2QuadrupedController::update(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period)
    {
        // 更新控制器组件的状态估计（传感器数据融合、状态计算）
        ctrl_comp_->updateState(time, period);

        // 有限状态机(FSM)状态管理
        if (mode_ == FSMMode::NORMAL)  // 正常运行模式
        {
            // 执行当前状态的主逻辑
            current_state_->run(time, period);
            
            // 检查是否需要切换状态
            next_state_name_ = current_state_->checkChange();
            
            // 状态需要切换时的处理
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;          // 切换到状态变更模式
                next_state_ = getNextState(next_state_name_);  // 获取目标状态
                
                // 打印状态切换日志
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)  // 状态变更模式
        {
            // 执行当前状态的退出逻辑
            current_state_->exit();
            
            // 切换到新状态
            current_state_ = next_state_;

            // 执行新状态的进入逻辑
            current_state_->enter();
            
            // 恢复正常运行模式
            mode_ = FSMMode::NORMAL;
        }

        // 返回更新成功状态
        return controller_interface::return_type::OK;
    }

    /**
     * @brief 控制器初始化回调函数
     * @return 初始化结果状态码
     * @note 在控制器加载时调用，完成：
     *       1. 参数读取和初始化
     *       2. 控制器组件创建
     *       3. 状态机状态列表初始化
     */
    controller_interface::CallbackReturn Ocs2QuadrupedController::on_init()
    {
        // 读取控制器更新频率参数
        get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
        RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

        // 硬件参数读取（带默认值的自动声明）
        command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);  // 命令接口前缀
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);  // 关节名称列表
        // 新增轮毂参数读取
        foot_joint_names_ = auto_declare<std::vector<std::string>>("foot_joints", foot_joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);  // 命令接口类型
        state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);        // 状态接口类型

        // IMU传感器参数
        imu_name_ = auto_declare<std::string>("imu_name", imu_name_);  // IMU传感器名称
        imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", imu_interface_types_);  // IMU接口类型

        // 里程计传感器参数（地面真值）
        estimator_type_ = auto_declare<std::string>("estimator_type", estimator_type_);  // 状态估计器类型
        if (estimator_type_ == "ground_truth")
        {
            odom_name_ = auto_declare<std::string>("odom_name", odom_name_);  // 里程计名称
            odom_interface_types_ = auto_declare<std::vector<std::string>>("odom_interfaces", odom_interface_types_);  // 里程计接口类型
        }
        
        // 足端力传感器参数
        foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);  // 足端力传感器名称
        foot_force_interface_types_ = auto_declare<std::vector<std::string>>("foot_force_interfaces", foot_force_interface_types_);  // 足端力接口类型

        // 创建控制器核心组件实例
        ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_);
        
        // 根据估计器类型配置状态估计模块
        ctrl_comp_->setupStateEstimate(estimator_type_);

        // 初始化状态机状态列表
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);          // 被动状态（无控制输出）
        state_list_.fixedDown = std::make_shared<StateOCS2>(ctrl_interfaces_, ctrl_comp_);  // OCS2控制状态（固定落地）

        // 返回初始化成功
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief 控制器配置回调函数
     * @param previous_state: 配置前的生命周期状态
     * @return 配置结果状态码
     * @note 在控制器配置阶段调用，主要完成ROS订阅者的创建和初始化
     */
    controller_interface::CallbackReturn Ocs2QuadrupedController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // 创建控制输入消息订阅者（接收上位机控制指令）
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // 回调函数：更新控制输入参数
                ctrl_interfaces_.control_inputs_.command = msg->command;  // 控制命令（如：站立、行走）
                ctrl_interfaces_.control_inputs_.lx = msg->lx;            // 线速度x方向指令
                ctrl_interfaces_.control_inputs_.ly = msg->ly;            // 线速度y方向指令
                ctrl_interfaces_.control_inputs_.rx = msg->rx;            // 角速度x方向指令
                ctrl_interfaces_.control_inputs_.ry = msg->ry;            // 角速度y方向指令
            });

        // 返回配置成功
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief 控制器激活回调函数
     * @param previous_state: 激活前的生命周期状态
     * @return 激活结果状态码
     * @note 在控制器激活时调用，完成：
     *       1. 接口资源的分配和映射
     *       2. 状态机初始状态的设置
     */
    controller_interface::CallbackReturn Ocs2QuadrupedController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // 清空接口向量（防止重启时数据残留）
        ctrl_interfaces_.clear();

        // 1. 分配命令接口：将硬件接口映射到控制器内部的接口映射表
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                // 提取接口类型（如：从"joint1/position"中提取"position"）
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                // 无分隔符的接口名称直接使用
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // 2. 分配状态接口：分类存储不同类型的传感器接口
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                // IMU状态接口
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else if (interface.get_prefix_name() == foot_force_name_)
            {
                // 足端力状态接口
                ctrl_interfaces_.foot_force_state_interface_.emplace_back(interface);
            }
            else if (interface.get_prefix_name() == odom_name_)
            {
                // 里程计状态接口
                ctrl_interfaces_.odom_state_interface_.emplace_back(interface);
            }
            else
            {
                // 关节状态接口
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // 3. 初始化有限状态机(FSM)
        current_state_ = state_list_.passive;    // 设置初始状态为被动状态
        current_state_->enter();                 // 执行初始状态的进入逻辑
        next_state_ = current_state_;            // 初始化下一个状态
        next_state_name_ = current_state_->state_name;  // 初始化下一个状态名称
        mode_ = FSMMode::NORMAL;                 // 设置状态机模式为正常运行

        // 返回激活成功
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief 控制器去激活回调函数
     * @param previous_state: 去激活前的生命周期状态
     * @return 去激活结果状态码
     * @note 在控制器停止时调用，释放占用的接口资源
     */
    controller_interface::CallbackReturn Ocs2QuadrupedController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // 释放所有接口资源
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief 控制器清理回调函数
     * @param previous_state: 清理前的生命周期状态
     * @return 清理结果状态码
     * @note 在控制器清理时调用，释放资源
     */
    controller_interface::CallbackReturn Ocs2QuadrupedController::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief 控制器关闭回调函数
     * @param previous_state: 关闭前的生命周期状态
     * @return 关闭结果状态码
     * @note 在控制器关闭时调用，完成最终的资源释放
     */
    controller_interface::CallbackReturn Ocs2QuadrupedController::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief 控制器错误处理回调函数
     * @param previous_state: 错误发生前的生命周期状态
     * @return 错误处理结果状态码
     * @note 在控制器发生错误时调用，进行错误恢复或资源清理
     */
    controller_interface::CallbackReturn Ocs2QuadrupedController::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief 根据状态名称获取对应的状态实例
     * @param stateName: 目标状态名称枚举值
     * @return 对应状态的共享指针
     * @note 状态机状态切换的核心函数，根据状态名称映射到具体的状态实现类
     */
    std::shared_ptr<FSMState> Ocs2QuadrupedController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::PASSIVE:      // 被动状态
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:    // 固定落地状态（OCS2控制）
            return state_list_.fixedDown;
        default:                         // 无效状态
            return state_list_.invalid;
        }
    }
}

// 将控制器类注册为ROS2控制器插件，使其可被控制器管理器加载
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ocs2::legged_robot::Ocs2QuadrupedController, controller_interface::ControllerInterface);

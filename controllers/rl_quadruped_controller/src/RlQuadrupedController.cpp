//
// Created by tlab-uav on 24-10-4.
//

#include "RlQuadrupedController.h"

namespace rl_quadruped_controller
{
    using config_type = controller_interface::interface_configuration_type;

    // 返回命令接口的配置，这里对于每个关节和命令类型是独立的
    controller_interface::InterfaceConfiguration LeggedGymController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        // 预留足够空间来存储关节名称和接口类型
        conf.names.reserve(joint_names_.size() * command_interface_types_.size());

        // 遍历每个关节和接口类型，创建命令接口所需的名称
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                // 如果有命令前缀，则将其添加到关节和接口类型之前
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    // 返回状态接口的配置，与命令接口类似
    controller_interface::InterfaceConfiguration LeggedGymController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        // 为状态接口配置预留空间
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());

        // 遍历关节名称和状态接口类型，创建状态接口名称
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        // 将IMU和足底力传感器接口类型添加到状态接口配置中
        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        for (const auto& interface_type : foot_force_interface_types_)
        {
            conf.names.push_back(foot_force_name_ + "/" += interface_type);
        }

        return conf;
    }

    // 主更新函数，执行控制器的逻辑
    controller_interface::return_type LeggedGymController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // 如果启用了估计器，则更新机器人模型和估计器
        if (ctrl_component_.enable_estimator_)
        {
            if (ctrl_component_.robot_model_ == nullptr)
            {
                return controller_interface::return_type::OK;
            }

            ctrl_component_.robot_model_->update();
            ctrl_component_.estimator_->update();
        }

        // 处理FSM的逻辑，判断是否进入正常状态或状态变更
        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            // 如果状态发生变化，则更新FSM模式和当前状态
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            // 过渡到下一个状态
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    // 初始化控制器，声明参数并在启用估计器时设置估计器
    controller_interface::CallbackReturn LeggedGymController::on_init()
    {
        try
        {
            // 声明关节名称、足部名称和各种接口类型的参数
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            feet_names_ = auto_declare<std::vector<std::string>>("feet_names", feet_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // 声明其他配置参数
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);

            // imu 传感器
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);

            // 足底传感器
            foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);
            foot_force_interface_types_ =
                auto_declare<std::vector<std::string>>("foot_force_interfaces", foot_force_interface_types_);
            feet_force_threshold_ = auto_declare<double>("feet_force_threshold", feet_force_threshold_);

            // 姿态参数
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);
            rx_scale_ = auto_declare<double>("rx_scale", rx_scale_);
            ly_scale_ = auto_declare<double>("ly_scale", ly_scale_);
            lx_scale_ = auto_declare<double>("lx_scale", lx_scale_);

            // 获取控制器更新频率
            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            // 如果存在足底力传感器接口，则启用估计器并初始化估计器
            if (foot_force_interface_types_.size() == 4)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Enable Estimator");
                ctrl_component_.enable_estimator_ = true;
                ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
            }
            ctrl_component_.node_ = get_node();
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    // 配置控制器，订阅必要的主题，例如robot_description和control_input
    controller_interface::CallbackReturn LeggedGymController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (ctrl_component_.enable_estimator_)
                {
                    // 从机器人描述消息初始化机器人模型
                    ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                        ctrl_interfaces_, msg->data, feet_names_, base_name_);
                }
            });

        // 处理控制输入订阅
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // 处理传入的控制输入消息
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx*lx_scale_;
                ctrl_interfaces_.control_inputs_.ly = msg->ly*ly_scale_;
                ctrl_interfaces_.control_inputs_.rx = msg->rx*rx_scale_;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        return CallbackReturn::SUCCESS;
    }

    // 激活控制器，清除向量并分配接口
    controller_interface::CallbackReturn LeggedGymController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // 清除向量以防重启
        ctrl_interfaces_.clear();

        // 将命令接口分配给适当的映射
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // 将状态接口分配给适当的映射
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else if (interface.get_prefix_name() == foot_force_name_)
            {
                ctrl_interfaces_.foot_force_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // 创建FSM列表并初始化状态
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.rl = std::make_shared<StateRL>(ctrl_interfaces_, ctrl_component_, stand_pos_);

        // 初始化FSM并设置当前状态
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    // 停用控制器并释放接口
    controller_interface::CallbackReturn LeggedGymController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    // 清理回调
    controller_interface::CallbackReturn
    LeggedGymController::on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_cleanup(previous_state);
    }

    // 关闭回调
    controller_interface::CallbackReturn
    LeggedGymController::on_shutdown(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_shutdown(previous_state);
    }

    // 错误回调
    controller_interface::CallbackReturn LeggedGymController::on_error(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_error(previous_state);
    }

    // 根据状态名获取下一个FSM状态
    std::shared_ptr<FSMState> LeggedGymController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::RL:
            return state_list_.rl;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
// 注册控制器插件到ROS2
PLUGINLIB_EXPORT_CLASS(rl_quadruped_controller::LeggedGymController, controller_interface::ControllerInterface);
//
// Created by biao on 24-10-6.
//

#include "rl_quadruped_controller/FSM/StateRL.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

// 从YAML文件中读取一个向量
template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
    std::vector<T> values;
    for (const auto& val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

// StateRL类的构造函数，初始化强化学习（RL）状态
StateRL::StateRL(CtrlInterfaces& ctrl_interfaces,
                 CtrlComponent& ctrl_component,
                 const std::vector<double>& target_pos) :
    FSMState(FSMStateName::RL, "rl", ctrl_interfaces),
    node_(ctrl_component.node_),
    enable_estimator_(ctrl_component.enable_estimator_),
    estimator_(ctrl_component.estimator_)
{
    // 声明并获取参数
    node_->declare_parameter("robot_pkg", robot_pkg_);
    node_->declare_parameter("model_folder", model_folder_);
    node_->declare_parameter("use_rl_thread", use_rl_thread_);
    robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
    model_folder_ = node_->get_parameter("model_folder").as_string();
    use_rl_thread_ = node_->get_parameter("use_rl_thread").as_bool();

    RCLCPP_INFO(node_->get_logger(), "Using robot model from %s", robot_pkg_.c_str());
    
    // 获取模型文件夹路径
    const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
    const std::string model_path = package_share_directory + "/config/" + model_folder_;

    // 从YAML文件加载参数
    loadYaml(model_path);

    // 将目标位置赋值给默认DOF位置（覆盖初始全0值）
    std::vector<double> default_pos_vec(params_.num_of_dofs, 0.0);
    int target_size = std::min(static_cast<int>(target_pos.size()), params_.num_of_dofs);
    for (int i = 0; i < target_size; i++) {
        default_pos_vec[i] = target_pos[i]; // 用输入的目标位置覆盖默认值
    }
    params_.default_dof_pos = torch::tensor(default_pos_vec, torch::kFloat).unsqueeze(0);
    RCLCPP_INFO(node_->get_logger(), "Default DOF pos initialized with target_pos, size=%d", target_size);

    // 初始化历史观测缓冲区
    if (!params_.observations_history.empty())
    {
        history_obs_buf_ = std::make_shared<ObservationBuffer>(1, params_.num_observations,
                                                               params_.observations_history.size());
    }

    RCLCPP_INFO(node_->get_logger(), "Model loading: %s", params_.model_name.c_str());
    // 加载模型
    model_ = torch::jit::load(model_path + "/" + params_.model_name);

    // 启动强化学习线程（如果需要）
    if (use_rl_thread_)
    {
        rl_thread_ = std::thread([&]{
            while (true)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (running_)
                            {
                                runModel();
                            }
                        },
                        ctrl_interfaces_.frequency_ / params_.decimation);
                }
                catch (const std::exception& e)
                {
                    running_ = false;
                    RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "Error in RL thread: %s", e.what());
                }
            }
        });
        setThreadPriority(60, rl_thread_);
    }
}

// 进入RL状态时初始化相关变量
void StateRL::enter()
{
    // 初始化观测值
    obs_.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs_.commands = torch::tensor({{0.0, 0.0, 0.0}});
    // base_quat follows the layout expected by quatRotateInverse():
    // isaacsim: [w, x, y, z], isaacgym: [x, y, z, w]
    if (params_.framework == "isaacsim")
    {
        obs_.base_quat = torch::tensor({{1.0, 0.0, 0.0, 0.0}});
    }
    else if (params_.framework == "isaacgym")
    {
        obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    }
    else
    {
        RCLCPP_WARN(
            rclcpp::get_logger("StateRL"),
            "Unknown framework '%s', defaulting base_quat initialization to isaacgym xyzw layout",
            params_.framework.c_str());
        obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    }
    obs_.dof_pos = params_.default_dof_pos;
    obs_.dof_vel = torch::zeros({1, params_.num_of_dofs});
    obs_.actions = torch::zeros({1, params_.num_of_dofs});

    // 初始化输出
    output_torques = torch::zeros({1, params_.num_of_dofs});
    output_dof_pos_ = params_.default_dof_pos;

    // 初始化控制
    control_.x = 0.0;
    control_.y = 0.0;
    control_.yaw = 0.0;

    // 初始化历史缓冲区（ IsaacLab 方式：第一帧观测复制填满缓冲区）
    if (!params_.observations_history.empty()) {
        torch::Tensor init_obs = computeObservation();  // 计算正确初始观测
        history_obs_buf_->resetAll(init_obs);           // 复制填充所有历史帧
    }


    running_ = true;
}

// 执行RL状态的主要逻辑
void StateRL::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    getState();
    if (!use_rl_thread_)
    {
        runModel();
    }
    setCommand();
}

// 退出RL状态时进行清理
void StateRL::exit()
{
    running_ = false;
}

// 检查是否需要切换状态
FSMStateName StateRL::checkChange()
{
    if (enable_estimator_ and !estimator_->safety())
    {
        return FSMStateName::PASSIVE;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDDOWN;
    default:
        return FSMStateName::RL;
    }
}

// 计算当前状态的观测值
torch::Tensor StateRL::computeObservation()
{
    std::vector<torch::Tensor> obs_list;

    // 根据配置的观测项目计算观测值
    for (const std::string& observation : params_.observations)
    {
        if (observation == "lin_vel")
        {
            obs_list.push_back(obs_.lin_vel * params_.lin_vel_scale);
        }
        else if (observation == "ang_vel")
        {
            obs_list.push_back(obs_.ang_vel * params_.ang_vel_scale);
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(obs_.commands * params_.commands_scale);
        }
        else if (observation == "dof_pos")
        {
            // 轮子关节的 dof_pos 置0
            torch::Tensor dof_pos_rel = (obs_.dof_pos - params_.default_dof_pos) * params_.dof_pos_scale;
            for (int i : params_.wheel_indices)
            {
                dof_pos_rel[0][i] = 0.0; // 轮子关节位置置0
            }
            obs_list.push_back(dof_pos_rel);
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(obs_.dof_vel * params_.dof_vel_scale);
        }
        else if (observation == "actions")
        {
            obs_list.push_back(obs_.actions);
        }
    }

    // 合并观测值并进行裁剪
    const torch::Tensor obs = cat(obs_list, 1);
    torch::Tensor clamped_obs = clamp(obs, -params_.clip_obs, params_.clip_obs);
    return clamped_obs;
}

// 从YAML文件加载配置
void StateRL::loadYaml(const std::string& config_path)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path + "/config.yaml");
    }
    catch ([[maybe_unused]] YAML::BadFile& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "The file '%s' does not exist", config_path.c_str());
        return;
    }

    // 读取YAML文件中的各种配置参数
    params_.model_name = config["model_name"].as<std::string>();
    params_.framework = config["framework"].as<std::string>();
    if (config["observations_history"].IsNull())
    {
        params_.observations_history = {};
    }
    else
    {
        params_.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    }
    params_.decimation = config["decimation"].as<int>();
    params_.num_observations = config["num_observations"].as<int>();
    params_.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    params_.clip_obs = config["clip_obs"].as<double>();
    
    // 处理动作裁剪范围和缩放因子
    if (config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull())
    {
        params_.clip_actions_upper = torch::tensor({}).view({1, -1});
        params_.clip_actions_lower = torch::tensor({}).view({1, -1});
    }
    else
    {
        params_.clip_actions_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_upper"])).view({1, -1});
        params_.clip_actions_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_lower"])).view({1, -1});
    }
    params_.action_scale = torch::tensor(
        ReadVectorFromYaml<double>(config["action_scale"])
    ).view({1, -1}); // 转为shape=[1,16]的张量，匹配16个DOF
    params_.hip_scale_reduction = config["hip_scale_reduction"].as<double>();
    params_.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    params_.num_of_dofs = config["num_of_dofs"].as<int>();
    params_.lin_vel_scale = config["lin_vel_scale"].as<double>();
    params_.ang_vel_scale = config["ang_vel_scale"].as<double>();
    params_.dof_pos_scale = config["dof_pos_scale"].as<double>();
    params_.dof_vel_scale = config["dof_vel_scale"].as<double>();
    params_.commands_scale = torch::tensor(
    ReadVectorFromYaml<double>(config["commands_scale"]),torch::TensorOptions().dtype(torch::kFloat32)).view({1, -1});
    params_.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"])).view({
        1, -1
    });
    params_.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"])).view({
        1, -1
    });
    params_.wheel_indices = ReadVectorFromYaml<int>(config["wheel_indices"]);
    params_.torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"])).view({1, -1});
    // ====================== 读取关节映射 ======================
    params_.model_to_real = ReadVectorFromYaml<int>(config["model_to_real"]);


    // 初始化default_dof_pos为全0，构造函数中会根据目标位置覆盖
    std::vector<double> default_pos_vec(params_.num_of_dofs, 0.0);
    params_.default_dof_pos = torch::tensor(default_pos_vec, torch::kFloat).unsqueeze(0);
    RCLCPP_INFO(rclcpp::get_logger("StateRL"), "loadYaml完成：num_of_dofs=%d", params_.num_of_dofs);
}

// 使用四元数旋转逆操作
torch::Tensor StateRL::quatRotateInverse(const torch::Tensor& q, const torch::Tensor& v, const std::string& framework)
{
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if (framework == "isaacsim")
    {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    }
    else if (framework == "isaacgym")
    {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    const c10::IntArrayRef shape = q.sizes();

    const torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    const torch::Tensor b = cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    const torch::Tensor c = q_vec * bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

// 前向传播计算动作
torch::Tensor StateRL::forward()
{
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = computeObservation();
    torch::Tensor actions;

    // 如果有观测历史，则使用历史观测来计算动作
    if (!params_.observations_history.empty())
    {
        history_obs_buf_->insert(clamped_obs);
        history_obs_ = history_obs_buf_->getObsVec(params_.observations_history);
        actions = model_.forward({history_obs_}).toTensor();
    }
    else
    {
        actions = model_.forward({clamped_obs}).toTensor();
    }

    // 如果有动作裁剪范围，则进行裁剪
    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0)
    {
        return clamp(actions, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    return actions;
}

// 获取机器人的状态
void StateRL::getState()
{
    // 获取IMU数据并更新
    if (params_.framework == "isaacgym")
    {
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[0].get().get_optional().value();
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[1].get().get_optional().value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[2].get().get_optional().value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[3].get().get_optional().value();
    }
    else if (params_.framework == "isaacsim")
    {
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[0].get().get_optional().value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[1].get().get_optional().value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[2].get().get_optional().value();
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[3].get().get_optional().value();
    }

    robot_state_.imu.gyroscope[0] = ctrl_interfaces_.imu_state_interface_[4].get().get_optional().value();
    robot_state_.imu.gyroscope[1] = ctrl_interfaces_.imu_state_interface_[5].get().get_optional().value();
    robot_state_.imu.gyroscope[2] = ctrl_interfaces_.imu_state_interface_[6].get().get_optional().value();

    robot_state_.imu.accelerometer[0] = ctrl_interfaces_.imu_state_interface_[7].get().get_optional().value();
    robot_state_.imu.accelerometer[1] = ctrl_interfaces_.imu_state_interface_[8].get().get_optional().value();
    robot_state_.imu.accelerometer[2] = ctrl_interfaces_.imu_state_interface_[9].get().get_optional().value();

    for (int model_idx = 0; model_idx < params_.num_of_dofs; model_idx++)
    {
        int real_idx = params_.model_to_real[model_idx];  // <--- 映射

        robot_state_.motor_state.q[model_idx] = ctrl_interfaces_.joint_position_state_interface_[real_idx].get().get_optional().value();
        robot_state_.motor_state.dq[model_idx] = ctrl_interfaces_.joint_velocity_state_interface_[real_idx].get().get_optional().value();
        robot_state_.motor_state.tauEst[model_idx] = ctrl_interfaces_.joint_effort_state_interface_[real_idx].get().get_optional().value();
    }


    control_.x = ctrl_interfaces_.control_inputs_.ly;
    control_.y = -ctrl_interfaces_.control_inputs_.lx;
    control_.yaw = -ctrl_interfaces_.control_inputs_.rx;

    updated_ = true;
}

// 执行模型并计算动作
void StateRL::runModel()
{
    if (enable_estimator_)
    {
        obs_.lin_vel = torch::from_blob(estimator_->getVelocity().data(), {3}, torch::kDouble).clone().
            to(torch::kFloat).unsqueeze(0);
    }
    obs_.ang_vel = torch::tensor(robot_state_.imu.gyroscope).unsqueeze(0);
    obs_.commands = torch::tensor({{control_.x, control_.y, control_.yaw}});
    obs_.base_quat = torch::tensor(robot_state_.imu.quaternion).unsqueeze(0);
    obs_.dof_pos = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);

    const torch::Tensor clamped_actions = forward();

    for (const int i : params_.hip_scale_reduction_indices)
    {
        clamped_actions[0][i] *= params_.hip_scale_reduction;
    }

    obs_.actions = clamped_actions;

    // ========== 新增：区分腿/轮子的动作缩放（核心修改） ==========
    torch::Tensor actions_scaled = clamped_actions * params_.action_scale;
    torch::Tensor pos_actions_scaled = actions_scaled.clone(); // 腿关节：位置动作
    torch::Tensor vel_actions_scaled = torch::zeros_like(actions_scaled); // 轮子关节：速度动作

    // 轮子关节：位置动作置0，速度动作保留
    for (int i : params_.wheel_indices)
    {
        pos_actions_scaled[0][i] = 0.0;
        vel_actions_scaled[0][i] = actions_scaled[0][i]; // 直接用action_scale后的轮子速度
    }

   // ========== 新增：分别计算位置/速度输出 ==========
    output_dof_pos_ = pos_actions_scaled + params_.default_dof_pos; // 腿关节目标位置
    torch::Tensor output_dof_vel_ = vel_actions_scaled; // 轮子关节目标速度

    // ========== 修改：同时填充位置和速度指令 ==========
    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        robot_command_.motor_command.q[i] = output_dof_pos_[0][i].item<double>();
        // ✅ 轮子关节：下发速度；腿关节：速度置0
        robot_command_.motor_command.dq[i] = output_dof_vel_[0][i].item<double>();
        robot_command_.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        robot_command_.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        robot_command_.motor_command.tau[i] = 0;
    }
}

void StateRL::setCommand() const
{
        for (int model_idx = 0; model_idx < params_.num_of_dofs; model_idx++)
        {
            int real_idx = params_.model_to_real[model_idx];  // <--- 映射

            std::ignore = ctrl_interfaces_.joint_position_command_interface_[real_idx].get().set_value(robot_command_.motor_command.q[model_idx]);
            std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[real_idx].get().set_value(robot_command_.motor_command.dq[model_idx]);
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[real_idx].get().set_value(robot_command_.motor_command.kp[model_idx]);
            std::ignore = ctrl_interfaces_.joint_kd_command_interface_[real_idx].get().set_value(robot_command_.motor_command.kd[model_idx]);
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[real_idx].get().set_value(robot_command_.motor_command.tau[model_idx]);
        }

}
//
// Created by tlab-uav on 25-2-27.
//


#include <pinocchio/fwd.hpp>

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <angles/angles.h>
// OCS2 ROS接口：ROS消息与Eigen数据转换
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
// OCS2核心工具：加载配置文件数据
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
// 全身体制控制器(WBC)相关实现：加权WBC、分层WBC、HQP求解器
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_quadruped_controller/wbc/HierarchicalWbc.h>
#include <ocs2_quadruped_controller/wbc/HoQp.h>
// OCS2 SQP求解器：MPC核心求解器
#include <ocs2_sqp/SqpMpc.h>
// OCS2机器人工具：提供欧拉角转旋转矩阵等坐标变换函数
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>  // std::min
#include <cmath>

// 命名空间：ocs2的legged_robot模块
namespace ocs2::legged_robot
{
    /**
     * @brief StateOCS2构造函数
     * @param ctrl_interfaces: 控制器接口集合，用于访问硬件指令接口和控制输入
     * @param ctrl_component: 控制器核心组件指针，提供MPC、状态估计、机器人模型等功能
     * @details 初始化OCS2控制状态：
     * 1. 初始化FSM基类，设置状态名称和控制器接口
     * 2. 加载PD控制参数（kp/kd）
     * 3. 创建分层全身体制控制器(WBC)并加载任务配置
     * 4. 创建安全检查器，用于监控系统状态
     */
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_component)
        : FSMState(FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),  // 初始化FSM状态基类
          ctrl_component_(ctrl_component),                              // 保存控制器核心组件指针
          node_(ctrl_component->node_),                                 // 保存ROS2节点指针
          wheel_command_pinocchio_interface_(ctrl_component->legged_interface_->getPinocchioInterface()),
          planned_wheel_pinocchio_interface_(ctrl_component->legged_interface_->getPinocchioInterface()),
          planned_wheel_mapping_(ctrl_component->legged_interface_->getCentroidalModelInfo())
    {
        // 声明并加载ROS2参数：默认PD控制的比例增益(kp)和微分增益(kd)
        node_->declare_parameter("default_kp", default_kp_);
        node_->declare_parameter("default_kd", default_kd_);
        default_kp_ = node_->get_parameter("default_kp").as_double();
        default_kd_ = node_->get_parameter("default_kd").as_double();


        // // ==================== 关键修复 1 ====================
        // // 为末端执行器运动学求解器设置Pinocchio接口（解决核心错误）
        // ctrl_component_->ee_kinematics_->setPinocchioInterface(
        //     ctrl_component_->legged_interface_->getPinocchioInterface()
        // );

        // 创建分层全身体制控制器(WBC)：将MPC输出转换为关节力矩/位置/速度指令
        wbc_ = std::make_shared<HierarchicalWbc>(
            ctrl_component_->legged_interface_->getPinocchioInterface(),    // 机器人Pinocchio模型接口
            ctrl_component_->legged_interface_->getCentroidalModelInfo(),  // 质心模型信息
            *ctrl_component_->ee_kinematics_,                              // 末端执行器运动学求解器
            ctrl_component_->legged_interface_->getSwitchedModelReferenceManagerPtr().get());
        // 加载WBC任务配置（权重、约束等）
        wbc_->loadTasksSetting(ctrl_component_->task_file_, ctrl_component_->verbose_);

        // 创建安全检查器：监控系统状态，防止异常
        safety_checker_ = std::make_shared<SafetyChecker>(
            ctrl_component_->legged_interface_->getCentroidalModelInfo());

        planned_wheel_kinematics_ = std::make_unique<PinocchioEndEffectorKinematics>(
            planned_wheel_pinocchio_interface_,
            planned_wheel_mapping_,
            ctrl_component_->legged_interface_->modelSettings().contactNames3DoF);
        planned_wheel_kinematics_->setPinocchioInterface(planned_wheel_pinocchio_interface_);

        // ===================== 新增：车轮参数初始化 =====================
        // 加载车轮半径参数
        // 替换构造函数中加载车轮半径的代码
        node_->declare_parameter("kalmanFilter.footRadius", 0.0992);
        wheel_radius_ = node_->get_parameter("kalmanFilter.footRadius").as_double();
        RCLCPP_INFO(node_->get_logger(), "加载车轮半径（与卡尔曼滤波器统一）：%.4f m", wheel_radius_);
        ;    

        // 创建车轮速度指令订阅器：订阅/wheel/target_vel话题
        wheel_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/wheel/target_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                // 将车轮速度指令写入线程安全缓冲区
                wheel_vel_buffer_.writeFromNonRT(*msg);
                RCLCPP_DEBUG(node_->get_logger(), "接收车轮速度指令：前向=%.3f m/s，偏航=%.3f rad/s",
                            msg->linear.x, msg->angular.z);
            });
        // ===================== 新增结束 =====================    
    }

    /**
     * @brief 进入OCS2控制状态时的回调函数
     * @details 初始化MPC控制器，确保首次控制策略求解完成
     */
    void StateOCS2::enter()
    {
        // 初始化控制器核心组件（MPC首次求解）
        ctrl_component_->init();
    }

    /**
     * @brief OCS2控制状态的主运行函数
     * @param time: 当前ROS2时间戳（未使用，参数保留）
     * @param period: 控制周期（两次运行的时间间隔）
     * @details 核心控制流程：
     * 1. 检查MPC是否运行，未运行则直接返回
     * 2. 更新并评估MPC控制策略，获取优化后的状态和输入
     * 3. 执行全身体制控制(WBC)，计算关节指令
     * 4. 分发关节指令到硬件接口（力矩/位置/速度/PD增益）
     * 5. 更新可视化
     */
    void StateOCS2::run(const rclcpp::Time& /*time*/,
                        const rclcpp::Duration& period)
    {
        // MPC未运行时，直接返回（避免无效计算）
        if (ctrl_component_->mpc_running_ == false)
        {
            return;
        }

        // 加载最新的MPC控制策略（从MPC求解器获取）
        ctrl_component_->mpc_mrt_interface_->updatePolicy();

        // 评估当前MPC策略：根据当前时间和状态，获取优化后的状态、输入和计划模式
        size_t planned_mode = 0;
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(
            ctrl_component_->observation_.time,          // 当前观测时间
            ctrl_component_->observation_.state,         // 当前观测状态
            optimized_state_,                            // 输出：优化后的状态
            optimized_input_,                             // 输出：优化后的输入
            planned_mode);                                // 输出：计划的接触模式

        // 将MPC优化后的输入设置到观测数据中，供WBC使用
        ctrl_component_->observation_.input = optimized_input_;
        // 执行全身体制控制(WBC)：将MPC的高层指令转换为关节级指令
        wbc_timer_.startTimer();  // 启动WBC耗时统计
        vector_t x = wbc_->update(
            optimized_state_,             // MPC优化后的状态
            optimized_input_,              // MPC优化后的输入
            ctrl_component_->measured_rbd_state_,  // 实测的刚体动力学状态
            planned_mode,                  // 计划的接触模式
            period.seconds());             // 控制周期（秒）
        wbc_timer_.endTimer();    // 结束WBC耗时统计



        // ---- 获取机器人模型信息 ----
        const auto& info = ctrl_component_->legged_interface_->getCentroidalModelInfo();
        const int n = static_cast<int>(info.actuatedDofNum);  // 驱动自由度数量（此处应为12）

        // ===================== 提取WBC优化的足端接触力（hardware wheel order） =====================
        // hardware wheel order: [FR, FL, RR, RL]
        // OCS2 contact order:   [FL, FR, RL, RR]
        constexpr std::array<size_t, 4> kHardwareToOcs2Contact = {1, 0, 3, 2};
        std::array<vector3_t, 4> foot_forces_world_hw{};
        const size_t forceStart = info.generalizedCoordinatesNum;
        for (size_t hw = 0; hw < foot_forces_world_hw.size(); ++hw) {
            const size_t contact = kHardwareToOcs2Contact[hw];
            const size_t forceIndex = forceStart + 3 * contact;
            if (forceIndex + 2 < static_cast<size_t>(x.size())) {
                foot_forces_world_hw[hw] = x.segment<3>(forceIndex);
            } else {
                foot_forces_world_hw[hw].setZero();
            }
        }

        // 从observation中提取基座欧拉角（zyx顺序：roll/pitch/yaw），构建「基体→世界」旋转矩阵。
        const vector_t currentPose = ctrl_component_->observation_.state.segment<6>(6);
        const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
        const matrix3_t R_base_to_world = getRotationMatrixFromZyxEulerAngles(zyx);

        // ---- 提取WBC输出的关节力矩：位于输出向量的末尾 ----
        vector_t torque = x.tail(n);

        // 从MPC优化结果中提取期望关节位置和速度（维度=驱动自由度）
        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_, info);  // 期望关节位置
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_, info);  // 期望关节速度

        // ---- 安全处理：匹配ROS2控制指令接口的维度 ----
        const int cmd_size = static_cast<int>(ctrl_interfaces_.joint_torque_command_interface_.size());
        const int m = std::min(n, cmd_size);  // 取较小值，避免数组越界


        // ---- 分发关节指令到硬件接口 ----
        for (int i = 0; i < m; ++i)
        {
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque(i));
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(default_kp_);
            std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(default_kd_);
        }

        // ==============================================
        // 新增代码：车轮角速度指令下发（核心）
        // 贴合论文公式：wheel_vel = v_roll / r_wheel
        // ==============================================
        // 1. 从线程安全缓冲区读取车轮速度指令（实时线程安全），作为 planned velocity 无效时的fallback。
        geometry_msgs::msg::Twist wheel_vel_cmd;
        const auto* wheelVelPtr = wheel_vel_buffer_.readFromRT();
        if (wheelVelPtr != nullptr)
        {
            wheel_vel_cmd = *wheelVelPtr;
        }
        else
        {
            // 无指令时默认速度为0
            wheel_vel_cmd.linear.x = 0.0;
            wheel_vel_cmd.angular.z = 0.0;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "未接收到车轮速度指令，默认速度为0");
        }

        updateWheelCommandPinocchioFrames(ctrl_component_->measured_rbd_state_);
        const vector3_t terrainNormal = getTerrainNormalWorldForWheelCommandOrUnitZ();
        const std::array<vector3_t, 4> measured_rolling_directions_hw = getWheelRollingDirectionsHardwareOrder(R_base_to_world);

        const vector3_t desired_body_velocity(wheel_vel_cmd.linear.x, 0.0, 0.0);
        const vector3_t desired_world_velocity = R_base_to_world * desired_body_velocity;
        const vector3_t yaw_rate_world = terrainNormal * wheel_vel_cmd.angular.z;
        const vector3_t base_position_world = ctrl_component_->measured_rbd_state_.segment<3>(3);
        const std::array<vector3_t, 4> wheel_positions_hw =
            getWheelPositionsWorldHardwareOrder(base_position_world, R_base_to_world);

        std::array<scalar_t, 4> fallback_wheel_omegas_hw{};
        for (size_t wheel_idx = 0; wheel_idx < fallback_wheel_omegas_hw.size(); ++wheel_idx)
        {
            const vector3_t r_wheel_from_base_world = wheel_positions_hw[wheel_idx] - base_position_world;
            const vector3_t desired_contact_velocity_world =
                desired_world_velocity + yaw_rate_world.cross(r_wheel_from_base_world);
            fallback_wheel_omegas_hw[wheel_idx] =
                desired_contact_velocity_world.dot(measured_rolling_directions_hw[wheel_idx]) / wheel_radius_;
        }

        std::array<vector3_t, 4> planned_contact_velocities_hw{};
        const bool planned_velocity_ok = getPlannedContactVelocitiesWorldHardwareOrder(
            optimized_state_, optimized_input_, planned_contact_velocities_hw);
        const std::array<vector3_t, 4> planned_rolling_directions_hw = planned_velocity_ok
            ? getWheelRollingDirectionsHardwareOrderFromInterface(planned_wheel_pinocchio_interface_, R_base_to_world)
            : measured_rolling_directions_hw;
        const auto contactFlags = modeNumber2StanceLeg(planned_mode);

        // 遍历4个车轮：stance优先使用MPC planned contact rolling velocity；异常时退回/wheel/target_vel。
        for (size_t wheel_idx = 0; wheel_idx < 4; ++wheel_idx)
        {
            const size_t wheel_interface_idx = wheel_idx + 12;
            if (wheel_interface_idx < ctrl_interfaces_.joint_velocity_command_interface_.size())
            {
                const size_t contact = kHardwareToOcs2Contact[wheel_idx];
                const bool planned_contact = contact < contactFlags.size() && contactFlags[contact];
                vector3_t rolling_direction_for_command = measured_rolling_directions_hw[wheel_idx];
                scalar_t wheel_omega = 0.0;

                if (planned_contact)
                {
                    const bool planned_data_valid = planned_velocity_ok &&
                        planned_contact_velocities_hw[wheel_idx].allFinite() &&
                        planned_rolling_directions_hw[wheel_idx].allFinite() &&
                        planned_rolling_directions_hw[wheel_idx].norm() > 1e-6;
                    if (planned_data_valid)
                    {
                        rolling_direction_for_command = planned_rolling_directions_hw[wheel_idx].normalized();
                        wheel_omega = planned_contact_velocities_hw[wheel_idx].dot(rolling_direction_for_command) / wheel_radius_;
                    }
                    else
                    {
                        wheel_omega = fallback_wheel_omegas_hw[wheel_idx];
                    }
                }

                std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[wheel_interface_idx].get().set_value(wheel_omega);

                const scalar_t contact_threshold = 7.0;
                // 简化牵引前馈：完整论文WBC轮扭矩需由 inverse dynamics / WBC torque vector 生成。
                scalar_t tangential_force = -foot_forces_world_hw[wheel_idx].dot(rolling_direction_for_command);
                const scalar_t normal_force = foot_forces_world_hw[wheel_idx].dot(terrainNormal);
                if (normal_force < contact_threshold) {
                    tangential_force = 0.0;
                }

                scalar_t wheel_torque = tangential_force * wheel_radius_;
                const scalar_t max_wheel_torque = 3.6;
                wheel_torque = std::clamp(wheel_torque, -max_wheel_torque, max_wheel_torque);

                if (wheel_interface_idx < ctrl_interfaces_.joint_torque_command_interface_.size()) {
                    std::ignore = ctrl_interfaces_.joint_torque_command_interface_[wheel_interface_idx].get().set_value(wheel_torque);
                    std::ignore = ctrl_interfaces_.joint_kd_command_interface_[wheel_interface_idx].get().set_value(2.5);
                }
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "车轮接口索引%zu越界，硬件接口总数：%zu",
                            wheel_interface_idx, ctrl_interfaces_.joint_velocity_command_interface_.size());
            }
        }




        // 更新可视化：在RViz中显示MPC策略和指令
        ctrl_component_->visualizer_->update(
            ctrl_component_->mpc_mrt_interface_->getPolicy(),    // MPC控制策略
            ctrl_component_->mpc_mrt_interface_->getCommand());  // MPC控制指令
    }

    /**
     * @brief 退出OCS2控制状态时的回调函数
     * @details 当前无额外处理逻辑，预留接口
     */
    void StateOCS2::exit()
    {
    }

    vector3_t StateOCS2::getTerrainNormalWorldForWheelCommandOrUnitZ() const
    {
        vector3_t terrainNormal = vector3_t::UnitZ();
        const auto referenceManagerPtr = ctrl_component_->legged_interface_->getSwitchedModelReferenceManagerPtr();
        if (referenceManagerPtr != nullptr)
        {
            terrainNormal = referenceManagerPtr->getTerrainNormalWorldOrUnitZ();
        }
        if (!terrainNormal.allFinite() || terrainNormal.norm() < 1e-6)
        {
            terrainNormal = vector3_t::UnitZ();
        }
        else
        {
            terrainNormal.normalize();
        }
        if (terrainNormal.z() < 0.0)
        {
            terrainNormal = -terrainNormal;
        }
        constexpr scalar_t kMinNormalZForWheelCommand = 0.70;
        if (terrainNormal.z() < kMinNormalZForWheelCommand)
        {
            terrainNormal = vector3_t::UnitZ();
        }
        return terrainNormal;
    }

    vector3_t StateOCS2::getTangentProjectionOrFallback(const vector3_t& direction, const vector3_t& normal)
    {
        vector3_t tangent = direction - direction.dot(normal) * normal;
        if (tangent.allFinite() && tangent.norm() > 1e-6)
        {
            return tangent.normalized();
        }

        tangent = vector3_t::UnitY() - vector3_t::UnitY().dot(normal) * normal;
        if (tangent.allFinite() && tangent.norm() > 1e-6)
        {
            return tangent.normalized();
        }

        tangent = vector3_t::UnitX() - vector3_t::UnitX().dot(normal) * normal;
        if (tangent.allFinite() && tangent.norm() > 1e-6)
        {
            return tangent.normalized();
        }

        return normal.unitOrthogonal().normalized();
    }

    void StateOCS2::updateWheelCommandPinocchioFrames(const vector_t& rbdStateMeasured)
    {
        const auto& info = ctrl_component_->legged_interface_->getCentroidalModelInfo();
        vector_t q(info.generalizedCoordinatesNum);
        q.head<3>() = rbdStateMeasured.segment<3>(3);
        q.segment<3>(3) = rbdStateMeasured.head<3>();
        q.tail(info.actuatedDofNum) = rbdStateMeasured.segment(6, info.actuatedDofNum);

        const auto& model = wheel_command_pinocchio_interface_.getModel();
        auto& data = wheel_command_pinocchio_interface_.getData();
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
    }

    std::array<vector3_t, 4> StateOCS2::getWheelPositionsWorldHardwareOrder(const vector3_t& basePositionWorld,
                                                                             const matrix3_t& /*R_base_to_world*/) const
    {
        constexpr std::array<size_t, 4> kHardwareToOcs2Contact = {1, 0, 3, 2};

        std::array<vector3_t, 4> wheelPositions{};
        wheelPositions.fill(basePositionWorld);
        const auto& info = ctrl_component_->legged_interface_->getCentroidalModelInfo();
        const auto& model = wheel_command_pinocchio_interface_.getModel();
        const auto& data = wheel_command_pinocchio_interface_.getData();

        for (size_t hw = 0; hw < wheelPositions.size(); ++hw)
        {
            const size_t contact = kHardwareToOcs2Contact[hw];
            if (contact < info.endEffectorFrameIndices.size())
            {
                const auto frameId = info.endEffectorFrameIndices[contact];
                if (frameId < model.frames.size() && frameId < data.oMf.size())
                {
                    const vector3_t wheelPositionWorld = data.oMf[frameId].translation();
                    if (wheelPositionWorld.allFinite())
                    {
                        wheelPositions[hw] = wheelPositionWorld;
                    }
                }
            }
        }

        return wheelPositions;
    }

    bool StateOCS2::getPlannedContactVelocitiesWorldHardwareOrder(
        const vector_t& plannedState,
        const vector_t& plannedInput,
        std::array<vector3_t, 4>& contactVelocitiesWorldHw)
    {
        contactVelocitiesWorldHw.fill(vector3_t::Zero());
        if (!planned_wheel_kinematics_)
        {
            return false;
        }

        const auto& info = ctrl_component_->legged_interface_->getCentroidalModelInfo();
        if (plannedState.size() < info.stateDim || plannedInput.size() < info.inputDim ||
            !plannedState.allFinite() || !plannedInput.allFinite())
        {
            return false;
        }

        planned_wheel_mapping_.setPinocchioInterface(planned_wheel_pinocchio_interface_);
        const vector_t q = planned_wheel_mapping_.getPinocchioJointPosition(plannedState);
        updateCentroidalDynamics(planned_wheel_pinocchio_interface_, info, q);
        const vector_t v = planned_wheel_mapping_.getPinocchioJointVelocity(plannedState, plannedInput);

        const auto& model = planned_wheel_pinocchio_interface_.getModel();
        auto& data = planned_wheel_pinocchio_interface_.getData();
        pinocchio::forwardKinematics(model, data, q, v);
        pinocchio::updateFramePlacements(model, data);

        planned_wheel_kinematics_->setPinocchioInterface(planned_wheel_pinocchio_interface_);
        const auto velocitiesOcs2 = planned_wheel_kinematics_->getVelocity(plannedState, plannedInput);
        if (velocitiesOcs2.size() < contactVelocitiesWorldHw.size())
        {
            return false;
        }

        constexpr std::array<size_t, 4> kHardwareToOcs2Contact = {1, 0, 3, 2};
        for (size_t hw = 0; hw < contactVelocitiesWorldHw.size(); ++hw)
        {
            const size_t contact = kHardwareToOcs2Contact[hw];
            if (contact >= velocitiesOcs2.size() || !velocitiesOcs2[contact].allFinite())
            {
                return false;
            }
            contactVelocitiesWorldHw[hw] = velocitiesOcs2[contact];
        }
        return true;
    }

    std::array<vector3_t, 4> StateOCS2::getWheelRollingDirectionsHardwareOrder(const matrix3_t& R_base_to_world) const
    {
        return getWheelRollingDirectionsHardwareOrderFromInterface(wheel_command_pinocchio_interface_, R_base_to_world);
    }

    std::array<vector3_t, 4> StateOCS2::getWheelRollingDirectionsHardwareOrderFromInterface(
        const PinocchioInterface& pinocchioInterface,
        const matrix3_t& R_base_to_world) const
    {
        constexpr std::array<size_t, 4> kHardwareToOcs2Contact = {1, 0, 3, 2};
        constexpr std::array<scalar_t, 4> kWheelRollingSignHardwareOrder = {1.0, 1.0, 1.0, 1.0};

        std::array<vector3_t, 4> rollingDirections{};
        const vector3_t terrainNormal = getTerrainNormalWorldForWheelCommandOrUnitZ();
        const auto& info = ctrl_component_->legged_interface_->getCentroidalModelInfo();
        const auto& model = pinocchioInterface.getModel();
        const auto& data = pinocchioInterface.getData();

        for (size_t hw = 0; hw < rollingDirections.size(); ++hw)
        {
            vector3_t wheelAxisWorld = R_base_to_world * vector3_t::UnitY();
            const size_t contact = kHardwareToOcs2Contact[hw];
            if (contact < info.endEffectorFrameIndices.size())
            {
                const auto frameId = info.endEffectorFrameIndices[contact];
                if (frameId < model.frames.size() && frameId < data.oMf.size())
                {
                    wheelAxisWorld = data.oMf[frameId].rotation() * vector3_t::UnitY();
                }
            }
            if (!wheelAxisWorld.allFinite() || wheelAxisWorld.norm() < 1e-6)
            {
                wheelAxisWorld = R_base_to_world * vector3_t::UnitY();
            }
            else
            {
                wheelAxisWorld.normalize();
            }

            const vector3_t lateralWorld = getTangentProjectionOrFallback(wheelAxisWorld, terrainNormal);
            vector3_t rollingWorld = lateralWorld.cross(terrainNormal);
            if (!rollingWorld.allFinite() || rollingWorld.norm() < 1e-6)
            {
                rollingWorld = vector3_t::UnitX() - vector3_t::UnitX().dot(terrainNormal) * terrainNormal;
            }
            if (!rollingWorld.allFinite() || rollingWorld.norm() < 1e-6)
            {
                rollingWorld = terrainNormal.unitOrthogonal();
            }
            rollingWorld.normalize();
            rollingDirections[hw] = kWheelRollingSignHardwareOrder[hw] * rollingWorld;
        }

        return rollingDirections;
    }

    void StateOCS2::publishFrictionConeMarkers(const rclcpp::Time& stamp,
                                               size_t planned_mode,
                                               const vector_t& optimized_state,
                                               const vector_t& optimized_input)
    {
        if (!friction_cone_marker_pub_ || optimized_input.size() == 0 || optimized_state.size() == 0) {
            return;
        }

        if (!optimized_input.allFinite() || !optimized_state.allFinite()) {
            return;
        }

        const auto& info = ctrl_component_->legged_interface_->getCentroidalModelInfo();
        if (optimized_input.size() < info.inputDim || optimized_state.size() < info.stateDim) {
            return;
        }

        const auto contactFlags = modeNumber2StanceLeg(planned_mode);
        const auto feetPositions = ctrl_component_->ee_kinematics_->getPosition(optimized_state);

        vector3_t terrainNormal = ctrl_component_->legged_interface_->getSwitchedModelReferenceManagerPtr()->getTerrainNormalWorldOrUnitZ();
        if (!terrainNormal.allFinite() || terrainNormal.norm() < 1e-6) {
            terrainNormal = vector3_t::UnitZ();
        } else {
            terrainNormal.normalize();
        }

        vector3_t terrainX = vector3_t::UnitX() - vector3_t::UnitX().dot(terrainNormal) * terrainNormal;
        if (!terrainX.allFinite() || terrainX.norm() < 1e-6) {
            terrainX = vector3_t::UnitY() - vector3_t::UnitY().dot(terrainNormal) * terrainNormal;
        }
        if (!terrainX.allFinite() || terrainX.norm() < 1e-6) {
            terrainX = terrainNormal.unitOrthogonal();
        }
        terrainX.normalize();
        vector3_t terrainY = terrainNormal.cross(terrainX);
        terrainY.normalize();

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clearMarker;
        clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clearMarker);

        const scalar_t forceScale = std::max<scalar_t>(friction_cone_normal_force_scale_, 1e-3);
        const scalar_t coneHeight = 0.20;
        const scalar_t coneRadius = friction_cone_mu_ * coneHeight;
        const int ringSegments = 24;
        const std::array<const char*, 4> legNames = {"FL", "FR", "RL", "RR"};

        auto toPoint = [](const vector3_t& p) {
            geometry_msgs::msg::Point point;
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();
            return point;
        };

        for (size_t i = 0; i < info.numThreeDofContacts && i < feetPositions.size() && i < contactFlags.size(); ++i) {
            const int baseId = static_cast<int>(100 * i);
            const vector3_t origin = feetPositions[i] + 0.025 * terrainNormal;
            const vector3_t forceWorld = centroidal_model::getContactForces(optimized_input, i, info);
            const scalar_t normalForce = forceWorld.dot(terrainNormal);
            const vector3_t tangentForce = forceWorld - normalForce * terrainNormal;
            const scalar_t tangentNorm = tangentForce.norm();
            const scalar_t margin = friction_cone_mu_ * normalForce - tangentNorm;
            const bool active = contactFlags[i];
            const bool insideCone = active && margin >= 0.0 && normalForce >= 0.0;

            visualization_msgs::msg::Marker cone;
            cone.header.frame_id = "odom";
            cone.header.stamp = stamp;
            cone.ns = "mpc_friction_cone_boundary";
            cone.id = baseId;
            cone.type = visualization_msgs::msg::Marker::LINE_LIST;
            cone.action = active ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
            cone.pose.orientation.w = 1.0;
            cone.scale.x = 0.006;
            cone.color.r = insideCone ? 0.1f : 1.0f;
            cone.color.g = insideCone ? 0.9f : 0.1f;
            cone.color.b = 1.0f;
            cone.color.a = 0.75f;

            const vector3_t ringCenter = origin + coneHeight * terrainNormal;
            for (int k = 0; k < ringSegments; ++k) {
                const scalar_t a0 = 2.0 * M_PI * static_cast<scalar_t>(k) / static_cast<scalar_t>(ringSegments);
                const scalar_t a1 = 2.0 * M_PI * static_cast<scalar_t>(k + 1) / static_cast<scalar_t>(ringSegments);
                const vector3_t p0 = ringCenter + coneRadius * (std::cos(a0) * terrainX + std::sin(a0) * terrainY);
                const vector3_t p1 = ringCenter + coneRadius * (std::cos(a1) * terrainX + std::sin(a1) * terrainY);

                cone.points.push_back(toPoint(p0));
                cone.points.push_back(toPoint(p1));

                if (k % 4 == 0) {
                    cone.points.push_back(toPoint(origin));
                    cone.points.push_back(toPoint(p0));
                }
            }
            markers.markers.push_back(cone);

            visualization_msgs::msg::Marker forceArrow;
            forceArrow.header.frame_id = "odom";
            forceArrow.header.stamp = stamp;
            forceArrow.ns = "mpc_contact_force";
            forceArrow.id = baseId + 1;
            forceArrow.type = visualization_msgs::msg::Marker::ARROW;
            forceArrow.action = active ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
            forceArrow.pose.orientation.w = 1.0;
            forceArrow.scale.x = 0.018;
            forceArrow.scale.y = 0.035;
            forceArrow.scale.z = 0.035;
            forceArrow.points.push_back(toPoint(origin));
            forceArrow.points.push_back(toPoint(origin + forceWorld / forceScale));
            forceArrow.color.r = insideCone ? 0.1f : 1.0f;
            forceArrow.color.g = insideCone ? 1.0f : 0.1f;
            forceArrow.color.b = 0.1f;
            forceArrow.color.a = 0.95f;
            markers.markers.push_back(forceArrow);

            visualization_msgs::msg::Marker capacityRing;
            capacityRing.header.frame_id = "odom";
            capacityRing.header.stamp = stamp;
            capacityRing.ns = "mpc_friction_cone_current_section";
            capacityRing.id = baseId + 2;
            capacityRing.type = visualization_msgs::msg::Marker::LINE_LIST;
            capacityRing.action = active ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
            capacityRing.pose.orientation.w = 1.0;
            capacityRing.scale.x = 0.004;
            capacityRing.color.r = 1.0f;
            capacityRing.color.g = 1.0f;
            capacityRing.color.b = 0.1f;
            capacityRing.color.a = 0.85f;

            if (normalForce > 0.0) {
                const scalar_t sectionHeight = normalForce / forceScale;
                const scalar_t sectionRadius = friction_cone_mu_ * sectionHeight;
                const vector3_t sectionCenter = origin + sectionHeight * terrainNormal;
                for (int k = 0; k < ringSegments; ++k) {
                    const scalar_t a0 = 2.0 * M_PI * static_cast<scalar_t>(k) / static_cast<scalar_t>(ringSegments);
                    const scalar_t a1 = 2.0 * M_PI * static_cast<scalar_t>(k + 1) / static_cast<scalar_t>(ringSegments);
                    const vector3_t p0 = sectionCenter + sectionRadius * (std::cos(a0) * terrainX + std::sin(a0) * terrainY);
                    const vector3_t p1 = sectionCenter + sectionRadius * (std::cos(a1) * terrainX + std::sin(a1) * terrainY);
                    capacityRing.points.push_back(toPoint(p0));
                    capacityRing.points.push_back(toPoint(p1));
                }
            }
            markers.markers.push_back(capacityRing);

            visualization_msgs::msg::Marker text;
            text.header.frame_id = "odom";
            text.header.stamp = stamp;
            text.ns = "mpc_friction_cone_margin";
            text.id = baseId + 3;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = active ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
            text.pose.position = toPoint(origin + (coneHeight + 0.08) * terrainNormal);
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.055;
            text.color.r = insideCone ? 0.1f : 1.0f;
            text.color.g = insideCone ? 1.0f : 0.1f;
            text.color.b = 0.1f;
            text.color.a = 0.95f;
            const char* legName = i < legNames.size() ? legNames[i] : "LEG";
            text.text = std::string(legName) + " margin=" + std::to_string(margin).substr(0, 6) + "N";
            markers.markers.push_back(text);
        }

        friction_cone_marker_pub_->publish(markers);
    }

    /**
     * @brief 检查状态切换条件
     * @return 下一个FSM状态名称
     * @details 状态切换逻辑：
     * 1. 执行安全检查，失败则切换到PASSIVE（被动）状态
     * 2. 根据控制指令判断：指令1切换到PASSIVE，否则保持OCS2状态
     */
    FSMStateName StateOCS2::checkChange()
    {
        // 安全检查：若失败则停止控制器，切换到PASSIVE状态
        if (!safety_checker_->check(ctrl_component_->observation_, optimized_state_, optimized_input_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[Legged Controller] Safety check failed, stopping the controller.");
            return FSMStateName::PASSIVE;
        }

        // 根据控制指令判断状态切换
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            // 指令1：切换到PASSIVE状态（停止主动控制）
            return FSMStateName::PASSIVE;
        default:
            // 其他指令：保持OCS2控制状态
            return FSMStateName::OCS2;
        }
    }
} // namespace ocs2::legged_robot

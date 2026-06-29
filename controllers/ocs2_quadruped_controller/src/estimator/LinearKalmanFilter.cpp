//
// Created by qiayuan on 2022/7/24.
//

// Pinocchio机器人动力学库：正运动学和帧位置更新
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
// 新增：Pinocchio逆动力学和雅克比矩阵
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
// 新增：Eigen矩阵分解（最小二乘）
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <algorithm> // count、clamp等
#include <array>
// 计算质量矩阵M
#include <pinocchio/algorithm/crba.hpp>          
#include <utility>
// 线性卡尔曼滤波器头文件：四足机器人状态估计核心实现
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
// OCS2核心工具：加载配置文件数据
#include <ocs2_core/misc/LoadData.h>
// OCS2机器人工具：旋转导数变换（欧拉角/角速度）
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
// OCS2机器人工具：旋转矩阵变换（四元数/欧拉角转换）
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace ocs2::legged_robot {
    /**
     * @brief 线性卡尔曼滤波器构造函数
     * @param pinocchio_interface: Pinocchio机器人模型接口（包含模型和数据）
     * @param info: 质心模型信息（接触点数量、自由度等）
     * @param ee_kinematics: 末端执行器运动学求解器（足端位姿计算）
     * @param ctrl_component: 控制器接口集合（硬件/传感器数据）
     * @param node: ROS2生命周期节点指针（话题发布）
     * @details 初始化卡尔曼滤波器：
     * 1. 初始化状态估计器基类
     * 2. 配置卡尔曼滤波器维度（状态/观测/接触点）
     * 3. 初始化状态估计值、系统矩阵(A/B/C)、协方差矩阵(Q/R/P)
     * 4. 初始化足端运动学求解器，创建话题发布器
     */
    KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchio_interface,
                                               CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics &ee_kinematics,
                                               CtrlInterfaces &ctrl_component,
                                               const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                                               std::shared_ptr<GaitSchedule> gait_schedule_ptr) // 新增参数
        : StateEstimateBase(std::move(info), ctrl_component, node),  // 初始化状态估计器基类
          pinocchio_interface_(std::move(pinocchio_interface)),      // 转移Pinocchio接口所有权
          ee_kinematics_(ee_kinematics.clone()),                     // 克隆末端执行器运动学求解器
          numContacts_(info_.numThreeDofContacts + info_.numSixDofContacts),  // 总接触点数量（3DOF+6DOF）
          dimContacts_(3 * numContacts_),                            // 接触点维度（每个点3维）
          numState_(6 + dimContacts_),                               // 状态维度：6(基座位姿/速度)+接触点维度
          numObserve_(2 * dimContacts_ + numContacts_),              // 观测维度：2*接触点(位置+速度)+接触点高度
          gait_schedule_ptr_(std::move(gait_schedule_ptr)) { // 初始化新增成员  

        // 初始化卡尔曼滤波器核心变量
        xHat_.setZero(numState_);    // 状态估计值（先验/后验）初始化为0
        ps_.setZero(dimContacts_);   // 足端位置观测值
        vs_.setZero(dimContacts_);   // 足端速度观测值
        a_.setIdentity(numState_, numState_);  // 系统状态转移矩阵A初始化为单位矩阵
        b_.setZero(numState_, 3);              // 系统输入矩阵B初始化为0（输入为IMU加速度）

        // 构建观测矩阵C的基础块
        matrix_t c1(3, 6), c2(3, 6);
        c1 << matrix3_t::Identity(), matrix3_t::Zero();   // c1: [I3, 0] 对应基座位置
        c2 << matrix3_t::Zero(), matrix3_t::Identity();   // c2: [0, I3] 对应基座速度
        c_.setZero(numObserve_, numState_);               // 观测矩阵C初始化为0

        // 填充观测矩阵C：关联状态与观测值
        for (ssize_t i = 0; i < numContacts_; ++i) {
            c_.block(3 * i, 0, 3, 6) = c1;                          // 第i个接触点位置观测关联基座位置
            c_.block(3 * (numContacts_ + i), 0, 3, 6) = c2;          // 第i个接触点速度观测关联基座速度
            // 高度伪观测的线性部分：
            // h_i(x) = p_foot_z - terrain_ref_i
            // 这里 Jacobian 对状态 p_foot_z 的偏导仍为 1，
            // terrain_ref_i 在 update() 中以常量偏置形式加入
            c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0;           
        }
        c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);  // 接触点位置观测修正

        // 初始化协方差矩阵
        q_.setIdentity(numState_, numState_);    // 过程噪声协方差Q（系统模型噪声）
        p_ = 100. * q_;                          // 状态估计协方差P初始化为100*Q（大初始不确定性）
        r_.setIdentity(numObserve_, numObserve_); // 观测噪声协方差R（传感器噪声）
        feet_heights_.setZero(numContacts_);      // 足端高度观测值初始化为0

        // 配置运动学求解器并初始化话题发布器
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_);  // 设置运动学求解器的Pinocchio接口
        initPublishers();                                            // 初始化里程计发布器（基类方法）


        // ========== 新增：GRF/地形估计初始化 ==========
        // 1. 足底接触力（GRF）初始化：4条腿×3维力
        grf_world_.resize(numContacts_);
        grf_body_.resize(numContacts_);
        grf_filter_prev_.resize(numContacts_);
        for (int i = 0; i < numContacts_; ++i) {
            grf_world_[i].setZero();
            grf_body_[i].setZero();
            grf_filter_prev_[i].setZero();
        }

        // 2. 地形倾角初始化：俯仰(Pitch)/横滚(Roll)，单位：弧度
        terrain_pitch_ = 0.0;
        terrain_roll_ = 0.0;
        terrain_normal_world_ = vector3_t::UnitZ();
        terrain_normal_valid_ = false;
        terrain_normal_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("terrain_normal_marker", 10);
        wheel_rolling_frame_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("wheel_rolling_frames", 10);
        // 3. 地形高度初始化
        fused_terrain_height_ = 0.0;
        fused_terrain_height_front_ = 0.0;
        fused_terrain_height_rear_ = 0.0;
        front_rear_height_diff_ = 0.0;

        // 4. 接触检测变量初始化
        contact_prob_.setZero(numContacts_);
        contact_states_.resize(numContacts_, false); // 初始全部为摆动状态
        
        // 5.接触检测卡尔曼滤波初始化（状态维度=腿的数量）
        contact_xHat_.setZero(numContacts_);
        contact_P_.setIdentity(numContacts_, numContacts_);
        contact_P_ *= 0.1; // 初始协方差，代表初始不确定性
        contact_Q_.setIdentity(numContacts_, numContacts_);
        contact_Q_ *= 0.001; // 过程噪声，越小越相信预测
        contact_R_.setIdentity(2*numContacts_, 2*numContacts_);
        contact_R_ *= 0.01;  // 观测噪声，越小越相信测量

        // 6.✅ 关键：绑定KalmanFilter指针到CtrlInterfaces
        ctrl_component.kalman_filter_ptr = this;

    }

    /**
     * @brief 卡尔曼滤波器状态更新主函数
     * @param time: 当前ROS2时间戳（用于里程计消息）
     * @param period: 控制周期（秒）
     * @return 更新后的刚体动力学(RBD)状态
     * @details 核心滤波流程：
     * 1. 更新传感器数据（关节/接触/IMU）
     * 2. 更新系统矩阵(A/B)和过程噪声(Q)（随时间步长变化）
     * 3. 计算足端位姿/速度（基于Pinocchio正运动学）
     * 4. 配置过程/观测噪声（根据接触状态调整权重）
     * 5. 执行卡尔曼滤波（预测+更新）
     * 6. 更新RBD状态并发布里程计消息
     */
    vector_t KalmanFilterEstimate::update(const SystemObservation& observation,const rclcpp::Time &time, const rclcpp::Duration &period) {
        // 1. 更新传感器基础数据（基类方法）
        updateJointStates();   // 更新关节位置/速度
        updateImu();           // 更新IMU数据（四元数/角速度/加速度）

        // 2. 更新卡尔曼滤波器系统矩阵（随时间步长dt变化）
        scalar_t dt = period.seconds();

        // 3.================ 缓存当前时间和步态模式，给后续函数用 ================
        current_time = observation.time;
        const auto& mode_schedule = gait_schedule_ptr_->getModeScheduleConst();
        current_mode = mode_schedule.modeAtTime(current_time);

        // 4.a_b_q_矩阵更新 ---------------
        a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();  // A矩阵：位置-速度关联项（x' = x + v*dt）
        // B矩阵：加速度输入项（x'' = 0.5*a*dt², v' = v + a*dt）
        b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
        b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();
        // Q矩阵：过程噪声协方差（位置/速度/接触点分别配置）
        q_.block(0, 0, 3, 3) = (dt / 20.f) * matrix3_t::Identity();          // 位置过程噪声
        q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * matrix3_t::Identity();  // 速度过程噪声（关联重力）
        q_.block(6, 6, dimContacts_, dimContacts_) = dt * matrix_t::Identity(dimContacts_, dimContacts_);  // 接触点过程噪声

        // 5. 计算足端位姿/速度（基于Pinocchio正运动学）
        const auto &model = pinocchio_interface_.getModel();  // Pinocchio机器人模型
        auto &data = pinocchio_interface_.getData();          // Pinocchio计算数据
        size_t actuatedDofNum = info_.actuatedDofNum;         // 驱动自由度数量

        // 6.构造Pinocchio输入的关节位置/速度（仅包含姿态和关节，位置/线速度设为0）
        vector_t qPino(info_.generalizedCoordinatesNum);
        vector_t vPino(info_.generalizedCoordinatesNum);
        qPino.setZero();
        qPino.segment<3>(3) = rbd_state_.head<3>();  // 设置基座姿态（欧拉角），位置设为原点
        qPino.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum);  // 设置关节位置
        vPino.setZero();
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(qPino.segment<3>(3), rbd_state_.segment<3>(info_.generalizedCoordinatesNum));      // 将全局角速度转换为欧拉角导数（用于Pinocchio）
        vPino.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);  // 设置关节速度

        // 7.执行Pinocchio正运动学计算
        forwardKinematics(model, data, qPino, vPino);       // 计算运动学（位置/速度）
        updateFramePlacements(model, data);                 // 更新所有帧的位姿

        // 8.获取足端位置/速度（相对于基座）
        const auto eePos = ee_kinematics_->getPosition(vector_t());// 足端相对于机身本体坐标系的位置在世界坐标系{s}中的描述
        const auto eeVel = ee_kinematics_->getVelocity(vector_t(), vector_t());// 足端相对于机身本体坐标系的速度在世界坐标系{s}中的描述

        //9.预计算转换坐标系
        const vector3_t euler_zyx = quatToZyx(quat_);
        const matrix3_t R_body_to_world = getRotationMatrixFromZyxEulerAngles(euler_zyx);
        const vector3_t body_pos = xHat_.segment<3>(0);

        //10.================ 读取关节力矩，运行GM观测器 ================
        vector_t tau_cmd(info_.generalizedCoordinatesNum);// 构造关节力矩向量（和你的qPino、vPino维度一致）
        tau_cmd.setZero();
        // 循环12次，遍历所有驱动关节，这里 leg_torque_joint_idx_ 的顺序是按 Pinocchio 驱动关节顺序排列的
        for(size_t j = 0; j < leg_torque_joint_idx_.size(); ++j) {
            int hw_idx = leg_torque_joint_idx_[j]; // 硬件接口索引
            double torque_value = ctrl_component_.joint_effort_state_interface_[hw_idx].get().get_optional().value_or(0.0);
            tau_cmd(6 + j) = torque_value;//映射表第0个元素对应 Pinocchio 的第6个广义坐标

            // // ========== 新增打印：正常读取到力矩 ==========
            // RCLCPP_INFO(
            //     node_->get_logger(),    // ROS2节点日志器
            //     "[Torque Read] j=%zu | 硬件索引=%d | 读取力矩=%.3f N·m | 赋值后值=%.3f N·m",
            //     j,                      // 循环索引
            //     hw_idx,                 // 硬件接口索引
            //     torque_value,           // 从硬件读取的原始力矩值
            //     tau_cmd(6 + j)          // 赋值后tau_cmd的实际值
            // );
        }
        
        //11.运行GM观测器，更新GRF
        updateGMObserver(qPino, vPino, tau_cmd, dt);

        //12.================ 运行概率接触检测 ================
        vector_t foot_heights_rel(numContacts_);
        for(int i = 0; i < numContacts_; ++i)
        {
            vector3_t foot_world = body_pos + eePos[i];
            scalar_t terrain_ref = fused_terrain_height_;
            if (terrain_height_per_leg_.size() == numContacts_) {
                terrain_ref = terrain_height_per_leg_(i);
            }
            foot_heights_rel(i) = foot_world.z() - foot_radius_ - terrain_ref;
        }        
        const auto expected_contact = modeNumber2StanceLeg(current_mode);// 获取步态调度的期望接触状态
        updateContactDetection(expected_contact, foot_heights_rel);// 运行接触检测，更新contact_prob_和contact_states_
        //13.================ 加上足端观测 ================
        feet_heights_ = foot_heights_rel;
        //14.================ 运行地形估计 ================
        estimateTerrainTilt(eePos, contact_prob_, dt,current_mode);
        publishWheelRollingFrameMarkers(time, eePos);
        // 14.1 ================ 为主KF高度伪观测准备每条腿的地形参考高度 ================
        vector_t terrain_ref_per_leg(numContacts_);
        terrain_ref_per_leg.setConstant(fused_terrain_height_);  // 默认退化到全局地形高度

        if (terrain_height_per_leg_.size() == numContacts_) {
            terrain_ref_per_leg = terrain_height_per_leg_;       // 优先使用单腿局部地形高度
        }

        //15 --------------- 原来的轮速读取、xi矩阵构建、主卡尔曼滤波代码全部保留不动 --------------
        //15.1 读取轮速编码器（修正索引映射：wheel_velocities_ 0-3 → FL FR RL RR）
        if (wheel_velocities_.size() != numContacts_) {
            wheel_velocities_.setZero(numContacts_);
        }

        // 关键：定义腿索引→硬件关节索引的映射表（按 FL FR RL RR 顺序）
        // 腿索引0=FL → 硬件索引13
        // 腿索引1=FR → 硬件索引12
        // 腿索引2=RL → 硬件索引15
        // 腿索引3=RR → 硬件索引14                  {13, 12, 15, 14}   验证成功
        std::vector<int> leg_to_wheel_joint_idx = {13, 12, 15, 14};

        // 遍历每条腿（0=FL,1=FR,2=RL,3=RR）
        for (int i = 0; i < numContacts_; ++i) {
            // 从映射表取当前腿对应的硬件关节索引
            int wheel_joint_idx = leg_to_wheel_joint_idx[i];
            
            // 读取轮速（逻辑不变，仅索引来源改）
            if (wheel_joint_idx < ctrl_component_.joint_velocity_state_interface_.size() &&
                ctrl_component_.joint_velocity_state_interface_[wheel_joint_idx].get().get_optional().has_value()) {
                wheel_velocities_(i) = ctrl_component_.joint_velocity_state_interface_[wheel_joint_idx].get().get_optional().value();
            } else {
                wheel_velocities_(i) = 0.0;
            }
        }


        //15.2 配置过程/观测噪声（根据接触状态调整权重）
        matrix_t q = matrix_t::Identity(numState_, numState_);  // 过程噪声矩阵（带缩放因子）
        q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imu_process_noise_position_;        // IMU位置过程噪声缩放
        q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imu_process_noise_velocity_;        // IMU速度过程噪声缩放
        q.block(6, 6, dimContacts_, dimContacts_) = q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_;  // 足端位置过程噪声缩放

        matrix_t r = matrix_t::Identity(numObserve_, numObserve_);  // 观测噪声矩阵（带缩放因子）
        r.block(0, 0, dimContacts_, dimContacts_) = r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;        // 足端位置观测噪声缩放
        r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) = r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;  // 足端速度观测噪声缩放
        r.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) = r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) * footHeightSensorNoise_;  // 足端高度观测噪声缩放


        //15.3初始化噪声增益xi（12x12块对角矩阵）
        matrix_t xi = matrix_t::Identity(dimContacts_, dimContacts_);
        //15.4遍历每条腿，计算信任度并构建xi
        for (int i = 0; i < numContacts_; i++) {
            const int i1 = 3 * i;

            // 计算接触相位、相位信任、高度差、高度信任
            const scalar_t phi_c = calculateContactPhase(i, current_time,current_mode);
            const scalar_t C_phi = calculatePhaseTrust(phi_c, expected_contact[i]);
            const scalar_t body_z = xHat_(2);
            // 足接触点世界系高度（接触点，而不是轮心/足中心）
            const scalar_t foot_contact_z_world = body_z + eePos[i].z() - foot_radius_;
            // 相对局部地形高度：理想接触时应接近0
            const scalar_t z_rel = foot_contact_z_world - terrain_ref_per_leg(i);
            const scalar_t C_z = calculateHeightTrust(z_rel);

            // 构建单腿信任矩阵C_i（3x3，论文公式11）
            matrix3_t C_i = matrix3_t::Zero();
            C_i(0, 0) = C_phi;
            C_i(1, 1) = C_phi;
            C_i(2, 2) = C_phi * C_z;

            // 构建xi的对应块（论文公式12）：xi = I + kappa*(I - C_i)
            const matrix3_t xi_i = matrix3_t::Identity() + kappa_ * (matrix3_t::Identity() - C_i);
            xi.block(i1, i1, 3, 3) = xi_i;

            // 计算足端位置/速度观测值（原有逻辑）
            ps_.segment(i1, 3) = -eePos[i];// 负号是为了套于宪元论文(2.54)特殊处理
            ps_.segment(i1, 3)[2] += foot_radius_;
            
            // 运动学计算的足端速度（机体→世界，匹配论文符号）
            vector3_t v_k = -eeVel[i]; // 负号匹配论文2.54的符号约定

            // 轮速贡献：按每个轮子的terrain-aware rolling direction投影，和MPC/WBC/车轮指令保持一致。
            const vector3_t rolling_direction_world =
                getWheelRollingDirectionWorld(static_cast<size_t>(i), R_body_to_world);
            const vector3_t v_w = rolling_direction_world * wheel_velocities_(i) * foot_radius_;
            // 非支撑腿的融合效果会被xi噪声矩阵自动抑制
            vs_.segment(i1, 3) = v_k + v_w;// 融合速度：ṗ_cp = 运动学速度 + 轮速速度（论文核心融合逻辑）
            // ✅ 新增：打印关键变量（ROS2日志）
            // RCLCPP_INFO(
            //     node_->get_logger(),  // 每100ms打印一次，避免刷屏
            //     "Leg %d: phi_c=%.3f, C_phi=%.3f, z_rel=%.3f, C_z=%.3f, xi_xy=%.3f, xi_z=%.3f",
            //     i, phi_c, C_phi, z_rel, C_z, xi_i(0,0), xi_i(2,2)
            // );

        }

        //15.5应用xi到过程噪声Q（论文公式17）
        q.block(6, 6, dimContacts_, dimContacts_) = q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_ * xi;

        //15.6应用xi到观测噪声R（论文公式24）
        r.block(0, 0, dimContacts_, dimContacts_) = r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_ * xi;
        r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) = r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_ * xi;
        //15.7高度观测噪声单独处理（1维 per leg）
        for (int i = 0; i < numContacts_; i++) {
            const int rIndex3 = 2 * dimContacts_ + i;
            const scalar_t phi_c = calculateContactPhase(i, current_time,current_mode);
            const scalar_t C_phi = calculatePhaseTrust(phi_c, expected_contact[i]);
            const scalar_t body_z = xHat_(2);
            const scalar_t foot_contact_z_world = body_z + eePos[i].z() - foot_radius_;
            const scalar_t z_rel = foot_contact_z_world - terrain_ref_per_leg(i);
            const scalar_t C_z = calculateHeightTrust(z_rel);
            const scalar_t C_height = C_phi * C_z;
            const scalar_t xi_height = 1.0 + kappa_ * (1.0 - C_height);
            r(rIndex3, rIndex3) = r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_)(i, i) * footHeightSensorNoise_ * xi_height;

            // ========== 新增：打印z_rel和关键信息 ==========
            // 可选：计算足端在世界坐标系的位置（更易区分物理腿）
            vector3_t ee_pos_body = eePos[i]; // 足端相对于基座的位置
            vector3_t ee_pos_world = ee_pos_body + xHat_.segment<3>(0); // 转换到世界坐标系

            // 取消节流，实时打印所有腿的状态（不再限制打印频率）
            // RCLCPP_INFO(
            //     node_->get_logger(),  // 移除节流的时钟和时间参数
            //     "Leg %d | z_rel=%.3f m | body_pos=(%.2f, %.2f, %.2f) m | contact=%d",
            //     i,                  // 腿索引对应顺序0-3----------FL FR RL RR
            //     z_rel,               // 足端与地形的高度差（核心判断值）
            //     ee_pos_body.x(),   // 足端世界坐标x（区分前后）
            //     ee_pos_body.y(),   // 足端世界坐标y（区分左右）
            //     ee_pos_body.z(),   // 足端世界坐标z
            //     expected_contact[i] // 是否期望接触（辅助判断）
            // );

        }



        // 16. 卡尔曼滤波器核心计算（预测+更新）
        vector3_t g(0, 0, -9.81);  // 重力加速度向量
        // IMU线加速度转换到全局坐标系（机体→全局 + 重力补偿）
        vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linear_accel_local_ + g;

        // 构建观测向量 y
        // 位置、速度量测保持原样；
        // 高度量测采用“相对地形高度≈0”的伪观测，因此尾部目标值设为0
        vector_t y(numObserve_);
        y.head(dimContacts_) = ps_;
        y.segment(dimContacts_, dimContacts_) = vs_;
        y.tail(numContacts_).setZero();

        // 预测步：先验状态估计 xHat = A*xHat + B*u
        xHat_ = a_ * xHat_ + b_ * accel;
        // 预测步：先验协方差估计 P = A*P*A^T + Q
        matrix_t at = a_.transpose();
        matrix_t pm = a_ * p_ * at + q;

        // 更新步：计算观测残差和卡尔曼增益
        matrix_t cT = c_.transpose();
        matrix_t yModel = c_ * xHat_;          // 模型预测的观测值
        for (int i = 0; i < numContacts_; ++i) {
            const int row = 2 * dimContacts_ + i;
            yModel(row) -= terrain_ref_per_leg(i);
        }

        matrix_t ey = y - yModel;              // 观测残差（实际-预测）
        matrix_t s = c_ * pm * cT + r;         // 残差协方差 S = C*P*C^T + R

        // 卡尔曼增益计算：K = P*C^T*S^{-1}（通过LU分解求解）
        vector_t sEy = s.lu().solve(ey);
        xHat_ += pm * cT * sEy;                // 后验状态估计：xHat = xHat + K*(y - C*xHat)

        // 更新步：后验协方差估计 P = (I - K*C)*P
        matrix_t sC = s.lu().solve(c_);
        p_ = (matrix_t::Identity(numState_, numState_) - pm * cT * sC) * pm;

        // 协方差矩阵对称化（数值稳定性）
        matrix_t pt = p_.transpose();
        p_ = (p_ + pt) / 2.0;

        // 注释：协方差裁剪逻辑（可选，用于限制位置协方差过大）
        //  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
        //    p_.block(0, 2, 2, 16).setZero();
        //    p_.block(2, 0, 16, 2).setZero();
        //    p_.block(0, 0, 2, 2) /= 10.;
        //  }

        // 17. 更新RBD状态的线性部分（位置/速度）
        updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

        // 18. 生成并发布里程计消息
        auto odom = getOdomMsg();              // 构建里程计消息
        odom.header.stamp = time;              // 设置时间戳
        odom.header.frame_id = "odom";         // 父坐标系：odom
        odom.child_frame_id = "base";          // 子坐标系：base
        publishMsgs(odom);                     // 发布里程计/位姿消息

        return rbd_state_;                     // 返回更新后的RBD状态
    }




    scalar_t KalmanFilterEstimate::calculateContactPhase(size_t leg, scalar_t time,size_t current_mode) const {

        // 1. 调用OCS2原生接口获取每条腿的接触相位
        const auto contact_phases = ocs2::legged_robot::getContactPhasePerLeg(time, gait_schedule_ptr_->getModeScheduleConst());
        const auto& leg_phase = contact_phases[leg];//不清楚是否对上

        // 2. 处理完全支撑相（OCS2原生返回0.0，我们转为0.5以匹配论文的C_phi=1）
        if (current_mode==15) { // 无限持续的支撑相（站立模式）
            return 0.5; 
        }

        // 3. 处理正常步态的相位（非站立模式）
        if (leg_phase.phase < 0.0) { // 摆动相
            return 0.0;
        } else { // 支撑相，返回归一化相位
            return leg_phase.phase;
        }
    }



    scalar_t KalmanFilterEstimate::calculatePhaseTrust(scalar_t phi_c, bool expected_contact) const {
        if (!expected_contact) return 0.0; // 期望摆动相，信任度为0

        // ✅ 修正：匹配论文相位信任度公式
        const scalar_t hat_s_=1.0;
        const scalar_t term1 = std::erf(4.0 * (phi_c / W_ - 0.5));  // 核心修正：2.0 → 0.5
        const scalar_t term2 = std::erf(4.0 * ((1.0 - phi_c) / W_ - 0.5)); // 核心修正：2.0 → 0.5
        const scalar_t C_phi = (hat_s_ / 2.0) * (term1 + term2); // 修正：hat_s → hat_s_（类成员变量）

        // 限制在[0,1]，避免数值溢出
        return std::max(0.0, std::min(1.0, C_phi));
    }




    scalar_t KalmanFilterEstimate::calculateHeightTrust(scalar_t z_rel) const {
        // 论文公式(9)：C_z = exp(-k*z_rel²)，k根据z_rel符号选择
        scalar_t C_z;
        if (z_rel >= 0.0) {
            C_z = std::exp(-k_plus_ * z_rel * z_rel);
        } else {
            C_z = std::exp(-k_minus_ * z_rel * z_rel);
        }
        return std::max(0.0, std::min(1.0, C_z)); // 限制在[0,1]
    }


        

    void KalmanFilterEstimate::updateGMObserver(
        const vector_t& q,
        const vector_t& v,
        const vector_t& tau,
        scalar_t dt)
    {
        const auto& model = pinocchio_interface_.getModel();
        auto& data = pinocchio_interface_.getData();

        const int nv = model.nv;
        const int joint_dim = info_.actuatedDofNum; // 12维关节

        /* ----- 1. 质量矩阵 M(q)（PDF定义：CRBA+对称化） ----- */
        pinocchio::crba(model, data, q);
        data.M.triangularView<Eigen::StrictlyLower>() =
            data.M.transpose().triangularView<Eigen::StrictlyLower>();

        /* ----- 2. 广义动量 p = Mv（PDF定义+数值保护） ----- */
        vector_t p = data.M * v;
        vector_t p_joint = p.tail(joint_dim);
        // PDF工程要求：NaN/Inf置0
        p_joint = p_joint.unaryExpr([](scalar_t x) {
            return std::isnan(x) || std::isinf(x) ? 0.0 : x;
        });

        /* ----- 3. 非线性项 Cv+g（PDF定义+数值保护） ----- */
        vector_t a_zero = vector_t::Zero(nv);
        vector_t h = pinocchio::rnea(model, data, q, v, a_zero);
        vector_t g = pinocchio::rnea(model, data, q, vector_t::Zero(nv), a_zero);
        vector_t Cv = h - g;

        vector_t Cv_joint = Cv.tail(joint_dim);
        vector_t g_joint = g.tail(joint_dim);
        vector_t tau_joint = tau.tail(joint_dim);
        // PDF工程要求：数值保护
        Cv_joint = Cv_joint.unaryExpr([](scalar_t x) { return std::isnan(x) || std::isinf(x) ? 0.0 : x; });
        g_joint = g_joint.unaryExpr([](scalar_t x) { return std::isnan(x) || std::isinf(x) ? 0.0 : x; });
        tau_joint = tau_joint.unaryExpr([](scalar_t x) { return std::isnan(x) || std::isinf(x) ? 0.0 : x; });

        /* ----- 4. GM观测器核心（PDF离散公式） ----- */
        const scalar_t alpha = 2 * M_PI * gm_cutoff_freq_;
        const scalar_t gamma = std::exp(-alpha * dt);

        // PDF核心公式：输入项（符号已正确）
        vector_t input = tau_joint - Cv_joint - g_joint + alpha * p_joint;
        // PDF工程要求：输入项数值保护+裁剪
        const scalar_t max_input = 200.0;
        input = input.unaryExpr([max_input](scalar_t x) {
            if (std::isnan(x) || std::isinf(x)) return 0.0;
            return std::clamp(x, -max_input, max_input);
        });

        // PDF工程要求：滤波状态初始化（首次运行）
        if (gm_filter_state_.size() != joint_dim) {
            gm_filter_state_ = vector_t::Zero(joint_dim);
        }
        // 滤波状态数值保护
        gm_filter_state_ = gm_filter_state_.unaryExpr([](scalar_t x) {
            return std::isnan(x) || std::isinf(x) ? 0.0 : x;
        });
        // PDF离散滤波公式
        gm_filter_state_ = gamma * gm_filter_state_ + (1 - gamma) * input;

        // PDF定义：扰动力矩 + 物理约束裁剪（四足关节力矩±35N·m）
        vector_t tau_dist = alpha * p_joint - gm_filter_state_;
        const scalar_t max_tau = 30.0;
        tau_dist = tau_dist.unaryExpr([max_tau](scalar_t x) {
            if (std::isnan(x) || std::isinf(x)) return 0.0;
            return std::clamp(x, -max_tau, max_tau);
        });

        /* ----- 5. GRF计算（PDF工程实现：SVD伪逆+正则化） ----- */
        const matrix3_t R_body2world = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_));
        // PDF工程要求：GRF滤波初始化
        if (grf_filter_prev_.empty()) {
            grf_filter_prev_.resize(numContacts_, vector3_t::Zero());
        }

        for(int leg=0; leg<numContacts_; ++leg)
        {
            // 5.1 提取单腿扰动力矩
            vector3_t tau_leg = tau_dist.segment(3*leg, 3);

            // 5.2 计算足端线速度雅克比（PDF要求：LOCAL_WORLD_ALIGNED）
            pinocchio::Data::Matrix6x J(6, nv);
            J.setZero();
            pinocchio::computeFrameJacobian(
                model,
                data,
                q,
                info_.endEffectorFrameIndices[leg],
                pinocchio::LOCAL_WORLD_ALIGNED, // 修正：线速度雅克比在世界系对齐
                J);

            // 5.3 提取单腿线速度雅克比（修正：topRows(3)是线速度，不是角速度）
            int col = 6 + 3*leg;
            matrix3_t J_leg = J.topRows(3).middleCols(col, 3); // 更准确的列提取
            // PDF工程要求：雅克比数值保护
            J_leg = J_leg.unaryExpr([](scalar_t x) {
                return std::isnan(x) || std::isinf(x) ? 0.0 : x;
            });

            // 5.4 SVD伪逆求解GRF（PDF工程核心：正则化避免奇异）
            Eigen::JacobiSVD<matrix3_t> svd(J_leg.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
            const scalar_t min_sing = 0.005; // PDF推荐正则化阈值
            matrix3_t S_inv = svd.singularValues().asDiagonal();
            for (int j=0; j<3; ++j) {
                S_inv(j,j) = (svd.singularValues()(j) > min_sing) ? 1.0/svd.singularValues()(j) : 1.0/min_sing;
            }
            matrix3_t J_pinv = svd.matrixV() * S_inv * svd.matrixU().transpose();

            // 5.5 计算机体坐标系GRF（符号已修正：足对地）
            vector3_t f_body = J_pinv * tau_leg;
            f_body = -f_body; // 地对足 → 足对地（Z+向下）
            // PDF工程要求：GRF物理裁剪（四足±50N）
            const scalar_t max_grf = 180.0;
            f_body = f_body.unaryExpr([max_grf](scalar_t x) {
                return std::clamp(x, -max_grf, max_grf);
            });

            // 5.6 转换到世界坐标系 + 低通滤波（PDF工程要求）
            vector3_t f_world = R_body2world * f_body;
            f_world = grf_filter_gain_ * f_world + (1 - grf_filter_gain_) * grf_filter_prev_[leg];
            grf_filter_prev_[leg] = f_world;

            // 5.7 存储结果
            grf_body_[leg] = f_body;
            grf_world_[leg] = f_world;

            // // 调试打印（PDF工程要求）
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 100,
            //     "[GM] Leg%d | tau=%.2f | Fz=%.2f", leg, tau_leg.norm(), f_world.z());
        }
    }







    // ================ 新增：论文概率接触检测实现 ================
    // 输入：expected_contact=步态调度的期望接触状态，foot_heights=每条腿的足端相对地形高度
    void KalmanFilterEstimate::updateContactDetection(const std::array<bool, 4>& expected_contact, const vector_t& foot_heights) {
        const size_t n = numContacts_; // 腿的数量，一般是4
        
        // ---------------- 1. 预测步（对应论文Section IV-A） ----------------
        // 论文里的状态转移矩阵A=0，输入矩阵B=I，所以先验状态直接等于输入u
        vector_t u(n); // 输入u=基于步态相位的接触概率先验
        for (int i = 0; i < n; ++i) {
            scalar_t phi_c = calculateContactPhase(i, current_time, current_mode);
            //对应论文的P(c|s_phi, phi)
            u(i) = calculatePhaseTrust(phi_c, expected_contact[i]);
        }
        vector_t xHat_minus = u; // 先验状态估计
        matrix_t P_minus = contact_Q_; // 先验协方差

        // ---------------- 2. 构造测量向量（对应论文Section IV-B） ----------------
        // 测量值z = [高度概率; 力概率]，对应论文的两个测量模型
        vector_t z(2 * n);
        // 论文Table I里的高斯参数，直接用论文推荐值
        const scalar_t mu_zg = 0.0;     // 地面高度均值
        const scalar_t sigma_zg = 0.16;  // 地面高度标准差
        const scalar_t mu_fc = feet_force_threshold_;    // 接触力均值（N，根据你的机器人重量调整）
        const scalar_t sigma_fc = 5.0;  // 接触力标准差

        for (int i = 0; i < n; ++i) {
            // 测量1：足端高度概率（对应论文公式18）
            scalar_t p_z = foot_heights(i);
            z(i) = 0.5 * (1 + std::erf( (mu_zg - p_z) / (sigma_zg * std::sqrt(2)) ));

            // 测量2：GM力估计的接触概率（对应论文公式22）
            scalar_t f_z = -grf_world_[i].z(); // 取Z向的接触力
            z(n + i) = 0.5 * (1 + std::erf( (f_z - mu_fc) / (sigma_fc * std::sqrt(2)) ));

            //  // ========== 新增：打印调试信息 ==========
            // RCLCPP_INFO(
            //     node_->get_logger(), // 每200ms打印一次，避免刷屏
            //     "[Contact Detect] Leg %d | Height=%.3fm (Prob=%.2f) | GRF_Z=%.1fN (Prob=%.2f)",
            //     i,                  // 腿编号
            //     p_z,                // 足端高度原始值
            //     z(i),               // 高度计算出的概率
            //     f_z,                // GRF_Z 原始值
            //     z(n + i)            // 力计算出的概率
            // );
        }

        // ---------------- 3. 更新步（卡尔曼滤波融合） ----------------
        // 观测矩阵H = [I; I]，对应论文里两个测量模型都直接观测接触概率
        matrix_t H(2 * n, n);
        H << matrix_t::Identity(n, n), matrix_t::Identity(n, n);

        // 计算卡尔曼增益
        matrix_t Ht = H.transpose();
        matrix_t S = H * P_minus * Ht + contact_R_;
        matrix_t K = P_minus * Ht * S.inverse();

        // 计算后验状态（最终的接触概率）
        vector_t y_model = H * xHat_minus;
        contact_xHat_ = xHat_minus + K * (z - y_model);
        
        // 更新后验协方差，对称化保证数值稳定
        contact_P_ = (matrix_t::Identity(n, n) - K * H) * P_minus;
        contact_P_ = (contact_P_ + contact_P_.transpose()) / 2.0;

        // ---------------- 4. 滞环二值化（工程优化，避免接触状态抖动） ----------------
        for (int i = 0; i < n; ++i) {
            // 把概率限制在0~1之间，避免数值异常
            contact_prob_(i) = std::max(0.0, std::min(1.0, contact_xHat_(i)));
            
            // 滞环逻辑：超过高阈值判定着地，低于低阈值判定离地，中间保持上一状态
            if (contact_prob_(i) > threshold_high_) {
                contact_states_[i] = true;
            } else if (contact_prob_(i) < threshold_low_) {
                contact_states_[i] = false;
            }
            // 中间区间：保持上一时刻的状态不变
        }
    }




void KalmanFilterEstimate::publishTerrainNormalMarker(const rclcpp::Time& stamp,
                                                      const vector3_t& normal_world,
                                                      const vector3_t& origin_world) {
    if (!terrain_normal_marker_pub_) {
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = stamp;
    marker.ns = "terrain_estimation";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    geometry_msgs::msg::Point start;
    start.x = origin_world.x();
    start.y = origin_world.y();
    start.z = origin_world.z();

    geometry_msgs::msg::Point end;
    const scalar_t marker_length = 0.35;
    end.x = origin_world.x() + marker_length * normal_world.x();
    end.y = origin_world.y() + marker_length * normal_world.y();
    end.z = origin_world.z() + marker_length * normal_world.z();

    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.color.r = 0.1f;
    marker.color.g = 0.8f;
    marker.color.b = 1.0f;
    marker.color.a = 0.9f;

    terrain_normal_marker_pub_->publish(marker);
}

vector3_t KalmanFilterEstimate::getTerrainNormalWorldForWheelFusionOrUnitZ() const {
    if (!terrain_normal_valid_) {
        return vector3_t::UnitZ();
    }

    vector3_t terrainNormal = terrain_normal_world_;
    if (!terrainNormal.allFinite() || terrainNormal.norm() < 1e-6) {
        return vector3_t::UnitZ();
    }

    terrainNormal.normalize();
    if (terrainNormal.z() < 0.0) {
        terrainNormal = -terrainNormal;
    }

    constexpr scalar_t kMinNormalZForWheelFusion = 0.70;
    if (terrainNormal.z() < kMinNormalZForWheelFusion) {
        return vector3_t::UnitZ();
    }

    return terrainNormal;
}

vector3_t KalmanFilterEstimate::getTangentProjectionOrFallback(const vector3_t& direction, const vector3_t& normal) {
    vector3_t tangent = direction - direction.dot(normal) * normal;
    if (tangent.allFinite() && tangent.norm() > 1e-6) {
        return tangent.normalized();
    }

    tangent = vector3_t::UnitY() - vector3_t::UnitY().dot(normal) * normal;
    if (tangent.allFinite() && tangent.norm() > 1e-6) {
        return tangent.normalized();
    }

    tangent = vector3_t::UnitX() - vector3_t::UnitX().dot(normal) * normal;
    if (tangent.allFinite() && tangent.norm() > 1e-6) {
        return tangent.normalized();
    }

    return normal.unitOrthogonal().normalized();
}

vector3_t KalmanFilterEstimate::getWheelRollingDirectionWorld(size_t contactIndex,
                                                              const matrix3_t& R_body_to_world) const {
    constexpr std::array<scalar_t, 4> kWheelRollingSignOcs2Order = {1.0, 1.0, 1.0, 1.0};

    const vector3_t terrainNormal = getTerrainNormalWorldForWheelFusionOrUnitZ();
    const auto& model = pinocchio_interface_.getModel();
    const auto& data = pinocchio_interface_.getData();

    vector3_t wheelAxisWorld = R_body_to_world * vector3_t::UnitY();
    if (contactIndex < info_.endEffectorFrameIndices.size()) {
        const auto frameId = info_.endEffectorFrameIndices[contactIndex];
        if (frameId < model.frames.size() && frameId < data.oMf.size()) {
            wheelAxisWorld = data.oMf[frameId].rotation() * vector3_t::UnitY();
        }
    }

    if (!wheelAxisWorld.allFinite() || wheelAxisWorld.norm() < 1e-6) {
        wheelAxisWorld = R_body_to_world * vector3_t::UnitY();
    } else {
        wheelAxisWorld.normalize();
    }

    const vector3_t lateralWorld = getTangentProjectionOrFallback(wheelAxisWorld, terrainNormal);
    vector3_t rollingWorld = lateralWorld.cross(terrainNormal);
    if (!rollingWorld.allFinite() || rollingWorld.norm() < 1e-6) {
        rollingWorld = R_body_to_world * vector3_t::UnitX();
        rollingWorld = rollingWorld - rollingWorld.dot(terrainNormal) * terrainNormal;
    }
    if (!rollingWorld.allFinite() || rollingWorld.norm() < 1e-6) {
        rollingWorld = terrainNormal.unitOrthogonal();
    }
    rollingWorld.normalize();

    const scalar_t sign = contactIndex < kWheelRollingSignOcs2Order.size()
                              ? kWheelRollingSignOcs2Order[contactIndex]
                              : 1.0;
    return sign * rollingWorld;
}

void KalmanFilterEstimate::publishWheelRollingFrameMarkers(const rclcpp::Time& stamp,
                                                           const std::vector<vector3_t>& ee_pos_base_to_foot_world) {
    if (!wheel_rolling_frame_marker_pub_) {
        return;
    }

    const vector3_t terrain_normal = getTerrainNormalWorldForWheelFusionOrUnitZ();

    const auto& model = pinocchio_interface_.getModel();
    const auto& data = pinocchio_interface_.getData();

    visualization_msgs::msg::MarkerArray markers;
    const vector3_t body_pos = xHat_.segment<3>(0);
    const vector3_t wheel_axis_local(0.0, 1.0, 0.0);
    const scalar_t marker_length = 0.22;
    const std::array<const char*, 4> leg_names = {"FL", "FR", "RL", "RR"};

    auto makeArrow = [&](int id, const vector3_t& origin, const vector3_t& direction,
                         float r, float g, float b, const std::string& ns) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.018;
        marker.scale.y = 0.035;
        marker.scale.z = 0.035;

        geometry_msgs::msg::Point start;
        start.x = origin.x();
        start.y = origin.y();
        start.z = origin.z();

        geometry_msgs::msg::Point end;
        end.x = origin.x() + marker_length * direction.x();
        end.y = origin.y() + marker_length * direction.y();
        end.z = origin.z() + marker_length * direction.z();

        marker.points.push_back(start);
        marker.points.push_back(end);
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.95f;
        return marker;
    };

    auto makeSphere = [&](int id, const vector3_t& position,
                          float r, float g, float b, const std::string& ns) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.045;
        marker.scale.y = 0.045;
        marker.scale.z = 0.045;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.9f;
        return marker;
    };

    for (size_t i = 0; i < info_.numThreeDofContacts && i < ee_pos_base_to_foot_world.size(); ++i) {
        const auto frame_id = info_.endEffectorFrameIndices[i];
        if (frame_id >= model.frames.size()) {
            continue;
        }

        vector3_t wheel_axis_world = data.oMf[frame_id].rotation() * wheel_axis_local;
        if (wheel_axis_world.norm() < 1e-6 || !std::isfinite(wheel_axis_world.norm())) {
            continue;
        }
        wheel_axis_world.normalize();

        vector3_t lateral_world = wheel_axis_world - wheel_axis_world.dot(terrain_normal) * terrain_normal;
        if (lateral_world.norm() < 1e-6 || !std::isfinite(lateral_world.norm())) {
            continue;
        }
        lateral_world.normalize();

        // 轮子允许滚动方向由“轮轴在地形切平面内的侧向方向 × 地形法向”确定。
        // 平地且轮轴为局部 +Y 时，该定义给出 +X 滚动方向；若实物轮正转相反，后续用每轮 sign 修正。
        vector3_t rolling_world = lateral_world.cross(terrain_normal);
        if (rolling_world.norm() < 1e-6 || !std::isfinite(rolling_world.norm())) {
            continue;
        }
        rolling_world.normalize();

        const vector3_t wheel_frame_center = body_pos + ee_pos_base_to_foot_world[i];
        const vector3_t marker_base = wheel_frame_center + 0.06 * terrain_normal;
        const int base_id = static_cast<int>(10 * i);

        // 蓝色 wheel axis 和紫色 lateral 在平地/小坡度时会几乎重合，
        // 因此沿地形法向错开起点，避免 RViz 中互相遮挡，看起来像少了一个箭头。
        markers.markers.push_back(makeSphere(base_id + 3, wheel_frame_center, 1.0f, 1.0f, 1.0f, "wheel_frame_center_white"));
        markers.markers.push_back(makeArrow(base_id + 0, marker_base + 0.00 * terrain_normal, wheel_axis_world, 0.1f, 0.3f, 1.0f, "wheel_axis_blue"));
        markers.markers.push_back(makeArrow(base_id + 1, marker_base + 0.04 * terrain_normal, lateral_world, 1.0f, 0.1f, 1.0f, "wheel_lateral_magenta"));
        markers.markers.push_back(makeArrow(base_id + 2, marker_base + 0.08 * terrain_normal, rolling_world, 0.1f, 1.0f, 0.1f, "wheel_rolling_green"));

        const char* leg_name = i < leg_names.size() ? leg_names[i] : "LEG";
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                              "[Wheel Frame] %s/%zu axis_W=(%.2f %.2f %.2f) lateral_W=(%.2f %.2f %.2f) rolling_W=(%.2f %.2f %.2f) normal_W=(%.2f %.2f %.2f)",
                              leg_name, i,
                              wheel_axis_world.x(), wheel_axis_world.y(), wheel_axis_world.z(),
                              lateral_world.x(), lateral_world.y(), lateral_world.z(),
                              rolling_world.x(), rolling_world.y(), rolling_world.z(),
                              terrain_normal.x(), terrain_normal.y(), terrain_normal.z());
    }

    wheel_rolling_frame_marker_pub_->publish(markers);
}

void KalmanFilterEstimate::estimateTerrainTilt(const std::vector<vector3_t>& eePos,
                                               const ocs2::vector_t& contact_prob,
                                               scalar_t dt,
                                               int current_mode) {
    (void)dt;

    std::vector<vector3_t> support_feet_world;   // 当前支撑腿瞬时世界坐标（用于融合高度）
    std::vector<int> support_leg_indices;        // 当前支撑腿索引（0-3）

    const vector3_t body_pos = xHat_.segment<3>(0);

    // 计算机体当前姿态（ZYX欧拉角）
    const Eigen::Matrix<scalar_t, 3, 1> euler_zyx = quatToZyx(quat_);
    const matrix3_t R_body_to_world = getRotationMatrixFromZyxEulerAngles(euler_zyx);

    // 【鲁棒性】输入长度校验
    if (contact_prob.size() != eePos.size() || eePos.size() != numContacts_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "[Terrain Est] 输入长度不匹配！contact_prob=%zu, eePos=%zu, numContacts_=%zu",
                             contact_prob.size(), eePos.size(), numContacts_);
        return;
    }

    // 初始化单腿地形高度
    if (terrain_height_per_leg_.size() != numContacts_) {
        terrain_height_per_leg_.setZero(numContacts_);
    }

    // =========================
    // 新增：TROT 落脚点锁存缓存（模仿旧代码 Pn_td 的思想）
    // =========================
    static std::vector<vector3_t> latched_foothold_world;
    static std::vector<bool> latched_valid;
    static std::vector<bool> last_contact_state;

    if (latched_foothold_world.size() != static_cast<size_t>(numContacts_)) {
        latched_foothold_world.assign(numContacts_, vector3_t::Zero());
        latched_valid.assign(numContacts_, false);
        last_contact_state.assign(numContacts_, false);
    }

    // =========================
    // 1. 收集当前支撑腿瞬时世界坐标 + 更新单腿地形高度 + 锁存新落脚点
    // =========================
    for (int i = 0; i < numContacts_; ++i) {
        const bool stable_contact = (contact_prob(i) > threshold_high_);

        if (stable_contact) {
            // 你已经确认：eePos 是“相对机身的位置，在世界系中表达”
            const vector3_t p_leg_world = body_pos + eePos[i];

            // 当前支撑腿瞬时位置，后面仍用于 fused_terrain_height_
            support_feet_world.push_back(p_leg_world);
            support_leg_indices.push_back(i);

            // 更新单腿地形高度
            const scalar_t leg_terrain_height = p_leg_world.z() - foot_radius_;
            if (std::isfinite(leg_terrain_height)) {
                terrain_height_per_leg_(i) =
                    terrain_height_filter_gain_ * leg_terrain_height +
                    (1.0 - terrain_height_filter_gain_) * terrain_height_per_leg_(i);
            }

            // ========== 新增：接触上升沿锁存该腿落脚点 ==========
            // 只有从“非接触 -> 稳定接触”这一拍，才更新锁存点
            if (!last_contact_state[i]||current_mode==15) {
                latched_foothold_world[i] = p_leg_world;
                latched_valid[i] = true;
            }
        }

        // 更新上一拍接触状态
        last_contact_state[i] = stable_contact;
    }

    // =========================
    // 2. 构造用于拟合地形的点集
    //    - TROT 下优先用四条腿最近一次落脚点缓存
    //    - 其他模式仍沿用当前支撑腿瞬时位置
    // =========================
    std::vector<vector3_t> fit_feet_world = support_feet_world;
    std::vector<int> fit_leg_indices = support_leg_indices;

    if (current_mode == 6 || current_mode == 9) {
        bool all_latched = true;
        for (int i = 0; i < numContacts_; ++i) {
            if (!latched_valid[i]) {
                all_latched = false;
                break;
            }
        }

        // 只有四条腿都至少锁存过一次落脚点，才切到“缓存点拟合”
        if (all_latched) {
            fit_feet_world.clear();
            fit_leg_indices.clear();

            for (int i = 0; i < numContacts_; ++i) {
                fit_feet_world.push_back(latched_foothold_world[i]);
                fit_leg_indices.push_back(i);
            }
        }
    }

    double roll = 0.0;
    double pitch = 0.0;
    bool is_estimated = false;

    // =========================
    // 3. 地形倾角估计
    // =========================

    // -------- 3.1 三脚及以上：最小二乘拟合平面 --------
    if (fit_feet_world.size() >= 3) {
        Eigen::MatrixXd A(fit_feet_world.size(), 3);
        Eigen::VectorXd b_vec(fit_feet_world.size());

        for (size_t j = 0; j < fit_feet_world.size(); ++j) {
            A(j, 0) = fit_feet_world[j].x();
            A(j, 1) = fit_feet_world[j].y();
            A(j, 2) = 1.0;
            b_vec(j) = -fit_feet_world[j].z();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (svd.rank() >= 2) {
            Eigen::Vector3d params = svd.solve(b_vec);
            double a = params(0);
            double b_param = params(1);

            // A * [a, b, c]^T = -z 表示 a*x + b*y + c + z = 0，
            // 因此上朝向地形法向为 [a, b, 1]。不要再对水平分量取反，
            // 否则斜坡上的前后/左右法向会反。
            vector3_t normal(a, b_param, 1.0);
            if (normal.norm() > 1e-6 && std::isfinite(normal.norm())) {
                normal.normalize();

                roll = std::atan2(-normal.y(), normal.z());
                pitch = std::atan2(normal.x(),
                                   std::sqrt(normal.y() * normal.y() + normal.z() * normal.z()));
                is_estimated = true;
            }
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                                 "[Terrain Est] 拟合平面退化：点集秩不足");
        }
    }
    // -------- 3.2 TROT 两对角支撑：原有两点逻辑保留（只有在还没收齐四个落脚点时才可能进来） --------
    else if ((current_mode == 6 || current_mode == 9) && fit_feet_world.size() == 2) {
        const bool is_diag1 =
            (fit_leg_indices[0] == 0 && fit_leg_indices[1] == 3) ||
            (fit_leg_indices[0] == 3 && fit_leg_indices[1] == 0);

        const bool is_diag2 =
            (fit_leg_indices[0] == 1 && fit_leg_indices[1] == 2) ||
            (fit_leg_indices[0] == 2 && fit_leg_indices[1] == 1);

        if (is_diag1 || is_diag2) {
            const vector3_t p1 = fit_feet_world[0];
            const vector3_t p2 = fit_feet_world[1];

            const double dx = p2.x() - p1.x();
            const double dy = p2.y() - p1.y();
            const double dz = p2.z() - p1.z();

            const double norm_xy = std::sqrt(dx * dx + dy * dy);
            if (norm_xy > 0.01) {
                // 两点退化估计时同样遵循上朝向法向的符号：坡面高度沿 +x/+y 增大时，
                // 法向水平分量应指向 -x/-y。
                pitch = -std::atan2(dz * dx, norm_xy * norm_xy);
                roll  = -std::atan2(dz * dy, norm_xy * norm_xy);
                is_estimated = true;
            }
        }
    }

    // =========================
    // 4. 将“世界系地形”转换到“机体航向对齐坐标系”
    //    这部分保留你已经验证通过的版本
    // =========================
    if (is_estimated) {
        // 先由世界系下的 pitch / roll 还原地形法向量
        vector3_t terrain_normal_world;
        terrain_normal_world.x() = std::sin(pitch);
        terrain_normal_world.y() = -std::sin(roll) * std::cos(pitch);
        terrain_normal_world.z() =  std::cos(roll) * std::cos(pitch);

        if (terrain_normal_world.norm() > 1e-6 && std::isfinite(terrain_normal_world.norm())) {
            terrain_normal_world.normalize();
            if (terrain_normal_world.z() < 0.0) {
                terrain_normal_world = -terrain_normal_world;
            }

            if (terrain_normal_valid_) {
                terrain_normal_world_ = terrain_filter_gain_ * terrain_normal_world +
                                        (1.0 - terrain_filter_gain_) * terrain_normal_world_;
                if (terrain_normal_world_.norm() > 1e-6 && std::isfinite(terrain_normal_world_.norm())) {
                    terrain_normal_world_.normalize();
                } else {
                    terrain_normal_world_ = terrain_normal_world;
                }
            } else {
                terrain_normal_world_ = terrain_normal_world;
                terrain_normal_valid_ = true;
            }

            const vector3_t marker_origin = body_pos + vector3_t(0.0, 0.0, 0.25);
            publishTerrainNormalMarker(node_->now(), terrain_normal_world_, marker_origin);
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                  "[Terrain Est] normal_W=(%.3f, %.3f, %.3f), pitch=%.2f deg, roll=%.2f deg, support=%zu, mode=%d",
                                  terrain_normal_world_.x(), terrain_normal_world_.y(), terrain_normal_world_.z(),
                                  terrain_pitch_ * 180.0 / M_PI, terrain_roll_ * 180.0 / M_PI,
                                  fit_feet_world.size(), current_mode);

            // 只取 yaw，把世界系法向量转到“机体航向对齐系”
            const scalar_t yaw = euler_zyx(0);

            matrix3_t R_world_to_heading;
            R_world_to_heading <<
                std::cos(yaw),  std::sin(yaw), 0.0,
               -std::sin(yaw),  std::cos(yaw), 0.0,
                0.0,            0.0,           1.0;

            vector3_t terrain_normal_heading = R_world_to_heading * terrain_normal_world;

            if (terrain_normal_heading.norm() > 1e-6 && std::isfinite(terrain_normal_heading.norm())) {
                terrain_normal_heading.normalize();

                if (terrain_normal_heading.z() < 0.0) {
                    terrain_normal_heading = -terrain_normal_heading;
                }

                // 在“机体航向对齐系”下提取前后坡度 / 左右坡度分量
                const scalar_t roll_heading =
                    std::atan2(-terrain_normal_heading.y(), terrain_normal_heading.z());

                const scalar_t pitch_heading =
                    std::atan2(terrain_normal_heading.x(),
                               std::sqrt(terrain_normal_heading.y() * terrain_normal_heading.y() +
                                         terrain_normal_heading.z() * terrain_normal_heading.z()));

                if (std::isfinite(roll_heading) && std::isfinite(pitch_heading)) {
                    terrain_roll_ =
                        terrain_filter_gain_ * roll_heading +
                        (1.0 - terrain_filter_gain_) * terrain_roll_;

                    terrain_pitch_ =
                        terrain_filter_gain_ * pitch_heading +
                        (1.0 - terrain_filter_gain_) * terrain_pitch_;

                    terrain_roll_  = std::clamp(terrain_roll_,  -M_PI / 6.0, M_PI / 6.0);
                    terrain_pitch_ = std::clamp(terrain_pitch_, -M_PI / 6.0, M_PI / 6.0);
                }
            }
        }
    }


    // ========== 5.融合全局地形高度 + 前后分区地形高度 ==========
    if (!support_leg_indices.empty()) {
        scalar_t total_height = 0.0;
        int total_count = 0;

        scalar_t front_height_sum = 0.0;
        int front_count = 0;

        scalar_t rear_height_sum = 0.0;
        int rear_count = 0;

        for (int idx : support_leg_indices) {
            const scalar_t h = terrain_height_per_leg_(idx);

            total_height += h;
            total_count++;

            // 约定：0=FL, 1=FR, 2=RL, 3=RR
            if (idx == 0 || idx == 1) {
                front_height_sum += h;
                front_count++;
            } else if (idx == 2 || idx == 3) {
                rear_height_sum += h;
                rear_count++;
            }
        }

        // 1) 全局融合高度（保留原逻辑）
        if (total_count > 0) {
            const scalar_t new_fused_height = total_height / static_cast<scalar_t>(total_count);
            fused_terrain_height_ =
                terrain_height_filter_gain_ * new_fused_height +
                (1.0 - terrain_height_filter_gain_) * fused_terrain_height_;
        }

        // 2) 前区融合高度（只要前腿里当前有支撑腿，就更新）
        if (front_count > 0) {
            const scalar_t new_front_height = front_height_sum / static_cast<scalar_t>(front_count);
            fused_terrain_height_front_ =
                terrain_height_filter_gain_ * new_front_height +
                (1.0 - terrain_height_filter_gain_) * fused_terrain_height_front_;
        }

        // 3) 后区融合高度（只要后腿里当前有支撑腿，就更新）
        if (rear_count > 0) {
            const scalar_t new_rear_height = rear_height_sum / static_cast<scalar_t>(rear_count);
            fused_terrain_height_rear_ =
                terrain_height_filter_gain_ * new_rear_height +
                (1.0 - terrain_height_filter_gain_) * fused_terrain_height_rear_;
        }

        // 4) 前后高度差：前高后低为正，前低后高为负
        front_rear_height_diff_ = fused_terrain_height_front_ - fused_terrain_height_rear_;
    }
    // 无支撑腿时，保持上一帧融合高度
}

    



    



    /**
     * @brief 构建里程计消息
     * @return nav_msgs::msg::Odometry 里程计消息
     * @details 将卡尔曼滤波估计的状态转换为ROS2里程计消息，包含：
     * 1. 基座位置/姿态（带协方差）
     * 2. 基座线速度/角速度（机体坐标系，带协方差）
     */
    nav_msgs::msg::Odometry KalmanFilterEstimate::getOdomMsg() {
        nav_msgs::msg::Odometry odom;

        // 填充位姿信息（全局坐标系）
        odom.pose.pose.position.x = xHat_.segment<3>(0)(0);  // x位置
        odom.pose.pose.position.y = xHat_.segment<3>(0)(1);  // y位置
        odom.pose.pose.position.z = xHat_.segment<3>(0)(2);  // z位置
        // 姿态：IMU四元数
        odom.pose.pose.orientation.x = quat_.x();
        odom.pose.pose.orientation.y = quat_.y();
        odom.pose.pose.orientation.z = quat_.z();
        odom.pose.pose.orientation.w = quat_.w();

        // 填充位姿协方差（位置协方差+姿态协方差）
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                odom.pose.covariance[i * 6 + j] = p_(i, j);  // 位置协方差（P矩阵前3x3）
                odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);  // 姿态协方差
            }
        }

        // 填充速度信息（机体坐标系）
        // 全局线速度转换为机体坐标系：v_body = R^T * v_world
        vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
        odom.twist.twist.linear.x = twist.x();  // 前向速度
        odom.twist.twist.linear.y = twist.y();  // 横向速度
        odom.twist.twist.linear.z = twist.z();  // 垂直速度
        // 角速度：直接使用IMU机体坐标系角速度
        odom.twist.twist.angular.x = angular_vel_local_.x();
        odom.twist.twist.angular.y = angular_vel_local_.y();
        odom.twist.twist.angular.z = angular_vel_local_.z();

        // 填充速度协方差（线速度协方差+角速度协方差）
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);  // 线速度协方差（P矩阵3-6行3-6列）
                odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);  // 角速度协方差
            }
        }

        return odom;
    }

    /**
     * @brief 加载卡尔曼滤波器配置参数
     * @param task_file: 配置文件路径
     * @param verbose: 是否输出加载日志
     * @details 从配置文件加载滤波所需的噪声参数和足端半径：
     * 1. 足端半径（用于位姿修正）
     * 2. IMU过程噪声（位置/速度）
     * 3. 足端过程/观测噪声（位置/速度/高度）
     */
    void KalmanFilterEstimate::loadSettings(const std::string &task_file, const bool verbose) {
        boost::property_tree::ptree pt;
        read_info(task_file, pt);  // 读取配置文件到属性树
        const std::string prefix = "kalmanFilter.";  // 配置参数前缀

        // 输出加载日志（verbose模式）
        if (verbose) {
            std::cerr << "\n #### Kalman Filter Noise:";
            std::cerr << "\n #### =============================================================================\n";
        }

        // 加载滤波参数
        loadData::loadPtreeValue(pt, foot_radius_, prefix + "footRadius", verbose);                  // 足端半径
        loadData::loadPtreeValue(pt, imu_process_noise_position_, prefix + "imuProcessNoisePosition", verbose);  // IMU位置过程噪声
        loadData::loadPtreeValue(pt, imu_process_noise_velocity_, prefix + "imuProcessNoiseVelocity", verbose);  // IMU速度过程噪声
        loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);    // 足端位置过程噪声
        loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);      // 足端位置观测噪声
        loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);      // 足端速度观测噪声
        loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);          // 足端高度观测噪声

        
        // ---------------- 新增：论文接触信任度参数加载 ----------------
        const std::string trust_prefix = prefix + "contactTrust.";
        loadData::loadPtreeValue(pt, W_, trust_prefix + "window", verbose);
        loadData::loadPtreeValue(pt, k_plus_, trust_prefix + "kPlus", verbose);
        loadData::loadPtreeValue(pt, k_minus_, trust_prefix + "kMinus", verbose);
        loadData::loadPtreeValue(pt, kappa_, trust_prefix + "kappa", verbose);   


        // ================ 新增：加载GM观测器和接触检测参数 ================
        loadData::loadPtreeValue(pt, gm_cutoff_freq_, prefix + "gmCutoffFreq", verbose);
        loadData::loadPtreeValue(pt, threshold_high_, prefix + "contactThresholdHigh", verbose);
        loadData::loadPtreeValue(pt, threshold_low_, prefix + "contactThresholdLow", verbose);
        loadData::loadPtreeValue(pt, grf_filter_gain_, prefix + "grfFilterGain", verbose);
        loadData::loadPtreeValue(pt, terrain_filter_gain_, prefix + "terrainFilterGain", verbose);
        // ========== 新增：加载被动高度感知参数 ==========
        loadData::loadPtreeValue(pt, terrain_height_filter_gain_, prefix + "terrainHeightFilterGain", verbose);

    }
} // namespace legged

//
// Created by tlab-uav on 24-9-30.
//

// TargetManager类头文件：四足机器人目标轨迹管理器核心类
#include "ocs2_quadruped_controller/control/TargetManager.h"

#include <ocs2_core/misc/LoadData.h>
// OCS2机器人工具：提供欧拉角转旋转矩阵等坐标变换函数
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "ocs2_quadruped_controller/estimator/LinearKalmanFilter.h"
#include <utility>


namespace ocs2::legged_robot
{
    /**
     * @brief TargetManager构造函数
     * @param ctrl_component: 控制器组件接口，用于获取控制输入和频率参数
     * @param node: ROS2生命周期节点指针，用于创建话题订阅器
     * @param referenceManagerPtr: 参考管理器指针，用于设置目标轨迹
     * @param task_file: 任务配置文件路径（MPC相关配置）
     * @param reference_file: 参考轨迹配置文件路径
     * @details 初始化目标管理器，加载配置参数，创建/cmd_vel话题订阅器
     */
    TargetManager::TargetManager(CtrlInterfaces& ctrl_component,
                                 rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                 const std::shared_ptr<ReferenceManagerInterface>& referenceManagerPtr,
                                 const std::string& task_file,
                                 const std::string& reference_file) // 新增参数
        : ctrl_component_(ctrl_component),                     // 保存控制器组件引用
          referenceManagerPtr_(referenceManagerPtr),           // 保存参考管理器指针
          node_(std::move(node))                              // 转移节点指针所有权
    {
        // 初始化默认关节状态为12维零向量，只包括腿部关节，没有包括轮毂
        default_joint_state_ = vector_t::Zero(12);
        
        // 从参考配置文件加载参数
        loadData::loadCppDataType(reference_file, "comHeight", command_height_);              // 质心目标高度
        loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joint_state_); // 默认关节状态
        flat_default_joint_state_ = default_joint_state_;
        loadData::loadCppDataType(task_file, "mpc.timeHorizon", time_to_target_);            // MPC时间域（到达目标的时间）
        loadData::loadCppDataType(reference_file, "targetRotationVelocity", target_rotation_velocity_);  // 目标旋转速度
        loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", target_displacement_velocity_);  // 目标位移速度
        // 新增：加载车轮速度占比系数（配置文件中新增该参数）
        loadData::loadCppDataType(reference_file, "wheelVelocityRatio", wheel_vel_ratio_);
        
        // 创建/cmd_vel话题订阅器：接收速度指令（如键盘/遥控指令）
        twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                // 将接收到的Twist消息写入线程安全缓冲区
                buffer_.writeFromNonRT(*msg);
                // 设置twist_count：控制指令有效期（控制器频率/5，即持续1/5秒）
                twist_count = ctrl_component_.frequency_ / 5;
                RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                      "Twist count: %i", twist_count);
            });

        // 新增1：创建车轮目标速度发布器（话题/wheel/target_vel，供车轮控制器订阅）
        wheel_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/wheel/target_vel", 10);   
    }
    
    /**
     * @brief 更新目标轨迹
     * @param observation: 系统观测数据（包含当前状态、时间等）
     * @details 核心功能函数：
     * 1. 融合本地控制输入和/cmd_vel话题指令，生成目标速度指令
     * 2. 基于当前位姿和目标速度，计算目标位姿                 ************重要*************
     * 3. 生成目标轨迹并设置到参考管理器中
     */
    void TargetManager::update(SystemObservation& observation)
    {
        // 直接用保存的estimator_ptr_，不用绕CtrlInterfaces！
        auto* kf = ctrl_component_.kalman_filter_ptr;

        // 初始化6维目标指令向量（x/y平移速度、偏航角速度等）
        vector_t cmdGoal = vector_t::Zero(6);
        // 初始化3维车轮速度、3维纯腿部迈步速度（适配不可转向车轮）
        vector_t wheel_vel = vector_t::Zero(3);    // 车轮驱动速度（仅x轴有效）
        vector_t leg_only_vel = vector_t::Zero(3); // 纯腿部迈步速度       
        // 分支1：无/cmd_vel指令或指令已过期 → 使用本地遥控控制输入
        if (buffer_.readFromRT() == nullptr || twist_count <= 0)
        {
            // 基于本地控制输入生成目标速度（ly/lx/ry/rx为摇杆/按键输入）
            cmdGoal[0] = ctrl_component_.control_inputs_.ly * target_displacement_velocity_;  // x轴位移速度（前向）
            cmdGoal[1] = -ctrl_component_.control_inputs_.lx * target_displacement_velocity_; // y轴位移速度（横向）
            cmdGoal[2] = ctrl_component_.control_inputs_.ry;                                 // 预留（未使用）
            cmdGoal[3] = -ctrl_component_.control_inputs_.rx * target_rotation_velocity_;    // 偏航旋转速度
             

            // 动态切换：有明确的“置1”和“复位”逻辑
            if (ctrl_component_.control_inputs_.command == 1) {
                wheel_vel_ratio_use = 1.0; // command=1：车轮承担100%前向速度
            } else {
                wheel_vel_ratio_use = wheel_vel_ratio_; // command≠1：车轮速度比例复位为0（可自定义）
            }
            
            // ===================== 适配不可转向车轮 =====================
            // 关键修正1：不可转向车轮仅提供基座x轴（前向）速度，y轴车轮速度强制为0
            wheel_vel(0) = cmdGoal[0] * wheel_vel_ratio_use; // 车轮前向速度（x轴）
            wheel_vel(1) = 0.0;                           // 横向（y轴）车轮速度=0（不可转向）

            // 先计算车轮实际平均角速度（从硬件状态接口读取）
            scalar_t wheel_omega_avg = 0.0;
            int valid_wheel_count = 0;
            // 遍历4个车轮接口（12-15）读取实际角速度
            for (int i=12; i<=15; i++) {
                if (i < static_cast<int>(ctrl_component_.joint_velocity_state_interface_.size())) {
                    // 安全读取optional类型的实际角速度值
                    auto omega_opt = ctrl_component_.joint_velocity_state_interface_[i].get().get_optional();
                    if (omega_opt.has_value()) {
                        wheel_omega_avg += omega_opt.value();
                        valid_wheel_count++;
                    }
                }
            }
            // 计算平均角速度（避免除0）
            if (valid_wheel_count > 0) {
                wheel_omega_avg /= valid_wheel_count;
            }
            // 车轮实际前向线速度 = 平均角速度 × 车轮半径（需提前加载wheel_radius_）
            scalar_t wheel_vel_actual = wheel_omega_avg * 0.0992;


            // 关键修正2：腿部速度拆分
            leg_only_vel(0) = cmdGoal[0] - wheel_vel_actual;  // 腿部前向速度 = 总前向速度 - 实际车轮前向速度
            leg_only_vel(1) = cmdGoal[1];                 // 腿部横向速度 = 总横向速度（车轮不提供）

            // 打印话题指令模式的速度信息
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            //                     "[话题指令] 目标前向速度：%.3f m/s | 横向速度：%.3f m/s | 偏航角速度：%.3f rad/s",
            //                     cmdGoal[0], cmdGoal[1], cmdGoal[3]);
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            //                     "[话题速度拆分] 车轮指令速度：%.3f m/s | 实际速度：%.3f m/s | 腿部速度：%.3f m/s",
            //                     wheel_vel(0), wheel_vel_actual, leg_only_vel(0));
        }
        // 分支2：有有效/cmd_vel指令 → 使用话题指令，这部分不使用
        else
        {
            // 从线程安全缓冲区读取Twist指令
            const geometry_msgs::msg::Twist twist = *buffer_.readFromRT();
            // 映射Twist指令到目标速度向量
            cmdGoal[0] = twist.linear.x;   // 前向速度
            cmdGoal[1] = twist.linear.y;   // 横向速度
            cmdGoal[2] = 0;                // 垂直速度（固定为0）
            cmdGoal[3] = twist.angular.z;  // 偏航角速度            
            twist_count--;
            if (twist_count <= 0)
            {
                buffer_.reset();
            }
        }

        // 提取当前位姿：observation.state[6:12]为基座部位姿[x, y, z, yaw, pitch, roll]
        const vector_t currentPose = observation.state.segment<6>(6);
        // 当前参考位姿：x/y/yaw跟随当前实际，z/pitch/roll做地形一致修正
        vector_t currentRefPose = currentPose;

        if (kf != nullptr) {
            currentRefPose(2) = command_height_ + kf->getFusedTerrainHeight();
            currentRefPose(4) = kf->getTerrainPitch();
            currentRefPose(5) = kf->getTerrainRoll();
        } else {
            currentRefPose(2) = command_height_;
            currentRefPose(4) = 0.0;
            currentRefPose(5) = 0.0;
        }
        // 提取当前欧拉角（zyx顺序：yaw/pitch/roll）
        const Eigen::Matrix<scalar_t, 3, 1> zyx = currentRefPose.tail(3);
        Eigen::Matrix<scalar_t, 3, 1> zyx_yaw_only;
        zyx_yaw_only << currentRefPose(3), 0.0, 0.0;       
        // =====================（改用zyx_yaw_only） =====================
        // 1. 总速度（车轮+腿）转世界坐标系 → 用于计算目标位姿（保证平衡）
        vector_t cmd_vel_total_rot = getRotationMatrixFromZyxEulerAngles(zyx_yaw_only) * cmdGoal.head(3);
        // 2. 纯腿速度转世界坐标系 → 用于MPC步态规划（仅腿响应）
        vector_t cmd_vel_leg_only_rot = getRotationMatrixFromZyxEulerAngles(zyx_yaw_only) * leg_only_vel;
        // 计算目标位姿：基于当前位姿和目标速度，在time_to_target_时间后到达的位姿
        const vector_t targetPose = [&]
        {
            vector_t target(6);
            target(0) = currentRefPose(0) + cmd_vel_total_rot(0) * time_to_target_;  // 总速度算x目标位置
            target(1) = currentRefPose(1) + cmd_vel_total_rot(1) * time_to_target_;  // 总速度算y目标位置
            target(3) = currentRefPose(3) + cmdGoal(3) * time_to_target_;      // yaw目标角度（基于偏航速度）
            if (kf != nullptr) {
                target(4) = ctrl_component_.kalman_filter_ptr->getTerrainPitch();
                target(5) = ctrl_component_.kalman_filter_ptr->getTerrainRoll();
                target(2) = command_height_+kf->getFusedTerrainHeight();    // z轴目标位置（固定质心高度）
                // RCLCPP_INFO(
                //     node_->get_logger(),
                //     "[TargetManager] 地形倾角：Pitch=%.3f rad | Roll=%.3f rad | hight=%.3f m |kf指针=%p",
                //     target(4), target(5),target(2), kf
                // );

            } else {
                target(4) = 0.0;
                target(5) = 0.0;
                target(2) = command_height_;
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "[TargetManager] kf指针为空！");
            }
            return target;
        }();



        vector_t adaptive_joint_state = flat_default_joint_state_;

        // 用当前参考pitch，不直接用原始地形估计，避免符号再绕一层
        scalar_t pitch_ref = currentRefPose(4);

        // 简单一阶滤波，避免地形估计抖动直接打到关节参考
        filtered_pitch_ref_ = 0.9 * filtered_pitch_ref_ + 0.1 * pitch_ref;

        // 限幅
        scalar_t pitch_use = std::clamp(filtered_pitch_ref_, -pitch_comp_limit_, pitch_comp_limit_);

        // -------- 版本A：四条腿同向补偿（保持大腿杆世界倾角更接近平地）--------
        adaptive_joint_state(1)  -= pitch_thigh_gain_ * pitch_use;   // FL_thigh
        adaptive_joint_state(4)  -= pitch_thigh_gain_ * pitch_use;   // FR_thigh
        adaptive_joint_state(7)  -= pitch_thigh_gain_ * pitch_use;   // RL_thigh
        adaptive_joint_state(10) -= pitch_thigh_gain_ * pitch_use;   // RR_thigh



        // 计算目标到达时间：当前观测时间 + MPC时间域
        const scalar_t targetReachingTime = observation.time + time_to_target_;
        // 将目标位姿转换为TargetTrajectories格式
        auto trajectories =
        targetPoseToTargetTrajectories(currentRefPose,
                                    targetPose,
                                    adaptive_joint_state,
                                    observation,
                                    targetReachingTime);
        trajectories.stateTrajectory[0].head(3) = cmd_vel_leg_only_rot;
        trajectories.stateTrajectory[1].head(3) = cmd_vel_leg_only_rot;
        // 将生成的目标轨迹设置到参考管理器，供MPC求解使用
        referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
        // 发布车轮目标速度（供车轮控制器订阅）
        publishWheelVelocity(wheel_vel, cmdGoal[3]);
    }


    
    /**
     * @brief 发布车轮目标速度到/wheel/target_vel话题（适配不可转向车轮）
     * @param wheel_vel: 3维车轮速度（仅x轴有效，y轴=0）
     */
    void ocs2::legged_robot::TargetManager::publishWheelVelocity(const vector_t& wheel_vel, const scalar_t yaw_vel)
    {
        geometry_msgs::msg::Twist wheel_vel_msg;
        // 不可转向车轮仅发布前向速度（x轴），横向速度强制为0
        wheel_vel_msg.linear.x = wheel_vel(0);  // 车轮前向目标速度
        wheel_vel_msg.linear.y = 0.0;           // 横向速度=0（不可转向）
        wheel_vel_msg.linear.z = 0.0;
        // 偏航角速度（用于车轮差速转弯）→ 用传入的参数，不再用cmdGoal
        wheel_vel_msg.angular.z = yaw_vel;
        // 发布消息给车轮控制器
        wheel_vel_pub_->publish(wheel_vel_msg);
    }
   

}

//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "StateEstimateBase.h"
// 新增：GaitSchedule 头文件
#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <cmath> // 用于erf函数
#include <tf2_ros/transform_broadcaster.h>
#include <ocs2_legged_robot/gait/LegLogic.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <array>




namespace ocs2::legged_robot {
    class KalmanFilterEstimate final : public StateEstimateBase {
    public:
        KalmanFilterEstimate(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                             const PinocchioEndEffectorKinematics &ee_kinematics,
                             CtrlInterfaces &ctrl_component,
                             const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                             std::shared_ptr<GaitSchedule> gait_schedule_ptr); // 新增

        vector_t update(const SystemObservation& observation, const rclcpp::Time &ros_time, const rclcpp::Duration &period) override;

        void loadSettings(const std::string &task_file, bool verbose);
        // ================ 新增：MPC调用的只读接口 ================
        // 返回世界坐标系下的足底接触力GRF
        const std::vector<vector3_t>& getGRFWorld() const { return grf_world_; }  
        // 返回每条腿的接触概率（0~1）
        const vector_t& getContactProbabilities() const { return contact_prob_; }        
        // 返回二值接触状态：true=腿着地，false=腿摆动
        const std::vector<bool>& getContactStates() const { return contact_states_; }             
        // 返回地形俯仰角（rad）
        scalar_t getTerrainPitch() const { return terrain_pitch_; }
        // 返回地形横滚角（rad）
        scalar_t getTerrainRoll() const { return terrain_roll_; }
        // 返回世界坐标系下的地形法向量（z朝上，单位向量）
        const vector3_t& getTerrainNormalWorld() const { return terrain_normal_world_; }
        // 返回地形法向量是否已有有效估计
        bool hasTerrainNormalEstimate() const { return terrain_normal_valid_; }
        // 新增：获取单腿地形高度（可选，用于调试）
        const vector_t& getTerrainHeightPerLeg() const { return terrain_height_per_leg_; }
        // 新增：获取融合后的全局地形高度（已踩过的地形）
        scalar_t getFusedTerrainHeight() const { return fused_terrain_height_; }      
        scalar_t getFusedTerrainHeightFront() const { return fused_terrain_height_front_; }
        scalar_t getFusedTerrainHeightRear()  const { return fused_terrain_height_rear_;  }
        scalar_t getFrontRearHeightDiff()     const { return front_rear_height_diff_;     }                  
                

    protected:
        nav_msgs::msg::Odometry getOdomMsg();

        PinocchioInterface pinocchio_interface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;

        vector_t feet_heights_;

        // Config
        scalar_t foot_radius_ = 0.0992;
        scalar_t imu_process_noise_position_ = 0.02;
        scalar_t imu_process_noise_velocity_ = 0.02;
        scalar_t footProcessNoisePosition_ = 0.002;
        scalar_t footSensorNoisePosition_ = 0.005;
        scalar_t footSensorNoiseVelocity_ = 0.1;
        scalar_t footHeightSensorNoise_ = 0.01;

    private:
        size_t numContacts_, dimContacts_, numState_, numObserve_;
        // ---------------- 新增：论文接触信任度参数 ----------------
        double W_ = 0.2;          // 不信任窗口（论文推荐0.1~0.3）
        double k_plus_ = 50.0;    // 正高度差不信任增益
        double k_minus_ = 10.0;   // 负高度差不信任增益（要求k_minus < k_plus）
        double kappa_ = 1e3;       // 高怀疑系数
            // ---------------- 新增：辅助函数 ----------------
        // 计算接触相位 phi_c
        scalar_t calculateContactPhase(size_t leg, scalar_t time,size_t current_mode) const;
        // 计算相位基信任 C_phi
        scalar_t calculatePhaseTrust(scalar_t phi_c, bool expected_contact) const;
        // 计算高度基信任 C_z
        scalar_t calculateHeightTrust(scalar_t z_rel) const;
         
        // ---------------- 新增：步态调度指针 ----------------
        std::shared_ptr<GaitSchedule> gait_schedule_ptr_;
        // 观测值存储
        vector_t vs_k_; // 运动学接触点速度 ṗ_cp_k (dimContacts_ x 1)
        vector_t vs_w_; // 驱动接触点速度 ṗ_cp_w (dimContacts_ x 1)
        vector_t wheel_velocities_; // 轮子角速度 (numContacts_ x 1)


        matrix_t a_, b_, c_, q_, p_, r_;
        vector_t xHat_, ps_, vs_;



        // ---------------- 新增：GRF估计相关 ----------------
        std::vector<vector3_t> grf_world_;       // 世界坐标系下的足底接触力（4条腿×3维）
        std::vector<vector3_t> grf_body_;        // 机体坐标系下的足底接触力
        std::vector<vector3_t> grf_filter_prev_; // GRF低通滤波历史值
        scalar_t grf_filter_gain_ = 0.2;        // GRF滤波系数（0~1，越小越平滑）                                  正确的不用调换，很奇怪从硬件读的自己转成FL开头的了
        std::vector<int> leg_torque_joint_idx_ = {0,1,2,3,4,5,6,7,8,9,10,11}; // 腿力矩关节索引映射{3,4,5,0,1,2,9,10,11,6,7,8}  {0,1,2,3,4,5,6,7,8,9,10,11}

        // ---------------- 新增：地形倾角估计相关 ----------------
        scalar_t terrain_pitch_ = 0.0;           // 地形俯仰角（rad）
        scalar_t terrain_roll_ = 0.0;            // 地形横滚角（rad）
        vector3_t terrain_normal_world_ = vector3_t::UnitZ();  // 世界系地形法向量（单位向量，z朝上）
        bool terrain_normal_valid_ = false;      // 至少一次由接触点/锁存点成功估计出法向
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr terrain_normal_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wheel_rolling_frame_marker_pub_;
        scalar_t fused_terrain_height_front_ = 0.0;   // 前腿区域融合地形高度（FL/FR）
        scalar_t fused_terrain_height_rear_  = 0.0;   // 后腿区域融合地形高度（RL/RR）
        scalar_t front_rear_height_diff_     = 0.0;   // 前后高度差 = front - rear
        scalar_t terrain_filter_gain_ = 0.1;     // 地形倾角滤波系数

        // ================ 新增1：接触检测核心变量（论文Section IV） ================
        vector_t contact_prob_;             // 每条腿的接触概率 [P_FL, P_FR, P_RL, P_RR]
        std::vector<bool> contact_states_;  // 二值接触状态（滞环输出，最终给MPC用）
        scalar_t threshold_high_ = 0.7;     // 滞环高阈值：概率>0.7判定为着地
        scalar_t threshold_low_ = 0.3;      // 滞环低阈值：概率<0.3判定为离地
        // 接触检测专用卡尔曼滤波矩阵（论文概率融合用）
        vector_t contact_xHat_;      // 接触概率状态估计
        matrix_t contact_P_;         // 接触概率协方差
        matrix_t contact_Q_;         // 接触检测过程噪声
        matrix_t contact_R_;         // 接触检测观测噪声

        // ================ 新增2：离散GM观测器变量（论文Section III） ================
        vector_t gm_p_;             // 广义动量 p = M*v（论文核心公式）
        vector_t gm_filter_state_;  // GM观测器的低通滤波状态
        scalar_t gm_gamma_ = 0.0;   // 离散滤波参数 γ（论文公式10）
        scalar_t gm_beta_ = 0.0;    // 离散滤波参数 β（论文公式10）
        scalar_t gm_cutoff_freq_ = 15.0; // 论文推荐的低通截止频率15Hz

        // ================ 新增3：临时缓存变量（避免重复计算） ================
        scalar_t current_time = 0.0;  // 当前时间戳
        size_t current_mode = 0;      // 当前步态模式（来自步态调度）     

        // ---------------- 新增：被动地形高度感知相关 ----------------
        vector_t terrain_height_per_leg_;  // 每条腿下的地形高度（4维：FL/FR/RL/RR）
        scalar_t fused_terrain_height_=0.0;    // 融合后的全局地形高度（支撑腿平均）
        scalar_t terrain_height_filter_gain_ = 0.2;  // 高度滤波系数（0~1，越小越平滑）
        
        // ================ 新增：核心功能函数声明 ================
        // 1. 论文离散时间广义动量观测器（GRF估计，降噪）
        void updateGMObserver(const vector_t& qPino, const vector_t& vPino, const vector_t& tau, scalar_t dt);
        // 2. 论文概率接触检测（基于C_phi、C_z、GM力估计融合）
        void updateContactDetection(const std::array<bool, 4>& expected_contact, const vector_t& foot_heights);
        // 3. 优化后的地形估计（用新的接触状态计算，结果存到成员变量）
        void estimateTerrainTilt(const std::vector<vector3_t>& eePos, const ocs2::vector_t& contact_prob, scalar_t dt ,int current_mode);
        vector3_t getTerrainNormalWorldForWheelFusionOrUnitZ() const;
        static vector3_t getTangentProjectionOrFallback(const vector3_t& direction, const vector3_t& normal);
        vector3_t getWheelRollingDirectionWorld(size_t contactIndex, const matrix3_t& R_body_to_world) const;
        void publishTerrainNormalMarker(const rclcpp::Time& stamp, const vector3_t& normal_world, const vector3_t& origin_world);
        void publishWheelRollingFrameMarkers(const rclcpp::Time& stamp,
                                             const std::vector<vector3_t>& ee_pos_base_to_foot_world);



    };
}

//
// Created by tlab-uav on 24-9-30.
//

#ifndef TARGETMANAGER_H
#define TARGETMANAGER_H


#include <memory>
#include <controller_common/CtrlInterfaces.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <geometry_msgs/msg/twist.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include "ocs2_quadruped_controller/estimator/StateEstimateBase.h" // 加头文件
namespace ocs2::legged_robot
{
    class TargetManager
    {
    public:
        TargetManager(CtrlInterfaces& ctrl_component,
                      rclcpp_lifecycle::LifecycleNode::SharedPtr  node,
                      const std::shared_ptr<ReferenceManagerInterface>& referenceManagerPtr,
                      const std::string& task_file,
                      const std::string& reference_file); // 新增参数

        ~TargetManager() = default;

        void update(SystemObservation& observation);

    private:
        TargetTrajectories targetPoseToTargetTrajectories(const vector_t& currentRefPose,
                                                        const vector_t& targetPose,
                                                        const vector_t& desiredJointState,
                                                        const SystemObservation& observation,
                                                        const scalar_t& targetReachingTime)
        {
            const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

            vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
            stateTrajectory[0] << vector_t::Zero(6), currentRefPose, desiredJointState;
            stateTrajectory[1] << vector_t::Zero(6), targetPose, desiredJointState;

            const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

            return {timeTrajectory, stateTrajectory, inputTrajectory};
        }

        CtrlInterfaces& ctrl_component_;
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> buffer_;
        int twist_count = 0;

        vector_t default_joint_state_{};
        vector_t flat_default_joint_state_{};
        scalar_t command_height_{};
        scalar_t time_to_target_{};
        scalar_t target_displacement_velocity_{};
        scalar_t target_rotation_velocity_{};



        scalar_t pitch_thigh_gain_{1.0};   // thigh补偿增益
        scalar_t pitch_calf_gain_{-0.70};   // calf联动补偿增益
        scalar_t pitch_comp_limit_{0.95};   // pitch补偿限幅
        scalar_t filtered_pitch_ref_{0.0};  // 可选：滤波后的pitch参考

        // ===================== 新增开始 =====================
        // 车轮速度占比系数（0~1，0=纯腿，1=纯车轮）
        scalar_t wheel_vel_ratio_use=0.0;
        scalar_t wheel_vel_ratio_=0.0;
        // 车轮目标速度发布器（核心，给车轮控制器传目标速度）
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_pub_;
        // 车轮速度线程安全缓冲区（配合wheel_twist_sub_）
        realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> wheel_buffer_;
        // 车轮指令有效期计数（和twist_count逻辑一致）
        int wheel_twist_count{0};
        // 发布车轮目标速度的函数声明（cpp里实现）
        void publishWheelVelocity(const vector_t& wheel_vel, const scalar_t yaw_vel);
        
        // ===================== 新增结束 =====================


        
    };
}

#endif //TARGETMANAGER_H

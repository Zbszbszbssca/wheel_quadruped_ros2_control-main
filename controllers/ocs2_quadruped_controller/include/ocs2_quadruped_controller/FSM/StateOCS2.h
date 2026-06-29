//
// Created by tlab-uav on 25-2-27.
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <SafetyChecker.h>
#include <array>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_quadruped_controller/control/CtrlComponent.h>
#include <ocs2_quadruped_controller/wbc/WbcBase.h>
#include <rclcpp/duration.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "controller_common/FSM/FSMState.h"

namespace ocs2
{
    class MPC_MRT_Interface;
    class MPC_BASE;
}

namespace ocs2::legged_robot
{
    class StateOCS2 final : public FSMState
    {
    public:
        StateOCS2(CtrlInterfaces& ctrl_interfaces,
                  const std::shared_ptr<CtrlComponent>& ctrl_component
        );

        void enter() override;

        void run(const rclcpp::Time& time,
                 const rclcpp::Duration& period) override;

        void exit() override;

        FSMStateName checkChange() override;

    private:
        void publishFrictionConeMarkers(const rclcpp::Time& stamp,
                                        size_t planned_mode,
                                        const vector_t& optimized_state,
                                        const vector_t& optimized_input);

        vector3_t getTerrainNormalWorldForWheelCommandOrUnitZ() const;

        static vector3_t getTangentProjectionOrFallback(const vector3_t& direction, const vector3_t& normal);

        void updateWheelCommandPinocchioFrames(const vector_t& rbdStateMeasured);

        std::array<vector3_t, 4> getWheelRollingDirectionsHardwareOrder(const matrix3_t& R_base_to_world) const;

        std::array<vector3_t, 4> getWheelRollingDirectionsHardwareOrderFromInterface(
            const PinocchioInterface& pinocchioInterface,
            const matrix3_t& R_base_to_world) const;

        std::array<vector3_t, 4> getWheelPositionsWorldHardwareOrder(const vector3_t& basePositionWorld,
                                                                     const matrix3_t& R_base_to_world) const;

        bool getPlannedContactVelocitiesWorldHardwareOrder(const vector_t& plannedState,
                                                           const vector_t& plannedInput,
                                                           std::array<vector3_t, 4>& contactVelocitiesWorldHw);


        std::shared_ptr<CtrlComponent> ctrl_component_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;
        PinocchioInterface wheel_command_pinocchio_interface_;
        PinocchioInterface planned_wheel_pinocchio_interface_;
        CentroidalModelPinocchioMapping planned_wheel_mapping_;
        std::unique_ptr<PinocchioEndEffectorKinematics> planned_wheel_kinematics_;
        benchmark::RepeatedTimer wbc_timer_;

        double default_kp_ = 0;
        double default_kd_ = 6;

        vector_t optimized_state_, optimized_input_;
        // ===================== 新增开始 =====================
        // 车轮速度指令订阅器：接收TargetManager发布的/wheel/target_vel
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_sub_;
        // 线程安全缓冲区：保存车轮速度指令（避免实时线程阻塞）
        realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> wheel_vel_buffer_;
        scalar_t wheel_radius_ = 0.0992;
        scalar_t friction_cone_mu_ = 0.7;
        scalar_t friction_cone_normal_force_scale_ = 120.0;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr friction_cone_marker_pub_;
    };
}


#endif //STATEOCS2_H

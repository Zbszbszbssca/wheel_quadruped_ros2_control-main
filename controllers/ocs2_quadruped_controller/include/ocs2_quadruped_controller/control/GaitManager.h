//
// Created by tlab-uav on 24-9-26.
//

#ifndef GAITMANAGER_H
#define GAITMANAGER_H

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <controller_common/CtrlInterfaces.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace ocs2::legged_robot
{
    class GaitManager final : public SolverSynchronizedModule
    {
    public:
        GaitManager(CtrlInterfaces& ctrl_interfaces,
                    std::shared_ptr<GaitSchedule> gait_schedule_ptr,
                    PinocchioInterface pinocchioInterface,
                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                    const CentroidalModelInfo& info);

        void preSolverRun(scalar_t initTime, scalar_t finalTime,
                          const vector_t& currentState,
                          const ReferenceManagerInterface& referenceManager) override;

        void postSolverRun(const PrimalSolution&/**primalSolution**/) override
        {
        }

        void init(const std::string& gait_file);

    private:
        struct KinematicOnlineGaitConfig
        {
            bool enabled{true};
            int activationCommand{4};
            scalar_t rollingThreshold{0.05};
            scalar_t lateralThreshold{0.03};
            scalar_t triggerScoreThreshold{1.10};
            scalar_t criticalScoreThreshold{1.60};
            scalar_t swingDuration{0.35};
            scalar_t preStanceDuration{0.08};
            scalar_t interStanceDuration{0.12};
            scalar_t stanceAfterSwing{0.20};
            scalar_t minTriggerInterval{0.80};
            scalar_t startDelay{0.20};
            scalar_t diagnosticLogPeriod{1.00};
            bool diagnosticVerbose{false};
        };

        void loadKinematicOnlineGaitConfig(const std::string& gait_file);
        void getTargetGait(scalar_t initTime, scalar_t finalTime, const vector_t& currentState);
        void updateKinematicOnlineGait(scalar_t initTime, scalar_t finalTime, const vector_t& currentState);
        void resetKinematicOnlineGait();
        bool initializeNominalFootPositions();
        bool getBaseYawPose(const vector_t& currentState, vector3_t& basePositionWorld, scalar_t& yaw) const;
        void initializeOnlineFootholdReferences(const vector_t& currentState);
        std::array<vector3_t, 4> getReferenceFootPositionsBaseFrame(const vector_t& currentState);
        ModeSequenceTemplate makeTrotRecoveryTemplate() const;
        void switchToFixedGait(size_t gaitIndex, int command);

        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<GaitSchedule> gait_schedule_ptr_;
        PinocchioInterface pinocchio_interface_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> ee_kinematics_;
        CentroidalModelInfo info_;

        ModeSequenceTemplate target_gait_;
        ModeSequenceTemplate stance_gait_;
        int last_command_ = 0;
        bool gait_updated_{false};
        bool verbose_{false};
        std::vector<ModeSequenceTemplate> gait_list_;
        std::vector<std::string> gait_name_list_;

        KinematicOnlineGaitConfig kinematic_online_gait_config_;
        std::array<vector3_t, 4> online_nominal_foot_pos_base_{};
        std::array<vector3_t, 4> online_reference_foothold_world_{};
        std::array<scalar_t, 4> online_last_rolling_error_{};
        std::array<scalar_t, 4> online_last_lateral_error_{};
        bool online_nominal_initialized_{false};
        bool online_reference_initialized_{false};
        bool online_mode_active_{false};
        bool online_cycle_active_{false};
        scalar_t online_mode_start_time_{-1.0};
        scalar_t online_cycle_end_time_{-std::numeric_limits<scalar_t>::infinity()};
        scalar_t online_next_allowed_trigger_time_{-std::numeric_limits<scalar_t>::infinity()};
        scalar_t online_last_diagnostic_time_{-std::numeric_limits<scalar_t>::infinity()};
        size_t online_last_swing_leg_{4};
    };
}

#endif //GAITMANAGER_H

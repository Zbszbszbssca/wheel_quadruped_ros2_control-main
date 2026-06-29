//
// Created by qiayuan on 2022/7/1.
//
#ifndef WBCBASE_H
#define WBCBASE_H

#include "Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2::legged_robot
{
    class SwitchedModelReferenceManager;

    // Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
    class WbcBase
    {
        using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
        using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

    public:
        virtual ~WbcBase() = default;

        WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                const PinocchioEndEffectorKinematics& eeKinematics,
                const SwitchedModelReferenceManager* referenceManagerPtr = nullptr);

        virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

        virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired,
                                const vector_t& rbdStateMeasured, size_t mode,
                                scalar_t period);

    protected:
        void updateMeasured(const vector_t& rbdStateMeasured);

        void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

        size_t getNumDecisionVars() const { return num_decision_vars_; }

        Task formulateFloatingBaseEomTask();

        Task formulateTorqueLimitsTask();

        Task formulateNoContactMotionTask();

        Task formulateWheelRollingMotionTask();

        Task formulateGroundWheelRollingTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

        matrix_t getWheelRollingProjectionMatrix(size_t contactPointIndex) const;

        vector3_t getWheelRollingDirectionWorld(size_t contactPointIndex) const;

        vector3_t getTerrainNormalWorldForWbcOrUnitZ() const;

        static matrix3_t getRotationWorldToTerrain(const vector3_t& surfaceNormalInWorld);

        static vector3_t getTangentProjectionOrFallback(const vector3_t& direction, const vector3_t& normal);

        Task formulateFrictionConeTask();

        Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

        Task formulateSwingLegTask();

        Task formulateContactForceTask(const vector_t& inputDesired) const;

        size_t num_decision_vars_;
        PinocchioInterface pinocchio_interface_measured_, pinocchio_interface_desired_;
        CentroidalModelInfo info_;

        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
        CentroidalModelPinocchioMapping mapping_;
        const SwitchedModelReferenceManager* referenceManagerPtr_{nullptr};

        vector_t q_measured_, v_measured_, input_last_;
        matrix_t j_, dj_;
        contact_flag_t contact_flag_{};
        size_t num_contacts_{};

        // Task Parameters:
        vector_t torque_limits_;
        scalar_t friction_coeff_{}, swing_kp_{}, swing_kd_{};
        scalar_t ground_wheel_rolling_kp_{0.0}, ground_wheel_rolling_kd_{3.0}, ground_wheel_rolling_max_accel_{3.0};
    };
} // namespace legged
#endif // WBCBASE_H

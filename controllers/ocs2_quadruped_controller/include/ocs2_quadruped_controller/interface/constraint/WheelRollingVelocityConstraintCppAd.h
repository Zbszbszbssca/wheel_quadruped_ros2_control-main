/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "../SwitchedModelReferenceManager.h"

namespace ocs2::legged_robot {
    /**
     * 支撑相轮式足端非完整速度约束。
     *
     * 接触时只约束轮子的侧向速度和地形法向速度为零，释放滚动方向速度：
     *   lateral_W^T * v_ee = 0
     *   normal_W^T  * v_ee = 0
     *
     * 其中 lateral_W 由轮轴在地形切平面内的投影得到，normal_W 来自地形法向估计，
     * 与状态估计器中的轮子 rolling/lateral/normal 可视化保持同一套定义。
     */
    class WheelRollingVelocityConstraintCppAd final : public StateInputConstraint {
    public:
        WheelRollingVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                            const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                            CentroidalModelInfo info,
                                            size_t contactPointIndex);

        ~WheelRollingVelocityConstraintCppAd() override = default;

        WheelRollingVelocityConstraintCppAd* clone() const override { return new WheelRollingVelocityConstraintCppAd(*this); }

        bool isActive(scalar_t time) const override;

        size_t getNumConstraints(scalar_t time) const override { return 2; }

        vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                          const PreComputation& preComp) const override;

        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                                 const vector_t& input,
                                                                 const PreComputation& preComp) const override;

    private:
        WheelRollingVelocityConstraintCppAd(const WheelRollingVelocityConstraintCppAd& rhs);

        matrix_t getVelocityProjectionMatrix(const PreComputation& preComp) const;
        static vector3_t getTangentProjectionOrFallback(const vector3_t& direction, const vector3_t& normal);

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
        CentroidalModelInfo info_;
        const size_t contactPointIndex_;
    };
}  // namespace ocs2::legged_robot

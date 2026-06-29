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

#include "ocs2_quadruped_controller/interface/constraint/WheelRollingVelocityConstraintCppAd.h"

#include <ocs2_core/PreComputation.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "ocs2_quadruped_controller/interface/LeggedRobotPreComputation.h"

namespace ocs2::legged_robot {

WheelRollingVelocityConstraintCppAd::WheelRollingVelocityConstraintCppAd(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    CentroidalModelInfo info,
    size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      info_(std::move(info)),
      contactPointIndex_(contactPointIndex) {
}

WheelRollingVelocityConstraintCppAd::WheelRollingVelocityConstraintCppAd(const WheelRollingVelocityConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      info_(rhs.info_),
      contactPointIndex_(rhs.contactPointIndex_) {
}

bool WheelRollingVelocityConstraintCppAd::isActive(scalar_t time) const {
    return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

vector_t WheelRollingVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const PreComputation& preComp) const {
    const matrix_t Av = getVelocityProjectionMatrix(preComp);
    const vector3_t eeVelocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();
    return Av * eeVelocity;
}

VectorFunctionLinearApproximation WheelRollingVelocityConstraintCppAd::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
    const matrix_t Av = getVelocityProjectionMatrix(preComp);
    const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();

    auto linearApproximation = VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
    linearApproximation.f.noalias() = Av * velocityApprox.f;
    linearApproximation.dfdx.noalias() = Av * velocityApprox.dfdx;
    linearApproximation.dfdu.noalias() = Av * velocityApprox.dfdu;
    return linearApproximation;
}

matrix_t WheelRollingVelocityConstraintCppAd::getVelocityProjectionMatrix(const PreComputation& preComp) const {
    vector3_t terrainNormal = referenceManagerPtr_->getTerrainNormalWorldOrUnitZ();
    if (!terrainNormal.allFinite() || terrainNormal.norm() < 1e-6) {
        terrainNormal = vector3_t::UnitZ();
    } else {
        terrainNormal.normalize();
    }

    vector3_t wheelAxisWorld = vector3_t::UnitY();
    const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
    const auto& pinocchioInterface = preCompLegged.getPinocchioInterface();
    const auto& model = pinocchioInterface.getModel();
    const auto& data = pinocchioInterface.getData();

    if (contactPointIndex_ < info_.endEffectorFrameIndices.size()) {
        const auto frameId = info_.endEffectorFrameIndices[contactPointIndex_];
        if (frameId < model.frames.size() && frameId < data.oMf.size()) {
            wheelAxisWorld = data.oMf[frameId].rotation() * vector3_t::UnitY();
        }
    }
    if (!wheelAxisWorld.allFinite() || wheelAxisWorld.norm() < 1e-6) {
        wheelAxisWorld = vector3_t::UnitY();
    } else {
        wheelAxisWorld.normalize();
    }

    const vector3_t lateralWorld = getTangentProjectionOrFallback(wheelAxisWorld, terrainNormal);

    matrix_t Av(2, 3);
    Av.row(0) = lateralWorld.transpose();
    Av.row(1) = terrainNormal.transpose();
    return Av;
}

vector3_t WheelRollingVelocityConstraintCppAd::getTangentProjectionOrFallback(const vector3_t& direction,
                                                                              const vector3_t& normal) {
    vector3_t lateral = direction - direction.dot(normal) * normal;
    if (lateral.allFinite() && lateral.norm() > 1e-6) {
        return lateral.normalized();
    }

    lateral = vector3_t::UnitY() - vector3_t::UnitY().dot(normal) * normal;
    if (lateral.allFinite() && lateral.norm() > 1e-6) {
        return lateral.normalized();
    }

    lateral = vector3_t::UnitX() - vector3_t::UnitX().dot(normal) * normal;
    if (lateral.allFinite() && lateral.norm() > 1e-6) {
        return lateral.normalized();
    }

    return normal.unitOrthogonal().normalized();
}

}  // namespace ocs2::legged_robot

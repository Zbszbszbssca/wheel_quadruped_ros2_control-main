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

#include "ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h"

#include <array>
#include <cmath>

namespace ocs2::legged_robot {



    // 构造函数：初始化新增的末端执行器运动学
    SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                                const EndEffectorKinematics<scalar_t>& endEffectorKinematics)
        : ReferenceManager(TargetTrajectories(), ModeSchedule()),
        gaitSchedulePtr_(std::move(gaitSchedulePtr)),
        swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
        endEffectorKinematicsPtr_(endEffectorKinematics.clone()) {}




    void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule &modeSchedule) {
        ReferenceManager::setModeSchedule(modeSchedule);
        gaitSchedulePtr_->setModeSchedule(modeSchedule);
    }


    contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
        return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
    }

    bool SwitchedModelReferenceManager::hasTerrainNormalEstimate() const {
        return kalman_filter_ptr_ != nullptr && kalman_filter_ptr_->hasTerrainNormalEstimate();
    }

    vector3_t SwitchedModelReferenceManager::getTerrainNormalWorldOrUnitZ() const {
        if (hasTerrainNormalEstimate()) {
            vector3_t terrainNormal = kalman_filter_ptr_->getTerrainNormalWorld();
            if (terrainNormal.allFinite() && terrainNormal.norm() > 1e-6) {
                terrainNormal.normalize();
                if (terrainNormal.z() < 0.0) {
                    terrainNormal = -terrainNormal;
                }

                // MPC 中的摩擦锥/支撑速度约束不能使用启动阶段或异常接触估计产生的近水平法向，
                // 否则 standing 初期会把重力支撑力错误分解到切向，导致摩擦锥不可行并污染可视化。
                // 这里先做保守保护：超过约 45deg 的估计直接退回世界系 z 轴。
                constexpr scalar_t kMinNormalZForMpc = 0.70;
                if (terrainNormal.z() >= kMinNormalZForMpc) {
                    return terrainNormal;
                }
            }
        }
        return vector3_t::UnitZ();
    }



    // 收集最近的接触点（仅保留接触状态为true的脚的位置）
    void SwitchedModelReferenceManager::collectContactPoints(const vector_t& initState, const contact_flag_t& contactFlags) {
        // 获取当前所有脚的位置（基于关节运动学测量，参考论文“融合IMU和关节测量”）
        const auto footPositions = endEffectorKinematicsPtr_->getPosition(initState);

        for (size_t leg = 0; leg < 4; ++leg) {
            if (contactFlags[leg]) {  // 仅收集接触状态为true的脚的位置
                recentContactPoints_.push_back(footPositions[leg]);
                if (recentContactPoints_.size() > maxContactPoints_) {
                    recentContactPoints_.pop_front();  // 保持最新的4个点
                }
            }
        }
    }

    // 拟合地形平面：ax + by + cz + d = 0，返回平面参数(a,b,c,d)
    vector4_t SwitchedModelReferenceManager::fitTerrainPlane() const {
        if (recentContactPoints_.size() < 3) {  // 至少3个点拟合平面
            return {0.0, 0.0, 1.0, 0.0};  // 默认平面（z=0）
        }

        // 最小二乘法拟合平面（参考论文“通过最近接触点拟合平面”）
        Eigen::MatrixXd A(recentContactPoints_.size(), 3);
        Eigen::VectorXd b(recentContactPoints_.size());
        for (size_t i = 0; i < recentContactPoints_.size(); ++i) {
            const auto& p = recentContactPoints_[i];
            A.row(i) = Eigen::Vector3d(p.x(), p.y(), 1.0f);  // 平面方程：z = ax + by + c → ax + by - z + c = 0
            b(i) = p.z();
        }

        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);  // 求解a, b, c
        return {x(0), x(1), -1.0, x(2)};  // (a, b, c, d)对应ax + by + cz + d = 0
    }

    // 计算给定位置(x,y)在地形平面上的高度z
    scalar_t SwitchedModelReferenceManager::getTerrainHeight(const vector3_t& footPos, const vector4_t& plane) const {
        // 平面方程：a*x + b*y + c*z + d = 0 → z = -(a*x + b*y + d)/c
        scalar_t a = plane(0), b = plane(1), c = plane(2), d = plane(3);
        if (std::fabs(c) < 1e-6) {
            return 0.0;  // 避免除以0
        }
        return -(a * footPos.x() + b * footPos.y() + d) / c;
    }


    // void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime,
    //                                                      const vector_t &initState,
    //                                                      TargetTrajectories &targetTrajectories,
    //                                                      ModeSchedule &modeSchedule) {
    //     const auto timeHorizon = finalTime - initTime;
    //     modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

    //     const scalar_t terrainHeight = 0.0;
    //     swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
    // }

    void SwitchedModelReferenceManager::modifyReferences(
        scalar_t initTime,
        scalar_t finalTime,
        const vector_t &initState,
        TargetTrajectories &targetTrajectories,
        ModeSchedule &modeSchedule)
    {
        (void)targetTrajectories;

        const auto timeHorizon = finalTime - initTime;

        modeSchedule =
            gaitSchedulePtr_->getModeSchedule(
                initTime - timeHorizon,
                finalTime + timeHorizon);

        /* -------- 1 获取接触状态 -------- */

        const auto contactFlags = getContactFlags(initTime);

        /* -------- 2 获取足端位置 -------- */

        const auto footPositions =
            endEffectorKinematicsPtr_->getPosition(initState);

        /* -------- 3 收集接触点 -------- */

        collectContactPoints(initState, contactFlags);

        /* -------- 4 拟合地形平面 -------- */

        vector4_t plane = fitTerrainPlane();

        /* -------- 5 估计每条腿的局部地形高度（先求稳：不再额外叠加坡度补偿） -------- */

        auto* kf = kalman_filter_ptr_;

        const bool kfAvailable =
            (kf != nullptr) &&
            (kf->getTerrainHeightPerLeg().size() == 4);

        // 当前每条腿对应的局部地形高度
        std::array<scalar_t, 4> legTerrainHeight{0.0, 0.0, 0.0, 0.0};

        for (size_t leg = 0; leg < 4; ++leg)
        {
            // 先用当前接触平面在该脚 (x,y) 的高度作为基础高度
            const scalar_t planeHeight =
                getTerrainHeight(
                    footPositions[leg],
                    plane);

            scalar_t localHeight = planeHeight;

            // 如果 Kalman 的单腿高度和当前平面高度接近，则轻微融合
            // 目的：保留一点真实接触高度信息，但避免 swing 腿旧高度把参考带偏
            if (kfAvailable)
            {
                const scalar_t kfHeight = kf->getTerrainHeightPerLeg()(leg);

                if (std::isfinite(kfHeight) && std::abs(kfHeight - planeHeight) < 0.03)
                {
                    localHeight = 0.7 * planeHeight + 0.3 * kfHeight;
                }
            }

            legTerrainHeight[leg] = localHeight;
        }

        /* -------- 6 生成每腿独立的高度序列（先不做 touchdown 额外抬高） -------- */

        const size_t numPhases = modeSchedule.modeSequence.size();

        feet_array_t<scalar_array_t> liftOffHeightSequence;
        feet_array_t<scalar_array_t> touchDownHeightSequence;

        for (size_t leg = 0; leg < 4; ++leg)
        {
            liftOffHeightSequence[leg].resize(numPhases);
            touchDownHeightSequence[leg].resize(numPhases);

            const scalar_t baseHeight = legTerrainHeight[leg];

            // 这一版先求稳：
            // liftOff 和 touchDown 先都用同一个局部地形高度
            // 不再额外叠加 slopeComp，避免在楼梯上重复补偿导致抬脚过高
            for (size_t p = 0; p < numPhases; ++p)
            {
                liftOffHeightSequence[leg][p] = baseHeight;
                touchDownHeightSequence[leg][p] = baseHeight;
            }
        }

        /* -------- 7 更新 Swing trajectory -------- */

        swingTrajectoryPtr_->update(
            modeSchedule,
            liftOffHeightSequence,
            touchDownHeightSequence);
    }



    // void SwitchedModelReferenceManager::modifyReferences(
    //     scalar_t initTime,
    //     scalar_t finalTime,
    //     const vector_t& initState,
    //     TargetTrajectories& targetTrajectories,
    //     ModeSchedule& modeSchedule)
    // {
    //     (void)initState;
    //     (void)targetTrajectories;

    //     const auto timeHorizon = finalTime - initTime;

    //     modeSchedule =
    //         gaitSchedulePtr_->getModeSchedule(
    //             initTime - timeHorizon,
    //             finalTime + timeHorizon);

    //     /* -------- 1 直接使用 KalmanFilter 的实时地形估计 -------- */

    //     auto* kf = kalman_filter_ptr_;

    //     const bool kfAvailable =
    //         (kf != nullptr) &&
    //         (kf->getTerrainHeightPerLeg().size() == 4);

    //     // 如果滤波器当前不可用，先直接返回，保持上一帧参考不变
    //     if (!kfAvailable)
    //     {
    //         return;
    //     }

    //     const auto terrainHeightPerLeg = kf->getTerrainHeightPerLeg();
    //     const scalar_t fusedTerrainHeight = kf->getFusedTerrainHeight();

    //     /* -------- 2 构造每条腿的局部地形高度 -------- */

    //     std::array<scalar_t, 4> legTerrainHeight{0.0, 0.0, 0.0, 0.0};

    //     for (size_t leg = 0; leg < 4; ++leg)
    //     {
    //         const scalar_t kfHeight = terrainHeightPerLeg(leg);

    //         // 先完全信 Kalman 的单腿地形高度；
    //         // 若该腿高度异常，则退回融合地形高度
    //         scalar_t localHeight = fusedTerrainHeight;
    //         if (std::isfinite(kfHeight))
    //         {
    //             localHeight = kfHeight;
    //         }

    //         legTerrainHeight[leg] = localHeight;
    //     }

    //     /* -------- 3 生成每腿独立的高度序列（当前版本：per-leg，所有 phase 先统一） -------- */

    //     const size_t numPhases = modeSchedule.modeSequence.size();

    //     feet_array_t<scalar_array_t> liftOffHeightSequence;
    //     feet_array_t<scalar_array_t> touchDownHeightSequence;

    //     for (size_t leg = 0; leg < 4; ++leg)
    //     {
    //         liftOffHeightSequence[leg].resize(numPhases);
    //         touchDownHeightSequence[leg].resize(numPhases);

    //         const scalar_t baseHeight = legTerrainHeight[leg];

    //         for (size_t p = 0; p < numPhases; ++p)
    //         {
    //             liftOffHeightSequence[leg][p]  = baseHeight;
    //             touchDownHeightSequence[leg][p] = baseHeight;
    //         }
    //     }

    //     /* -------- 4 更新 Swing trajectory -------- */

    //     swingTrajectoryPtr_->update(
    //         modeSchedule,
    //         liftOffHeightSequence,
    //         touchDownHeightSequence);
    // }


} // namespace ocs2::legged_robot





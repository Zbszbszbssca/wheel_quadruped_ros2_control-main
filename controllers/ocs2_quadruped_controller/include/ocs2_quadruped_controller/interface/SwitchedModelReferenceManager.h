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
#include <deque>  // 新增：引入std::deque的定义
#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>// 新增
// 线性卡尔曼滤波器头文件：四足机器人状态估计核心实现
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
#include <ocs2_legged_robot/common/Types.h>// 新增
#include "constraint/SwingTrajectoryPlanner.h"

namespace ocs2::legged_robot {
    using vector4_t = Eigen::Matrix<scalar_t, 4, 1>; // 新增
    /**
    * Manages the ModeSchedule and the TargetTrajectories for switched model.
    */
    class SwitchedModelReferenceManager : public ReferenceManager {
    public:
        SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                    std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics);
        ~SwitchedModelReferenceManager() override = default;

        void setModeSchedule(const ModeSchedule &modeSchedule) override;

        contact_flag_t getContactFlags(scalar_t time) const;

        const std::shared_ptr<GaitSchedule> &getGaitSchedule() { return gaitSchedulePtr_; }

        const std::shared_ptr<SwingTrajectoryPlanner> &getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }
        const std::shared_ptr<SwingTrajectoryPlanner> &getSwingTrajectoryPlanner() const { return swingTrajectoryPtr_; }

        void setKalmanFilter(
            KalmanFilterEstimate* kf)
        {
            kalman_filter_ptr_ = kf;
        }

        bool hasTerrainNormalEstimate() const;
        vector3_t getTerrainNormalWorldOrUnitZ() const;

    protected:
        void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                              TargetTrajectories &targetTrajectories,
                              ModeSchedule &modeSchedule) override;

        // 辅助函数：收集最近接触点
        void collectContactPoints(const vector_t& initState, const contact_flag_t& contactFlags);

        // 辅助函数：最小二乘拟合地形平面（ax + by + cz + d = 0）
        vector4_t fitTerrainPlane() const;

        // 辅助函数：根据拟合平面计算指定脚位置的地形高度
        scalar_t getTerrainHeight(const vector3_t& footPos, const vector4_t& plane) const;     
        std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
        std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;         
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;  // 末端执行器运动学（获取脚位置）

        // 地形拟合相关
        std::deque<vector3_t> recentContactPoints_;  // 存储最近接触点（最多4个）
        const size_t maxContactPoints_ = 4;         // 拟合平面的最大接触点数量
        KalmanFilterEstimate* kalman_filter_ptr_{nullptr};

    };
}

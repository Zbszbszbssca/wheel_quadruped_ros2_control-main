/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

// Pinocchio前向声明：避免包含完整头文件，加速编译
#include <pinocchio/fwd.hpp>

// Pinocchio核心算法头文件：运动学、雅克比、帧变换计算
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// OCS2质心模型辅助函数：质心动力学计算
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
// OCS2数值工具：浮点数比较
#include <ocs2_core/misc/Numerics.h>

// 当前类头文件：声明LeggedRobotPreComputation类
#include "ocs2_quadruped_controller/interface/LeggedRobotPreComputation.h"

// 命名空间：ocs2的legged_robot模块，封装四足机器人相关控制逻辑
namespace ocs2::legged_robot {

/**
 * @brief LeggedRobotPreComputation构造函数
 * @param pinocchioInterface [in] Pinocchio机器人模型接口（包含模型和数据）
 * @param info [in] 质心模型信息结构体（接触点数量、维度等）
 * @param swingTrajectoryPlanner [in] 摆动轨迹规划器实例（用于获取摆动腿速度/位置约束）
 * @param settings [in] 模型配置参数（位置误差增益等）
 * @note 该类是OCS2预计算接口的实现，封装四足机器人运动学、约束配置等预计算逻辑
 */
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface,
                                                     CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner &swingTrajectoryPlanner,
                                                     ModelSettings settings)
    // 移动语义保存Pinocchio接口（避免拷贝开销）
    : pinocchioInterface_(std::move(pinocchioInterface)),
      // 移动语义保存质心模型信息
      info_(std::move(info)),
      // 保存摆动轨迹规划器指针（只读，无需拷贝）
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      // 创建质心模型->Pinocchio映射器（用于状态/输入与关节空间的转换）
      mappingPtr_(new CentroidalModelPinocchioMapping(info_)),
      // 移动语义保存模型配置参数
      settings_(std::move(settings)) {
    // 初始化末端执行器法向速度约束配置数组（长度=接触点数量）
    eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
    // 为映射器设置Pinocchio接口（关联机器人模型）
    mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/**
 * @brief 拷贝构造函数
 * @param rhs [in] 要拷贝的LeggedRobotPreComputation实例
 * @note 深拷贝映射器指针（clone()），确保每个实例拥有独立的映射器；Pinocchio接口浅拷贝（只读）
 */
LeggedRobotPreComputation::LeggedRobotPreComputation(const LeggedRobotPreComputation &rhs)
    // 浅拷贝Pinocchio接口（模型数据只读，无需深拷贝）
    : pinocchioInterface_(rhs.pinocchioInterface_),
      // 拷贝质心模型信息
      info_(rhs.info_),
      // 拷贝摆动轨迹规划器指针
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      // 深拷贝质心模型映射器（避免多个实例共享同一映射器）
      mappingPtr_(rhs.mappingPtr_->clone()),
      // 拷贝模型配置参数
      settings_(rhs.settings_) {
    // 初始化法向速度约束配置数组（与源实例长度一致）
    eeNormalVelConConfigs_.resize(rhs.eeNormalVelConConfigs_.size());
    // 为新映射器设置Pinocchio接口
    mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/**
 * @brief 执行预计算请求（核心函数）
 * @param request [in] 预计算请求集合（指定需要计算的内容：代价、约束、近似等）
 * @param t [in] 当前时间戳
 * @param x [in] 系统状态向量（质心状态+关节状态）
 * @param u [in] 系统输入向量（接触力+关节力矩）
 * @note 该函数是OCS2框架的核心接口，根据请求类型执行必要的预计算：
 *       1. 约束请求：配置末端执行器法向速度约束参数
 *       2. 近似请求：执行完整运动学计算+质心动力学导数计算
 *       3. 基础请求：仅执行前向运动学+帧位置更新
 */
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t &x, const vector_t &u) {
    // 如果请求不包含代价/约束/软约束相关计算，直接返回（无需预计算）
    if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
        return;
    }

    // ---------------- Lambda表达式：生成单条腿的法向速度约束配置 ----------------
    // 功能：根据摆动轨迹规划器的目标速度/位置，生成EndEffectorLinearConstraint的配置参数
    auto eeNormalVelConConfig = [&](size_t footIndex) {
        // 初始化约束配置结构体
        EndEffectorLinearConstraint::Config config;
        
        // 配置b向量（1维）：-目标Z轴速度（约束公式：Av*vel + Ax*pos + b = 0 → vel_z = 目标速度）
        config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
        
        // 配置Av矩阵（1×3）：[0,0,1] → 仅约束Z轴速度
        config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
        
        // 如果位置误差增益非零，添加位置约束项（增强轨迹跟踪精度）
        if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
            // 更新b向量：b -= 位置误差增益 × 目标Z轴位置
            config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
            // 配置Ax矩阵（1×3）：[0,0,位置误差增益] → 约束Z轴位置
            config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
        }
        
        return config;
    };

    // ---------------- 约束请求处理：生成所有腿的法向速度约束配置 ----------------
    if (request.contains(Request::Constraint)) {
        // 遍历所有接触点（腿），生成对应的法向速度约束配置
        for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
            eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
        }
    }

    // ---------------- 运动学计算：根据请求类型执行不同级别的计算 ----------------
    // 获取Pinocchio模型（只读）和数据（可写）
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    
    // 将系统状态向量转换为Pinocchio关节位置（q）
    vector_t q = mappingPtr_->getPinocchioJointPosition(x);

    // 如果请求包含近似计算（需要雅克比/导数）：执行完整运动学+质心动力学计算
    if (request.contains(Request::Approximation)) {
        // 1. 前向运动学：计算关节位置对应的连杆位置/速度
        forwardKinematics(model, data, q);
        // 2. 更新所有帧的位置（包含末端执行器）
        updateFramePlacements(model, data);
        // 3. 更新全局位置（基座、连杆等）
        updateGlobalPlacements(model, data);
        // 4. 计算关节雅克比矩阵（用于导数计算）
        computeJointJacobians(model, data);

        // 5. 更新质心动力学（基于当前关节位置）
        updateCentroidalDynamics(pinocchioInterface_, info_, q);
        // 6. 将系统状态/输入转换为Pinocchio关节速度（v）
        vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
        // 7. 更新质心动力学导数（用于MPC优化的梯度计算）
        updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);

    } else {
        // 基础请求（仅需要位置）：仅执行前向运动学+帧位置更新
        forwardKinematics(model, data, q);
        updateFramePlacements(model, data);
    }
}

} // namespace ocs2::legged_robot

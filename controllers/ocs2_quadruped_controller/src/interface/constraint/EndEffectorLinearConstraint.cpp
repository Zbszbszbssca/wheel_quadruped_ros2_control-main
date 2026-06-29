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

//声明末端执行器线性约束类
#include "ocs2_quadruped_controller/interface/constraint/EndEffectorLinearConstraint.h"

// 命名空间：ocs2的legged_robot模块，封装四足机器人相关控制逻辑
namespace ocs2::legged_robot {

/**
 * @brief 末端执行器线性约束类构造函数
 * @param endEffectorKinematics [in] 末端执行器运动学接口，用于计算位置/速度及其雅克比
 * @param numConstraints [in] 约束维度（如位置约束3维、法向速度约束1维）
 * @param config [in] 约束配置参数（包含Ax/Av矩阵、b向量，定义线性约束：Ax*pos + Av*vel + b = 0）
 * @note 该类是通用线性约束模板，可用于位置、速度、法向速度等多种末端执行器约束场景
 */
EndEffectorLinearConstraint::EndEffectorLinearConstraint(
    const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
    size_t numConstraints, Config config)
    // 父类构造：声明约束类型为线性约束（ConstraintOrder::Linear）
    : StateInputConstraint(ConstraintOrder::Linear),
      // 深拷贝运动学接口（clone()确保每个约束实例拥有独立的运动学计算实例）
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      // 保存约束维度
      numConstraints_(numConstraints),
      // 移动语义保存配置参数
      config_(std::move(config)) {
    // 校验：该类仅支持单个末端执行器的约束（不支持多末端执行器批量约束）
    if (endEffectorKinematicsPtr_->getIds().size() != 1) {
        throw std::runtime_error(
            "[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
    }
}

/**
 * @brief 拷贝构造函数
 * @param rhs [in] 要拷贝的EndEffectorLinearConstraint实例
 * @note 深拷贝运动学接口指针，避免多个实例共享同一资源导致的内存管理问题
 */
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorLinearConstraint &rhs)
    // 调用父类的拷贝构造函数
    : StateInputConstraint(rhs),
      // 深拷贝运动学接口实例
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      // 拷贝约束维度
      numConstraints_(rhs.numConstraints_),
      // 拷贝配置参数
      config_(rhs.config_) {
}

/**
 * @brief 动态配置约束参数（更新Ax/Av/b矩阵）
 * @param config [in] 新的约束配置参数（右值引用，优化性能）
 * @note 包含严格的维度校验，确保配置参数与约束维度匹配，避免运行时维度错误
 */
void EndEffectorLinearConstraint::configure(Config &&config) {
    // 校验1：b向量的行数必须等于约束维度
    assert(config.b.rows() == numConstraints_);
    // 校验2：必须至少配置位置矩阵(Ax)或速度矩阵(Av)中的一个（不能全为空）
    assert(config.Ax.size() > 0 || config.Av.size() > 0);
    // 校验3：Ax矩阵非空时，行数必须等于约束维度
    assert((config.Ax.size() > 0 && config.Ax.rows() == numConstraints_) || config.Ax.size() == 0);
    // 校验4：Ax矩阵非空时，列数必须为3（对应x/y/z三个方向）
    assert((config.Ax.size() > 0 && config.Ax.cols() == 3) || config.Ax.size() == 0);
    // 校验5：Av矩阵非空时，行数必须等于约束维度
    assert((config.Av.size() > 0 && config.Av.rows() == numConstraints_) || config.Av.size() == 0);
    // 校验6：Av矩阵非空时，列数必须为3（对应vx/vy/vz三个方向）
    assert((config.Av.size() > 0 && config.Av.cols() == 3) || config.Av.size() == 0);
    
    // 保存新的配置参数（移动语义）
    config_ = std::move(config);
}

/**
 * @brief 计算约束的当前值
 * @param time [in] 当前时间戳（当前未使用）
 * @param state [in] 系统状态向量（用于计算末端执行器位置/速度）
 * @param input [in] 系统输入向量（用于计算末端执行器速度）
 * @param preComp [in] 预计算数据（当前未使用）
 * @return 约束值向量（维度=numConstraints_），计算公式：f = Ax*pos + Av*vel + b
 * @note 仅计算配置了的项（Ax/Av为空时跳过对应项）
 */
vector_t EndEffectorLinearConstraint::getValue(scalar_t time, const vector_t &state, const vector_t &input,
                                               const PreComputation &preComp) const {
    // 初始化约束值为b向量
    vector_t f = config_.b;
    
    // 如果配置了位置矩阵Ax，计算Ax*pos并累加到约束值
    if (config_.Ax.size() > 0) {
        // getPosition(state).front()：获取单个末端执行器的位置（3维向量）
        f.noalias() += config_.Ax * endEffectorKinematicsPtr_->getPosition(state).front();
    }
    
    // 如果配置了速度矩阵Av，计算Av*vel并累加到约束值
    if (config_.Av.size() > 0) {
        // getVelocity(state, input).front()：获取单个末端执行器的速度（3维向量）
        f.noalias() += config_.Av * endEffectorKinematicsPtr_->getVelocity(state, input).front();
    }
    
    return f;
}

/**
 * @brief 计算约束的线性近似（一阶导数/雅克比矩阵）
 * @param time [in] 当前时间戳（当前未使用）
 * @param state [in] 系统状态向量
 * @param input [in] 系统输入向量
 * @param preComp [in] 预计算数据（当前未使用）
 * @return 约束的线性近似结构（约束值f、状态雅克比dfdx、输入雅克比dfdu）
 * @note 线性近似公式：
 *       f = Ax*pos + Av*vel + b
 *       dfdx = Ax*dpos/dx + Av*dvel/dx
 *       dfdu = Av*dvel/du （位置对输入无导数）
 */
VectorFunctionLinearApproximation EndEffectorLinearConstraint::getLinearApproximation(
    scalar_t time, const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const {
    // 初始化线性近似结构为全零矩阵
    // 维度：约束数 × 状态维度 × 输入维度
    VectorFunctionLinearApproximation linearApproximation =
            VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

    // 初始化约束值为b向量
    linearApproximation.f = config_.b;

    // 如果配置了位置矩阵Ax，计算位置项的贡献
    if (config_.Ax.size() > 0) {
        // 获取末端执行器位置的线性近似（位置值pos + 状态雅克比dpos/dx）
        const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
        // 累加位置项到约束值：Ax*pos
        linearApproximation.f.noalias() += config_.Ax * positionApprox.f;
        // 累加位置项到状态雅克比：Ax*dpos/dx
        linearApproximation.dfdx.noalias() += config_.Ax * positionApprox.dfdx;
    }

    // 如果配置了速度矩阵Av，计算速度项的贡献
    if (config_.Av.size() > 0) {
        // 获取末端执行器速度的线性近似（速度值vel + 状态雅克比dvel/dx + 输入雅克比dvel/du）
        const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
        // 累加速度项到约束值：Av*vel
        linearApproximation.f.noalias() += config_.Av * velocityApprox.f;
        // 累加速度项到状态雅克比：Av*dvel/dx
        linearApproximation.dfdx.noalias() += config_.Av * velocityApprox.dfdx;
        // 累加速度项到输入雅克比：Av*dvel/du
        linearApproximation.dfdu.noalias() += config_.Av * velocityApprox.dfdu;
    }

    return linearApproximation;
}

} // namespace ocs2::legged_robot

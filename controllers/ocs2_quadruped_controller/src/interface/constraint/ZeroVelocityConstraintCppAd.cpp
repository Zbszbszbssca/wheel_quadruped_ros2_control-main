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


#include "ocs2_quadruped_controller/interface/constraint/ZeroVelocityConstraintCppAd.h"

// 命名空间：ocs2的legged_robot模块，封装四足机器人相关控制逻辑
namespace ocs2::legged_robot {

/**
 * @brief 基于CppAD自动微分的末端执行器零速度约束类构造函数
 * @param referenceManager [in] 切换模型参考管理器，用于获取机器人接触状态等参考信息
 * @param endEffectorKinematics [in] 末端执行器运动学计算接口，用于求解末端速度
 * @param contactPointIndex [in] 要约束的接触点索引（四足机器人0-3对应四条腿）
 * @param config [in] 末端执行器线性约束配置参数（如约束类型、上下限等）
 * @note CppAD版本通过自动微分实现雅克比矩阵的精确计算，避免手动推导的误差
 */
ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager &referenceManager,
                                                         const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
                                                         size_t contactPointIndex,
                                                         EndEffectorLinearConstraint::Config config)
    // 父类构造：声明约束类型为线性约束（ConstraintOrder::Linear）
    : StateInputConstraint(ConstraintOrder::Linear),
      // 保存参考管理器指针，用于运行时判断接触状态
      referenceManagerPtr_(&referenceManager),
      // 创建末端执行器线性约束实例（3表示约束维度：x/y/z方向速度）
      // 使用智能指针管理内存，std::move优化配置参数的传递
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 3, std::move(config))),
      // 保存要约束的接触点索引
      contactPointIndex_(contactPointIndex) {
}

/**
 * @brief 拷贝构造函数
 * @param rhs [in] 要拷贝的ZeroVelocityConstraintCppAd实例
 * @note 深拷贝末端执行器约束指针，避免多个实例共享同一资源导致的内存问题
 */
ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd &rhs)
    // 调用父类的拷贝构造函数
    : StateInputConstraint(rhs),
      // 拷贝参考管理器指针（只读，无需深拷贝）
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      // 深拷贝末端执行器约束实例（clone()确保创建独立的新实例）
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
      // 拷贝接触点索引
      contactPointIndex_(rhs.contactPointIndex_) {
}

/**
 * @brief 判断当前约束是否激活
 * @param time [in] 当前时间戳
 * @return 约束激活状态：接触点接触地面时返回true（激活零速度约束），非接触时返回false
 * @note 核心逻辑：接触阶段需要约束末端执行器（脚掌）速度为零，保证落地稳定性
 */
bool ZeroVelocityConstraintCppAd::isActive(scalar_t time) const {
    // 获取指定时间的接触标志数组，接触点接触地面时激活约束
    return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/**
 * @brief 获取约束的当前值
 * @param time [in] 当前时间戳
 * @param state [in] 系统状态向量（包含质心状态、关节状态等）
 * @param input [in] 系统输入向量（包含接触力、关节力矩等）
 * @param preComp [in] 预计算数据（如运动学预计算结果）
 * @return 3维向量，表示末端执行器当前速度（vx, vy, vz）
 * @note 约束目标：激活时该返回值应等于零向量（零速度）
 */
vector_t ZeroVelocityConstraintCppAd::getValue(scalar_t time, const vector_t &state, const vector_t &input,
                                               const PreComputation &preComp) const {
    // 委托给末端执行器线性约束实例计算约束值（封装了速度求解逻辑）
    return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/**
 * @brief 获取约束的线性近似（雅克比矩阵）
 * @param time [in] 当前时间戳
 * @param state [in] 系统状态向量
 * @param input [in] 系统输入向量
 * @param preComp [in] 预计算数据
 * @return 约束的线性近似结构（约束值、状态雅克比df/dx、输入雅克比df/du）
 * @note CppAD会自动计算雅克比矩阵，无需手动编写导数计算逻辑，精度更高
 */
VectorFunctionLinearApproximation ZeroVelocityConstraintCppAd::getLinearApproximation(
    scalar_t time, const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const {
    // 委托给末端执行器线性约束实例计算线性近似（自动微分核心逻辑在此封装）
    return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

} // namespace ocs2::legged_robot

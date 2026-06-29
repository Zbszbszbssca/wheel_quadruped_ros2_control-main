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


#include "ocs2_quadruped_controller/interface/constraint/ZeroForceConstraint.h"
// 包含质心模型的辅助函数头文件，用于接触力的获取
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

// 命名空间：ocs2的legged_robot模块，包含四足机器人相关的控制逻辑
namespace ocs2::legged_robot {

/**
 * @brief 零力约束类的构造函数
 * @param referenceManager [in] 切换模型参考管理器，用于获取机器人接触状态等参考信息
 * @param contactPointIndex [in] 要约束的接触点索引（例如四足机器人的0-3分别对应四条腿）
 * @param info [in] 质心模型信息结构体，包含模型维度、接触点数量等关键参数
 */
ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManager &referenceManager,
                                         size_t contactPointIndex,
                                         CentroidalModelInfo info)
    // 父类构造：声明约束类型为线性约束（ConstraintOrder::Linear）
    : StateInputConstraint(ConstraintOrder::Linear),
      // 保存参考管理器的指针（用于运行时获取接触状态）
      referenceManagerPtr_(&referenceManager),
      // 保存要约束的接触点索引
      contactPointIndex_(contactPointIndex),
      // 移动语义保存质心模型信息（避免拷贝，提高效率）
      info_(std::move(info)) {
}

/**
 * @brief 判断当前约束是否激活
 * @param time [in] 当前时间戳
 * @return 约束激活状态：当指定接触点未接触地面时返回true（激活约束），接触时返回false（禁用约束）
 * @note 核心逻辑：非接触阶段需要约束该接触点的力为零
 */
bool ZeroForceConstraint::isActive(scalar_t time) const {
    // 获取指定时间的接触标志数组，并检查目标接触点的状态
    // !contactFlag 表示：接触点未接触地面时，激活零力约束
    return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/**
 * @brief 获取约束的当前值
 * @param time [in] 当前时间戳
 * @param state [in] 系统状态向量（包含质心状态、关节状态等）
 * @param input [in] 系统输入向量（包含接触力、关节力矩等）
 * @param preComp [in] 预计算数据（当前未使用）
 * @return 3维向量，表示指定接触点的接触力（Fx, Fy, Fz）
 * @note 约束目标：当约束激活时，该返回值应等于零向量
 */
vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t &state, const vector_t &input,
                                       const PreComputation &preComp) const {
    // 从输入向量中提取指定接触点的接触力
    return centroidal_model::getContactForces(input, contactPointIndex_, info_);
}

/**
 * @brief 获取约束的线性近似（雅克比矩阵）
 * @param time [in] 当前时间戳
 * @param state [in] 系统状态向量
 * @param input [in] 系统输入向量
 * @param preComp [in] 预计算数据（当前未使用）
 * @return 约束的线性近似结构（包含约束值、状态雅克比、输入雅克比）
 * @note 线性近似用于模型预测控制(MPC)中的优化问题构建
 */
VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(
    scalar_t time, const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const {
    // 初始化线性近似结构
    VectorFunctionLinearApproximation approx;
    
    // 1. 设置约束的当前值（与getValue返回值一致）
    approx.f = getValue(time, state, input, preComp);
    
    // 2. 设置状态雅克比矩阵（df/dx）：3行×状态维度列的零矩阵
    // 原因：接触力约束仅与输入（接触力）相关，与状态无关
    approx.dfdx = matrix_t::Zero(3, state.size());
    
    // 3. 设置输入雅克比矩阵（df/du）：3行×输入维度列的矩阵
    approx.dfdu = matrix_t::Zero(3, input.size());
    
    // 4. 填充输入雅克比的有效部分：
    //    接触力在输入向量中按接触点索引连续存储（每个接触点3个分量：Fx,Fy,Fz）
    //    middleCols<3>(3*contactPointIndex_) 定位到目标接触点的力分量列
    //    diagonal() = Ones(3) 表示：接触力对自身输入的偏导数为1（线性关系）
    approx.dfdu.middleCols<3>(3 * contactPointIndex_).diagonal() = vector_t::Ones(3);
    
    // 返回构建好的线性近似结构
    return approx;
}

} // namespace ocs2::legged_robot

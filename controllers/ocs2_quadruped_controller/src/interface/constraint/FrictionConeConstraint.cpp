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

// 声明摩擦力锥约束类
#include "ocs2_quadruped_controller/interface/constraint/FrictionConeConstraint.h"

// OCS2质心模型辅助函数：用于从输入向量中提取接触力
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

// 命名空间：ocs2的legged_robot模块，封装四足机器人相关控制逻辑
namespace ocs2::legged_robot {

/**
 * @brief 摩擦力锥约束类构造函数
 * @param referenceManager [in] 切换模型参考管理器，用于获取机器人接触状态
 * @param config [in] 摩擦力锥配置参数（摩擦系数、正则化项、hessian偏移等）
 * @param contactPointIndex [in] 要约束的接触点索引（四足机器人0-3对应四条腿）
 * @param info [in] 质心模型信息结构体，包含输入向量维度、接触力存储格式等
 * @note 约束类型为二次约束（ConstraintOrder::Quadratic），因摩擦力锥约束包含平方/平方根项
 */
FrictionConeConstraint::FrictionConeConstraint(const SwitchedModelReferenceManager &referenceManager, Config config,
                                               size_t contactPointIndex, CentroidalModelInfo info)
    // 父类构造：声明约束类型为二次约束（包含二阶导数）
    : StateInputConstraint(ConstraintOrder::Quadratic),
      // 保存参考管理器指针，用于判断接触状态
      referenceManagerPtr_(&referenceManager),
      // 移动语义保存配置参数（避免拷贝开销）
      config_(std::move(config)),
      // 保存要约束的接触点索引
      contactPointIndex_(contactPointIndex),
      // 移动语义保存质心模型信息
      info_(std::move(info)) {
}

/**
 * @brief 设置世界坐标系下的表面法向量（未实现）
 * @param surfaceNormalInWorld [in] 世界坐标系下的表面法向量
 * @note 该函数预留接口用于处理非平面地形的摩擦力锥约束，当前版本未实现，调用会抛出异常
 */
void FrictionConeConstraint::setSurfaceNormalInWorld(const vector3_t &surfaceNormalInWorld) {
    // 根据世界系地形法向构造 world -> terrain 旋转；terrain z 轴与地形法向对齐。
    t_R_w = getRotationWorldToTerrain(surfaceNormalInWorld);
}

matrix3_t FrictionConeConstraint::getRotationWorldToTerrain(const vector3_t &surfaceNormalInWorld) {
    vector3_t terrainZ = surfaceNormalInWorld;
    if (!terrainZ.allFinite() || terrainZ.norm() < 1e-6) {
        terrainZ = vector3_t::UnitZ();
    } else {
        terrainZ.normalize();
    }
    if (terrainZ.z() < 0.0) {
        terrainZ = -terrainZ;
    }

    vector3_t terrainX = vector3_t::UnitX() - vector3_t::UnitX().dot(terrainZ) * terrainZ;
    if (!terrainX.allFinite() || terrainX.norm() < 1e-6) {
        terrainX = vector3_t::UnitY() - vector3_t::UnitY().dot(terrainZ) * terrainZ;
    }
    if (!terrainX.allFinite() || terrainX.norm() < 1e-6) {
        terrainX = terrainZ.unitOrthogonal();
    }
    terrainX.normalize();

    vector3_t terrainY = terrainZ.cross(terrainX);
    terrainY.normalize();

    matrix3_t terrain_R_world;
    terrain_R_world.row(0) = terrainX.transpose();
    terrain_R_world.row(1) = terrainY.transpose();
    terrain_R_world.row(2) = terrainZ.transpose();
    return terrain_R_world;
}

matrix3_t FrictionConeConstraint::getCurrentRotationWorldToTerrain() const {
    return getRotationWorldToTerrain(referenceManagerPtr_->getTerrainNormalWorldOrUnitZ());
}

/**
 * @brief 判断当前约束是否激活
 * @param time [in] 当前时间戳
 * @return 约束激活状态：接触点接触地面时返回true（激活摩擦力锥约束），非接触时返回false
 * @note 核心逻辑：仅支撑阶段需要约束接触力在摩擦力锥内，防止打滑
 */
bool FrictionConeConstraint::isActive(scalar_t time) const {
    // 获取指定时间的接触标志数组，接触阶段激活约束
    return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/**
 * @brief 获取摩擦力锥约束的当前值
 * @param time [in] 当前时间戳
 * @param state [in] 系统状态向量（质心状态、关节状态等）
 * @param input [in] 系统输入向量（包含各接触点的接触力）
 * @param preComp [in] 预计算数据（当前未使用）
 * @return 1维向量，表示摩擦力锥约束值（≥0表示满足约束，<0表示打滑）
 * @note 约束公式：μ*(Fz + Fg) - √(Fx² + Fy² + ε) ≥ 0，其中ε为正则化项避免除零
 */
vector_t FrictionConeConstraint::getValue(scalar_t time, const vector_t &state, const vector_t &input,
                                          const PreComputation &preComp) const {
    // 从输入向量中提取指定接触点在世界坐标系下的接触力（Fx, Fy, Fz）
    const auto forcesInWorldFrame = centroidal_model::getContactForces(input, contactPointIndex_, info_);
    // 将世界坐标系下的接触力转换到地形坐标系，terrain z 轴与估计地形法向对齐。
    const matrix3_t terrain_R_world = getCurrentRotationWorldToTerrain();
    const vector3_t localForce = terrain_R_world * forcesInWorldFrame;
    // 计算摩擦力锥约束值并返回
    return coneConstraint(localForce);
}

/**
 * @brief 获取摩擦力锥约束的线性近似（一阶导数/雅克比矩阵）
 * @param time [in] 当前时间戳
 * @param state [in] 系统状态向量
 * @param input [in] 系统输入向量
 * @param preComp [in] 预计算数据（当前未使用）
 * @return 约束的线性近似结构（约束值、状态雅克比df/dx、输入雅克比df/du）
 * @note 状态雅克比为零（接触力仅与输入相关），输入雅克比仅对应目标接触点的力分量
 */
VectorFunctionLinearApproximation FrictionConeConstraint::getLinearApproximation(
    scalar_t time, const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const {
    // 提取世界坐标系下的接触力
    const vector3_t forcesInWorldFrame = centroidal_model::getContactForces(input, contactPointIndex_, info_);
    // 转换到地形坐标系
    const matrix3_t terrain_R_world = getCurrentRotationWorldToTerrain();
    const vector3_t localForce = terrain_R_world * forcesInWorldFrame;

    // 计算局部力的导数（dF_local/du）
    const auto localForceDerivatives = computeLocalForceDerivatives(terrain_R_world);
    // 计算摩擦力锥约束在局部力下的一阶/二阶导数
    const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
    // 链式法则：计算约束对输入的导数（dCone/du = dCone/dF_local * dF_local/du）
    const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

    // 初始化线性近似结构
    VectorFunctionLinearApproximation linearApproximation;
    // 设置约束当前值
    linearApproximation.f = coneConstraint(localForce);
    // 状态雅克比为零（接触力与状态无关）
    linearApproximation.dfdx = matrix_t::Zero(1, state.size());
    // 构建输入雅克比矩阵（仅目标接触点的3个力分量有非零值）
    linearApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
    
    return linearApproximation;
}

/**
 * @brief 获取摩擦力锥约束的二次近似（二阶导数/海森矩阵）
 * @param time [in] 当前时间戳
 * @param state [in] 系统状态向量
 * @param input [in] 系统输入向量
 * @param preComp [in] 预计算数据（当前未使用）
 * @return 约束的二次近似结构（约束值、一阶导数、二阶导数）
 * @note 二次近似包含海森矩阵（dfduu, dfdxx, dfdux），用于二次规划(QP)求解
 */
VectorFunctionQuadraticApproximation FrictionConeConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const {
    // 提取世界坐标系下的接触力
    const vector3_t forcesInWorldFrame = centroidal_model::getContactForces(input, contactPointIndex_, info_);
    // 转换到地形坐标系
    const matrix3_t terrain_R_world = getCurrentRotationWorldToTerrain();
    const vector3_t localForce = terrain_R_world * forcesInWorldFrame;

    // 计算局部力的导数
    const auto localForceDerivatives = computeLocalForceDerivatives(terrain_R_world);
    // 计算摩擦力锥约束在局部力下的一阶/二阶导数
    const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
    // 链式法则：计算约束对输入的一阶/二阶导数
    const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

    // 初始化二次近似结构
    VectorFunctionQuadraticApproximation quadraticApproximation;
    // 设置约束当前值
    quadraticApproximation.f = coneConstraint(localForce);
    // 状态雅克比为零
    quadraticApproximation.dfdx = matrix_t::Zero(1, state.size());
    // 输入雅克比矩阵
    quadraticApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
    // 状态二阶导数（海森矩阵）：零矩阵 + 对角偏移（保证正定性）
    quadraticApproximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(state.size(), coneDerivatives));
    // 输入二阶导数（海森矩阵）：仅目标接触点的3x3块有值 + 对角偏移
    quadraticApproximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(input.size(), coneDerivatives));
    // 状态-输入交叉二阶导数：零矩阵
    quadraticApproximation.dfdux.emplace_back(matrix_t::Zero(input.size(), state.size()));
    
    return quadraticApproximation;
}

/**
 * @brief 计算局部力（地形坐标系）对输入的导数
 * @param forcesInWorldFrame [in] 世界坐标系下的接触力（当前未使用）
 * @return 局部力导数结构体（dF_du：3x3矩阵）
 * @note 因t_R_w为单位矩阵，dF_local/du = t_R_w = 单位矩阵
 */
FrictionConeConstraint::LocalForceDerivatives FrictionConeConstraint::computeLocalForceDerivatives(
    const matrix3_t &terrain_R_world) const {
    LocalForceDerivatives localForceDerivatives{};
    // 输入中的接触力为世界系，局部力对世界系接触力的导数就是 world -> terrain 旋转。
    localForceDerivatives.dF_du = terrain_R_world;
    return localForceDerivatives;
}

/**
 * @brief 计算摩擦力锥约束对局部力的一阶/二阶导数
 * @param localForces [in] 地形坐标系下的接触力（Fx, Fy, Fz）
 * @return 摩擦力锥局部导数结构体（一阶导数dCone_dF，二阶导数d2Cone_dF2）
 * @note 数学推导核心：
 *       约束函数 C(F) = μ*(Fz+Fg) - √(Fx²+Fy²+ε)
 *       一阶导数：dC/dFx = -Fx/√(Fx²+Fy²+ε), dC/dFy = -Fy/√(...), dC/dFz = μ
 *       二阶导数：通过链式法则和商数法则推导，避免除零
 */
FrictionConeConstraint::ConeLocalDerivatives FrictionConeConstraint::computeConeLocalDerivatives(
    const vector3_t &localForces) const {
    // 计算Fx², Fy²
    const auto F_x_square = localForces.x() * localForces.x();
    const auto F_y_square = localForces.y() * localForces.y();
    // 切向力平方和 + 正则化项（避免√0和除零）
    const auto F_tangent_square = F_x_square + F_y_square + config_.regularization;
    // 切向力的范数（√(Fx²+Fy²+ε)）
    const auto F_tangent_norm = sqrt(F_tangent_square);
    // 切向力平方的3/2次方（用于二阶导数计算）
    const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square; // = F_tangent_square ^ (3/2)

    // 初始化导数结构体
    ConeLocalDerivatives coneDerivatives{};
    
    // ---------------- 一阶导数（dCone/dF）----------------
    coneDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm;  // dC/dFx
    coneDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm;  // dC/dFy
    coneDerivatives.dCone_dF(2) = config_.frictionCoefficient;        // dC/dFz = μ

    // ---------------- 二阶导数（d2Cone/dF2）----------------
    coneDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config_.regularization) / F_tangent_square_pow32; // d²C/dFx²
    coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;       // d²C/dFxFy
    coneDerivatives.d2Cone_dF2(0, 2) = 0.0;                                                               // d²C/dFxFz
    coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1);                                   // d²C/dFyFx（对称）
    coneDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config_.regularization) / F_tangent_square_pow32; // d²C/dFy²
    coneDerivatives.d2Cone_dF2(1, 2) = 0.0;                                                               // d²C/dFyFz
    coneDerivatives.d2Cone_dF2(2, 0) = 0.0;                                                               // d²C/dFzFx
    coneDerivatives.d2Cone_dF2(2, 1) = 0.0;                                                               // d²C/dFzFy
    coneDerivatives.d2Cone_dF2(2, 2) = 0.0;                                                               // d²C/dFz²

    return coneDerivatives;
}

/**
 * @brief 计算摩擦力锥约束的核心公式
 * @param localForces [in] 地形坐标系下的接触力（Fx, Fy, Fz）
 * @return 1维向量，表示约束值（≥0表示满足约束）
 * @note 约束公式：C = μ*(Fz + Fg) - √(Fx² + Fy² + ε)
 *       - μ：摩擦系数
 *       - Fg：抓地力偏移（增强约束鲁棒性）
 *       - ε：正则化项（避免平方根为零）
 */
vector_t FrictionConeConstraint::coneConstraint(const vector3_t &localForces) const {
    // 切向力平方和 + 正则化项
    const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization;
    // 切向力范数
    const auto F_tangent_norm = sqrt(F_tangent_square);
    // 计算摩擦力锥约束值
    const scalar_t coneConstraint = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm;
    // 封装为1维向量并返回
    return (vector_t(1) << coneConstraint).finished();
}

/**
 * @brief 链式法则计算摩擦力锥约束对输入的导数
 * @param coneLocalDerivatives [in] 约束对局部力的导数
 * @param localForceDerivatives [in] 局部力对输入的导数
 * @return 约束对输入的一阶/二阶导数结构体
 * @note 一阶导数：dC/du = (dC/dF)^T * dF/du
 *       二阶导数：d²C/du² = (dF/du)^T * (d²C/dF²) * dF/du
 */
FrictionConeConstraint::ConeDerivatives FrictionConeConstraint::computeConeConstraintDerivatives(
    const ConeLocalDerivatives &coneLocalDerivatives, const LocalForceDerivatives &localForceDerivatives) const {
    ConeDerivatives coneDerivatives;
    
    // ---------------- 一阶导数（dCone/du）----------------
    // dC/du = (dC/dF)^T * dF/du （1x3 * 3x3 = 1x3）
    coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

    // ---------------- 二阶导数（d2Cone/du2）----------------
    // d²C/du² = (dF/du)^T * (d²C/dF²) * dF/du （3x3 * 3x3 * 3x3 = 3x3）
    coneDerivatives.d2Cone_du2.noalias() =
            localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;

    return coneDerivatives;
}

/**
 * @brief 构建摩擦力锥约束的输入雅克比矩阵（dfdu）
 * @param inputDim [in] 输入向量维度（所有接触点的力 + 关节力矩等）
 * @param coneDerivatives [in] 约束对输入的导数
 * @return 1×inputDim的雅克比矩阵，仅目标接触点的3个力分量有非零值
 * @note 输入向量中接触力按索引存储：contactPointIndex×3 ~ contactPointIndex×3+2 对应Fx/Fy/Fz
 */
matrix_t FrictionConeConstraint::frictionConeInputDerivative(size_t inputDim,
                                                             const ConeDerivatives &coneDerivatives) const {
    // 初始化1×inputDim的零矩阵
    matrix_t dhdu = matrix_t::Zero(1, inputDim);
    // 将1x3的dCone_du填充到目标接触点对应的列位置
    dhdu.block<1, 3>(0, 3 * contactPointIndex_) = coneDerivatives.dCone_du;
    return dhdu;
}

/**
 * @brief 构建摩擦力锥约束的输入海森矩阵（dfduu）
 * @param inputDim [in] 输入向量维度
 * @param coneDerivatives [in] 约束对输入的二阶导数
 * @return inputDim×inputDim的海森矩阵，仅目标接触点的3x3块有值
 * @note 添加对角偏移（hessianDiagonalShift）保证矩阵正定性，避免QP求解失败
 */
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeInput(size_t inputDim,
                                                                   const ConeDerivatives &coneDerivatives) const {
    // 初始化inputDim×inputDim的零矩阵
    matrix_t ddhdudu = matrix_t::Zero(inputDim, inputDim);
    // 将3x3的d2Cone_du2填充到目标接触点对应的位置
    ddhdudu.block<3, 3>(3 * contactPointIndex_, 3 * contactPointIndex_) = coneDerivatives.d2Cone_du2;
    // 对角元素减去偏移值（保证海森矩阵正定）
    ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
    return ddhdudu;
}

/**
 * @brief 构建摩擦力锥约束的状态海森矩阵（dfdxx）
 * @param stateDim [in] 状态向量维度
 * @param coneDerivatives [in] 约束对输入的导数（当前未使用）
 * @return stateDim×stateDim的海森矩阵（零矩阵 + 对角偏移）
 * @note 接触力与状态无关，故状态海森矩阵为零，仅添加对角偏移保证正定性
 */
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeState(size_t stateDim,
                                                                   const ConeDerivatives &coneDerivatives) const {
    // 初始化stateDim×stateDim的零矩阵
    matrix_t ddhdxdx = matrix_t::Zero(stateDim, stateDim);
    // 对角元素减去偏移值（保证海森矩阵正定）
    ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
    return ddhdxdx;
}

} // namespace ocs2::legged_robot

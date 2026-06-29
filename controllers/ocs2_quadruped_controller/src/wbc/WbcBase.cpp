//
// Created by qiayuan on 2022/7/1.
//

#include "ocs2_quadruped_controller/wbc/WbcBase.h"

#include "ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h"

#include <algorithm>
#include <utility>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace ocs2::legged_robot
{
    /**
     * @brief WbcBase类构造函数
     * @param pinocchioInterface: Pinocchio机器人模型接口，包含机器人的运动学/动力学模型
     * @param info: 质心模型信息，包含机器人自由度、接触点数量等关键参数
     * @param eeKinematics: 末端执行器运动学计算类
     * 
     * 初始化WBC（Whole-Body Control）的基础参数，包括测量和期望的Pinocchio接口、
     * 末端执行器运动学、状态/输入映射关系，以及调试信息打印
     */
    WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics& eeKinematics,
                     const SwitchedModelReferenceManager* referenceManagerPtr)
        : pinocchio_interface_measured_(pinocchioInterface),  // 测量状态的Pinocchio接口
          pinocchio_interface_desired_(pinocchioInterface),   // 期望状态的Pinocchio接口
          info_(std::move(info)),                             // 质心模型信息（移动语义优化）
          ee_kinematics_(eeKinematics.clone()),               // 末端执行器运动学（克隆避免外部修改）
          mapping_(info_),                                    // 状态/输入映射器
          referenceManagerPtr_(referenceManagerPtr),
          input_last_(vector_t::Zero(info_.inputDim))         // 上一时刻输入（初始化零向量）
    { 
        // 调试打印：输出关键参数，便于排查维度不匹配等问题
        std::cerr << "[DEBUG] WbcBase::Constructor - info_.actuatedDofNum = " << info_.actuatedDofNum << std::endl;
        std::cerr << "[DEBUG] WbcBase::Constructor - info_.numThreeDofContacts = " << info_.numThreeDofContacts << std::endl;
        std::cerr << "[DEBUG] WbcBase::Constructor - info_.generalizedCoordinatesNum = " << info_.generalizedCoordinatesNum << std::endl;
        
        // 决策变量总数 = 广义坐标数 + 3*接触点数量（每个接触点3维力） + 驱动自由度
        num_decision_vars_ = info_.generalizedCoordinatesNum  + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
        
        // 初始化测量的广义坐标和广义速度向量
        q_measured_ = vector_t(info_.generalizedCoordinatesNum );
        v_measured_ = vector_t(info_.generalizedCoordinatesNum );
    }

    /**
     * @brief WBC主更新函数
     * @param stateDesired: 期望状态向量（质心状态+关节状态）
     * @param inputDesired: 期望输入向量（质心加速度+关节加速度/力矩）
     * @param rbdStateMeasured: 测量的刚体动力学状态（位置+速度）
     * @param mode: 接触模式（对应不同的支撑腿组合）
     * @param period: 控制周期（未使用）
     * @return vector_t: 预留返回值（实际由子类实现具体控制输出）
     * 
     * 核心流程：
     * 1. 将接触模式转换为支撑腿标志位
     * 2. 统计当前支撑腿数量
     * 3. 更新测量状态（运动学/动力学计算）
     * 4. 更新期望状态（运动学/动力学计算）
     */
    vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired,
                             const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t /*period*/)
    {
        // 将模式编号转换为支撑腿标志位（true表示支撑，false表示摆动）
        contact_flag_ = modeNumber2StanceLeg(mode);
        num_contacts_ = 0;
        
        // 统计当前支撑腿数量
        for (const bool flag : contact_flag_)
        {
            if (flag)
            {
                num_contacts_++;
            }
        }

        // 更新测量状态的运动学/动力学数据
        updateMeasured(rbdStateMeasured);
        // 更新期望状态的运动学/动力学数据
        updateDesired(stateDesired, inputDesired);

                // 控制打印频率：每隔 100 次 update 才打印一次
        static int print_counter = 0;
        const int print_interval = 100;

        // if (print_counter % print_interval == 0) {
        //     std::cerr << "\n==============================================================" << std::endl;
        //     std::cerr << "stateDesired 维度: " << stateDesired.size() << "\n" << stateDesired.transpose() << std::endl;
        //     std::cerr << "inputDesired 维度: " << inputDesired.size() << "\n" << inputDesired.transpose() << std::endl;
        //     std::cerr << "rbdStateMeasured 维度: " << rbdStateMeasured.size() << "\n" << rbdStateMeasured.transpose() << std::endl;
        //     std::cerr << "mode: " << mode << std::endl;
        //     std::cerr << "==============================================================" << std::endl;
        // }
        // print_counter++;


        return {};  // 基类仅做基础计算，具体控制输出由子类实现
    }
    

    /**
     * @brief 更新测量状态的运动学/动力学数据
     * @param rbdStateMeasured: 原始测量的刚体动力学状态（包含位置和速度）
     * 
     * 核心功能：
     * 1. 解析原始测量数据到广义坐标q和广义速度v
     * 2. 计算机器人正运动学、雅克比矩阵、质量矩阵、非线性项等动力学参数
     * 3. 计算接触点雅克比矩阵及其时间导数（用于约束构建）
     */
    void WbcBase::updateMeasured(const vector_t& rbdStateMeasured)
    {
        // ===================== 步骤1：解析广义坐标q_measured_ =====================
        // 广义坐标组成：[基座位置(3) | 基座姿态(欧拉角ZYX,3) | 关节角度(actuatedDofNum)]
        q_measured_.head<3>() = rbdStateMeasured.segment<3>(3);          // 基座位置 (x,y,z)
        q_measured_.segment<3>(3) = rbdStateMeasured.head<3>();          // 基座姿态（欧拉角ZYX）
        q_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);  // 关节角度

        // ===================== 步骤2：解析广义速度v_measured_ =====================
        // 广义速度组成：[基座线速度(3) | 基座角速度(欧拉角导数,3) | 关节速度(actuatedDofNum)]
        v_measured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum  + 3);  // 基座线速度
        // 将全局角速度转换为欧拉角ZYX的导数（姿态率转换）
        v_measured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            q_measured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum ));
        // 关节速度
        v_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(
            info_.generalizedCoordinatesNum  + 6, info_.actuatedDofNum);

        // ===================== 步骤3：计算动力学参数 =====================
        const auto& model = pinocchio_interface_measured_.getModel();  // 机器人模型
        auto& data = pinocchio_interface_measured_.getData();          // 机器人数据（存储计算结果）

        // 正运动学计算（更新关节/连杆位置和速度）
        forwardKinematics(model, data, q_measured_, v_measured_);
        // 计算关节雅克比矩阵
        computeJointJacobians(model, data);
        // 更新所有Frame的位姿
        updateFramePlacements(model, data);
        // CRBA算法计算质量矩阵（稀疏形式）
        crba(model, data, q_measured_);
        // 将质量矩阵转换为对称矩阵（CRBA输出上三角矩阵）
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        // 计算非线性项（科氏力、离心力、重力）
        nonLinearEffects(model, data, q_measured_, v_measured_);

        // ===================== 步骤4：计算接触点雅克比矩阵J =====================
        // j_维度：3*接触点数量 × 广义坐标数（每个接触点3维线速度雅克比）
        j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum );
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum );
            // 获取Frame雅克比矩阵（LOCAL_WORLD_ALIGNED：局部坐标系与世界坐标系对齐）
            getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED,
                             jac);
            // 只保留线速度部分（前3行）
            j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum ) = jac.template topRows<3>();
        }

        // ===================== 步骤5：计算雅克比矩阵时间导数dJ =====================
        // 计算雅克比矩阵的时间导数（用于加速度约束）
        computeJointJacobiansTimeVariation(model, data, q_measured_, v_measured_);
        dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum );
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum );
            // 获取Frame雅克比矩阵的时间导数
            getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                          pinocchio::LOCAL_WORLD_ALIGNED, jac);
            // 只保留线速度部分的时间导数
            dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum ) = jac.template topRows<3>();
        }
    }

    /**
     * @brief 更新期望状态的运动学/动力学数据
     * @param stateDesired: 期望状态向量
     * @param inputDesired: 期望输入向量
     * 
     * 计算期望状态下的机器人正运动学、雅克比矩阵、质心动力学等参数，
     * 为后续的控制任务（如基座加速度跟踪、摆动腿跟踪）提供参考
     */
    void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired)
    {
        const auto& model = pinocchio_interface_desired_.getModel();
        auto& data = pinocchio_interface_desired_.getData();

        // 设置映射器的Pinocchio接口（用于状态/输入转换）
        mapping_.setPinocchioInterface(pinocchio_interface_desired_);
        // 从期望状态中提取Pinocchio关节位置
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
        
        // 计算期望位置下的正运动学
        forwardKinematics(model, data, qDesired);
        // 计算期望位置下的关节雅克比矩阵
        computeJointJacobians(model, data, qDesired);
        // 更新期望位置下的Frame位姿
        updateFramePlacements(model, data);
        // 更新质心动力学参数
        updateCentroidalDynamics(pinocchio_interface_desired_, info_, qDesired);
        
        // 从期望状态和输入中提取Pinocchio关节速度
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
        // 计算期望位置+速度下的正运动学
        forwardKinematics(model, data, qDesired, vDesired);
    }

    /**
     * @brief 构建浮基动力学方程任务（核心动力学约束）
     * @return Task: 任务约束（A*x = b）
     * 
     * 浮基机器人动力学方程：M*qdd + NLE = J^T*F + S^T*tau
     * 转换为约束形式：M*qdd - J^T*F - S^T*tau = -NLE
     * 其中：
     * - M: 质量矩阵
     * - qdd: 广义加速度（决策变量）
     * - NLE: 非线性项（科氏力+离心力+重力）
     * - J^T*F: 接触力的广义力
     * - S^T*tau: 关节力矩的广义力（S为选择矩阵）
     */
    Task WbcBase::formulateFloatingBaseEomTask()
    {
        const auto& data = pinocchio_interface_measured_.getData();

        // 构建选择矩阵S：仅选择驱动关节部分（维度：驱动自由度 × 广义坐标数）
        matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum );
        s.block(0, 0, info_.actuatedDofNum, 6).setZero();          // 基座部分置零
        s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();  // 关节部分单位矩阵

        // 构建约束矩阵A：[M | -J^T | -S^T]
        matrix_t a = (matrix_t(info_.generalizedCoordinatesNum , num_decision_vars_) << data.M, -j_.transpose(), -s.
            transpose()).finished();
        // 构建约束向量b：-NLE
        vector_t b = -data.nle;

        // 返回任务约束（无不等式约束）
        return {a, b, matrix_t(), vector_t()};
    }

    /**
     * @brief 构建力矩限制任务（Torque Limits Task）
     * @return 构建完成的力矩限制任务对象，包含约束矩阵和约束向量
     * @note 核心功能：为WBC（加权伪逆控制）添加关节力矩上下限约束，
     *       约束形式为 -τ_max ≤ τ ≤ τ_max，转换为矩阵形式的不等式约束
     */
    Task WbcBase::formulateTorqueLimitsTask()
    {
        // ========================= 初始化约束矩阵d =========================
        // 约束矩阵d维度：[2*驱动自由度数量 × 决策变量总数]
        // 2*驱动自由度：因为每个关节需要上下两个限制（τ ≤ τ_max 和 -τ ≤ τ_max）
        // num_decision_vars_：WBC优化问题的总决策变量数（包含状态、输入等）
        matrix_t d(2 * info_.actuatedDofNum, num_decision_vars_);
        d.setZero(); // 初始化约束矩阵为全零

        // 构建单位矩阵：维度[驱动自由度数量 × 驱动自由度数量]，用于赋值约束矩阵的对角块
        matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);

        // 第一部分约束：τ ≤ τ_max （力矩上限）
        // 赋值区域：d矩阵前info_.actuatedDofNum行，对应力矩决策变量的列范围
        // 列起始位置：广义坐标数量 + 3*3DOF接触点数量（跳过状态变量和接触力变量，定位到力矩变量列）
        // 赋值内容：单位矩阵i，代表对力矩变量的直接约束
        d.block(0, info_.generalizedCoordinatesNum  + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
                info_.actuatedDofNum) = i;

        // 第二部分约束：-τ ≤ τ_max （力矩下限，等价于 τ ≥ -τ_max）
        // 赋值区域：d矩阵后info_.actuatedDofNum行，对应力矩决策变量的列范围
        // 赋值内容：-单位矩阵-i，将下限约束转换为标准的不等式形式 Ax ≤ b
        d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum  + 3 * info_.numThreeDofContacts,
                info_.actuatedDofNum,
                info_.actuatedDofNum) = -i;

        // ========================= 初始化约束向量f =========================
        // 约束向量f维度：[2*驱动自由度数量]，对应每个力矩约束的上限值
        vector_t f(2 * info_.actuatedDofNum);

        // 循环赋值力矩限制值到约束向量f
        // 2 * info_.actuatedDofNum / 3：按3个关节为一组遍历（适配多足机器人关节分组特性）
        for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l)
        {
            // 按3维段赋值：将torque_limits_（关节力矩上限值）依次填入f的每个3维段
            // 前info_.actuatedDofNum个元素：对应τ ≤ τ_max 的上限值τ_max
            // 后info_.actuatedDofNum个元素：对应-τ ≤ τ_max 的上限值τ_max（即τ ≥ -τ_max）
            f.segment<3>(3 * l) = torque_limits_;
        }

        // 返回力矩限制任务：Task对象包含（等式约束矩阵, 等式约束向量, 不等式约束矩阵d, 不等式约束向量f）
        // 此处等式约束为空（matrix_t()/vector_t()），仅包含不等式约束d和f
        return {matrix_t(), vector_t(), d, f};
    }



    Task WbcBase::formulateNoContactMotionTask()
    {
        matrix_t a(3 * num_contacts_, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; i++)
        {
            if (contact_flag_[i])
            {
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(
                    3 * i, 0, 3, info_.generalizedCoordinatesNum);
                b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * v_measured_;
                j++;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateWheelRollingMotionTask()
    {
        constexpr size_t kConstraintsPerContact = 2;

        matrix_t a(kConstraintsPerContact * num_contacts_, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();

        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (contact_flag_[i])
            {
                const matrix_t Av = getWheelRollingProjectionMatrix(i);
                const auto Jlin = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
                const auto dJlin = dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);

                a.block(kConstraintsPerContact * j, 0, kConstraintsPerContact, info_.generalizedCoordinatesNum) = Av * Jlin;
                b.segment(kConstraintsPerContact * j, kConstraintsPerContact) = -Av * dJlin * v_measured_;
                ++j;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    matrix_t WbcBase::getWheelRollingProjectionMatrix(size_t contactPointIndex) const
    {
        const vector3_t terrainNormal = getTerrainNormalWorldForWbcOrUnitZ();

        vector3_t wheelAxisWorld = vector3_t::UnitY();
        const auto& model = pinocchio_interface_measured_.getModel();
        const auto& data = pinocchio_interface_measured_.getData();
        if (contactPointIndex < info_.endEffectorFrameIndices.size())
        {
            const auto frameId = info_.endEffectorFrameIndices[contactPointIndex];
            if (frameId < model.frames.size() && frameId < data.oMf.size())
            {
                wheelAxisWorld = data.oMf[frameId].rotation() * vector3_t::UnitY();
            }
        }
        if (!wheelAxisWorld.allFinite() || wheelAxisWorld.norm() < 1e-6)
        {
            wheelAxisWorld = vector3_t::UnitY();
        }
        else
        {
            wheelAxisWorld.normalize();
        }

        const vector3_t lateralWorld = getTangentProjectionOrFallback(wheelAxisWorld, terrainNormal);

        matrix_t Av(2, 3);
        Av.row(0) = lateralWorld.transpose();
        Av.row(1) = terrainNormal.transpose();
        return Av;
    }

    vector3_t WbcBase::getWheelRollingDirectionWorld(size_t contactPointIndex) const
    {
        const vector3_t terrainNormal = getTerrainNormalWorldForWbcOrUnitZ();

        vector3_t wheelAxisWorld = vector3_t::UnitY();
        const auto& model = pinocchio_interface_measured_.getModel();
        const auto& data = pinocchio_interface_measured_.getData();
        if (contactPointIndex < info_.endEffectorFrameIndices.size())
        {
            const auto frameId = info_.endEffectorFrameIndices[contactPointIndex];
            if (frameId < model.frames.size() && frameId < data.oMf.size())
            {
                wheelAxisWorld = data.oMf[frameId].rotation() * vector3_t::UnitY();
            }
        }
        if (!wheelAxisWorld.allFinite() || wheelAxisWorld.norm() < 1e-6)
        {
            wheelAxisWorld = vector3_t::UnitY();
        }
        else
        {
            wheelAxisWorld.normalize();
        }

        const vector3_t lateralWorld = getTangentProjectionOrFallback(wheelAxisWorld, terrainNormal);
        vector3_t rollingWorld = lateralWorld.cross(terrainNormal);
        if (!rollingWorld.allFinite() || rollingWorld.norm() < 1e-6)
        {
            rollingWorld = vector3_t::UnitX() - vector3_t::UnitX().dot(terrainNormal) * terrainNormal;
        }
        if (!rollingWorld.allFinite() || rollingWorld.norm() < 1e-6)
        {
            rollingWorld = terrainNormal.unitOrthogonal();
        }
        return rollingWorld.normalized();
    }

    vector3_t WbcBase::getTerrainNormalWorldForWbcOrUnitZ() const
    {
        vector3_t terrainNormal = vector3_t::UnitZ();
        if (referenceManagerPtr_ != nullptr)
        {
            terrainNormal = referenceManagerPtr_->getTerrainNormalWorldOrUnitZ();
        }
        if (!terrainNormal.allFinite() || terrainNormal.norm() < 1e-6)
        {
            terrainNormal = vector3_t::UnitZ();
        }
        else
        {
            terrainNormal.normalize();
        }
        if (terrainNormal.z() < 0.0)
        {
            terrainNormal = -terrainNormal;
        }
        constexpr scalar_t kMinNormalZForWbc = 0.70;
        if (terrainNormal.z() < kMinNormalZForWbc)
        {
            terrainNormal = vector3_t::UnitZ();
        }
        return terrainNormal;
    }

    matrix3_t WbcBase::getRotationWorldToTerrain(const vector3_t& surfaceNormalInWorld)
    {
        vector3_t terrainZ = surfaceNormalInWorld;
        if (!terrainZ.allFinite() || terrainZ.norm() < 1e-6)
        {
            terrainZ = vector3_t::UnitZ();
        }
        else
        {
            terrainZ.normalize();
        }
        if (terrainZ.z() < 0.0)
        {
            terrainZ = -terrainZ;
        }

        vector3_t terrainX = vector3_t::UnitX() - vector3_t::UnitX().dot(terrainZ) * terrainZ;
        if (!terrainX.allFinite() || terrainX.norm() < 1e-6)
        {
            terrainX = vector3_t::UnitY() - vector3_t::UnitY().dot(terrainZ) * terrainZ;
        }
        if (!terrainX.allFinite() || terrainX.norm() < 1e-6)
        {
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

    vector3_t WbcBase::getTangentProjectionOrFallback(const vector3_t& direction, const vector3_t& normal)
    {
        vector3_t lateral = direction - direction.dot(normal) * normal;
        if (lateral.allFinite() && lateral.norm() > 1e-6)
        {
            return lateral.normalized();
        }

        lateral = vector3_t::UnitY() - vector3_t::UnitY().dot(normal) * normal;
        if (lateral.allFinite() && lateral.norm() > 1e-6)
        {
            return lateral.normalized();
        }

        lateral = vector3_t::UnitX() - vector3_t::UnitX().dot(normal) * normal;
        if (lateral.allFinite() && lateral.norm() > 1e-6)
        {
            return lateral.normalized();
        }

        return normal.unitOrthogonal().normalized();
    }

    // /**
    //  * @brief 构建非接触运动约束任务（轮式接触专用）
    //  * @return Task: 任务约束（A*x = b）
    //  * 
    //  * 约束逻辑：
    //  * 对于轮式接触，仅约束侧向(y)和法向(z)加速度为0，放开滚动方向(x)
    //  * 数学表达：C*(J*qdd + dJ*qdot) = 0
    //  * 其中C为投影矩阵，只保留y和z分量
    //  */
    // Task WbcBase::formulateNoContactMotionTask()
    // {
    //     // 每个接触点的约束数量：2（y和z方向）
    //     constexpr int kConstraintsPerContact = 2;

    //     // 初始化约束矩阵和向量
    //     matrix_t a(kConstraintsPerContact * num_contacts_, num_decision_vars_);
    //     vector_t b(a.rows());
    //     a.setZero();
    //     b.setZero();

    //     const auto& model = pinocchio_interface_measured_.getModel();
    //     const auto& data  = pinocchio_interface_measured_.getData();

    //     // 投影矩阵S：只保留y和z分量
    //     Eigen::Matrix<scalar_t, 2, 3> S;
    //     S << scalar_t(0), scalar_t(1), scalar_t(0),   // 侧向（y轴）
    //         scalar_t(0), scalar_t(0), scalar_t(1);   // 法向（z轴）

    //     size_t jContact = 0;  // 支撑腿计数器
    //     for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    //     {
    //         if (!contact_flag_[i]) continue;  // 跳过非支撑腿

    //         const auto frameId = info_.endEffectorFrameIndices[i];

    //         // 获取Frame的旋转矩阵：R_wf = 世界坐标系到Frame坐标系的旋转
    //         const Eigen::Matrix<scalar_t,3,3> R_wf = data.oMf[frameId].rotation().template cast<scalar_t>();

    //         // 构建投影矩阵C：将世界坐标系的速度投影到Frame的y、z轴
    //         // C = S * R_wf^T （先转换到Frame坐标系，再保留y、z分量）
    //         const Eigen::Matrix<scalar_t,2,3> C = S * R_wf.transpose();

    //         // 获取该接触点的线速度雅克比矩阵和其时间导数
    //         const matrix_t Jlin  = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
    //         const matrix_t dJlin = dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);

    //         // 构建约束：C*(J*qdd + dJ*qdot) = 0 --> C*J*qdd = -C*dJ*qdot
    //         a.block(kConstraintsPerContact * jContact, 0, kConstraintsPerContact, info_.generalizedCoordinatesNum) = C * Jlin;
    //         b.segment(kConstraintsPerContact * jContact, kConstraintsPerContact) = - C * dJlin * v_measured_;

    //         ++jContact;
    //     }

    //     return {a, b, matrix_t(), vector_t()};
    // }





// /**
//  * @brief 构建非接触运动约束任务（轮式接触专用）
//  * @return Task: 任务约束（A*x = b）
//  * 
//  * 约束逻辑：
//  * 对于轮式接触，仅约束侧向(y)和法向(z)加速度为0，放开滚动方向(x)
//  * 数学表达：C*(J*qdd + dJ*qdot) = 0
//  * 其中C为投影矩阵，只保留y和z分量
//  */
// Task WbcBase::formulateNoContactMotionTask()
// {
//     // 每个接触点的约束数量：2（y和z方向）
//     constexpr int kConstraintsPerContact = 2;

//     // ========== 修复1：先统计实际支撑腿数（避免冗余约束行） ==========
//     size_t support_leg_count = 0;
//     for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
//         if (contact_flag_[i]) support_leg_count++;
//     }
//     std::cout << "[NoContactMotionTask] 总腿数: " << info_.numThreeDofContacts 
//               << " | 实际支撑腿数: " << support_leg_count << std::endl;

//     // 初始化约束矩阵/向量：用实际支撑腿数，而非总腿数
//     matrix_t a(kConstraintsPerContact * support_leg_count, num_decision_vars_);
//     vector_t b(a.rows());
//     a.setZero();
//     b.setZero();

//     const auto& model = pinocchio_interface_measured_.getModel();
//     const auto& data  = pinocchio_interface_measured_.getData();

//     // 投影矩阵S：Frame系下保留y/z分量（轮式约束核心）
//     Eigen::Matrix<scalar_t, 2, 3> S;
//     S << scalar_t(0), scalar_t(1), scalar_t(0),   // 轮系侧向（y轴）
//          scalar_t(0), scalar_t(0), scalar_t(1);   // 轮系法向（z轴）
//     std::cout << "[投影矩阵S] Frame系y/z保留矩阵:\n" << S << std::endl;

//     size_t jContact = 0;  // 支撑腿计数器
//     for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
//     {
//         if (!contact_flag_[i]) {
//             std::cout << "[ContactPoint " << i << "] 非支撑腿，跳过约束构建" << std::endl;
//             continue;  // 跳过非支撑腿
//         }

//         const auto frameId = info_.endEffectorFrameIndices[i];
//         std::string wheel_pos = (i == 0 || i == 1) ? "前轮" : "后轮";

//         // 获取Frame旋转矩阵：R_wf = 世界坐标系 → Frame坐标系的旋转
//         // 优化：逐元素转换，兼容自动微分标量
//         const auto& R_wf_raw = data.oMf[frameId].rotation();
//         Eigen::Matrix<scalar_t, 3, 3> R_wf;
//         R_wf << static_cast<scalar_t>(R_wf_raw(0,0)), static_cast<scalar_t>(R_wf_raw(0,1)), static_cast<scalar_t>(R_wf_raw(0,2)),
//                 static_cast<scalar_t>(R_wf_raw(1,0)), static_cast<scalar_t>(R_wf_raw(1,1)), static_cast<scalar_t>(R_wf_raw(1,2)),
//                 static_cast<scalar_t>(R_wf_raw(2,0)), static_cast<scalar_t>(R_wf_raw(2,1)), static_cast<scalar_t>(R_wf_raw(2,2));
        
//         // 打印旋转矩阵 + 验证正交性（坐标系旋转矩阵必须正交）
//         std::cout << "\n[ContactPoint " << i << "] " << wheel_pos 
//                   << " | 世界→Frame旋转矩阵R_wf:\n" << R_wf << std::endl;
//         Eigen::Matrix3d R_check = R_wf * R_wf.transpose();
//         std::cout << "[ContactPoint " << i << "] 旋转矩阵正交性验证（应接近单位矩阵）:\n" << R_check << std::endl;

//         // ========== 修正：坐标系转换方向（添加transpose()） ==========
//         Eigen::Matrix<scalar_t,2,3> C = S * R_wf.transpose(); // 关键：加transpose()
//                 // 新增：对C的每一行归一化（避免数值波动）
//         for (int row = 0; row < C.rows(); ++row) {
//             const scalar_t norm = C.row(row).norm();
//             if (norm > 1e-6) { // 避免除0
//                 C.row(row) /= norm;
//             }
//         }
//         std::cout << "[ContactPoint " << i << "] 最终投影矩阵C（2x3）:\n" << C << std::endl;

//         // 验证XYZ三个方向的约束权重（世界坐标系轴）
//         Eigen::Vector3<scalar_t> world_x(1, 0, 0); // 世界x轴
//         Eigen::Vector3<scalar_t> world_y(0, 1, 0); // 世界y轴
//         Eigen::Vector3<scalar_t> world_z(0, 0, 1); // 世界z轴
        
//         Eigen::Vector2<scalar_t> c_world_x = C * world_x; // x轴约束权重
//         Eigen::Vector2<scalar_t> c_world_y = C * world_y; // y轴约束权重
//         Eigen::Vector2<scalar_t> c_world_z = C * world_z; // z轴约束权重
        
//         std::cout << "[ContactPoint " << i << "] " << wheel_pos 
//                   << " | X轴约束权重: " << c_world_x.transpose() 
//                   << " | Y轴约束权重: " << c_world_y.transpose() 
//                   << " | Z轴约束权重: " << c_world_z.transpose() << std::endl;

//         // Z向约束告警（原逻辑保留）
//         if (fabs(c_world_z(1)) < 0.8) {
//             std::cerr << "[WARNING] 接触点" << i << " z向约束权重过低: " << c_world_z(1) << std::endl;
//         }

//         // 获取线速度雅克比+时间导数（索引验证通过，保留原逻辑）
//         const matrix_t Jlin  = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
//         const matrix_t dJlin = dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);

//         // 打印雅克比关键信息
//         std::cout << "[ContactPoint " << i << "] 线速度雅克比Jlin维度: " << Jlin.rows() << "x" << Jlin.cols() 
//                   << " | 非零元素数: " << Jlin.nonZeros() << std::endl;

//         // 维度校验（防御性编程）
//         assert(Jlin.rows() == 3 && Jlin.cols() == info_.generalizedCoordinatesNum);
//         assert(dJlin.rows() == 3 && dJlin.cols() == info_.generalizedCoordinatesNum);

//         // 后续约束构建逻辑不变，仅C的计算修正
//         a.block(kConstraintsPerContact * jContact, 0, kConstraintsPerContact, info_.generalizedCoordinatesNum) = C * Jlin;
//         b.segment(kConstraintsPerContact * jContact, kConstraintsPerContact) = - C * dJlin * v_measured_;

//         // 打印当前接触点的约束矩阵A和向量b片段
//         std::cout << "[ContactPoint " << i << "] 约束矩阵A片段（C*Jlin）:\n" 
//                   << a.block(kConstraintsPerContact * jContact, 0, kConstraintsPerContact, 10) // 只打印前10列避免刷屏
//                   << "\n[ContactPoint " << i << "] 约束向量b片段: " 
//                   << b.segment(kConstraintsPerContact * jContact, kConstraintsPerContact).transpose() << std::endl;

//         ++jContact;
//     }

//     // 最终校验：支撑腿计数与实际赋值数一致
//     assert(jContact == support_leg_count && "支撑腿计数与赋值数不匹配！");

//     // 打印最终约束矩阵整体信息
//     std::cout << "\n[NoContactMotionTask] 最终约束矩阵A维度: " << a.rows() << "x" << a.cols() 
//               << " | 约束向量b维度: " << b.rows() << std::endl;
//     std::cout << "[NoContactMotionTask] 约束矩阵A非零元素数: " << a.nonZeros() << std::endl;

//     return {a, b, matrix_t(), vector_t()};
// }





    /**
     * @brief 构建摩擦锥约束任务（不等式约束）
     * @return Task: 任务约束（A*x = b, D*x ≤ f）
     * 
     * 约束逻辑：
     * 1. 非支撑腿：接触力为0（等式约束）
     * 2. 支撑腿：接触力在摩擦锥内（不等式约束）
     * 摩擦锥采用金字塔近似：
     * - Fz ≥ 0（法向力向上）
     * - |Fx| ≤ μ*Fz, |Fy| ≤ μ*Fz（切向力不超过摩擦极限）
     */
    Task WbcBase::formulateFrictionConeTask()
    {
        // ===================== 等式约束：非支撑腿接触力为0 =====================
        matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        a.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (!contact_flag_[i])  // 非支撑腿
            {
                // 约束对应接触力分量为0
                a.block(3 * j++, info_.generalizedCoordinatesNum  + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
            }
        }
        vector_t b(a.rows());
        b.setZero();

        // ===================== 不等式约束：支撑腿摩擦锥 =====================
        // 摩擦金字塔矩阵（5行3列）：
        // [0  0 -1]  --> -Fz ≤ 0 (Fz ≥ 0)
        // [1  0 -μ]  --> Fx - μ*Fz ≤ 0
        // [-1 0 -μ]  --> -Fx - μ*Fz ≤ 0
        // [0  1 -μ]  --> Fy - μ*Fz ≤ 0
        // [0 -1 -μ]  --> -Fy - μ*Fz ≤ 0
        matrix_t frictionPyramic(5, 3);
        frictionPyramic << 0, 0, -1,
                1, 0, -friction_coeff_,
                -1, 0, -friction_coeff_,
                0, 1, -friction_coeff_,
                0, -1, -friction_coeff_;

        const matrix3_t terrain_R_world = getRotationWorldToTerrain(getTerrainNormalWorldForWbcOrUnitZ());

        // 约束矩阵D维度：5*支撑腿数 + 3*非支撑腿数 × 决策变量数
        matrix_t d(5 * num_contacts_ + 3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        d.setZero();
        j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (contact_flag_[i])  // 支撑腿：添加摩擦锥约束
            {
                d.block(5 * j++, info_.generalizedCoordinatesNum  + 3 * i, 5, 3) = frictionPyramic * terrain_R_world;
            }
        }
        vector_t f = Eigen::VectorXd::Zero(d.rows());  // 约束上限为0

        return {a, b, d, f};
    }

    /**
     * @brief 构建基座加速度跟踪任务（等式约束）
     * @param stateDesired: 期望状态
     * @param inputDesired: 期望输入
     * @param period: 控制周期
     * @return Task: 任务约束（A*x = b）
     * 
     * 基于质心动力学计算期望的基座加速度，构建约束：qdd_base = 期望基座加速度
     * 其中基座加速度由质心动量率、关节加速度等推导得到
     */
    Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
    {
        // 约束矩阵A：仅约束基座加速度（前6维广义加速度）
        matrix_t a(6, num_decision_vars_);
        a.setZero();
        a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

        // 计算关节加速度（由输入变化率推导）
        vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - input_last_, info_) / period;
        input_last_ = inputDesired;  // 更新上一时刻输入
        mapping_.setPinocchioInterface(pinocchio_interface_desired_);

        const auto& model = pinocchio_interface_desired_.getModel();
        auto& data = pinocchio_interface_desired_.getData();
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

        // ===================== 质心动力学计算 =====================
        // 获取质心动量矩阵A
        const auto& A = getCentroidalMomentumMatrix(pinocchio_interface_desired_);
        // 提取基座部分的质心动量矩阵Ab
        const Matrix6 Ab = A.template leftCols<6>();
        // 计算Ab的逆矩阵
        const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
        // 提取关节部分的质心动量矩阵Aj
        const auto Aj = A.rightCols(info_.actuatedDofNum);
        // 计算质心动量矩阵的时间导数ADot
        const auto ADot = dccrba(model, data, qDesired, vDesired);
        
        // 计算期望的质心动量率
        Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(
            pinocchio_interface_desired_, info_, inputDesired);
        // 补偿ADot*vDesired和Aj*jointAccel的影响
        centroidalMomentumRate.noalias() -= ADot * vDesired;
        centroidalMomentumRate.noalias() -= Aj * jointAccel;

        // 计算期望的基座加速度
        Vector6 b = AbInv * centroidalMomentumRate;

        return {a, b, matrix_t(), vector_t()};
    }

    /**
     * @brief 构建摆动腿跟踪任务（等式约束）
     * @return Task: 任务约束（A*x = b）
     * 
     * 采用PD控制律计算期望的摆动腿加速度：
     * accel = kp*(pos_des - pos_meas) + kd*(vel_des - vel_meas)
     * 约束逻辑：J*qdd + dJ*qdot = accel --> J*qdd = accel - dJ*qdot
     */
    Task WbcBase::formulateGroundWheelRollingTask(const vector_t& /*stateDesired*/, const vector_t& /*inputDesired*/, scalar_t /*period*/)
    {
        matrix_t a(num_contacts_, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();

        if (num_contacts_ == 0)
        {
            return {a, b, matrix_t(), vector_t()};
        }

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_measured_);
        const std::vector<vector3_t> velMeasured = ee_kinematics_->getVelocity(vector_t(), vector_t());

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_desired_);
        const std::vector<vector3_t> velDesired = ee_kinematics_->getVelocity(vector_t(), vector_t());

        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (contact_flag_[i])
            {
                if (i >= velMeasured.size() || i >= velDesired.size())
                {
                    ++j;
                    continue;
                }

                const vector3_t rollingWorld = getWheelRollingDirectionWorld(i);
                if (!rollingWorld.allFinite() || rollingWorld.norm() < 1e-6 ||
                    !velMeasured[i].allFinite() || !velDesired[i].allFinite())
                {
                    ++j;
                    continue;
                }

                const scalar_t vMeasuredRolling = rollingWorld.dot(velMeasured[i]);
                const scalar_t vDesiredRolling = rollingWorld.dot(velDesired[i]);
                scalar_t accelCommand = ground_wheel_rolling_kd_ * (vDesiredRolling - vMeasuredRolling);
                if (ground_wheel_rolling_max_accel_ > 0.0)
                {
                    accelCommand = std::clamp(accelCommand,
                                              -ground_wheel_rolling_max_accel_,
                                              ground_wheel_rolling_max_accel_);
                }

                const auto Jlin = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
                const auto dJlin = dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
                a.block(j, 0, 1, info_.generalizedCoordinatesNum) = rollingWorld.transpose() * Jlin;
                b(j) = accelCommand - rollingWorld.transpose() * dJlin * v_measured_;
                ++j;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateSwingLegTask()
    {
        // 获取测量状态下的末端执行器位置和速度
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_measured_);
        std::vector<vector3_t> posMeasured = ee_kinematics_->getPosition(vector_t());
        std::vector<vector3_t> velMeasured = ee_kinematics_->getVelocity(vector_t(), vector_t());
        
        // 获取期望状态下的末端执行器位置和速度
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_desired_);
        std::vector<vector3_t> posDesired = ee_kinematics_->getPosition(vector_t());
        std::vector<vector3_t> velDesired = ee_kinematics_->getVelocity(vector_t(), vector_t());

        // 初始化约束矩阵和向量
        matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();
        
        size_t j = 0;  // 摆动腿计数器
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (!contact_flag_[i])  // 仅处理摆动腿
            {
                // PD控制律计算期望加速度
                vector3_t accel = swing_kp_ * (posDesired[i] - posMeasured[i]) + swing_kd_ * (
                    velDesired[i] - velMeasured[i]);
                
                // 构建约束：J*qdd = accel - dJ*qdot
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum ) = j_.block(
                    3 * i, 0, 3, info_.generalizedCoordinatesNum );
                b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum ) * v_measured_;
                j++;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }




    //     Task WbcBase::formulateGroundLegTask()
    // {
    //     matrix_t a(3 * num_contacts_, num_decision_vars_);
    //     vector_t b(a.rows());
    //     a.setZero();
    //     b.setZero();

    //     size_t j = 0;
    //     for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    //     {
    //         if (contact_flag_[i]) {  // 仅处理地面腿
    //             // PD控制律计算车轮滚动方向期望加速度
    //             vector3_t accel = ground_kp_ * (wheel_pos_des[i] - wheel_pos_meas[i]) + ground_kd_ * (wheel_vel_des[i] - wheel_vel_meas[i]);
    //             // 构建约束：J_ground * qdd = accel - dJ_ground * v_measured_
    //             a.block(3*j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3*i, 0, 3, info_.generalizedCoordinatesNum);
    //             b.segment(3*j, 3) = accel - dj_.block(3*i, 0, 3, info_.generalizedCoordinatesNum) * v_measured_;
    //             j++;
    //         }
    //     }
    //     return {a, b, matrix_t(), vector_t()};
    // }


    /**
     * @brief 构建接触力跟踪任务（等式约束）
     * @param inputDesired: 期望输入（包含期望接触力）
     * @return Task: 任务约束（A*x = b）
     * 
     * 约束逻辑：将决策变量中的接触力分量约束为期望接触力
     */
    Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const
    {
        // 约束矩阵A：仅选择接触力分量
        matrix_t a(3 * info_.numThreeDofContacts, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();

        // 为每个接触点的3维力构建单位矩阵约束
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            a.block(3 * i, info_.generalizedCoordinatesNum  + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }
        // 约束向量b：期望接触力（从输入中提取）
        b = inputDesired.head(a.rows());

        return {a, b, matrix_t(), vector_t()};
    }

    /**
     * @brief 加载任务配置参数
     * @param taskFile: 配置文件路径
     * @param verbose: 是否打印加载信息
     * 
     * 从配置文件中加载：
     * 1. 力矩限位参数
     * 2. 摩擦锥系数
     * 3. 摆动腿PD控制参数
     */
    void WbcBase::loadTasksSetting(const std::string& taskFile, const bool verbose)
    {
        // 加载力矩限位参数
        torque_limits_ = vector_t(info_.actuatedDofNum/4);
        loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torque_limits_);
        if (verbose)
        {
            std::cerr << "\n #### Torque Limits Task:";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << "\n #### Hip_joint Thigh_joint Calf_joint: " << torque_limits_.transpose() << "\n";
            std::cerr << " #### =============================================================================\n";
        }

        // 加载摩擦锥和摆动腿参数
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        
        // 加载摩擦锥系数
        std::string prefix = "frictionConeTask.";
        if (verbose)
        {
            std::cerr << "\n #### Friction Cone Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, friction_coeff_, prefix + "frictionCoefficient", verbose);
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
        }

        // 加载摆动腿PD参数
        prefix = "swingLegTask.";
        if (verbose)
        {
            std::cerr << "\n #### Swing Leg Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, swing_kp_, prefix + "kp", verbose);
        loadData::loadPtreeValue(pt, swing_kd_, prefix + "kd", verbose);
    }
} // namespace legged

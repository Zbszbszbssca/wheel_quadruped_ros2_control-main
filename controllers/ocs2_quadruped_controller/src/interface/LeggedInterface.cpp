//
// Created by qiayuan on 2022/7/16.
//

#include "ocs2_quadruped_controller/interface/LeggedInterface.h"

#include <memory>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <pinocchio/fwd.hpp>  // pinocchio库的前置声明必须最先包含
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "ocs2_quadruped_controller/interface/LeggedRobotPreComputation.h"
#include "ocs2_quadruped_controller/interface/constraint/FrictionConeConstraint.h"
#include "ocs2_quadruped_controller/interface/constraint/LeggedSelfCollisionConstraint.h"
#include "ocs2_quadruped_controller/interface/constraint/NormalVelocityConstraintCppAd.h"
#include "ocs2_quadruped_controller/interface/constraint/WheelRollingVelocityConstraintCppAd.h"}providernameили Nope, malformed JSON due extra. Need valid. Use proper. We'll retry. Since failed? It didn't run likely. Need commentary tool call.}                        (json)իջOops no tool result because malformed in analysis? Actually previous message to tool invalid? Need send proper.} тру канал commentary to=functions.Edit  voud json _日本一级特黄大片.encode({
#include "ocs2_quadruped_controller/interface/constraint/ZeroForceConstraint.h"
#include "ocs2_quadruped_controller/interface/constraint/ZeroVelocityConstraintCppAd.h"
#include "ocs2_quadruped_controller/interface/cost/LeggedRobotQuadraticTrackingCost.h"
#include "ocs2_quadruped_controller/interface/initialization/LeggedRobotInitializer.h"

// Boost文件系统库，用于文件路径和存在性检查
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2::legged_robot
{
    /**
     * @brief LeggedInterface构造函数
     * @param task_file 任务配置文件路径（包含MPC/SQP/模型参数等）
     * @param urdf_file 机器人URDF模型文件路径
     * @param reference_file 参考轨迹/步态配置文件路径
     * @param use_hard_friction_cone_constraint 是否使用硬约束版本的摩擦锥约束
     * @note 主要完成文件存在性检查和基础配置加载
     */
    LeggedInterface::LeggedInterface(const std::string& task_file,
                                     const std::string& urdf_file,
                                     const std::string& reference_file,
                                     const bool use_hard_friction_cone_constraint)
        : use_hard_friction_cone_constraint_(use_hard_friction_cone_constraint)
    {
        // 检查任务配置文件是否存在
        if (const boost::filesystem::path task_file_path(task_file); exists(task_file_path))
        {
            std::cerr << "[LeggedInterface] Loading task file: " << task_file_path << std::endl;
        }
        else
        {
            throw std::invalid_argument("[LeggedInterface] Task file not found: " + task_file_path.string());
        }

        // 检查URDF模型文件是否存在
        if (const boost::filesystem::path urdf_file_path(urdf_file); exists(urdf_file_path))
        {
            std::cerr << "[LeggedInterface] Loading Pinocchio model from: " << urdf_file_path << std::endl;
        }
        else
        {
            throw std::invalid_argument("[LeggedInterface] URDF file not found: " + urdf_file_path.string());
        }

        // 检查参考轨迹/步态配置文件是否存在
        if (const boost::filesystem::path reference_file_path(reference_file); exists(reference_file_path))
        {
            std::cerr << "[LeggedInterface] Loading target command settings from: " << reference_file_path << std::endl;
        }
        else
        {
            throw std::invalid_argument(
                "[LeggedInterface] targetCommand file not found: " + reference_file_path.string());
        }

        // 加载是否启用详细日志输出的配置
        bool verbose = false;
        loadData::loadCppDataType(task_file, "legged_robot_interface.verbose", verbose);

        // 从配置文件加载模型设置（关节名、接触点、CppAD编译路径等）
        model_settings_ = loadModelSettings(task_file, "model_settings", verbose);

        // 处理CppAD模型文件夹路径：如果不是绝对路径，自动拼接用户主目录
        if (!model_settings_.modelFolderCppAd.empty() && model_settings_.modelFolderCppAd.front() != '/')
        {
            if (const char* home_dir = std::getenv("HOME"); home_dir != nullptr)
            {
                model_settings_.modelFolderCppAd = std::string(home_dir) + "/" + model_settings_.modelFolderCppAd;
            }
        }

        // 加载MPC（模型预测控制）配置
        mpc_settings_ = mpc::loadSettings(task_file, "mpc", verbose);
        // 加载SQP（序列二次规划）求解器配置
        sqp_settings_ = sqp::loadSettings(task_file, "sqp", verbose);
        // 加载Rollout（轨迹推演）配置
        rollout_settings_ = rollout::loadSettings(task_file, "rollout", verbose);
    }

    /**
     * @brief 设置关节名和足端接触点名
     * @param joint_names 机器人驱动关节名称列表
     * @param foot_names 足端接触点名称列表（3DoF接触）
     * @note 这些名称需要与URDF文件中的关节/连杆名称一致
     */
    void LeggedInterface::setupJointNames(const std::vector<std::string>& joint_names,
                                          const std::vector<std::string>& foot_names)
    {
        model_settings_.jointNames = joint_names;
        model_settings_.contactNames3DoF = foot_names;
    }

    /**
     * @brief 构建最优控制问题（核心函数）
     * @param task_file 任务配置文件路径
     * @param urdf_file URDF模型文件路径
     * @param reference_file 参考轨迹/步态配置文件路径
     * @param verbose 是否输出详细日志
     * @note 完成动力学、代价函数、约束、轨迹推演、初始化器等核心组件的构建
     */
    void LeggedInterface::setupOptimalControlProblem(const std::string& task_file,
                                                     const std::string& urdf_file,
                                                     const std::string& reference_file,
                                                     const bool verbose)
    {
        // 第一步：初始化机器人模型（Pinocchio+质心模型）
        setupModel(task_file, urdf_file, reference_file);

        // 加载初始状态（从配置文件读取，默认全零）
        initial_state_.setZero(centroidal_model_info_.stateDim);
        loadData::loadEigenMatrix(task_file, "initialState", initial_state_);

        // 初始化参考管理器（步态调度、摆动腿轨迹规划）
        setupReferenceManager(task_file, urdf_file, reference_file, verbose);

        // 创建最优控制问题实例
        problem_ptr_ = std::make_unique<OptimalControlProblem>();

        // 1. 设置动力学模型（自动微分版本的四足机器人动力学）
        std::unique_ptr<SystemDynamicsBase> dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(
            *pinocchio_interface_ptr_, centroidal_model_info_, "dynamics",
            model_settings_);
        problem_ptr_->dynamicsPtr = std::move(dynamicsPtr);

        // 2. 设置代价函数（基座轨迹跟踪代价）
        problem_ptr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(task_file, centroidal_model_info_, verbose));

        // 3. 设置约束条件
        // 3.1 加载摩擦锥配置（摩擦系数、屏障惩罚参数）
        scalar_t frictionCoefficient = 0.7;
        RelaxedBarrierPenalty::Config barrierPenaltyConfig;
        std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(task_file, verbose);

        // 为每个足端接触点添加约束
        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; i++)
        {
            const std::string& footName = model_settings_.contactNames3DoF[i];
            // 创建该足端的末端执行器运动学实例
            std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr =
                getEeKinematicsPtr({footName}, footName);

            // 3.2 摩擦锥约束（硬约束/软约束二选一）
            if (use_hard_friction_cone_constraint_)
            {
                // 硬约束：不允许违反，直接限制接触力在摩擦锥内
                problem_ptr_->inequalityConstraintPtr->add(footName + "_frictionCone",
                                                           getFrictionConeConstraint(i, frictionCoefficient));
            }
            else
            {
                // 软约束：允许轻微违反，但会施加惩罚代价
                problem_ptr_->softConstraintPtr->add(footName + "_frictionCone",
                                                     getFrictionConeSoftConstraint(
                                                         i, frictionCoefficient, barrierPenaltyConfig));
            }

            // 3.3 零力约束（摆动腿接触力为零）
            problem_ptr_->equalityConstraintPtr->add(footName + "_zeroForce", std::make_unique<ZeroForceConstraint>(
                                                         *reference_manager_ptr_, i, centroidal_model_info_));
            
            // 3.4 轮式足端非完整速度约束：支撑相约束侧向/法向速度，释放滚动方向
            problem_ptr_->equalityConstraintPtr->add(footName + "_rollingVelocity",
                                                     getWheelRollingVelocityConstraint(*eeKinematicsPtr, i));
            
            // 3.5 法向速度约束（摆动腿Z向速度/高度跟踪）
            problem_ptr_->equalityConstraintPtr->add(
                footName + "_normalVelocity",
                std::make_unique<NormalVelocityConstraintCppAd>(*reference_manager_ptr_, *eeKinematicsPtr, i));

        }

        // 3.6 自碰撞避免约束（软约束，防止机器人连杆碰撞）
        problem_ptr_->stateSoftConstraintPtr->add("selfCollision",
                                                  getSelfCollisionConstraint(
                                                      *pinocchio_interface_ptr_, task_file, "selfCollision", verbose));

        // 4. 设置预计算模块（缓存运动学/动力学计算结果，提升效率）
        problem_ptr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
            *pinocchio_interface_ptr_, centroidal_model_info_, *reference_manager_ptr_->getSwingTrajectoryPlanner(),
            model_settings_);

        // 5. 创建轨迹推演器（基于动力学模型的前向仿真）
        rollout_ptr_ = std::make_unique<TimeTriggeredRollout>(*problem_ptr_->dynamicsPtr, rollout_settings_);

        // 6. 创建初始化解算器（为MPC提供初始输入猜测）
        constexpr bool extend_normalized_momentum = true;
        initializer_ptr_ = std::make_unique<LeggedRobotInitializer>(centroidal_model_info_, *reference_manager_ptr_,
                                                                    extend_normalized_momentum);
    }

    /**
     * @brief 初始化机器人模型（Pinocchio+质心模型）
     * @param task_file 任务配置文件路径
     * @param urdf_file URDF模型文件路径
     * @param reference_file 参考轨迹/步态配置文件路径
     * @note 包含大量调试输出，用于验证模型加载和接触点配置的正确性
     */
    void LeggedInterface::setupModel(const std::string& task_file,
                                    const std::string& urdf_file,
                                    const std::string& reference_file)
    {
        // 1. 创建Pinocchio机器人模型接口（从URDF加载）
        pinocchio_interface_ptr_ = std::make_unique<PinocchioInterface>(
            centroidal_model::createPinocchioInterface(urdf_file, model_settings_.jointNames));

        // ===== DEBUG: 打印Pinocchio模型基础信息 =====
        const auto& model = pinocchio_interface_ptr_->getModel();
        std::cerr << "配置的关节数量: " << model_settings_.jointNames.size() << std::endl;
        std::cerr << "模型广义坐标数 model.nq: " << model.nq << std::endl;
        std::cerr << "模型自由度 model.nv: " << model.nv << std::endl;

        std::cerr << "模型关节名列表(model.names): " << std::endl;
        for (const auto& name : model.names) {
            std::cerr << "  - " << name << std::endl;
        }

        // 打印所有关节的nq(广义坐标数)/nv(自由度)
        for (pinocchio::JointIndex joint_id = 1; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id) {
            const auto& joint = model.joints[joint_id];
            std::cerr << "关节: " << model.names[joint_id]
                    << ", nq=" << joint.nq()
                    << ", nv=" << joint.nv() << std::endl;
        }

        // 验证：基座6DoF + 驱动关节数 应等于模型总自由度nv
        if (model.nv != 6 + model_settings_.jointNames.size()) {
            std::cerr << "ERROR: 模型自由度不匹配！期望 6 + " << model_settings_.jointNames.size()
                    << " = " << 6 + model_settings_.jointNames.size()
                    << ", 实际 model.nv = " << model.nv << std::endl;
        }

        // ===== DEBUG: 打印contactNames3DoF顺序和对应的frame id =====
        std::cerr << "contactNames3DoF (order used by OCS2):" << std::endl;
        for (size_t i = 0; i < model_settings_.contactNames3DoF.size(); ++i) {
            const auto& cname = model_settings_.contactNames3DoF[i];
            const auto fid = model.getFrameId(cname);
            std::cerr << "  [" << i << "] " << cname;

            // Pinocchio: 如果找不到frame，返回model.nframes
            if (fid == model.nframes) {
            std::cerr << "  -> frame NOT FOUND in pinocchio model!" << std::endl;
            } else {
            std::cerr << "  -> frameId=" << fid;
            if (fid < model.frames.size()) {
                std::cerr << ", frameName=" << model.frames[fid].name;
            }
            std::cerr << std::endl;
            }
        }

        // ===== 初始化质心模型信息 =====
        centroidal_model_info_ = centroidal_model::createCentroidalModelInfo(
            *pinocchio_interface_ptr_,
            centroidal_model::loadCentroidalType(task_file),  // 加载质心模型类型（完整/简化）
            centroidal_model::loadDefaultJointState(pinocchio_interface_ptr_->getModel().nq - 6, reference_file),  // 默认关节状态
            model_settings_.contactNames3DoF,  // 3DoF接触点列表
            model_settings_.contactNames6DoF); // 6DoF接触点列表（此处为空）

        // ===== DEBUG: 验证质心模型中末端执行器frame索引的顺序 =====
        std::cerr << "CentroidalModelInfo endEffectorFrameIndices:" << std::endl;
        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; ++i) {
            const auto fid = centroidal_model_info_.endEffectorFrameIndices[i];
            std::cerr << "  [" << i << "] fid=" << fid;
            if (fid < model.frames.size()) {
            std::cerr << " frame=" << model.frames[fid].name << std::endl;
            } else {
            std::cerr << " frame=OUT_OF_RANGE!" << std::endl;
            }
        }

        // （可选）打印contactNames6DoF（如果后续会用到）
        if (!model_settings_.contactNames6DoF.empty()) {
            std::cerr << "contactNames6DoF:" << std::endl;
            for (size_t i = 0; i < model_settings_.contactNames6DoF.size(); ++i) {
            const auto& cname = model_settings_.contactNames6DoF[i];
            const auto fid = model.getFrameId(cname);
            std::cerr << "  [" << i << "] " << cname;
            if (fid == model.nframes) {
                std::cerr << "  -> frame NOT FOUND in pinocchio model!" << std::endl;
            } else {
                std::cerr << "  -> frameId=" << fid;
                if (fid < model.frames.size()) {
                std::cerr << ", frameName=" << model.frames[fid].name;
                }
                std::cerr << std::endl;
            }
            }
        }
    }

    /**
     * @brief 初始化参考管理器
     * @param taskFile 任务配置文件路径
     * @param urdfFile URDF模型文件路径
     * @param referenceFile 参考轨迹/步态配置文件路径
     * @param verbose 是否输出详细日志
     * @note 参考管理器包含步态调度和摆动腿轨迹规划，是MPC的核心参考输入
     */
    void LeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& urdfFile,
                                                const std::string& referenceFile,
                                                const bool verbose)
    {
        // 创建摆动腿轨迹规划器（4条腿）
        auto swingTrajectoryPlanner =
            std::make_unique<SwingTrajectoryPlanner>(
                loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
        
        // 创建末端执行器运动学实例（用于计算足端位置/速度）
        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr(
            {model_settings_.contactNames3DoF}, "ALL_FOOT");      
        
        // 创建切换模型参考管理器（核心：步态调度+摆动腿轨迹）
        reference_manager_ptr_ =
            std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose),
                                                            std::move(swingTrajectoryPlanner),
                                                            *eeKinematicsPtr);
                                                         
    }

    /**
     * @brief 加载步态调度配置
     * @param file 参考轨迹/步态配置文件路径
     * @param verbose 是否输出详细日志
     * @return 步态调度实例的共享指针
     * @note 步态调度包含初始模式序列、默认步态模板（周期、相位切换点、接触模式）
     */
    std::shared_ptr<GaitSchedule> LeggedInterface::loadGaitSchedule(const std::string& file, bool verbose) const
    {
        // 加载初始模式调度（启动阶段的接触模式）
        const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
        // 加载默认模式序列模板（周期性步态的基础模板）
        const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

        // 从模板构建默认步态
        const auto defaultGait = [defaultModeSequenceTemplate]
        {
            Gait gait{};
            // 步态周期 = 模板中最后一个切换时间
            gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
            // 事件相位：将时间转换为相位（0~1）
            std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1,
                          defaultModeSequenceTemplate.switchingTimes.end() - 1,
                          [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
            // 接触模式序列
            gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
            return gait;
        }();

        // 打印步态配置（如果启用详细日志）
        if (verbose)
        {
            std::cerr << "\n#### Modes Schedule: ";
            std::cerr << "\n#### =============================================================================\n";
            std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
            std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
            std::cerr << "#### =============================================================================\n";
        }

        // 创建步态调度实例（包含初始调度、默认模板、相位切换时间）
        return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate,
                                              model_settings_.phaseTransitionStanceTime);
    }






/**
 * @brief 初始化输入代价权重矩阵R
 * @param taskFile 任务配置文件路径
 * @param info 质心模型信息
 * @return 输入代价权重矩阵R
 * @note 核心逻辑：将足端速度代价映射到关节空间
 */
matrix_t LeggedInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info)
{
    // ========================= 调试信息打印 =========================
    // 打印质心模型的关键维度信息，用于调试矩阵维度匹配问题
    std::cerr << "[DEBUG] initializeInputCostWeight - info.generalizedCoordinatesNum = " << info.generalizedCoordinatesNum << std::endl; // 广义坐标数量（包含基座+关节）
    std::cerr << "[DEBUG] initializeInputCostWeight - info.actuatedDofNum = " << info.actuatedDofNum << std::endl; // 驱动自由度数量（仅关节）
    std::cerr << "[DEBUG] initializeInputCostWeight - info.stateDim = " << info.stateDim << std::endl; // 状态维度
    std::cerr << "[DEBUG] initializeInputCostWeight - info.inputDim = " << info.inputDim << std::endl; // 输入维度

    // ========================= 接触维度计算 =========================
    // 总接触维度 = 3(每个3DOF接触的维度) * 3DOF接触点数量（如足端接触点）
    const size_t totalContactDim = 3 * info.numThreeDofContacts;

    // ========================= 雅克比矩阵计算准备 =========================
    // 获取Pinocchio机器人模型（只读）
    const auto& model = pinocchio_interface_ptr_->getModel();
    // 获取Pinocchio机器人数据（可写，用于存储计算结果）
    auto& data = pinocchio_interface_ptr_->getData();
    // 从初始状态中提取广义坐标q（包含基座位姿+关节角度）
    const auto q = centroidal_model::getGeneralizedCoordinates(initial_state_, centroidal_model_info_);
    
    // 计算关节雅克比矩阵（更新data中的jacobian数据）
    computeJointJacobians(model, data, q);
    // 更新所有帧的位姿（基于当前广义坐标q）
    updateFramePlacements(model, data);

    // ========================= 构建基座到足端的雅克比矩阵 =========================
    // 初始化足端雅克比矩阵：维度为[总接触维度 × 驱动自由度数量]
    // 行：所有足端接触点的3维线速度 列：所有驱动关节的自由度
    matrix_t base2feetJac(totalContactDim, info.actuatedDofNum);
    
    // 遍历每个3DOF接触点（如每个足端）
    for (size_t i = 0; i < info.numThreeDofContacts; i++)
    {
        // 初始化单个接触点的雅克比矩阵：6维（线速度+角速度）× 广义坐标数量
        matrix_t jac = matrix_t::Zero(6, info.generalizedCoordinatesNum);
        
        // 获取当前接触帧的雅克比矩阵
        // 参数说明：
        // - model: 机器人模型
        // - data: 计算数据存储对象
        // - model.getBodyId(...)：通过接触点名称获取对应Body的ID
        // - pinocchio::LOCAL_WORLD_ALIGNED：雅克比矩阵的参考坐标系（局部对齐世界坐标系）
        // - jac: 输出的雅克比矩阵
        getFrameJacobian(model, data, model.getBodyId(model_settings_.contactNames3DoF[i]),
                         pinocchio::LOCAL_WORLD_ALIGNED, jac);
        
        // 提取雅克比矩阵的线速度部分（前3行）和关节部分（列从6开始，跳过基座6个自由度）
        // 并将其赋值到足端雅克比矩阵的对应位置：第i个接触点对应3*i ~ 3*(i+1)行
        base2feetJac.block(3 * i, 0, 3, info.actuatedDofNum) = jac.block(0, 6, 3, info.actuatedDofNum);
    }

    // ========================= 加载任务空间代价矩阵并映射到关节空间 =========================
    // 从配置文件加载任务空间的输入代价权重矩阵R（初始值）
    matrix_t rTaskspace(info.inputDim, info.inputDim);
    loadData::loadEigenMatrix(taskFile, "R", rTaskspace);
    
    // 初始化最终的代价矩阵r，先拷贝任务空间的初始值
    matrix_t r = rTaskspace;
    
    // 核心映射逻辑：将足端速度代价从任务空间（笛卡尔空间）映射到关节空间
    // 数学原理：J^T * R_task * J （雅克比转置 × 任务空间代价 × 雅克比）
    // 赋值区域：矩阵中对应关节速度的子块（跳过接触维度部分）
    r.block(totalContactDim, totalContactDim, info.actuatedDofNum, info.actuatedDofNum) =
        base2feetJac.transpose() *                  // 足端雅克比矩阵转置
        rTaskspace.block(totalContactDim, totalContactDim, info.actuatedDofNum, info.actuatedDofNum) *  // 任务空间中关节速度对应的代价子矩阵
        base2feetJac;                               // 足端雅克比矩阵

    // 返回映射后的最终输入代价权重矩阵R
    return r;
}

   

    /**
     * @brief 获取基座轨迹跟踪代价函数
     * @param taskFile 任务配置文件路径
     * @param info 质心模型信息
     * @param verbose 是否输出详细日志
     * @return 状态输入二次代价函数实例
     * @note 代价函数形式：0.5*(x-x_ref)^T Q (x-x_ref) + 0.5*u^T R u
     */
    std::unique_ptr<StateInputCost> LeggedInterface::getBaseTrackingCost(
        const std::string& taskFile, const CentroidalModelInfo& info,
        bool verbose)
    {
        // 调试输出：打印状态维度（预期22：质心6DoF+动量6+关节10）
        std::cerr << "[DEBUG] getBaseTrackingCost - info.stateDim = " << info.stateDim << std::endl;

        // 加载状态代价权重矩阵Q
        matrix_t Q(info.stateDim, info.stateDim);
        loadData::loadEigenMatrix(taskFile, "Q", Q);
        // 初始化输入代价权重矩阵R
        matrix_t R = initializeInputCostWeight(taskFile, info);

        // 打印Q/R矩阵（如果启用详细日志）
        if (verbose)
        {
            std::cerr << "\n #### Base Tracking Cost Coefficients: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << "Q:\n" << Q << "\n";
            std::cerr << "R:\n" << R << "\n";
            std::cerr << " #### =============================================================================\n";
        }

        // 创建四足机器人状态输入二次代价函数
        return std::make_unique<LeggedRobotStateInputQuadraticCost>(std::move(Q), std::move(R), info,
                                                                    *reference_manager_ptr_);
    }

    /**
     * @brief 加载摩擦锥约束配置
     * @param taskFile 任务配置文件路径
     * @param verbose 是否输出详细日志
     * @return 摩擦系数 + 屏障惩罚配置
     */
    std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedInterface::loadFrictionConeSettings(
        const std::string& taskFile, bool verbose)
    {
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        const std::string prefix = "frictionConeSoftConstraint.";

        // 默认摩擦系数=1.0
        scalar_t frictionCoefficient = 1.0;
        // 屏障惩罚配置（mu=惩罚系数，delta=屏障阈值）
        RelaxedBarrierPenalty::Config barrierPenaltyConfig;
        
        if (verbose)
        {
            std::cerr << "\n #### Friction Cone Settings: ";
            std::cerr << "\n #### =============================================================================\n";
        }
        
        // 从配置文件加载参数
        loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
        loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
        loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
        
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
        }

        return {frictionCoefficient, barrierPenaltyConfig};
    }

    /**
     * @brief 创建硬约束版本的摩擦锥约束
     * @param contactPointIndex 接触点索引
     * @param frictionCoefficient 摩擦系数
     * @return 摩擦锥约束实例
     */
    std::unique_ptr<StateInputConstraint> LeggedInterface::getFrictionConeConstraint(
        size_t contactPointIndex, scalar_t frictionCoefficient)
    {
        // 摩擦锥约束配置（仅需摩擦系数）
        FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
        return std::make_unique<FrictionConeConstraint>(*reference_manager_ptr_, frictionConeConConfig,
                                                        contactPointIndex,
                                                        centroidal_model_info_);
    }

    /**
     * @brief 创建软约束版本的摩擦锥约束（带屏障惩罚）
     * @param contactPointIndex 接触点索引
     * @param frictionCoefficient 摩擦系数
     * @param barrierPenaltyConfig 屏障惩罚配置
     * @return 软约束版本的摩擦锥代价
     */
    std::unique_ptr<StateInputCost> LeggedInterface::getFrictionConeSoftConstraint(
        size_t contactPointIndex, scalar_t frictionCoefficient,
        const RelaxedBarrierPenalty::Config& barrierPenaltyConfig)
    {
        // 将硬约束包装为软约束（违反时施加屏障惩罚）
        return std::make_unique<StateInputSoftConstraint>(
            getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
            std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
    }

    /**
     * @brief 创建末端执行器运动学实例（CppAD自动微分版本）
     * @param foot_names 足端名称列表
     * @param model_name 模型名称（用于CppAD编译缓存）
     * @return 末端执行器运动学实例
     * @note 用于计算足端的位置、速度、雅克比等运动学信息
     */
    std::unique_ptr<EndEffectorKinematics<scalar_t>> LeggedInterface::getEeKinematicsPtr(
        const std::vector<std::string>& foot_names,
        const std::string& model_name)
    {
        // 转换质心模型信息为CppAD兼容格式
        const auto infoCppAd = centroidal_model_info_.toCppAd();
        const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
        
        // 速度更新回调函数（用于自动微分计算）
        auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state,
                                                   PinocchioInterfaceCppAd& pinocchioInterfaceAd)
        {
            // 从状态中提取广义坐标
            const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
            // 更新质心动力学
            updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
        };
        
        // 创建CppAD版本的Pinocchio末端执行器运动学
        std::unique_ptr<EndEffectorKinematics<scalar_t>> end_effector_kinematics = std::make_unique<
            PinocchioEndEffectorKinematicsCppAd>(
            *pinocchio_interface_ptr_, pinocchioMappingCppAd,
            foot_names,
            centroidal_model_info_.stateDim,
            centroidal_model_info_.inputDim,
            velocityUpdateCallback, model_name,
            model_settings_.modelFolderCppAd,      // CppAD编译缓存路径
            model_settings_.recompileLibrariesCppAd,  // 是否重新编译
            model_settings_.verboseCppAd);         // 是否输出编译日志

        return end_effector_kinematics;
    }

    std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroVelocityConstraint(
        const EndEffectorKinematics<scalar_t>& end_effector_kinematics,
        const size_t contact_point_index)
    {
        auto eeZeroVelConConfig = [](scalar_t positionErrorGain)
        {
            EndEffectorLinearConstraint::Config config;
            config.b.setZero(3);
            config.Av.setIdentity(3, 3);
            if (!numerics::almost_eq(positionErrorGain, 0.0))
            {
                config.Ax.setZero(3, 3);
                config.Ax(2, 2) = positionErrorGain;
            }
            return config;
        };
        return std::make_unique<ZeroVelocityConstraintCppAd>(
            *reference_manager_ptr_, end_effector_kinematics, contact_point_index,
            eeZeroVelConConfig(model_settings_.positionErrorGain));
    }

    std::unique_ptr<StateInputConstraint> LeggedInterface::getWheelRollingVelocityConstraint(
        const EndEffectorKinematics<scalar_t>& end_effector_kinematics,
        const size_t contact_point_index)
    {
        return std::make_unique<WheelRollingVelocityConstraintCppAd>(
            *reference_manager_ptr_, end_effector_kinematics, centroidal_model_info_, contact_point_index);
    }

    // /**
    //  * @brief 创建非完整性零速度约束（沿着机身坐标系x方向可以滚动）
    //  * @param end_effector_kinematics 末端执行器运动学实例
    //  * @param contact_point_index 接触点索引
    //  * @return 非完整性滚动约束实例
    //  */
    // std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroVelocityConstraint(
    //     const EndEffectorKinematics<scalar_t>& end_effector_kinematics,
    //     const size_t contact_point_index)
    // {
    //     // 构建末端执行器线性约束配置
    //             auto eeZeroVelConConfig = [](scalar_t positionErrorGain)
    //     {
    //         EndEffectorLinearConstraint::Config config;
    //         config.b.setZero(3);  // 约束目标值：0
            
    //         // ========== 核心修改：仅约束y(1)/z(2)方向速度，x(0)方向不约束 ==========
    //         config.Av.setZero(3, 3);  // 先置零
    //         config.Av(1, 1) = 1.0;    // 侧向（y）速度约束
    //         config.Av(2, 2) = 1.0;    // 法向（z）速度约束
    //         config.Av(0, 0) = 0.0;  // 滚动（x）速度不约束（默认0，可省略）
            
    //         // 如果位置误差增益非零，添加位置误差惩罚（用于微调）
    //         if (!numerics::almost_eq(positionErrorGain, 0.0))
    //         {
    //             config.Ax.setZero(3, 3);
    //             config.Ax(2, 2) = positionErrorGain;  // z方向位置误差惩罚
    //         }
    //         // 修改后：置0，完全移除位置约束
    //         //config.Ax.setZero(3, 3); // 无任何位置惩罚
    //         return config;
    //     };
        
    //     // 创建CppAD版本的非完整零速度约束
    //     return std::make_unique<ZeroVelocityConstraintCppAd>(
    //         *reference_manager_ptr_, end_effector_kinematics, contact_point_index,
    //         eeZeroVelConConfig(model_settings_.positionErrorGain));
    // }

    // /**
    //  * @brief 创建非完整性零速度约束（沿着机身坐标系x方向可以滚动）
    //  * @param end_effector_kinematics 末端执行器运动学实例
    //  * @param contact_point_index 接触点索引
    //  * @return 非完整性滚动约束实例
    //  */
    // std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroVelocityConstraint(
    //     const EndEffectorKinematics<scalar_t>& end_effector_kinematics,
    //     const size_t contact_point_index)
    // {
    //     // 构建末端执行器线性约束配置（捕获this以访问实时状态）
    //     auto eeZeroVelConConfig = [this](scalar_t positionErrorGain)
    //     {
    //         EndEffectorLinearConstraint::Config config;
    //         config.b.setZero(3);  // 约束目标值：0

    //         // ========== 核心修改：用实时基座欧拉角计算旋转矩阵（替代Pinocchio方式） ==========
    //         // 1. 从实时观测中提取基座当前位姿（x/y/z/roll/pitch/yaw，对应你代码中的observation.state）
    //         // 注意：这里需要确保能访问到实时的observation（可通过reference_manager_ptr_或类成员传递）
    //         const vector_t currentPose = observation.state.segment<6>(6); // 基座位姿：x/y/z/roll/pitch/yaw
    //         const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3); // 提取zyx欧拉角（roll/pitch/yaw）
            
    //         // 2. 用你提供的函数生成世界→机身的旋转矩阵 R_world_to_body
    //         // 关键：该函数的欧拉角顺序必须是zyx（yaw(z)→pitch(y)→roll(x)），与currentPose的欧拉角顺序一致
    //         const Eigen::Matrix3d R_world_to_body = getRotationMatrixFromZyxEulerAngles(zyx);

    //         // ========== 验证旋转矩阵（可选，调试用）==========
    //         std::cerr << "[DEBUG] 实时旋转矩阵 R_world_to_body (欧拉角生成):\n" << R_world_to_body << std::endl;
    //         std::cerr << "[DEBUG] 机身y轴在世界系方向: " << R_world_to_body.col(1).transpose() << std::endl;

    //         // ========== 构建机身系约束矩阵 ==========
    //         Eigen::Matrix3d Av_body;
    //         Av_body.setZero();
    //         Av_body(1, 1) = 1.0;  // 机身系y轴（侧向）速度约束
    //         Av_body(2, 2) = 1.0;  // 机身系z轴（法向）速度约束

    //         // ========== 映射到LOCAL_WORLD_ALIGNED系 ==========
    //         config.Av = R_world_to_body.transpose() * Av_body * R_world_to_body;

    //         // ========== 位置误差惩罚（保持不变） ==========
    //         if (!numerics::almost_eq(positionErrorGain, 0.0))
    //         {
    //             config.Ax.setZero(3, 3);
    //             config.Ax(2, 2) = positionErrorGain;  // z方向位置误差惩罚
    //         }

    //         return config;
    //     };

    //     // 创建CppAD版本的非完整零速度约束
    //     return std::make_unique<ZeroVelocityConstraintCppAd>(
    //         *reference_manager_ptr_, end_effector_kinematics, contact_point_index,
    //         eeZeroVelConConfig(model_settings_.positionErrorGain));
    // }




    /**
     * @brief 创建自碰撞避免软约束
     * @param pinocchioInterface Pinocchio模型接口
     * @param taskFile 任务配置文件路径
     * @param prefix 配置参数前缀
     * @param verbose 是否输出详细日志
     * @return 自碰撞避免软约束代价
     */
    std::unique_ptr<StateCost> LeggedInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                           const std::string& taskFile,
                                                                           const std::string& prefix,
                                                                           bool verbose)
    {
        // 碰撞检测配置参数
        std::vector<std::pair<size_t, size_t>> collisionObjectPairs;  // 碰撞物体对（索引）
        std::vector<std::pair<std::string, std::string>> collisionLinkPairs;  // 碰撞连杆对（名称）
        scalar_t mu = 1e-2;  // 惩罚系数
        scalar_t delta = 1e-3;  // 屏障阈值
        scalar_t minimumDistance = 0.0;  // 最小安全距离

        // 加载配置参数
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        if (verbose)
        {
            std::cerr << "\n #### SelfCollision Settings: ";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, mu, prefix + ".mu", verbose);
        loadData::loadPtreeValue(pt, delta, prefix + ".delta", verbose);
        loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", verbose);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, verbose);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, verbose);

        // 创建Pinocchio几何接口（用于碰撞检测）
        geometry_interface_ptr_ = std::make_unique<PinocchioGeometryInterface>(
            pinocchioInterface, collisionLinkPairs, collisionObjectPairs);
        
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
            const size_t numCollisionPairs = geometry_interface_ptr_->getNumCollisionPairs();
            std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";
        }

        // 创建自碰撞硬约束（限制连杆间距离≥最小安全距离）
        std::unique_ptr<StateConstraint> constraint = std::make_unique<LeggedSelfCollisionConstraint>(
            CentroidalModelPinocchioMapping(centroidal_model_info_), *geometry_interface_ptr_, minimumDistance);

        // 创建屏障惩罚（违反约束时施加惩罚）
        auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

        // 将硬约束包装为软约束代价
        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
    }
} // namespace legged

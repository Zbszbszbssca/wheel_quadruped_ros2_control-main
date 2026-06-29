//
// Created by tlab-uav on 24-9-26.
//

#include <utility>
#include <algorithm>
#include <cmath>

// GaitManager类头文件：四足机器人步态管理器核心类
#include "ocs2_quadruped_controller/control/GaitManager.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

// 命名空间：ocs2的legged_robot模块
namespace ocs2::legged_robot
{
    namespace
    {
        constexpr size_t kNumLegs = 4;
        constexpr scalar_t kEpsilon = 1e-6;
        constexpr std::array<const char*, kNumLegs> kLegNames = {"FL", "FR", "RL", "RR"};
    }

    /**
     * @brief GaitManager构造函数
     * @param ctrl_interfaces: 控制器接口集合，用于获取控制指令
     * @param gait_schedule_ptr: 步态调度器指针，用于管理和更新机器人步态序列
     * @details 初始化步态管理器，设置默认目标步态（0.0到1.0秒为支撑相）
     */
    GaitManager::GaitManager(CtrlInterfaces& ctrl_interfaces,
                             std::shared_ptr<GaitSchedule> gait_schedule_ptr,
                             PinocchioInterface pinocchioInterface,
                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                             const CentroidalModelInfo& info)
        : ctrl_interfaces_(ctrl_interfaces),                    // 保存控制器接口引用
          gait_schedule_ptr_(std::move(gait_schedule_ptr)),     // 转移步态调度器指针所有权
          pinocchio_interface_(std::move(pinocchioInterface)),
          ee_kinematics_(endEffectorKinematics.clone()),
          info_(info),
          target_gait_({0.0, 1.0}, {STANCE}),                   // 默认目标步态：时间区间[0,1]秒，模式为支撑相
          stance_gait_({0.0, 0.5}, {STANCE})
    {
        if (auto* pinocchioKinematics = dynamic_cast<PinocchioEndEffectorKinematics*>(ee_kinematics_.get()))
        {
            pinocchioKinematics->setPinocchioInterface(pinocchio_interface_);
        }
        online_nominal_foot_pos_base_.fill(vector3_t::Zero());
        online_last_rolling_error_.fill(0.0);
        online_last_lateral_error_.fill(0.0);
    }

    /**
     * @brief SQP求解器运行前的回调函数
     * @param initTime: MPC优化的初始时间
     * @param finalTime: MPC优化的最终时间
     * @param currentState: 当前机器人状态
     * @param referenceManager: 参考管理器（未使用，参数保留）
     * @details 在每次MPC求解前执行，检查并更新步态序列：
     * 1. 获取目标步态
     * 2. 若步态已更新，则将新步态插入到步态调度器中
     */
    void GaitManager::preSolverRun(const scalar_t initTime, const scalar_t finalTime,
                                   const vector_t& currentState,
                                   const ReferenceManagerInterface& referenceManager)
    {
        (void)referenceManager;

        // 获取最新的目标步态（检查控制指令是否变化）
        getTargetGait(initTime, finalTime, currentState);

        // 如果步态标记为已更新，则更新步态调度器
        if (gait_updated_)
        {
            // 计算MPC优化的时间范围
            const auto timeHorizon = finalTime - initTime;
            RCLCPP_INFO(rclcpp::get_logger("GaitManager"),
                        "Updating gait schedule - target_gait applied at finalTime: %.3f s, timeHorizon: %.3f s",
                        finalTime, timeHorizon);
            const auto startTime = finalTime;
            const auto endTime = finalTime + timeHorizon;
            // 将目标步态插入到步态调度器中，生效时间为startTime，覆盖到endTime
            gait_schedule_ptr_->insertModeSequenceTemplate(target_gait_, startTime,
                                                           endTime);
            // 重置步态更新标记
            gait_updated_ = false;
        }
    }

    /**
     * @brief 初始化步态管理器
     * @param gait_file: 步态配置文件路径
     * @details 从配置文件加载步态列表和对应的步态模板：
     * 1. 加载步态名称列表
     * 2. 遍历名称列表，加载每个名称对应的模式序列模板
     * 3. 输出初始化完成日志
     */
    void GaitManager::init(const std::string& gait_file)
    {
        // 清空步态名称列表（防止重复加载）
        gait_name_list_.clear();
        // 从配置文件加载步态名称列表（配置项为"list"）
        loadData::loadStdVector(gait_file, "list", gait_name_list_, verbose_);

        // 清空步态模板列表
        gait_list_.clear();
        // 遍历步态名称列表，加载每个名称对应的步态模板
        for (const auto& name : gait_name_list_)
        {
            // 加载指定名称的步态模式序列模板并添加到列表
            gait_list_.push_back(loadModeSequenceTemplate(gait_file, name, verbose_));
        }

        if (!gait_list_.empty())
        {
            stance_gait_ = gait_list_.front();
        }

        loadKinematicOnlineGaitConfig(gait_file);
        initializeNominalFootPositions();

        // 输出初始化完成日志
        RCLCPP_INFO(rclcpp::get_logger("gait_manager"), "GaitManager is ready.");
    }

    void GaitManager::loadKinematicOnlineGaitConfig(const std::string& gait_file)
    {
        boost::property_tree::ptree pt;
        read_info(gait_file, pt);
        const std::string prefix = "kinematicOnlineGait.";

        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.enabled, prefix + "enabled", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.activationCommand, prefix + "activationCommand", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.rollingThreshold, prefix + "rollingThreshold", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.lateralThreshold, prefix + "lateralThreshold", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.triggerScoreThreshold, prefix + "triggerScoreThreshold", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.criticalScoreThreshold, prefix + "criticalScoreThreshold", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.swingDuration, prefix + "swingDuration", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.preStanceDuration, prefix + "preStanceDuration", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.interStanceDuration, prefix + "interStanceDuration", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.stanceAfterSwing, prefix + "stanceAfterSwing", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.minTriggerInterval, prefix + "minTriggerInterval", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.startDelay, prefix + "startDelay", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.diagnosticLogPeriod, prefix + "diagnosticLogPeriod", verbose_);
        loadData::loadPtreeValue(pt, kinematic_online_gait_config_.diagnosticVerbose, prefix + "diagnosticVerbose", verbose_);

        kinematic_online_gait_config_.activationCommand = std::max(kinematic_online_gait_config_.activationCommand, 0);
        kinematic_online_gait_config_.rollingThreshold = std::max(kinematic_online_gait_config_.rollingThreshold, kEpsilon);
        kinematic_online_gait_config_.lateralThreshold = std::max(kinematic_online_gait_config_.lateralThreshold, kEpsilon);
        kinematic_online_gait_config_.triggerScoreThreshold = std::max(kinematic_online_gait_config_.triggerScoreThreshold, 1.0 + kEpsilon);
        kinematic_online_gait_config_.criticalScoreThreshold = std::max(kinematic_online_gait_config_.criticalScoreThreshold,
                                                                        kinematic_online_gait_config_.triggerScoreThreshold);
        kinematic_online_gait_config_.swingDuration = std::max(kinematic_online_gait_config_.swingDuration, kEpsilon);
        kinematic_online_gait_config_.preStanceDuration = std::max(kinematic_online_gait_config_.preStanceDuration, 0.0);
        kinematic_online_gait_config_.interStanceDuration = std::max(kinematic_online_gait_config_.interStanceDuration, 0.0);
        kinematic_online_gait_config_.stanceAfterSwing = std::max(kinematic_online_gait_config_.stanceAfterSwing, 0.0);
        kinematic_online_gait_config_.minTriggerInterval = std::max(kinematic_online_gait_config_.minTriggerInterval, 0.0);
        kinematic_online_gait_config_.startDelay = std::max(kinematic_online_gait_config_.startDelay, 0.0);
        kinematic_online_gait_config_.diagnosticLogPeriod = std::max(kinematic_online_gait_config_.diagnosticLogPeriod, kEpsilon);
    }

    /**
     * @brief 获取目标步态（根据控制指令更新）
     */
    void GaitManager::getTargetGait(const scalar_t initTime, const scalar_t finalTime, const vector_t& currentState)
    {
        const int command = ctrl_interfaces_.control_inputs_.command;

        if (command == kinematic_online_gait_config_.activationCommand)
        {
            updateKinematicOnlineGait(initTime, finalTime, currentState);
            return;
        }

        if (command == 0)
        {
            if (online_mode_active_)
            {
                updateKinematicOnlineGait(initTime, finalTime, currentState);
            }
            return;
        }

        resetKinematicOnlineGait();
        if (command == last_command_) return;

        const int gaitIndex = command - 2;
        if (gaitIndex < 0)
        {
            return;
        }
        switchToFixedGait(static_cast<size_t>(gaitIndex), command);
    }

    void GaitManager::updateKinematicOnlineGait(const scalar_t initTime, const scalar_t finalTime,
                                                const vector_t& currentState)
    {
        if (!kinematic_online_gait_config_.enabled)
        {
            resetKinematicOnlineGait();
            if (last_command_ == kinematic_online_gait_config_.activationCommand) return;
            last_command_ = kinematic_online_gait_config_.activationCommand;
            return;
        }

        if (!online_mode_active_ || last_command_ != kinematic_online_gait_config_.activationCommand)
        {
            online_mode_active_ = true;
            online_cycle_active_ = false;
            online_mode_start_time_ = initTime;
            online_cycle_end_time_ = -std::numeric_limits<scalar_t>::infinity();
            online_next_allowed_trigger_time_ = initTime + kinematic_online_gait_config_.startDelay;
            online_last_swing_leg_ = kNumLegs;
            online_reference_initialized_ = false;
            initializeOnlineFootholdReferences(currentState);
            last_command_ = kinematic_online_gait_config_.activationCommand;
            target_gait_ = stance_gait_;
            gait_updated_ = true;
            RCLCPP_INFO(rclcpp::get_logger("GaitManager"),
                        "Kinematic online gait enter command=%d, stance monitor active.",
                        kinematic_online_gait_config_.activationCommand);
        }

        if (!online_nominal_initialized_ && !initializeNominalFootPositions())
        {
            const auto currentFeetBase = getReferenceFootPositionsBaseFrame(currentState);
            online_nominal_foot_pos_base_ = currentFeetBase;
            online_nominal_initialized_ = true;
            RCLCPP_WARN(rclcpp::get_logger("GaitManager"),
                        "Kinematic online gait nominal footholds initialized from current state fallback.");
        }

        if (online_cycle_active_)
        {
            if (initTime >= online_cycle_end_time_)
            {
                initializeOnlineFootholdReferences(currentState);
                online_cycle_active_ = false;
                online_next_allowed_trigger_time_ = initTime + kinematic_online_gait_config_.minTriggerInterval;
                target_gait_ = stance_gait_;
                gait_updated_ = true;
                RCLCPP_INFO(rclcpp::get_logger("GaitManager"),
                            "Kinematic online gait return to stance, next trigger allowed after %.2f s.",
                            online_next_allowed_trigger_time_);
            }
            return;
        }

        if (!online_reference_initialized_)
        {
            initializeOnlineFootholdReferences(currentState);
        }

        const auto footPositionsBase = getReferenceFootPositionsBaseFrame(currentState);
        size_t selectedLeg = kNumLegs;
        scalar_t selectedScore = 1.0;
        for (size_t leg = 0; leg < kNumLegs; ++leg)
        {
            const vector3_t error = footPositionsBase[leg] - online_nominal_foot_pos_base_[leg];
            const scalar_t rollingError = error.x();
            const scalar_t lateralError = error.y();
            online_last_rolling_error_[leg] = rollingError;
            online_last_lateral_error_[leg] = lateralError;

            const scalar_t rollingScore = std::abs(rollingError) / kinematic_online_gait_config_.rollingThreshold;
            const scalar_t lateralScore = std::abs(lateralError) / kinematic_online_gait_config_.lateralThreshold;
            const scalar_t score = std::max(rollingScore, lateralScore);
            if (score > selectedScore)
            {
                selectedScore = score;
                selectedLeg = leg;
            }
        }

        if (kinematic_online_gait_config_.diagnosticVerbose &&
            initTime - online_last_diagnostic_time_ >= kinematic_online_gait_config_.diagnosticLogPeriod)
        {
            online_last_diagnostic_time_ = initTime;
            RCLCPP_INFO(rclcpp::get_logger("GaitManager"),
                        "Kinematic online gait errors rolling=[%.3f %.3f %.3f %.3f] lateral=[%.3f %.3f %.3f %.3f] selected=%s score=%.2f",
                        online_last_rolling_error_[0], online_last_rolling_error_[1],
                        online_last_rolling_error_[2], online_last_rolling_error_[3],
                        online_last_lateral_error_[0], online_last_lateral_error_[1],
                        online_last_lateral_error_[2], online_last_lateral_error_[3],
                        selectedLeg < kNumLegs ? kLegNames[selectedLeg] : "none", selectedScore);
        }

        const bool criticalDeviation = selectedScore >= kinematic_online_gait_config_.criticalScoreThreshold;
        if (selectedLeg >= kNumLegs || selectedScore < kinematic_online_gait_config_.triggerScoreThreshold ||
            (!criticalDeviation && initTime < online_next_allowed_trigger_time_))
        {
            return;
        }

        target_gait_ = makeTrotRecoveryTemplate();
        gait_updated_ = true;
        online_cycle_active_ = true;
        online_cycle_end_time_ = finalTime + kinematic_online_gait_config_.preStanceDuration +
                                 2.0 * kinematic_online_gait_config_.swingDuration +
                                 kinematic_online_gait_config_.interStanceDuration +
                                 kinematic_online_gait_config_.stanceAfterSwing;
        online_next_allowed_trigger_time_ = std::numeric_limits<scalar_t>::infinity();
        online_last_swing_leg_ = selectedLeg;

        RCLCPP_INFO(rclcpp::get_logger("GaitManager"),
                    "Kinematic online gait trigger trot recovery from leg=%s/%zu rolling=%.3f lateral=%.3f score=%.2f",
                    kLegNames[selectedLeg], selectedLeg, online_last_rolling_error_[selectedLeg],
                    online_last_lateral_error_[selectedLeg], selectedScore);
    }

    void GaitManager::resetKinematicOnlineGait()
    {
        if (!online_mode_active_ && !online_cycle_active_)
        {
            return;
        }
        online_mode_active_ = false;
        online_cycle_active_ = false;
        online_mode_start_time_ = -1.0;
        online_cycle_end_time_ = -std::numeric_limits<scalar_t>::infinity();
        online_next_allowed_trigger_time_ = -std::numeric_limits<scalar_t>::infinity();
        online_reference_initialized_ = false;
        online_last_swing_leg_ = kNumLegs;
    }

    bool GaitManager::initializeNominalFootPositions()
    {
        const auto& model = pinocchio_interface_.getModel();
        if (!ee_kinematics_ || info_.qPinocchioNominal.size() != model.nq ||
            !info_.qPinocchioNominal.allFinite())
        {
            return false;
        }

        auto& data = pinocchio_interface_.getData();
        pinocchio::forwardKinematics(model, data, info_.qPinocchioNominal);
        pinocchio::updateFramePlacements(model, data);
        if (auto* pinocchioKinematics = dynamic_cast<PinocchioEndEffectorKinematics*>(ee_kinematics_.get()))
        {
            pinocchioKinematics->setPinocchioInterface(pinocchio_interface_);
        }
        const auto nominalFootPositionsWorld = ee_kinematics_->getPosition(info_.qPinocchioNominal);
        if (nominalFootPositionsWorld.size() < kNumLegs)
        {
            return false;
        }

        const vector3_t nominalBasePosition = info_.qPinocchioNominal.head<3>();
        for (size_t leg = 0; leg < kNumLegs; ++leg)
        {
            if (!nominalFootPositionsWorld[leg].allFinite())
            {
                return false;
            }
            online_nominal_foot_pos_base_[leg] = nominalFootPositionsWorld[leg] - nominalBasePosition;
        }
        online_nominal_initialized_ = true;
        return true;
    }

    bool GaitManager::getBaseYawPose(const vector_t& currentState, vector3_t& basePositionWorld, scalar_t& yaw) const
    {
        if (currentState.size() < static_cast<Eigen::Index>(info_.stateDim) || !currentState.allFinite())
        {
            return false;
        }
        const vector_t basePose = centroidal_model::getBasePose(currentState, info_);
        basePositionWorld = basePose.head<3>();
        yaw = basePose(3);
        return basePositionWorld.allFinite() && std::isfinite(yaw);
    }

    void GaitManager::initializeOnlineFootholdReferences(const vector_t& currentState)
    {
        vector3_t basePositionWorld = vector3_t::Zero();
        scalar_t yaw = 0.0;
        if (!getBaseYawPose(currentState, basePositionWorld, yaw))
        {
            online_reference_initialized_ = false;
            return;
        }

        vector3_t zyx = vector3_t::Zero();
        zyx.x() = yaw;
        const matrix3_t R_base_to_world = getRotationMatrixFromZyxEulerAngles(zyx);
        for (size_t leg = 0; leg < kNumLegs; ++leg)
        {
            online_reference_foothold_world_[leg] = basePositionWorld + R_base_to_world * online_nominal_foot_pos_base_[leg];
        }
        online_reference_initialized_ = true;
    }

    std::array<vector3_t, 4> GaitManager::getReferenceFootPositionsBaseFrame(const vector_t& currentState)
    {
        std::array<vector3_t, 4> footPositionsBase{};
        footPositionsBase.fill(vector3_t::Zero());
        vector3_t basePositionWorld = vector3_t::Zero();
        scalar_t yaw = 0.0;
        if (!getBaseYawPose(currentState, basePositionWorld, yaw))
        {
            return footPositionsBase;
        }

        vector3_t zyx = vector3_t::Zero();
        zyx.x() = yaw;
        const matrix3_t R_base_to_world = getRotationMatrixFromZyxEulerAngles(zyx);
        const matrix3_t R_world_to_base = R_base_to_world.transpose();
        for (size_t leg = 0; leg < kNumLegs; ++leg)
        {
            if (online_reference_foothold_world_[leg].allFinite())
            {
                footPositionsBase[leg] = R_world_to_base * (online_reference_foothold_world_[leg] - basePositionWorld);
            }
        }
        return footPositionsBase;
    }

    ModeSequenceTemplate GaitManager::makeTrotRecoveryTemplate() const
    {
        const scalar_t swingDuration = kinematic_online_gait_config_.swingDuration;
        const scalar_t preStance = kinematic_online_gait_config_.preStanceDuration;
        const scalar_t interStance = kinematic_online_gait_config_.interStanceDuration;
        const scalar_t stanceAfterSwing = kinematic_online_gait_config_.stanceAfterSwing;
        return {{0.0,
                 preStance,
                 preStance + swingDuration,
                 preStance + swingDuration + interStance,
                 preStance + 2.0 * swingDuration + interStance,
                 preStance + 2.0 * swingDuration + interStance + stanceAfterSwing},
                {STANCE, LF_RH, STANCE, RF_LH, STANCE}};
    }

    void GaitManager::switchToFixedGait(const size_t gaitIndex, const int command)
    {
        if (gaitIndex >= gait_list_.size() || gaitIndex >= gait_name_list_.size())
        {
            RCLCPP_WARN(rclcpp::get_logger("GaitManager"), "Ignore gait command %d: gait index %zu is out of range.",
                        command, gaitIndex);
            return;
        }

        last_command_ = command;
        target_gait_ = gait_list_[gaitIndex];
        RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Switch to gait: %s", gait_name_list_[gaitIndex].c_str());
        gait_updated_ = true;
    }
}

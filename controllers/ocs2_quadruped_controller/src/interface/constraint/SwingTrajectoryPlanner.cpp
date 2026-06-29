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

// Boost属性树库：用于解析配置文件（.info格式）
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

// 摆动轨迹规划器头文件
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h"


#include <iostream>
// OCS2核心工具：数据加载、时间索引查找
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Lookup.h>

// OCS2legged_robot库：步态和运动阶段定义
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

// 命名空间：ocs2的legged_robot模块，封装四足机器人相关控制逻辑
namespace ocs2::legged_robot {

/**
 * @brief 摆动轨迹规划器构造函数
 * @param config [in] 摆动轨迹配置参数（离地速度、触地速度、摆动高度、时间缩放等）
 * @param numFeet [in] 机器人脚的数量（四足机器人通常为4）
 * @note 使用std::move优化配置参数的传递，避免拷贝开销
 */
SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, size_t numFeet) 
    : config_(std::move(config)),  // 移动语义保存配置参数
      numFeet_(numFeet) {          // 保存脚的数量
}

/**
 * @brief 获取指定腿在指定时间的Z轴速度约束值
 * @param leg [in] 腿的索引（0-3对应四足机器人四条腿）
 * @param time [in] 目标时间戳
 * @return 指定时间该腿的Z轴速度约束值
 * @note 通过查找时间索引，从预先生成的三次样条轨迹中获取速度值
 */
scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const {
    // 查找当前时间在该腿轨迹事件时间数组中的索引
    const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
    // 从三次样条轨迹中获取指定时间的速度值
    return feetHeightTrajectories_[leg][index].velocity(time);
}

/**
 * @brief 获取指定腿在指定时间的Z轴位置约束值
 * @param leg [in] 腿的索引（0-3对应四足机器人四条腿）
 * @param time [in] 目标时间戳
 * @return 指定时间该腿的Z轴位置约束值
 * @note 通过查找时间索引，从预先生成的三次样条轨迹中获取位置值
 */
scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const {
    // 查找当前时间在该腿轨迹事件时间数组中的索引
    const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
    // 从三次样条轨迹中获取指定时间的位置值
    return feetHeightTrajectories_[leg][index].position(time);
}

/**
 * @brief 重载更新函数：基于地形高度更新摆动轨迹（简化版）
 * @param modeSchedule [in] 模式调度表（包含各阶段的模式ID和事件时间）
 * @param terrainHeight [in] 地形高度（统一设置所有腿的离地/触地高度）
 * @note 该函数将地形高度扩展为数组，调用更通用的update重载版本
 */
void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule, scalar_t terrainHeight) {
    // 创建与模式序列长度相同的地形高度数组（所有阶段高度相同）
    const scalar_array_t terrainHeightSequence(modeSchedule.modeSequence.size(), terrainHeight);
    // 初始化所有腿的离地高度序列为地形高度
    feet_array_t<scalar_array_t> liftOffHeightSequence;
    liftOffHeightSequence.fill(terrainHeightSequence);
    // 初始化所有腿的触地高度序列为地形高度
    feet_array_t<scalar_array_t> touchDownHeightSequence;
    touchDownHeightSequence.fill(terrainHeightSequence);
    // 调用通用更新函数
    update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}

/**
 * @brief 重载更新函数：基于离地/触地高度更新摆动轨迹
 * @param modeSchedule [in] 模式调度表（包含各阶段的模式ID和事件时间）
 * @param liftOffHeightSequence [in] 各腿在各阶段的离地高度序列
 * @param touchDownHeightSequence [in] 各腿在各阶段的触地高度序列
 * @note 计算各腿的最大高度序列（离地/触地高度的最大值），调用最终更新函数
 */
void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule,
                                    const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                    const feet_array_t<scalar_array_t> &touchDownHeightSequence) {
    // 临时数组：存储单条腿的高度序列
    scalar_array_t heightSequence(modeSchedule.modeSequence.size());
    // 所有腿的最大高度序列（离地/触地高度的最大值）
    feet_array_t<scalar_array_t> maxHeightSequence;

    // 遍历每条腿
    for (size_t j = 0; j < numFeet_; j++) {
        // 遍历每个阶段
        for (size_t p = 0; p < modeSchedule.modeSequence.size(); ++p) {
            // 计算当前阶段的最大高度（离地和触地高度的最大值）
            heightSequence[p] = std::max(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]);

            // // 调试代码：打印当前脚的离地高度和触地高度
            // std::cout << "脚索引: " << j 
            //       << ", 阶段索引: " << p 
            //       << ", 离地高度: " << liftOffHeightSequence[j][p] 
            //       << ", 触地高度: " << touchDownHeightSequence[j][p] << std::endl;
        }
        // 保存当前腿的最大高度序列
        maxHeightSequence[j] = heightSequence;
    }

    // 调用最终的更新函数（传入离地、触地、最大高度序列）
    update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);
}

/**
 * @brief 核心更新函数：生成各腿的摆动高度轨迹（三次样条）
 * @param modeSchedule [in] 模式调度表（包含各阶段的模式ID和事件时间）
 * @param liftOffHeightSequence [in] 各腿在各阶段的离地高度序列
 * @param touchDownHeightSequence [in] 各腿在各阶段的触地高度序列
 * @param maxHeightSequence [in] 各腿在各阶段的最大高度序列
 * @note 核心逻辑：为每条腿的每个阶段生成三次样条轨迹（摆动腿生成摆动轨迹，支撑腿生成固定高度轨迹）
 */
void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule,
                                    const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                    const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                                    const feet_array_t<scalar_array_t> &maxHeightSequence) {
    // 提取模式序列（各阶段的模式ID）和事件时间（阶段切换时间）
    const auto &modeSequence = modeSchedule.modeSequence;
    const auto &eventTimes = modeSchedule.eventTimes;

    // 从模式序列中提取每条腿的接触标志（true=支撑，false=摆动）
    const auto eesContactFlagStocks = extractContactFlags(modeSequence);

    // 存储每条腿各阶段的摆动起始/结束时间索引
    feet_array_t<std::vector<int> > startTimesIndices;
    feet_array_t<std::vector<int> > finalTimesIndices;

    // 为每条腿更新摆动时间调度（计算各阶段的起始/结束时间索引）
    for (size_t leg = 0; leg < numFeet_; leg++) {
        std::tie(startTimesIndices[leg], finalTimesIndices[leg]) =
                updateFootSchedule(eesContactFlagStocks[leg]);
    }

    // 为每条腿生成高度轨迹
    for (size_t j = 0; j < numFeet_; j++) {
        // 清空当前腿的旧轨迹
        feetHeightTrajectories_[j].clear();
        // 预分配内存（避免频繁扩容）
        feetHeightTrajectories_[j].reserve(modeSequence.size());

        // 遍历每个阶段
        for (size_t p = 0; p < modeSequence.size(); ++p) {
            // 判断当前阶段该腿是否为摆动腿（!contactFlag=true）
            if (!eesContactFlagStocks[j][p]) {
                // ---------------- 摆动腿：生成三次样条摆动轨迹 ----------------
                // 获取当前阶段的摆动起始/结束时间索引
                const int swingStartIndex = startTimesIndices[j][p];
                const int swingFinalIndex = finalTimesIndices[j][p];
                // 检查索引有效性（避免越界）
                checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

                // 获取摆动起始/结束时间（事件时间数组中的对应值）
                const scalar_t swingStartTime = eventTimes[swingStartIndex];
                const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

                // 计算摆动轨迹缩放因子（根据实际摆动时间调整速度/高度）
                const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime,
                                                                config_.swingTimeScale);

                // 定义离地节点（时间、高度、速度）
                const CubicSpline::Node liftOff{
                    swingStartTime,                  // 离地时间
                    liftOffHeightSequence[j][p],     // 离地高度
                    scaling * config_.liftOffVelocity  // 离地速度（缩放后）
                };

                // 定义触地节点（时间、高度、速度）
                const CubicSpline::Node touchDown{
                    swingFinalTime,                  // 触地时间
                    touchDownHeightSequence[j][p],   // 触地高度
                    scaling * config_.touchDownVelocity  // 触地速度（缩放后）
                };
                //修改开始--------------------------------------------------------------------------
                // 计算当前这一步的高度变化：目标落点比起点高多少
                const scalar_t stepUp =
                    std::max<scalar_t>(touchDownHeightSequence[j][p] - liftOffHeightSequence[j][p], 0.0);

                // 如果是下台阶，保留一个较小的安全余量（可选）
                const scalar_t stepDown =
                    std::max<scalar_t>(liftOffHeightSequence[j][p] - touchDownHeightSequence[j][p], 0.0);

                // 上台阶时额外抬高；下台阶只给一点点保险余量
                const scalar_t extraClearance =
                    0.8 * stepUp + 0.15 * stepDown;

                // 限制额外抬高，避免异常高度把摆腿抬得过分夸张
                const scalar_t extraClearanceClamped =
                    std::clamp<scalar_t>(extraClearance, 0.0, 0.18);

                // 计算摆动中点高度：基础摆高 + 台阶额外摆高
                const scalar_t midHeight =
                    maxHeightSequence[j][p] + scaling * config_.swingHeight + extraClearanceClamped;
                //修改结束--------------------------------------------------------------------------

                // 生成三次样条轨迹并添加到当前腿的轨迹列表
                feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

            } else {
                // ---------------- 支撑腿：生成固定高度轨迹 ----------------
                // 注意：此处时间设为0.0->1.0是占位符（支撑腿轨迹不参与实际计算）
                // 若设为相同时间会导致三次样条断言失败，因此设为0.0和1.0
                const CubicSpline::Node liftOff{0.0, liftOffHeightSequence[j][p], 0.0};
                const CubicSpline::Node touchDown{1.0, liftOffHeightSequence[j][p], 0.0};
                // 生成固定高度的三次样条（位置不变，速度为0）
                feetHeightTrajectories_[j].emplace_back(liftOff, liftOffHeightSequence[j][p], touchDown);
            }
        }
        // 保存当前腿的事件时间数组（用于后续时间索引查找）
        feetHeightTrajectoriesEvents_[j] = eventTimes;
    }
}

/**
 * @brief 更新单条腿的摆动时间调度（计算各阶段的起始/结束时间索引）
 * @param contactFlagStock [in] 该腿各阶段的接触标志（true=支撑，false=摆动）
 * @return 包含各阶段起始时间索引和结束时间索引的pair
 * @note 仅处理摆动腿阶段，支撑腿阶段返回{0,0}
 */
std::pair<std::vector<int>, std::vector<int> > SwingTrajectoryPlanner::updateFootSchedule(
    const std::vector<bool> &contactFlagStock) {
    // 阶段总数
    const size_t numPhases = contactFlagStock.size();

    // 初始化起始/结束时间索引数组（默认值0）
    std::vector<int> startTimeIndexStock(numPhases, 0);
    std::vector<int> finalTimeIndexStock(numPhases, 0);

    // 遍历所有阶段，为摆动腿计算起始/结束时间索引
    for (size_t i = 0; i < numPhases; i++) {
        if (!contactFlagStock[i]) {  // 仅处理摆动腿阶段
            std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
        }
    }
    return {startTimeIndexStock, finalTimeIndexStock};
}

/**
 * @brief 从模式序列中提取每条腿的接触标志
 * @param phaseIDsStock [in] 各阶段的模式ID（编码了各腿的接触状态）
 * @return 每条腿各阶段的接触标志数组（true=支撑，false=摆动）
 * @note 调用modeNumber2StanceLeg将模式ID转换为接触标志数组
 */
feet_array_t<std::vector<bool> > SwingTrajectoryPlanner::extractContactFlags(
    const std::vector<size_t> &phaseIDsStock) const {
    // 阶段总数
    const size_t numPhases = phaseIDsStock.size();

    // 初始化接触标志数组：每条腿对应一个bool数组（长度=阶段数）
    feet_array_t<std::vector<bool> > contactFlagStock;
    std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

    // 遍历每个阶段
    for (size_t i = 0; i < numPhases; i++) {
        // 将模式ID转换为各腿的接触标志（bool数组）
        const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
        // 为每条腿保存当前阶段的接触标志
        for (size_t j = 0; j < numFeet_; j++) {
            contactFlagStock[j][i] = contactFlag[j];
        }
    }
    return contactFlagStock;
}

/**
 * @brief 查找摆动腿阶段的起始/结束时间索引
 * @param index [in] 当前摆动阶段的索引
 * @param contactFlagStock [in] 该腿各阶段的接触标志
 * @return 起始时间索引（离地前最后一个支撑阶段）和结束时间索引（触地前最后一个摆动阶段）
 * @note 向前查找起始索引（最近的支撑阶段），向后查找结束索引（最近的支撑阶段前一阶段）
 */
std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool> &contactFlagStock) {
    // 阶段总数
    const size_t numPhases = contactFlagStock.size();

    // 支撑腿阶段直接返回{0,0}（无需计算）
    if (contactFlagStock[index]) {
        return {0, 0};
    }

    // ---------------- 查找起始时间索引（离地时间） ----------------
    int startTimesIndex = -1;
    // 从当前阶段向前遍历，找到第一个支撑阶段
    for (int ip = index - 1; ip >= 0; ip--) {
        if (contactFlagStock[ip]) {
            startTimesIndex = ip;
            break;
        }
    }

    // ---------------- 查找结束时间索引（触地时间） ----------------
    int finalTimesIndex = numPhases - 1;
    // 从当前阶段向后遍历，找到第一个支撑阶段
    for (size_t ip = index + 1; ip < numPhases; ip++) {
        if (contactFlagStock[ip]) {
            finalTimesIndex = ip - 1;  // 结束索引为支撑阶段前一阶段
            break;
        }
    }

    return {startTimesIndex, finalTimesIndex};
}

/**
 * @brief 检查摆动轨迹的起始/结束索引是否有效
 * @param leg [in] 腿的索引
 * @param index [in] 当前阶段索引
 * @param startIndex [in] 起始时间索引
 * @param finalIndex [in] 结束时间索引
 * @param phaseIDsStock [in] 模式ID序列
 * @note 索引无效时抛出运行时异常，并打印调试信息
 */
void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t> &phaseIDsStock) {
    const size_t numSubsystems = phaseIDsStock.size();

    // 检查起始索引是否有效（<0表示未找到起始支撑阶段）
    if (startIndex < 0) {
        // 打印调试信息：当前阶段、所有阶段的模式ID
        std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
        for (size_t i = 0; i < numSubsystems; i++) {
            std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
        }
        std::cerr << std::endl;

        // 抛出异常：离地时间未定义
        throw std::runtime_error(
            "The time of take-off for the first swing of the EE with ID " + std::to_string(leg) +
            " is not defined.");
    }

    // 检查结束索引是否有效（>=阶段数-1表示未找到结束支撑阶段）
    if (finalIndex >= numSubsystems - 1) {
        // 打印调试信息：当前阶段、所有阶段的模式ID
        std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
        for (size_t i = 0; i < numSubsystems; i++) {
            std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
        }
        std::cerr << std::endl;

        // 抛出异常：触地时间未定义
        throw std::runtime_error(
            "The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) +
            " is not defined.");
    }
}

/**
 * @brief 计算摆动轨迹的缩放因子
 * @param startTime [in] 摆动起始时间
 * @param finalTime [in] 摆动结束时间
 * @param swingTimeScale [in] 参考摆动时间（配置参数）
 * @return 缩放因子（≤1.0）
 * @note 缩放因子 = 实际摆动时间 / 参考摆动时间，最大值为1.0（避免过度缩放）
 */
scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime,
                                                        scalar_t swingTimeScale) {
    // 计算实际摆动时间与参考时间的比值，最大值限制为1.0
    return std::min(1.0, (finalTime - startTime) / swingTimeScale);
}

/**
 * @brief 从配置文件加载摆动轨迹参数
 * @param fileName [in] 配置文件路径（.info格式）
 * @param fieldName [in] 配置参数的根节点名称
 * @param verbose [in] 是否打印加载信息（true=打印，false=静默）
 * @return 加载后的摆动轨迹配置结构体
 * @note 使用Boost.PropertyTree解析配置文件，支持离地速度、触地速度、摆动高度、时间缩放等参数
 */
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string &fileName,
                                                           const std::string &fieldName, bool verbose) {
    // 初始化Boost属性树
    boost::property_tree::ptree pt;
    // 读取.info配置文件
    read_info(fileName, pt);

    // 打印配置加载信息（verbose=true时）
    if (verbose) {
        std::cerr << "\n #### Swing Trajectory Config:";
        std::cerr << "\n #### =============================================================================\n";
    }

    // 初始化配置结构体
    SwingTrajectoryPlanner::Config config;
    // 配置参数的前缀（根节点名称 + 点）
    const std::string prefix = fieldName + ".";

    // 从属性树加载配置参数
    loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);  // 离地速度
    loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);  // 触地速度
    loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);  // 摆动高度
    loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);  // 摆动时间缩放

    // 打印配置加载结束信息（verbose=true时）
    if (verbose) {
        std::cerr << " #### =============================================================================" <<
                std::endl;
    }

    // 返回加载后的配置
    return config;
}

} // namespace ocs2::legged_robot

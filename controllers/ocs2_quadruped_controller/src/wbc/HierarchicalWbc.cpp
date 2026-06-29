//
// Created by qiayuan on 22-12-23.
//
#include "ocs2_quadruped_controller/wbc/HierarchicalWbc.h"
#include "ocs2_quadruped_controller/wbc/HoQp.h"

// 命名空间：ocs2开源控制库的legged_robot模块             浮基动力学方程的核心是「世界坐标系下的牛顿 - 欧拉方程」，所有力 / 加速度都以世界坐标系为基准，因此求解出的足端接触力必然是世界坐标系下的数值；
namespace ocs2::legged_robot
{
    /**
     * @brief 分层式全身体控制器（Hierarchical WBC）的核心更新函数
     * @details 该函数是分层WBC的核心实现，通过构建不同优先级的控制任务，
     *          并使用分层二次规划（HQP）求解器得到最优的控制输入
     * 
     * @param stateDesired 期望的机器人状态（如基座位姿、关节角度等）
     * @param inputDesired 期望的控制输入（如关节力矩、接触力等）
     * @param rbdStateMeasured 实测的机器人刚体动力学状态（来自传感器）
     * @param mode 机器人的运动模式（如行走、小跑、站立等）
     * @param period 控制周期（单位：秒）
     * @return vector_t 求解得到的最优控制输入（如关节力矩）
     */
    vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                                     const vector_t& rbdStateMeasured, size_t mode,
                                     scalar_t period)
    {
        // 调用基类的update函数，完成基础的状态更新和初始化
        WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

        // ====================== 构建不同优先级的控制任务 ======================
        // 0级任务（最高优先级）：基础约束任务
        // 包含：浮基动力学方程约束 + 力矩极限约束 + 摩擦锥约束 + 无接触运动约束
        // 优先级最高，必须优先满足，保障机器人的物理可行性
        Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
            formulateWheelRollingMotionTask();

        // 1级任务（中优先级）：运动跟踪任务
        // 包含：基座加速度跟踪任务 + 摆动腿轨迹跟踪任务
        // 在满足0级约束的前提下，尽可能跟踪期望的运动轨迹
        Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();

        // 2级任务（最低优先级）：接触力优化任务
        // 优化接触力分布，在满足高优先级任务的前提下，使接触力更合理
        Task task2 = formulateContactForceTask(inputDesired);

        // ====================== 构建分层二次规划求解器 ======================
        // 构建分层HQP求解器，任务优先级：task0 > task1 > task2
        // 使用嵌套的方式构建分层结构：task2依赖task1的解，task1依赖task0的解
        HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

        // 求解HQP问题，返回最优的控制输入（如关节力矩）
        return hoQp.getSolutions();
    }
} // namespace legged

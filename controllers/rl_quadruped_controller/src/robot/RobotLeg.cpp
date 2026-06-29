//
// Created by biao on 24-9-12.
//

#include <memory>
#include "rl_quadruped_controller/robot/RobotLeg.h"

// 构造函数，初始化机器人的腿部链条和求解器
RobotLeg::RobotLeg(const KDL::Chain &chain) {
    chain_ = chain;

    // 创建前向运动学求解器，用于计算末端执行器的位姿
    fk_pose_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
    
    // 创建逆向运动学求解器，用于根据目标位姿计算关节角度
    ik_pose_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain);
    
    // 创建雅可比矩阵求解器，用于计算关节的速度和力矩
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain);
}

// 计算末端执行器相对于基坐标系的位姿
KDL::Frame RobotLeg::calcPEe2B(const KDL::JntArray &joint_positions) const {
    KDL::Frame pEe;
    // 使用前向运动学求解器计算位姿
    fk_pose_solver_->JntToCart(joint_positions, pEe);
    return pEe;
}

// 根据末端执行器的位姿和初始关节角度计算目标关节角度
KDL::JntArray RobotLeg::calcQ(const KDL::Frame &pEe, const KDL::JntArray &q_init) const {
    KDL::JntArray q(chain_.getNrOfJoints());
    // 使用逆向运动学求解器计算关节角度
    ik_pose_solver_->CartToJnt(q_init, pEe, q);
    return q;
}

// 计算雅可比矩阵，表示关节速度和末端执行器速度之间的关系
KDL::Jacobian RobotLeg::calcJaco(const KDL::JntArray &joint_positions) const {
    KDL::Jacobian jacobian(chain_.getNrOfJoints());
    // 使用雅可比矩阵求解器计算雅可比矩阵
    jac_solver_->JntToJac(joint_positions, jacobian);
    return jacobian;
}

// 计算基于末端执行器力和雅可比矩阵的关节力矩
KDL::JntArray RobotLeg::calcTorque(const KDL::JntArray &joint_positions, const Vec3 &force) const {
    // 计算雅可比矩阵，并提取前三行（对应力）
    const Eigen::Matrix<double, 3, Eigen::Dynamic> jacobian = calcJaco(joint_positions).data.topRows(3);
    
    // 使用雅可比矩阵计算力矩
    Eigen::VectorXd torque_eigen = jacobian.transpose() * force;
    
    // 将力矩从Eigen格式转换为KDL的JntArray格式
    KDL::JntArray torque(chain_.getNrOfJoints());
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        torque(i) = torque_eigen(i);
    }
    return torque;
}
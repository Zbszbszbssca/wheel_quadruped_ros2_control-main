//
// Created by qiayuan on 2021/11/15.
//

// 状态估计器基类，定义四足机器人状态估计的通用接口和基础功能
#include "ocs2_quadruped_controller/estimator/StateEstimateBase.h"

// OCS2质心模型工具：提供质心模型相关工厂函数
#include <ocs2_centroidal_model/FactoryFunctions.h>
// OCS2四足机器人通用类型定义
#include <ocs2_legged_robot/common/Types.h>
// OCS2机器人工具：提供旋转导数变换（欧拉角/角速度转换）
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

// 标准库头文件：智能指针和移动语义
#include <memory>
#include <utility>

// 命名空间：ocs2的legged_robot模块
namespace ocs2::legged_robot
{
    /**
     * @brief StateEstimateBase构造函数
     * @param info: 质心模型信息，包含广义坐标数量、驱动自由度等关键参数
     * @param ctrl_component: 控制器接口集合，用于获取传感器和硬件状态
     * @param node: ROS2生命周期节点指针，用于参数声明、话题发布等
     * @details 初始化状态估计器基类：
     * 1. 保存控制器接口、模型信息和节点指针
     * 2. 初始化刚体动力学(RBD)状态向量
     * 3. 声明并加载足端力阈值参数（用于判断接触状态）
     */
    StateEstimateBase::StateEstimateBase(CentroidalModelInfo info,
                                         CtrlInterfaces& ctrl_component,
                                         rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ctrl_component_(ctrl_component),                     // 保存控制器接口引用
          info_(std::move(info)),                              // 转移质心模型信息所有权
          rbd_state_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)),  // 初始化RBD状态（2倍广义坐标数：位置+速度）
          node_(std::move(node))                               // 转移节点指针所有权
    {
        // 声明ROS2参数：足端接触力阈值（用于判断是否着地）
        node_->declare_parameter("feet_force_threshold", feet_force_threshold_);
        // 从参数服务器加载足端力阈值
        feet_force_threshold_ = node_->get_parameter("feet_force_threshold").as_double();
    }

    /**
     * @brief 更新关节状态
     * @details 从控制器硬件接口读取关节位置和速度，更新到RBD状态向量中：
     * 1. 读取所有关节的位置和速度
     * 2. 将关节位置写入RBD状态的广义坐标部分（索引6开始）
     * 3. 将关节速度写入RBD状态的广义速度部分（索引6+广义坐标数开始）
     */
    void StateEstimateBase::updateJointStates()
    {
        // 获取关节力接口的数量（等同于关节数量）
        const size_t size = ctrl_component_.joint_effort_state_interface_.size();
        // 初始化关节位置和速度向量
        vector_t joint_pos(size), joint_vel(size);

        // 遍历所有关节，读取位置和速度
        for (int i = 0; i < size; i++)
        {
            // 读取关节位置（从硬件接口的optional中取值）
            joint_pos(i) = ctrl_component_.joint_position_state_interface_[i].get().get_optional().value();
            // 读取关节速度（从硬件接口的optional中取值）
            joint_vel(i) = ctrl_component_.joint_velocity_state_interface_[i].get().get_optional().value();

            // 打印关节速度（使用DEBUG级别，避免影响正常日志输出）RCLCPP_DEBUG  RCLCPP_INFO
            RCLCPP_DEBUG(node_->get_logger(), 
                        "Joint %d: velocity = %.4f rad/s",  // 保留4位小数，单位rad/s（弧度/秒）
                        i,                                  // 关节索引
                        joint_vel(i));                      // 关节速度值
        }


        // 将关节位置写入RBD状态：广义坐标[6:6+驱动自由度]为关节位置
        rbd_state_.segment(6, info_.actuatedDofNum) = joint_pos;
        // 将关节速度写入RBD状态：广义速度[6+广义坐标数:6+广义坐标数+驱动自由度]为关节速度
        rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = joint_vel;
    }

    /**
     * @brief 更新足端接触状态
     * @details 基于足端力传感器数据判断每个足端是否与地面接触：
     * 1. 读取每个足端的力值
     * 2. 与力阈值比较，更新接触标志位
     * 3. （可选）打印接触状态调试日志
     */
    void StateEstimateBase::updateContact()
    {
        // 获取足端力接口的数量（等同于足端数量）
        const size_t size = ctrl_component_.foot_force_state_interface_.size();
        // 遍历所有足端，判断接触状态
        for (int i = 0; i < size; i++)
        {
            // 获取当前足部的力值（从硬件接口的optional中取值）
            const auto foot_force = ctrl_component_.foot_force_state_interface_[i].get().get_optional().value();
            // 判断接触状态：力值大于阈值则视为接触地面
            contact_flag_[i] = foot_force > feet_force_threshold_;
            
            // // 打印调试日志（注释掉，避免正常运行时日志冗余）
            // RCLCPP_DEBUG(node_->get_logger(), 
            //             "Foot %d: force = %.2f N, threshold = %.2f N, contact = %s",
            //             i,  // 足部索引
            //             foot_force,  // 当前力值
            //             feet_force_threshold_,  // 力阈值
            //             contact_flag_[i] ? "true" : "false");  // 接触状态
        }
    }

    /**
     * @brief 更新IMU数据并转换为全局坐标系
     * @details 核心IMU数据处理流程：
     * 1. 读取IMU原始数据（四元数、角速度、线加速度）
     * 2. 将四元数转换为ZYEuler角并减去偏移量
     * 3. 将机体坐标系角速度转换为全局坐标系角速度
     * 4. 更新机器人的角度和角速度到RBD状态
     */
    void StateEstimateBase::updateImu()
    {
        // 读取IMU四元数（w/x/y/z）：imu_state_interface[0-3]
        quat_ = {
            ctrl_component_.imu_state_interface_[0].get().get_optional().value(),
            ctrl_component_.imu_state_interface_[1].get().get_optional().value(),
            ctrl_component_.imu_state_interface_[2].get().get_optional().value(),
            ctrl_component_.imu_state_interface_[3].get().get_optional().value()
        };

        // 读取机体坐标系下的角速度：imu_state_interface[4-6]
        angular_vel_local_ = {
            ctrl_component_.imu_state_interface_[4].get().get_optional().value(),
            ctrl_component_.imu_state_interface_[5].get().get_optional().value(),
            ctrl_component_.imu_state_interface_[6].get().get_optional().value()
        };

        // 读取机体坐标系下的线加速度：imu_state_interface[7-9]
        linear_accel_local_ = {
            ctrl_component_.imu_state_interface_[7].get().get_optional().value(),
            ctrl_component_.imu_state_interface_[8].get().get_optional().value(),
            ctrl_component_.imu_state_interface_[9].get().get_optional().value()
        };

        // 注释：协方差数据暂未使用
        // orientationCovariance_ = orientationCovariance;
        // angularVelCovariance_ = angularVelCovariance;
        // linearAccelCovariance_ = linearAccelCovariance;

        // 四元数转ZYEuler角并减去偏移量（校准初始姿态）
        const vector3_t zyx = quatToZyx(quat_) - zyx_offset_;
        // 机体坐标系角速度转换为全局坐标系角速度：
        // 1. 先将机体角速度转换为欧拉角导数
        // 2. 再将欧拉角导数转换为全局坐标系角速度
        const vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat_), angular_vel_local_));

        // 更新角度和角速度到RBD状态
        updateAngular(zyx, angularVelGlobal);
    }

    /**
     * @brief 初始化ROS2发布器
     * @details 创建里程计和位姿话题发布器：
     * 1. /odom：发布机器人里程计信息（位置、速度、姿态）
     * 2. /pose：发布机器人位姿信息（带协方差）
     */
    void StateEstimateBase::initPublishers()
    {
        // 创建里程计发布器：话题名"odom"，队列大小10
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        // 创建位姿发布器：话题名"pose"，队列大小10
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
    }

    /**
     * @brief 更新机器人角度和角速度到RBD状态
     * @param zyx: ZY顺序欧拉角（roll/pitch/yaw）
     * @param angularVel: 全局坐标系下的角速度
     * @details 将角度写入RBD状态的位置部分，角速度写入速度部分
     */
    void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel)
    {
        // RBD状态[0:3]：欧拉角（姿态）
        rbd_state_.segment<3>(0) = zyx;
        // RBD状态[广义坐标数:广义坐标数+3]：全局角速度
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }

    /**
     * @brief 更新机器人位置和线速度到RBD状态
     * @param pos: 全局坐标系下的位置（x/y/z）
     * @param linearVel: 全局坐标系下的线速度
     * @details 将位置写入RBD状态的位置部分，线速度写入速度部分
     */
    void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel)
    {
        // RBD状态[3:6]：位置（x/y/z）
        rbd_state_.segment<3>(3) = pos;
        // RBD状态[广义坐标数+3:广义坐标数+6]：全局线速度
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    /**
     * @brief 发布里程计和位姿消息
     * @param odom: 待发布的里程计消息
     * @details 1. 发布里程计消息到/odom话题
     *          2. 从里程计消息提取位姿，发布到/pose话题
     */
    void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry& odom) const
    {
        // 获取消息时间戳
        rclcpp::Time time = odom.header.stamp;
        // 发布里程计消息
        odom_pub_->publish(odom);

        // 构造位姿消息
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header = odom.header;          // 复用里程计的header（时间戳、坐标系）
        pose.pose.pose = odom.pose.pose;    // 复用里程计的位姿数据
        // 发布位姿消息
        pose_pub_->publish(pose);
    }
} // namespace legged

//
// Created by biao on 3/15/25.
//
// 包含CtrlComponent类的头文件，该类是四足机器人控制器的核心组件
#include "ocs2_quadruped_controller/control/CtrlComponent.h"

// ROS2相关工具头文件：获取功能包路径
#include <ament_index_cpp/get_package_share_directory.hpp>
// 角度处理工具：提供角度归一化、最短角度差计算等功能
#include <angles/angles.h>
// OCS2核心工具：加载配置文件数据
#include <ocs2_core/misc/LoadData.h>
// OCS2线程工具：设置线程优先级
#include <ocs2_core/thread_support/SetThreadPriority.h>
// 状态估计器相关头文件（三种不同的估计器实现）
#include <ocs2_quadruped_controller/estimator/FromOdomTopic.h>
#include <ocs2_quadruped_controller/estimator/GroundTruth.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>

// 质心模型与机器人动力学转换工具
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
// OCS2线程工具：周期性执行任务并休眠
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
// OCS2可视化工具：四足机器人状态可视化
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
// 步态管理器：处理机器人步态调度
#include <ocs2_quadruped_controller/control/GaitManager.h>
// 感知相关接口：处理地形感知、足端放置等感知功能
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h>
// OCS2 SQP求解器：模型预测控制(MPC)的核心求解器
#include <ocs2_sqp/SqpMpc.h>


namespace ocs2::legged_robot
{
    /**
     * @brief CtrlComponent构造函数
     * @param node: ROS2生命周期节点指针，用于参数获取、日志输出、话题通信等
     * @param ctrl_interfaces: 控制器接口集合，包含硬件接口、通信接口等
     * @details 初始化控制器核心组件，加载配置参数，创建核心接口和可视化工具
     */
    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces) : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        // 声明并获取ROS2参数：机器人功能包名、足端名称、是否启用感知功能
        node_->declare_parameter("robot_pkg", robot_pkg_);
        node_->declare_parameter("feet", feet_names_);
        node_->declare_parameter("enable_perceptive", enable_perceptive_);

        // 从参数服务器读取配置值
        robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
        joint_names_ = node_->get_parameter("joints").as_string_array();
        feet_names_ = node_->get_parameter("feet").as_string_array();
        enable_perceptive_ = node_->get_parameter("enable_perceptive").as_bool();

        // 拼接配置文件路径：基于功能包路径构建URDF和OCS2配置文件路径
        const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
        urdf_file_ = package_share_directory + "/urdf/robot.urdf";          // 机器人URDF模型文件
        task_file_ = package_share_directory + "/config/ocs2/task.info";    // 控制任务配置文件
        reference_file_ = package_share_directory + "/config/ocs2/reference.info";  // 参考轨迹配置文件
        gait_file_ = package_share_directory + "/config/ocs2/gait.info";    // 步态配置文件

        // 从任务配置文件加载日志输出等级
        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        // 初始化核心组件：legged接口、末端执行器运动学、MPC控制器、MRT接口
        setupLeggedInterface();

        // 创建末端执行器运动学求解器：用于计算足端的位姿
        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        ee_kinematics_ = std::make_unique<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);

        setupMpc();
        setupMrt();

        // 创建质心模型与机器人动力学转换工具：实现两种模型状态的相互转换
        rbd_conversions_ = std::make_unique<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        // 初始化可视化器：用于在RViz中显示机器人状态
        visualizer_ = std::make_unique<LeggedRobotVisualizer>(
            legged_interface_->getPinocchioInterface(),
            legged_interface_->getCentroidalModelInfo(),
            *ee_kinematics_,
            node_);

        // 初始化观测数据结构：状态、输入、模式均初始化为0/默认值
        observation_.state.setZero(static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
        observation_.input.setZero(
            static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
        observation_.mode = STANCE;  // 默认初始模式为支撑相
    }

    /**
     * @brief 设置状态估计器
     * @param estimator_type: 估计器类型字符串（ground_truth/linear_kalman/默认使用odom）
     * @details 根据指定类型创建对应的状态估计器实例，用于获取机器人的状态（位姿、速度、关节状态等）
     */
    void CtrlComponent::setupStateEstimate(const std::string& estimator_type)
    {
        if (estimator_type == "ground_truth")
        {
            // 真值估计器：直接使用仿真/硬件提供的真值状态
            estimator_ = std::make_unique<GroundTruth>(legged_interface_->getCentroidalModelInfo(),
                                                       ctrl_interfaces_,
                                                       node_);
            RCLCPP_INFO(node_->get_logger(), "Using Ground Truth Estimator");
        }
        else if (estimator_type == "linear_kalman")
        {
            // 线性卡尔曼滤波估计器：融合多源传感器数据估计状态
            estimator_ = std::make_unique<KalmanFilterEstimate>(
                legged_interface_->getPinocchioInterface(),
                legged_interface_->getCentroidalModelInfo(),
                *ee_kinematics_, ctrl_interfaces_,
                node_, gaitSchedulePtr_); // 新增最后一个参数
            // 加载卡尔曼滤波器配置参数
            dynamic_cast<KalmanFilterEstimate&>(*estimator_).loadSettings(task_file_, verbose_);
            // ⭐ 新增这一行
            legged_interface_->getSwitchedModelReferenceManagerPtr()
                ->setKalmanFilter(dynamic_cast<KalmanFilterEstimate*>(estimator_.get()));
            RCLCPP_INFO(node_->get_logger(), "Using Kalman Filter Estimator");
        }
        else
        {
            // 默认使用里程计话题估计器：从ROS2里程计话题获取状态
            estimator_ = std::make_unique<FromOdomTopic>(
                legged_interface_->getCentroidalModelInfo(), ctrl_interfaces_, node_);
            RCLCPP_INFO(node_->get_logger(), "Using Odom Topic Based Estimator");
        }
        // 初始化观测时间戳
        observation_.time = 0;
    }

    /**
     * @brief 更新机器人状态
     * @param time: 当前ROS2时间戳
     * @param period: 两次更新的时间间隔
     * @details 核心状态更新函数：
     * 1. 更新状态估计
     * 2. 转换并修正状态（偏航角归一化）
     * 3. 更新可视化
     * 4. 更新目标轨迹
     * 5. 将观测值传递给MPC
     */
    void CtrlComponent::updateState(const rclcpp::Time& time, const rclcpp::Duration& period)

    {

        // 1. 更新状态估计：从估计器获取机器人的RBD（刚体动力学）状态  RBD即为状态估计后得到的全面的测量值
        measured_rbd_state_ = estimator_->update(observation_,time, period);
        // 更新观测时间戳
        observation_.time += period.seconds();
        
        // 2. 状态转换与偏航角修正：避免偏航角跳变（如从359°到0°）
        const scalar_t yaw_last = observation_.state(9);  // 获取上一时刻偏航角（state[9]对应偏航角）
        // 将RBD状态转换为质心模型状态，减少维度化简模型，便于分析
        observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        // 修正偏航角：使用最短角度差更新，保证角度连续性
        observation_.state(9) = yaw_last + angles::shortest_angular_distance(
            yaw_last, observation_.state(9));
        // 更新观测的模式（支撑相/摆动相）
        observation_.mode = estimator_->getMode();

        // 3. 更新可视化：在RViz中显示当前状态
        visualizer_->update(observation_);
        if (enable_perceptive_)
        {
            // 启用感知功能时，更新足端放置和球体可视化
            footPlacementVisualizationPtr_->update(observation_);
            sphereVisualizationPtr_->update(observation_);
        }

        // 4. 计算目标轨迹：根据当前观测更新参考轨迹
        target_manager_->update(observation_);
        // 5. 将当前观测设置到MPC-MRT接口，供MPC求解使用
        mpc_mrt_interface_->setCurrentObservation(observation_);
    }

    /**
     * @brief 初始化MPC控制器
     * @details 执行MPC的首次初始化：
     * 1. 设置初始目标轨迹
     * 2. 等待MPC求解出初始控制策略
     * 3. 标记MPC为运行状态
     */
    void CtrlComponent::init()
    {
        // 仅在MPC未运行时执行初始化
        if (mpc_running_ == false)
        {
            // 创建初始目标轨迹：以当前观测为初始点
            const TargetTrajectories target_trajectories({observation_.time},
                                                         {observation_.state},
                                                         {observation_.input});

            // 设置初始观测和目标轨迹，等待MPC完成首次优化
            mpc_mrt_interface_->setCurrentObservation(observation_);
            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
            
            // 循环等待，直到接收到初始控制策略
            while (!mpc_mrt_interface_->initialPolicyReceived())
            {
                mpc_mrt_interface_->advanceMpc();  // 推进MPC求解
                // 按照MPC期望频率休眠
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");

            // 标记MPC为运行状态
            mpc_running_ = true;
        }
    }

    /**
     * @brief 初始化legged接口
     * @details 根据是否启用感知功能，创建不同的legged接口实例：
     * 1. 普通接口：基础的四足机器人控制接口
     * 2. 感知接口：增加地形感知、足端放置优化等功能
     * 同时初始化关节/足端名称、最优控制问题和感知相关可视化
     */
    void CtrlComponent::setupLeggedInterface()
    {
        if (enable_perceptive_)
        {
            // 启用感知功能：创建感知型legged接口
            legged_interface_ = std::make_unique<PerceptiveLeggedInterface>(task_file_, urdf_file_, reference_file_);
        }
        else
        {
            // 禁用感知功能：创建基础型legged接口
            legged_interface_ = std::make_unique<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        }

        // 设置关节和足端名称：关联硬件接口与模型
        legged_interface_->setupJointNames(joint_names_, feet_names_);
        // 初始化最优控制问题：加载配置，构建优化目标和约束
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);

        if (enable_perceptive_)
        {
            // 初始化感知相关可视化：足端放置区域和球体可视化
            footPlacementVisualizationPtr_ = std::make_unique<FootPlacementVisualization>(
                *dynamic_cast<PerceptiveLeggedReferenceManager&>(*legged_interface_->getReferenceManagerPtr()).
                getConvexRegionSelectorPtr(),
                legged_interface_->getCentroidalModelInfo().numThreeDofContacts, node_);

            sphereVisualizationPtr_ = std::make_unique<SphereVisualization>(
                legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(),
                *dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPinocchioSphereInterfacePtr(), node_);
        }
    }

    /**
     * @brief 设置MPC控制器、步态管理器和参考管理器
     * @details 核心控制器初始化：
     * 1. 创建SQP MPC求解器
     * 2. 初始化步态管理器并加载步态配置
     * 3. 创建目标轨迹管理器
     * 4. 启用感知时，添加地形接收器同步模块
     */
    void CtrlComponent::setupMpc()
    {
        // 创建SQP MPC求解器实例：传入MPC配置、SQP配置、最优控制问题和初始化器
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(),
                                        legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());

        // 初始化步态管理器：加载步态配置，管理机器人步态切换
        const auto gait_manager_ptr = std::make_shared<GaitManager>(
            ctrl_interfaces_,
            legged_interface_->getSwitchedModelReferenceManagerPtr()->
                               getGaitSchedule(),
            legged_interface_->getPinocchioInterface(),
            *ee_kinematics_,
            legged_interface_->getCentroidalModelInfo());
        gait_manager_ptr->init(gait_file_);  // 加载步态配置文件

        // ✅ 新增：保存 GaitSchedule 指针供后续使用
        gaitSchedulePtr_ = legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule();
        // 将步态管理器添加为SQP求解器的同步模块
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr);
        // 设置参考管理器：用于提供MPC的参考轨迹
        mpc_->getSolverPtr()->setReferenceManager(legged_interface_->getReferenceManagerPtr());

        // 创建目标轨迹管理器：处理用户指令，生成目标轨迹
        target_manager_ = std::make_unique<TargetManager>(ctrl_interfaces_,
                                                          node_,
                                                          legged_interface_->getReferenceManagerPtr(),
                                                          task_file_,
                                                          reference_file_);

        if (enable_perceptive_)
        {
            // 启用感知功能：创建平面地形接收器，订阅地形话题并更新地形模型
            const auto planarTerrainReceiver =
                std::make_shared<PlanarTerrainReceiver>(
                    node_, dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPlanarTerrainPtr(),
                    dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getSignedDistanceFieldPtr(),
                    "/convex_plane_decomposition_ros/planar_terrain", "elevation");
            // 将地形接收器添加为SQP求解器的同步模块
            mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
        }
    }

    /**
     * @brief 设置MPC-MRT接口并启动MPC线程
     * @details MRT (Model Reference Tracking) 接口是MPC与控制器的桥梁：
     * 1. 创建MPC-MRT接口实例
     * 2. 初始化轨迹推演器
     * 3. 创建并启动MPC线程，周期性执行MPC求解
     * 4. 设置MPC线程优先级
     */
    void CtrlComponent::setupMrt()
    {
        // 创建MPC-MRT接口实例：连接MPC求解器和控制器
        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);
        // 初始化轨迹推演器：用于根据控制输入推演系统状态
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        // 重置MPC计时器：用于统计MPC求解耗时
        mpc_timer_.reset();

        // 标记控制器为运行状态，启动MPC线程
        controller_running_ = true;
        mpc_thread_ = std::thread([&]
        {
            // MPC线程主循环：周期性执行MPC求解
            while (controller_running_)
            {
                try
                {
                    // 周期性执行MPC推进函数（按照期望频率）
                    executeAndSleep(
                        [&]
                        {
                            if (mpc_running_)
                            {
                                mpc_timer_.startTimer();          // 开始计时
                                mpc_mrt_interface_->advanceMpc(); // 推进MPC求解（单次迭代）
                                mpc_timer_.endTimer();            // 结束计时，统计耗时
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);  // MPC期望执行频率
                }
                catch (const std::exception& e)
                {
                    // 捕获异常，停止控制器并输出错误日志
                    controller_running_ = false;
                    RCLCPP_WARN(node_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        // 设置MPC线程优先级：确保实时性
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(node_->get_logger(), "MRT initialized. MPC thread started.");
    }
}

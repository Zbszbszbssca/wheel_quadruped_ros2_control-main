//
// Created by biao on 24-9-9.
//

#include "hardware_unitree_sdk2/HardwareUnitree.h"
#include "hardware_unitree_sdk2/unitree_udp_wire.h"
#include "crc32.h"
#include <iomanip>
#include <sstream>
#include <string>
#include <cstring>
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using hardware_interface::return_type;


namespace
{

     std::string hexBufferToString(const void* data, std::size_t len, std::size_t bytes_per_line = 16)
    {
        const auto* bytes = reinterpret_cast<const uint8_t*>(data);
        std::ostringstream oss;
        oss << std::hex << std::setfill('0');

        for (std::size_t i = 0; i < len; ++i)
        {
            if (i % bytes_per_line == 0)
            {
                oss << "  ";
            }

            oss << std::setw(2) << static_cast<unsigned int>(bytes[i]) << " ";

            if ((i + 1) % bytes_per_line == 0 || i + 1 == len)
            {
                oss << '\n';
            }
        }

        return oss.str();
    }

    void toWireLowCmd(const unitree_go::msg::dds_::LowCmd_& in, LowCmdWire& out)
    {
        std::memset(&out, 0, sizeof(out));

        out.head[0] = in.head()[0];
        out.head[1] = in.head()[1];
        out.level_flag = in.level_flag();
        out.frame_reserve = 0;
        out.gpio = static_cast<uint32_t>(in.gpio());

        for (int i = 0; i < UNITREE_UDP_MOTOR_SLOTS; ++i)
        {
            out.motor_cmd[i].mode = in.motor_cmd()[i].mode();
            out.motor_cmd[i].q    = in.motor_cmd()[i].q();
            out.motor_cmd[i].dq   = in.motor_cmd()[i].dq();
            out.motor_cmd[i].tau  = in.motor_cmd()[i].tau();
            out.motor_cmd[i].kp   = in.motor_cmd()[i].kp();
            out.motor_cmd[i].kd   = in.motor_cmd()[i].kd();
        }
    }

    void fromWireLowState(const LowStateWire& in, unitree_go::msg::dds_::LowState_& out)
    {
        for (int i = 0; i < UNITREE_UDP_MOTOR_SLOTS; ++i)
        {
            out.motor_state()[i].q()       = in.motor_state[i].q;
            out.motor_state()[i].dq()      = in.motor_state[i].dq;
            out.motor_state()[i].tau_est() = in.motor_state[i].tau_est;
        }

        out.imu_state().quaternion()[0] = in.imu_state.quaternion[0];
        out.imu_state().quaternion()[1] = in.imu_state.quaternion[1];
        out.imu_state().quaternion()[2] = in.imu_state.quaternion[2];
        out.imu_state().quaternion()[3] = in.imu_state.quaternion[3];

        out.imu_state().gyroscope()[0] = in.imu_state.gyroscope[0];
        out.imu_state().gyroscope()[1] = in.imu_state.gyroscope[1];
        out.imu_state().gyroscope()[2] = in.imu_state.gyroscope[2];

        out.imu_state().accelerometer()[0] = in.imu_state.accelerometer[0];
        out.imu_state().accelerometer()[1] = in.imu_state.accelerometer[1];
        out.imu_state().accelerometer()[2] = in.imu_state.accelerometer[2];

        out.foot_force()[0] = in.foot_force[0];
        out.foot_force()[1] = in.foot_force[1];
        out.foot_force()[2] = in.foot_force[2];
        out.foot_force()[3] = in.foot_force[3];

        
    }
}



HardwareUnitree::~HardwareUnitree()
{
    stopUdp();
}

static inline bool str2bool(const std::string& s)
{
    return (s == "true" || s == "1" || s == "True" || s == "TRUE");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareUnitree::on_init(const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    joint_torque_command_.assign(16, 0);
    joint_position_command_.assign(16, 0);
    joint_velocities_command_.assign(16, 0);
    joint_kp_command_.assign(16, 0);
    joint_kd_command_.assign(16, 0);

    joint_position_.assign(16, 0);
    joint_velocities_.assign(16, 0);
    joint_effort_.assign(16, 0);

    imu_states_.assign(10, 0);
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);

    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }

    // ---- existing params ----
    if (const auto it = info.hardware_parameters.find("network_interface"); it != info.hardware_parameters.end())
    {
        network_interface_ = it->second;
    }
    if (const auto it = info.hardware_parameters.find("domain"); it != info.hardware_parameters.end())
    {
        domain_ = std::stoi(it->second);
    }
    if (const auto it = info.hardware_parameters.find("show_foot_force"); it != info.hardware_parameters.end())
    {
        show_foot_force_ = str2bool(it->second);
    }

    // ---- new transport params ----
    if (const auto it = info.hardware_parameters.find("enable_dds"); it != info.hardware_parameters.end())
    {
        enable_dds_ = str2bool(it->second);
    }
    if (const auto it = info.hardware_parameters.find("enable_udp"); it != info.hardware_parameters.end())
    {
        enable_udp_ = str2bool(it->second);
    }
    if (const auto it = info.hardware_parameters.find("udp_remote_ip"); it != info.hardware_parameters.end())
    {
        udp_remote_ip_ = it->second;
    }
    if (const auto it = info.hardware_parameters.find("udp_remote_port"); it != info.hardware_parameters.end())
    {
        udp_remote_port_ = std::stoi(it->second);
    }
    if (const auto it = info.hardware_parameters.find("udp_local_port"); it != info.hardware_parameters.end())
    {
        udp_local_port_ = std::stoi(it->second);
    }
    if (const auto it = info.hardware_parameters.find("udp_poll_timeout_ms"); it != info.hardware_parameters.end())
    {
        udp_poll_timeout_ms_ = std::stoi(it->second);
    }
    if (const auto it = info.hardware_parameters.find("udp_state_timeout_ms"); it != info.hardware_parameters.end())
    {
        udp_state_timeout_ms_ = std::stoi(it->second);
    }

    RCLCPP_INFO(get_logger(),
                "Transport: enable_dds=%s enable_udp=%s",
                enable_dds_ ? "true" : "false",
                enable_udp_ ? "true" : "false");

    // ---- DDS init (optional) ----
    if (enable_dds_)
    {
        RCLCPP_INFO(get_logger(), "DDS init: network_interface=%s domain=%d",
                    network_interface_.c_str(), domain_);

        ChannelFactory::Instance()->Init(domain_, network_interface_);

        low_cmd_publisher_ =
            std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
        low_cmd_publisher_->InitChannel();

        lows_tate_subscriber_ =
            std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
        lows_tate_subscriber_->InitChannel(
            [this](auto&& PH1)
            {
                lowStateMessageHandle(std::forward<decltype(PH1)>(PH1));
            },
            1);

        high_state_subscriber_ =
            std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(TOPIC_HIGHSTATE);
        high_state_subscriber_->InitChannel(
            [this](auto&& PH1)
            {
                highStateMessageHandle(std::forward<decltype(PH1)>(PH1));
            },
            1);
    }

    initLowCmd();

    // ---- UDP init (optional) ----
    if (enable_udp_)
    {
        if (!initUdp())
        {
            RCLCPP_ERROR(get_logger(), "UDP init failed (remote=%s:%d local_port=%d).",
                         udp_remote_ip_.c_str(), udp_remote_port_, udp_local_port_);
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "UDP init OK: send LowCmd -> %s:%d, bind local_port=%d (recv LowState).",
                    udp_remote_ip_.c_str(), udp_remote_port_, udp_local_port_);

        udp_last_state_tp_ = std::chrono::steady_clock::now();
        udp_rx_running_.store(true);
        udp_rx_thread_ = std::thread(&HardwareUnitree::udpRxLoop, this);
    }

    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareUnitree::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    // export imu sensor state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
    }

    // export foot force sensor state interface
    if (info_.sensors.size() > 1)
    {
        for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
        }
    }

    // export odometer state interface
    if (info_.sensors.size() > 2)
    {
        for (uint i = 0; i < info_.sensors[2].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[2].name, info_.sensors[2].state_interfaces[i].name, &high_states_[i]);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareUnitree::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }
    return command_interfaces;
}

return_type HardwareUnitree::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Copy to local to reduce lock time
    unitree_go::msg::dds_::LowState_ ls;
    unitree_go::msg::dds_::SportModeState_ hs;

    {
        std::lock_guard<std::mutex> lk(low_state_mtx_);
        ls = low_state_;
    }
    {
        std::lock_guard<std::mutex> lk(high_state_mtx_);
        hs = high_state_;
    }

    // joint states
    for (int i = 0; i < 16; ++i)
    {
        joint_position_[i]  = ls.motor_state()[i].q();
        joint_velocities_[i] = ls.motor_state()[i].dq();
        joint_effort_[i]    = ls.motor_state()[i].tau_est();
    }

    // imu states
    imu_states_[0] = ls.imu_state().quaternion()[0]; // w
    imu_states_[1] = ls.imu_state().quaternion()[1]; // x
    imu_states_[2] = ls.imu_state().quaternion()[2]; // y
    imu_states_[3] = ls.imu_state().quaternion()[3]; // z
    imu_states_[4] = ls.imu_state().gyroscope()[0];
    imu_states_[5] = ls.imu_state().gyroscope()[1];
    imu_states_[6] = ls.imu_state().gyroscope()[2];
    imu_states_[7] = ls.imu_state().accelerometer()[0];
    imu_states_[8] = ls.imu_state().accelerometer()[1];
    imu_states_[9] = ls.imu_state().accelerometer()[2];

    // contact states
    foot_force_[0] = ls.foot_force()[0];
    foot_force_[1] = ls.foot_force()[1];
    foot_force_[2] = ls.foot_force()[2];
    foot_force_[3] = ls.foot_force()[3];


    // // ===================== 打印读取到的所有信息 =====================
    // RCLCPP_INFO(get_logger(), "========================================");
    // RCLCPP_INFO(get_logger(), "HardwareUnitree read() 数据打印");
    // RCLCPP_INFO(get_logger(), "========================================");

    // // 打印关节状态（前4个方便调试，全部打印会刷屏）
    // RCLCPP_INFO(get_logger(), "[Joint 0] pos=%.3f vel=%.3f tau=%.3f",
    //             joint_position_[0], joint_velocities_[0], joint_effort_[0]);
    // RCLCPP_INFO(get_logger(), "[Joint 1] pos=%.3f vel=%.3f tau=%.3f",
    //             joint_position_[1], joint_velocities_[1], joint_effort_[1]);
    // RCLCPP_INFO(get_logger(), "[Joint 2] pos=%.3f vel=%.3f tau=%.3f",
    //             joint_position_[2], joint_velocities_[2], joint_effort_[2]);
    // RCLCPP_INFO(get_logger(), "[Joint 3] pos=%.3f vel=%.3f tau=%.3f",
    //             joint_position_[3], joint_velocities_[3], joint_effort_[3]);

    // ===================== 调试 IMU 数据（DEBUG 级别，默认不影响控制日志）=====================
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);


    RCLCPP_DEBUG_THROTTLE(
        get_logger(), steady_clock, 5000,
        "[Cmd Last 4 Joints] dq[12]=%.3f  dq[13]=%.3f  dq[14]=%.3f  dq[15]=%.3f",
        joint_velocities_[12],
        joint_velocities_[13],
        joint_velocities_[14],
        joint_velocities_[15]);

    RCLCPP_DEBUG_THROTTLE(
        get_logger(), steady_clock, 5000,
        "=== IMU 实时数据 ==="
    );
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), steady_clock, 5000,
        "四元数 W: %.3f  X: %.3f  Y: %.3f  Z: %.3f",
        imu_states_[0], imu_states_[1], imu_states_[2], imu_states_[3]
    );
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), steady_clock, 5000,
        "陀螺仪  X: %.3f  Y: %.3f  Z: %.3f  (rad/s)",
        imu_states_[4], imu_states_[5], imu_states_[6]
    );
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), steady_clock, 5000,
        "加速度 X: %.3f  Y: %.3f  Z: %.3f  (m/s²)\n",
        imu_states_[7], imu_states_[8], imu_states_[9]
    );
    // ==========================================================================

    // // 打印足力
    // RCLCPP_INFO(get_logger(), "[FootForce] FR=%.1f FL=%.1f RR=%.1f RL=%.1f",
    //             foot_force_[0], foot_force_[1], foot_force_[2], foot_force_[3]);

    // // 打印底盘高位状态
    // RCLCPP_INFO(get_logger(), "[HighState] position: %.3f %.3f %.3f",
    //             high_states_[0], high_states_[1], high_states_[2]);
    // RCLCPP_INFO(get_logger(), "[HighState] velocity: %.3f %.3f %.3f",
    //             high_states_[3], high_states_[4], high_states_[5]);

    // RCLCPP_INFO(get_logger(), "========================================\n");
    

    if (show_foot_force_)
    {
       // RCLCPP_INFO(get_logger(), "foot_force(): %f, %f, %f, %f",
                   // foot_force_[0], foot_force_[1], foot_force_[2], foot_force_[3]);
    }

    // high states (if DDS disabled and no source, they stay default zero)
    high_states_[0] = hs.position()[0];
    high_states_[1] = hs.position()[1];
    high_states_[2] = hs.position()[2];
    high_states_[3] = hs.velocity()[0];
    high_states_[4] = hs.velocity()[1];
    high_states_[5] = hs.velocity()[2];

    // UDP state freshness warning (throttle)
    if (enable_udp_)
    {
        const auto now = std::chrono::steady_clock::now();
        const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - udp_last_state_tp_).count();
        if (udp_state_received_.load() && dt_ms > udp_state_timeout_ms_)
        {
            // 这里建议后续做节流；先最少实现，不刷屏
            // RCLCPP_WARN(get_logger(), "UDP LowState stale: %ld ms", (long)dt_ms);
        }
    }

    return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // fill LowCmd from ros2_control commands
    for (int i = 0; i < 16; ++i)
    {
        low_cmd_.motor_cmd()[i].mode() = 0x01;
        low_cmd_.motor_cmd()[i].q()   = static_cast<float>(joint_position_command_[i]);
        low_cmd_.motor_cmd()[i].dq()  = static_cast<float>(joint_velocities_command_[i]);
        low_cmd_.motor_cmd()[i].kp()  = static_cast<float>(joint_kp_command_[i]);
        low_cmd_.motor_cmd()[i].kd()  = static_cast<float>(joint_kd_command_[i]);
        low_cmd_.motor_cmd()[i].tau() = static_cast<float>(joint_torque_command_[i]);
    }

    // ---------------- UDP wire packet ----------------
    // 先转成 UDP 线上协议结构体
    LowCmdWire wire_cmd{};
    toWireLowCmd(low_cmd_, wire_cmd);

    // UDP CRC：只给 UDP 包使用
    wire_cmd.crc = crc32_core(
        reinterpret_cast<const uint32_t*>(&wire_cmd),
        (sizeof(LowCmdWire) >> 2) - 1
    );


    // ====== 借鉴 read() 里的日志风格，但加 throttle 防刷屏 ======
    // static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    //     // 2) 新增：打印后四个电机（12、13、14、15）的速度指令 dq
    // RCLCPP_INFO_THROTTLE(
    //     get_logger(), steady_clock, 1000,
    //     "[Cmd Last 4 Joints] dq[12]=%.3f  dq[13]=%.3f  dq[14]=%.3f  dq[15]=%.3f",
    //     joint_velocities_command_[12],
    //     joint_velocities_command_[13],
    //     joint_velocities_command_[14],
    //     joint_velocities_command_[15]);


    // // 1) 先打印几个关节命令，风格和你 read() 里完全一致
    // RCLCPP_INFO_THROTTLE(
    //     get_logger(), steady_clock, 1000,
    //     "[Cmd Joint 0] q=%.3f dq=%.3f kp=%.3f kd=%.3f tau=%.3f",
    //     joint_position_command_[0], joint_velocities_command_[0],
    //     joint_kp_command_[0], joint_kd_command_[0], joint_torque_command_[0]);

    // RCLCPP_INFO_THROTTLE(
    //     get_logger(), steady_clock, 1000,
    //     "[Cmd Joint 1] q=%.3f dq=%.3f kp=%.3f kd=%.3f tau=%.3f",
    //     joint_position_command_[1], joint_velocities_command_[1],
    //     joint_kp_command_[1], joint_kd_command_[1], joint_torque_command_[1]);

    // // 2) 单独打印 CRC
    // RCLCPP_INFO_THROTTLE(
    //     get_logger(), steady_clock, 1000,
    //     "[UDP] packet_size=%zu crc=0x%08x",
    //     sizeof(LowCmdWire),
    //     static_cast<uint32_t>(wire_cmd.crc));

    // // 3) 打印整个 UDP 包的十六进制内容
    // RCLCPP_INFO_STREAM_THROTTLE(
    //     get_logger(), steady_clock, 1000,
    //     "[UDP] wire_cmd hex:\n" << hexBufferToString(&wire_cmd, sizeof(LowCmdWire)));
    // // =================================================


    // UDP send (optional)
    if (enable_udp_ && udp_sock_ >= 0)
    {
        const ssize_t n = ::sendto(
            udp_sock_,
            reinterpret_cast<const void*>(&wire_cmd),
            sizeof(LowCmdWire),
            0,
            reinterpret_cast<const sockaddr*>(&udp_remote_addr_),
            sizeof(udp_remote_addr_));

        (void)n;
    }

    // ---------------- DDS packet ----------------
    // DDS 必须按 DDS 对象自身重新计算 CRC
    if (enable_dds_ && low_cmd_publisher_)
    {
        low_cmd_.crc() = crc32_core(
            reinterpret_cast<const uint32_t*>(&low_cmd_),
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1
        );

        low_cmd_publisher_->Write(low_cmd_);
    }

    return return_type::OK;
}

void HardwareUnitree::initLowCmd()
{
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd_.motor_cmd()[i].mode() = 0x01; // servo mode
        low_cmd_.motor_cmd()[i].q() = 0;
        low_cmd_.motor_cmd()[i].kp() = 0;
        low_cmd_.motor_cmd()[i].dq() = 0;
        low_cmd_.motor_cmd()[i].kd() = 0;
        low_cmd_.motor_cmd()[i].tau() = 0;
    }
}

void HardwareUnitree::lowStateMessageHandle(const void* messages)
{
    const auto* p = static_cast<const unitree_go::msg::dds_::LowState_*>(messages);
    std::lock_guard<std::mutex> lk(low_state_mtx_);
    low_state_ = *p;
}

void HardwareUnitree::highStateMessageHandle(const void* messages)
{
    const auto* p = static_cast<const unitree_go::msg::dds_::SportModeState_*>(messages);
    std::lock_guard<std::mutex> lk(high_state_mtx_);
    high_state_ = *p;
}

bool HardwareUnitree::initUdp()
{
    udp_sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock_ < 0) return false;

    int yes = 1;
    ::setsockopt(udp_sock_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));


    // ==============================================
    // ✅ 强制绑定物理网卡 enp2s0（永远走有线）
    // ==============================================
    const char* iface = "enp2s0";
    if (setsockopt(udp_sock_, SOL_SOCKET, SO_BINDTODEVICE, iface, strlen(iface)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "Failed to bind to interface enp2s0");
        ::close(udp_sock_);
        udp_sock_ = -1;
        return false;
    }

    // bind local port (for receiving LowState and as source port for LowCmd)
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(udp_local_port_));
    local.sin_addr.s_addr = INADDR_ANY;

    if (::bind(udp_sock_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0)
    {
        ::close(udp_sock_);
        udp_sock_ = -1;
        return false;
    }

    // remote address for LowCmd
    udp_remote_addr_.sin_family = AF_INET;
    udp_remote_addr_.sin_port = htons(static_cast<uint16_t>(udp_remote_port_));
    if (::inet_pton(AF_INET, udp_remote_ip_.c_str(), &udp_remote_addr_.sin_addr) != 1)
    {
        ::close(udp_sock_);
        udp_sock_ = -1;
        return false;
    }

    return true;
}

void HardwareUnitree::stopUdp()
{
    udp_rx_running_.store(false);

    if (udp_sock_ >= 0)
    {
        ::shutdown(udp_sock_, SHUT_RDWR);
    }

    if (udp_rx_thread_.joinable())
    {
        udp_rx_thread_.join();
    }

    if (udp_sock_ >= 0)
    {
        ::close(udp_sock_);
        udp_sock_ = -1;
    }
}
void HardwareUnitree::udpRxLoop()
{
    const int fd = udp_sock_;
    if (fd < 0) return;

    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    while (udp_rx_running_.load())
    {
        const int pret = ::poll(&pfd, 1, udp_poll_timeout_ms_);
        if (pret <= 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        LowStateWire wire_state{};
        sockaddr_in src{};
        socklen_t slen = sizeof(src);

        const ssize_t n = ::recvfrom(fd, &wire_state, sizeof(wire_state), 0,
                                     reinterpret_cast<sockaddr*>(&src), &slen);
        
        // ========== 打印原始接收长度 ==========
        RCLCPP_INFO_THROTTLE(get_logger(), steady_clock, 1000,
                            "[UDP RX] 实际长度：%zd 字节 | 期望长度：%zu 字节",
                            n, sizeof(wire_state));

        if (n != (ssize_t)sizeof(wire_state))
        {
            udp_rx_size_fail_count_.fetch_add(1);
            // ========== 打印丢包数 ==========
            RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock, 1000,
                                 "[UDP 丢包] 累计丢包：%u",
                                 udp_rx_size_fail_count_.load());
            continue;
        }

        const uint32_t crc_calc = crc32_core(reinterpret_cast<const uint32_t*>(&wire_state),
                                             (sizeof(LowStateWire) >> 2) - 1);
        const uint32_t crc_rx = wire_state.crc;

        if (crc_calc != crc_rx)
        {
            udp_rx_crc_fail_count_.fetch_add(1);
            // ========== 打印校验错误数 ==========
            RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock, 1000,
                                 "[UDP 校验错误] 累计错误：%u",
                                 udp_rx_crc_fail_count_.load());
            continue;
        }

        unitree_go::msg::dds_::LowState_ tmp{};
        fromWireLowState(wire_state, tmp);

         // ====================== 打印UDP接收的数据 ======================
        // RCLCPP_INFO_STREAM_THROTTLE(
        //     get_logger(), steady_clock, 1000,
        //     "\n=== UDP 接收原始数据 (LowStateWire) ==="
        //     << hexBufferToString(&wire_state, sizeof(wire_state))
        // );
        // RCLCPP_INFO_THROTTLE(
        //     get_logger(), steady_clock, 1000,
        //     "[电机0] 位置=%.3f 速度=%.3f 扭矩=%.3f",
        //     tmp.motor_state()[0].q(), tmp.motor_state()[0].dq(), tmp.motor_state()[0].tau_est()
        // );
        // RCLCPP_INFO_THROTTLE(
        //     get_logger(), steady_clock, 1000,
        //     "[IMU] 四元数(wxyz)=%.3f %.3f %.3f %.3f",
        //     tmp.imu_state().quaternion()[0], tmp.imu_state().quaternion()[1],
        //     tmp.imu_state().quaternion()[2], tmp.imu_state().quaternion()[3]
        // );
        // ========== 打印汇总统计 ==========
        RCLCPP_INFO_THROTTLE(get_logger(), steady_clock, 1000,
                            "[UDP 统计] 正确接收：%u | 丢包：%u | 校验错误：%u",
                            udp_rx_ok_count_.load(),
                            udp_rx_size_fail_count_.load(),
                            udp_rx_crc_fail_count_.load());
        // ========================================================================

        {
            std::lock_guard<std::mutex> lk(low_state_mtx_);
            low_state_ = tmp;
        }

        udp_state_received_.store(true);
        udp_last_state_tp_ = std::chrono::steady_clock::now();
        udp_rx_ok_count_.fetch_add(1);
    }
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(HardwareUnitree, hardware_interface::SystemInterface)

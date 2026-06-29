//
// Created by biao on 24-9-9.
//

#ifndef HARDWAREUNITREE_H
#define HARDWAREUNITREE_H

#include "hardware_interface/system_interface.hpp"
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// POSIX UDP
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

class HardwareUnitree final : public hardware_interface::SystemInterface {
public:
    HardwareUnitree() = default;
    ~HardwareUnitree() override;

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    std::vector<double> imu_states_;
    std::vector<double> foot_force_;
    std::vector<double> high_states_;

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };

    void initLowCmd();

    void lowStateMessageHandle(const void *messages);
    void highStateMessageHandle(const void *messages);

    // ======== DDS data ========
    unitree_go::msg::dds_::LowCmd_ low_cmd_{};         // default init
    unitree_go::msg::dds_::LowState_ low_state_{};     // default init
    unitree_go::msg::dds_::SportModeState_ high_state_{}; // default init

    // Mutex for state shared by DDS callback and UDP RX thread
    std::mutex low_state_mtx_;
    std::mutex high_state_mtx_;

    // ======== Original DDS config ========
    std::string network_interface_ = "lo";
    int domain_ = 1;
    bool show_foot_force_ = false;

    // ======== Transport switches ========
    bool enable_dds_ = true;   // keep current behavior by default
    bool enable_udp_ = false;  // off by default

    // ======== UDP config ========
    std::string udp_remote_ip_ = "127.0.0.1";
    int udp_remote_port_ = 9000;   // send LowCmd to this port
    int udp_local_port_  = 9001;   // bind local port to receive LowState (also as source port for LowCmd)
    int udp_poll_timeout_ms_ = 5;  // RX poll timeout
    int udp_state_timeout_ms_ = 50; // if no state for this long -> warn (does not stop, just warn)

    int udp_sock_ = -1;
    sockaddr_in udp_remote_addr_{};

    std::atomic_bool udp_rx_running_{false};
    std::thread udp_rx_thread_;
    std::atomic_bool udp_state_received_{false};
    std::atomic<uint64_t> udp_rx_ok_count_{0};
    std::atomic<uint64_t> udp_rx_crc_fail_count_{0};
    std::atomic<uint64_t> udp_rx_size_fail_count_{0};
    std::chrono::steady_clock::time_point udp_last_state_tp_{};

    bool initUdp();
    void stopUdp();
    void udpRxLoop();

    // ======== DDS pub/sub ========
    /*publisher*/
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_publisher_;
    /*subscriber*/
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lows_tate_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_state_subscriber_;
};

#endif //HARDWAREUNITREE_H

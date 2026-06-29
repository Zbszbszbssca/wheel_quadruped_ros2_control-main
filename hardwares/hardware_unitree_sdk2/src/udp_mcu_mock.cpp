#include <cstdio>
#include <cstring>
#include <string>
#include <cstdint>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hardware_unitree_sdk2/unitree_udp_wire.h"
#include "crc32.h"

static int parse_int_arg(int argc, char** argv, const char* key, int default_val)
{
    for (int i = 1; i + 1 < argc; ++i)
    {
        if (std::string(argv[i]) == key)
        {
            return std::stoi(argv[i + 1]);
        }
    }
    return default_val;
}

int main(int argc, char** argv)
{
    // mock MCU：监听 mcu_port，接收 LowCmdWire，然后回 LowStateWire 到发送方源地址/源端口
    const int mcu_port = parse_int_arg(argc, argv, "--port", 9000);

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        std::perror("socket");
        return 1;
    }

    int yes = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(mcu_port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        std::perror("bind");
        ::close(sock);
        return 1;
    }

    std::printf("[udp_mcu_mock] Listening on 0.0.0.0:%d\n", mcu_port);
    std::printf("[udp_mcu_mock] Expect LowCmdWire size=%zu, reply LowStateWire size=%zu\n",
                sizeof(LowCmdWire), sizeof(LowStateWire));

    uint64_t ok = 0;
    uint64_t crc_fail = 0;
    uint64_t size_fail = 0;

    while (true)
    {
        LowCmdWire cmd{};
        sockaddr_in src{};
        socklen_t slen = sizeof(src);

        const ssize_t n = ::recvfrom(sock, &cmd, sizeof(cmd), 0,
                                     reinterpret_cast<sockaddr*>(&src), &slen);
        if (n != static_cast<ssize_t>(sizeof(cmd)))
        {
            size_fail++;
            continue;
        }

        const uint32_t crc_calc = crc32_core(
            reinterpret_cast<const uint32_t*>(&cmd),
            (sizeof(LowCmdWire) >> 2) - 1
        );
        const uint32_t crc_rx = cmd.crc;

        if (crc_calc != crc_rx)
        {
            crc_fail++;
            continue;
        }

        ok++;

        if ((ok % 200) == 0)
        {
            char ipbuf[INET_ADDRSTRLEN]{};
            ::inet_ntop(AF_INET, &src.sin_addr, ipbuf, sizeof(ipbuf));

            std::printf(
                "[udp_mcu_mock] ok=%lu crc_fail=%lu size_fail=%lu last_from=%s:%d cmd0.q=%.4f tau0=%.4f kp0=%.4f kd0=%.4f\n",
                static_cast<unsigned long>(ok),
                static_cast<unsigned long>(crc_fail),
                static_cast<unsigned long>(size_fail),
                ipbuf,
                ntohs(src.sin_port),
                cmd.motor_cmd[0].q,
                cmd.motor_cmd[0].tau,
                cmd.motor_cmd[0].kp,
                cmd.motor_cmd[0].kd
            );
        }

        // 构造回传状态
        LowStateWire st{};
        std::memset(&st, 0, sizeof(st));

        // 前16个关节状态跟随命令，便于验证 PC 侧 read()/joint state 更新
        for (int i = 0; i < UNITREE_UDP_ACTIVE_JOINTS; ++i)
        {
            st.motor_state[i].q       = cmd.motor_cmd[i].q;
            st.motor_state[i].dq      = cmd.motor_cmd[i].dq;
            st.motor_state[i].tau_est = cmd.motor_cmd[i].tau;
            st.motor_state[i].reserve = 0.0f;
        }

        // 剩余 4 个槽位清零
        for (int i = UNITREE_UDP_ACTIVE_JOINTS; i < UNITREE_UDP_MOTOR_SLOTS; ++i)
        {
            st.motor_state[i].q       = 0.0f;
            st.motor_state[i].dq      = 0.0f;
            st.motor_state[i].tau_est = 0.0f;
            st.motor_state[i].reserve = 0.0f;
        }

        // IMU 给一个固定合法值
        st.imu_state.quaternion[0] = 1.0f;
        st.imu_state.quaternion[1] = 0.0f;
        st.imu_state.quaternion[2] = 0.0f;
        st.imu_state.quaternion[3] = 0.0f;

        st.imu_state.gyroscope[0] = 0.0f;
        st.imu_state.gyroscope[1] = 0.0f;
        st.imu_state.gyroscope[2] = 0.0f;

        st.imu_state.accelerometer[0] = 0.0f;
        st.imu_state.accelerometer[1] = 0.0f;
        st.imu_state.accelerometer[2] = 9.81f;

        // 足端力给固定值
        st.foot_force[0] = 0.0f;
        st.foot_force[1] = 0.0f;
        st.foot_force[2] = 0.0f;
        st.foot_force[3] = 0.0f;

        // LowStateWire CRC
        st.crc = crc32_core(
            reinterpret_cast<const uint32_t*>(&st),
            (sizeof(LowStateWire) >> 2) - 1
        );

        // 回复给发送方（即 PC 插件绑定的 udp_local_port）
        const ssize_t sn = ::sendto(sock, &st, sizeof(st), 0,
                                    reinterpret_cast<sockaddr*>(&src), slen);
        (void)sn;
    }

    ::close(sock);
    return 0;
}
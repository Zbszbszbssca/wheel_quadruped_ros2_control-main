#ifndef UNITREE_UDP_WIRE_H
#define UNITREE_UDP_WIRE_H

#ifdef __cplusplus
#include <cstdint>
#include <cstring>
#else
#include <stdint.h>
#include <string.h>
#endif

#define UNITREE_UDP_MOTOR_SLOTS   20
#define UNITREE_UDP_ACTIVE_JOINTS 16

typedef struct
{
    uint8_t mode;
    uint8_t reserve[3];   // 保证 4 字节对齐
    float q;
    float dq;
    float tau;
    float kp;
    float kd;
} MotorCmdWire;

typedef struct
{
    float q;
    float dq;
    float tau_est;
    float reserve;        // 保证 16 字节整齐
} MotorStateWire;

typedef struct
{
    float quaternion[4];     // w x y z
    float gyroscope[3];      // xyz
    float accelerometer[3];  // xyz
} ImuStateWire;

typedef struct
{
    uint8_t head[2];         // 0xFE 0xEF
    uint8_t level_flag;      // 0xFF
    uint8_t frame_reserve;   // 预留
    uint32_t gpio;

    MotorCmdWire motor_cmd[UNITREE_UDP_MOTOR_SLOTS];

    uint32_t crc;
} LowCmdWire;

typedef struct
{
    MotorStateWire motor_state[UNITREE_UDP_MOTOR_SLOTS];
    ImuStateWire imu_state;
    float foot_force[4];

    uint32_t crc;
} LowStateWire;

#if defined(__cplusplus)
static_assert(sizeof(MotorCmdWire)   == 24,  "MotorCmdWire size mismatch");
static_assert(sizeof(MotorStateWire) == 16,  "MotorStateWire size mismatch");
static_assert(sizeof(ImuStateWire)   == 40,  "ImuStateWire size mismatch");
static_assert(sizeof(LowCmdWire)     == 492, "LowCmdWire size mismatch");
static_assert(sizeof(LowStateWire)   == 380, "LowStateWire size mismatch");
static_assert((sizeof(LowCmdWire)  % 4) == 0, "LowCmdWire must be 4-byte aligned in size");
static_assert((sizeof(LowStateWire) % 4) == 0, "LowStateWire must be 4-byte aligned in size");
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(MotorCmdWire)   == 24,  "MotorCmdWire size mismatch");
_Static_assert(sizeof(MotorStateWire) == 16,  "MotorStateWire size mismatch");
_Static_assert(sizeof(ImuStateWire)   == 40,  "ImuStateWire size mismatch");
_Static_assert(sizeof(LowCmdWire)     == 492, "LowCmdWire size mismatch");
_Static_assert(sizeof(LowStateWire)   == 380, "LowStateWire size mismatch");
_Static_assert((sizeof(LowCmdWire)  % 4) == 0, "LowCmdWire must be 4-byte aligned in size");
_Static_assert((sizeof(LowStateWire) % 4) == 0, "LowStateWire must be 4-byte aligned in size");
#endif

#endif // UNITREE_UDP_WIRE_H
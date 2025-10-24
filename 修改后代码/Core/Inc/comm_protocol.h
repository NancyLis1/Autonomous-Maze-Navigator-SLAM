/**
  ******************************************************************************
  * @file           : comm_protocol.h
  * @brief          : 通信协议模块 - 实现可靠的PC-MCU通信
  ******************************************************************************
  * @attention
  * 本模块实现任务3c要求：带包头和CRC校验的串行通信协议
  * 数据包格式: [Header(2B)] [Type(1B)] [Length(2B)] [Payload(NB)] [CRC16(2B)]
  ******************************************************************************
  */

#ifndef __COMM_PROTOCOL_H
#define __COMM_PROTOCOL_H

#include <stdint.h>
#include "main.h"

/* 协议常量 */
#define COMM_HEADER_0 0xAA
#define COMM_HEADER_1 0x55
#define COMM_MAX_PAYLOAD 128
#define COMM_PACKET_OVERHEAD 7  // Header(2) + Type(1) + Length(2) + CRC(2)

/* 消息类型 - 兼容原代码和 robot_serial_interface.py */
typedef enum {
    MSG_TYPE_POSE_SETPOINT = 0x01,  // PC -> MCU: 目标位姿设定 (兼容原代码)
    MSG_TYPE_POSE = 0x01,           // MCU -> PC: 位姿数据 (robot_serial)
    MSG_TYPE_SET_ODOMETRY = 0x02,   // PC -> MCU: 【新增】设置里程计初始位置
    MSG_TYPE_VELOCITY_CMD = 0x02,   // PC -> MCU: 速度指令 (兼容原代码)
    MSG_TYPE_SCAN = 0x02,           // MCU -> PC: 激光扫描 (robot_serial)
    MSG_TYPE_STATUS = 0x03,         // MCU -> PC: 状态信息
    MSG_TYPE_LIDAR_DATA = 0x10,     // MCU -> PC: 激光雷达数据 (兼容原代码)
    MSG_TYPE_SETPOINT = 0x10,       // PC -> MCU: 目标位姿 (robot_serial)
    MSG_TYPE_ODOMETRY = 0x11,       // MCU -> PC: 里程计数据 (兼容原代码)
    MSG_TYPE_IMU_DATA = 0x12,       // MCU -> PC: IMU数据 (兼容原代码)
    MSG_TYPE_ACK = 0x80,            // 应答
    MSG_TYPE_NACK = 0x81            // 否定应答
} MessageType_t;

/* 数据包结构 */
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];     // 包头 0xAA 0x55
    uint8_t msg_type;      // 消息类型
    uint16_t length;       // 载荷长度
    uint8_t payload[COMM_MAX_PAYLOAD];
    uint16_t crc16;        // CRC校验 (计算范围: msg_type ~ payload末尾)
} CommPacket_t;

/* 位姿设定点数据 - 匹配 robot_serial: <uint16 seq><float x,y,theta,v,omega> */
typedef struct {
    uint16_t seq;    // 序列号
    float x;         // 目标X坐标 (米)
    float y;         // 目标Y坐标 (米)
    float theta;     // 目标航向角 (弧度)
    float v;         // 目标线速度 (m/s)
    float omega;     // 目标角速度 (rad/s)
} PoseSetpoint_t;

/* 【新增】设置里程计初始位置数据 - 用于坐标系对齐 */
typedef struct {
    float x;         // 初始X坐标 (米)
    float y;         // 初始Y坐标 (米)
    float theta;     // 初始航向角 (弧度)
} SetOdometryData_t;

/* 位姿数据 - 匹配 robot_serial: <float x,y,theta,vx,omega> */
typedef struct {
    float x;              // 当前X坐标 (米)
    float y;              // 当前Y坐标 (米)
    float theta;          // 当前航向角 (弧度)
    float vx;             // X方向速度 (m/s) 或线速度
    float omega;          // 角速度 (rad/s)
} PoseData_t;

/* 里程计数据 - 兼容原代码 */
typedef struct {
    uint32_t timestamp;   // 时间戳 (ms)
    float x;              // 当前X坐标 (米)
    float y;              // 当前Y坐标 (米)
    float theta;          // 当前航向角 (弧度)
    float v_linear;       // 线速度 (m/s)
    float v_angular;      // 角速度 (rad/s)
} OdometryData_t;

/* 激光雷达数据 */
typedef struct {
    uint32_t timestamp;   // 时间戳 (ms)
    uint16_t num_points;  // 点数
    struct {
        float angle;      // 角度 (度)
        float distance;   // 距离 (mm)
        uint8_t quality;  // 质量
    } points[10];         // 每次最多发送10个点
} LidarData_t;

/* IMU数据 */
typedef struct {
    uint32_t timestamp;   // 时间戳 (ms)
    float yaw;            // 偏航角 (度)
    float roll;           // 横滚角 (度)
    float pitch;          // 俯仰角 (度)
    int16_t accel[3];     // 加速度 X/Y/Z
    int16_t gyro[3];      // 角速度 X/Y/Z
} ImuData_t;
#pragma pack(pop)

/* 接收状态机 */
typedef enum {
    RX_STATE_WAIT_HEADER0,
    RX_STATE_WAIT_HEADER1,
    RX_STATE_GET_TYPE,
    RX_STATE_GET_LENGTH_H,
    RX_STATE_GET_LENGTH_L,
    RX_STATE_GET_PAYLOAD,
    RX_STATE_GET_CRC_H,
    RX_STATE_GET_CRC_L
} RxState_t;

/* 接收缓冲区 */
typedef struct {
    RxState_t state;
    uint8_t msg_type;
    uint16_t length;
    uint16_t payload_index;
    uint8_t payload[COMM_MAX_PAYLOAD];
    uint16_t crc_received;
    uint32_t error_count;
    uint32_t packet_count;
} CommRxBuffer_t;

/* 函数声明 */
uint16_t Comm_CRC16(const uint8_t* data, uint16_t length);
uint16_t Comm_BuildPacket(CommPacket_t* packet, uint8_t msg_type,
                          const uint8_t* payload, uint16_t length);
void Comm_InitRxBuffer(CommRxBuffer_t* rx_buf);
int8_t Comm_ProcessByte(CommRxBuffer_t* rx_buf, uint8_t byte,
                        uint8_t* msg_type, uint8_t* payload, uint16_t* length);

/* 便捷发送函数 */
void Comm_SendPoseSetpoint(UART_HandleTypeDef* huart, const PoseSetpoint_t* pose);
void Comm_SendOdometry(UART_HandleTypeDef* huart, const OdometryData_t* odom);
void Comm_SendLidarData(UART_HandleTypeDef* huart, const LidarData_t* lidar);
void Comm_SendImuData(UART_HandleTypeDef* huart, const ImuData_t* imu);
void Comm_SendSetOdometryAck(UART_HandleTypeDef* huart);  // 【新增】发送设置里程计的ACK
void Comm_SendAck(UART_HandleTypeDef* huart, uint8_t msg_type);

#endif /* __COMM_PROTOCOL_H */

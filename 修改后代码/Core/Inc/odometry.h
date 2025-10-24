/**
  ******************************************************************************
  * @file           : odometry.h
  * @brief          : 里程计模块 - 双编码器位姿估计
  ******************************************************************************
  * @attention
  * 本模块实现任务3a要求：高频率(20-30Hz)发送位姿数据给PC
  ******************************************************************************
  */

#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"
#include <stdint.h>

/* 里程计配置 */
typedef struct {
    float wheel_radius;     // 车轮半径 (米)
    float wheel_base;       // 轮距 (米)
    uint16_t encoder_ppr;   // 编码器每转脉冲数
    float gear_ratio;       // 齿轮减速比
} OdometryConfig_t;

/* 里程计状态 */
typedef struct {
    float x;                // 全局X坐标 (米)
    float y;                // 全局Y坐标 (米)
    float theta;            // 航向角 (弧度)
    float v_linear;         // 线速度 (m/s)
    float v_angular;        // 角速度 (rad/s)
    int32_t last_encoder_left;
    int32_t last_encoder_right;
    uint32_t last_update_time;
} Odometry_t;

/* 函数声明 */
void Odometry_Init(Odometry_t* odom, const OdometryConfig_t* config);
void Odometry_Update(Odometry_t* odom, int32_t encoder_left, int32_t encoder_right);
void Odometry_Reset(Odometry_t* odom);
void Odometry_SetPose(Odometry_t* odom, float x, float y, float theta);
void Odometry_GetPose(const Odometry_t* odom, float* x, float* y, float* theta);
void Odometry_GetVelocity(const Odometry_t* odom, float* v_linear, float* v_angular);

/* 默认配置 */
extern const OdometryConfig_t DEFAULT_ODOM_CONFIG;

#endif /* __ODOMETRY_H */

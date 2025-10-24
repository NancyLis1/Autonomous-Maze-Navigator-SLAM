/**
  ******************************************************************************
  * @file           : pose_controller.h
  * @brief          : 位姿控制模块头文件 - 接收PC目标位姿并精确控制
  ******************************************************************************
  * @attention
  * 本模块实现任务3b要求：接收PC发送的目标位姿(x*, y*, θ*)并控制到达
  ******************************************************************************
  */

#ifndef __POSE_CONTROLLER_H
#define __POSE_CONTROLLER_H

#include "main.h"
#include <stdint.h>

/* 位姿结构体 */
typedef struct {
    float x;        // X坐标 (米)
    float y;        // Y坐标 (米)
    float theta;    // 航向角 (度, 0-360)
} Pose_t;

/* 位姿控制器配置 */
typedef struct {
    float linear_kp;      // 线性速度P增益
    float linear_ki;      // 线性速度I增益
    float linear_kd;      // 线性速度D增益
    float angular_kp;     // 角速度P增益
    float angular_ki;     // 角速度I增益
    float angular_kd;     // 角速度D增益
    float tolerance_pos;  // 位置容差 (米)
    float tolerance_ang;  // 角度容差 (度)
    float max_linear_vel; // 最大线速度 (m/s)
    float max_angular_vel;// 最大角速度 (rad/s)
} PoseControllerConfig_t;

/* 位姿控制器状态 */
typedef struct {
    Pose_t current_pose;  // 当前位姿
    Pose_t target_pose;   // 目标位姿
    float linear_integral;
    float angular_integral;
    float prev_linear_error;
    float prev_angular_error;
    uint8_t is_active;    // 是否激活位姿控制
    uint8_t reached;      // 是否到达目标
} PoseController_t;

/* 电机输出 */
typedef struct {
    float left_speed;     // 左轮速度 (PWM值)
    float right_speed;    // 右轮速度 (PWM值)
    uint8_t direction;    // 运动方向
} MotorOutput_t;

/* 函数声明 */
void PoseController_Init(PoseController_t* ctrl, PoseControllerConfig_t* config);
void PoseController_SetTarget(PoseController_t* ctrl, float x, float y, float theta);
void PoseController_UpdateCurrent(PoseController_t* ctrl, float x, float y, float theta);
MotorOutput_t PoseController_Update(PoseController_t* ctrl, float dt);
uint8_t PoseController_IsReached(PoseController_t* ctrl);
void PoseController_Reset(PoseController_t* ctrl);
void PoseController_Enable(PoseController_t* ctrl, uint8_t enable);

/* 默认配置 */
extern const PoseControllerConfig_t DEFAULT_POSE_CONFIG;

#endif /* __POSE_CONTROLLER_H */

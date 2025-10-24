/**
  ******************************************************************************
  * @file           : pose_controller.c
  * @brief          : 位姿控制模块实现 - 差分驱动机器人位姿闭环控制
  ******************************************************************************
  * @attention
  * 控制策略：先转向目标角度，再直线行驶到目标位置，最后调整最终朝向
  ******************************************************************************
  */

#include "pose_controller.h"
#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846f
#define WHEEL_BASE 0.20f  // 轮距 (米) - 根据实际机器人调整
#define MAX_PWM 400.0f    // 最大PWM值

/* 默认配置参数 */
const PoseControllerConfig_t DEFAULT_POSE_CONFIG = {
    .linear_kp = 200.0f,
    .linear_ki = 5.0f,
    .linear_kd = 10.0f,
    .angular_kp = 150.0f,
    .angular_ki = 3.0f,
    .angular_kd = 8.0f,
    .tolerance_pos = 0.05f,   // 5cm
    .tolerance_ang = 3.0f,    // 3度
    .max_linear_vel = 0.5f,   // 0.5m/s
    .max_angular_vel = 1.5f   // 约86度/秒
};

/* 静态配置 */
static PoseControllerConfig_t g_config;

/**
  * @brief  归一化角度到[-180, 180]
  */
static float NormalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
  * @brief  限幅函数
  */
static float Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
  * @brief  初始化位姿控制器
  */
void PoseController_Init(PoseController_t* ctrl, PoseControllerConfig_t* config) {
    memset(ctrl, 0, sizeof(PoseController_t));

    if (config != NULL) {
        memcpy(&g_config, config, sizeof(PoseControllerConfig_t));
    } else {
        memcpy(&g_config, &DEFAULT_POSE_CONFIG, sizeof(PoseControllerConfig_t));
    }
}

/**
  * @brief  设置目标位姿
  */
void PoseController_SetTarget(PoseController_t* ctrl, float x, float y, float theta) {
    ctrl->target_pose.x = x;
    ctrl->target_pose.y = y;
    ctrl->target_pose.theta = theta;
    ctrl->reached = 0;

    // 重置积分项
    ctrl->linear_integral = 0.0f;
    ctrl->angular_integral = 0.0f;
}

/**
  * @brief  更新当前位姿 (从里程计获取)
  */
void PoseController_UpdateCurrent(PoseController_t* ctrl, float x, float y, float theta) {
    ctrl->current_pose.x = x;
    ctrl->current_pose.y = y;
    ctrl->current_pose.theta = theta;
}

/**
  * @brief  启用/禁用位姿控制器
  */
void PoseController_Enable(PoseController_t* ctrl, uint8_t enable) {
    ctrl->is_active = enable;
    if (!enable) {
        ctrl->linear_integral = 0.0f;
        ctrl->angular_integral = 0.0f;
    }
}

/**
  * @brief  检查是否到达目标
  */
uint8_t PoseController_IsReached(PoseController_t* ctrl) {
    float dx = ctrl->target_pose.x - ctrl->current_pose.x;
    float dy = ctrl->target_pose.y - ctrl->current_pose.y;
    float distance = sqrtf(dx * dx + dy * dy);
    float angle_diff = fabsf(NormalizeAngle(ctrl->target_pose.theta - ctrl->current_pose.theta));

    if (distance < g_config.tolerance_pos && angle_diff < g_config.tolerance_ang) {
        ctrl->reached = 1;
        return 1;
    }
    return 0;
}

/**
  * @brief  位姿控制更新 (主控制循环调用)
  * @param  ctrl: 控制器
  * @param  dt: 时间步长 (秒)
  * @retval 电机输出指令
  */
MotorOutput_t PoseController_Update(PoseController_t* ctrl, float dt) {
    MotorOutput_t output = {0};

    // 未激活或已到达
    if (!ctrl->is_active || ctrl->reached) {
        return output;
    }

    // 计算位置误差
    float dx = ctrl->target_pose.x - ctrl->current_pose.x;
    float dy = ctrl->target_pose.y - ctrl->current_pose.y;
    float distance = sqrtf(dx * dx + dy * dy);

    // 计算目标航向角 (从当前位置指向目标位置)
    float target_heading = atan2f(dy, dx) * 180.0f / M_PI;

    // 角度误差 (先转向目标方向)
    float heading_error = NormalizeAngle(target_heading - ctrl->current_pose.theta);

    // 最终朝向误差
    float final_angle_error = NormalizeAngle(ctrl->target_pose.theta - ctrl->current_pose.theta);

    // 状态机：先转向 -> 直线行驶 -> 最终调整朝向
    float linear_velocity = 0.0f;
    float angular_velocity = 0.0f;

    if (distance < g_config.tolerance_pos) {
        // 阶段3：已到达目标位置，调整最终朝向
        if (fabsf(final_angle_error) > g_config.tolerance_ang) {
            // 角度PID控制
            ctrl->angular_integral += final_angle_error * dt;
            ctrl->angular_integral = Clamp(ctrl->angular_integral, -50.0f, 50.0f);

            float angular_derivative = (final_angle_error - ctrl->prev_angular_error) / dt;
            ctrl->prev_angular_error = final_angle_error;

            angular_velocity = g_config.angular_kp * final_angle_error +
                             g_config.angular_ki * ctrl->angular_integral +
                             g_config.angular_kd * angular_derivative;
            angular_velocity = Clamp(angular_velocity, -g_config.max_angular_vel, g_config.max_angular_vel);
        } else {
            ctrl->reached = 1;
        }
    } else if (fabsf(heading_error) > 30.0f) {
        // 【修复】阶段1：朝向误差大于30度，原地转向
        // 原来是15度，太小导致频繁切换阶段，改为30度
        ctrl->angular_integral += heading_error * dt;
        ctrl->angular_integral = Clamp(ctrl->angular_integral, -50.0f, 50.0f);

        float angular_derivative = (heading_error - ctrl->prev_angular_error) / dt;
        ctrl->prev_angular_error = heading_error;

        angular_velocity = g_config.angular_kp * heading_error +
                         g_config.angular_ki * ctrl->angular_integral +
                         g_config.angular_kd * angular_derivative;
        angular_velocity = Clamp(angular_velocity, -g_config.max_angular_vel, g_config.max_angular_vel);
    } else {
        // 阶段2：朝向基本对准，开始前进并微调
        ctrl->linear_integral += distance * dt;
        ctrl->linear_integral = Clamp(ctrl->linear_integral, -20.0f, 20.0f);

        float linear_derivative = (distance - ctrl->prev_linear_error) / dt;
        ctrl->prev_linear_error = distance;

        linear_velocity = g_config.linear_kp * distance +
                         g_config.linear_ki * ctrl->linear_integral +
                         g_config.linear_kd * linear_derivative;
        linear_velocity = Clamp(linear_velocity, 0.0f, g_config.max_linear_vel);

        // 同时进行小幅度转向修正
        // 【修复】降低转向修正系数，避免过度修正导致频繁转弯
        angular_velocity = g_config.angular_kp * 0.15f * heading_error;  // 从0.3f改为0.15f
        angular_velocity = Clamp(angular_velocity, -g_config.max_angular_vel * 0.3f,
                                g_config.max_angular_vel * 0.3f);  // 从0.5f改为0.3f
    }

    // 差分驱动运动学逆解
    float v_left = linear_velocity - (angular_velocity * WHEEL_BASE / 2.0f);
    float v_right = linear_velocity + (angular_velocity * WHEEL_BASE / 2.0f);

    // 转换为PWM值 (假设线速度1m/s对应PWM=400)
    output.left_speed = v_left * MAX_PWM / g_config.max_linear_vel;
    output.right_speed = v_right * MAX_PWM / g_config.max_linear_vel;

    // 限幅到[-MAX_PWM, MAX_PWM]
    output.left_speed = Clamp(output.left_speed, -MAX_PWM, MAX_PWM);
    output.right_speed = Clamp(output.right_speed, -MAX_PWM, MAX_PWM);

    // 判断前进/后退方向
    if (output.left_speed > 0 && output.right_speed > 0) {
        output.direction = 1; // 前进
    } else if (output.left_speed < 0 && output.right_speed < 0) {
        output.direction = 2; // 后退
        output.left_speed = -output.left_speed;
        output.right_speed = -output.right_speed;
    } else {
        output.direction = 3; // 转向
    }

    return output;
}

/**
  * @brief  重置控制器
  */
void PoseController_Reset(PoseController_t* ctrl) {
    ctrl->linear_integral = 0.0f;
    ctrl->angular_integral = 0.0f;
    ctrl->prev_linear_error = 0.0f;
    ctrl->prev_angular_error = 0.0f;
    ctrl->reached = 0;
}

/**
  ******************************************************************************
  * @file           : odometry.c
  * @brief          : 差分驱动机器人里程计实现
  ******************************************************************************
  */

#include "odometry.h"
#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846f

/* 默认配置 (根据实际机器人调整) */
const OdometryConfig_t DEFAULT_ODOM_CONFIG = {
    .wheel_radius = 0.0325f,  // 车轮半径32.5mm
    .wheel_base = 0.165f,     // 轮距165mm
    .encoder_ppr = 360,       // 360线编码器 (需确认实际值)
    .gear_ratio = 1.0f        // 减速比 (需确认实际值)
};

static OdometryConfig_t g_odom_config;

/**
  * @brief  初始化里程计
  */
void Odometry_Init(Odometry_t* odom, const OdometryConfig_t* config) {
    memset(odom, 0, sizeof(Odometry_t));

    if (config != NULL) {
        memcpy(&g_odom_config, config, sizeof(OdometryConfig_t));
    } else {
        memcpy(&g_odom_config, &DEFAULT_ODOM_CONFIG, sizeof(OdometryConfig_t));
    }

    odom->last_update_time = HAL_GetTick();
}

/**
  * @brief  更新里程计 (建议30Hz调用)
  * @param  odom: 里程计结构体
  * @param  encoder_left: 左编码器计数
  * @param  encoder_right: 右编码器计数
  */
void Odometry_Update(Odometry_t* odom, int32_t encoder_left, int32_t encoder_right) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - odom->last_update_time) / 1000.0f;

    if (dt < 0.001f) {
        return; // 时间间隔过短,跳过
    }

    // 计算编码器增量
    int32_t delta_left = encoder_left - odom->last_encoder_left;
    int32_t delta_right = encoder_right - odom->last_encoder_right;

    // 处理编码器溢出 (假设16位计数器)
    if (delta_left > 32768) delta_left -= 65536;
    if (delta_left < -32768) delta_left += 65536;
    if (delta_right > 32768) delta_right -= 65536;
    if (delta_right < -32768) delta_right += 65536;

    // 更新上次值
    odom->last_encoder_left = encoder_left;
    odom->last_encoder_right = encoder_right;
    odom->last_update_time = current_time;

    // 计算车轮移动距离 (米)
    float pulses_per_meter = g_odom_config.encoder_ppr * g_odom_config.gear_ratio /
                             (2.0f * M_PI * g_odom_config.wheel_radius);
    float dist_left = delta_left / pulses_per_meter;
    float dist_right = delta_right / pulses_per_meter;

    // 差分驱动运动学
    float dist_center = (dist_left + dist_right) / 2.0f;
    float delta_theta = (dist_right - dist_left) / g_odom_config.wheel_base;

    // 更新速度
    odom->v_linear = dist_center / dt;
    odom->v_angular = delta_theta / dt;

    // 更新位姿 (使用中点法积分)
    float theta_mid = odom->theta + delta_theta / 2.0f;
    odom->x += dist_center * cosf(theta_mid);
    odom->y += dist_center * sinf(theta_mid);
    odom->theta += delta_theta;

    // 归一化角度到[-π, π]
    while (odom->theta > M_PI) odom->theta -= 2.0f * M_PI;
    while (odom->theta < -M_PI) odom->theta += 2.0f * M_PI;
}

/**
  * @brief  重置里程计到原点
  */
void Odometry_Reset(Odometry_t* odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
    odom->v_linear = 0.0f;
    odom->v_angular = 0.0f;
}

/**
  * @brief  手动设置位姿 (用于初始化或重定位)
  */
void Odometry_SetPose(Odometry_t* odom, float x, float y, float theta) {
    odom->x = x;
    odom->y = y;
    odom->theta = theta;
}

/**
  * @brief  获取当前位姿
  * 【修复】保持弧度单位，与Python端和PoseSetpoint协议一致
  */
void Odometry_GetPose(const Odometry_t* odom, float* x, float* y, float* theta) {
    *x = odom->x;
    *y = odom->y;
    *theta = odom->theta;  // ✅ 保持弧度，不转换
}

/**
  * @brief  获取当前速度
  */
void Odometry_GetVelocity(const Odometry_t* odom, float* v_linear, float* v_angular) {
    *v_linear = odom->v_linear;
    *v_angular = odom->v_angular;
}

/**
  ******************************************************************************
  * @file           : safety.h
  * @brief          : 安全模块 - 紧急停止和安全限制
  ******************************************************************************
  * @attention
  * 本模块实现任务3d要求：速度限制和紧急停止(E-stop)机制
  ******************************************************************************
  */

#ifndef __SAFETY_H
#define __SAFETY_H

#include "main.h"
#include <stdint.h>

/* 安全限制配置 */
typedef struct {
    float max_linear_vel;     // 最大线速度 (m/s)
    float max_angular_vel;    // 最大角速度 (rad/s)
    uint16_t max_pwm;         // 最大PWM值
    uint16_t estop_gpio_pin;  // 急停按钮GPIO引脚
    GPIO_TypeDef* estop_gpio_port;
    uint32_t watchdog_timeout_ms; // 看门狗超时 (ms)
} SafetyConfig_t;

/* 安全状态 */
typedef struct {
    uint8_t estop_active;        // 紧急停止激活
    uint8_t watchdog_expired;    // 看门狗超时
    uint32_t last_command_time;  // 最后指令时间戳
    uint32_t estop_count;        // 急停触发次数
    uint8_t speed_limit_exceeded;// 速度超限标志
} SafetyStatus_t;

/* 函数声明 */
void Safety_Init(SafetyConfig_t* config);
void Safety_Update(SafetyStatus_t* status);
uint8_t Safety_CheckEstop(void);
uint8_t Safety_CheckWatchdog(SafetyStatus_t* status);
void Safety_FeedWatchdog(SafetyStatus_t* status);
void Safety_EmergencyStop(void);
float Safety_LimitSpeed(float speed, float max_speed);
uint16_t Safety_LimitPWM(float pwm, uint16_t max_pwm);

/* 默认配置 */
extern const SafetyConfig_t DEFAULT_SAFETY_CONFIG;

#endif /* __SAFETY_H */

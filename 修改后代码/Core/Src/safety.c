/**
  ******************************************************************************
  * @file           : safety.c
  * @brief          : 安全模块实现
  ******************************************************************************
  */

#include "safety.h"
#include <math.h>

/* 外部变量 (从main.c) */
extern TIM_HandleTypeDef htim3;

/* 默认安全配置 */
const SafetyConfig_t DEFAULT_SAFETY_CONFIG = {
    .max_linear_vel = 0.8f,      // 最大0.8m/s
    .max_angular_vel = 2.0f,     // 最大2rad/s (约114度/秒)
    .max_pwm = 400,              // 最大PWM=400
    .estop_gpio_pin = GPIO_PIN_0,  // 假设使用PA0 (需根据实际修改)
    .estop_gpio_port = GPIOA,
    .watchdog_timeout_ms = 1000  // 1秒无指令则停止
};

static SafetyConfig_t g_safety_config;

/**
  * @brief  初始化安全模块
  */
void Safety_Init(SafetyConfig_t* config) {
    if (config != NULL) {
        g_safety_config = *config;
    } else {
        g_safety_config = DEFAULT_SAFETY_CONFIG;
    }
}

/**
  * @brief  检查紧急停止按钮 (需连接物理按钮)
  * @retval 1=按下, 0=未按下
  */
uint8_t Safety_CheckEstop(void) {
    // ⚠️ 临时禁用急停检查(调试模式)
    // TODO: 生产环境需启用并配置正确的GPIO引脚
    return 0;  // 始终返回0 (未按下)

    /* 原实现(生产环境启用):
    // 读取GPIO引脚电平 (低电平有效)
    if (HAL_GPIO_ReadPin(g_safety_config.estop_gpio_port,
                         g_safety_config.estop_gpio_pin) == GPIO_PIN_RESET) {
        return 1; // 急停按钮按下
    }
    return 0;
    */
}

/**
  * @brief  检查通信看门狗
  * @retval 1=超时, 0=正常
  */
uint8_t Safety_CheckWatchdog(SafetyStatus_t* status) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - status->last_command_time > g_safety_config.watchdog_timeout_ms) {
        status->watchdog_expired = 1;
        return 1;
    }
    status->watchdog_expired = 0;
    return 0;
}

/**
  * @brief  喂看门狗 (接收到有效指令时调用)
  */
void Safety_FeedWatchdog(SafetyStatus_t* status) {
    status->last_command_time = HAL_GetTick();
    status->watchdog_expired = 0;
}

/**
  * @brief  执行紧急停止
  */
void Safety_EmergencyStop(void) {
    // 立即停止所有电机PWM输出
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

/**
  * @brief  更新安全状态 (主循环调用)
  */
void Safety_Update(SafetyStatus_t* status) {
    // 检查紧急停止按钮
    if (Safety_CheckEstop()) {
        if (!status->estop_active) {
            status->estop_active = 1;
            status->estop_count++;
            Safety_EmergencyStop();
        }
    } else {
        status->estop_active = 0;
    }

    // 检查看门狗
    if (Safety_CheckWatchdog(status)) {
        Safety_EmergencyStop();
    }
}

/**
  * @brief  限制速度值
  */
float Safety_LimitSpeed(float speed, float max_speed) {
    if (speed > max_speed) return max_speed;
    if (speed < -max_speed) return -max_speed;
    return speed;
}

/**
  * @brief  限制PWM值
  */
uint16_t Safety_LimitPWM(float pwm, uint16_t max_pwm) {
    if (pwm > (float)max_pwm) return max_pwm;
    if (pwm < 0.0f) return 0;
    return (uint16_t)pwm;
}

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ������ - ���ɵ�����ơ�PID��RPLIDAR��MPU6500�����Ż��Զ�ת���ܣ�
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms in LICENSE file or provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ========== 新增: 补充模块头文件 ========== */
#include "pose_controller.h"
#include "comm_protocol.h"
#include "odometry.h"
#include "safety.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float integral_limit;
} PID_Controller;

// �״����ݽṹ�� (�Ż���)
#pragma pack(push, 1)
typedef struct {
    uint8_t sync_quality;  // ͬ��λ+����λ
    uint16_t angle_q6;     // �Ƕ�ֵ (Q6��ʽ)
    uint16_t distance_q2;  // ����ֵ (Q2��ʽ)
} LidarDataPacket;
#pragma pack(pop)

// 【新增】激光雷达角度缓存 - 用于存储0-359度的数据
typedef struct {
    float distance;        // 距离(mm)
    uint8_t quality;       // 质量(0-63)
    uint32_t update_time;  // 更新时间戳(ms)
    uint8_t valid;         // 有效标志(1=有效, 0=无效)
} AngleCache;

/* Private define ------------------------------------------------------------*/
#define MAX_SPEED 400
#define MAX_SPEED_RIGHT 420
#define SPEED_UPDATE_INTERVAL 10
#define TURN_DURATION 800
#define TURN_SPEED 300
#define DIR_STOP 0
#define DIR_FORWARD 1
#define DIR_BACKWARD 2
#define DIR_LEFT 3
#define DIR_RIGHT 4
#define DIR_AUTO_LEFT 5  // �Զ���ת80��
#define DIR_AUTO_RIGHT 6 // �Զ���ת80��
#define PPR 360
#define SAMPLE_TIME_MS 100
#define M_PI 3.14159265358979323846
#define DMA_BUFFER_SIZE 256
#define LIDAR_PACKET_SIZE 5
#define ANGLE_FILTER_THRESHOLD 1.0f
#define MIN_VALID_DISTANCE 50.0f
#define MAX_VALID_DISTANCE 12000.0f
#define LIDAR_TIMEOUT_THRESHOLD 500

// PID����
#define PID_KP 1.2f
#define PID_KI 0.1f
#define PID_KD 0.05f
#define PID_INTEGRAL_LIMIT 200.0f
#define RPM_TO_PWM_FACTOR 2.0f

// MPU6500����
#define MPU6500_ADDR 0xD0
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75
#define CALIBRATION_SAMPLES 500
#define ACCEL_FS_SEL_4G 0x08
#define GYRO_FS_SEL_500DPS 0x08

// �Զ�ת�����
#define TURN_ANGLE_THRESHOLD 3.0f      // �Ƕ��ݲ���ֵ ��3��
#define AUTO_TURN_SPEED 300            // �Զ�ת���ٶ�
#define MIN_TURN_SPEED 120             // ��Сת���ٶ�
#define SLOWDOWN_ANGLE 45.0f           // ��ʼ���ٵĽǶ�
#define OVERSHOOT_THRESHOLD 30.0f      // ��������ֵ

// ��ƫУ׼����
#define GYRO_BIAS_UPDATE_INTERVAL 10   // ��ƫ���¼��(ms)
#define COMPLEMENTARY_FILTER_GAIN 0.02f // �����˲�������
#define STATIONARY_THRESHOLD 0.1f      // ��ֹ�����ֵ(m/s2)
#define STATIONARY_DURATION 1000       // ��ֹ����ʱ��(ms)

/* Private variables ---------------------------------------------------------*/
uint8_t bluetooth_rx_data = 0;
uint8_t target_direction = DIR_STOP;
uint8_t current_direction = DIR_STOP;
uint32_t current_speed = 0;
uint32_t target_speed = 0;
uint32_t last_speed_update_time = 0;
uint32_t last_encoder_time = 0;  // 编码器上次处理时间戳

uint8_t bluetooth_connected = 0;
uint8_t connection_announced = 0;
uint8_t connection_msg[] = "Connected\r\n";

int32_t lastEncoderA = 0;
int32_t lastEncoderB = 0;

char uart_buf[256];

uint8_t lidar_dma_buffer[DMA_BUFFER_SIZE];
uint8_t lidar_dataBuffer[LIDAR_PACKET_SIZE];
uint8_t lidar_dataIndex = 0;
uint32_t lidar_rxIndex = 0;
volatile uint8_t lidar_process_packet = 0;
float lastLidarAngle = 0.0f;
uint32_t lastLidarRxTime = 0;

// 【新增】激光雷达角度缓存 - 存储0-359度的数据
AngleCache angle_cache[360] = { 0 };
uint8_t current_output_angle = 0;  // 当前输出的角度(0-359)
uint32_t last_output_time = 0;     // 上次输出时间戳

int16_t accel_offset_x, accel_offset_y, accel_offset_z;
int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temp;
float roll, pitch;
float yaw = 0.0f;  // ƫ����
uint32_t last_imu_time = 0;  // ���ڼ���ʱ����

PID_Controller pid_left = { PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, PID_INTEGRAL_LIMIT };
PID_Controller pid_right = { PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, PID_INTEGRAL_LIMIT };
float target_rpm = 0.0f;
float pwm_left = 0, pwm_right = 0;

// �Զ�ת����Ʊ���
uint8_t auto_turn_mode = 0;        // 0:���Զ�ת�� 1:��ת80�� 2:��ת80��
float start_yaw = 0.0f;            // ��ʼת��ʱ�ĳ�ʼƫ����
float target_yaw = 0.0f;           // Ŀ��ƫ����
float prev_angle_diff = 0.0f;      // ��һ�νǶȲ���ڼ����壩
uint8_t overshoot_count = 0;       // ���������

// ��ƫУ׼����
float gyro_z_bias = 0.0f;          // ������Z����ƫ
uint8_t is_stationary = 0;         // �Ƿ�ֹ��־
uint32_t stationary_start_time = 0;// ��ֹ��ʼʱ��

/* ========== 新增: 补充模块全局变量 ========== */
// 位姿控制器
PoseController_t g_pose_controller;
PoseControllerConfig_t g_pose_config = {
    // 【修复】优化PID参数，避免频繁转弯
    .linear_kp = 150.0f,    // 降低线速度P增益，避免过度加速
    .linear_ki = 3.0f,      // 降低积分增益
    .linear_kd = 5.0f,      // 降低微分增益
    .angular_kp = 100.0f,   // 降低角速度P增益，避免过度转向
    .angular_ki = 2.0f,     // 降低积分增益
    .angular_kd = 5.0f,     // 降低微分增益
    .tolerance_pos = 0.05f,
    .tolerance_ang = 3.0f,
    .max_linear_vel = 0.3f,    // 【修复】降低最大线速度，使运动更平稳
    .max_angular_vel = 1.0f    // 【修复】降低最大角速度，避免过度转向
};

// 里程计
Odometry_t g_odometry;
OdometryConfig_t g_odom_config = {
    .wheel_radius = 0.0325f,  // 【修复】轮半径32.5mm（与odometry.c一致）
    .wheel_base = 0.165f,     // 【修复】轮距165mm（与odometry.c一致）
    .encoder_ppr = 360,
    .gear_ratio = 1.0f
};

// 通信协议接收缓冲区
CommRxBuffer_t g_comm_rx_buffer;

// 安全模块
SafetyStatus_t g_safety_status = {0};
SafetyConfig_t g_safety_config = {
    .max_linear_vel = 0.8f,
    .max_angular_vel = 2.0f,
    .max_pwm = 400,
    .estop_gpio_pin = GPIO_PIN_0,  // ⚠️ 需根据实际硬件修改
    .estop_gpio_port = GPIOA,
    .watchdog_timeout_ms = 60000  // 调试模式: 60秒超时(生产环境改回1000)
};

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ControlMotor(uint8_t direction, uint32_t speed);
void UpdateSpeedRamp(void);
void SendConnectionNotification(void);
void ProcessLidarData(uint8_t* data);
void SendLidarToBluetooth(float angle, float distance, uint8_t quality);
float PID_Update(PID_Controller* pid, float setpoint, float measurement);
void MPU6500_Init(I2C_HandleTypeDef* hi2c);
void MPU6500_Calibrate(I2C_HandleTypeDef* hi2c);
void MPU6500_Read_All(I2C_HandleTypeDef* hi2c);
void Calculate_Angles(void);
void Send_IMU_Data_Bluetooth(void);
float CalculateAngleDifference(float current, float target);
void HandleAutoTurn(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  ����ǶȲ�ֵ������360�ȱ߽磩
  * @param  current: ��ǰ�Ƕ�
  * @param  target: Ŀ��Ƕ�
  * @retval �ǶȲ�ֵ
  */
float CalculateAngleDifference(float current, float target) {
    float diff = target - current;

    // ����360�ȱ߽����
    if (diff > 180.0f) {
        diff -= 360.0f;
    }
    else if (diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}

/**
  * @brief  PID����������
  * @param  pid: PID�������ṹ��
  * @param  setpoint: Ŀ��ֵ
  * @param  measurement: ����ֵ
  * @retval PID���ֵ
  */
float PID_Update(PID_Controller* pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->integral += error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

/**
  * @brief  UART������ɻص�
  * @param  huart: UART���
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        if (!bluetooth_connected) bluetooth_connected = 1;

        /* ========== 新增: 喂看门狗 ========== */
        Safety_FeedWatchdog(&g_safety_status);

        /* ========== 新增: 处理通信协议数据包 ========== */
        uint8_t msg_type;
        uint8_t payload[128];
        uint16_t length;
        int8_t result = Comm_ProcessByte(&g_comm_rx_buffer, bluetooth_rx_data,
                                         &msg_type, payload, &length);

        if (result == 1) {  // 成功接收完整数据包
            if (msg_type == MSG_TYPE_POSE_SETPOINT) {
                PoseSetpoint_t* setpoint = (PoseSetpoint_t*)payload;

                // 【修复】将theta从弧度转换为度（PoseController内部使用度）
                float theta_deg = setpoint->theta * 180.0f / M_PI;

                // 设置目标位姿
                PoseController_SetTarget(&g_pose_controller,
                                        setpoint->x, setpoint->y, theta_deg);
                PoseController_Enable(&g_pose_controller, 1);

                // 发送ACK
                Comm_SendAck(&huart1, MSG_TYPE_POSE_SETPOINT);

                // 退出自动转向和手动模式
                auto_turn_mode = 0;
                target_direction = DIR_STOP;
            }
            else if (msg_type == MSG_TYPE_SET_ODOMETRY) {
                // 【新增】处理设置里程计初始位置
                SetOdometryData_t* set_odom = (SetOdometryData_t*)payload;

                // 设置里程计的初始位置
                Odometry_SetPose(&g_odometry, set_odom->x, set_odom->y, set_odom->theta);

                // 发送ACK
                Comm_SendAck(&huart1, MSG_TYPE_SET_ODOMETRY);

                // 调试输出
                char dbg[100];
                snprintf(dbg, sizeof(dbg), "[ODOM-SET] 里程计初始位置已设置: x=%.3f y=%.3f θ=%.3f\r\n",
                        set_odom->x, set_odom->y, set_odom->theta);
                HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
            }
        }

        /* ========== 原有: 手动控制指令 ========== */
        if (bluetooth_rx_data >= '0' && bluetooth_rx_data <= '7') {  // ��չ��7'
            // 禁用位姿控制器,切换到手动模式
            PoseController_Enable(&g_pose_controller, 0);

            target_direction = bluetooth_rx_data - '0';

            // �����Զ�ת��ָ��
            if (target_direction == DIR_AUTO_LEFT || target_direction == DIR_AUTO_RIGHT) {
                auto_turn_mode = target_direction;  // �����Զ�ת��ģʽ
                start_yaw = yaw;  // ��¼��ǰƫ����
                prev_angle_diff = 0.0f;
                overshoot_count = 0;

                // ����Ŀ��ƫ���ǣ�ʹ����ԽǶȣ�
                if (target_direction == DIR_AUTO_LEFT) {
                    target_yaw = yaw + 80.0f;
                    if (target_yaw >= 360.0f) target_yaw -= 360.0f;
                }
                else {
                    target_yaw = yaw - 80.0f;
                    if (target_yaw < 0.0f) target_yaw += 360.0f;
                }

                // ����ת���ٶ�
                current_speed = AUTO_TURN_SPEED;
                target_speed = AUTO_TURN_SPEED;

                // ����ȷ����Ϣ
                char msg[60];
                snprintf(msg, sizeof(msg), "Auto turn started. Start:%.1f��, Target:%.1f��\r\n", start_yaw, target_yaw);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            }
            // ������������ָ��
            else {
                auto_turn_mode = 0;  // �˳��Զ�ת��ģʽ

                switch (target_direction) {
                case DIR_STOP:
                    target_speed = 0;
                    current_speed = 0;
                    target_rpm = 0.0f;
                    break;
                case DIR_FORWARD:
                case DIR_BACKWARD:
                    target_speed = MAX_SPEED;
                    current_speed = MAX_SPEED;
                    target_rpm = 60.0f;  // 设置目标转速为60RPM
                    break;
                case DIR_LEFT:
                case DIR_RIGHT:
                    target_speed = TURN_SPEED;
                    current_speed = TURN_SPEED;
                    target_rpm = 30.0f;  // 转向时目标转速30RPM
                    break;
                }
                ControlMotor(target_direction, current_speed);
            }
        }
        HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);
    }
    else if (huart == &huart6) {
        // �����״����ݴ�����־
        lidar_process_packet = 1;
        // ��������DMA����
        HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);
    }
}

/**
  * @brief  ������ƺ���
  * @param  direction: ����
  * @param  speed: �ٶ�
  * @retval None
  */
void ControlMotor(uint8_t direction, uint32_t speed) {
    switch (direction) {
    case DIR_STOP:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_FORWARD:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (pwm_left > 0) ? (uint32_t)pwm_left : speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (pwm_right > 0) ? (uint32_t)pwm_right : speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_BACKWARD:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (pwm_left > 0) ? (uint32_t)pwm_left : speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (pwm_right > 0) ? (uint32_t)pwm_right : speed);
        break;
    case DIR_LEFT:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_RIGHT:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, TURN_SPEED);
        break;
    }
}

/**
  * @brief  �ٶ�б�¸���
  * @retval None
  */
void UpdateSpeedRamp(void) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL) {
        last_speed_update_time = current_time;

        // �Զ�ת��ģʽ�²������ٶ�б��
        if (auto_turn_mode != 0) return;

        if (target_direction == DIR_STOP && current_direction != DIR_STOP) {
            current_speed = 0;
            current_direction = DIR_STOP;
            ControlMotor(DIR_STOP, 0);
            return;
        }

        if (target_direction != current_direction) {
            current_direction = target_direction;
            if (current_direction == DIR_LEFT || current_direction == DIR_RIGHT) {
                current_speed = TURN_SPEED;
                ControlMotor(current_direction, current_speed);
            }
            else if (current_direction != DIR_STOP) {
                current_speed = target_speed = MAX_SPEED;
                ControlMotor(current_direction, current_speed);
            }
        }
        else if (current_direction != DIR_STOP) {
            if (current_speed < target_speed) {
                current_speed = target_speed;
                ControlMotor(current_direction, current_speed);
            }
        }
    }
}

/**
  * @brief  ��������֪ͨ
  * @retval None
  */
void SendConnectionNotification(void) {
    if (bluetooth_connected && !connection_announced) {
        HAL_UART_Transmit(&huart1, connection_msg, sizeof(connection_msg) - 1, 1000);
        connection_announced = 1;
    }
}

/**
  * @brief  �����Զ�ת��
  * @retval None
  */
void HandleAutoTurn(void) {
    // ���㵱ǰ�Ƕ���Ŀ��ǶȵĲ�ֵ
    float angle_diff = CalculateAngleDifference(yaw, target_yaw);
    float abs_diff = fabsf(angle_diff);

    // �����壨����ת��
    if ((angle_diff * prev_angle_diff < 0) && (abs_diff > OVERSHOOT_THRESHOLD)) {
        overshoot_count++;
        if (overshoot_count > 2) {
            // ��ι��壬ǿ��ֹͣ
            ControlMotor(DIR_STOP, 0);
            auto_turn_mode = 0;
            overshoot_count = 0;

            // ���ݹ���Ƕȸ�����ƫ
            gyro_z_bias += angle_diff * 0.01f;

            // ����֪ͨ
            char msg[60];
            snprintf(msg, sizeof(msg), "Auto turn overshoot! Stopped. Adjusted bias: %.3f\r\n", gyro_z_bias);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            return;
        }
    }
    else {
        overshoot_count = 0;
    }
    prev_angle_diff = angle_diff;

    // ����Ƿ�ﵽĿ��Ƕ�
    if (abs_diff <= TURN_ANGLE_THRESHOLD) {
        // ���ת��
        ControlMotor(DIR_STOP, 0);
        auto_turn_mode = 0;

        // ת����ɺ�У׼��ƫ
        float current_gyro = gyro_z / 65.5f;
        gyro_z_bias = 0.95f * gyro_z_bias + 0.05f * current_gyro;

        HAL_UART_Transmit(&huart1, (uint8_t*)"Auto turn completed!\r\n", 22, 100);
    }
    else {
        // ���ܵ��٣�����Ŀ��Խ���ٶ�Խ��
        float speed_factor = 1.0f;
        if (abs_diff < SLOWDOWN_ANGLE) {
            speed_factor = 0.3f + 0.7f * (abs_diff / SLOWDOWN_ANGLE);
        }

        uint32_t adjusted_speed = (uint32_t)(AUTO_TURN_SPEED * speed_factor);
        if (adjusted_speed < MIN_TURN_SPEED) adjusted_speed = MIN_TURN_SPEED;

        // Ӧ��ת��
        if (auto_turn_mode == DIR_AUTO_LEFT) {
            // ��ת������ǰ�������ֺ���
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, adjusted_speed); // �����
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, adjusted_speed); // ��ǰ��
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        }
        else if (auto_turn_mode == DIR_AUTO_RIGHT) {
            // ��ת������ǰ�������ֺ���
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, adjusted_speed); // ��ǰ��
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, adjusted_speed); // �Һ���
        }

        // ������Ϣ
        static uint32_t last_debug_time = 0;
        if (HAL_GetTick() - last_debug_time > 200) {
            last_debug_time = HAL_GetTick();

            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
        }
    }
}

/**
  * @brief  处理激光雷达数据 (带角度有效性检查)
  * @param  data: 数据指针
  * @retval None
  */
void ProcessLidarData(uint8_t* data) {
    LidarDataPacket* pkt = (LidarDataPacket*)data;

    // 检查同步位和起始位
    if ((pkt->sync_quality & 0x80) && (pkt->angle_q6 & 0x01)) {
        uint8_t quality = pkt->sync_quality & 0x7F;

        // 计算原始角度（可能超出0-360范围）
        float raw_angle = (pkt->angle_q6 >> 1) / 64.0f;
        float distance = pkt->distance_q2 / 4.0f;


        // 检查角度是否在有效范围内 (0-360度)
        if (raw_angle >= 0.0f && raw_angle <= 360.0f) {
            // 规范化角度到0-360度范围（处理浮点误差）
            float angle = fmodf(raw_angle, 360.0f);
            if (angle < 0) angle += 360.0f;

            // 检查距离和质量是否有效
            if (distance >= MIN_VALID_DISTANCE &&
                distance <= MAX_VALID_DISTANCE &&
                quality > 0) {

                // 【修复】使用角度缓存存储数据，而不是直接发送
                // 这样可以确保每个角度的数据都被保存，避免数据丢失
                int32_t angle_deg = (int32_t)(angle + 0.5f);  // 四舍五入到整数
                angle_deg %= 360;
                if (angle_deg < 0) angle_deg += 360;

                angle_cache[angle_deg].distance = distance;
                angle_cache[angle_deg].quality = quality;
                angle_cache[angle_deg].update_time = HAL_GetTick();
                angle_cache[angle_deg].valid = 1;
            }
        }
        else {
            // 可选：记录无效角度数据（调试用）
            static uint32_t invalid_angle_count = 0;
            invalid_angle_count++;

            // 每100个无效数据报告一次
            if (invalid_angle_count % 100 == 0) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Invalid angle detected: %.2f (Count: %lu)\n",
                        raw_angle, invalid_angle_count);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            }
        }
    }
}

/**
  * @brief  ���ͼ����״����ݵ����� (���Ż�)
  * @param  angle: �Ƕ�
  * @param  distance: ����
  * @param  quality: ����
  * @retval None
  */
void SendLidarToBluetooth(float angle, float distance, uint8_t quality) {
    // 【修复】移除频率限制，直接发送激光雷达数据
    // 原来的200ms限制导致激光雷达数据发送太慢，无法建图
    // 现在直接发送，让Python端控制接收频率

    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "Lidar: A=%.2f, D=%.2fmm, Q=%d\n",
                      angle, distance, quality);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
}

/**
  * @brief  【新增】按顺序输出0-359度的激光雷达数据
  * @param  None
  * @retval None
  */
void OutputAngleDataInOrder(void) {
    // 【新增】每10ms输出一个角度的数据，确保所有角度都被发送
    #define ANGLE_DATA_TIMEOUT 1000  // 数据超时时间(ms)

    uint32_t current_time = HAL_GetTick();
    AngleCache* cache = &angle_cache[current_output_angle];

    // 检查数据是否超时
    if (cache->valid && (current_time - cache->update_time > ANGLE_DATA_TIMEOUT)) {
        cache->valid = 0;
    }

    // 发送当前角度的数据
    if (cache->valid) {
        char buffer[32];
        int len = snprintf(buffer, sizeof(buffer),
            "Lidar: A=%d, D=%.0fmm, Q=%d\n",
            current_output_angle,
            cache->distance,
            cache->quality);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 10);
    }

    // 移动到下一个角度(0-359循环)
    current_output_angle = (current_output_angle + 1) % 360;
}


/**
  * @brief  MPU6500��ʼ��
  * @param  hi2c: I2C���
  * @retval None
  */
void MPU6500_Init(I2C_HandleTypeDef* hi2c) {
    uint8_t check, data;

    // ����豸ID
    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, WHO_AM_I, 1, &check, 1, 100);
    if (check == 0x70) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6500 Found!\r\n", 16, 100);
    }
    else {
        char msg[30];
        snprintf(msg, sizeof(msg), "MPU6500 Not Found! ID:0x%X\r\n", check);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        Error_Handler();
    }

    // �����豸
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
    HAL_Delay(100);

    // ���������� ��500��/s
    data = GYRO_FS_SEL_500DPS;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, GYRO_CONFIG, 1, &data, 1, 100);

    // ���ü��ٶȼ� ��4g
    data = ACCEL_FS_SEL_4G;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, ACCEL_CONFIG, 1, &data, 1, 100);

    // ���õ�ͨ�˲��� 5Hz
    data = 0x06;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, CONFIG, 1, &data, 1, 100);

    // ������ 1kHz/(1+7)=125Hz
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, SMPLRT_DIV, 1, &data, 1, 100);
}

/**
  * @brief  MPU6500У׼
  * @param  hi2c: I2C���
  * @retval None
  */
void MPU6500_Calibrate(I2C_HandleTypeDef* hi2c) {
    int32_t accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)"Calibrating MPU6500...\r\n", 24, 100);

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        uint8_t buf[14];
        HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

        // �ۼ�ԭʼ����
        accel_sum_x += (int16_t)((buf[0] << 8) | buf[1]);
        accel_sum_y += (int16_t)((buf[2] << 8) | buf[3]);
        accel_sum_z += (int16_t)((buf[4] << 8) | buf[5]);
        gyro_sum_x += (int16_t)((buf[8] << 8) | buf[9]);
        gyro_sum_y += (int16_t)((buf[10] << 8) | buf[11]);
        gyro_sum_z += (int16_t)((buf[12] << 8) | buf[13]);

        HAL_Delay(5);
    }

    // ����ƽ��ֵ��Ϊƫ��
    accel_offset_x = accel_sum_x / CALIBRATION_SAMPLES;
    accel_offset_y = accel_sum_y / CALIBRATION_SAMPLES;
    accel_offset_z = (accel_sum_z / CALIBRATION_SAMPLES) - 16384; // 1g��Ӧֵ

    gyro_offset_x = gyro_sum_x / CALIBRATION_SAMPLES;
    gyro_offset_y = gyro_sum_y / CALIBRATION_SAMPLES;
    gyro_offset_z = gyro_sum_z / CALIBRATION_SAMPLES;

    // ��ӡУ׼���
    char cal_msg[128];
    snprintf(cal_msg, sizeof(cal_msg),
        "Calibration Complete:\r\n"
        "Accel Offsets: X:%d Y:%d Z:%d\r\n"
        "Gyro Offsets: X:%d Y:%d Z:%d\r\n",
        accel_offset_x, accel_offset_y, accel_offset_z,
        gyro_offset_x, gyro_offset_y, gyro_offset_z);
    HAL_UART_Transmit(&huart1, (uint8_t*)cal_msg, strlen(cal_msg), 100);
}

/**
  * @brief  ��ȡMPU6500��������
  * @param  hi2c: I2C���
  * @retval None
  */
void MPU6500_Read_All(I2C_HandleTypeDef* hi2c) {
    uint8_t buf[14];

    // ��ȡ14�ֽ����� (���ٶ�+�¶�+������)
    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

    // ��ȥУ׼ƫ����
    accel_x = (int16_t)((buf[0] << 8) | buf[1]) - accel_offset_x;
    accel_y = (int16_t)((buf[2] << 8) | buf[3]) - accel_offset_y;
    accel_z = (int16_t)((buf[4] << 8) | buf[5]) - accel_offset_z;

    temp = (int16_t)((buf[6] << 8) | buf[7]);

    gyro_x = (int16_t)((buf[8] << 8) | buf[9]) - gyro_offset_x;
    gyro_y = (int16_t)((buf[10] << 8) | buf[11]) - gyro_offset_y;
    gyro_z = (int16_t)((buf[12] << 8) | buf[13]) - gyro_offset_z;
}

/**
  * @brief  ����Ƕȣ���ƫ���ǣ� - �Ż���
  * @retval None
  */
void Calculate_Angles(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    // ����ʱ���룩
    float delta_time = (current_time - last_time) / 1000.0f;
    if (delta_time > 0.2f) delta_time = 0.01f; // �������ʱ���
    last_time = current_time;

    // ת��Ϊg��λ (��4g����, 8192 LSB/g)
    float ax = accel_x / 8192.0f;
    float ay = accel_y / 8192.0f;
    float az = accel_z / 8192.0f;

    // ������ٶȷ�ֵ�����ھ�ֹ��⣩
    float accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);

    // ����Ƿ�ֹ���������ٶȡ�1g��
    if (fabsf(accel_magnitude - 1.0f) < STATIONARY_THRESHOLD) {
        if (is_stationary == 0) {
            is_stationary = 1;
            stationary_start_time = current_time;
        }
        // ��ʱ�侲ֹ�������ƫ
        else if (current_time - stationary_start_time > STATIONARY_DURATION) {
            // ��ͨ�˲�������ƫ
            float current_bias = gyro_z / 65.5f;
            gyro_z_bias = 0.98f * gyro_z_bias + 0.02f * current_bias;
        }
    }
    else {
        is_stationary = 0;
    }

    // �������Ǻ͸�����
    roll = atan2f(ay, az) * 180.0f / M_PI;
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    // ����ƫ���ǣ������ǻ��֣�����ȥ��ƫ
    float gyro_z_dps = (gyro_z / 65.5f) - gyro_z_bias;
    yaw += gyro_z_dps * delta_time;

    // ���ƽǶȷ�Χ��0-360��
    yaw = fmodf(yaw, 360.0f);
    if (yaw < 0.0f) yaw += 360.0f;
}

/**
  * @brief  ����IMU���ݵ�������ƫ����Ϊ��λ��
  * @retval None
  */
void Send_IMU_Data_Bluetooth(void) {
    char bluetooth_buf[256];


    // JSON��ʽ�����angles����˳��Ϊ[yaw, roll, pitch]
    int len = snprintf(bluetooth_buf, sizeof(bluetooth_buf),
        "\"angles\":[%.1f,%.1f,%.1f],\n\"accel\":[%d,%d,%d],\n\"gyro\":[%d,%d,%d]\r\n",
        yaw, roll, pitch,  // ƫ����Ϊ��λ
        accel_x, accel_y, accel_z,
        gyro_x, gyro_y, gyro_z
        );

    // ͨ����������
    HAL_UART_Transmit(&huart1, (uint8_t*)bluetooth_buf, len, 100);
}

/**
  * @brief  Ӧ�ó������
  * @retval int
  */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    // 添加中断优先级设置:
HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);    // 降低雷达串口中断优先级
HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 0); // 降低雷达DMA优先级
HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);   // 提高I2C事件中断优先级
HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1, 0);   // 提高I2C错误中断优先级

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_I2C1_Init();

    // === 调试: 启动信息 ===
    HAL_Delay(100);  // 等待串口稳定
    char startup_msg[] = "\r\n=== STM32 STARTED (v2025-10-21) ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)startup_msg, strlen(startup_msg), 1000);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // 启动蓝牙接收中断
    HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);

    // 初始化MPU6500
    uint8_t mpu_ok = 0;
    uint8_t mpu_id = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, WHO_AM_I, 1, &mpu_id, 1, 100) == HAL_OK && mpu_id == 0x70) {
        MPU6500_Init(&hi2c1);
        MPU6500_Calibrate(&hi2c1);
        mpu_ok = 1;
    } else {
        char msg[50];
        snprintf(msg, sizeof(msg), "MPU6500 Skip (ID:0x%02X)\r\n", mpu_id);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
        yaw = 0.0f;  // 默认值
    }

    // ��ʼ����ƫУ׼��ر���
    gyro_z_bias = 0.0f;
    is_stationary = 0;
    stationary_start_time = 0;

    /* ========== 新增: 初始化补充模块 ========== */
    // 初始化位姿控制器
    PoseController_Init(&g_pose_controller, &g_pose_config);

    // 初始化里程计
    Odometry_Init(&g_odometry, &g_odom_config);

    // 初始化通信协议
    Comm_InitRxBuffer(&g_comm_rx_buffer);

    // 初始化安全模块
    Safety_Init(&g_safety_config);
    g_safety_status.last_command_time = HAL_GetTick();

    // ���������״�
    HAL_Delay(50);
    uint8_t startCmd[] = { 0xA5, 0x20 }; // RPLIDAR��������
    HAL_UART_Transmit(&huart6, startCmd, sizeof(startCmd), 100);
    uint8_t setScanMode[] = {0xA5, 0x60, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00}; // 降低扫描速率
HAL_UART_Transmit(&huart6, setScanMode, sizeof(setScanMode), 100);
HAL_Delay(50); // 等待设置生效
    HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);

    // ��ʼ��������ֵ
    lastEncoderA = __HAL_TIM_GET_COUNTER(&htim2);
    lastEncoderB = __HAL_TIM_GET_COUNTER(&htim4);

    // ��ʼ��PID������
    pid_left.Kp = PID_KP;
    pid_left.Ki = PID_KI;
    pid_left.Kd = PID_KD;
    pid_left.integral = 0.0f;
    pid_left.prev_error = 0.0f;
    pid_left.integral_limit = PID_INTEGRAL_LIMIT;

    pid_right.Kp = PID_KP;
    pid_right.Ki = PID_KI;
    pid_right.Kd = PID_KD;
    pid_right.integral = 0.0f;
    pid_right.prev_error = 0.0f;
    pid_right.integral_limit = PID_INTEGRAL_LIMIT;

    uint32_t last_imu_time = HAL_GetTick();
    uint32_t last_debug_time = HAL_GetTick();
    uint32_t heartbeat_time = HAL_GetTick();

    while (1) {
    uint32_t current_time = HAL_GetTick(); // 获取当前系统时间(ms)

    /* ========== 新增: 安全检查 (最高优先级) ========== */
    Safety_Update(&g_safety_status);
    if (g_safety_status.estop_active || g_safety_status.watchdog_expired) {
        // 【调试】输出watchdog状态(每5秒一次)
        static uint32_t last_warn = 0;
        if (current_time - last_warn >= 5000) {
            char warn[80];
            snprintf(warn, sizeof(warn), "[WARN] Safety block! estop=%d wd=%d last_cmd=%lu\r\n",
                    g_safety_status.estop_active, g_safety_status.watchdog_expired,
                    g_safety_status.last_command_time);
            HAL_UART_Transmit(&huart1, (uint8_t*)warn, strlen(warn), 100);
            last_warn = current_time;
        }
        Safety_EmergencyStop();
        HAL_Delay(5);
        continue;  // 跳过后续控制
    }

    // ============= 1. 高优先级任务：必须实时处理 =============
    // (0) 心跳包 (每1秒,用于调试)
    if (current_time - heartbeat_time >= 1000) {
        char hb[30];
        snprintf(hb, sizeof(hb), "HB:%lu\r\n", current_time);
        HAL_UART_Transmit(&huart1, (uint8_t*)hb, strlen(hb), 100);
        heartbeat_time = current_time;
    }

    // (1) 蓝牙连接状态检查
    SendConnectionNotification();

    /* ========== 新增: 里程计更新和发送 (30Hz) ========== */
    static uint32_t last_odom_update = 0;
    static int32_t last_encoderA = 0;
    static int32_t last_encoderB = 0;

    if (current_time - last_odom_update >= 33) {  // 约30Hz
        int32_t encoderA = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t encoderB = __HAL_TIM_GET_COUNTER(&htim4);

        // 【修复】处理16位计数器溢出
        // 如果计数器从高值跳到低值，说明发生了溢出
        int32_t deltaA = encoderA - last_encoderA;
        int32_t deltaB = encoderB - last_encoderB;

        // 如果差值超过32768（16位的一半），说明发生了溢出
        if (deltaA > 32768) deltaA -= 65536;
        else if (deltaA < -32768) deltaA += 65536;

        if (deltaB > 32768) deltaB -= 65536;
        else if (deltaB < -32768) deltaB += 65536;

        // 累积编码器值（处理溢出后）
        static int32_t accum_encoderA = 0;
        static int32_t accum_encoderB = 0;
        accum_encoderA += deltaA;
        accum_encoderB += deltaB;

        last_encoderA = encoderA;
        last_encoderB = encoderB;

        // 更新里程计（使用累积值）
        Odometry_Update(&g_odometry, accum_encoderA, accum_encoderB);

        // 构建并发送里程计数据给PC
        OdometryData_t odom_data;
        odom_data.timestamp = current_time;
        Odometry_GetPose(&g_odometry, &odom_data.x, &odom_data.y, &odom_data.theta);
        Odometry_GetVelocity(&g_odometry, &odom_data.v_linear, &odom_data.v_angular);
        Comm_SendOdometry(&huart1, &odom_data);

        // DEBUG: 每秒打印一次编码器和里程计数据
        static uint32_t last_encoder_debug = 0;
        if (current_time - last_encoder_debug >= 1000) {
            char dbg[120];
            snprintf(dbg, sizeof(dbg), "[ENC-DBG] A=%ld B=%ld | Odom: x=%.3f y=%.3f θ=%.3f | V: lin=%.3f ang=%.3f\r\n",
                    encoderA, encoderB, odom_data.x, odom_data.y, odom_data.theta,
                    odom_data.v_linear, odom_data.v_angular);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
            last_encoder_debug = current_time;
        }

        // 更新位姿控制器的当前位姿
        PoseController_UpdateCurrent(&g_pose_controller,
                                    odom_data.x, odom_data.y, odom_data.theta);

        last_odom_update = current_time;
    }

    /* ========== 新增: 位姿控制更新 (50Hz) ========== */
    static uint32_t last_pose_ctrl = 0;
    if (current_time - last_pose_ctrl >= 20 && g_pose_controller.is_active) {
        float dt = 0.02f;  // 50Hz = 20ms = 0.02s
        MotorOutput_t motor_out = PoseController_Update(&g_pose_controller, dt);

        if (!PoseController_IsReached(&g_pose_controller)) {
            // 限制PWM值
            uint16_t left_pwm = Safety_LimitPWM(fabsf(motor_out.left_speed), g_safety_config.max_pwm);
            uint16_t right_pwm = Safety_LimitPWM(fabsf(motor_out.right_speed), g_safety_config.max_pwm);

            // 应用PWM
            if (motor_out.direction == 1) {  // 前进
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_pwm);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_pwm);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            } else if (motor_out.direction == 2) {  // 后退
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
            } else {  // 转向
                if (motor_out.left_speed > 0) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_pwm);
                } else {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                }
                if (motor_out.right_speed > 0) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_pwm);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                } else {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
                }
            }
        } else {
            // 到达目标,停止
            Safety_EmergencyStop();
            PoseController_Enable(&g_pose_controller, 0);
            HAL_UART_Transmit(&huart1, (uint8_t*)"Pose reached!\r\n", 15, 100);
        }
        last_pose_ctrl = current_time;
    }

    // (2) 电机速度斜坡控制（每10ms执行一次）
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL) {
        UpdateSpeedRamp();
        last_speed_update_time = current_time;
    }

    // (3) 自动转向控制（如果处于自动转向模式）
    if (auto_turn_mode != 0) {
        HandleAutoTurn();
    }

    // ============= 2. 中优先级任务：传感器数据采集 =============
    // (1) IMU数据处理（每100ms固定周期）- 仅在MPU6500可用时
    if (mpu_ok && current_time - last_imu_time >= 100) {
        MPU6500_Read_All(&hi2c1);      // 读取MPU6500原始数据
        Calculate_Angles();             // 计算姿态角

        /* ========== 修改: 使用新通信协议发送IMU数据 ========== */
        ImuData_t imu_data;
        imu_data.timestamp = current_time;
        imu_data.yaw = yaw;
        imu_data.roll = roll;
        imu_data.pitch = pitch;
        imu_data.accel[0] = accel_x;
        imu_data.accel[1] = accel_y;
        imu_data.accel[2] = accel_z;
        imu_data.gyro[0] = gyro_x;
        imu_data.gyro[1] = gyro_y;
        imu_data.gyro[2] = gyro_z;
        Comm_SendImuData(&huart1, &imu_data);

        last_imu_time = current_time;
    }

    // (2) 编码器数据处理（每50ms固定周期）
    if (current_time - last_encoder_time >= 50) {
        int32_t encoderA = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t encoderB = __HAL_TIM_GET_COUNTER(&htim4);
        int32_t deltaA = encoderA - lastEncoderA;
        int32_t deltaB = encoderB - lastEncoderB;
        lastEncoderA = encoderA;
        lastEncoderB = encoderB;

        // 计算电机RPM（仅在前进/后退时启用PID）
        if (current_direction == DIR_FORWARD || current_direction == DIR_BACKWARD) {
            float rpmA = (float)deltaA / PPR * (60000.0f / 50); // 50ms采样周期
            float rpmB = (float)deltaB / PPR * (60000.0f / 50);

            // PID计算
            float pid_output_left = PID_Update(&pid_left, target_rpm, rpmA);
            float pid_output_right = PID_Update(&pid_right, target_rpm, rpmB);

            pwm_left = fmaxf(0, fminf(current_speed + pid_output_left, MAX_SPEED));
            pwm_right = fmaxf(0, fminf(current_speed + pid_output_right, MAX_SPEED_RIGHT));

            // 调试输出（每200ms一次）
            if (current_time - last_debug_time > 200) {
                snprintf(uart_buf, sizeof(uart_buf),
                    "PID: Tgt=%.1f, RPM_A=%.1f, RPM_B=%.1f, PWM_L=%d, PWM_R=%d\n",
                    target_rpm, rpmA, rpmB, (int)pwm_left, (int)pwm_right);
                HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
                last_debug_time = current_time;
            }
        }
        last_encoder_time = current_time;
    }

    // ============= 3. 低优先级任务：雷达数据处理 =============
    // (1) 限制雷达数据处理频率（每次最多处理5个数据包）
    if (lidar_process_packet) {
        lidar_process_packet = 0;
        uint16_t data_length = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        uint8_t processed_packets = 0;

        for (uint16_t i = 0; i <= data_length - sizeof(LidarDataPacket) && processed_packets < 5; i++) {
            if (lidar_dma_buffer[i] & 0x80) { // 检查同步位
                LidarDataPacket packet;
                memcpy(&packet, &lidar_dma_buffer[i], sizeof(LidarDataPacket));
                ProcessLidarData((uint8_t*)&packet);
                i += sizeof(LidarDataPacket) - 1; // 跳过已处理的数据包
                processed_packets++;
            }
        }

        // 重置DMA接收
        __HAL_DMA_DISABLE(huart6.hdmarx);
        huart6.hdmarx->Instance->NDTR = DMA_BUFFER_SIZE;
        __HAL_DMA_ENABLE(huart6.hdmarx);
    }

    // 【新增】(2) 按顺序输出激光雷达数据（每10ms输出一个角度）
    if (current_time - last_output_time >= 10) {
        OutputAngleDataInOrder();
        last_output_time = current_time;
    }

    // ============= 4. 空闲时延时 =============
    HAL_Delay(5); // 降低CPU占用率
}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

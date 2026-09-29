/**
 * @file motor_control.h
 * @brief 电压开环、虚拟角度电流环和编码器角度电流环的统一控制入口。
 *
 * 上层只能通过本接口提交命令和模式请求，不能直接修改 TIM8 CCR。模式切换
 * 在固定控制边界生效，确保开环与电流闭环不会同时写 PWM。
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32h7xx_hal.h"

#include "biss_frame.h"
#include "motor_runtime_policy.h"

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_INVALID_CONFIG,
    MOTOR_FAULT_DRIVER,
    MOTOR_FAULT_ADC_START,
    MOTOR_FAULT_CURRENT_CALIBRATION,
    MOTOR_FAULT_ADC_RANGE,
    MOTOR_FAULT_ADC_TIMEOUT,
    MOTOR_FAULT_OVERCURRENT,
    MOTOR_FAULT_CONTROL_MATH,
    MOTOR_FAULT_ENCODER_NOT_READY,
    MOTOR_FAULT_ENCODER_STALE,
    MOTOR_FAULT_ENCODER_FRAME,
    MOTOR_FAULT_ENCODER_CRC,
    MOTOR_FAULT_ENCODER_STATUS,
    MOTOR_FAULT_ENCODER_NOT_CALIBRATED,
    MOTOR_FAULT_ENCODER_ALIGNMENT,
    MOTOR_FAULT_CONFIG_STORAGE
} MotorFaultCode;

typedef struct
{
    /** 编译时启动模式；运行中切换必须调用MotorControl_RequestMode系列接口。 */
    MotorControlMode mode;
    /** 模式1/2虚拟电角频率斜坡，单位Hz/s；模式3/4不使用。 */
    float frequency_slew_hz_per_s;
    /** 电流闭环退回模式1时，Ud/Uq回到开环目标的最大变化率，单位pu/s。 */
    float voltage_slew_pu_per_s;
} MotorControlConfig;

/**
 * @brief Keil/Ozone 可直接观察的稳定调试符号。
 * @note 仅用于观察，应用代码不应直接写这些字段。
 */
typedef struct
{
    volatile MotorControlMode mode;
    volatile MotorControlMode requested_mode;
    volatile MotorRunState run_state;
    volatile MotorFaultCode fault;
    volatile uint32_t phase_a_raw;
    volatile uint32_t phase_b_raw;
    volatile float phase_a_offset;
    volatile float phase_b_offset;
    volatile float id_a;
    volatile float iq_a;
    volatile float id_ref_a;
    volatile float iq_ref_a;
    volatile float electrical_angle_pu;
    volatile float speed_target_rpm;
    volatile float speed_active_target_rpm;
    volatile float speed_filtered_rpm;
    volatile float speed_iq_command_a;
    volatile uint8_t speed_pi_saturated;
    volatile float position_target_deg;
    volatile float position_feedback_deg;
    volatile float position_error_deg;
    volatile uint32_t adc_age_ticks;
    volatile uint32_t overcurrent_count;
    volatile uint8_t current_sense_ready;
    volatile uint8_t voltage_saturated;
    volatile uint8_t encoder_ready;
    volatile uint8_t encoder_calibrated;
    volatile BissFrameStatus encoder_frame_status;
    volatile uint32_t encoder_position_raw;
    volatile uint32_t encoder_age_ticks;
    volatile uint32_t encoder_crc_error_count;
    volatile uint32_t encoder_frame_error_count;
    volatile uint32_t encoder_spi_error_count;
    volatile uint32_t encoder_timeout_count;
    volatile uint32_t encoder_dma_guard_error_count;
} MotorControlDebug;

extern volatile MotorControlDebug g_motor_control_debug;

/**
 * 初始化软件状态、参数、Flash校准记录和硬件适配对象，但不使能PWM。
 * 调用顺序必须是BissEncoder_Init -> MotorControl_Init -> 设置三套命令
 * -> MotorControl_Start。
 */
HAL_StatusTypeDef MotorControl_Init(const MotorControlConfig *config);

/** 设置电压开环 dq 标幺电压和目标电角频率。 */
void MotorControl_SetOpenLoopCommand(float ud_pu,
                                     float uq_pu,
                                     float electrical_frequency_hz);

/** 设置电流闭环 d/q 电流和目标电角频率，电流会限幅到安全范围。 */
void MotorControl_SetCurrentCommand(float id_a,
                                    float iq_a,
                                    float electrical_frequency_hz);

/** 设置编码器角度电流闭环的 d/q 电流，不包含虚拟角频率。 */
void MotorControl_SetEncoderCurrentCommand(float id_a, float iq_a);

/** 设置模式4的电机轴机械转速目标，单位rpm，内部限制到安全范围。 */
void MotorControl_SetSpeedCommand(float mechanical_speed_rpm);

/** 设置单圈轴侧目标角 [0, 360) 度；非法角度不更改旧目标。
 * 模式 6 启动前可预设一次目标；未预设时启动会捕获并保持当前位置。
 * 运行中仍要求模式 6、有效编码器和就绪的电流/位置控制。 */
HAL_StatusTypeDef MotorControl_SetPositionCommand(float target_deg);

/**
 * 设置命令并请求切换到电压开环模式。
 * 模式切换在下一个 10 kHz 控制边界生效。
 */
HAL_StatusTypeDef MotorControl_SwitchToOpenVoltage(
    float ud_pu,
    float uq_pu,
    float electrical_frequency_hz);

/**
 * 设置命令并请求切换到虚拟电角度、电流闭环模式。
 * Id/Iq 会被限制在 MOTOR_CURRENT_COMMAND_LIMIT_A 范围内。
 */
HAL_StatusTypeDef MotorControl_SwitchToOpenAngleCurrent(
    float id_a,
    float iq_a,
    float electrical_frequency_hz);

/**
 * 设置命令并请求切换到编码器电角度、电流闭环模式。
 * 编码器未校准、数据未就绪或已过期时返回 HAL_ERROR。
 */
HAL_StatusTypeDef MotorControl_SwitchToEncoderAngleCurrent(float id_a,
                                                           float iq_a);

/** 设置速度目标并请求切换到编码器速度/电流双闭环模式。 */
HAL_StatusTypeDef MotorControl_SwitchToEncoderSpeedCurrent(
    float mechanical_speed_rpm);

/** 请求位置模式；生效时捕获当前轴侧角度作为保持目标。 */
HAL_StatusTypeDef MotorControl_SwitchToEncoderPositionCurrent(void);

/** 在线设置速度PI；参数单位依次为A/rpm、A/(rpm*s)和1/s。 */
HAL_StatusTypeDef MotorControl_SetSpeedPiGains(float kp_a_per_rpm,
                                               float ki_a_per_rpm_s,
                                               float kaw_per_s);

/** 在线缩小或恢复速度PI的Iq限幅，但不能突破首版0.6 A安全上限。 */
HAL_StatusTypeDef MotorControl_SetSpeedIqLimit(float iq_limit_a);

/** 请求执行一次低电流编码器方向/电角度零点校准。 */
HAL_StatusTypeDef MotorControl_RequestEncoderCalibration(void);

/** 主循环前台服务：启动校准请求并在停机后保存 Flash，禁止放入中断。 */
void MotorControl_Service(void);

/**
 * 请求在下一个10 kHz控制边界切换模式。
 * @note 本函数只发布requested_mode，不直接写PWM；实际切换由FastTick执行。
 * @note 模式3还要求电流反馈有效、编码器ready且校准记录有效。
 */
HAL_StatusTypeDef MotorControl_RequestMode(MotorControlMode mode);

/** 设置电流 PI；异常值会被拒绝，参数组在关中断的短临界区内更新。 */
HAL_StatusTypeDef MotorControl_SetCurrentPiGains(float kp_v_per_a,
                                                 float ki_v_per_a_s,
                                                 float kaw_per_s);

/** 设置电流环电压矢量上限，范围为 0～0.45 pu。 */
HAL_StatusTypeDef MotorControl_SetCurrentVoltageLimit(float limit_pu);

/** 启动ADC零偏校准、DRV8323和实时触发链；PWM在校准通过后才使能。 */
HAL_StatusTypeDef MotorControl_Start(void);
/** 先关功率级，再停止TIM8/ADC实时链路；不会自动重新启动。 */
HAL_StatusTypeDef MotorControl_Stop(void);
/** 仅在FAULT状态使用：执行安全停机并清除锁存故障。 */
HAL_StatusTypeDef MotorControl_ClearFault(void);

/**
 * TIM8更新中断入口：编码器调度、模式切换、角度推进和模式1电压输出。
 * 电流PI不在这里执行，而在ADC注入完成入口执行。
 */
void MotorControl_FastTick(float dt_s);

/**
 * ADC注入序列完成入口：电流零偏/编码器校准，或模式2/3电流闭环。
 * 两个ADC原始值必须来自同一次TIM8 CH4触发的A相、B相注入序列。
 */
void MotorControl_CurrentSampleComplete(uint32_t phase_a_raw,
                                        uint32_t phase_b_raw,
                                        float dt_s);

#endif /* MOTOR_CONTROL_H */

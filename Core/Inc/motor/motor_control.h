/**
 * @file motor_control.h
 * @brief 电压开环与“角度开环、电流闭环”的统一控制入口。
 *
 * 上层只能通过本接口提交命令和模式请求，不能直接修改 TIM8 CCR。模式切换
 * 在固定控制边界生效，确保开环与电流闭环不会同时写 PWM。
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32h7xx_hal.h"

typedef enum
{
    MOTOR_CONTROL_STOPPED = 0,
    MOTOR_CONTROL_OPEN_VOLTAGE,
    MOTOR_CONTROL_OPEN_ANGLE_CURRENT,
    MOTOR_CONTROL_ENCODER_CURRENT,
    MOTOR_CONTROL_FAULT
} MotorControlMode;

/* 兼容旧代码中的模式名称。 */
#define MOTOR_CONTROL_OPEN_LOOP MOTOR_CONTROL_OPEN_VOLTAGE

typedef enum
{
    MOTOR_RUN_STATE_STOPPED = 0,
    MOTOR_RUN_STATE_CURRENT_CALIBRATING,
    MOTOR_RUN_STATE_READY,
    MOTOR_RUN_STATE_ALIGNING,
    MOTOR_RUN_STATE_RUNNING,
    MOTOR_RUN_STATE_FAULT
} MotorRunState;

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
    MOTOR_FAULT_CONTROL_MATH
} MotorFaultCode;

typedef struct
{
    MotorControlMode mode;             /**< 上电校准结束后进入的模式。 */
    float frequency_slew_hz_per_s;    /**< 开环电角频率斜坡，Hz/s。 */
    float voltage_slew_pu_per_s;      /**< 闭环切回开环时的电压过渡斜坡。 */
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
    volatile float ia_a;
    volatile float ib_a;
    volatile float ic_a;
    volatile float i_alpha_a;
    volatile float i_beta_a;
    volatile float id_a;
    volatile float iq_a;
    volatile float id_ref_a;
    volatile float iq_ref_a;
    volatile float id_error_a;
    volatile float iq_error_a;
    volatile float ud_integrator_v;
    volatile float uq_integrator_v;
    volatile float ud_v;
    volatile float uq_v;
    volatile float ud_pu;
    volatile float uq_pu;
    volatile float electrical_angle_pu;
    volatile float electrical_frequency_hz;
    volatile float current_kp_v_per_a;
    volatile float current_ki_v_per_a_s;
    volatile float current_kaw_per_s;
    volatile float current_voltage_limit_pu;
    volatile uint32_t calibration_sample_count;
    volatile uint32_t adc_age_ticks;
    volatile uint32_t overcurrent_count;
    volatile uint8_t current_sense_ready;
    volatile uint8_t voltage_saturated;
} MotorControlDebug;

extern volatile MotorControlDebug g_motor_control_debug;

HAL_StatusTypeDef MotorControl_Init(const MotorControlConfig *config);

/** 设置电压开环 dq 标幺电压和目标电角频率。 */
void MotorControl_SetOpenLoopCommand(float ud_pu,
                                     float uq_pu,
                                     float electrical_frequency_hz);

/** 设置电流闭环 d/q 电流和目标电角频率，电流会限幅到安全范围。 */
void MotorControl_SetCurrentCommand(float id_a,
                                    float iq_a,
                                    float electrical_frequency_hz);

/** 请求在下一个 10 kHz 控制边界切换模式。 */
HAL_StatusTypeDef MotorControl_RequestMode(MotorControlMode mode);

/** 设置电流 PI；异常值会被拒绝，参数组在关中断的短临界区内更新。 */
HAL_StatusTypeDef MotorControl_SetCurrentPiGains(float kp_v_per_a,
                                                 float ki_v_per_a_s,
                                                 float kaw_per_s);

/** 设置电流环电压矢量上限，范围为 0～0.45 pu。 */
HAL_StatusTypeDef MotorControl_SetCurrentVoltageLimit(float limit_pu);

HAL_StatusTypeDef MotorControl_Start(void);
HAL_StatusTypeDef MotorControl_Stop(void);
HAL_StatusTypeDef MotorControl_ClearFault(void);

/** TIM8 更新中断入口：推进统一电角度，并在电压开环模式更新 PWM。 */
void MotorControl_FastTick(float dt_s);

/** ADC 注入序列完成入口：校准或执行电流闭环。 */
void MotorControl_CurrentSampleComplete(uint32_t phase_a_raw,
                                        uint32_t phase_b_raw,
                                        float dt_s);

#endif /* MOTOR_CONTROL_H */

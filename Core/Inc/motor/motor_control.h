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
#include "encoder_calibration.h"
#include "motor_runtime_policy.h"
#include "motor_startup_trace.h"

/* 兼容旧代码中的模式名称。 */
#define MOTOR_CONTROL_OPEN_LOOP MOTOR_CONTROL_OPEN_VOLTAGE
#define MOTOR_CONTROL_ENCODER_CURRENT MOTOR_CONTROL_ENCODER_ANGLE_CURRENT

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
    volatile float speed_target_rpm;
    volatile float speed_active_target_rpm;
    volatile float speed_raw_rpm;
    volatile float speed_filtered_rpm;
    volatile float speed_error_rpm;
    volatile float speed_pi_proportional_a;
    volatile float speed_pi_integrator_a;
    volatile float speed_iq_command_a;
    volatile uint32_t speed_control_tick_count;
    volatile uint8_t speed_pi_saturated;
    volatile uint8_t speed_estimator_ready;
    volatile float current_kp_v_per_a;
    volatile float current_ki_v_per_a_s;
    volatile float current_kaw_per_s;
    volatile float current_voltage_limit_pu;
    volatile uint32_t calibration_sample_count;
    volatile uint32_t adc_age_ticks;
    volatile uint32_t overcurrent_count;
    volatile uint8_t current_sense_ready;
    volatile uint8_t voltage_saturated;
    volatile uint8_t encoder_ready;
    volatile uint8_t encoder_warning;
    volatile uint8_t encoder_calibrated;
    volatile uint8_t encoder_raw[6];
    volatile uint8_t encoder_received_crc;
    volatile uint8_t encoder_calculated_crc;
    volatile int8_t encoder_direction;
    volatile BissFrameStatus encoder_frame_status;
    volatile EncoderCalibrationState encoder_calibration_state;
    volatile EncoderCalibrationFailure encoder_calibration_failure;
    volatile uint32_t encoder_position_raw;
    volatile uint32_t encoder_sequence;
    volatile uint32_t encoder_age_ticks;
    volatile uint32_t encoder_valid_count;
    volatile uint32_t encoder_crc_error_count;
    volatile uint32_t encoder_frame_error_count;
    volatile uint32_t encoder_spi_error_count;
    volatile uint32_t encoder_timeout_count;
    volatile uint32_t encoder_dma_guard_error_count;
    volatile uint32_t encoder_electrical_zero_raw;
    volatile float encoder_mechanical_angle_pu;
    volatile float encoder_electrical_angle_pu;
    volatile uint8_t startup_trace_count;
    volatile uint8_t startup_trace_stage[MOTOR_STARTUP_TRACE_CAPACITY];
    volatile uint8_t startup_trace_mode[MOTOR_STARTUP_TRACE_CAPACITY];
    volatile uint8_t startup_invalid_detected;
    volatile uint8_t first_invalid_requested_mode;
    volatile uint8_t first_invalid_startup_stage;
    volatile uint32_t first_invalid_calibration_sample;
    volatile uint32_t mode_integrity_error_count;
    /*
     * Flash保存前台轨迹：0=空闲，1=中断已请求，2=Service已进入，
     * 3=外设已停，4=正在写Flash，5=成功，6=停机失败，7=写入/校验失败。
     */
    volatile uint8_t encoder_save_stage;
    volatile uint8_t encoder_save_hal_status;
    volatile uint32_t foreground_service_count;
    volatile uint32_t service_count_at_save_request;
    /*
     * 校准终止路径轨迹：kind 0=无、1=完成待保存、2=校准失败。
     * stage 1=中断检测到终止，2=功率级已关，3=实时中断已静默，
     * 4=即将退出ADC回调，10=Service已进入失败分支，
     * 11=Stop已返回，12=已进入编码器对齐故障。
     */
    volatile uint8_t encoder_terminal_kind;
    volatile uint8_t encoder_terminal_stage;
    volatile uint8_t encoder_terminal_hal_status;
    volatile uint32_t service_count_at_terminal;
    /* Stop轨迹：1=进入，2=功率级已关，3/4=CH4停止前/后，
     * 5=TIM8基准已停，6/7=ADC停止前/后，8=全部完成。 */
    volatile uint8_t motor_stop_stage;
    volatile uint8_t motor_stop_hal_status;
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

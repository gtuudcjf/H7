/**
 * @file motor_runtime_policy.h
 * @brief 电机状态机与编码器后台采集之间的纯策略函数。
 */
#ifndef MOTOR_RUNTIME_POLICY_H
#define MOTOR_RUNTIME_POLICY_H

#include <stdbool.h>

typedef enum
{
    MOTOR_CONTROL_STOPPED = 0,
    MOTOR_CONTROL_OPEN_VOLTAGE,
    MOTOR_CONTROL_OPEN_ANGLE_CURRENT,
    MOTOR_CONTROL_ENCODER_ANGLE_CURRENT,
    MOTOR_CONTROL_FAULT
} MotorControlMode;

typedef enum
{
    MOTOR_RUN_STATE_STOPPED = 0,
    MOTOR_RUN_STATE_CURRENT_CALIBRATING,
    MOTOR_RUN_STATE_READY,
    MOTOR_RUN_STATE_ALIGNING,
    MOTOR_RUN_STATE_ENCODER_CALIBRATING,
    MOTOR_RUN_STATE_RUNNING,
    MOTOR_RUN_STATE_FAULT
} MotorRunState;

typedef enum
{
    MOTOR_START_ACTION_INVALID = 0,
    MOTOR_START_ACTION_OPEN_VOLTAGE,
    MOTOR_START_ACTION_CURRENT_CONTROL
} MotorStartAction;

/**
 * @brief 判断当前状态是否允许启动一次编码器 SPI4 DMA 传输。
 *
 * 电流零偏校准是功率级启动的安全关键阶段。该阶段只允许 ADC/TIM8
 * 参与，避免新增外设 DMA 改变已经验证过的电流采样启动时序。
 */
bool MotorRuntimePolicy_EncoderAcquisitionAllowed(MotorRunState run_state);

/** 将启动模式分类；损坏或未实现的值必须返回INVALID，禁止静默降级。 */
MotorStartAction MotorRuntimePolicy_ClassifyStartMode(MotorControlMode mode);

/**
 * @brief 当前已进入有效控制模式时才允许处理运行时模式切换请求。
 * @note STOPPED用于上电电流校准，绝不能把待启动模式回退成STOPPED。
 */
bool MotorRuntimePolicy_ModeRequestAllowed(MotorControlMode active_mode);

#endif /* MOTOR_RUNTIME_POLICY_H */

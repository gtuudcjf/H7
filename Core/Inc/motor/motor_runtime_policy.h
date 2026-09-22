/**
 * @file motor_runtime_policy.h
 * @brief 电机状态机与编码器后台采集之间的纯策略函数。
 */
#ifndef MOTOR_RUNTIME_POLICY_H
#define MOTOR_RUNTIME_POLICY_H

#include <stdbool.h>

typedef enum
{
    /* 功率级未运行；上电电流零偏校准期间motor_mode也暂时保持此值。 */
    MOTOR_CONTROL_STOPPED = 0,
    /* 模式1：虚拟电角度，Ud/Uq由上层直接给定，不闭合电流环。 */
    MOTOR_CONTROL_OPEN_VOLTAGE,
    /* 模式2：虚拟电角度，Id/Iq经PI生成Ud/Uq。 */
    MOTOR_CONTROL_OPEN_ANGLE_CURRENT,
    /* 模式3：编码器电角度，Id/Iq经与模式2相同的PI生成Ud/Uq。 */
    MOTOR_CONTROL_ENCODER_ANGLE_CURRENT,
    /* 模式4：编码器机械速度PI生成Iq，复用模式3的编码器电角度和电流PI。 */
    MOTOR_CONTROL_ENCODER_SPEED_CURRENT,
    /* 锁存故障状态；必须先停机并显式清故障，不能直接请求其他模式。 */
    MOTOR_CONTROL_FAULT
} MotorControlMode;

/*
 * MotorControlMode回答“现在使用哪种控制算法”；MotorRunState回答“启动或
 * 校准流程走到哪一步”。例如模式2启动初期，mode已经是模式2，而
 * run_state可能仍是ALIGNING。调试时必须同时观察这两个枚举。
 */
typedef enum
{
    MOTOR_RUN_STATE_STOPPED = 0,
    /* DRV_CAL有效，正在丢弃/平均ADC样本，此时三相PWM尚未使能。 */
    MOTOR_RUN_STATE_CURRENT_CALIBRATING,
    /* 电流零偏有效；模式3可能在此等待第一批新鲜编码器帧。 */
    MOTOR_RUN_STATE_READY,
    /* 模式2先用固定d轴电流建立初始磁场，再进入RUNNING。 */
    MOTOR_RUN_STATE_ALIGNING,
    /* 正在执行编码器方向/电角度零点校准状态机。 */
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

/** 将速度模式的q轴活动参考限制到当前运行时转矩电流边界。 */
float MotorRuntimePolicy_ClampSpeedIq(float iq_a, float limit_a);

#endif /* MOTOR_RUNTIME_POLICY_H */

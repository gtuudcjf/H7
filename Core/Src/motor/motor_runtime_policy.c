/**
 * @file motor_runtime_policy.c
 * @brief 电机运行阶段的后台外设调度策略。
 */
#include "motor_runtime_policy.h"

bool MotorRuntimePolicy_EncoderAcquisitionAllowed(MotorRunState run_state)
{
    return (run_state == MOTOR_RUN_STATE_READY) ||
           (run_state == MOTOR_RUN_STATE_ALIGNING) ||
           (run_state == MOTOR_RUN_STATE_ENCODER_CALIBRATING) ||
           (run_state == MOTOR_RUN_STATE_RUNNING);
}

MotorStartAction MotorRuntimePolicy_ClassifyStartMode(MotorControlMode mode)
{
    if (mode == MOTOR_CONTROL_OPEN_VOLTAGE)
    {
        return MOTOR_START_ACTION_OPEN_VOLTAGE;
    }
    if ((mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ||
        (mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT))
    {
        return MOTOR_START_ACTION_CURRENT_CONTROL;
    }
    return MOTOR_START_ACTION_INVALID;
}

bool MotorRuntimePolicy_ModeRequestAllowed(MotorControlMode active_mode)
{
    return MotorRuntimePolicy_ClassifyStartMode(active_mode) !=
           MOTOR_START_ACTION_INVALID;
}

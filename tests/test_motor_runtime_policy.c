/**
 * @file test_motor_runtime_policy.c
 * @brief 电机启动阶段与编码器后台采集之间的隔离策略测试。
 */
#include <assert.h>
#include <stdio.h>

#include "motor_runtime_policy.h"

static void Test_EncoderDmaIsBlockedWhileCurrentOffsetsAreCalibrated(void)
{
    assert(!MotorRuntimePolicy_EncoderAcquisitionAllowed(
        MOTOR_RUN_STATE_CURRENT_CALIBRATING));
}

static void Test_EncoderDmaIsAllowedAfterCurrentCalibration(void)
{
    assert(MotorRuntimePolicy_EncoderAcquisitionAllowed(
        MOTOR_RUN_STATE_READY));
    assert(MotorRuntimePolicy_EncoderAcquisitionAllowed(
        MOTOR_RUN_STATE_ALIGNING));
    assert(MotorRuntimePolicy_EncoderAcquisitionAllowed(
        MOTOR_RUN_STATE_ENCODER_CALIBRATING));
    assert(MotorRuntimePolicy_EncoderAcquisitionAllowed(
        MOTOR_RUN_STATE_RUNNING));
}

static void Test_EncoderDmaIsBlockedWhenStoppedOrFaulted(void)
{
    assert(!MotorRuntimePolicy_EncoderAcquisitionAllowed(
        MOTOR_RUN_STATE_STOPPED));
    assert(!MotorRuntimePolicy_EncoderAcquisitionAllowed(
        MOTOR_RUN_STATE_FAULT));
}

static void Test_InvalidRequestedModeCannotFallBackToVoltageOpenLoop(void)
{
    assert(MotorRuntimePolicy_ClassifyStartMode(MOTOR_CONTROL_OPEN_VOLTAGE) ==
           MOTOR_START_ACTION_OPEN_VOLTAGE);
    assert(MotorRuntimePolicy_ClassifyStartMode(MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ==
           MOTOR_START_ACTION_CURRENT_CONTROL);
    assert(MotorRuntimePolicy_ClassifyStartMode(MOTOR_CONTROL_ENCODER_ANGLE_CURRENT) ==
           MOTOR_START_ACTION_CURRENT_CONTROL);
    assert(MotorRuntimePolicy_ClassifyStartMode(MOTOR_CONTROL_ENCODER_SPEED_CURRENT) ==
           MOTOR_START_ACTION_CURRENT_CONTROL);
    assert(MOTOR_CONTROL_FAULT == 5);
    assert(MOTOR_CONTROL_ENCODER_POSITION_CURRENT == 6);
    assert(MotorRuntimePolicy_ClassifyStartMode(MOTOR_CONTROL_ENCODER_POSITION_CURRENT) ==
           MOTOR_START_ACTION_CURRENT_CONTROL);
    assert(MotorRuntimePolicy_ClassifyStartMode(MOTOR_CONTROL_STOPPED) ==
           MOTOR_START_ACTION_INVALID);
    assert(MotorRuntimePolicy_ClassifyStartMode(MOTOR_CONTROL_FAULT) ==
           MOTOR_START_ACTION_INVALID);
    assert(MotorRuntimePolicy_ClassifyStartMode((MotorControlMode)0x55U) ==
           MOTOR_START_ACTION_INVALID);
}

static void Test_ModeRequestsRunOnlyFromAnActiveControlMode(void)
{
    assert(!MotorRuntimePolicy_ModeRequestAllowed(MOTOR_CONTROL_STOPPED));
    assert(MotorRuntimePolicy_ModeRequestAllowed(MOTOR_CONTROL_OPEN_VOLTAGE));
    assert(MotorRuntimePolicy_ModeRequestAllowed(MOTOR_CONTROL_OPEN_ANGLE_CURRENT));
    assert(MotorRuntimePolicy_ModeRequestAllowed(MOTOR_CONTROL_ENCODER_ANGLE_CURRENT));
    assert(MotorRuntimePolicy_ModeRequestAllowed(MOTOR_CONTROL_ENCODER_SPEED_CURRENT));
    assert(MotorRuntimePolicy_ModeRequestAllowed(MOTOR_CONTROL_ENCODER_POSITION_CURRENT));
    assert(!MotorRuntimePolicy_ModeRequestAllowed(MOTOR_CONTROL_FAULT));
}

static void Test_SpeedCurrentHandoffIsAlwaysClampedToItsRuntimeLimit(void)
{
    assert(MotorRuntimePolicy_ClampSpeedIq(2.0f, 0.6f) == 0.6f);
    assert(MotorRuntimePolicy_ClampSpeedIq(-2.0f, 0.6f) == -0.6f);
    assert(MotorRuntimePolicy_ClampSpeedIq(0.3f, 0.6f) == 0.3f);
}

int main(void)
{
    Test_EncoderDmaIsBlockedWhileCurrentOffsetsAreCalibrated();
    Test_EncoderDmaIsAllowedAfterCurrentCalibration();
    Test_EncoderDmaIsBlockedWhenStoppedOrFaulted();
    Test_InvalidRequestedModeCannotFallBackToVoltageOpenLoop();
    Test_ModeRequestsRunOnlyFromAnActiveControlMode();
    Test_SpeedCurrentHandoffIsAlwaysClampedToItsRuntimeLimit();

    puts("motor runtime policy tests passed");
    return 0;
}

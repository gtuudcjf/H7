/**
 * @file test_motor_startup_trace.c
 * @brief 启动模式完整性诊断的主机无关行为测试。
 */
#include <assert.h>
#include <stdio.h>

#include "motor_startup_trace.h"

static void Test_CheckpointsPreserveStageAndObservedMode(void)
{
    MotorStartupTrace trace;

    MotorStartupTrace_Init(&trace, 2U);
    MotorStartupTrace_RecordCheckpoint(&trace, MOTOR_STARTUP_STAGE_INIT_DONE, 2U);
    MotorStartupTrace_RecordCheckpoint(&trace, MOTOR_STARTUP_STAGE_ADC_CALIBRATED, 2U);

    assert(trace.checkpoint_count == 2U);
    assert(trace.checkpoint_stage[0] == MOTOR_STARTUP_STAGE_INIT_DONE);
    assert(trace.checkpoint_mode[0] == 2U);
    assert(trace.checkpoint_stage[1] == MOTOR_STARTUP_STAGE_ADC_CALIBRATED);
    assert(trace.checkpoint_mode[1] == 2U);
}

static void Test_FirstInvalidModeAndSampleCannotBeOverwritten(void)
{
    MotorStartupTrace trace;

    MotorStartupTrace_Init(&trace, 2U);
    MotorStartupTrace_ObserveCalibration(&trace, 2U, 0U);
    MotorStartupTrace_ObserveCalibration(&trace, 0x55U, 37U);
    MotorStartupTrace_ObserveCalibration(&trace, 4U, 256U);

    assert(trace.invalid_detected);
    assert(trace.first_invalid_mode == 0x55U);
    assert(trace.first_invalid_sample == 37U);
    assert(trace.integrity_error_count == 2U);
}

static void Test_ValidModeNeverRaisesIntegrityError(void)
{
    MotorStartupTrace trace;
    uint32_t sample;

    MotorStartupTrace_Init(&trace, 2U);
    for (sample = 0U; sample <= 256U; ++sample)
    {
        MotorStartupTrace_ObserveCalibration(&trace, 2U, sample);
    }

    assert(!trace.invalid_detected);
    assert(trace.integrity_error_count == 0U);
}

int main(void)
{
    Test_CheckpointsPreserveStageAndObservedMode();
    Test_FirstInvalidModeAndSampleCannotBeOverwritten();
    Test_ValidModeNeverRaisesIntegrityError();

    puts("motor startup trace tests passed");
    return 0;
}

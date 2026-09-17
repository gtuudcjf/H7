/**
 * @file motor_startup_trace.c
 * @brief 电机启动模式完整性诊断实现。
 */
#include "motor_startup_trace.h"

#include <limits.h>
#include <string.h>

static void MotorStartupTrace_Observe(MotorStartupTrace *trace,
                                      MotorStartupStage stage,
                                      uint8_t observed_mode,
                                      uint32_t sample_count)
{
    if ((trace == 0) || (observed_mode == trace->expected_mode))
    {
        return;
    }

    if (!trace->invalid_detected)
    {
        trace->invalid_detected = true;
        trace->first_invalid_mode = observed_mode;
        trace->first_invalid_stage = (uint8_t)stage;
        trace->first_invalid_sample = sample_count;
    }
    if (trace->integrity_error_count < UINT32_MAX)
    {
        ++trace->integrity_error_count;
    }
}

void MotorStartupTrace_Init(MotorStartupTrace *trace, uint8_t expected_mode)
{
    if (trace == 0)
    {
        return;
    }

    memset(trace, 0, sizeof(*trace));
    trace->expected_mode = expected_mode;
    trace->first_invalid_sample = MOTOR_STARTUP_SAMPLE_UNKNOWN;
}

void MotorStartupTrace_RecordCheckpoint(MotorStartupTrace *trace,
                                        MotorStartupStage stage,
                                        uint8_t observed_mode)
{
    uint8_t index;

    if (trace == 0)
    {
        return;
    }

    index = trace->checkpoint_count;
    if (index < MOTOR_STARTUP_TRACE_CAPACITY)
    {
        trace->checkpoint_stage[index] = (uint8_t)stage;
        trace->checkpoint_mode[index] = observed_mode;
        trace->checkpoint_count = (uint8_t)(index + 1U);
    }
    MotorStartupTrace_Observe(trace,
                              stage,
                              observed_mode,
                              MOTOR_STARTUP_SAMPLE_UNKNOWN);
}

void MotorStartupTrace_ObserveCalibration(MotorStartupTrace *trace,
                                          uint8_t observed_mode,
                                          uint32_t sample_count)
{
    MotorStartupTrace_Observe(trace,
                              MOTOR_STARTUP_STAGE_CURRENT_SAMPLE,
                              observed_mode,
                              sample_count);
}

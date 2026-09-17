/**
 * @file motor_startup_trace.h
 * @brief 保存电机启动期间模式完整性的首个异常证据。
 */
#ifndef MOTOR_STARTUP_TRACE_H
#define MOTOR_STARTUP_TRACE_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_STARTUP_TRACE_CAPACITY (10U)
#define MOTOR_STARTUP_SAMPLE_UNKNOWN (UINT32_MAX)

typedef enum
{
    MOTOR_STARTUP_STAGE_NONE = 0,
    MOTOR_STARTUP_STAGE_INIT_DONE,
    MOTOR_STARTUP_STAGE_START_ENTER,
    MOTOR_STARTUP_STAGE_ADC_CALIBRATED,
    MOTOR_STARTUP_STAGE_DRIVER_ENABLED,
    MOTOR_STARTUP_STAGE_CURRENT_CALIBRATION_STARTED,
    MOTOR_STARTUP_STAGE_ADC_STARTED,
    MOTOR_STARTUP_STAGE_TIM8_STARTED,
    MOTOR_STARTUP_STAGE_SAMPLING_STARTED,
    MOTOR_STARTUP_STAGE_CURRENT_SAMPLE,
    MOTOR_STARTUP_STAGE_BEFORE_MODE_SELECT
} MotorStartupStage;

typedef struct
{
    uint8_t expected_mode;
    uint8_t checkpoint_count;
    uint8_t checkpoint_stage[MOTOR_STARTUP_TRACE_CAPACITY];
    uint8_t checkpoint_mode[MOTOR_STARTUP_TRACE_CAPACITY];
    uint8_t first_invalid_mode;
    uint8_t first_invalid_stage;
    bool invalid_detected;
    uint32_t first_invalid_sample;
    uint32_t integrity_error_count;
} MotorStartupTrace;

void MotorStartupTrace_Init(MotorStartupTrace *trace, uint8_t expected_mode);
void MotorStartupTrace_RecordCheckpoint(MotorStartupTrace *trace,
                                        MotorStartupStage stage,
                                        uint8_t observed_mode);
void MotorStartupTrace_ObserveCalibration(MotorStartupTrace *trace,
                                          uint8_t observed_mode,
                                          uint32_t sample_count);

#endif /* MOTOR_STARTUP_TRACE_H */

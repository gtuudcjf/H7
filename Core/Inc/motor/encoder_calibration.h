/**
 * @file encoder_calibration.h
 * @brief 编码器方向和 FOC 电角度零点的纯算法校准状态机。
 */
#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ENCODER_CAL_IDLE = 0,
    ENCODER_CAL_WAIT_VALID,
    ENCODER_CAL_ALIGN_ZERO,
    ENCODER_CAL_SETTLE_ZERO,
    ENCODER_CAL_DIRECTION_MOVE,
    ENCODER_CAL_SETTLE_FINAL,
    ENCODER_CAL_COMPLETE,
    ENCODER_CAL_FAILED,
    /* 保留上面已用于调试的 0..7 数值，新阶段从 8 开始追加。 */
    ENCODER_CAL_PREALIGN_MOVE,
    ENCODER_CAL_PREALIGN_RETURN,
    ENCODER_CAL_RETURN_ZERO,
    ENCODER_CAL_SETTLE_RETURN,
    ENCODER_CAL_RELEASE_CURRENT
} EncoderCalibrationState;

typedef enum
{
    ENCODER_CAL_FAILURE_NONE = 0,
    ENCODER_CAL_FAILURE_ARGUMENT,
    ENCODER_CAL_FAILURE_CURRENT_NOT_READY,
    ENCODER_CAL_FAILURE_ENCODER_MISSING,
    ENCODER_CAL_FAILURE_UNSTABLE,
    ENCODER_CAL_FAILURE_MOVEMENT_RANGE,
    ENCODER_CAL_FAILURE_TIMEOUT,
    ENCODER_CAL_FAILURE_RETURN_MISMATCH
} EncoderCalibrationFailure;

typedef struct
{
    bool current_sense_ready;
    bool encoder_valid;
    uint32_t position_raw;
} EncoderCalibrationInput;

typedef struct
{
    float id_ref_a;
    float iq_ref_a;
    float forced_electrical_angle_pu;
    bool active;
} EncoderCalibrationCommand;

typedef struct
{
    uint32_t electrical_zero_raw;
    int8_t encoder_direction;
} EncoderCalibrationResult;

typedef struct
{
    EncoderCalibrationState state;
    EncoderCalibrationFailure failure;
    float state_elapsed_s;
    uint32_t invalid_sample_ticks;
    uint32_t sample_count;
    uint32_t sample_reference_raw;
    int64_t sample_delta_sum;
    int32_t sample_delta_min;
    int32_t sample_delta_max;
    uint32_t zero_average_raw;
    uint32_t final_average_raw;
    uint32_t return_average_raw;
    int32_t direction_movement_count;
    int32_t return_error_count;
    float release_electrical_angle_pu;
    bool release_success;
    EncoderCalibrationResult result;
    bool result_valid;
} EncoderCalibration;

void EncoderCalibration_Init(EncoderCalibration *calibration);
bool EncoderCalibration_Start(EncoderCalibration *calibration);
EncoderCalibrationCommand EncoderCalibration_Step(
    EncoderCalibration *calibration,
    const EncoderCalibrationInput *input,
    float dt_s);
bool EncoderCalibration_GetResult(const EncoderCalibration *calibration,
                                  EncoderCalibrationResult *result);

#endif /* ENCODER_CALIBRATION_H */

/**
 * @file test_encoder_calibration.c
 * @brief 编码器方向和电角度零点校准状态机的主机侧单元测试。
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "encoder_calibration.h"
#include "motor_params.h"

#define TEST_DT_S      (0.0001f)
#define TEST_EPSILON   (0.002f)

static EncoderCalibrationInput ValidInput(uint32_t position)
{
    EncoderCalibrationInput input;

    input.current_sense_ready = true;
    input.encoder_valid = true;
    input.position_raw = position;
    return input;
}

static EncoderCalibrationCommand Step(EncoderCalibration *calibration,
                                      uint32_t position)
{
    const EncoderCalibrationInput input = ValidInput(position);

    return EncoderCalibration_Step(calibration, &input, TEST_DT_S);
}

static void AdvanceUntilState(EncoderCalibration *calibration,
                              EncoderCalibrationState target,
                              uint32_t position,
                              uint32_t maximum_steps)
{
    uint32_t index;

    for (index = 0U; index < maximum_steps; ++index)
    {
        if (calibration->state == target)
        {
            return;
        }
        (void)Step(calibration, position);
    }
    assert(calibration->state == target);
}

static void AdvanceToDirectionMove(EncoderCalibration *calibration,
                                   uint32_t zero_position)
{
    assert(EncoderCalibration_Start(calibration));
    AdvanceUntilState(calibration, ENCODER_CAL_ALIGN_ZERO, zero_position, 4U);
    AdvanceUntilState(calibration, ENCODER_CAL_PREALIGN_MOVE, zero_position, 2500U);
    AdvanceUntilState(calibration, ENCODER_CAL_PREALIGN_RETURN, zero_position, 5000U);
    AdvanceUntilState(calibration, ENCODER_CAL_SETTLE_ZERO, zero_position, 5000U);
    AdvanceUntilState(calibration, ENCODER_CAL_DIRECTION_MOVE, zero_position, 6000U);
}

static void AdvanceDirectionAndReturn(EncoderCalibration *calibration,
                                      uint32_t final_position,
                                      uint32_t return_position)
{
    AdvanceUntilState(calibration, ENCODER_CAL_SETTLE_FINAL,
                      final_position, 5000U);
    AdvanceUntilState(calibration, ENCODER_CAL_RETURN_ZERO,
                      final_position, 6000U);
    AdvanceUntilState(calibration, ENCODER_CAL_SETTLE_RETURN,
                      return_position, 4000U);
    AdvanceUntilState(calibration, ENCODER_CAL_COMPLETE,
                      return_position, 6000U);
}

static void Test_AlignmentCurrentRampsAndIqRemainsZero(void)
{
    EncoderCalibration calibration;
    EncoderCalibrationCommand command;
    uint32_t index;

    EncoderCalibration_Init(&calibration);
    assert(EncoderCalibration_Start(&calibration));
    command = Step(&calibration, 1000U);
    assert(calibration.state == ENCODER_CAL_ALIGN_ZERO);
    assert(command.id_ref_a == 0.0f);
    assert(command.iq_ref_a == 0.0f);

    for (index = 0U; index < 2000U; ++index)
    {
        command = Step(&calibration, 1000U);
    }
    assert(fabsf(command.id_ref_a -
                 (0.5f * MOTOR_ENCODER_ALIGN_CURRENT_A)) < TEST_EPSILON);
    assert(command.iq_ref_a == 0.0f);

    AdvanceUntilState(&calibration, ENCODER_CAL_PREALIGN_MOVE, 1000U, 2100U);
    command = Step(&calibration, 1000U);
    assert(fabsf(command.id_ref_a - MOTOR_ENCODER_ALIGN_CURRENT_A) < TEST_EPSILON);
    assert(command.forced_electrical_angle_pu >= 0.0f);
}

static void Test_PositiveAndNegativeMovementDetermineDirection(void)
{
    EncoderCalibration calibration;
    EncoderCalibrationResult result;

    EncoderCalibration_Init(&calibration);
    AdvanceToDirectionMove(&calibration, 10000U);
    AdvanceDirectionAndReturn(&calibration, 11311U, 10000U);
    assert(EncoderCalibration_GetResult(&calibration, &result));
    assert(result.electrical_zero_raw == 10000U);
    assert(result.encoder_direction == 1);
    assert(calibration.direction_movement_count == 1311);
    assert(calibration.return_error_count == 0);

    EncoderCalibration_Init(&calibration);
    AdvanceToDirectionMove(&calibration, 10000U);
    AdvanceDirectionAndReturn(&calibration, 8689U, 10000U);
    assert(EncoderCalibration_GetResult(&calibration, &result));
    assert(result.electrical_zero_raw == 10000U);
    assert(result.encoder_direction == -1);
    assert(calibration.direction_movement_count == -1311);
}

static void Test_DirectionAngleRampsBeforeFinalSettling(void)
{
    EncoderCalibration calibration;
    EncoderCalibrationCommand command = {0};
    uint32_t index;

    EncoderCalibration_Init(&calibration);
    AdvanceToDirectionMove(&calibration, 10000U);
    for (index = 0U; index < 2000U; ++index)
    {
        command = Step(&calibration, 10655U);
    }

    assert(calibration.state == ENCODER_CAL_DIRECTION_MOVE);
    assert(command.active);
    assert(fabsf(command.forced_electrical_angle_pu -
                 (0.5f * MOTOR_ENCODER_DIRECTION_STEP_PU)) < TEST_EPSILON);
    assert(fabsf(command.id_ref_a - MOTOR_ENCODER_ALIGN_CURRENT_A) < TEST_EPSILON);
    assert(command.iq_ref_a == 0.0f);
}

static void Test_CircularAverageHandlesPositionWrap(void)
{
    EncoderCalibration calibration;
    EncoderCalibrationResult result;
    uint32_t index;

    EncoderCalibration_Init(&calibration);
    assert(EncoderCalibration_Start(&calibration));
    AdvanceUntilState(&calibration, ENCODER_CAL_SETTLE_ZERO, 131071U, 12000U);

    /* 完成500 ms等待，然后让128个样本在131071/0之间交替。 */
    for (index = 0U; index < 5000U; ++index)
    {
        (void)Step(&calibration, 131071U);
    }
    for (index = 0U; index < MOTOR_ENCODER_ALIGN_SAMPLE_COUNT; ++index)
    {
        (void)Step(&calibration, (index & 1U) ? 0U : 131071U);
    }
    assert(calibration.state == ENCODER_CAL_DIRECTION_MOVE);

    AdvanceDirectionAndReturn(&calibration, 1310U, 0U);
    assert(EncoderCalibration_GetResult(&calibration, &result));
    assert((result.electrical_zero_raw <= 1U) ||
           (result.electrical_zero_raw >= 131070U));
}

static void Test_ReturnMismatchRejectsNonRepeatableZero(void)
{
    EncoderCalibration calibration;

    EncoderCalibration_Init(&calibration);
    AdvanceToDirectionMove(&calibration, 10000U);
    AdvanceUntilState(&calibration, ENCODER_CAL_SETTLE_FINAL, 11311U, 5000U);
    AdvanceUntilState(&calibration, ENCODER_CAL_RETURN_ZERO, 11311U, 6000U);
    AdvanceUntilState(&calibration, ENCODER_CAL_SETTLE_RETURN, 10600U, 4000U);
    AdvanceUntilState(&calibration, ENCODER_CAL_FAILED, 10600U, 10000U);
    assert(calibration.failure == ENCODER_CAL_FAILURE_RETURN_MISMATCH);
    assert(calibration.return_error_count == 600);
    assert(!calibration.result_valid);
}

static void Test_MovementOutsideAllowedRangeFails(void)
{
    EncoderCalibration calibration;

    EncoderCalibration_Init(&calibration);
    AdvanceToDirectionMove(&calibration, 10000U);
    AdvanceUntilState(&calibration, ENCODER_CAL_SETTLE_FINAL, 10100U, 5000U);
    AdvanceUntilState(&calibration, ENCODER_CAL_FAILED, 10100U, 10000U);
    assert(calibration.failure == ENCODER_CAL_FAILURE_MOVEMENT_RANGE);
    assert(calibration.final_average_raw == 10100U);
    assert(calibration.direction_movement_count == 100);

    EncoderCalibration_Init(&calibration);
    AdvanceToDirectionMove(&calibration, 10000U);
    AdvanceUntilState(&calibration, ENCODER_CAL_SETTLE_FINAL, 15000U, 5000U);
    AdvanceUntilState(&calibration, ENCODER_CAL_FAILED, 15000U, 10000U);
    assert(calibration.failure == ENCODER_CAL_FAILURE_MOVEMENT_RANGE);
}

static void Test_UnstableAndMissingSamplesFailSafely(void)
{
    EncoderCalibration calibration;
    EncoderCalibrationInput invalid = {true, false, 0U};
    EncoderCalibrationCommand command;
    uint32_t index;

    EncoderCalibration_Init(&calibration);
    assert(EncoderCalibration_Start(&calibration));
    for (index = 0U; index <= MOTOR_ENCODER_CAL_INVALID_LIMIT_TICKS; ++index)
    {
        command = EncoderCalibration_Step(&calibration, &invalid, TEST_DT_S);
    }
    assert(calibration.state == ENCODER_CAL_FAILED);
    assert(calibration.failure == ENCODER_CAL_FAILURE_ENCODER_MISSING);
    assert(command.id_ref_a == 0.0f);
    assert(command.iq_ref_a == 0.0f);

    EncoderCalibration_Init(&calibration);
    assert(EncoderCalibration_Start(&calibration));
    AdvanceUntilState(&calibration, ENCODER_CAL_SETTLE_ZERO, 10000U, 12000U);
    for (index = 0U; index < 5000U; ++index)
    {
        (void)Step(&calibration, 10000U);
    }
    for (index = 0U; index < MOTOR_ENCODER_ALIGN_SAMPLE_COUNT; ++index)
    {
        (void)Step(&calibration, (index & 1U) ? 10129U : 10000U);
    }
    assert(calibration.state == ENCODER_CAL_FAILED);
    assert(calibration.failure == ENCODER_CAL_FAILURE_UNSTABLE);
}

static void Test_InvalidTimingAndStateTimeoutFail(void)
{
    EncoderCalibration calibration;
    const EncoderCalibrationInput input = ValidInput(1000U);
    EncoderCalibrationCommand command;

    EncoderCalibration_Init(&calibration);
    assert(EncoderCalibration_Start(&calibration));
    command = EncoderCalibration_Step(
        &calibration, &input, MOTOR_ENCODER_CAL_STATE_TIMEOUT_S + 0.1f);
    assert(calibration.state == ENCODER_CAL_FAILED);
    assert(calibration.failure == ENCODER_CAL_FAILURE_TIMEOUT);
    assert(command.id_ref_a == 0.0f);

    EncoderCalibration_Init(&calibration);
    assert(EncoderCalibration_Start(&calibration));
    command = EncoderCalibration_Step(&calibration, &input, 0.0f);
    assert(calibration.state == ENCODER_CAL_FAILED);
    assert(calibration.failure == ENCODER_CAL_FAILURE_ARGUMENT);
    assert(command.id_ref_a == 0.0f);
}

int main(void)
{
    Test_AlignmentCurrentRampsAndIqRemainsZero();
    Test_PositiveAndNegativeMovementDetermineDirection();
    Test_DirectionAngleRampsBeforeFinalSettling();
    Test_CircularAverageHandlesPositionWrap();
    Test_MovementOutsideAllowedRangeFails();
    Test_ReturnMismatchRejectsNonRepeatableZero();
    Test_UnstableAndMissingSamplesFailSafely();
    Test_InvalidTimingAndStateTimeoutFail();

    puts("encoder-calibration tests passed");
    return 0;
}

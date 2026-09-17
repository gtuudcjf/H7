/**
 * @file encoder_calibration.c
 * @brief 低电流定向、圆周均值零点采样和编码器方向辨识。
 */
#include "encoder_calibration.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "biss_frame.h"
#include "motor_params.h"

#define ENCODER_COUNTS_PER_TURN     (131072L)
#define ENCODER_HALF_TURN_COUNTS    (65536L)

static EncoderCalibrationCommand EncoderCalibration_ZeroCommand(void)
{
    EncoderCalibrationCommand command = {0.0f, 0.0f, 0.0f, false};

    return command;
}

static EncoderCalibrationCommand EncoderCalibration_AlignmentCommand(
    float id_ref_a,
    float electrical_angle_pu)
{
    EncoderCalibrationCommand command;

    if (id_ref_a < 0.0f)
    {
        id_ref_a = 0.0f;
    }
    else if (id_ref_a > MOTOR_ENCODER_ALIGN_CURRENT_MAX_A)
    {
        id_ref_a = MOTOR_ENCODER_ALIGN_CURRENT_MAX_A;
    }
    command.id_ref_a = id_ref_a;
    command.iq_ref_a = 0.0f;
    command.forced_electrical_angle_pu = electrical_angle_pu;
    command.active = true;
    return command;
}

static void EncoderCalibration_SetState(EncoderCalibration *calibration,
                                        EncoderCalibrationState state)
{
    calibration->state = state;
    calibration->state_elapsed_s = 0.0f;
}

static EncoderCalibrationCommand EncoderCalibration_Fail(
    EncoderCalibration *calibration,
    EncoderCalibrationFailure failure)
{
    calibration->failure = failure;
    calibration->result_valid = false;
    EncoderCalibration_SetState(calibration, ENCODER_CAL_FAILED);
    return EncoderCalibration_ZeroCommand();
}

static int32_t EncoderCalibration_CircularDelta(uint32_t position,
                                                uint32_t reference)
{
    int32_t delta = (int32_t)position - (int32_t)reference;

    if (delta > ENCODER_HALF_TURN_COUNTS)
    {
        delta -= ENCODER_COUNTS_PER_TURN;
    }
    else if (delta < -ENCODER_HALF_TURN_COUNTS)
    {
        delta += ENCODER_COUNTS_PER_TURN;
    }
    return delta;
}

static void EncoderCalibration_BeginSamples(EncoderCalibration *calibration)
{
    calibration->sample_count = 0U;
    calibration->sample_reference_raw = 0U;
    calibration->sample_delta_sum = 0;
    calibration->sample_delta_min = INT32_MAX;
    calibration->sample_delta_max = INT32_MIN;
}

static bool EncoderCalibration_AddSample(EncoderCalibration *calibration,
                                         uint32_t position,
                                         uint32_t *average_raw)
{
    int32_t delta;
    int64_t average;

    if (calibration->sample_count == 0U)
    {
        calibration->sample_reference_raw = position;
    }

    delta = EncoderCalibration_CircularDelta(
        position, calibration->sample_reference_raw);
    calibration->sample_delta_sum += delta;
    if (delta < calibration->sample_delta_min)
    {
        calibration->sample_delta_min = delta;
    }
    if (delta > calibration->sample_delta_max)
    {
        calibration->sample_delta_max = delta;
    }
    ++calibration->sample_count;

    if (calibration->sample_count < MOTOR_ENCODER_ALIGN_SAMPLE_COUNT)
    {
        return false;
    }

    if ((calibration->sample_delta_max - calibration->sample_delta_min) >
        (int32_t)MOTOR_ENCODER_ALIGN_STABILITY_COUNT)
    {
        EncoderCalibration_Fail(calibration, ENCODER_CAL_FAILURE_UNSTABLE);
        return false;
    }

    average = (int64_t)calibration->sample_reference_raw +
              (calibration->sample_delta_sum /
               (int64_t)MOTOR_ENCODER_ALIGN_SAMPLE_COUNT);
    while (average < 0)
    {
        average += ENCODER_COUNTS_PER_TURN;
    }
    while (average >= ENCODER_COUNTS_PER_TURN)
    {
        average -= ENCODER_COUNTS_PER_TURN;
    }
    *average_raw = (uint32_t)average;
    return true;
}

void EncoderCalibration_Init(EncoderCalibration *calibration)
{
    if (calibration != 0)
    {
        memset(calibration, 0, sizeof(*calibration));
        calibration->state = ENCODER_CAL_IDLE;
    }
}

bool EncoderCalibration_Start(EncoderCalibration *calibration)
{
    if (calibration == 0)
    {
        return false;
    }

    EncoderCalibration_Init(calibration);
    EncoderCalibration_SetState(calibration, ENCODER_CAL_WAIT_VALID);
    return true;
}

EncoderCalibrationCommand EncoderCalibration_Step(
    EncoderCalibration *calibration,
    const EncoderCalibrationInput *input,
    float dt_s)
{
    EncoderCalibrationCommand command = EncoderCalibration_ZeroCommand();
    uint32_t average_raw;

    if ((calibration == 0) || (input == 0))
    {
        return command;
    }
    if (!isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return EncoderCalibration_Fail(calibration, ENCODER_CAL_FAILURE_ARGUMENT);
    }
    if ((calibration->state == ENCODER_CAL_IDLE) ||
        (calibration->state == ENCODER_CAL_COMPLETE) ||
        (calibration->state == ENCODER_CAL_FAILED))
    {
        return command;
    }

    calibration->state_elapsed_s += dt_s;
    if (calibration->state_elapsed_s > MOTOR_ENCODER_CAL_STATE_TIMEOUT_S)
    {
        return EncoderCalibration_Fail(calibration, ENCODER_CAL_FAILURE_TIMEOUT);
    }
    if (!input->current_sense_ready)
    {
        return EncoderCalibration_Fail(
            calibration, ENCODER_CAL_FAILURE_CURRENT_NOT_READY);
    }
    if (!input->encoder_valid || (input->position_raw > BISS_POSITION_MAX))
    {
        if (calibration->invalid_sample_ticks < UINT32_MAX)
        {
            ++calibration->invalid_sample_ticks;
        }
        if (calibration->invalid_sample_ticks > MOTOR_ENCODER_CAL_INVALID_LIMIT_TICKS)
        {
            return EncoderCalibration_Fail(
                calibration, ENCODER_CAL_FAILURE_ENCODER_MISSING);
        }
    }
    else
    {
        calibration->invalid_sample_ticks = 0U;
    }

    switch (calibration->state)
    {
        case ENCODER_CAL_WAIT_VALID:
            if (input->encoder_valid && (input->position_raw <= BISS_POSITION_MAX))
            {
                EncoderCalibration_SetState(calibration, ENCODER_CAL_ALIGN_ZERO);
            }
            break;

        case ENCODER_CAL_ALIGN_ZERO:
        {
            float ramp = calibration->state_elapsed_s / MOTOR_ENCODER_ALIGN_RAMP_S;

            if (ramp > 1.0f)
            {
                ramp = 1.0f;
            }
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A * ramp, 0.0f);
            if (calibration->state_elapsed_s >= MOTOR_ENCODER_ALIGN_RAMP_S)
            {
                EncoderCalibration_SetState(calibration, ENCODER_CAL_PREALIGN_MOVE);
            }
            break;
        }

        case ENCODER_CAL_PREALIGN_MOVE:
        {
            float ramp = calibration->state_elapsed_s /
                         MOTOR_ENCODER_PREALIGN_RAMP_S;

            if (ramp > 1.0f)
            {
                ramp = 1.0f;
            }
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A,
                MOTOR_ENCODER_PREALIGN_STEP_PU * ramp);
            if (calibration->state_elapsed_s >= MOTOR_ENCODER_PREALIGN_RAMP_S)
            {
                EncoderCalibration_SetState(calibration, ENCODER_CAL_PREALIGN_RETURN);
            }
            break;
        }

        case ENCODER_CAL_PREALIGN_RETURN:
        {
            float ramp = calibration->state_elapsed_s /
                         MOTOR_ENCODER_PREALIGN_RAMP_S;

            if (ramp > 1.0f)
            {
                ramp = 1.0f;
            }
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A,
                MOTOR_ENCODER_PREALIGN_STEP_PU * (1.0f - ramp));
            if (calibration->state_elapsed_s >= MOTOR_ENCODER_PREALIGN_RAMP_S)
            {
                EncoderCalibration_BeginSamples(calibration);
                EncoderCalibration_SetState(calibration, ENCODER_CAL_SETTLE_ZERO);
            }
            break;
        }

        case ENCODER_CAL_SETTLE_ZERO:
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A, 0.0f);
            if ((calibration->state_elapsed_s >= MOTOR_ENCODER_ALIGN_SETTLE_S) &&
                input->encoder_valid &&
                EncoderCalibration_AddSample(calibration, input->position_raw, &average_raw))
            {
                calibration->zero_average_raw = average_raw;
                EncoderCalibration_SetState(calibration, ENCODER_CAL_DIRECTION_MOVE);
            }
            break;

        case ENCODER_CAL_DIRECTION_MOVE:
        {
            float ramp = calibration->state_elapsed_s /
                         MOTOR_ENCODER_DIRECTION_RAMP_S;

            if (ramp > 1.0f)
            {
                ramp = 1.0f;
            }
            /*
             * 保持 d 轴对齐电流不变，用 200 ms 缓慢推进定子电角度，避免
             * 直接跳变 0.1 pu（36 电角度）造成机械冲击或电流瞬态。
             */
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A,
                MOTOR_ENCODER_DIRECTION_STEP_PU * ramp);
            if (calibration->state_elapsed_s >= MOTOR_ENCODER_DIRECTION_RAMP_S)
            {
                EncoderCalibration_BeginSamples(calibration);
                EncoderCalibration_SetState(calibration, ENCODER_CAL_SETTLE_FINAL);
            }
            break;
        }

        case ENCODER_CAL_SETTLE_FINAL:
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A, MOTOR_ENCODER_DIRECTION_STEP_PU);
            if ((calibration->state_elapsed_s >= MOTOR_ENCODER_ALIGN_SETTLE_S) &&
                input->encoder_valid &&
                EncoderCalibration_AddSample(calibration, input->position_raw, &average_raw))
            {
                const int32_t movement = EncoderCalibration_CircularDelta(
                    average_raw, calibration->zero_average_raw);
                const int32_t magnitude = (movement < 0) ? -movement : movement;

                /* 先保留失败现场，使 Keil Watch 也能直接看到实际位移。 */
                calibration->final_average_raw = average_raw;
                calibration->direction_movement_count = movement;
                if ((magnitude < MOTOR_ENCODER_DIRECTION_MIN_COUNT) ||
                    (magnitude > MOTOR_ENCODER_DIRECTION_MAX_COUNT))
                {
                    /*
                     * 位移范围不合格不是电气危险故障：先保持当前
                     * 强制角并缓降 Id，再进入 FAILED，避免减速器突然回弹。
                     */
                    calibration->failure = ENCODER_CAL_FAILURE_MOVEMENT_RANGE;
                    calibration->result_valid = false;
                    calibration->release_electrical_angle_pu =
                        MOTOR_ENCODER_DIRECTION_STEP_PU;
                    calibration->release_success = false;
                    EncoderCalibration_SetState(
                        calibration, ENCODER_CAL_RELEASE_CURRENT);
                    command = EncoderCalibration_AlignmentCommand(
                        MOTOR_ENCODER_ALIGN_CURRENT_A,
                        calibration->release_electrical_angle_pu);
                    break;
                }

                calibration->result.encoder_direction = (movement > 0) ? 1 : -1;
                EncoderCalibration_SetState(calibration, ENCODER_CAL_RETURN_ZERO);
            }
            break;

        case ENCODER_CAL_RETURN_ZERO:
        {
            float ramp = calibration->state_elapsed_s /
                         MOTOR_ENCODER_RETURN_RAMP_S;

            if (ramp > 1.0f)
            {
                ramp = 1.0f;
            }
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A,
                MOTOR_ENCODER_DIRECTION_STEP_PU * (1.0f - ramp));
            if (calibration->state_elapsed_s >= MOTOR_ENCODER_RETURN_RAMP_S)
            {
                EncoderCalibration_BeginSamples(calibration);
                EncoderCalibration_SetState(calibration, ENCODER_CAL_SETTLE_RETURN);
            }
            break;
        }

        case ENCODER_CAL_SETTLE_RETURN:
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A, 0.0f);
            if ((calibration->state_elapsed_s >= MOTOR_ENCODER_ALIGN_SETTLE_S) &&
                input->encoder_valid &&
                EncoderCalibration_AddSample(calibration, input->position_raw, &average_raw))
            {
                const int32_t return_error = EncoderCalibration_CircularDelta(
                    average_raw, calibration->zero_average_raw);
                const int32_t magnitude =
                    (return_error < 0) ? -return_error : return_error;

                calibration->return_average_raw = average_raw;
                calibration->return_error_count = return_error;
                if (magnitude > MOTOR_ENCODER_RETURN_MAX_ERROR_COUNT)
                {
                    calibration->failure = ENCODER_CAL_FAILURE_RETURN_MISMATCH;
                    calibration->result_valid = false;
                    calibration->release_electrical_angle_pu = 0.0f;
                    calibration->release_success = false;
                    EncoderCalibration_SetState(
                        calibration, ENCODER_CAL_RELEASE_CURRENT);
                    command = EncoderCalibration_AlignmentCommand(
                        MOTOR_ENCODER_ALIGN_CURRENT_A, 0.0f);
                    break;
                }

                calibration->result.electrical_zero_raw =
                    calibration->zero_average_raw;
                calibration->release_electrical_angle_pu = 0.0f;
                calibration->release_success = true;
                EncoderCalibration_SetState(calibration, ENCODER_CAL_RELEASE_CURRENT);
            }
            break;

        case ENCODER_CAL_RELEASE_CURRENT:
        {
            float ramp = calibration->state_elapsed_s /
                         MOTOR_ENCODER_RELEASE_RAMP_S;

            if (ramp > 1.0f)
            {
                ramp = 1.0f;
            }
            /* 保持已校准的 0 pu 磁场方向，只缓降电流幅值。 */
            command = EncoderCalibration_AlignmentCommand(
                MOTOR_ENCODER_ALIGN_CURRENT_A * (1.0f - ramp),
                calibration->release_electrical_angle_pu);
            if (calibration->state_elapsed_s >= MOTOR_ENCODER_RELEASE_RAMP_S)
            {
                if (calibration->release_success)
                {
                    calibration->result_valid = true;
                    calibration->failure = ENCODER_CAL_FAILURE_NONE;
                    EncoderCalibration_SetState(calibration, ENCODER_CAL_COMPLETE);
                }
                else
                {
                    calibration->result_valid = false;
                    EncoderCalibration_SetState(calibration, ENCODER_CAL_FAILED);
                }
                command = EncoderCalibration_ZeroCommand();
            }
            break;
        }

        default:
            return EncoderCalibration_Fail(
                calibration, ENCODER_CAL_FAILURE_ARGUMENT);
    }

    if (calibration->state == ENCODER_CAL_FAILED)
    {
        return EncoderCalibration_ZeroCommand();
    }
    return command;
}

bool EncoderCalibration_GetResult(const EncoderCalibration *calibration,
                                  EncoderCalibrationResult *result)
{
    if ((calibration == 0) || (result == 0) ||
        (calibration->state != ENCODER_CAL_COMPLETE) ||
        !calibration->result_valid)
    {
        return false;
    }

    *result = calibration->result;
    return true;
}

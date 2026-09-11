/**
 * @file encoder_angle.c
 * @brief 17 位编码器机械角与电角度换算。
 */
#include "encoder_angle.h"

#include <math.h>

#include "biss_frame.h"

#define ENCODER_POSITION_SCALE_PU  (1.0f / 131072.0f)

static bool EncoderAngle_IsConfigValid(const EncoderAngleConfig *config)
{
    return (config != 0) &&
           (config->zero_raw <= BISS_POSITION_MAX) &&
           ((config->direction == 1) || (config->direction == -1)) &&
           (config->pole_pairs > 0U);
}

static float EncoderAngle_WrapPu(float angle_pu)
{
    float wrapped;

    if (!isfinite(angle_pu))
    {
        return 0.0f;
    }

    wrapped = angle_pu - floorf(angle_pu);
    if (wrapped >= 1.0f)
    {
        wrapped = 0.0f;
    }
    if (wrapped < 0.0f)
    {
        wrapped += 1.0f;
    }
    return wrapped;
}

bool EncoderAngle_Init(EncoderAngleConfig *config,
                       uint32_t zero_raw,
                       int8_t direction,
                       uint8_t pole_pairs)
{
    EncoderAngleConfig candidate;

    if (config == 0)
    {
        return false;
    }

    candidate.zero_raw = zero_raw;
    candidate.direction = direction;
    candidate.pole_pairs = pole_pairs;
    if (!EncoderAngle_IsConfigValid(&candidate))
    {
        return false;
    }

    *config = candidate;
    return true;
}

bool EncoderAngle_Update(const EncoderAngleConfig *config,
                         uint32_t position_raw,
                         EncoderAngleSample *sample)
{
    int32_t relative_count;
    float mechanical_angle_pu;

    if (!EncoderAngle_IsConfigValid(config) ||
        (sample == 0) ||
        (position_raw > BISS_POSITION_MAX))
    {
        return false;
    }

    relative_count = (int32_t)position_raw - (int32_t)config->zero_raw;
    mechanical_angle_pu =
        (float)(relative_count * (int32_t)config->direction) *
        ENCODER_POSITION_SCALE_PU;

    sample->position_raw = position_raw;
    sample->mechanical_angle_pu = EncoderAngle_WrapPu(mechanical_angle_pu);
    sample->electrical_angle_pu = EncoderAngle_WrapPu(
        sample->mechanical_angle_pu * (float)config->pole_pairs);
    return true;
}

/**
 * @file speed_estimator.c
 * @brief 电机轴机械转速换算、跨零处理和一阶低通滤波。
 */
#include "speed_estimator.h"

#include <math.h>

#define SPEED_ESTIMATOR_TWO_PI (6.28318530717958647692f)

static bool SpeedEstimator_ConfigIsValid(const SpeedEstimatorConfig *config)
{
    return (config != 0) &&
           (config->counts_per_turn >= 2U) &&
           ((config->direction == 1) || (config->direction == -1)) &&
           isfinite(config->filter_cutoff_hz) &&
           (config->filter_cutoff_hz > 0.0f);
}

bool SpeedEstimator_Init(SpeedEstimator *estimator,
                         const SpeedEstimatorConfig *config)
{
    if ((estimator == 0) || !SpeedEstimator_ConfigIsValid(config))
    {
        return false;
    }

    estimator->config = *config;
    SpeedEstimator_Reset(estimator);
    return true;
}

void SpeedEstimator_Reset(SpeedEstimator *estimator)
{
    if (estimator == 0)
    {
        return;
    }

    estimator->last_position_raw = 0U;
    estimator->last_sequence = 0U;
    estimator->elapsed_s = 0.0f;
    estimator->raw_rpm = 0.0f;
    estimator->filtered_rpm = 0.0f;
    estimator->has_position = false;
    estimator->ready = false;
}

SpeedEstimatorUpdateStatus SpeedEstimator_Update(
    SpeedEstimator *estimator,
    uint32_t position_raw,
    uint32_t sequence,
    float scheduler_period_s)
{
    int32_t delta_count;
    int32_t half_turn_count;
    int32_t directed_delta_count;
    float tau_s;
    float alpha;

    if ((estimator == 0) ||
        !SpeedEstimator_ConfigIsValid(&estimator->config) ||
        (position_raw >= estimator->config.counts_per_turn) ||
        !isfinite(scheduler_period_s) || (scheduler_period_s <= 0.0f))
    {
        return SPEED_ESTIMATOR_UPDATE_INVALID;
    }

    if (!estimator->has_position)
    {
        estimator->last_position_raw = position_raw;
        estimator->last_sequence = sequence;
        estimator->elapsed_s = 0.0f;
        estimator->has_position = true;
        return SPEED_ESTIMATOR_UPDATE_PRIMED;
    }

    estimator->elapsed_s += scheduler_period_s;
    if (!isfinite(estimator->elapsed_s) || (estimator->elapsed_s <= 0.0f))
    {
        return SPEED_ESTIMATOR_UPDATE_INVALID;
    }

    if (sequence == estimator->last_sequence)
    {
        return SPEED_ESTIMATOR_UPDATE_NO_NEW_SAMPLE;
    }

    delta_count = (int32_t)position_raw -
                  (int32_t)estimator->last_position_raw;
    half_turn_count = (int32_t)(estimator->config.counts_per_turn / 2U);
    if (delta_count > half_turn_count)
    {
        delta_count -= (int32_t)estimator->config.counts_per_turn;
    }
    else if (delta_count < -half_turn_count)
    {
        delta_count += (int32_t)estimator->config.counts_per_turn;
    }

    directed_delta_count = delta_count * (int32_t)estimator->config.direction;
    estimator->raw_rpm =
        ((float)directed_delta_count * 60.0f) /
        ((float)estimator->config.counts_per_turn * estimator->elapsed_s);

    tau_s = 1.0f /
            (SPEED_ESTIMATOR_TWO_PI * estimator->config.filter_cutoff_hz);
    alpha = estimator->elapsed_s / (tau_s + estimator->elapsed_s);
    estimator->filtered_rpm +=
        alpha * (estimator->raw_rpm - estimator->filtered_rpm);

    if (!isfinite(estimator->raw_rpm) ||
        !isfinite(estimator->filtered_rpm))
    {
        return SPEED_ESTIMATOR_UPDATE_INVALID;
    }

    estimator->last_position_raw = position_raw;
    estimator->last_sequence = sequence;
    estimator->elapsed_s = 0.0f;
    estimator->ready = true;
    return SPEED_ESTIMATOR_UPDATE_READY;
}

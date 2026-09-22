/**
 * @file speed_estimator.h
 * @brief 基于单圈绝对值编码器位置差分的电机轴机械转速估算器。
 */
#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t counts_per_turn;
    int8_t direction;
    float filter_cutoff_hz;
} SpeedEstimatorConfig;

typedef enum
{
    SPEED_ESTIMATOR_UPDATE_INVALID = 0,
    SPEED_ESTIMATOR_UPDATE_NO_NEW_SAMPLE,
    SPEED_ESTIMATOR_UPDATE_PRIMED,
    SPEED_ESTIMATOR_UPDATE_READY
} SpeedEstimatorUpdateStatus;

typedef struct
{
    SpeedEstimatorConfig config;
    uint32_t last_position_raw;
    uint32_t last_sequence;
    float elapsed_s;
    float raw_rpm;
    float filtered_rpm;
    bool has_position;
    bool ready;
} SpeedEstimator;

bool SpeedEstimator_Init(SpeedEstimator *estimator,
                         const SpeedEstimatorConfig *config);
void SpeedEstimator_Reset(SpeedEstimator *estimator);
SpeedEstimatorUpdateStatus SpeedEstimator_Update(
    SpeedEstimator *estimator,
    uint32_t position_raw,
    uint32_t sequence,
    float scheduler_period_s);

#endif /* SPEED_ESTIMATOR_H */

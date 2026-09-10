/**
 * @file current_sense.h
 * @brief 两路低侧采样电阻的电流换算接口。
 */
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float adc_reference_v;
    float adc_full_scale_count;
    float shunt_resistance_ohm;
    float amplifier_gain_v_per_v;
    float phase_a_polarity;
    float phase_b_polarity;
} CurrentSenseConfig;

typedef struct
{
    float phase_a_count;
    float phase_b_count;
} CurrentSenseOffsets;

typedef struct
{
    float ia_a;
    float ib_a;
    float ic_a;
} CurrentPhaseCurrents;

typedef struct
{
    uint32_t discard_remaining;
    uint32_t target_sample_count;
    uint32_t sample_count;
    uint64_t phase_a_sum;
    uint64_t phase_b_sum;
    uint32_t phase_a_min;
    uint32_t phase_a_max;
    uint32_t phase_b_min;
    uint32_t phase_b_max;
} CurrentSenseCalibration;

void CurrentSenseCalibration_Start(CurrentSenseCalibration *calibration,
                                   uint32_t discard_count,
                                   uint32_t sample_count);
bool CurrentSenseCalibration_AddSample(CurrentSenseCalibration *calibration,
                                       uint32_t phase_a_raw,
                                       uint32_t phase_b_raw);
bool CurrentSenseCalibration_GetOffsets(const CurrentSenseCalibration *calibration,
                                        float adc_full_scale_count,
                                        float rail_margin_count,
                                        uint32_t maximum_span_count,
                                        CurrentSenseOffsets *offsets);

/**
 * @brief 将 PB1/PB0 的 ADC 原始值换算为三相电流。
 * @note C 相没有采样电阻，依据三相电流和为零进行重构。
 * @return 配置和结果均有效时返回 true，否则不写出有效控制量。
 */
bool CurrentSense_Convert(const CurrentSenseConfig *config,
                          const CurrentSenseOffsets *offsets,
                          uint32_t phase_a_raw,
                          uint32_t phase_b_raw,
                          CurrentPhaseCurrents *current);

#endif /* CURRENT_SENSE_H */

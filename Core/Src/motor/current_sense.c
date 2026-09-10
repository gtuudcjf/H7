/**
 * @file current_sense.c
 * @brief 两路低侧电流采样的物理量换算与第三相重构。
 */
#include "current_sense.h"

#include <math.h>
#include <limits.h>

void CurrentSenseCalibration_Start(CurrentSenseCalibration *calibration,
                                   uint32_t discard_count,
                                   uint32_t sample_count)
{
    if (calibration == 0)
    {
        return;
    }

    calibration->discard_remaining = discard_count;
    calibration->target_sample_count = sample_count;
    calibration->sample_count = 0U;
    calibration->phase_a_sum = 0U;
    calibration->phase_b_sum = 0U;
    calibration->phase_a_min = UINT_MAX;
    calibration->phase_a_max = 0U;
    calibration->phase_b_min = UINT_MAX;
    calibration->phase_b_max = 0U;
}

bool CurrentSenseCalibration_AddSample(CurrentSenseCalibration *calibration,
                                       uint32_t phase_a_raw,
                                       uint32_t phase_b_raw)
{
    if ((calibration == 0) || (calibration->target_sample_count == 0U))
    {
        return false;
    }

    if (calibration->discard_remaining > 0U)
    {
        --calibration->discard_remaining;
        return false;
    }

    if (calibration->sample_count >= calibration->target_sample_count)
    {
        return true;
    }

    calibration->phase_a_sum += phase_a_raw;
    calibration->phase_b_sum += phase_b_raw;
    if (phase_a_raw < calibration->phase_a_min)
    {
        calibration->phase_a_min = phase_a_raw;
    }
    if (phase_a_raw > calibration->phase_a_max)
    {
        calibration->phase_a_max = phase_a_raw;
    }
    if (phase_b_raw < calibration->phase_b_min)
    {
        calibration->phase_b_min = phase_b_raw;
    }
    if (phase_b_raw > calibration->phase_b_max)
    {
        calibration->phase_b_max = phase_b_raw;
    }

    ++calibration->sample_count;
    return calibration->sample_count >= calibration->target_sample_count;
}

bool CurrentSenseCalibration_GetOffsets(const CurrentSenseCalibration *calibration,
                                        float adc_full_scale_count,
                                        float rail_margin_count,
                                        uint32_t maximum_span_count,
                                        CurrentSenseOffsets *offsets)
{
    float phase_a_average;
    float phase_b_average;

    if ((calibration == 0) || (offsets == 0) ||
        !isfinite(adc_full_scale_count) || !isfinite(rail_margin_count) ||
        (adc_full_scale_count <= 0.0f) || (rail_margin_count < 0.0f) ||
        (calibration->target_sample_count == 0U) ||
        (calibration->sample_count != calibration->target_sample_count) ||
        ((calibration->phase_a_max - calibration->phase_a_min) > maximum_span_count) ||
        ((calibration->phase_b_max - calibration->phase_b_min) > maximum_span_count))
    {
        return false;
    }

    phase_a_average = (float)calibration->phase_a_sum /
                      (float)calibration->target_sample_count;
    phase_b_average = (float)calibration->phase_b_sum /
                      (float)calibration->target_sample_count;
    if ((phase_a_average < rail_margin_count) ||
        (phase_b_average < rail_margin_count) ||
        (phase_a_average > (adc_full_scale_count - rail_margin_count)) ||
        (phase_b_average > (adc_full_scale_count - rail_margin_count)))
    {
        return false;
    }

    offsets->phase_a_count = phase_a_average;
    offsets->phase_b_count = phase_b_average;
    return true;
}

static bool CurrentSense_IsConfigValid(const CurrentSenseConfig *config,
                                       const CurrentSenseOffsets *offsets)
{
    if ((config == 0) || (offsets == 0))
    {
        return false;
    }

    return isfinite(config->adc_reference_v) &&
           isfinite(config->adc_full_scale_count) &&
           isfinite(config->shunt_resistance_ohm) &&
           isfinite(config->amplifier_gain_v_per_v) &&
           isfinite(config->phase_a_polarity) &&
           isfinite(config->phase_b_polarity) &&
           isfinite(offsets->phase_a_count) &&
           isfinite(offsets->phase_b_count) &&
           (config->adc_reference_v > 0.0f) &&
           (config->adc_full_scale_count > 0.0f) &&
           (config->shunt_resistance_ohm > 0.0f) &&
           (config->amplifier_gain_v_per_v > 0.0f) &&
           ((config->phase_a_polarity == 1.0f) || (config->phase_a_polarity == -1.0f)) &&
           ((config->phase_b_polarity == 1.0f) || (config->phase_b_polarity == -1.0f));
}

bool CurrentSense_Convert(const CurrentSenseConfig *config,
                          const CurrentSenseOffsets *offsets,
                          uint32_t phase_a_raw,
                          uint32_t phase_b_raw,
                          CurrentPhaseCurrents *current)
{
    float ampere_per_count;

    if ((current == 0) || !CurrentSense_IsConfigValid(config, offsets))
    {
        return false;
    }

    if (((float)phase_a_raw > config->adc_full_scale_count) ||
        ((float)phase_b_raw > config->adc_full_scale_count))
    {
        return false;
    }

    ampere_per_count = config->adc_reference_v /
                       (config->adc_full_scale_count *
                        config->amplifier_gain_v_per_v *
                        config->shunt_resistance_ohm);

    current->ia_a = config->phase_a_polarity *
                    ((float)phase_a_raw - offsets->phase_a_count) * ampere_per_count;
    current->ib_a = config->phase_b_polarity *
                    ((float)phase_b_raw - offsets->phase_b_count) * ampere_per_count;
    current->ic_a = -(current->ia_a + current->ib_a);

    return isfinite(current->ia_a) && isfinite(current->ib_a) && isfinite(current->ic_a);
}

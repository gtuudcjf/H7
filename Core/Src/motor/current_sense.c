/**
 * @file current_sense.c
 * @brief 两路低侧电流采样的物理量换算与第三相重构。
 */
#include "current_sense.h"

#include <math.h>

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

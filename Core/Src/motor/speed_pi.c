/**
 * @file speed_pi.c
 * @brief 带安培限幅和回算式抗积分饱和的机械速度PI。
 */
#include "speed_pi.h"

#include <math.h>

static float SpeedPi_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static bool SpeedPi_ConfigIsValid(const SpeedPiConfig *config)
{
    return (config != 0) &&
           isfinite(config->kp_a_per_rpm) &&
           isfinite(config->ki_a_per_rpm_s) &&
           isfinite(config->kaw_per_s) &&
           isfinite(config->output_limit_a) &&
           (config->kp_a_per_rpm >= 0.0f) &&
           (config->ki_a_per_rpm_s >= 0.0f) &&
           (config->kaw_per_s >= 0.0f) &&
           (config->output_limit_a > 0.0f);
}

bool SpeedPi_Init(SpeedPiController *controller,
                  const SpeedPiConfig *config)
{
    if ((controller == 0) || !SpeedPi_ConfigIsValid(config))
    {
        return false;
    }

    controller->config = *config;
    controller->integrator_a = 0.0f;
    return true;
}

bool SpeedPi_Step(SpeedPiController *controller,
                  float reference_rpm,
                  float feedback_rpm,
                  float sample_period_s,
                  SpeedPiResult *result)
{
    SpeedPiResult next;
    float correction_a_per_s;
    float next_integrator_a;

    if ((controller == 0) || (result == 0) ||
        !SpeedPi_ConfigIsValid(&controller->config) ||
        !isfinite(controller->integrator_a) ||
        !isfinite(reference_rpm) || !isfinite(feedback_rpm) ||
        !isfinite(sample_period_s) || (sample_period_s <= 0.0f))
    {
        return false;
    }

    next.error_rpm = reference_rpm - feedback_rpm;
    next.proportional_a =
        controller->config.kp_a_per_rpm * next.error_rpm;
    next.unsaturated_a = next.proportional_a + controller->integrator_a;
    next.iq_command_a = SpeedPi_Clamp(
        next.unsaturated_a, controller->config.output_limit_a);
    next.saturated = next.iq_command_a != next.unsaturated_a;

    correction_a_per_s =
        (controller->config.ki_a_per_rpm_s * next.error_rpm) +
        (controller->config.kaw_per_s *
         (next.iq_command_a - next.unsaturated_a));
    next_integrator_a = SpeedPi_Clamp(
        controller->integrator_a + (correction_a_per_s * sample_period_s),
        controller->config.output_limit_a);

    if (!isfinite(next.error_rpm) || !isfinite(next.proportional_a) ||
        !isfinite(next.unsaturated_a) || !isfinite(next.iq_command_a) ||
        !isfinite(next_integrator_a))
    {
        return false;
    }

    controller->integrator_a = next_integrator_a;
    next.integrator_a = next_integrator_a;
    *result = next;
    return true;
}

bool SpeedPi_PreloadOutput(SpeedPiController *controller,
                           float reference_rpm,
                           float feedback_rpm,
                           float requested_iq_a)
{
    float error_rpm;
    float next_integrator_a;

    if ((controller == 0) ||
        !SpeedPi_ConfigIsValid(&controller->config) ||
        !isfinite(reference_rpm) || !isfinite(feedback_rpm) ||
        !isfinite(requested_iq_a))
    {
        return false;
    }

    error_rpm = reference_rpm - feedback_rpm;
    next_integrator_a = requested_iq_a -
                        (controller->config.kp_a_per_rpm * error_rpm);
    if (!isfinite(next_integrator_a))
    {
        return false;
    }

    controller->integrator_a = SpeedPi_Clamp(
        next_integrator_a, controller->config.output_limit_a);
    return true;
}

bool SpeedPi_SetGains(SpeedPiController *controller,
                      float kp_a_per_rpm,
                      float ki_a_per_rpm_s,
                      float kaw_per_s)
{
    SpeedPiConfig candidate;

    if (controller == 0)
    {
        return false;
    }

    candidate = controller->config;
    candidate.kp_a_per_rpm = kp_a_per_rpm;
    candidate.ki_a_per_rpm_s = ki_a_per_rpm_s;
    candidate.kaw_per_s = kaw_per_s;
    if (!SpeedPi_ConfigIsValid(&candidate))
    {
        return false;
    }

    controller->config = candidate;
    return true;
}

bool SpeedPi_SetOutputLimit(SpeedPiController *controller,
                            float output_limit_a)
{
    SpeedPiConfig candidate;

    if (controller == 0)
    {
        return false;
    }

    candidate = controller->config;
    candidate.output_limit_a = output_limit_a;
    if (!SpeedPi_ConfigIsValid(&candidate))
    {
        return false;
    }

    controller->config = candidate;
    controller->integrator_a = SpeedPi_Clamp(
        controller->integrator_a, output_limit_a);
    return true;
}

void SpeedPi_Reset(SpeedPiController *controller)
{
    if (controller != 0)
    {
        controller->integrator_a = 0.0f;
    }
}

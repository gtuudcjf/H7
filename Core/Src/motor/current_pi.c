/**
 * @file current_pi.c
 * @brief 使用伏特/安培物理量的保守电流 PI 实现。
 */
#include "current_pi.h"

#include <math.h>

static float CurrentPi_Clamp(float value, float limit)
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

static bool CurrentPi_IsConfigValid(const CurrentPiConfig *config)
{
    if (config == 0)
    {
        return false;
    }

    return isfinite(config->kp_v_per_a) &&
           isfinite(config->ki_v_per_a_s) &&
           isfinite(config->kaw_per_s) &&
           isfinite(config->output_limit_v) &&
           (config->kp_v_per_a >= 0.0f) &&
           (config->ki_v_per_a_s >= 0.0f) &&
           (config->kaw_per_s >= 0.0f) &&
           (config->output_limit_v > 0.0f);
}

bool CurrentPi_Init(CurrentPiController *controller, const CurrentPiConfig *config)
{
    if ((controller == 0) || !CurrentPi_IsConfigValid(config))
    {
        return false;
    }

    controller->config = *config;
    CurrentPi_Reset(controller);
    return true;
}

bool CurrentPi_SetGains(CurrentPiController *controller,
                        float kp_v_per_a,
                        float ki_v_per_a_s,
                        float kaw_per_s)
{
    CurrentPiConfig config;

    if (controller == 0)
    {
        return false;
    }

    config = controller->config;
    config.kp_v_per_a = kp_v_per_a;
    config.ki_v_per_a_s = ki_v_per_a_s;
    config.kaw_per_s = kaw_per_s;
    if (!CurrentPi_IsConfigValid(&config))
    {
        return false;
    }

    controller->config = config;
    return true;
}

bool CurrentPi_SetOutputLimit(CurrentPiController *controller, float output_limit_v)
{
    CurrentPiConfig config;

    if (controller == 0)
    {
        return false;
    }

    config = controller->config;
    config.output_limit_v = output_limit_v;
    if (!CurrentPi_IsConfigValid(&config))
    {
        return false;
    }

    controller->config = config;
    controller->integrator_d_v = CurrentPi_Clamp(controller->integrator_d_v, output_limit_v);
    controller->integrator_q_v = CurrentPi_Clamp(controller->integrator_q_v, output_limit_v);
    return true;
}

void CurrentPi_Reset(CurrentPiController *controller)
{
    if (controller != 0)
    {
        controller->integrator_d_v = 0.0f;
        controller->integrator_q_v = 0.0f;
    }
}

bool CurrentPi_PreloadOutput(CurrentPiController *controller,
                             const FocDq *reference_a,
                             const FocDq *feedback_a,
                             const FocDq *requested_voltage_v)
{
    float error_d;
    float error_q;

    if ((controller == 0) || (reference_a == 0) || (feedback_a == 0) ||
        (requested_voltage_v == 0) || !CurrentPi_IsConfigValid(&controller->config) ||
        !isfinite(reference_a->d) || !isfinite(reference_a->q) ||
        !isfinite(feedback_a->d) || !isfinite(feedback_a->q) ||
        !isfinite(requested_voltage_v->d) || !isfinite(requested_voltage_v->q))
    {
        return false;
    }

    error_d = reference_a->d - feedback_a->d;
    error_q = reference_a->q - feedback_a->q;
    controller->integrator_d_v = CurrentPi_Clamp(
        requested_voltage_v->d - (controller->config.kp_v_per_a * error_d),
        controller->config.output_limit_v);
    controller->integrator_q_v = CurrentPi_Clamp(
        requested_voltage_v->q - (controller->config.kp_v_per_a * error_q),
        controller->config.output_limit_v);
    return true;
}

bool CurrentPi_StepDq(CurrentPiController *controller,
                      const FocDq *reference_a,
                      const FocDq *feedback_a,
                      float sample_period_s,
                      CurrentPiResult *result)
{
    float magnitude;
    float scale = 1.0f;
    float correction_d;
    float correction_q;
    const float epsilon = 1.0e-12f;

    if ((controller == 0) || (reference_a == 0) || (feedback_a == 0) || (result == 0) ||
        !CurrentPi_IsConfigValid(&controller->config) ||
        !isfinite(sample_period_s) || (sample_period_s <= 0.0f) ||
        !isfinite(reference_a->d) || !isfinite(reference_a->q) ||
        !isfinite(feedback_a->d) || !isfinite(feedback_a->q) ||
        !isfinite(controller->integrator_d_v) || !isfinite(controller->integrator_q_v))
    {
        return false;
    }

    result->error_a.d = reference_a->d - feedback_a->d;
    result->error_a.q = reference_a->q - feedback_a->q;
    result->proportional_v.d = controller->config.kp_v_per_a * result->error_a.d;
    result->proportional_v.q = controller->config.kp_v_per_a * result->error_a.q;
    result->unsaturated_v.d = result->proportional_v.d + controller->integrator_d_v;
    result->unsaturated_v.q = result->proportional_v.q + controller->integrator_q_v;

    magnitude = sqrtf((result->unsaturated_v.d * result->unsaturated_v.d) +
                      (result->unsaturated_v.q * result->unsaturated_v.q));
    result->saturated = magnitude > controller->config.output_limit_v;
    if (result->saturated && (magnitude > epsilon))
    {
        scale = controller->config.output_limit_v / magnitude;
    }

    result->voltage_v.d = result->unsaturated_v.d * scale;
    result->voltage_v.q = result->unsaturated_v.q * scale;

    /*
     * 回算项把联合矢量限幅后的差值反馈到两个积分器。额外对积分器按
     * 电压上限裁剪，用于防止配置错误或极长故障瞬态造成数值失控。
     */
    correction_d = controller->config.ki_v_per_a_s * result->error_a.d +
                   controller->config.kaw_per_s *
                       (result->voltage_v.d - result->unsaturated_v.d);
    correction_q = controller->config.ki_v_per_a_s * result->error_a.q +
                   controller->config.kaw_per_s *
                       (result->voltage_v.q - result->unsaturated_v.q);
    controller->integrator_d_v = CurrentPi_Clamp(
        controller->integrator_d_v + (correction_d * sample_period_s),
        controller->config.output_limit_v);
    controller->integrator_q_v = CurrentPi_Clamp(
        controller->integrator_q_v + (correction_q * sample_period_s),
        controller->config.output_limit_v);

    result->integrator_v.d = controller->integrator_d_v;
    result->integrator_v.q = controller->integrator_q_v;

    return isfinite(result->voltage_v.d) && isfinite(result->voltage_v.q) &&
           isfinite(controller->integrator_d_v) && isfinite(controller->integrator_q_v);
}

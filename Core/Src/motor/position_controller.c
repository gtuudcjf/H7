/**
 * @file position_controller.c
 * @brief 无硬件依赖的单圈最短路径位置控制。
 */
#include "position_controller.h"

#include <math.h>

static bool PositionController_ConfigIsValid(
    const PositionControllerConfig *config)
{
    return (config != 0) &&
           isfinite(config->kp_rpm_per_deg) &&
           isfinite(config->max_speed_rpm) &&
           isfinite(config->tolerance_deg) &&
           (config->kp_rpm_per_deg > 0.0f) &&
           (config->max_speed_rpm > 0.0f) &&
           (config->tolerance_deg >= 0.0f) &&
           (config->tolerance_deg < 180.0f);
}

bool PositionController_Init(PositionController *controller,
                             const PositionControllerConfig *config)
{
    if ((controller == 0) || !PositionController_ConfigIsValid(config))
    {
        return false;
    }

    controller->config = *config;
    controller->startup_target_deg = 0.0f;
    controller->startup_target_pending = false;
    return true;
}

bool PositionController_SetStartupTarget(PositionController *controller,
                                         float target_deg)
{
    if ((controller == 0) ||
        !PositionController_ConfigIsValid(&controller->config) ||
        !isfinite(target_deg) || (target_deg < 0.0f) ||
        (target_deg >= 360.0f))
    {
        return false;
    }

    controller->startup_target_deg = target_deg;
    controller->startup_target_pending = true;
    return true;
}

bool PositionController_CaptureStartupTarget(PositionController *controller,
                                             float feedback_deg,
                                             float *target_deg)
{
    if ((controller == 0) || (target_deg == 0) ||
        !PositionController_ConfigIsValid(&controller->config) ||
        !isfinite(feedback_deg) || (feedback_deg < 0.0f) ||
        (feedback_deg >= 360.0f))
    {
        return false;
    }

    *target_deg = controller->startup_target_pending ?
        controller->startup_target_deg : feedback_deg;
    controller->startup_target_pending = false;
    return true;
}

bool PositionController_Step(const PositionController *controller,
                             float target_deg,
                             float actual_angle_pu,
                             PositionControllerResult *result)
{
    PositionControllerResult next;
    float requested_speed_rpm;

    if ((controller == 0) || (result == 0) ||
        !PositionController_ConfigIsValid(&controller->config) ||
        !isfinite(target_deg) || (target_deg < 0.0f) ||
        (target_deg >= 360.0f) ||
        !isfinite(actual_angle_pu) || (actual_angle_pu < 0.0f) ||
        (actual_angle_pu >= 1.0f))
    {
        return false;
    }

    next.error_deg = target_deg - actual_angle_pu * 360.0f;
    if (next.error_deg <= -180.0f)
    {
        next.error_deg += 360.0f;
    }
    else if (next.error_deg > 180.0f)
    {
        next.error_deg -= 360.0f;
    }

    next.within_tolerance =
        fabsf(next.error_deg) <= controller->config.tolerance_deg;
    requested_speed_rpm = next.within_tolerance ?
        0.0f : controller->config.kp_rpm_per_deg * next.error_deg;
    if (!isfinite(requested_speed_rpm))
    {
        return false;
    }
    if (requested_speed_rpm > controller->config.max_speed_rpm)
    {
        requested_speed_rpm = controller->config.max_speed_rpm;
    }
    else if (requested_speed_rpm < -controller->config.max_speed_rpm)
    {
        requested_speed_rpm = -controller->config.max_speed_rpm;
    }
    next.speed_target_rpm = requested_speed_rpm;
    *result = next;
    return true;
}

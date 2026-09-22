/**
 * @file speed_pi.h
 * @brief 以机械转速误差生成q轴电流命令的有界PI控制器。
 */
#ifndef SPEED_PI_H
#define SPEED_PI_H

#include <stdbool.h>

typedef struct
{
    float kp_a_per_rpm;
    float ki_a_per_rpm_s;
    float kaw_per_s;
    float output_limit_a;
} SpeedPiConfig;

typedef struct
{
    SpeedPiConfig config;
    float integrator_a;
} SpeedPiController;

typedef struct
{
    float error_rpm;
    float proportional_a;
    float integrator_a;
    float unsaturated_a;
    float iq_command_a;
    bool saturated;
} SpeedPiResult;

bool SpeedPi_Init(SpeedPiController *controller,
                  const SpeedPiConfig *config);
bool SpeedPi_Step(SpeedPiController *controller,
                  float reference_rpm,
                  float feedback_rpm,
                  float sample_period_s,
                  SpeedPiResult *result);
bool SpeedPi_PreloadOutput(SpeedPiController *controller,
                           float reference_rpm,
                           float feedback_rpm,
                           float requested_iq_a);
bool SpeedPi_SetGains(SpeedPiController *controller,
                      float kp_a_per_rpm,
                      float ki_a_per_rpm_s,
                      float kaw_per_s);
bool SpeedPi_SetOutputLimit(SpeedPiController *controller,
                            float output_limit_a);
void SpeedPi_Reset(SpeedPiController *controller);

#endif /* SPEED_PI_H */

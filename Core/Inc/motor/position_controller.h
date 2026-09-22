/**
 * @file position_controller.h
 * @brief 单圈机械位置 P 环：输出供现有速度 PI 使用的速度目标。
 */
#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <stdbool.h>

typedef struct
{
    float kp_rpm_per_deg;
    float max_speed_rpm;
    float tolerance_deg;
} PositionControllerConfig;

typedef struct
{
    PositionControllerConfig config;
} PositionController;

typedef struct
{
    float error_deg;
    float speed_target_rpm;
    bool within_tolerance;
} PositionControllerResult;

bool PositionController_Init(PositionController *controller,
                             const PositionControllerConfig *config);

/** 目标为[0,360)度，反馈为[0,1)圈；误差在(-180,180]度。 */
bool PositionController_Step(const PositionController *controller,
                             float target_deg,
                             float actual_angle_pu,
                             PositionControllerResult *result);

#endif /* POSITION_CONTROLLER_H */

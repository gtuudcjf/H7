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
    float startup_target_deg;
    bool startup_target_pending;
} PositionController;

typedef struct
{
    float error_deg;
    float speed_target_rpm;
    bool within_tolerance;
} PositionControllerResult;

bool PositionController_Init(PositionController *controller,
                             const PositionControllerConfig *config);

/** 预设下一次进入位置模式时的单圈目标角；只接受 [0, 360) 度。 */
bool PositionController_SetStartupTarget(PositionController *controller,
                                         float target_deg);

/** 捕获启动目标：有预设时使用预设，否则保持当前角度；预设仅消费一次。 */
bool PositionController_CaptureStartupTarget(PositionController *controller,
                                             float feedback_deg,
                                             float *target_deg);

/** 目标为[0,360)度，反馈为[0,1)圈；误差在(-180,180]度。 */
bool PositionController_Step(const PositionController *controller,
                             float target_deg,
                             float actual_angle_pu,
                             PositionControllerResult *result);

#endif /* POSITION_CONTROLLER_H */

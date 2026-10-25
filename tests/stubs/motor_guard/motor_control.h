#include <stdint.h>
#include "main.h"
typedef struct { float speed_rpm,iq_a,active_target_rpm; uint16_t fault; uint8_t calibrated,running,valid; } MotorControlFeedback;
void MotorControl_GetFeedback(MotorControlFeedback *f);
HAL_StatusTypeDef MotorControl_Start(void);
HAL_StatusTypeDef MotorControl_Stop(void);
HAL_StatusTypeDef MotorControl_ClearFault(void);
void MotorControl_SetSpeedCommand(float rpm);
typedef struct {float iq_a,active_frequency_hz;uint16_t motor_fault,bridge_fault;uint8_t start_permitted,running,control_valid,output_off,stop_requested;} MotorOpenAngleFeedback;
void MotorControl_GetOpenAngleFeedback(MotorOpenAngleFeedback *f);
void MotorControl_SetOpenAngleSpeedCommand(float rpm);

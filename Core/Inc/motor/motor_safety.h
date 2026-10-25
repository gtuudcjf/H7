#ifndef MOTOR_SAFETY_H
#define MOTOR_SAFETY_H
#include <stdint.h>
void MotorSafety_Init(void);
void MotorSafety_Feed(void);
uint8_t MotorSafety_Arm(void);
void MotorSafety_Disarm(void);
void MotorSafety_Revoke(void);
void MotorSafety_Reset(void);
void MotorSafety_Tick(void);
uint8_t MotorSafety_IsTripped(void);
uint8_t MotorSafety_TripReason(void);
void MotorSafety_MarkMotion(void);
uint8_t MotorSafety_OutputPermitted(void);
/* Physical high transition is serialized against the timer's revoke. */
uint8_t MotorSafety_TryEnableDriver(void);
/* TIM8 CH4 also sets MOE in HAL; protect its start just like phase PWM. */
uint8_t MotorSafety_TryStartSamplingPwm(void);
#endif

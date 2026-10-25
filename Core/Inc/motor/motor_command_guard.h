#ifndef MOTOR_COMMAND_GUARD_H
#define MOTOR_COMMAND_GUARD_H
#include <stdint.h>
/* Foreground callers serialize compound operations against the timer IRQ. */
typedef struct {
    volatile uint32_t last_ms;
    volatile uint8_t fed, permitted, latched;
    volatile uint32_t session_ms,motion_ms;
    volatile uint8_t session_active,motion_active,trip_reason;
} MotorCommandGuard;
void MCG_Init(MotorCommandGuard *g);
void MCG_Reset(MotorCommandGuard *g);
void MCG_Feed(MotorCommandGuard *g, uint32_t now_ms);
uint8_t MCG_Permit(MotorCommandGuard *g, uint32_t now_ms);
void MCG_Disarm(MotorCommandGuard *g);
void MCG_Revoke(MotorCommandGuard *g);
uint8_t MCG_Tick(MotorCommandGuard *g, uint32_t now_ms);
void MCG_StartSession(MotorCommandGuard *g,uint32_t now_ms);
void MCG_MarkMotion(MotorCommandGuard *g,uint32_t now_ms);
uint8_t MCG_TripReason(const MotorCommandGuard *g);
#endif

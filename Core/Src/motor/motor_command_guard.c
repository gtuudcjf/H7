#include "motor_command_guard.h"
#include "../ethercat/ethercat_build.h"
#if ETHERCAT_MOTOR_OPEN_ANGLE
#include "open_angle_experiment.h"
/* Keep both shutdown paths on the same profile-specific time limits. */
#define MCG_SESSION_LIMIT_MS OAE_SESSION_MS
#define MCG_MOTION_LIMIT_MS OAE_MOTION_MS
#else
#define MCG_SESSION_LIMIT_MS 5000U
#define MCG_MOTION_LIMIT_MS 3000U
#endif
void MCG_Init(MotorCommandGuard *g)
{ g->last_ms=0; g->fed=0; g->permitted=0; g->latched=0;
  g->session_ms=0;g->motion_ms=0;g->session_active=0;g->motion_active=0;g->trip_reason=0; }
void MCG_Reset(MotorCommandGuard *g) { MCG_Init(g); }
void MCG_Feed(MotorCommandGuard *g, uint32_t now_ms)
{ g->last_ms=now_ms; g->fed=1; }
uint8_t MCG_Permit(MotorCommandGuard *g, uint32_t now_ms)
{
    if(g->latched || !g->fed || (uint32_t)(now_ms-g->last_ms)>=100U) return 0;
    g->permitted=1; return 1;
}
void MCG_Disarm(MotorCommandGuard *g) { g->permitted=0;g->session_active=0;g->motion_active=0; }
void MCG_Revoke(MotorCommandGuard *g) { g->permitted=0; g->latched=1; }
uint8_t MCG_Tick(MotorCommandGuard *g, uint32_t now_ms)
{
    if(g->permitted && g->session_active &&
       ((uint32_t)(now_ms-g->session_ms)>=MCG_SESSION_LIMIT_MS ||
        (g->motion_active && (uint32_t)(now_ms-g->motion_ms)>=MCG_MOTION_LIMIT_MS))) {
        g->trip_reason=2; MCG_Revoke(g);return 1;
    }
    if(g->permitted && (uint32_t)(now_ms-g->last_ms)>=100U) {
        g->trip_reason=1;MCG_Revoke(g); return 1;
    }
    return 0;
}
void MCG_StartSession(MotorCommandGuard *g,uint32_t now_ms)
{if(g->permitted && !g->latched && !g->session_active){g->session_ms=now_ms;g->session_active=1;}}
void MCG_MarkMotion(MotorCommandGuard *g,uint32_t now_ms)
{if(g->permitted && g->session_active && !g->motion_active){g->motion_ms=now_ms;g->motion_active=1;}}
uint8_t MCG_TripReason(const MotorCommandGuard *g){return g->trip_reason;}

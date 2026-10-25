#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#undef assert
#define assert(x) do{if(!(x)){fprintf(stderr,"guard line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#include "motor_command_guard.h"
static MotorCommandGuard g;
static void feed(uint32_t start,uint32_t elapsed){MCG_Feed(&g,start+elapsed);}
#if ETHERCAT_MOTOR_OPEN_ANGLE
enum { session_limit_ms=10000U,motion_limit_ms=6000U };
#else
enum { session_limit_ms=5000U,motion_limit_ms=3000U };
#endif
int main(void)
{
    MCG_Init(&g);MCG_Feed(&g,0);assert(MCG_Permit(&g,0));MCG_StartSession(&g,0);
    feed(0,session_limit_ms-1);assert(!MCG_Tick(&g,session_limit_ms-1));MCG_StartSession(&g,session_limit_ms-1);
    feed(0,session_limit_ms);assert(MCG_Tick(&g,session_limit_ms));assert(MCG_TripReason(&g)==2 && !g.permitted);
    MCG_Reset(&g);MCG_Feed(&g,0);assert(MCG_Permit(&g,0));MCG_StartSession(&g,0);
    MCG_MarkMotion(&g,10);MCG_MarkMotion(&g,2000);feed(0,motion_limit_ms+9);assert(!MCG_Tick(&g,motion_limit_ms+9));
    feed(0,motion_limit_ms+10);assert(MCG_Tick(&g,motion_limit_ms+10));assert(MCG_TripReason(&g)==2);
    MCG_Reset(&g);feed(0,0);assert(MCG_Permit(&g,0));MCG_StartSession(&g,0);
    assert(MCG_Tick(&g,100));assert(MCG_TripReason(&g)==1);
    MCG_Reset(&g);feed(0,200);assert(MCG_Permit(&g,200));MCG_StartSession(&g,200);
    MCG_Revoke(&g);feed(0,201);assert(!MCG_Permit(&g,201));
    MCG_Reset(&g);feed(UINT32_MAX-100,0);assert(MCG_Permit(&g,UINT32_MAX-100));
    MCG_StartSession(&g,UINT32_MAX-100);MCG_MarkMotion(&g,UINT32_MAX-90);
    feed(0,motion_limit_ms-92);assert(!MCG_Tick(&g,motion_limit_ms-92));
    feed(0,motion_limit_ms-91);assert(MCG_Tick(&g,motion_limit_ms-91));
    MCG_Reset(&g);feed(0,0);assert(MCG_Permit(&g,0));feed(0,6000);
    assert(!MCG_Tick(&g,6000)); /* Legacy profile without a session remains unchanged. */
    printf("independent session guard: 7 scenarios passed (session=%u ms, motion=%u ms)\n",
           (unsigned)session_limit_ms,(unsigned)motion_limit_ms);return 0;
}

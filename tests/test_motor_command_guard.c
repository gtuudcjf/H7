#include <stdio.h>
#include <stdlib.h>
#include "motor_command_guard.h"
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"guard %d: %s\n",__LINE__,#x);exit(1);}}while(0)
int main(void)
{
    MotorCommandGuard g;
    MCG_Init(&g); CHECK(!g.permitted && !g.latched);
    CHECK(!MCG_Permit(&g,0));
    MCG_Feed(&g,10); CHECK(MCG_Permit(&g,10));
    CHECK(!MCG_Tick(&g,109)); CHECK(g.permitted);
    CHECK(MCG_Tick(&g,110)); CHECK(!g.permitted && g.latched);
    MCG_Feed(&g,111); CHECK(!MCG_Permit(&g,111)); /* lease cannot undo latch */
    CHECK(!MCG_Tick(&g,112));
    MCG_Reset(&g); CHECK(!g.permitted && !g.fed && !g.latched);
    CHECK(!MCG_Permit(&g,112));
    MCG_Feed(&g,UINT32_MAX-50); CHECK(MCG_Permit(&g,UINT32_MAX-50));
    CHECK(!MCG_Tick(&g,48)); CHECK(MCG_Tick(&g,49));
    MCG_Reset(&g); MCG_Feed(&g,0); CHECK(!MCG_Permit(&g,100));
    MCG_Reset(&g); MCG_Feed(&g,200); CHECK(MCG_Permit(&g,200));
    MCG_Disarm(&g); CHECK(!g.permitted && !g.latched);
    MCG_Revoke(&g); CHECK(g.latched && !g.permitted);
    MCG_Feed(&g,201); CHECK(!MCG_Permit(&g,201));
    printf("motor independent guard: %u checks passed\n",checks);return 0;
}

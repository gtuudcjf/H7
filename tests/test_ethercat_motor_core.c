#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ethercat_motor_core.h"

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"motor core %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static EMC_Core c;
static EMC_Feedback fb;
static void Command(uint16_t control, int16_t speed, uint32_t seq, uint32_t now)
{
    uint8_t b[8];
    b[0]=(uint8_t)control; b[1]=(uint8_t)(control>>8);
    b[2]=(uint8_t)speed; b[3]=(uint8_t)((uint16_t)speed>>8);
    b[4]=(uint8_t)seq; b[5]=(uint8_t)(seq>>8);
    b[6]=(uint8_t)(seq>>16); b[7]=(uint8_t)(seq>>24);
    EMC_Receive(&c,b,8,now);
}
static uint16_t Status(void)
{
    uint8_t b[20]; EMC_Pack(&c,b); return (uint16_t)(b[0]|((uint16_t)b[1]<<8));
}
static void Init(void)
{
    memset(&fb,0,sizeof(fb)); fb.calibrated=1;
    EMC_Init(&c); EMC_SetFeedback(&c,&fb); EMC_SetLink(&c,1,1);
    CHECK((Status() & EMC_STATUS_RUNNING)==0);
}
static void Arm(uint32_t seq, uint32_t now)
{
    Command(0,0,seq,now); Command(0,0,seq+1,now+10); Command(0,0,seq+2,now+20);
    CHECK((Status() & EMC_STATUS_READY)!=0);
    CHECK((EMC_TakeActions(&c) & EMC_ACTION_START)==0);
}
static void Start(uint32_t seq, uint32_t now)
{
    Command(1,0,seq,now);
    CHECK((EMC_TakeActions(&c)&EMC_ACTION_START)!=0);
    EMC_ActionResult(&c,EMC_ACTION_START,1,now);
    CHECK((Status()&EMC_STATUS_STARTING)!=0);
    fb.running=1; fb.valid=1; EMC_SetFeedback(&c,&fb); EMC_Poll(&c,now+1);
    CHECK((Status()&EMC_STATUS_RUNNING)!=0);
}
static void StartupAndWire(void)
{
    uint8_t b[22]; const uint8_t expected[20]={0x9a,0,1,0,0,0,0xce,0xff,0xd4,0xfe,0,0,0,0,0x64,0,4,0,0,0};
    Init(); Command(1,0,1,0); CHECK((EMC_TakeActions(&c)&EMC_ACTION_START)==0);
    Init(); Arm(1,0); Start(4,30);
    Command(1,-50,5,40); CHECK(c.target_speed_d10==-50);
    fb.speed_d10=-50; fb.iq_ma=-300; fb.active_target_d10=100; EMC_SetFeedback(&c,&fb);
    /* Hand-derived wire fixture: status OP/fresh/running/valid, counter 1. */
    c.accepted_sequence=4; c.cycle_counter=1;
    memset(b,0xa5,sizeof(b)); EMC_Pack(&c,b+1);
    CHECK(memcmp(b+1,expected,20)==0); CHECK(b[0]==0xa5 && b[21]==0xa5);
    Command(0,0,6,50); CHECK((EMC_TakeActions(&c)&EMC_ACTION_STOP)!=0);
    CHECK((Status()&EMC_STATUS_RUNNING)==0);
}
static void LeaseAndReconnect(void)
{
    Init(); Arm(1,0); Start(4,30); (void)EMC_TakeActions(&c);
    Command(1,0,4,90); CHECK((EMC_TakeActions(&c)&EMC_ACTION_FEED)==0);
    EMC_Poll(&c,129); CHECK(c.fault==EMC_FAULT_NONE);
    EMC_Poll(&c,130); CHECK(c.fault==EMC_FAULT_HEARTBEAT);
    CHECK((EMC_TakeActions(&c)&EMC_ACTION_STOP)!=0);
    EMC_SetLink(&c,0,1); EMC_SetLink(&c,1,1);
    Command(1,0,5,140); CHECK((EMC_TakeActions(&c)&EMC_ACTION_START)==0);
    fb.running=0; fb.valid=0; EMC_SetFeedback(&c,&fb);
    Command(0x80,0,6,150); CHECK((EMC_TakeActions(&c)&EMC_ACTION_CLEAR)!=0);
    EMC_ActionResult(&c,EMC_ACTION_CLEAR,1,150); CHECK(c.fault==EMC_FAULT_NONE);
    Command(1,0,7,160); CHECK((EMC_TakeActions(&c)&EMC_ACTION_START)==0);
    Arm(8,170); Start(11,200);
    EMC_SetLink(&c,0,1); CHECK(c.fault==EMC_FAULT_NOT_OP);
    CHECK((EMC_TakeActions(&c)&EMC_ACTION_STOP)!=0);
}
static void InvalidAndPriority(void)
{
    uint8_t bad[9]={0};
    Init(); Arm(1,0); Start(4,30);
    Command(1,201,5,40); CHECK(c.fault==EMC_FAULT_COMMAND);
    CHECK((EMC_TakeActions(&c)&EMC_ACTION_STOP)!=0);
    Init(); Arm(1,0); Start(4,30);
    Command(2,0,5,40); CHECK(c.fault==EMC_FAULT_COMMAND);
    Init(); EMC_Receive(&c,bad,7,0); CHECK(c.fault==EMC_FAULT_COMMAND);
    Init(); EMC_Receive(&c,bad,9,0); CHECK(c.fault==EMC_FAULT_COMMAND);
    Init(); Arm(1,0); Start(4,30);
    Command(0,0,4,40); CHECK((EMC_TakeActions(&c)&EMC_ACTION_STOP)!=0);
    CHECK((Status()&EMC_STATUS_READY)==0); /* duplicate cannot rearm */
    Init(); Arm(1,0); Start(4,30);
    Command(1,0,3,40); CHECK(c.fault==EMC_FAULT_SEQUENCE);
    Init(); Arm(1,0); Command(1,50,4,30);
    CHECK((EMC_TakeActions(&c)&EMC_ACTION_START)==0); CHECK(c.fault==EMC_FAULT_COMMAND);
}
static void WrapAndStartupFaults(void)
{
    unsigned i;
    Init(); Arm(UINT32_MAX-2,UINT32_MAX-30); Start(0,UINT32_MAX);
    Command(1,0,1,9); EMC_Poll(&c,108); CHECK(c.fault==EMC_FAULT_NONE);
    EMC_Poll(&c,109); CHECK(c.fault==EMC_FAULT_HEARTBEAT);
    Init(); fb.calibrated=0; EMC_SetFeedback(&c,&fb);
    Command(0,0,1,0); Command(0,0,2,10); Command(0,0,3,20);
    CHECK((Status()&EMC_STATUS_READY)==0);
    Init(); Arm(1,0); Command(1,0,4,30); (void)EMC_TakeActions(&c);
    EMC_ActionResult(&c,EMC_ACTION_START,1,30);
    for(i=1;i<=20;++i) { Command(1,0,4+i,30+i*99); EMC_Poll(&c,30+i*99); }
    EMC_Poll(&c,2030); CHECK(c.fault==EMC_FAULT_START_TIMEOUT);
    Init(); Arm(1,0); Command(1,0,4,30); (void)EMC_TakeActions(&c);
    EMC_ActionResult(&c,EMC_ACTION_START,0,30); CHECK(c.fault==EMC_FAULT_START);
    Init(); Arm(1,0); Start(4,30); EMC_SetLink(&c,1,0);
    CHECK(c.fault==EMC_FAULT_SPI); fb.running=0; EMC_SetFeedback(&c,&fb);
    Command(0x80,0,5,40); CHECK((EMC_TakeActions(&c)&EMC_ACTION_CLEAR)==0);
    Init(); Arm(1,0); Start(4,30); fb.motor_fault=7; fb.running=0;
    EMC_SetFeedback(&c,&fb); EMC_Poll(&c,40); CHECK(c.fault==EMC_FAULT_MOTOR);
}
int main(void)
{
    /* Receive-before-Poll must not renew a lease which already expired. */
    Init(); Arm(1,0); Command(1,0,4,150);
    CHECK(c.fault==EMC_FAULT_HEARTBEAT);
    CHECK((EMC_TakeActions(&c)&EMC_ACTION_START)==0);
    /* Enabled frames interrupt the three consecutive disabled-zero frames. */
    Init(); Command(0,0,1,0); Command(1,0,2,10);
    Command(0,0,3,20); Command(0,0,4,30); Command(1,0,5,40);
    CHECK((EMC_TakeActions(&c)&EMC_ACTION_START)==0);
    CHECK(c.state==EMC_DISARMED);
    StartupAndWire(); LeaseAndReconnect(); InvalidAndPriority(); WrapAndStartupFaults();
    printf("EtherCAT motor core: %u checks passed\n",checks); return 0;
}

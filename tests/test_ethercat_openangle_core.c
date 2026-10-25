#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ethercat_motor_core.h"
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"openangle core %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static EMC_Core c;
static EMC_Feedback f;
static void cmd(unsigned cw,int speed,uint32_t seq,uint32_t ms)
{
    uint8_t b[8]={(uint8_t)cw,(uint8_t)(cw>>8),(uint8_t)speed,(uint8_t)((uint16_t)speed>>8),
        (uint8_t)seq,(uint8_t)(seq>>8),(uint8_t)(seq>>16),(uint8_t)(seq>>24)};
    EMC_Receive(&c,b,sizeof(b),ms);
}
static void setup(void)
{
    memset(&f,0,sizeof(f));f.start_permitted=1;f.output_off=1;
    EMC_InitProfile(&c,EMC_PROFILE_OPEN_ANGLE);EMC_SetFeedback(&c,&f);EMC_SetLink(&c,1,1);
}
static void ready(void){setup();cmd(0x100,0,1,0);cmd(0x100,0,2,10);cmd(0x100,0,3,20);CHECK(c.state==EMC_READY);}
int main(void)
{
    uint8_t tx[20];unsigned status;
    setup();cmd(0,0,1,0);cmd(0,0,2,10);cmd(0,0,3,20);
    CHECK(c.state==EMC_DISARMED && !c.sequence_valid);cmd(1,0,4,30);
    CHECK(c.fault==EMC_FAULT_PROFILE);CHECK(!(EMC_TakeActions(&c)&EMC_ACTION_START));
    EMC_Init(&c);EMC_SetLink(&c,1,1);cmd(0x100,0,1,0);CHECK(c.fault==EMC_FAULT_COMMAND);
    ready();CHECK(!c.feedback.calibrated);cmd(0x101,0,4,30);CHECK(c.state==EMC_STARTING);
    CHECK(EMC_TakeActions(&c)&EMC_ACTION_START);
    f.running=1;f.control_valid=1;f.output_off=0;EMC_SetFeedback(&c,&f);EMC_Poll(&c,40);
    CHECK(c.state==EMC_RUNNING);EMC_Pack(&c,tx);status=tx[0]|(tx[1]<<8);
    CHECK((status&0x380)==0x300);CHECK(!(status&0x80));CHECK(status&2);
    cmd(0x101,50,5,50);CHECK(c.target_speed_d10==50);
    cmd(0x100,0,5,60);CHECK(c.state==EMC_DISARMED);CHECK(EMC_TakeActions(&c)&EMC_ACTION_STOP);
    ready();cmd(0x101,1,4,30);CHECK(c.fault==EMC_FAULT_COMMAND);
    ready();cmd(0,0,4,30);CHECK(c.fault==EMC_FAULT_PROFILE);
    ready();cmd(0x102,0,4,30);CHECK(c.fault==EMC_FAULT_COMMAND);
    ready();cmd(0x101,0,4,30);(void)EMC_TakeActions(&c);
    f.running=1;f.control_valid=1;f.output_off=0;EMC_SetFeedback(&c,&f);EMC_Poll(&c,40);
    f.stop_requested=1;EMC_SetFeedback(&c,&f);EMC_Poll(&c,41);
    CHECK(c.fault==0 && c.state==EMC_DISARMED);CHECK(EMC_TakeActions(&c)&EMC_ACTION_STOP);
    cmd(0x101,0,5,50);CHECK(c.state==EMC_DISARMED);CHECK(!(EMC_TakeActions(&c)&EMC_ACTION_START));
    f.running=0;f.control_valid=0;f.output_off=1;EMC_SetFeedback(&c,&f);EMC_Pack(&c,tx);
    CHECK((tx[1]&4)!=0 && !(tx[0]&0x80));
    setup();cmd(0x100,0,UINT32_MAX-1,0);cmd(0x100,0,UINT32_MAX,10);cmd(0x100,0,0,20);
    CHECK(c.state==EMC_READY && c.accepted_sequence==0);cmd(0x100,0,0,100);EMC_Poll(&c,120);
    CHECK(c.fault==EMC_FAULT_HEARTBEAT);
    cmd(0x180,0,1,130);CHECK(EMC_TakeActions(&c)&EMC_ACTION_CLEAR);
    EMC_ActionResult(&c,EMC_ACTION_CLEAR,1,130);CHECK(c.fault==0 && c.state==EMC_DISARMED);
    EMC_Init(&c);EMC_Pack(&c,tx);CHECK(tx[1]==0);
    printf("openangle core: %u checks passed\n",checks);return 0;
}

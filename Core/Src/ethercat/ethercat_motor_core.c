#include "ethercat_motor_core.h"
#include <string.h>

static uint16_t Read16(const uint8_t *b)
{ return (uint16_t)((uint16_t)b[0]|((uint16_t)b[1]<<8)); }
static uint32_t Read32(const uint8_t *b)
{ return (uint32_t)Read16(b)|((uint32_t)Read16(b+2)<<16); }
static void Write16(uint8_t *b, uint16_t v)
{ b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8); }
static void Write32(uint8_t *b, uint32_t v)
{ Write16(b,(uint16_t)v); Write16(b+2,(uint16_t)(v>>16)); }

void EMC_Init(EMC_Core *c)
{ memset(c,0,sizeof(*c)); c->spi_ok=1; }
void EMC_InitProfile(EMC_Core *c, EMC_Profile profile)
{ EMC_Init(c); c->profile=profile; }
static uint8_t StartPermitted(const EMC_Core *c)
{ return c->profile==EMC_PROFILE_OPEN_ANGLE ? c->feedback.start_permitted : c->feedback.calibrated; }
static uint8_t ControlValid(const EMC_Core *c)
{ return c->profile==EMC_PROFILE_OPEN_ANGLE ? c->feedback.control_valid : c->feedback.valid; }

void EMC_Fail(EMC_Core *c, EMC_Fault fault)
{
    /* Keep the first fault; transport loss can never be reset via PDO. */
    if(c->fault==EMC_FAULT_NONE || fault==EMC_FAULT_SPI) c->fault=fault;
    c->state=EMC_DISARMED; c->zeros=0; c->target_speed_d10=0;
    c->cycle_counter=0; c->actions=EMC_ACTION_STOP;
}

void EMC_SetLink(EMC_Core *c, uint8_t active, uint8_t spi_ok)
{
    if(!spi_ok) EMC_Fail(c,EMC_FAULT_SPI);
    else if(c->link && !active) EMC_Fail(c,EMC_FAULT_NOT_OP);
    if(!active || !spi_ok) {
        c->fresh=0; c->sequence_valid=0; c->zeros=0;
        c->previous_control=0; c->cycle_counter=0;
    }
    c->link=active!=0; c->spi_ok=spi_ok!=0;
}

void EMC_SetFeedback(EMC_Core *c, const EMC_Feedback *feedback)
{ c->feedback=*feedback; }

void EMC_Receive(EMC_Core *c, const uint8_t *wire, size_t size, uint32_t now_ms)
{
    uint16_t control;
    int16_t speed;
    uint32_t seq, delta;
    uint8_t previous_enable, reset_edge;
    if(!wire || size!=EMC_RX_BYTES) { EMC_Fail(c,EMC_FAULT_COMMAND); return; }
    if(!c->link || !c->spi_ok) return;
    /* A delayed first receive on foreground resume cannot erase an expired
     * lease before Poll observes it, including while READY but not powered. */
    if(c->sequence_valid && (uint32_t)(now_ms-c->last_command_ms)>=EMC_HEARTBEAT_MS) {
        c->fresh=0; EMC_Fail(c,EMC_FAULT_HEARTBEAT);
    }
    control=Read16(wire); speed=(int16_t)Read16(wire+2); seq=Read32(wire+4);
    /* A disable is effective even on a duplicate sequence. It grants no lease. */
    if(!(control&1U) && (c->state==EMC_STARTING || c->state==EMC_RUNNING)) {
        c->actions|=EMC_ACTION_STOP; c->state=EMC_DISARMED;
        c->zeros=0; c->target_speed_d10=0;
    }
    if(c->profile==EMC_PROFILE_OPEN_ANGLE && !(control&EMC_CONTROL_OPEN_ANGLE_ACK)) {
        if(c->profile_seen || control!=0 || speed!=0) EMC_Fail(c,EMC_FAULT_PROFILE);
        return;
    }
    if((control & (uint16_t)~(c->profile==EMC_PROFILE_OPEN_ANGLE ? UINT16_C(0x0181) : UINT16_C(0x0081))) ||
       speed>EMC_SPEED_LIMIT_D10 || speed < -EMC_SPEED_LIMIT_D10 ||
       ((control&0x80U) && ((control&1U) || speed!=0))) {
        EMC_Fail(c,EMC_FAULT_COMMAND); return;
    }
    delta=seq-c->accepted_sequence;
    if(c->sequence_valid) {
        if(delta==0) return;
        if(delta>=UINT32_C(0x80000000)) { EMC_Fail(c,EMC_FAULT_SEQUENCE); return; }
    }
    previous_enable=(uint8_t)(c->previous_control&1U);
    reset_edge=(uint8_t)((control&0x80U) && !(c->previous_control&0x80U));
    c->accepted_sequence=seq; c->sequence_valid=1; c->fresh=1;
    c->profile_seen=1;
    c->last_command_ms=now_ms; c->previous_control=control;
    c->actions|=EMC_ACTION_FEED;
    if(c->fault!=EMC_FAULT_NONE) {
        if(reset_edge && !c->feedback.running && c->fault!=EMC_FAULT_SPI)
            c->actions|=EMC_ACTION_CLEAR;
        return;
    }
    if(!(control&1U)) {
        c->target_speed_d10=0;
        if(control==(c->profile==EMC_PROFILE_OPEN_ANGLE ? EMC_CONTROL_OPEN_ANGLE_ACK : 0U) && speed==0) {
            if(c->zeros<3) ++c->zeros;
            if(c->zeros==3 && StartPermitted(c) && !c->feedback.motor_fault)
                c->state=EMC_READY;
        } else { c->zeros=0; c->state=EMC_DISARMED; }
        return;
    }
    if(c->state==EMC_READY && !previous_enable && StartPermitted(c) &&
       !c->feedback.motor_fault) {
        if(speed!=0) { EMC_Fail(c,EMC_FAULT_COMMAND); return; }
        c->state=EMC_STARTING; c->start_ms=now_ms; c->zeros=0;
        c->actions|=EMC_ACTION_START;
    } else if(c->state==EMC_RUNNING) c->target_speed_d10=speed;
    else if(c->state==EMC_STARTING && speed!=0) EMC_Fail(c,EMC_FAULT_COMMAND);
    else if(c->state==EMC_DISARMED) c->zeros=0;
}

void EMC_Poll(EMC_Core *c, uint32_t now_ms)
{
    if(c->sequence_valid && (uint32_t)(now_ms-c->last_command_ms)>=EMC_HEARTBEAT_MS) {
        c->fresh=0; EMC_Fail(c,EMC_FAULT_HEARTBEAT);
    }
    if(c->feedback.motor_fault && c->fault==EMC_FAULT_NONE) EMC_Fail(c,EMC_FAULT_MOTOR);
    if(c->fault!=EMC_FAULT_NONE) return;
    if(c->profile==EMC_PROFILE_OPEN_ANGLE &&
       (c->state==EMC_STARTING || c->state==EMC_RUNNING) && c->feedback.stop_requested) {
        c->actions|=EMC_ACTION_STOP; c->state=EMC_DISARMED;
        c->zeros=0; c->target_speed_d10=0; return;
    }
    if(c->state==EMC_STARTING) {
        if((uint32_t)(now_ms-c->start_ms)>=EMC_START_TIMEOUT_MS)
            EMC_Fail(c,EMC_FAULT_START_TIMEOUT);
        else if(c->feedback.running && ControlValid(c)) c->state=EMC_RUNNING;
    } else if(c->state==EMC_RUNNING && (!c->feedback.running || !ControlValid(c)))
        EMC_Fail(c,EMC_FAULT_MOTOR);
    if(c->link && c->fresh && c->state==EMC_RUNNING) ++c->cycle_counter;
}

uint8_t EMC_TakeActions(EMC_Core *c)
{ uint8_t actions=c->actions; c->actions=0; return actions; }

void EMC_ActionResult(EMC_Core *c, uint8_t action, uint8_t success, uint32_t now_ms)
{
    (void)now_ms;
    if(action==EMC_ACTION_START && !success) EMC_Fail(c,EMC_FAULT_START);
    if(action==EMC_ACTION_STOP && !success) EMC_Fail(c,EMC_FAULT_STOP);
    if(action==EMC_ACTION_CLEAR && success && c->link && c->spi_ok &&
       c->fault!=EMC_FAULT_SPI && !c->feedback.running) {
        c->fault=EMC_FAULT_NONE; c->state=EMC_DISARMED;
        c->zeros=0; c->previous_control=0; c->target_speed_d10=0;
        c->actions=0;
    }
}

void EMC_Pack(const EMC_Core *c, uint8_t wire[EMC_TX_BYTES])
{
    uint16_t status=0;
    if(c->state==EMC_READY && c->fault==EMC_FAULT_NONE && StartPermitted(c))
        status|=EMC_STATUS_READY;
    if(c->state==EMC_RUNNING) status|=EMC_STATUS_RUNNING;
    if(c->fault || c->feedback.motor_fault) status|=EMC_STATUS_FAULT;
    if(c->link && c->spi_ok) status|=EMC_STATUS_OP;
    if(c->fresh) status|=EMC_STATUS_FRESH;
    if(c->state==EMC_DISARMED) status|=EMC_STATUS_REARM;
    if(c->state==EMC_STARTING) status|=EMC_STATUS_STARTING;
    if(c->profile==EMC_PROFILE_OPEN_ANGLE) {
        status|=EMC_STATUS_OPEN_ANGLE;
        if(c->feedback.control_valid && c->link && c->spi_ok) status|=EMC_STATUS_CONTROL_VALID;
        if(c->feedback.output_off) status|=EMC_STATUS_OUTPUT_OFF;
    } else if(c->feedback.valid && c->link && c->spi_ok) status|=EMC_STATUS_VALID;
    Write16(wire,status); Write32(wire+2,c->cycle_counter);
    Write16(wire+6,(uint16_t)c->feedback.speed_d10);
    Write16(wire+8,(uint16_t)c->feedback.iq_ma);
    Write16(wire+10,c->feedback.motor_fault); Write16(wire+12,(uint16_t)c->fault);
    Write16(wire+14,(uint16_t)c->feedback.active_target_d10);
    Write32(wire+16,c->accepted_sequence);
}

#include "ethercat_motor_bridge.h"
#include "motor_control.h"
#include "motor_safety.h"
#include "main.h"
#include <math.h>
#include <string.h>
#if ETHERCAT_MOTOR_OPEN_ANGLE
#include "open_angle_experiment.h"
#endif

static EMC_Core core;
static uint8_t platform_failed;
static int16_t Quantize(float value, float scale)
{
    float v=value*scale;
    if(!isfinite(v)) return 0;
    if(v>32767.0f) return 32767;
    if(v< -32768.0f) return (int16_t)-32768;
    return (int16_t)v;
}
static void Observe(void)
{
    EMC_Feedback f;
    memset(&f,0,sizeof(f));
#if ETHERCAT_MOTOR_OPEN_ANGLE
    {
    MotorOpenAngleFeedback motor;
    MotorControl_GetOpenAngleFeedback(&motor);
    f.iq_ma=Quantize(motor.iq_a,1000.0f);
    f.active_target_d10=Quantize(OAE_HzToRpm(motor.active_frequency_hz),10.0f);
    f.motor_fault=motor.motor_fault;f.running=motor.running;
    f.start_permitted=motor.start_permitted;f.control_valid=motor.control_valid &&
        isfinite(motor.iq_a) && isfinite(motor.active_frequency_hz);
    f.output_off=motor.output_off;f.stop_requested=motor.stop_requested;
    if(motor.bridge_fault && core.fault==EMC_FAULT_NONE) EMC_Fail(&core,(EMC_Fault)motor.bridge_fault);
    if(motor.motor_fault && core.fault==EMC_FAULT_NONE) EMC_Fail(&core,EMC_FAULT_MOTOR);
    }
#else
    {
    MotorControlFeedback motor;
    MotorControl_GetFeedback(&motor);
    f.speed_d10=Quantize(motor.speed_rpm,10.0f);
    f.iq_ma=Quantize(motor.iq_a,1000.0f);
    f.active_target_d10=Quantize(motor.active_target_rpm,10.0f);
    f.motor_fault=motor.fault; f.calibrated=motor.calibrated;
    f.running=motor.running;
    f.valid=(uint8_t)(motor.valid && isfinite(motor.speed_rpm) &&
        isfinite(motor.iq_a) && isfinite(motor.active_target_rpm));
    }
#endif
    EMC_SetFeedback(&core,&f);
}
void EtherCATMotorBridge_Init(void) {
    platform_failed=0;
    EMC_InitProfile(&core,ETHERCAT_MOTOR_OPEN_ANGLE ? EMC_PROFILE_OPEN_ANGLE : EMC_PROFILE_SPEED);
    Observe();
}
static EMC_Fault GuardFault(void)
{return MotorSafety_TripReason()==2 ? EMC_FAULT_RUNTIME : EMC_FAULT_HEARTBEAT;}
static void SetTarget(float rpm)
{
#if ETHERCAT_MOTOR_OPEN_ANGLE
    MotorControl_SetOpenAngleSpeedCommand(rpm);
#else
    MotorControl_SetSpeedCommand(rpm);
#endif
}
void EtherCATMotorBridge_StartupFailed(void)
{ platform_failed=1; EMC_Fail(&core,EMC_FAULT_START); MotorSafety_Revoke(); }
const EMC_Core *EtherCATMotorBridge_Get(void) { return &core; }
void EtherCATMotorBridge_SetLink(uint8_t active, uint8_t spi_ok)
{
    EMC_SetLink(&core,active,spi_ok);
    if(core.fault!=EMC_FAULT_NONE) MotorSafety_Revoke();
}
void EtherCATMotorBridge_Receive(const uint8_t *wire, size_t size)
{
    /* Called only from PDO output mapping, never from SDO writes. */
#if ETHERCAT_MOTOR_OPEN_ANGLE
    /* ADC-local revoke may precede this PDO callback and foreground Service.
     * Preserve its specific cause before interpreting a generic guard latch. */
    Observe();
#endif
    if(MotorSafety_IsTripped() && core.fault==EMC_FAULT_NONE)
        EMC_Fail(&core,GuardFault());
    EMC_Receive(&core,wire,size,HAL_GetTick());
#if ETHERCAT_MOTOR_OPEN_ANGLE
    if(core.fault==EMC_FAULT_NONE && core.state==EMC_RUNNING && core.target_speed_d10!=0)
        MotorSafety_MarkMotion();
#endif
    if(core.actions & EMC_ACTION_FEED) {
        core.actions &= (uint8_t)~EMC_ACTION_FEED;
        if(core.fault==EMC_FAULT_NONE) MotorSafety_Feed();
    }
    if(core.fault!=EMC_FAULT_NONE) MotorSafety_Revoke();
    else if(core.actions & EMC_ACTION_STOP) MotorSafety_Disarm();
}
void EtherCATMotorBridge_Service(void)
{
    uint8_t actions;
    HAL_StatusTypeDef result;
    Observe();
    if(MotorSafety_IsTripped() && core.fault==EMC_FAULT_NONE)
        EMC_Fail(&core,GuardFault());
    EMC_Poll(&core,HAL_GetTick());
    actions=EMC_TakeActions(&core);
    if(actions & EMC_ACTION_STOP) {
        if(core.fault!=EMC_FAULT_NONE) MotorSafety_Revoke();
        else MotorSafety_Disarm();
        result=MotorControl_Stop();
        EMC_ActionResult(&core,EMC_ACTION_STOP,result==HAL_OK,HAL_GetTick());
        Observe();
    }
    if((actions & EMC_ACTION_CLEAR) && !platform_failed && core.link && core.spi_ok && core.fault!=EMC_FAULT_SPI) {
        MotorSafety_Disarm();
        result=core.feedback.motor_fault ? MotorControl_ClearFault() : MotorControl_Stop();
        Observe();
        if(result==HAL_OK && !core.feedback.motor_fault && !core.feedback.running) {
            MotorSafety_Reset();
            EMC_ActionResult(&core,EMC_ACTION_CLEAR,1,HAL_GetTick());
        }
    }
    if((actions & EMC_ACTION_START) && core.state==EMC_STARTING && core.fault==EMC_FAULT_NONE) {
        if(!ETHERCAT_MOTOR_POWER_ENABLED) EMC_Fail(&core,EMC_FAULT_POWER_LOCKED);
        else if(!MotorSafety_Arm()) EMC_Fail(&core,EMC_FAULT_HEARTBEAT);
        else {
            SetTarget(0.0f);
            result=MotorControl_Start();
            if(MotorSafety_IsTripped() || !MotorSafety_OutputPermitted())
                EMC_Fail(&core,GuardFault());
            else EMC_ActionResult(&core,EMC_ACTION_START,result==HAL_OK,HAL_GetTick());
        }
    }
    if(core.fault!=EMC_FAULT_NONE) {
        MotorSafety_Revoke();
        if(actions & EMC_ACTION_START) (void)MotorControl_Stop();
    } else if(core.state==EMC_RUNNING && MotorSafety_OutputPermitted())
        SetTarget((float)core.target_speed_d10*0.1f);
}
void EtherCATMotorBridge_Pack(uint8_t wire[EMC_TX_BYTES]) { EMC_Pack(&core,wire); }

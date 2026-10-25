#include "motor_safety.h"
#include "motor_command_guard.h"
#include "main.h"
#include "tim.h"

static MotorCommandGuard guard;
static void GateOff(void)
{
    HAL_GPIO_WritePin(DRV_ENA_GPIO_Port,DRV_ENA_Pin,GPIO_PIN_RESET);
    if(htim8.Instance) htim8.Instance->BDTR &= ~TIM_BDTR_MOE;
}
void MotorSafety_Init(void) { MCG_Init(&guard); GateOff(); }
void MotorSafety_Feed(void)
{
    uint32_t p=__get_PRIMASK(); __disable_irq();
    MCG_Feed(&guard,HAL_GetTick()); __set_PRIMASK(p);
}
uint8_t MotorSafety_Arm(void)
{
    uint8_t ok; uint32_t p=__get_PRIMASK(); __disable_irq();
    ok=MCG_Permit(&guard,HAL_GetTick());
#if ETHERCAT_MOTOR_OPEN_ANGLE
    if(ok) MCG_StartSession(&guard,HAL_GetTick());
#endif
    __set_PRIMASK(p); return ok;
}
void MotorSafety_Disarm(void)
{
    uint32_t p=__get_PRIMASK(); __disable_irq();
    MCG_Disarm(&guard); GateOff(); __set_PRIMASK(p);
}
void MotorSafety_Revoke(void)
{
    uint32_t p=__get_PRIMASK(); __disable_irq();
    MCG_Revoke(&guard); GateOff(); __set_PRIMASK(p);
}
void MotorSafety_Reset(void)
{
    uint32_t p=__get_PRIMASK(); __disable_irq();
    GateOff(); MCG_Reset(&guard); __set_PRIMASK(p);
}
void MotorSafety_Tick(void)
{ if(MCG_Tick(&guard,HAL_GetTick()) || guard.latched) GateOff(); }
uint8_t MotorSafety_IsTripped(void) { return guard.latched; }
uint8_t MotorSafety_TripReason(void){return MCG_TripReason(&guard);}
void MotorSafety_MarkMotion(void)
{
    uint32_t p=__get_PRIMASK();__disable_irq();
#if ETHERCAT_MOTOR_OPEN_ANGLE
    MCG_MarkMotion(&guard,HAL_GetTick());
#endif
    __set_PRIMASK(p);
}
uint8_t MotorSafety_OutputPermitted(void)
{
    return ETHERCAT_MOTOR_POWER_ENABLED && guard.permitted && !guard.latched &&
        guard.fed && (uint32_t)(HAL_GetTick()-guard.last_ms)<100U;
}
uint8_t MotorSafety_TryEnableDriver(void)
{
    uint8_t ok; uint32_t p=__get_PRIMASK(); __disable_irq();
    ok=MotorSafety_OutputPermitted();
    if(ok) HAL_GPIO_WritePin(DRV_ENA_GPIO_Port,DRV_ENA_Pin,GPIO_PIN_SET);
    else GateOff();
    __set_PRIMASK(p); return ok;
}
uint8_t MotorSafety_TryStartSamplingPwm(void)
{
    uint8_t ok; uint32_t p=__get_PRIMASK(); __disable_irq();
    ok=MotorSafety_OutputPermitted();
    if(ok) ok=(HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4)==HAL_OK);
    if(!ok) GateOff();
    __set_PRIMASK(p);
    return ok && MotorSafety_OutputPermitted();
}

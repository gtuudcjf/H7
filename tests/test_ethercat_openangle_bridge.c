#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#undef assert
#define assert(x) do{if(!(x)){fprintf(stderr,"bridge line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#include <string.h>
#include "main.h"
#include "tim.h"
#include "motor_control.h"
#include "motor_safety.h"
#include "ethercat_motor_bridge.h"
GPIO_TypeDef test_drv_gpio;
static TestTimer timer;TIM_HandleTypeDef htim8={&timer};
static uint32_t now,mask,starts;static uint8_t delayed;
static float target;static MotorOpenAngleFeedback f;
uint32_t HAL_GetTick(void){return now;}uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}void __set_PRIMASK(uint32_t p){mask=p;}
void HAL_GPIO_WritePin(GPIO_TypeDef *g,uint32_t p,uint32_t s){if(s)g->ODR|=p;else g->ODR&=~p;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,uint32_t ch){(void)ch;h->Instance->BDTR|=TIM_BDTR_MOE;return HAL_OK;}
void MotorControl_GetFeedback(MotorControlFeedback *v){memset(v,0,sizeof(*v));}
void MotorControl_GetOpenAngleFeedback(MotorOpenAngleFeedback *v){*v=f;}
void MotorControl_SetSpeedCommand(float rpm){(void)rpm;assert(0 && "wrong speed-loop API");}
void MotorControl_SetOpenAngleSpeedCommand(float rpm){target=rpm;}
HAL_StatusTypeDef MotorControl_Start(void)
{
    ++starts;assert(target==0);assert(MotorSafety_TryEnableDriver());timer.BDTR=TIM_BDTR_MOE;
    if(delayed){now+=100;MotorSafety_Tick();assert(!MotorSafety_TryEnableDriver());assert(!MotorSafety_TryStartSamplingPwm());}
    f.running=1;f.control_valid=1;f.output_off=0;return HAL_OK;
}
HAL_StatusTypeDef MotorControl_Stop(void){f.running=f.control_valid=f.stop_requested=0;f.output_off=1;f.bridge_fault=0;timer.BDTR=test_drv_gpio.ODR=0;return HAL_OK;}
HAL_StatusTypeDef MotorControl_ClearFault(void){f.motor_fault=0;return MotorControl_Stop();}
static void cmd(unsigned cw,int speed,uint32_t seq)
{
    uint8_t b[8]={(uint8_t)cw,(uint8_t)(cw>>8),(uint8_t)speed,(uint8_t)((uint16_t)speed>>8),
       (uint8_t)seq,(uint8_t)(seq>>8),(uint8_t)(seq>>16),(uint8_t)(seq>>24)};
    EtherCATMotorBridge_Receive(b,8);EtherCATMotorBridge_Service();now+=10;
}
static void setup(void){now=starts=0;delayed=0;target=123;memset(&f,0,sizeof(f));f.start_permitted=f.output_off=1;MotorSafety_Init();EtherCATMotorBridge_Init();EtherCATMotorBridge_SetLink(1,1);}
static void ready(void){setup();cmd(0x100,0,1);cmd(0x100,0,2);cmd(0x100,0,3);assert(EtherCATMotorBridge_Get()->state==EMC_READY);}
int main(void)
{
    const EMC_Core *c;
    setup();cmd(0,0,1);cmd(0,0,2);cmd(0,0,3);assert(!starts && EtherCATMotorBridge_Get()->state!=EMC_READY);
    ready();c=EtherCATMotorBridge_Get();cmd(0x101,0,4);
#if ETHERCAT_MOTOR_POWER_ENABLED
    assert(starts==1);EtherCATMotorBridge_Service();assert(c->state==EMC_RUNNING);
    cmd(0x101,50,5);assert(target==5);
    EtherCATMotorBridge_SetLink(0,1);assert(!test_drv_gpio.ODR && !timer.BDTR);
    EtherCATMotorBridge_SetLink(1,1);cmd(0x101,0,6);assert(starts==1 && c->fault==1);
    cmd(0x180,0,7);assert(c->fault==0);cmd(0x100,0,8);cmd(0x100,0,9);cmd(0x100,0,10);
    cmd(0x101,0,11);assert(starts==2);
    f.stop_requested=1;EtherCATMotorBridge_Service();assert(c->fault==0 && c->state==EMC_DISARMED && !test_drv_gpio.ODR);
    cmd(0x101,0,12);assert(starts==2);
    ready();cmd(0x101,0,4);EtherCATMotorBridge_Service();assert(c->state==EMC_RUNNING);
    cmd(0x101,50,5);now=3040;MotorSafety_Feed();MotorSafety_Tick();
    assert(test_drv_gpio.ODR && timer.BDTR);
    now=6040;MotorSafety_Feed();MotorSafety_Tick();
    assert(!test_drv_gpio.ODR && !timer.BDTR);EtherCATMotorBridge_Service();assert(c->fault==12);
    ready();delayed=1;cmd(0x101,0,4);assert(c->fault==4 && !test_drv_gpio.ODR && !timer.BDTR);
    ready();cmd(0x101,0,4);cmd(0x100,0,4);assert(!test_drv_gpio.ODR && !timer.BDTR);
    ready();cmd(0x101,0,4);cmd(0,0,5);assert(c->fault==11 && !test_drv_gpio.ODR);
    {unsigned reason;for(reason=12;reason<=14;++reason){
        ready();cmd(0x101,0,4);f.bridge_fault=(uint16_t)reason;MotorSafety_Revoke();
        cmd(0x101,0,5);assert(c->fault==reason); /* real main order: Receive before Service */
    }}
    ready();cmd(0x101,0,4);assert(c->state==EMC_STARTING);
    f.running=0;f.control_valid=0;f.bridge_fault=8; /* adapter reports stale ALIGNING control */
    cmd(0x101,0,5);assert(c->fault==8 && now<2000 && !timer.BDTR && !test_drv_gpio.ODR);
#else
    assert(starts==0 && c->fault==9 && !test_drv_gpio.ODR && !timer.BDTR);
#endif
    printf("openangle real bridge/core/guard power=%d passed\n",ETHERCAT_MOTOR_POWER_ENABLED);return 0;
}

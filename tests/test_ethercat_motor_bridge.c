#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "tim.h"
#include "motor_control.h"
#include "motor_safety.h"
#include "ethercat_motor_bridge.h"
GPIO_TypeDef test_drv_gpio;
static TestTimer timer;
TIM_HandleTypeDef htim8={&timer};
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,uint32_t channel)
{(void)channel;h->Instance->BDTR|=TIM_BDTR_MOE;return HAL_OK;}
static uint32_t now,mask,starts,stops,checks;
static uint8_t stall_start;
static float target;
static MotorControlFeedback feedback;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"bridge %d: %s\n",__LINE__,#x);exit(1);}}while(0)
uint32_t HAL_GetTick(void){return now;}
uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t v){mask=v;}
void HAL_GPIO_WritePin(GPIO_TypeDef *g,uint32_t p,uint32_t s){if(s)g->ODR|=p;else g->ODR&=~p;}
void MotorControl_GetFeedback(MotorControlFeedback *f){*f=feedback;}
HAL_StatusTypeDef MotorControl_Start(void)
{
    ++starts; CHECK(target==0);
    CHECK(MotorSafety_TryEnableDriver()); timer.BDTR=TIM_BDTR_MOE;
    if(stall_start){now+=100;MotorSafety_Tick();}
    feedback.running=1;feedback.valid=1;return HAL_OK;
}
HAL_StatusTypeDef MotorControl_Stop(void)
{++stops;feedback.running=0;feedback.valid=0;timer.BDTR=0;test_drv_gpio.ODR=0;return HAL_OK;}
HAL_StatusTypeDef MotorControl_ClearFault(void){feedback.fault=0;return MotorControl_Stop();}
void MotorControl_SetSpeedCommand(float rpm){target=rpm;}
static void Cmd(uint16_t control,int16_t speed,uint32_t seq)
{
    uint8_t b[8];b[0]=(uint8_t)control;b[1]=(uint8_t)(control>>8);
    b[2]=(uint8_t)speed;b[3]=(uint8_t)((uint16_t)speed>>8);
    b[4]=(uint8_t)seq;b[5]=(uint8_t)(seq>>8);b[6]=(uint8_t)(seq>>16);b[7]=(uint8_t)(seq>>24);
    EtherCATMotorBridge_Receive(b,8);EtherCATMotorBridge_Service();now+=10;
}
static void Setup(void)
{
    now=0;starts=0;stops=0;target=0;stall_start=0;
    memset(&feedback,0,sizeof(feedback));feedback.calibrated=1;
    MotorSafety_Init();EtherCATMotorBridge_Init();EtherCATMotorBridge_SetLink(1,1);
    Cmd(0,0,1);Cmd(0,0,2);Cmd(0,0,3);
}
int main(void)
{
    const EMC_Core *c;
    Setup();c=EtherCATMotorBridge_Get();CHECK(c->state==EMC_READY);
    Cmd(1,0,4);
#if ETHERCAT_MOTOR_POWER_ENABLED
    CHECK(starts==1); EtherCATMotorBridge_Service();CHECK(c->state==EMC_RUNNING);
    Cmd(1,-50,5); CHECK(target==-5.0f);
    now=139;MotorSafety_Tick();CHECK(!MotorSafety_IsTripped());
    now=140;MotorSafety_Tick();CHECK(MotorSafety_IsTripped());
    CHECK(!test_drv_gpio.ODR && !timer.BDTR);
    EtherCATMotorBridge_Service();CHECK(c->fault==EMC_FAULT_HEARTBEAT && stops>0);
    EtherCATMotorBridge_SetLink(0,1);EtherCATMotorBridge_SetLink(1,1);
    Cmd(1,0,6);CHECK(starts==1);
    Cmd(0x80,0,7);CHECK(c->fault==EMC_FAULT_NONE);
    Cmd(0,0,8);Cmd(0,0,9);Cmd(0,0,10);CHECK(c->state==EMC_READY);
    stall_start=1;Cmd(1,0,11);
    CHECK(c->fault==EMC_FAULT_HEARTBEAT);CHECK(!MotorSafety_OutputPermitted());
    CHECK(!test_drv_gpio.ODR && !timer.BDTR);
    Setup();Cmd(1,0,4);EtherCATMotorBridge_Service();
    EtherCATMotorBridge_SetLink(0,1);CHECK(!test_drv_gpio.ODR && !timer.BDTR);
    EtherCATMotorBridge_Service();CHECK(c->fault==EMC_FAULT_NOT_OP);
#else
    CHECK(starts==0);CHECK(c->fault==EMC_FAULT_POWER_LOCKED);
    CHECK(!test_drv_gpio.ODR && !timer.BDTR);
#endif
    printf("motor bridge (power=%d): %u checks passed\n",ETHERCAT_MOTOR_POWER_ENABLED,checks);return 0;
}

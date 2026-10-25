#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "tim.h"
#include "motor_safety.h"
GPIO_TypeDef test_drv_gpio;
static TestTimer timer;
TIM_HandleTypeDef htim8={&timer};
static uint32_t now, mask, pending, checks, starts, stall_pwm;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"safety %d: %s\n",__LINE__,#x);exit(1);}}while(0)
uint32_t HAL_GetTick(void){return now;}
uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t v){mask=v;if(!v && pending){pending=0;MotorSafety_Tick();}}
void HAL_GPIO_WritePin(GPIO_TypeDef *g,uint32_t p,uint32_t s)
{if(s)g->ODR|=p;else g->ODR&=~p;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,uint32_t channel)
{
    CHECK(channel==TIM_CHANNEL_4); ++starts; h->Instance->BDTR|=TIM_BDTR_MOE;
    if(stall_pwm){now+=100; pending=1;}
    return HAL_OK;
}
int main(void)
{
    MotorSafety_Init(); CHECK(!MotorSafety_OutputPermitted());
    CHECK(!MotorSafety_Arm());
    MotorSafety_Feed(); CHECK(MotorSafety_Arm());
#if ETHERCAT_MOTOR_POWER_ENABLED
    CHECK(MotorSafety_TryEnableDriver()); CHECK(test_drv_gpio.ODR==16);
#else
    CHECK(!MotorSafety_TryEnableDriver()); CHECK(test_drv_gpio.ODR==0);
#endif
    timer.BDTR=TIM_BDTR_MOE;
    now=99;MotorSafety_Tick();CHECK(!MotorSafety_IsTripped());
    now=100;MotorSafety_Tick();CHECK(MotorSafety_IsTripped());
    CHECK(timer.BDTR==0 && test_drv_gpio.ODR==0);
    MotorSafety_Feed();CHECK(!MotorSafety_Arm());CHECK(!MotorSafety_TryEnableDriver());
    MotorSafety_Reset();CHECK(!MotorSafety_OutputPermitted());
    MotorSafety_Feed();CHECK(MotorSafety_Arm());
    now=200;pending=1; /* revoke at the first permitted interrupt boundary */
    (void)MotorSafety_TryEnableDriver();
    CHECK(MotorSafety_IsTripped()); CHECK(test_drv_gpio.ODR==0);
    CHECK(!MotorSafety_TryEnableDriver());
    CHECK(!MotorSafety_TryStartSamplingPwm()); CHECK(starts==0 && timer.BDTR==0);
    MotorSafety_Reset(); MotorSafety_Feed(); CHECK(MotorSafety_Arm());
    stall_pwm=1;
#if ETHERCAT_MOTOR_POWER_ENABLED
    (void)MotorSafety_TryStartSamplingPwm();
    CHECK(starts==1 && MotorSafety_IsTripped());
#else
    CHECK(!MotorSafety_TryStartSamplingPwm()); CHECK(starts==0);
#endif
    CHECK(timer.BDTR==0 && test_drv_gpio.ODR==0);
    printf("motor safety adapter (power=%d): %u checks passed\n",ETHERCAT_MOTOR_POWER_ENABLED,checks);return 0;
}

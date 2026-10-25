#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#undef assert
#define assert(x) do{if(!(x)){fprintf(stderr,"feedback line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#include <math.h>
#include <string.h>
typedef struct {float iq_a,active_frequency_hz;uint16_t motor_fault,bridge_fault;uint8_t start_permitted,running,control_valid,output_off,stop_requested;} MotorOpenAngleFeedback;
enum {MOTOR_CONTROL_STOPPED=0,MOTOR_CONTROL_OPEN_ANGLE_CURRENT=2,MOTOR_CONTROL_FAULT=5};
enum {MOTOR_RUN_STATE_STOPPED=0,MOTOR_RUN_STATE_ALIGNING=3,MOTOR_RUN_STATE_RUNNING=5};
#define MOTOR_FAULT_NONE 0
#define CURRENT_ADC_TIMEOUT_TICKS 3
#define OAE_CONTROL_STALE_MS 2U
#define DRV_ENA_Pin 16U
#define TIM_BDTR_MOE 32768U
static struct {unsigned ODR;} gpio;
#define DRV_ENA_GPIO_Port (&gpio)
static struct {void *unused;int mode;} motor_config;
/* Match only the peripheral registers read by the production snapshot. */
static struct {float q;} current_feedback_dq;
static struct {float electrical_frequency_hz;} open_loop_state;
static struct {uint16_t bridge_fault;uint8_t request_stop;} oae_result;
static struct {uint32_t active;} g_motor_openangle_baseline_diagnostics;
typedef struct {unsigned BDTR;} Timer;
static Timer real_timer;
static struct {Timer *Instance;} htim8={&real_timer};
static unsigned motor_mode,run_state,fault_code,adc_age_ticks,irq;
static unsigned pwm_enabled,sampling_started,current_sense_ready,current_feedback_valid;
static unsigned open_angle_initialized,permit;
static uint32_t now,oae_last_adc_ms;
uint32_t HAL_GetTick(void){return now;}
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t p){irq=p;}
static unsigned MotorSafety_OutputPermitted(void){return permit;}
#include "openangle_feedback_under_test.inc"
int main(void)
{
    MotorOpenAngleFeedback f;
    open_angle_initialized=1;motor_config.mode=2;MotorControl_GetOpenAngleFeedback(&f);
    assert(f.start_permitted && f.output_off && !f.running && !f.control_valid);
    sampling_started=1;MotorControl_GetOpenAngleFeedback(&f);assert(!f.output_off && !f.start_permitted);
    pwm_enabled=1;permit=1;motor_mode=2;run_state=3;current_sense_ready=current_feedback_valid=1;
    MotorControl_GetOpenAngleFeedback(&f);assert(!f.running && f.control_valid);
    now=3;MotorControl_GetOpenAngleFeedback(&f);assert(!f.control_valid && f.bridge_fault==8);now=0;
    current_sense_ready=0;pwm_enabled=0;now=3;
    MotorControl_GetOpenAngleFeedback(&f);assert(f.bridge_fault==0); /* initial calibration is not active control */
    g_motor_openangle_baseline_diagnostics.active=1;
    MotorControl_GetOpenAngleFeedback(&f);assert(f.bridge_fault==0); /* calibration not yet ready */
    current_sense_ready=1;
    MotorControl_GetOpenAngleFeedback(&f);assert(f.bridge_fault==8 && !f.running && !f.control_valid);
    now=0;MotorControl_GetOpenAngleFeedback(&f);assert(f.bridge_fault==0 && !f.running);
    g_motor_openangle_baseline_diagnostics.active=0;
    current_sense_ready=1;pwm_enabled=1;now=0;
    run_state=5;MotorControl_GetOpenAngleFeedback(&f);assert(f.running && f.control_valid);
    now=3;MotorControl_GetOpenAngleFeedback(&f);assert(!f.control_valid);now=0;
    adc_age_ticks=4;MotorControl_GetOpenAngleFeedback(&f);assert(!f.control_valid);
    adc_age_ticks=0;current_feedback_dq.q=NAN;MotorControl_GetOpenAngleFeedback(&f);assert(!f.control_valid);
    current_feedback_dq.q=0;permit=0;MotorControl_GetOpenAngleFeedback(&f);assert(!f.control_valid && !f.running);
    motor_mode=0;run_state=0;sampling_started=pwm_enabled=0;
    gpio.ODR=16;MotorControl_GetOpenAngleFeedback(&f);assert(!f.output_off);
    gpio.ODR=0;real_timer.BDTR=32768;MotorControl_GetOpenAngleFeedback(&f);assert(!f.output_off);
    real_timer.BDTR=0;MotorControl_GetOpenAngleFeedback(&f);assert(f.output_off);
    fault_code=4;MotorControl_GetOpenAngleFeedback(&f);assert(!f.start_permitted && f.motor_fault==4);
    puts("production openangle feedback: 15 scenarios passed");return 0;
}

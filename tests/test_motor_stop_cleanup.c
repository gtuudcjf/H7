/* The runner extracts the unchanged production Stop/ClearFault functions.
 * Only their hardware dependencies are faked; cleanup state is real code. */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "motor_runtime_policy.h"
typedef enum {HAL_OK=0,HAL_ERROR=1} HAL_StatusTypeDef;
static MotorControlMode motor_mode,requested_mode;
static MotorRunState run_state;
static struct {MotorControlMode mode;} motor_config;
static bool encoder_calibration_save_pending,encoder_calibration_failure_pending;
static bool sampling_started,encoder_calibration_requested,encoder_calibration_active;
static bool current_calibration_failed;
static int current_pi,encoder_calibration_command,adc_age_ticks,overcurrent_count;
static int htim8,hadc1,fault_code,adc_calls,base_calls,pwm_calls,fail_mask;
static int current_calibration;
static bool current_sense_ready,current_feedback_valid,permit=true,base_start_fails;
static unsigned sampling_start_calls;
#define ETHERCAT_MOTOR_CONTROL 1
#define ADC_CALIB_OFFSET 0
#define ADC_SINGLE_ENDED 0
#define CURRENT_CALIBRATION_DISCARD_COUNT 1
#define CURRENT_CALIBRATION_SAMPLE_COUNT 1
#define MOTOR_FAULT_ADC_START 8
#define MOTOR_FAULT_DRIVER 9
#define MOTOR_FAULT_NONE 0
#define TIM_CHANNEL_4 12
static void MotorControl_DisablePowerStage(void){}
static void CurrentPi_Reset(int *p){*p=0;}
static void MotorControl_ResetSpeedState(bool reset){(void)reset;}
static void MotorControl_PublishStateDebug(void){}
static void MotorControl_PublishEncoderDebug(void){}
static void MotorControl_PublishMotionDebug(void){}
static bool MotorSafety_OutputPermitted(void){return permit;}
static bool MotorSafety_TryStartSamplingPwm(void){++sampling_start_calls;return permit;}
static HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(int *h,int a,int b)
{(void)h;(void)a;(void)b;return HAL_OK;}
static HAL_StatusTypeDef Drv8323Board_EnableForPwm(void){return HAL_OK;}
static void Drv8323Board_SetCurrentCalibration(bool b){(void)b;}
static void Drv8323Board_Disable(void){}
static void MotorControl_EnterFault(int f){fault_code=f;motor_mode=MOTOR_CONTROL_FAULT;}
static void CurrentSenseCalibration_Start(int *c,int a,int b){(void)c;(void)a;(void)b;}
static HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(int *h){(void)h;return HAL_OK;}
static HAL_StatusTypeDef HAL_TIM_Base_Start_IT(int *h){(void)h;return base_start_fails?HAL_ERROR:HAL_OK;}
static HAL_StatusTypeDef HAL_TIM_PWM_Stop(int *p,int ch)
{(void)p;(void)ch;++pwm_calls;return fail_mask&1?HAL_ERROR:HAL_OK;}
static HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(int *p)
{(void)p;++base_calls;return fail_mask&2?HAL_ERROR:HAL_OK;}
static HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(int *p)
{(void)p;++adc_calls;return fail_mask&4?HAL_ERROR:HAL_OK;}
#include "motor_stop_under_test.inc"
int main(void)
{
    int bit;
    /* A failure after ADC startup must not hide failed rollback from reset. */
    motor_mode=MOTOR_CONTROL_STOPPED; sampling_started=false;
    base_start_fails=true; fail_mask=4;
    assert(MotorControl_Start()==HAL_ERROR);
    assert(sampling_started);
    fail_mask=0; assert(MotorControl_ClearFault()==HAL_OK);
    assert(!sampling_started && fault_code==0);
    base_start_fails=false;
    assert(MotorControl_Start()==HAL_OK); assert(sampling_start_calls==1);
    assert(MotorControl_Stop()==HAL_OK);
    for(bit=1;bit<=4;bit*=2){
        sampling_started=true;motor_mode=MOTOR_CONTROL_FAULT;fault_code=7;
        adc_calls=base_calls=pwm_calls=0;fail_mask=bit;
        assert(MotorControl_Stop()==HAL_ERROR);
        assert(sampling_started); /* failed cleanup must remain pending */
        assert(MotorControl_ClearFault()==HAL_ERROR);
        assert(fault_code==7 && sampling_started);
        assert(adc_calls==2 && base_calls==2 && pwm_calls==2);
        fail_mask=0;
        assert(MotorControl_ClearFault()==HAL_OK);
        assert(!sampling_started && fault_code==0);
        assert(adc_calls==3 && base_calls==3 && pwm_calls==3);
    }
    puts("actual motor Stop/ClearFault: failed cleanup retried and latch preserved (3 peripherals)");
    return 0;
}

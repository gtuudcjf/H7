/* Real calibration/baseline/protection/Stop functions. Only hardware and the
 * existing selected-mode entry boundary are replaced; conversion is real. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#undef assert
#define assert(x) do{if(!(x)){fprintf(stderr,"baseline line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#include "open_angle_experiment.h"
#include "current_sense.h"
#include "foc_transform.h"
#include "motor_params.h"
#include "motor_runtime_policy.h"
float sinf(float x){return (float)sin((double)x);}
float cosf(float x){return (float)cos((double)x);}
float fmodf(float x,float y){return (float)fmod((double)x,(double)y);}
#define ETHERCAT_MOTOR_OPEN_ANGLE 1
#define MOTOR_FAULT_NONE 0
#define MOTOR_FAULT_DRIVER 2
#define MOTOR_FAULT_CURRENT_CALIBRATION 4
#define MOTOR_FAULT_ADC_RANGE 5
#define MOTOR_FAULT_ADC_TIMEOUT 6
#define MOTOR_FAULT_OVERCURRENT 7
#define MOTOR_FAULT_CONTROL_MATH 8
#define CURRENT_CALIBRATION_RAIL_MARGIN 512.0f
#define CURRENT_CALIBRATION_MAX_SPAN 64U
#define CURRENT_ADC_RUNTIME_RAIL_MARGIN 32U
#define CURRENT_OVERCURRENT_CONFIRM_COUNT 2U
#define CURRENT_ADC_TIMEOUT_TICKS 3U
#define TIM_CHANNEL_4 12U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC1NE 4U
#define TIM_CCER_CC2E 16U
#define TIM_CCER_CC2NE 64U
#define TIM_CCER_CC3E 256U
#define TIM_CCER_CC3NE 1024U
typedef unsigned MotorFaultCode;
typedef enum {HAL_OK=0,HAL_ERROR=1} HAL_StatusTypeDef;
static MotorControlMode motor_mode,requested_mode;
static MotorRunState run_state;
static struct {MotorControlMode mode;} motor_config;
static unsigned fault_code,overcurrent_count,adc_age_ticks,now,starts;
static uint32_t oae_last_adc_ms;
static bool stop_callback;
static bool permit,pwm_enabled,sampling_started,current_feedback_valid;
static bool current_sense_ready,current_calibration_failed,encoder_calibration_valid;
static bool encoder_calibration_requested,encoder_calibration_active;
static bool encoder_calibration_save_pending,encoder_calibration_failure_pending;
static int current_pi,encoder_calibration_command;
static int hadc1;
typedef struct {unsigned CCER;} TestTimer;
static TestTimer timer;
static struct {TestTimer *Instance;} htim8;
static struct {float electrical_frequency_hz;} open_loop_state;
static float oae_requested_rpm,current_target_frequency_hz;
static OAE_State oae_state;
static OAE_Result oae_result;
static volatile OAE_Diagnostics g_motor_openangle_diagnostics;
static volatile OAE_Diagnostics g_motor_openangle_baseline_diagnostics;
static struct {uint32_t phase_a_raw,phase_b_raw;float phase_a_offset,phase_b_offset,id_a,iq_a;unsigned overcurrent_count;} g_motor_control_debug;
static CurrentSenseConfig current_sense_config={3.3f,4095.0f,0.02f,5.0f,1.0f,1.0f};
static CurrentSenseCalibration current_calibration;
static CurrentSenseOffsets current_offsets;
static CurrentPhaseCurrents phase_current;
static FocAlphaBeta alpha_beta_current;
static FocDq current_feedback_dq;
static uint32_t HAL_GetTick(void){return now;}
static void CurrentPi_Reset(int *p){(void)p;}
static void MotorControl_ResetSpeedState(bool reset){(void)reset;}
static void MotorControl_PublishStateDebug(void){}
static void MotorControl_PublishMotionDebug(void){}
static void MotorControl_PublishEncoderDebug(void){}
static void Drv8323Board_SetCurrentCalibration(bool b){assert(!b);}
static bool MotorControl_ModeUsesCurrentLoop(MotorControlMode m){return m==MOTOR_CONTROL_OPEN_ANGLE_CURRENT;}
static bool MotorControl_ModeUsesEncoderAngle(MotorControlMode m){(void)m;return false;}
static void MotorSafety_Revoke(void){permit=false;pwm_enabled=false;}
static bool MotorSafety_OutputPermitted(void){return permit;}
static void MotorControl_StartSelectedMode(void)
{
    assert(permit);assert(!g_motor_openangle_baseline_diagnostics.active);
    ++starts;pwm_enabled=true;motor_mode=MOTOR_CONTROL_OPEN_ANGLE_CURRENT;
    run_state=MOTOR_RUN_STATE_ALIGNING;
}
static bool MotorControl_RunPwmOffBaseline(uint32_t a,uint32_t b,float dt_s);
static void MotorControl_DisablePowerStage(void){pwm_enabled=false;}
static HAL_StatusTypeDef HAL_TIM_PWM_Stop(void *h,unsigned c)
{
    (void)h;(void)c;
    if(stop_callback){stop_callback=false;now=1200;MotorControl_RunPwmOffBaseline(2033,2057,0.0001f);}
    return HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(void *h){(void)h;return HAL_OK;}
static HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(void *h){(void)h;return HAL_OK;}
#include "openangle_baseline_under_test.inc"
static void setup(void)
{
    unsigned i;
    memset((void *)&g_motor_openangle_baseline_diagnostics,0,sizeof(g_motor_openangle_baseline_diagnostics));
    memset((void *)&g_motor_openangle_diagnostics,0,sizeof(g_motor_openangle_diagnostics));
    memset(&g_motor_control_debug,0,sizeof(g_motor_control_debug));
    permit=sampling_started=true;pwm_enabled=stop_callback=false;starts=0;fault_code=0;now=1000;
    current_feedback_valid=false;adc_age_ticks=0;overcurrent_count=0;
    timer.CCER=0;htim8.Instance=&timer;
    motor_mode=MOTOR_CONTROL_STOPPED;requested_mode=MOTOR_CONTROL_OPEN_ANGLE_CURRENT;
    motor_config.mode=requested_mode;run_state=MOTOR_RUN_STATE_CURRENT_CALIBRATING;
    OAE_Start(&oae_state,now);OAE_DiagnosticsBegin(&g_motor_openangle_diagnostics,now);
    CurrentSenseCalibration_Start(&current_calibration,0,256);
    for(i=0;i<256;++i)CurrentSenseCalibration_AddSample(&current_calibration,2033,2057);
    MotorControl_FinishCalibration();
}
int main(void)
{
    setup();
    /* RED before implementation: calibration used to enable phase PWM now. */
    assert(starts==0 && !pwm_enabled && run_state==MOTOR_RUN_STATE_READY);
    assert(g_motor_openangle_baseline_diagnostics.active);
    assert(g_motor_openangle_baseline_diagnostics.calibrated);
    assert(g_motor_openangle_baseline_diagnostics.offset_a==2033.0f);
    now=1099;
    assert(MotorControl_RunPwmOffBaseline(2033,2057,0.0001f));
    assert(starts==0 && !pwm_enabled);
    assert(g_motor_openangle_baseline_diagnostics.sample_count==1);
    assert(g_motor_openangle_diagnostics.sample_count==0);
    now=1100;
    assert(MotorControl_RunPwmOffBaseline(2033,2057,0.0001f));
    assert(starts==1 && pwm_enabled && !g_motor_openangle_baseline_diagnostics.active);
    assert(g_motor_openangle_baseline_diagnostics.duration_ms==100);
    assert(g_motor_openangle_baseline_diagnostics.stop_reason==OAE_DIAG_STOP_AUTO);
    assert(!MotorControl_RunPwmOffBaseline(2033,2057,0.0001f));
    assert(starts==1);
    assert(MotorControl_UpdateCurrentMeasurement(2035,2054,0.0f));
    assert(g_motor_openangle_diagnostics.sample_count==1);
    assert(g_motor_openangle_baseline_diagnostics.sample_count==2);

    setup();
    {unsigned i;for(i=1;i<=1000;++i){
        now=1000+i/10;
        MotorControl_RunPwmOffBaseline(2033,2057,0.0001f);
        assert(starts==(i==1000?1U:0U));
    }}
    assert(g_motor_openangle_baseline_diagnostics.sample_count==1000);
    assert(g_motor_openangle_diagnostics.sample_count==0);
    assert(g_motor_openangle_baseline_diagnostics.peak_abs_phase_a==0.0f);

    /* The old diagnostic-level excursions no longer block mode entry. */
    setup();
    now=1099;MotorControl_RunPwmOffBaseline(2171,2043,0.0001f);
    now=1100;MotorControl_RunPwmOffBaseline(2171,2043,0.0001f);
    assert(starts==1 && g_motor_openangle_baseline_diagnostics.max_consecutive_over_limit==2);
    now=1101;g_motor_control_debug.phase_a_raw=2171;g_motor_control_debug.phase_b_raw=2043;
    assert(MotorControl_UpdateCurrentMeasurement(2171,2043,0.0f));
    assert(fault_code==0 && permit);
    assert(g_motor_openangle_diagnostics.sample_count==1);
    assert(g_motor_openangle_diagnostics.last.raw_a==2171);
    assert(g_motor_openangle_baseline_diagnostics.sample_count==2);

    /* Do not forgive a >10 A streak merely because the log changed. */
    setup();now=1100;MotorControl_RunPwmOffBaseline(3400,2057,0.0001f);
    assert(starts==1 && permit);
    now=1101;g_motor_control_debug.phase_a_raw=3400;g_motor_control_debug.phase_b_raw=2057;
    assert(!MotorControl_UpdateCurrentMeasurement(3400,2057,0.0f));
    assert(fault_code==7 && !permit);
    assert(g_motor_openangle_baseline_diagnostics.sample_count==1);
    assert(g_motor_openangle_diagnostics.sample_count==1);

    setup();
    now=1001;MotorControl_RunPwmOffBaseline(3400,2057,0.0001f);
    assert(permit && starts==0);
    now=1002;g_motor_control_debug.phase_a_raw=3400;g_motor_control_debug.phase_b_raw=2057;
    MotorControl_RunPwmOffBaseline(3400,2057,0.0001f);
    assert(!permit && starts==0 && fault_code==7);
    assert(!g_motor_openangle_baseline_diagnostics.active);
    assert(g_motor_openangle_baseline_diagnostics.sample_count==2);
    assert(g_motor_openangle_baseline_diagnostics.max_consecutive_over_limit==2);
    assert(g_motor_openangle_baseline_diagnostics.motor_fault==7);
    assert(g_motor_openangle_diagnostics.motor_fault==7);
    assert(MotorControl_ClearFault()==HAL_OK);
    assert(g_motor_openangle_baseline_diagnostics.motor_fault==7);

    setup();now=1001;MotorControl_RunPwmOffBaseline(2033,2057,0.0001f);
    stop_callback=true; /* ADC can interrupt foreground HAL cleanup. */
    assert(MotorControl_Stop()==HAL_OK);
    now=1200;assert(!MotorControl_RunPwmOffBaseline(2033,2057,0.0001f));
    assert(starts==0 && !pwm_enabled && !sampling_started);
    assert(g_motor_openangle_baseline_diagnostics.stop_reason==OAE_DIAG_STOP_API);
    assert(g_motor_openangle_baseline_diagnostics.sample_count==1);
    assert(MotorControl_ClearFault()==HAL_ERROR); /* no fault to clear */
    assert(g_motor_openangle_baseline_diagnostics.sample_count==1);

    setup();permit=false;now=1100;
    MotorControl_RunPwmOffBaseline(2033,2057,0.0001f);
    assert(starts==0 && !pwm_enabled);
    setup();timer.CCER=TIM_CCER_CC2NE;now=1001;
    MotorControl_RunPwmOffBaseline(2033,2057,0.0001f);
    assert(starts==0 && fault_code==MOTOR_FAULT_DRIVER && !permit);
    setup();now=1001;MotorControl_RunPwmOffBaseline(2033,2057,NAN);
    assert(starts==0 && fault_code==MOTOR_FAULT_CONTROL_MATH);
    setup();now=1001;MotorControl_RunPwmOffBaseline(0,2057,0.0001f);
    assert(starts==0 && fault_code==MOTOR_FAULT_ADC_RANGE);
    /* Freshness and the same short timeout are exercised on the real helper. */
    setup();now=1003;MotorControl_CheckPwmOffBaselineAge();
    assert(starts==0 && fault_code==MOTOR_FAULT_ADC_TIMEOUT && !permit);
    setup();{unsigned i;for(i=0;i<4;++i)MotorControl_CheckPwmOffBaselineAge();}
    assert(fault_code==MOTOR_FAULT_ADC_TIMEOUT && starts==0);
    /* Wraparound still waits the full 100 ms rather than starting early. */
    setup();g_motor_openangle_baseline_diagnostics.started_ms=0xFFFFFFCEU;
    now=49;MotorControl_RunPwmOffBaseline(2033,2057,0.0001f);assert(starts==0);
    now=50;MotorControl_RunPwmOffBaseline(2033,2057,0.0001f);assert(starts==1);
    puts("production PWM-off baseline: delayed phase entry, isolated records, unchanged protection, stop/revoke/timeout passed");
    return 0;
}

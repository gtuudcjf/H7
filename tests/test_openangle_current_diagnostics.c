/* Production measurement/fault functions + real conversion/Clarke/Park.
 * Only hardware shutdown and unrelated controller bookkeeping are faked. */
#include <assert.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "open_angle_experiment.h"
#include "current_sense.h"
#include "foc_transform.h"
#include "motor_params.h"
/* This TinyCC Windows CRT exports double math only. Preserve the production
 * float API using actual CRT operations, not mocked transform results. */
float sinf(float x){return (float)sin((double)x);}
float cosf(float x){return (float)cos((double)x);}
float fmodf(float x,float y){return (float)fmod((double)x,(double)y);}
#define ETHERCAT_MOTOR_OPEN_ANGLE 1
#define MOTOR_FAULT_NONE 0
#define MOTOR_FAULT_ADC_RANGE 5
#define MOTOR_FAULT_OVERCURRENT 7
#define MOTOR_FAULT_CONTROL_MATH 8
#define MOTOR_CONTROL_FAULT 99
#define MOTOR_RUN_STATE_FAULT 99
#define CURRENT_ADC_RUNTIME_RAIL_MARGIN 32U
#define CURRENT_OVERCURRENT_CONFIRM_COUNT 2U
typedef unsigned MotorFaultCode;
static unsigned motor_mode,run_state=3,requested_mode,fault_code,overcurrent_count;
static bool encoder_calibration_requested,encoder_calibration_active;
static bool encoder_calibration_save_pending,encoder_calibration_failure_pending,current_feedback_valid;
static int encoder_calibration_command,current_pi;
static uint32_t now,expected_samples_before_revoke;
static bool permit=true,revoked;
static OAE_State oae_state;
static OAE_Result oae_result;
static volatile OAE_Diagnostics g_motor_openangle_diagnostics;
static volatile OAE_Diagnostics g_motor_openangle_baseline_diagnostics;
static struct {uint32_t phase_a_raw,phase_b_raw;float id_a,iq_a;unsigned overcurrent_count;} g_motor_control_debug;
static CurrentSenseConfig current_sense_config={3.3f,4095.0f,0.02f,5.0f,1.0f,1.0f};
static CurrentSenseOffsets current_offsets={2033.796875f,2060.7734375f};
static CurrentPhaseCurrents phase_current;
static FocAlphaBeta alpha_beta_current;
static FocDq current_feedback_dq;
static uint32_t HAL_GetTick(void){if(fault_code==7)assert(revoked);return now;}
static void CurrentPi_Reset(int *p){(void)p;}
static void MotorControl_ResetSpeedState(bool reset){(void)reset;}
static void MotorControl_PublishStateDebug(void){}
static void MotorSafety_Revoke(void)
{
    /* No diagnostic sample copy may precede the original shutdown. */
    assert(g_motor_openangle_diagnostics.sample_count==expected_samples_before_revoke);
    permit=false;revoked=true;
}
#include "openangle_current_under_test.inc"
static bool sample(uint32_t a,uint32_t b)
{
    ++now;g_motor_control_debug.phase_a_raw=a;g_motor_control_debug.phase_b_raw=b;
    return MotorControl_UpdateCurrentMeasurement(a,b,0.0f);
}
int main(void)
{
    OAE_Start(&oae_state,0);OAE_DiagnosticsBegin(&g_motor_openangle_diagnostics,0);
    assert(sample(2037,2265));
    assert(sample(2037,2265));
    assert(g_motor_openangle_diagnostics.consecutive_over_limit==2 && fault_code==0 && permit);
    assert(sample(2034,2061));
    assert(g_motor_openangle_diagnostics.consecutive_over_limit==0);
    assert(sample(2037,2265));assert(sample(2037,2265));
    /* Three historical-threshold excursions must no longer stop mode 2. */
    assert(sample(2037,2265));
    assert(permit && fault_code==0 && g_motor_openangle_diagnostics.active);
    assert(g_motor_openangle_diagnostics.over_limit_total==5);
    /* The original >10 A / two-sample guard must still revoke immediately. */
    assert(sample(2037,3400));
    expected_samples_before_revoke=7;
    assert(!sample(2037,3400));
    assert(revoked && !permit && fault_code==7 && !g_motor_openangle_diagnostics.active);
    assert(g_motor_openangle_diagnostics.sample_count==8);
    assert(g_motor_openangle_diagnostics.max_consecutive_over_limit==5);
    assert(g_motor_openangle_diagnostics.over_limit_total==7);
    assert(g_motor_openangle_diagnostics.stop_reason==OAE_DIAG_STOP_MOTOR_FAULT);
    assert(g_motor_openangle_diagnostics.motor_fault==7);
    assert(g_motor_openangle_diagnostics.stop_run_state==3);
    assert(g_motor_openangle_diagnostics.last.raw_b==3400);
    assert(g_motor_openangle_diagnostics.last.at_ms==8);
    assert(g_motor_openangle_diagnostics.last.iq_a>10.0f);
    OAE_DiagnosticsStop(&g_motor_openangle_diagnostics,100,OAE_DIAG_STOP_API,0,0,0,0,0);
    assert(g_motor_openangle_diagnostics.motor_fault==7 && g_motor_openangle_diagnostics.duration_ms==8);
    puts("production current diagnostics: legacy 10 A/two-sample shutdown, separate historical counter, frozen history passed");
    return 0;
}

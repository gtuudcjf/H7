#ifndef OPEN_ANGLE_EXPERIMENT_H
#define OPEN_ANGLE_EXPERIMENT_H
#include <stdint.h>
#include "motor_params.h"
#define OAE_ALIGN_CURRENT_A 0.5f
#define OAE_CURRENT_A 0.8f
#define OAE_CURRENT_LIMIT_A 1.0f
#define OAE_VOLTAGE_LIMIT_PU 0.15f
/* Operator-requested legacy threshold, not a newly validated motor rating.
 * The original motor guard still trips above this level on two samples. */
#define OAE_PHASE_TRIP_A MOTOR_CURRENT_TRIP_A
/* Retain comparable historical diagnostics without using them to shut down. */
#define OAE_DIAGNOSTIC_PHASE_A 0.6f
#define OAE_RPM_LIMIT 20.0f
#define OAE_RPM_SLEW_PER_S 20.0f
/* Shared with the independent guard; milliseconds, not continuous-duty approval. */
#define OAE_SESSION_MS 10000U
#define OAE_MOTION_MS 6000U
#define OAE_ZERO_WAIT_MS 500U
#define OAE_STOP_MS 2000U
#define OAE_CONTROL_STALE_MS 2U
#define OAE_PWM_OFF_BASELINE_MS 100U
typedef struct {
    uint32_t start_ms,motion_ms,wait_ms,stop_ms;
    uint16_t fault;
    uint8_t stage,started,motion_seen,overcurrent_samples;
    int8_t direction;
} OAE_State;
typedef struct {
    uint32_t now_ms;
    float command_rpm,active_frequency_hz,active_id_a,active_iq_a;
    uint8_t aligning,control_valid;
} OAE_Input;
typedef struct {
    float target_frequency_hz,id_ref_a,iq_ref_a;
    uint8_t request_stop;
    uint16_t bridge_fault;
} OAE_Result;
/* Read-only RAM history for one start attempt. No control decisions use it.
 * Inspect only after active=0 for a stable snapshot; power loss erases it. */
typedef enum {
    OAE_DIAG_STOP_NONE=0, OAE_DIAG_STOP_API=1, OAE_DIAG_STOP_AUTO=2,
    OAE_DIAG_STOP_POLICY=3, OAE_DIAG_STOP_MOTOR_FAULT=4
} OAE_DiagnosticStop;
typedef struct {
    uint32_t at_ms,raw_a,raw_b,run_state;
    float ia_a,ib_a,ic_a,id_a,iq_a;
} OAE_DiagnosticSample;
typedef struct {
    uint32_t session,active,calibrated,started_ms,duration_ms,sample_count;
    uint32_t over_limit_total,consecutive_over_limit,max_consecutive_over_limit;
    uint32_t stop_reason,motor_fault,policy_fault,stop_run_state,stop_raw_a,stop_raw_b;
    float offset_a,offset_b,peak_abs_phase_a;
    OAE_DiagnosticSample first,last,peak,first_over_limit;
} OAE_Diagnostics;
void OAE_DiagnosticsBegin(volatile OAE_Diagnostics *d,uint32_t now_ms);
void OAE_DiagnosticsSample(volatile OAE_Diagnostics *d,const OAE_DiagnosticSample *sample);
void OAE_DiagnosticsStop(volatile OAE_Diagnostics *d,uint32_t now_ms,
                         OAE_DiagnosticStop reason,uint32_t motor_fault,
                         uint32_t policy_fault,uint32_t run_state,
                         uint32_t raw_a,uint32_t raw_b);
void OAE_Init(OAE_State *s);
void OAE_Start(OAE_State *s,uint32_t now_ms);
OAE_Result OAE_Step(OAE_State *s,const OAE_Input *in);
uint8_t OAE_CheckPhaseCurrent(OAE_State *s,float ia,float ib,float ic);
float OAE_RpmToHz(float rpm);
float OAE_HzToRpm(float hz);
#endif

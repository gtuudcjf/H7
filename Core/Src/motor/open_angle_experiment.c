#include "open_angle_experiment.h"
#include "motor_params.h"
#include <math.h>
#include <string.h>
enum { OAE_ALIGN=0,OAE_WAIT,OAE_MOTION,OAE_DRAIN_FREQUENCY,OAE_DRAIN_CURRENT,OAE_DONE };
void OAE_Init(OAE_State *s){memset(s,0,sizeof(*s));}
void OAE_Start(OAE_State *s,uint32_t now_ms){OAE_Init(s);s->started=1;s->start_ms=now_ms;}
float OAE_RpmToHz(float rpm){return rpm*(float)MOTOR_POLE_PAIRS/60.0f;}
float OAE_HzToRpm(float hz){return hz*60.0f/(float)MOTOR_POLE_PAIRS;}
OAE_Result OAE_Step(OAE_State *s,const OAE_Input *in)
{
    OAE_Result r={0};
    if(!s->started){r.request_stop=1;return r;}
    if(!isfinite(in->command_rpm) || fabs(in->command_rpm)>OAE_RPM_LIMIT) s->fault=3;
    else if(!in->control_valid || !isfinite(in->active_frequency_hz) ||
            !isfinite(in->active_id_a) || !isfinite(in->active_iq_a)) s->fault=8;
    if(!s->fault && ((uint32_t)(in->now_ms-s->start_ms)>=OAE_SESSION_MS ||
        (s->motion_seen && (uint32_t)(in->now_ms-s->motion_ms)>=OAE_MOTION_MS))) s->fault=12;
    if(s->fault){r.bridge_fault=s->fault;r.request_stop=1;return r;}
    if(in->aligning){
        if(in->command_rpm!=0.0f){s->fault=3;r.bridge_fault=3;r.request_stop=1;}
        else r.id_ref_a=OAE_ALIGN_CURRENT_A;
        return r;
    }
    if(s->stage==OAE_ALIGN){s->stage=OAE_WAIT;s->wait_ms=in->now_ms;}
    if(s->stage==OAE_WAIT && in->command_rpm!=0.0f){
        s->stage=OAE_MOTION;s->motion_seen=1;s->motion_ms=in->now_ms;
        s->direction=in->command_rpm>0.0f ? 1 : -1;
    }
    if(s->stage==OAE_MOTION && in->command_rpm!=0.0f &&
       ((in->command_rpm>0.0f ? 1 : -1)!=s->direction)){
        s->fault=13;r.bridge_fault=13;r.request_stop=1;return r;
    }
    if((s->stage==OAE_WAIT && (uint32_t)(in->now_ms-s->wait_ms)>=OAE_ZERO_WAIT_MS) ||
       (s->stage==OAE_MOTION && in->command_rpm==0.0f)){
        s->stage=OAE_DRAIN_FREQUENCY;s->stop_ms=in->now_ms;
    }
    if(s->stage==OAE_DRAIN_FREQUENCY || s->stage==OAE_DRAIN_CURRENT){
        if((uint32_t)(in->now_ms-s->stop_ms)>=OAE_STOP_MS){
            s->fault=14;r.bridge_fault=14;r.request_stop=1;return r;
        }
        if(in->active_frequency_hz==0.0f) s->stage=OAE_DRAIN_CURRENT;
        if(s->stage==OAE_DRAIN_CURRENT && in->active_id_a==0.0f && in->active_iq_a==0.0f) s->stage=OAE_DONE;
    }
    if(s->stage==OAE_DONE) r.request_stop=1;
    else if(s->stage==OAE_DRAIN_CURRENT){ /* Explicit zero references. */ }
    else {
        r.iq_ref_a=OAE_CURRENT_A;
        if(s->stage==OAE_MOTION) r.target_frequency_hz=OAE_RpmToHz(in->command_rpm);
    }
    return r;
}
uint8_t OAE_CheckPhaseCurrent(OAE_State *s,float ia,float ib,float ic)
{
    if(!isfinite(ia)||!isfinite(ib)||!isfinite(ic)) return 1;
    if(fabs(ia)>OAE_PHASE_TRIP_A || fabs(ib)>OAE_PHASE_TRIP_A || fabs(ic)>OAE_PHASE_TRIP_A){
        if(s->overcurrent_samples<3) ++s->overcurrent_samples;
    } else s->overcurrent_samples=0;
    return s->overcurrent_samples>=3;
}

void OAE_DiagnosticsBegin(volatile OAE_Diagnostics *d,uint32_t now_ms)
{
    uint32_t session=d->session+1U;
    /* Called before sampling starts, never from a running control session. */
    memset((void *)d,0,sizeof(*d));
    d->session=session;
    d->started_ms=now_ms;
    d->active=1U;
}

void OAE_DiagnosticsSample(volatile OAE_Diagnostics *d,const OAE_DiagnosticSample *sample)
{
    float peak;
    if(!d->active) return;
    /* Caller supplies only successfully converted finite current samples. */
    peak=fmaxf(fabsf(sample->ia_a),fmaxf(fabsf(sample->ib_a),fabsf(sample->ic_a)));
    if(d->sample_count==0U) d->first=*sample;
    if(d->sample_count==0U || peak>d->peak_abs_phase_a){
        d->peak_abs_phase_a=peak;
        d->peak=*sample;
    }
    ++d->sample_count;
    d->last=*sample;
    if(peak>OAE_DIAGNOSTIC_PHASE_A){
        if(d->over_limit_total==0U) d->first_over_limit=*sample;
        ++d->over_limit_total;
        ++d->consecutive_over_limit;
        if(d->consecutive_over_limit>d->max_consecutive_over_limit)
            d->max_consecutive_over_limit=d->consecutive_over_limit;
    }else d->consecutive_over_limit=0U;
}

void OAE_DiagnosticsStop(volatile OAE_Diagnostics *d,uint32_t now_ms,
                         OAE_DiagnosticStop reason,uint32_t motor_fault,
                         uint32_t policy_fault,uint32_t run_state,
                         uint32_t raw_a,uint32_t raw_b)
{
    if(!d->active) return;
    /* Called after hardware permission is revoked or sampling is stopped.
     * Later Stop/ClearFault calls must preserve this first termination. */
    d->duration_ms=(uint32_t)(now_ms-d->started_ms);
    d->stop_reason=(uint32_t)reason;
    d->motor_fault=motor_fault;
    d->policy_fault=policy_fault;
    d->stop_run_state=run_state;
    d->stop_raw_a=raw_a;
    d->stop_raw_b=raw_b;
    d->active=0U;
}

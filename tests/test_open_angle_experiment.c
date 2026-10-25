#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "open_angle_experiment.h"
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"experiment %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static OAE_State s;static OAE_Input in;static OAE_Result out;
static void reset(uint32_t now){OAE_Init(&s);OAE_Start(&s,now);memset(&in,0,sizeof(in));in.now_ms=now;in.control_valid=1;}
static void step(uint32_t now,float rpm){in.now_ms=now;in.command_rpm=rpm;out=OAE_Step(&s,&in);}
static void diagnostic_sample(volatile OAE_Diagnostics *d,uint32_t now,float ib)
{
    OAE_DiagnosticSample sample={0};
    sample.at_ms=now;sample.raw_a=2037;sample.raw_b=2265;sample.run_state=3;
    sample.ia_a=0.026f;sample.ib_a=ib;sample.ic_a=-0.026f-ib;
    sample.id_a=0.026f;sample.iq_a=1.915f;
    OAE_DiagnosticsSample(d,&sample);
}
static void test_diagnostics(void)
{
    volatile OAE_Diagnostics d={0};
    OAE_DiagnosticsBegin(&d,100);
    d.offset_a=2033.797f;d.offset_b=2060.773f;d.calibrated=1;
    diagnostic_sample(&d,110,0.1f);
    CHECK(d.active && d.session==1 && d.sample_count==1);
    CHECK(d.first.at_ms==110 && d.first.raw_b==2265);
    diagnostic_sample(&d,120,1.646f);
    CHECK(d.over_limit_total==1 && d.consecutive_over_limit==1);
    CHECK(d.first_over_limit.at_ms==120);
    CHECK(fabs(d.peak_abs_phase_a-1.672f)<0.00001f);
    CHECK(d.peak.raw_b==2265 && d.peak.at_ms==120);
    diagnostic_sample(&d,130,0.7f);
    CHECK(d.max_consecutive_over_limit==2 && d.consecutive_over_limit==2);
    /* A quiet sample resets the current streak, not the peak or history. */
    diagnostic_sample(&d,140,0.1f);
    CHECK(d.consecutive_over_limit==0 && d.max_consecutive_over_limit==2);
    CHECK(d.over_limit_total==2 && d.peak.at_ms==120 && d.last.at_ms==140);
    OAE_DiagnosticsStop(&d,150,OAE_DIAG_STOP_AUTO,0,0,3,2038,2061);
    CHECK(!d.active && d.stop_reason==OAE_DIAG_STOP_AUTO && d.duration_ms==50);
    CHECK(d.stop_raw_a==2038 && d.stop_raw_b==2061 && d.stop_run_state==3);
    /* Stop/clear-fault cleanup must not overwrite the first termination. */
    OAE_DiagnosticsStop(&d,160,OAE_DIAG_STOP_API,0,0,0,0,0);
    diagnostic_sample(&d,170,10);
    CHECK(d.sample_count==4 && d.stop_reason==OAE_DIAG_STOP_AUTO);
    CHECK(d.duration_ms==50 && d.first.raw_a==2037 && d.calibrated==1);
    OAE_DiagnosticsBegin(&d,200);
    CHECK(d.session==2 && d.active && d.sample_count==0 && !d.calibrated);
    CHECK(d.over_limit_total==0 && d.peak_abs_phase_a==0 && d.stop_reason==0);
    diagnostic_sample(&d,201,0.7f);diagnostic_sample(&d,202,0.7f);diagnostic_sample(&d,203,0.7f);
    CHECK(d.max_consecutive_over_limit==3 && d.consecutive_over_limit==3);
    OAE_DiagnosticsStop(&d,204,OAE_DIAG_STOP_MOTOR_FAULT,7,0,3,2037,2265);
    CHECK(d.motor_fault==7 && d.stop_reason==OAE_DIAG_STOP_MOTOR_FAULT);
    OAE_DiagnosticsBegin(&d,UINT32_MAX-10);
    OAE_DiagnosticsStop(&d,9,OAE_DIAG_STOP_POLICY,0,14,3,1,2);
    CHECK(d.duration_ms==20 && d.policy_fault==14 && d.sample_count==0);
    /* The 0.6 A boundary is not counted as a strict-over-limit sample. */
    OAE_DiagnosticsBegin(&d,0);
    {
        OAE_DiagnosticSample sample={0};sample.ia_a=0.6f;sample.ic_a=-0.6f;
        OAE_DiagnosticsSample(&d,&sample);
        CHECK(d.over_limit_total==0 && d.max_consecutive_over_limit==0);
    }
}
int main(void)
{
    test_diagnostics();
    CHECK(fabs(OAE_RpmToHz(5)-0.83333333f)<0.00001f);CHECK(fabs(OAE_HzToRpm(-1)+6)<0.00001f);
    CHECK(fabs(OAE_RpmToHz(6)-1.0f)<0.00001f);
    CHECK(fabs(OAE_RpmToHz(-6)+1.0f)<0.00001f);
    reset(0);in.aligning=1;step(10,0);CHECK(out.id_ref_a==0.5f && out.iq_ref_a==0 && !out.bridge_fault);
    CHECK(out.id_ref_a*out.id_ref_a+out.iq_ref_a*out.iq_ref_a<=OAE_CURRENT_LIMIT_A*OAE_CURRENT_LIMIT_A);
    step(20,1);CHECK(out.bridge_fault==3);
    /* Leaving alignment must select the independent running amplitude. */
    reset(0);in.aligning=1;step(10,0);
    in.aligning=0;step(760,6);
    CHECK(out.id_ref_a==0 && out.iq_ref_a==0.8f && !out.bridge_fault);
    CHECK(fabs(out.target_frequency_hz-1.0f)<0.00001f);
    CHECK(out.id_ref_a*out.id_ref_a+out.iq_ref_a*out.iq_ref_a<=OAE_CURRENT_LIMIT_A*OAE_CURRENT_LIMIT_A);
    reset(0);step(100,0);step(599,0);CHECK(!out.request_stop);step(600,0);CHECK(out.request_stop);
    reset(0);step(10,-5);CHECK(out.target_frequency_hz<0 && out.iq_ref_a==0.8f);
    step(20,5);CHECK(out.bridge_fault==13);
    reset(0);step(10,5);CHECK(out.target_frequency_hz>0 && out.iq_ref_a==0.8f && !out.bridge_fault);
    in.active_frequency_hz=0.8f;in.active_iq_a=0.8f;
    step(20,0);CHECK(out.target_frequency_hz==0 && out.iq_ref_a==0.8f);
    step(30,5);CHECK(out.target_frequency_hz==0);in.active_frequency_hz=0;
    step(40,0);CHECK(out.iq_ref_a==0 && !out.request_stop);
    in.active_iq_a=0;step(50,0);CHECK(out.request_stop);
    reset(0);step(10,5);in.active_frequency_hz=1;step(20,0);step(2020,0);CHECK(out.bridge_fault==14);
    reset(0);step(10,5);step(3010,5);CHECK(!out.bridge_fault);
    step(6009,5);CHECK(!out.bridge_fault);step(6010,5);CHECK(out.bridge_fault==12);
    reset(0);in.aligning=1;step(5000,0);CHECK(!out.bridge_fault);
    step(9999,0);CHECK(!out.bridge_fault);step(10000,0);CHECK(out.bridge_fault==12);
    reset(UINT32_MAX-100);step(UINT32_MAX-90,5);step(5908,5);CHECK(!out.bridge_fault);step(5909,5);CHECK(out.bridge_fault==12);
    reset(0);step(1,20.1f);CHECK(out.bridge_fault==3);
    reset(0);step(1,NAN);CHECK(out.bridge_fault==3);
    reset(0);in.active_frequency_hz=INFINITY;step(1,0);CHECK(out.bridge_fault!=0);
    /* Restore the legacy trip level without treating the historical 0.6 A
     * diagnostic counter as a shutdown command. */
    reset(0);{unsigned i;for(i=0;i<20;++i)CHECK(!OAE_CheckPhaseCurrent(&s,0.08f,2.12f,-2.20f));}
    CHECK(!OAE_CheckPhaseCurrent(&s,10.0f,0,-10.0f));
    CHECK(!OAE_CheckPhaseCurrent(&s,10.01f,0,0));CHECK(!OAE_CheckPhaseCurrent(&s,0,10.01f,0));
    CHECK(OAE_CheckPhaseCurrent(&s,0,0,-10.01f));
    reset(0);CHECK(!OAE_CheckPhaseCurrent(&s,11,0,0));CHECK(!OAE_CheckPhaseCurrent(&s,0,0,0));
    CHECK(!OAE_CheckPhaseCurrent(&s,11,0,0));CHECK(!OAE_CheckPhaseCurrent(&s,11,0,0));CHECK(OAE_CheckPhaseCurrent(&s,11,0,0));
    reset(0);CHECK(OAE_CheckPhaseCurrent(&s,NAN,0,0));
    printf("openangle experiment: %u checks passed\n",checks);return 0;
}

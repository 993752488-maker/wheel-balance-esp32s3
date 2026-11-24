#include "product_job.h"
#include <math.h>
#include <string.h>
product_capture_config_t product_capture_default(void)
{return (product_capture_config_t){.mode=CAPTURE_COAST,.vehicle_type=CAPTURE_BICYCLE};}
bool product_capture_config_valid(const product_capture_config_t *c)
{return c&&c->mode<=CAPTURE_TIMED&&c->vehicle_type<=CAPTURE_OTHER&&(!c->seconds||(c->seconds>=10&&c->seconds<=60));}
void product_capture_apply_preset(product_capture_config_t *c)
{
    if(c->vehicle_type==CAPTURE_ELECTRIC)c->mode=CAPTURE_TIMED;
    else if(c->vehicle_type==CAPTURE_BICYCLE)c->mode=CAPTURE_COAST;
}
unsigned product_capture_max_rpm(product_capture_mode_t mode)
{
    return mode==CAPTURE_TIMED?PRODUCT_TIMED_MAX_RPM_EXCLUSIVE:PRODUCT_COAST_MAX_RPM_EXCLUSIVE;
}
unsigned product_capture_auto_seconds(double rpm)
{
    if(!isfinite(rpm)||rpm<PRODUCT_MIN_RPM||rpm>=PRODUCT_TIMED_MAX_RPM_EXCLUSIVE)return 30;
    /* Leave time for startup rejection and collect more turns than the
     * retained comparison set, distributed across the whole window. */
    return (unsigned)fmax(20,fmin(60,ceil(144*60/rpm)+3));
}
double product_capture_baseline_rpm(const product_stage_summary_t *baseline)
{
    if(!baseline||!baseline->captured||!baseline->integrity_valid||!isfinite(baseline->min_rpm)||!isfinite(baseline->max_rpm)||
       baseline->min_rpm<PRODUCT_MIN_RPM||baseline->max_rpm>=PRODUCT_TIMED_MAX_RPM_EXCLUSIVE||baseline->min_rpm>baseline->max_rpm)return 0;
    return (baseline->min_rpm+baseline->max_rpm)/2;
}
void product_job_arm(product_job_t *j,uint64_t now,uint64_t edges,uint32_t epoch,product_capture_config_t capture)
{memset(j,0,sizeof(*j));j->capture=capture;j->state=PRODUCT_ARMED;j->epoch=epoch;j->initial_edges=j->last_edges=edges;j->deadline_us=now+120000000;}
void product_job_recording(product_job_t *j,uint64_t now,uint64_t edges)
{j->state=PRODUCT_RECORDING;j->start_us=j->last_motion_us=now;j->last_edges=edges;
 j->deadline_us=now+(uint64_t)(j->capture.mode==CAPTURE_TIMED?j->capture.seconds*1000:PRODUCT_CAPTURE_MAX_MS)*1000;}
bool product_job_finish(product_job_t *j)
{if(!j||j->state!=PRODUCT_RECORDING)return false;
 j->end_reason=CAPTURE_END_OPERATOR;j->state=PRODUCT_PROCESSING;return true;}
static product_job_action_t wait_for_steady_speed(product_job_t *j,uint64_t edges,bool in_range,double rpm)
{
    if(!in_range){j->steady_turns=0;j->last_edges=edges;return JOB_NONE;}
    if(edges==j->last_edges)return JOB_NONE;
    const bool consecutive=edges-j->last_edges==1;
    j->last_edges=edges;
    const double lower=fmin(rpm,j->steady_min_rpm),upper=fmax(rpm,j->steady_max_rpm);
    if(!consecutive||!j->steady_turns||upper-lower>PRODUCT_STEADY_SPEED_TOLERANCE*upper){
        j->steady_turns=1;
        j->steady_min_rpm=j->steady_max_rpm=rpm;
        return JOB_NONE;
    }
    j->steady_min_rpm=lower;j->steady_max_rpm=upper;
    if(++j->steady_turns<3)return JOB_NONE;
    j->target_rpm=(lower+upper)/2;
    if(!j->capture.seconds)j->capture.seconds=(uint16_t)fmax(j->minimum_seconds,product_capture_auto_seconds(j->target_rpm));
    return JOB_START;
}
product_job_action_t product_job_tick(product_job_t *j,uint64_t now,uint64_t edges,
    uint32_t epoch,bool online,bool reference_ok,bool data_fault,bool rpm_valid,double rpm)
{
    if(j->state!=PRODUCT_ARMED&&j->state!=PRODUCT_RECORDING)return JOB_NONE;
    product_reason_t failure=!online?PRODUCT_SENSOR:epoch!=j->epoch?PRODUCT_CONTEXT:
        !reference_ok?PRODUCT_REFERENCE:data_fault?PRODUCT_DATA_FAULT:PRODUCT_REASON_NONE;
    if(edges<j->last_edges)failure=PRODUCT_REFERENCE;
    if(failure){j->state=PRODUCT_NO_RESULT;j->reason=failure;return JOB_FAIL;}
    if(j->state==PRODUCT_ARMED){
        if(now>=j->deadline_us){j->state=PRODUCT_TIMEOUT;j->reason=PRODUCT_WAIT_TIMEOUT;return JOB_FAIL;}
        const bool in_range=rpm_valid&&rpm>=PRODUCT_MIN_RPM&&rpm<product_capture_max_rpm(j->capture.mode);
        if(j->capture.mode==CAPTURE_TIMED){
            const bool matches=!j->reference_rpm||fabs(rpm-j->reference_rpm)<=.03*fmax(rpm,j->reference_rpm);
            return wait_for_steady_speed(j,edges,in_range&&matches,rpm);
        }
        if(edges-j->initial_edges>=2&&in_range)return JOB_START;
        return JOB_NONE;
    }
    if(edges>j->last_edges){j->last_edges=edges;j->last_motion_us=now;}
    if(j->capture.mode==CAPTURE_TIMED){
        if(now>=j->deadline_us){j->end_reason=CAPTURE_END_TIMED;j->state=PRODUCT_PROCESSING;return JOB_STOP;}
        return JOB_NONE;}
    /* Remember the last period even after the display RPM becomes stale.
     * One optical mark cannot prove physical standstill: this is a bounded
     * no-reference observation, never a count/amplitude completion target. */
    if(rpm_valid&&isfinite(rpm)&&rpm>0)j->last_period_us=(uint64_t)fmin(12000000,60000000/rpm);
    const uint64_t silence=(uint64_t)fmax(8000000,fmin(12000000,3.0*j->last_period_us));
    if(now-j->last_motion_us>=silence){
        j->end_reason=CAPTURE_END_REFERENCE_SILENCE;j->state=PRODUCT_PROCESSING;return JOB_STOP;}
    if(now>=j->deadline_us){
        j->end_reason=CAPTURE_END_LIMIT;j->state=PRODUCT_NO_RESULT;j->reason=PRODUCT_CAPTURE_LIMIT;return JOB_FAIL;}
    return JOB_NONE;
}

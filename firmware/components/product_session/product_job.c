#include "product_job.h"
#include <math.h>
#include <string.h>
product_capture_config_t product_capture_default(void)
{return (product_capture_config_t){.mode=CAPTURE_COAST,.vehicle_type=CAPTURE_BICYCLE,.seconds=30};}
bool product_capture_config_valid(const product_capture_config_t *c)
{return c&&c->mode<=CAPTURE_TIMED&&c->vehicle_type<=CAPTURE_OTHER&&c->seconds>=10&&c->seconds<=60;}
void product_job_arm(product_job_t *j,uint64_t now,uint64_t edges,uint32_t epoch)
{memset(j,0,sizeof(*j));j->capture=product_capture_default();j->state=PRODUCT_ARMED;j->epoch=epoch;j->initial_edges=j->last_edges=edges;j->deadline_us=now+120000000;}
void product_job_recording(product_job_t *j,uint64_t now,uint64_t edges)
{j->state=PRODUCT_RECORDING;j->start_us=j->last_motion_us=now;j->last_edges=edges;
 j->deadline_us=now+(uint64_t)(j->capture.mode==CAPTURE_TIMED?j->capture.seconds*1000:PRODUCT_CAPTURE_MAX_MS)*1000;}
bool product_job_finish(product_job_t *j)
{if(!j||j->state!=PRODUCT_RECORDING)return false;
 j->end_reason=CAPTURE_END_OPERATOR;j->state=PRODUCT_PROCESSING;return true;}
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
        if(edges>=j->initial_edges && edges-j->initial_edges>=2 && rpm_valid&&isfinite(rpm)&&rpm>=20&&rpm<300)return JOB_START;
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
